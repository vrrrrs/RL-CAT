#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RL-CAT 树莓派侧模拟器（PC 上运行，通过串口与 STM32 对打）

用途（docs/33-firmware-plan.md P4 交付物之一）：
  · 在没有树莓派、没有 ONNX 策略的情况下验证【固件侧链路】
  · 实测模型 A 的时序可行性：4 ms 等待窗口够不够
  · 验收单 P4 的各项：100 Hz 收发的连续性、丢帧、失联降级、周期抖动

⚠️ 这个脚本【只做链路与协议】，不做策略推理。
   Pi 侧真正运行时用 pi/runtime.py（读 ONNX 策略），
   两者共用 pi/rlcat_proto.py，所以协议行为一致。

时序：
    收 TEL_FAST → （组装观测）→ 算一个动作 → 回 CMD_JOINT
  ⚠️ 关键指标是【回复耗时】。固件等 4 ms；这里统计它的 P50/P95/P99/MAX。
     若 P99 > 4 ms，说明模型 A 不可行，需要切模型 B。

用法：
    python pi/sim_esp32.py --port COM5
    python pi/sim_esp32.py --port COM5 --seconds 60 --mode stand
    python pi/sim_esp32.py --port /dev/ttyAMA0 --verbose

退出码：0 = 跑完且没有致命问题
"""

from __future__ import annotations

# ⚠️ Windows 控制台默认是 GBK，中文帮助/报告会乱码。
#    这里在任何输出之前把 stdout/stderr 切成 UTF-8。
#    ⚠️ 必须放在 `from __future__` 之后 —— 它要求位于文件最前面。
import sys as _sys
for _s in (_sys.stdout, _sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import argparse
import math
import os
import statistics
import struct
import sys
import time
from typing import Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rlcat_proto import (  # noqa: E402
    FrameDecoder, Frame,
    TYPE_CMD_JOINT, TYPE_TEL_FAST, TYPE_TEL_STATUS, TYPE_TEL_ACK, TYPE_TEL_LOG,
    TYPE_HEARTBEAT,
    LEN_TEL_FAST, MODE_IDLE, MODE_STAND, MODE_ACTIVE, MODE_DAMP, MODE_ESTOP,
    FLAG_ENABLE,
    make_cmd_joint, make_cmd_mode, parse_tel_fast, parse_tel_status,
    build_observation, action_to_q_target,
    ST_READY, ST_CALIBRATED, ST_LINK_OK, ST_IMU_OK, ST_ESTOP,
    N_JOINTS, GAIT_FREQ_HZ,
)

# ---------------------------------------------------------------------------
# 串口打开（termios 配置见 docs/31-pi-runtime.md §3.1）
# ---------------------------------------------------------------------------

def open_serial(port: str, baud: int):
    try:
        import serial
    except ImportError:
        print("!! pyserial 未安装。请运行：pip install pyserial")
        sys.exit(2)

    # ⚠️ 必须关掉 canonical 模式、流控，设 VMIN=0/VTIME=0（非阻塞读），
    #    否则 read() 会卡住整个 100 Hz 循环。
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,            # 非阻塞
        write_timeout=0.05,
    )
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass
    return ser


# ---------------------------------------------------------------------------
# 统计
# ---------------------------------------------------------------------------

class Stats:
    def __init__(self) -> None:
        self.tel_fast = 0
        self.tel_status = 0
        self.tel_ack = 0
        self.tel_log = 0
        self.unknown = 0
        self.crc_err = 0
        self.timeouts = 0
        self.seq_gaps = 0
        self.seq_lost_total = 0
        self.last_cycle: Optional[int] = None
        self.reply_us: List[float] = []
        self.cycle_us: List[float] = []
        self.status_seen: Dict[int, int] = {}
        self.cmd_sent = 0
        self.last_status: Optional[object] = None
        self.last_tel: Optional[object] = None

    def pct(self, arr: List[float], p: float) -> float:
        if not arr:
            return 0.0
        s = sorted(arr)
        k = max(0, min(len(s) - 1, int(round(p / 100.0 * (len(s) - 1)))))
        return s[k]

    def report(self) -> None:
        print("\n" + "=" * 60)
        print(" 链路统计")
        print("=" * 60)
        print("  TEL_FAST   : %d" % self.tel_fast)
        print("  TEL_STATUS : %d" % self.tel_status)
        print("  TEL_ACK    : %d" % self.tel_ack)
        print("  TEL_LOG    : %d" % self.tel_log)
        print("  未知 TYPE  : %d" % self.unknown)
        print("  CRC 错     : %d" % self.crc_err)
        print("  接收超时   : %d" % self.timeouts)
        print("  cycle_cnt 跳变 : %d 次（累计丢 %d 帧）"
              % (self.seq_gaps, self.seq_lost_total))
        print("  CMD_JOINT 已发 : %d" % self.cmd_sent)

        if self.reply_us:
            print("\n 回复耗时（收到 TEL_FAST 到发出 CMD_JOINT）")
            print("   mean %.0f us   P50 %.0f   P95 %.0f   P99 %.0f   max %.0f"
                  % (statistics.mean(self.reply_us),
                     self.pct(self.reply_us, 50), self.pct(self.reply_us, 95),
                     self.pct(self.reply_us, 99), max(self.reply_us)))
            p99 = self.pct(self.reply_us, 99)
            if p99 <= 4000:
                print("   ✅ P99 %.0f us <= 4000 us —— 模型 A 可行" % p99)
            else:
                print("   ❌ P99 %.0f us > 4000 us —— 模型 A 不可行，需切模型 B"
                      % p99)
        else:
            print("\n  ⚠️ 一次 TEL_FAST 都没收到")

        if self.last_tel is not None:
            t = self.last_tel
            print("\n 最后一帧 TEL_FAST")
            print("   quat=(%.4f,%.4f,%.4f,%.4f)" % t.quat)
            print("   gyro=(%.3f,%.3f,%.3f) rad/s" % t.gyro)
            print("   gait sin/cos = %.4f / %.4f" % (t.gait_sin, t.gait_cos))
            print("   voltage = %d mV" % t.voltage_mv)
            print("   status  = 0x%02X%s" % (
                t.status,
                "  [" + ",".join(
                    n for n, b in (("READY", ST_READY), ("CALIBRATED", ST_CALIBRATED),
                                   ("LINK_OK", ST_LINK_OK), ("IMU_OK", ST_IMU_OK),
                                   ("ESTOP", ST_ESTOP))
                    if t.status & b) + "]"))
            print("   err_flags = 0x%02X" % t.err_flags)
            print("   q[0..3] = " + " ".join("%.4f" % v for v in t.q[:4]))
            print("   q[8..11]= " + " ".join("%.4f" % v for v in t.q[8:12]))

        if self.last_status is not None:
            s = self.last_status
            print("\n 最后一帧 TEL_STATUS")
            print("   uptime = %d ms   voltage = %d mV   mode = %d"
                  % (s.uptime_ms, s.voltage_mv, s.mode))
            print("   rx_overflow = %d   bus_err = %d   bus_timeout = %d   seq_lost = %d"
                  % (s.rx_overflow, s.bus_err, s.bus_timeout, s.seq_lost))
            print("   status = 0x%02X   err_flags = 0x%02X" % (s.status, s.err_flags))
            print("   温度(0.1C) = " + " ".join(str(v) for v in s.temperature))


# ---------------------------------------------------------------------------
# 简易"假策略"：让关节做低频正弦摆动，便于肉眼确认动作生效
# ---------------------------------------------------------------------------

def fake_action(t_sec: float, mode: int) -> List[float]:
    if mode == MODE_IDLE:
        return [0.0] * N_JOINTS
    if mode == MODE_STAND:
        return [0.0] * N_JOINTS
    # ACTIVE：给每个关节一个错开相位的正弦，幅度不大（±0.1 → ±0.025 rad）
    out = []
    w = 2.0 * math.pi * 0.5          # 0.5 Hz
    for i in range(N_JOINTS):
        ph = (i / N_JOINTS) * 2.0 * math.pi
        out.append(0.10 * math.sin(w * t_sec + ph))
    return out


# ---------------------------------------------------------------------------
# 主循环
# ---------------------------------------------------------------------------

def run(args: argparse.Namespace) -> int:
    ser = open_serial(args.port, args.baud)
    st = Stats()
    dec = FrameDecoder()

    mode = {"idle": MODE_IDLE, "stand": MODE_STAND, "active": MODE_ACTIVE,
            "damp": MODE_DAMP, "estop": MODE_ESTOP}[args.mode]

    print("RL-CAT Pi simulator")
    print("  port=%s baud=%d mode=%s seconds=%s"
          % (args.port, args.baud, args.mode, args.seconds))
    print("  Ctrl-C 停止\n")

    # 先发一次模式，确保固件不是 IDLE
    ser.write(make_cmd_mode(mode, 0, 0))
    ser.flush()

    seq = 0
    t_start = time.perf_counter()
    next_cycle = t_start
    period = 1.0 / args.hz
    last_status_ts = 0.0
    last_action = [0.0] * N_JOINTS

    try:
        while True:
            now = time.perf_counter()
            if args.seconds > 0 and (now - t_start) >= args.seconds:
                break

            # ---- 等下一个 100 Hz 节拍 ----
            if now < next_cycle:
                # 忙等一小段（Linux 上 sleep 精度不够，见 31-pi-runtime §4.2）
                while time.perf_counter() < next_cycle:
                    pass
            cyc0 = time.perf_counter()
            next_cycle += period
            if (time.perf_counter() - next_cycle) > period:
                next_cycle = time.perf_counter() + period    # 落后太多就重新对齐

            # ---- ② 收 TEL_FAST（阻塞至多 timeout） ----
            deadline = time.perf_counter() + args.frame_timeout
            tel: Optional[object] = None
            while time.perf_counter() < deadline:
                chunk = ser.read(4096)
                if not chunk:
                    continue
                for fr in dec.feed(chunk, int(time.perf_counter() * 1000)):
                    if fr.type == TYPE_TEL_FAST:
                        st.tel_fast += 1
                        try:
                            tel = parse_tel_fast(fr.payload)
                            st.last_tel = tel
                        except ValueError as e:
                            st.crc_err += 1
                            print("  !! TEL_FAST 解析失败: %s" % e)
                        # cycle_cnt 连续性
                        if tel is not None:
                            if st.last_cycle is not None:
                                gap = (tel.cycle_cnt - st.last_cycle) & 0xFFFF
                                if gap != 1:
                                    st.seq_gaps += 1
                                    st.seq_lost_total += max(0, gap - 1)
                            st.last_cycle = tel.cycle_cnt
                    elif fr.type == TYPE_TEL_STATUS:
                        st.tel_status += 1
                        st.last_status = parse_tel_status(fr.payload)
                    elif fr.type == TYPE_TEL_ACK:
                        st.tel_ack += 1
                    elif fr.type == TYPE_TEL_LOG:
                        st.tel_log += 1
                    elif fr.type == TYPE_HEARTBEAT:
                        pass
                    else:
                        st.unknown += 1
                if tel is not None:
                    break
            if tel is None:
                st.timeouts += 1
                if args.verbose and (st.timeouts % 50 == 1):
                    print("  .. 等待 TEL_FAST 超时（第 %d 次）" % st.timeouts)
                # ⚠️ 超时就跳过本周期，但仍要推进 next_cycle（不要追赶式连发）
                continue

            # ---- ③ 组装 47 维观测（证明链路数据够用） ----
            obs = build_observation(
                tel,
                cmd_vx=args.vx, cmd_vy=0.0, cmd_wz=0.0,
                last_action=last_action,
            )
            if args.verbose and (st.tel_fast % 100 == 1):
                print("  obs[0:3]=%.4f %.4f %.4f  obs[45:47]=%.4f %.4f"
                      % (obs[0], obs[1], obs[2], obs[45], obs[46]))

            # ---- ④ 假策略 → 动作 → 目标角 ----
            action = fake_action(time.perf_counter() - t_start, mode)
            last_action = action
            q_target = action_to_q_target(action)

            # ---- ⑤ 发 CMD_JOINT ----
            frame = make_cmd_joint(q_target, mode=mode,
                                   flags=FLAG_ENABLE, seq=seq)
            seq = (seq + 1) & 0xFF
            try:
                ser.write(frame)
            except Exception as e:
                print("  !! 串口写失败: %s" % e)
                break
            st.cmd_sent += 1
            st.reply_us.append((time.perf_counter() - cyc0) * 1e6)

            # ---- ⑥ 低频打印 ----
            if args.verbose and (st.tel_fast % 500 == 0):
                print("  [%6.1fs] fast=%d status=%d gaps=%d replyP50=%.0fus"
                      % (time.perf_counter() - t_start, st.tel_fast, st.tel_status,
                         st.seq_gaps, st.pct(st.reply_us, 50)))

    except KeyboardInterrupt:
        print("\n(用户中断)")

    ser.close()
    st.report()

    # 退出码：有实质问题就非 0，便于 CI/脚本判断
    problems = 0
    if st.tel_fast == 0:
        print("\n❌ 一帧 TEL_FAST 都没收到")
        problems += 1
    if st.crc_err > 0:
        print("\n❌ 出现 CRC 错误 %d 次 —— 链路有误码" % st.crc_err)
        problems += 1
    if st.seq_lost_total > (st.tel_fast // 100):
        print("\n❌ cycle_cnt 丢帧 %d（超过 1%%）" % st.seq_lost_total)
        problems += 1
    return 1 if problems else 0


def main() -> int:
    ap = argparse.ArgumentParser(description="RL-CAT Pi-side link simulator")
    ap.add_argument("--port", required=True, help="串口设备，如 COM5 或 /dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--seconds", type=float, default=10.0, help="0 = 一直跑")
    ap.add_argument("--mode", choices=["idle", "stand", "active", "damp", "estop"],
                    default="stand")
    ap.add_argument("--frame-timeout", type=float, default=0.005,
                    help="单周期等 TEL_FAST 的超时（秒），默认 5 ms")
    ap.add_argument("--vx", type=float, default=0.0, help="写进观测的速度指令")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
