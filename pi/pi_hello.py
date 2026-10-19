#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 树莓派侧：阶段 1 最小可运行程序（链路验收用）

做的事只有一件：连上 STM32，把 TEL_FAST 连续读出来，并给出判据。

验收判据（docs/42-pi-setup.md §C3）：
    □ 能持续收到 TEL_FAST，CRC 零错
    □ cycle_cnt 连续递增（不丢周期）
    □ 用手掰某条腿 → 打印出的 q 跟着变
    □ 把机身倾斜 → 姿态（重力方向）跟着变
    □ 电压读数与万用表对比，误差 < 50 mV
    □ 记录周期抖动直方图：均值 / P95 / P99 / 最大

⚠️ 本程序默认【不发送】CMD_JOINT。固件的失联判定基于"距上次收到任意帧"，
   所以不发任何帧的话，它会在 100 ms 后开始趴下（这是它的安全设计）。
   要让机器人保持不动，加 --keepalive（周期性发 MODE_IDLE）。

⚠️ 关于"耗时"这个指标，这里报两个不同的东西，别混：
   · 处理耗时 = 从【收到这一帧】到【本周期工作做完】  ← 与固件 4 ms 等待窗口对应
   · 帧间隔   = 相邻两帧 TEL_FAST 的到达间隔        ← 反映 STM32 的 100 Hz 稳不稳

用法：
    python3 pi_hello.py                                   # /dev/ttyAMA0，跑 10 秒
    python3 pi_hello.py --seconds 60 --keepalive
    python3 pi_hello.py --port /dev/pts/0 --seconds 5     # 对着假 STM32 跑
    python3 pi_hello.py --quiet                           # 只打印总结

退出码：0 = 通过；1 = 有判据没通过；2 = 打不开串口
"""

from __future__ import annotations

# ⚠️ Windows 控制台默认是 GBK，中文输出会乱码。在任何输出之前把 stdout/stderr
#    切成 UTF-8。（与 pi/sim_esp32.py 同一手法）
import sys as _sys
for _s in (_sys.stdout, _sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import config as C                                  # noqa: E402
import stats as S                                   # noqa: E402
import telemetry as T                               # noqa: E402
from link import PiLink, LinkError                  # noqa: E402
from rlcat_proto import (                           # noqa: E402
    MODE_IDLE,
    ST_BATT_FORCE,
    ST_BATT_STOP,
    ST_BATT_WARN,
    ST_CALIBRATED,
    ST_ESTOP,
    ST_IMU_OK,
    ST_LINK_OK,
    ST_READY,
    TYPE_TEL_FAST,
    quat_to_gravity,
)

status_str = T.status_str          # 语义层里那份是唯一的定义，这里不重复实现


def main() -> int:
    ap = argparse.ArgumentParser(description="RL-CAT 阶段 1：链路观察")
    ap.add_argument("--port", default=C.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=C.SERIAL_BAUD)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--rate", type=float, default=5.0, help="打印频率 Hz")
    ap.add_argument("--keepalive", action="store_true",
                    help="周期性发 MODE_IDLE，避免固件失联降级")
    ap.add_argument("--quiet", action="store_true", help="不逐行打印，只出总结")
    ap.add_argument("--tilt", action="store_true",
                    help="额外打印机身姿态（重力方向），用于'倾斜机身'验收")
    args = ap.parse_args()

    print("=" * 78)
    print(" RL-CAT 阶段 1：链路观察   %s @ %d" % (args.port, args.baud))
    print("=" * 78)

    try:
        link = PiLink(args.port, args.baud, verbose=True)
        link.open()
    except LinkError as e:
        print("!! %s" % e)
        return 2

    work = S.Percentiles(capacity=20_000)       # 处理耗时（对齐固件 4 ms 窗口）
    interval = S.Percentiles(capacity=20_000)   # TEL_FAST 帧间隔
    latency = S.Percentiles(capacity=20_000)    # 收到帧 → 发出指令
    t_prev_rx = 0.0

    n_start = 0
    t_start = 0.0

    try:
        # ---------------------------------------------------------- 启动自检
        print("\n[启动自检] 等一帧 TEL_STATUS ...")
        st = link.wait_status(timeout_ms=2000)
        if st is None:
            print("  !! 2 s 内没收到 TEL_STATUS")
            print("     固件只在 (a) 每 100 ms 或 (b) 收到带 FLAG_REQ_STATUS 的")
            print("     CMD_JOINT 时发它。一帧都没有 → 查接线/波特率/设备节点。")
            print(link.report())
            return 1

        print("  电压(TEL_STATUS) = %d mV%s"
              % (st.voltage_mv,
                 "   ← ⚠️ 固件把这个字段写死为 0，真电压只在 TEL_FAST 里"
                 if st.voltage_mv == 0 else ""))
        print("  uptime = %d ms   mode = %d   err_flags = 0x%02X   seq_lost = %d"
              % (st.uptime_ms, st.mode, st.err_flags, st.seq_lost))
        print("  温度 = %s" % " ".join(str(v) for v in st.temperature[:12]))
        print("     ⚠️ 协议单位是 0.1°C，固件实际写的是原始 °C —— 差 10 倍，"
              "上真机时用体温捂一下舵机就能分辨")
        print("  status = 0x%02X [%s]" % (st.status, status_str(st.status)))

        bad = []
        if not (st.status & ST_READY):
            bad.append("STAT_READY 未置位（时钟/DWT 自检没过）")
        if not (st.status & ST_CALIBRATED):
            bad.append("STAT_CALIBRATED 未置位（标定无效）—— ACTIVE 会被静默降级成 STAND")
        if not (st.status & ST_IMU_OK):
            bad.append("STAT_IMU_OK 未置位（IMU 读不到）")
        if st.err_flags:
            bad.append("err_flags=0x%02X 非 0（⚠️ 固件里是 latch，只有重启才清）"
                       % st.err_flags)
        if bad:
            print("\n  ⚠️ 启动自检有告警：")
            for b in bad:
                print("     · %s" % b)
        else:
            print("\n  ✅ 启动自检通过")
        if st.status & ST_ESTOP:
            print("  ℹ️  当前处于 ESTOP —— 上电后固件在收到第一帧之前都会这样，"
                  "发几帧后会自动解除")

        # ---------------------------------------------------------- 主观察循环
        print("\n[连续观察] %.1f 秒（帧驱动；%s）...\n"
              % (args.seconds, "周期发 IDLE 保活" if args.keepalive else "只读不发"))

        t_end = time.perf_counter() + args.seconds
        t_next_print = time.perf_counter()
        t_next_keep = 0.0
        print_interval = 1.0 / args.rate if args.rate > 0 else 0.2
        n_start = link.stats.rx_by_type.get(TYPE_TEL_FAST, 0)
        t_start = time.perf_counter()

        while time.perf_counter() < t_end:
            tel = link.wait_tel_fast(timeout_ms=100)
            if tel is None:
                continue

            t_rx = link.t_rx_tel_fast            # 这一帧的到达时刻
            if t_prev_rx:
                interval.add((t_rx - t_prev_rx) * 1e6)
            t_prev_rx = t_rx

            # ---- 本周期的"工作" ----
            if args.keepalive and time.perf_counter() >= t_next_keep:
                link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE, flags=0)
                t_next_keep = time.perf_counter() + 0.05

            # 从收到帧到本周期工作结束 —— 这就是要和 4 ms 比的量
            work.add((time.perf_counter() - t_rx) * 1e6)
            latency.add((time.perf_counter() - t_rx) * 1e6)

            # ---- 打印节流 ----
            now = time.perf_counter()
            if now >= t_next_print and not args.quiet:
                t_next_print = now + print_interval
                g = quat_to_gravity(tel.quat)
                line = ("  #{cnt:5d}  V={v:5d}mV  {st:28s}  q[0..3]={q}"
                        .format(cnt=tel.cycle_cnt, v=tel.voltage_mv,
                                st=status_str(tel.status),
                                q=" ".join("%+.3f" % x for x in tel.q[:4])))
                if args.tilt:
                    line += "  g=(%+.2f,%+.2f,%+.2f)" % tuple(g)
                print(line)
                print("           q[4..7]=%s  q[8..11]=%s  gait=(%+.3f,%+.3f)"
                      % (" ".join("%+.3f" % x for x in tel.q[4:8]),
                         " ".join("%+.3f" % x for x in tel.q[8:12]),
                         tel.gait_sin, tel.gait_cos))

    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
    finally:
        # ⚠️ 优雅退出：先让机器人卸力，等它真收到，再关串口
        #    （docs/31-pi-runtime.md §5.2 / docs/42-pi-setup.md §C4 第 3 条）
        try:
            if link.is_open:
                link.send_cmd_mode(MODE_IDLE)
                time.sleep(0.2)
        except Exception:
            pass
        link.close()

    # ---------------------------------------------------------- 总结与判据
    elapsed = max(1e-9, time.perf_counter() - t_start)
    n_end = link.stats.rx_by_type.get(TYPE_TEL_FAST, 0)
    hz = (n_end - n_start) / elapsed

    print("\n" + "=" * 78)
    print(" 总结")
    print("=" * 78)
    print(link.report())
    print()
    print("  TEL_FAST 平均帧率 : %.1f Hz   (期望 100)" % hz)
    if interval.n_total:
        print("  帧间隔            : 均值 %.2f ms  P95 %.2f  P99 %.2f  最大 %.2f"
              % (interval.mean / 1000, interval.p95 / 1000,
                 interval.p99 / 1000, interval.maximum / 1000))
    print("  处理耗时(收帧→完成): %s" % work.line(""))
    print("     ⚠️ 判据：P99 < 4 ms（固件等指令的窗口）")

    t = link.last_tel
    verdict = []
    if link.stats.rx_frames == 0:
        verdict.append(("一帧都没收到", False))
    else:
        verdict.append(("收到 %d 帧" % link.stats.rx_frames, True))
    verdict.append(("CRC 零错（实际 %d）" % link.stats.crc_errors,
                    link.stats.crc_errors == 0))
    verdict.append(("cycle_cnt 连续（跳变 %d 次，丢 %d 周期）"
                    % (link.stats.cycle_gaps, link.stats.cycle_lost),
                    link.stats.cycle_gaps == 0))
    verdict.append(("TEL_FAST 帧率 ~100 Hz（实际 %.1f）" % hz, 90.0 <= hz <= 110.0))
    if work.n_total:
        verdict.append(("处理耗时 P99 < 4 ms（实际 %.0f us）" % work.p99,
                        work.p99 < 4000.0))
    if t is not None:
        verdict.append(("TEL_FAST 电压读数 %d mV" % t.voltage_mv, t.voltage_mv > 0))

    print()
    ok = True
    for text, good in verdict:
        print("  %s %s" % ("✅" if good else "❌", text))
        ok = ok and good

    print()
    print("  以下必须人工确认（脚本测不了）：")
    print("    · 用手掰某条腿 → 上面 q 的对应分量应跟着变")
    print("    · 倾斜机身（加 --tilt）→ 重力向量应指向地面方向")
    print("    · 万用表量电池 → 与 V= 的读数差应 < 50 mV")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
