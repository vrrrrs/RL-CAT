#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 假 STM32（测试替身 / test double）

⚠️ 这不是仿真，也不是模型 —— 它只是一个"能说协议、有状态机"的对端，
   用来在没有真板子的时候把树莓派侧的基础程序跑通、跑对。

它在一对内存 PTY 上工作：
    假 STM32 持 master 端
    pi/link.py 打开 slave 端（脚本会把 /dev/pts/N 打印出来）

模拟的行为（尽量贴近固件实际，包括它的"怪癖"）：
    · 100 Hz 无条件发 TEL_FAST（任何模式下都发）
    · 10 Hz 发 TEL_STATUS
    · 收 CMD_JOINT → 限幅(软限位) + 限速(0.15 rad/周期) + 一阶跟随
    · 收 CMD_MODE / CMD_JOINT.mode → 切模式（含 100 ms 最短保持）
    · 收 CMD_CALIB → 回 TEL_ACK(action, result=0)  ← ⚠️ 固件【永远】回 0
      action=0 启动一个约 8.4 s 的标定流程，成功后置 STAT_CALIBRATED
    · 失联 >30 ms → 降级；>100 ms → 急停趴下（q 以 0.5 rad/s 往趴下姿态走）
    · 关节反馈只有"整轮读成功"才刷新（否则保持上一次有效值）

⚠️ 刻意保留的固件怪癖（默认开启，可用开关关掉，用来对照）：
    · TEL_STATUS.voltage_mv 恒为 0 —— 真实电压只在 TEL_FAST 里
    · TEL_STATUS 的温度写的是【原始 °C】，而协议单位写的是 0.1 °C
    · 启动时在同一个串口上打一段 ASCII banner（树莓派必须能容忍垃圾字节）
    · CMD_CALIB 的 START 即使启动失败也回 result=0（静默失败）
    · err_flags 是 latch，只有重启才清

用法（单独跑，给另一个终端里的 pi_hello.py 用）：
    python3 fake_stm32.py --pty-file /tmp/fake_stm32.pty
    # 它会打印 /dev/pts/N，然后一直跑到 Ctrl-C
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
import pty
import random
import select
import sys
import threading
import time
from typing import Dict, List, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_PI = os.path.dirname(_HERE)
for p in (_PI, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from rlcat_proto import (  # noqa: E402
    FLAG_REQ_STATUS,
    LEN_TEL_STATUS,
    MODE_ACTIVE,
    MODE_DAMP,
    MODE_ESTOP,
    MODE_IDLE,
    MODE_STAND,
    N_JOINTS,
    ST_BATT_WARN,
    ST_CALIBRATED,
    ST_ESTOP,
    ST_IMU_OK,
    ST_LINK_OK,
    ST_READY,
    TYPE_CMD_CALIB,
    TYPE_CMD_JOINT,
    TYPE_CMD_MODE,
    TYPE_HEARTBEAT,
    TYPE_TEL_ACK,
    TYPE_TEL_FAST,
    TYPE_TEL_LOG,
    TelFast,
    TYPE_TEL_STATUS,
    FrameDecoder,
    encode_tel_fast,
    pack,
    parse_cmd_joint,
)

import config as C  # noqa: E402


# ============================================================================
# PTY 工具
# ============================================================================

def create_pty() -> tuple:
    """建一对 PTY，返回 (master_fd, keepalive_fd, slave_path)。

    ⚠️ keepalive 必须留着：如果从端一个 fd 都没有，往 master 写会直接 EIO。
       假 STM32 保留它但【从不读写】它，所以不会偷走客户端的数据。
    ⚠️ 必须把 tty 设成 raw —— 否则默认的 ECHO 会把假 STM32 自己发出的
       TEL_FAST 回显给它的 master，它会读到自己的帧，整个测试失效。
    """
    master, slave = pty.openpty()
    from link import PiLink
    PiLink._force_raw(slave)
    slave_path = os.ttyname(slave)
    return master, slave, slave_path


# ============================================================================
# 假 STM32
# ============================================================================

class FakeStm32:
    ITER_MS = 1                # 内层轮询周期
    CTRL_MS = 10               # 控制周期 = 100 Hz
    STATUS_MS = 100            # TEL_STATUS = 10 Hz
    MIN_HOLD_MS = 100          # 模式最短保持（固件同值）
    LINK_LOST_MS = 30
    LINK_ESTOP_MS = 100
    MAX_DELTA = C.MAX_DELTA_PER_CYCLE
    SIT_RATE = 0.5             # rad/s（固件 s ramp_rate）

    def __init__(self,
                 master_fd: int,
                 *,
                 calib_ms: int = 8400,
                 calib_fail: bool = False,
                 calib_start_ok: bool = True,
                 initial_calibrated: bool = False,
                 voltage_mv: int = 11800,
                 battery_valid: bool = True,
                 temp_c: int = 32,
                 hot_servo: int = -1,
                 rx_overflow: int = 0,
                 bus_err: int = 0,
                 bus_timeout: int = 0,
                 err_flags: int = 0,
                 status_voltage_zero: bool = True,
                 status_temp_raw_c: bool = True,
                 banner: bool = True,
                 drop_rate: float = 0.0,
                 crc_rate: float = 0.0,
                 seed: int = 12345,
                 startup_gate_ms: int = 1000,
                 verbose: bool = False):
        self.master = master_fd
        self.verbose = verbose
        self.calib_ms = calib_ms
        self.calib_fail = calib_fail
        self.calib_start_ok = calib_start_ok
        self.voltage_mv = voltage_mv
        self.battery_valid = battery_valid
        self.temp_c = temp_c
        # 每个舵机给一点差异，便于确认"12 路是分别读到的"（真实舵机也不会同温）
        self.temps = [temp_c + (i * 2) % 7 for i in range(N_JOINTS)]
        if 0 <= hot_servo < N_JOINTS:
            self.temps[hot_servo] = 75          # 超过固件 70°C 阈值 → 会强制趴下
        self.rx_overflow = rx_overflow
        self.bus_err = bus_err
        self.bus_timeout = bus_timeout
        self.status_voltage_zero = status_voltage_zero
        self.status_temp_raw_c = status_temp_raw_c
        self.banner = banner
        self.drop_rate = drop_rate
        self.crc_rate = crc_rate
        # ⚠️ 固件上电后有 RLCA_SERVO_STARTUP_GATE_MS(1000 ms) 不允许写舵机位置，
        #    这期间关节【根本不会动】。默认按固件值建模。
        self.startup_gate_ms = startup_gate_ms
        self.t_boot = time.perf_counter()

        self._rnd = random.Random(seed)
        self._stop = threading.Event()
        self._thr: Optional[threading.Thread] = None

        self.decoder = FrameDecoder(frame_timeout_ms=5)

        # ---- 状态（全部按逻辑索引 A）----
        self.q = [0.0] * N_JOINTS                 # 舵机反馈
        self.q_issued = [0.0] * N_JOINTS          # 固件已经下发给舵机的目标（受限速约束）
        self.servo_target = [0.0] * N_JOINTS      # Pi 要求的目标（已限幅）
        self.mode_req = MODE_IDLE                 # 请求的模式
        self.mode_eff = MODE_IDLE                 # 生效的模式
        self.calibrated = initial_calibrated
        self.calib_busy = False
        self.calib_t0 = 0.0
        self.torque = False

        self.cycle = 0
        self.phase = 0.0
        self.uptime0 = time.perf_counter()

        self.t_last_rx = 0.0
        self.ever_rx = False
        self.estop_ramp = [0.0] * N_JOINTS
        self.estop_active = False

        self.err_flags_latch = err_flags           # ⚠️ latch，只有 reset_err_flags() 才清
        # 测试用：强制关节反馈为某个固定值（模拟"零点偏了 / 关节卡住"）
        self.force_q: Optional[List[float]] = None
        self.n_rx_cmd = 0
        self.n_rx_seq_lost = 0
        self.last_seq: Optional[int] = None
        # ⭐ 记录【收到过哪些模式】。用来给"未标定时绝不能发 STAND/ACTIVE"
        #    这条安全规则做回归测试（STAND 无准入门，未标定会冲舵机）。
        self.modes_seen: set = set()

        # 统计（给测试断言用）
        self.tx_tel_fast = 0
        self.tx_tel_status = 0
        self.dropped = 0
        self.corrupted = 0

    # ------------------------------------------------------------- 生命周期

    def start(self) -> None:
        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thr is not None:
            self._thr.join(timeout=2.0)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
        return False

    def reset_err_flags(self) -> None:
        self.err_flags_latch = 0

    # ------------------------------------------------------------- 主循环

    def _run(self) -> None:
        if self.banner:
            # 固件上电会在 USART1 上打 ASCII banner（共享同一个串口）
            self._write(b"\r\nRL-CAT STM32 (fake) boot\r\n> ")

        t0 = time.perf_counter()
        next_ctrl = t0
        next_status = t0 + self.STATUS_MS / 1000.0

        while not self._stop.is_set():
            now = time.perf_counter()

            self._poll_rx(now)

            if now >= next_ctrl:
                self._step_ctrl(now)
                next_ctrl += self.CTRL_MS / 1000.0
                if next_ctrl < now - 0.05:            # 落后太多就重对齐，避免追帧雪崩
                    next_ctrl = now + self.CTRL_MS / 1000.0

            if now >= next_status:
                self._send_tel_status(now)
                next_status += self.STATUS_MS / 1000.0
                if next_status < now - 0.2:
                    next_status = now + self.STATUS_MS / 1000.0

            time.sleep(self.ITER_MS / 1000.0)

    # ------------------------------------------------------------- 收

    def _poll_rx(self, now: float) -> None:
        while True:
            try:
                r, _, _ = select.select([self.master], [], [], 0)
            except (OSError, ValueError):
                return
            if not r:
                return
            try:
                data = os.read(self.master, 4096)
            except OSError:
                return
            if not data:
                return
            for fr in self.decoder.feed(data, int(now * 1000)):
                self._on_frame(fr, now)

    def _on_frame(self, fr, now: float) -> None:
        self.t_last_rx = now
        self.ever_rx = True

        if fr.type == TYPE_CMD_JOINT:
            try:
                c = parse_cmd_joint(fr.payload)
            except ValueError:
                return
            self.n_rx_cmd += 1
            self._note_seq(fr.seq)
            if c.mode <= MODE_ESTOP:
                self.modes_seen.add(c.mode)      # 记录"Pi 请求过这个模式"

            # 模式（CMD_MODE 优先于 CMD_JOINT.mode，固件里是这么排的）
            # ⚠️ 标定期间固件用 app_mode_enter_calib() 把模式硬设成 IDLE，
            #    不接受任何模式切换请求。
            if c.mode <= MODE_ESTOP and not self.calib_busy:
                self.mode_req = c.mode
            if self.calib_busy:
                self.mode_req = MODE_IDLE
                self.mode_eff = MODE_IDLE
            elif self.mode_req == MODE_ACTIVE:
                # 准入：未标定 → 降级到 STAND
                if not self.calibrated:
                    self.mode_eff = MODE_STAND
                else:
                    self.mode_eff = MODE_ACTIVE
            else:
                self.mode_eff = self.mode_req

            # 限幅 + 限速（对"已下发的值"限速，固件就是这么做的）
            # ⚠️ 固件夹的是【硬件限位】（rlca_q_range），不是 Pi 的软限位。
            #    这里必须用同一套，否则测不出"Pi 侧过度钳位"这类问题。
            tgt = [C.clamp_q_hard(v, i) for i, v in enumerate(c.q_target)]
            for i in range(N_JOINTS):
                d = tgt[i] - self.q_issued[i]
                if d > self.MAX_DELTA:
                    d = self.MAX_DELTA
                elif d < -self.MAX_DELTA:
                    d = -self.MAX_DELTA
                self.q_issued[i] = self.q_issued[i] + d
                self.servo_target[i] = self.q_issued[i]

            if c.flags & FLAG_REQ_STATUS:
                self._send_tel_status(now, force=True)

        elif fr.type == TYPE_CMD_MODE:
            if len(fr.payload) >= 1 and fr.payload[0] <= MODE_ESTOP:
                self.modes_seen.add(fr.payload[0])
                if not self.calib_busy:
                    self.mode_req = fr.payload[0]
                    self.mode_eff = self.mode_req

        elif fr.type == TYPE_CMD_CALIB:
            action = fr.payload[0] if fr.payload else 0xFF
            self._on_calib(action, now)

        elif fr.type == TYPE_HEARTBEAT:
            pass                                   # 只刷新活性

    def _note_seq(self, seq: int) -> None:
        if self.last_seq is not None:
            expect = (self.last_seq + 1) & 0xFF
            if seq != expect:
                self.n_rx_seq_lost += (seq - expect) & 0xFF
        self.last_seq = seq

    def _on_calib(self, action: int, now: float) -> None:
        # ⚠️ 固件：解码器只接受 0..4；5(MANUAL_ZERO) 会被拒收，连 ACK 都不回
        if action > 4:
            return
        # ⚠️ 固件：所有分支一律回 result=0，没有成功/失败信息
        self._send(pack(TYPE_TEL_ACK, self.cycle & 0xFF, bytes([action, 0])))

        if action == 0:
            if not self.calib_start_ok:
                return                             # 静默失败（result 仍然是 0）
            self.calib_busy = True
            self.calib_t0 = now
            self.mode_req = MODE_IDLE
            self.mode_eff = MODE_IDLE
        elif action == 1:
            self.calib_busy = False
        elif action == 4:
            self.calibrated = True
        # action 2(QUERY)/3(SAVE)：固件里是空操作

    # ------------------------------------------------------------- 控制周期

    def _step_ctrl(self, now: float) -> None:
        self.cycle = (self.cycle + 1) & 0xFFFF
        self.phase = (self.phase + 0.03) % 1.0     # 3.0 Hz @ 100 Hz

        # 标定流程
        if self.calib_busy:
            if (now - self.calib_t0) * 1000.0 >= self.calib_ms:
                self.calib_busy = False
                self.calibrated = not self.calib_fail

        # 失联判定（固件：基于"距上次收到【任意】帧"）
        lost = (now - self.t_last_rx) * 1000.0 if self.ever_rx else float("inf")
        estop_now = lost > self.LINK_ESTOP_MS
        self.estop_active = estop_now

        # ⚠️ 上电门控：固件前 RLCA_SERVO_STARTUP_GATE_MS(1000 ms) 不写舵机位置，
        #    所以这期间关节【根本不会动】，不管模式是什么。
        gated = (now - self.t_boot) * 1000.0 < self.startup_gate_ms

        if estop_now:
            self.mode_eff = MODE_ESTOP
            if not any(self.estop_ramp):
                self.estop_ramp = list(self.q)
            # 目标角以 0.5 rad/s 往趴下姿态走（这是【指令】，不一定真的动）
            sit = [C.Q_MIN[i] + 0.1 * (C.Q_MAX[i] - C.Q_MIN[i]) for i in range(N_JOINTS)]
            step = self.SIT_RATE * (self.CTRL_MS / 1000.0)
            done = True
            for i in range(N_JOINTS):
                d = sit[i] - self.estop_ramp[i]
                if abs(d) > 0.01:
                    done = False
                if d > step:
                    d = step
                elif d < -step:
                    d = -step
                self.estop_ramp[i] += d
            if not gated:
                self.q = list(self.estop_ramp)
            self.torque = (not done) and (not gated)
            self._send_tel_fast(now)
            return

        # 模式 → 力矩与目标
        if self.mode_eff == MODE_ACTIVE and self.calibrated and lost <= self.LINK_LOST_MS:
            self.torque = True
        elif self.mode_eff == MODE_STAND:
            self.torque = True
            self.servo_target = list(C.Q_DEFAULT)
        elif self.mode_eff == MODE_IDLE or self.mode_eff == MODE_DAMP:
            self.torque = False
            self.servo_target = list(C.Q_DEFAULT)

        if gated:
            self.torque = False
            self._send_tel_fast(now)
            return

        # 舵机模型：力矩关 → 慢慢松回默认；开 → 一阶跟随目标
        if self.torque:
            for i in range(N_JOINTS):
                self.q[i] += (self.servo_target[i] - self.q[i]) * 0.25
        else:
            for i in range(N_JOINTS):
                self.q[i] += (C.Q_DEFAULT[i] - self.q[i]) * 0.05

        if self.force_q is not None:          # 测试用：钉死反馈值
            self.q = list(self.force_q)

        self._send_tel_fast(now)

    # ------------------------------------------------------------- 发

    def _write(self, blob: bytes) -> None:
        try:
            os.write(self.master, blob)
        except OSError:
            pass

    def _send(self, blob: bytes) -> None:
        self._write(blob)

    def _status_bits(self, now: float) -> int:
        """status 位域（固件 app_status_bits）。TEL_FAST 与 TEL_STATUS 共用同一份。"""
        lost = (now - self.t_last_rx) * 1000.0 if self.ever_rx else float("inf")
        status = ST_READY | ST_IMU_OK
        if self.calibrated:
            status |= ST_CALIBRATED
        if self.ever_rx and lost <= self.LINK_LOST_MS:
            status |= ST_LINK_OK
        if self.estop_active:
            status |= ST_ESTOP
        if self.battery_valid:
            if self.voltage_mv <= C.BATT_WARN_MV:
                status |= ST_BATT_WARN
        return status & 0xFF

    def _send_tel_fast(self, now: float) -> None:
        # 故障注入
        if self.drop_rate and self._rnd.random() < self.drop_rate:
            self.dropped += 1
            return

        import math
        t = TelFast(
            q=list(self.q),
            qd=[0.0] * N_JOINTS,
            tau=[0.0] * N_JOINTS,
            quat=(1.0, 0.0, 0.0, 0.0),
            gyro=(0.0, 0.0, 0.0),
            gait_sin=math.sin(2 * math.pi * self.phase),
            gait_cos=math.cos(2 * math.pi * self.phase),
            cycle_cnt=self.cycle,
            voltage_mv=self.voltage_mv if self.battery_valid else 0,
            status=self._status_bits(now),
            err_flags=self.err_flags_latch & 0xFF,
        )
        blob = pack(TYPE_TEL_FAST, self.cycle & 0xFF, encode_tel_fast(t))

        if self.crc_rate and self._rnd.random() < self.crc_rate:
            self.corrupted += 1
            b = bytearray(blob)
            b[-1] ^= 0xFF
            blob = bytes(b)

        self._write(blob)
        self.tx_tel_fast += 1

    def _send_tel_status(self, now: float, force: bool = False) -> None:
        # ⚠️ 固件怪癖：voltage_mv 写死 0；温度写原始 °C（协议单位是 0.1 °C）
        volt = 0 if self.status_voltage_zero else (
            self.voltage_mv if self.battery_valid else 0)

        payload = bytearray(LEN_TEL_STATUS)
        import struct
        struct.pack_into("<I", payload, 0, int((now - self.uptime0) * 1000) & 0xFFFFFFFF)
        struct.pack_into("<H", payload, 4, volt & 0xFFFF)
        for i in range(N_JOINTS):
            v = self.temps[i] if self.status_temp_raw_c else self.temps[i] * 10
            struct.pack_into("<h", payload, 6 + 2 * i, v)
        struct.pack_into("<H", payload, 30, self.rx_overflow & 0xFFFF)
        struct.pack_into("<H", payload, 32, self.bus_err & 0xFFFF)
        struct.pack_into("<H", payload, 34, self.bus_timeout & 0xFFFF)
        payload[36] = self.mode_req & 0xFF                  # ⚠️ 请求的模式，不是生效的
        payload[37] = self._status_bits(now) & 0xFF         # 与 TEL_FAST 同一份
        payload[38] = self.err_flags_latch & 0xFF
        payload[39] = self.n_rx_seq_lost & 0xFF
        self._send(pack(TYPE_TEL_STATUS, self.cycle & 0xFF, bytes(payload)))
        self.tx_tel_status += 1


# ============================================================================
# 命令行
# ============================================================================

def main() -> int:
    ap = argparse.ArgumentParser(description="RL-CAT 假 STM32（PTY 测试替身）")
    ap.add_argument("--pty-file", default="/tmp/fake_stm32.pty",
                    help="把 PTY 从设备路径写到这里，供 pi_hello.py 使用")
    ap.add_argument("--calib-ms", type=int, default=8400, help="模拟标定耗时")
    ap.add_argument("--calib-fail", action="store_true", help="注入标定失败")
    ap.add_argument("--calib-start-fail", action="store_true",
                    help="注入 START 静默失败（回 ACK 但不启动）")
    ap.add_argument("--calibrated", action="store_true", help="一开始就处于已标定")
    ap.add_argument("--voltage-mv", type=int, default=11800)
    ap.add_argument("--battery-invalid", action="store_true")
    ap.add_argument("--temp-c", type=int, default=32)
    ap.add_argument("--hot-servo", type=int, default=-1,
                    help="让逻辑索引 N 的舵机报 75°C（超过固件 70°C 阈值）")
    ap.add_argument("--rx-overflow", type=int, default=0)
    ap.add_argument("--bus-err", type=int, default=0)
    ap.add_argument("--bus-timeout", type=int, default=0)
    ap.add_argument("--err-flags", type=lambda s: int(s, 0), default=0,
                    help="注入 latch 的错误位，例如 0x04")
    ap.add_argument("--no-banner", action="store_true")
    ap.add_argument("--status-voltage-real", action="store_true",
                    help="关掉固件怪癖：TEL_STATUS 里给真电压")
    ap.add_argument("--status-temp-deci", action="store_true",
                    help="关掉固件怪癖：TEL_STATUS 温度给 0.1°C")
    ap.add_argument("--drop-rate", type=float, default=0.0, help="丢 TEL_FAST 的概率")
    ap.add_argument("--crc-rate", type=float, default=0.0, help="TEL_FAST CRC 出错概率")
    ap.add_argument("--startup-gate-ms", type=int, default=1000,
                    help="上电后多久内不写舵机位置（固件是 1000 ms）")
    ap.add_argument("--seconds", type=float, default=0.0, help="跑多久后自动退出（0=一直跑）")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    master, keepalive, path = create_pty()
    fake = FakeStm32(
        master,
        calib_ms=args.calib_ms,
        calib_fail=args.calib_fail,
        calib_start_ok=not args.calib_start_fail,
        initial_calibrated=args.calibrated,
        voltage_mv=args.voltage_mv,
        battery_valid=not args.battery_invalid,
        temp_c=args.temp_c,
        hot_servo=args.hot_servo,
        rx_overflow=args.rx_overflow,
        bus_err=args.bus_err,
        bus_timeout=args.bus_timeout,
        err_flags=args.err_flags,
        status_voltage_zero=not args.status_voltage_real,
        status_temp_raw_c=not args.status_temp_deci,
        banner=not args.no_banner,
        drop_rate=args.drop_rate,
        crc_rate=args.crc_rate,
        startup_gate_ms=args.startup_gate_ms,
        verbose=args.verbose,
    )
    fake.start()

    try:
        with open(args.pty_file, "w") as f:
            f.write(path)
    except OSError as e:
        print("!! 写不了 %s: %s" % (args.pty_file, e))

    print("=" * 66)
    print(" 假 STM32 已启动")
    print("   PTY 从设备 : %s" % path)
    print("   路径已写到 : %s" % args.pty_file)
    print("   用法       : python3 pi_hello.py --port %s" % path)
    print("=" * 66)

    t0 = time.perf_counter()
    try:
        while True:
            time.sleep(0.5)
            if args.verbose:
                print("  cycle=%d mode_req=%d mode_eff=%d q0=%.3f tx=%d rx=%d"
                      % (fake.cycle, fake.mode_req, fake.mode_eff, fake.q[0],
                         fake.tx_tel_fast, fake.n_rx_cmd))
            if args.seconds and (time.perf_counter() - t0) >= args.seconds:
                break
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
    finally:
        fake.stop()
        os.close(keepalive)
        os.close(master)
    return 0


if __name__ == "__main__":
    sys.exit(main())
