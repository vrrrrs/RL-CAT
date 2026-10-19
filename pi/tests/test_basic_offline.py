#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 树莓派侧基础层：离线端到端测试（不需要任何硬件）

原理：用一对内存 PTY 把【假 STM32】和【真的 pi/link.py】连起来，
      于是链路层、遥测、模式、标定、故障处理全都能在 PC 或 Pi 上跑通。

⚠️ 它验证的是【协议与状态机】，不是电信号。1 Mbps 的物理层误码率
   要用 pi/tools/uart_loopback.py（短接 GPIO14/15）单独实测。

用法：
    python3 tests/test_basic_offline.py            # 全部
    python3 tests/test_basic_offline.py -v         # 打印每条通过项
退出码：0 = 全过；1 = 有失败
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
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PI = os.path.dirname(_HERE)
for p in (_PI, os.path.join(_PI, "tools")):
    if p not in sys.path:
        sys.path.insert(0, p)

import config as C                                              # noqa: E402
from link import PiLink                                         # noqa: E402
from fake_stm32 import FakeStm32, create_pty                    # noqa: E402
from rlcat_proto import (                                       # noqa: E402
    MODE_ACTIVE,
    MODE_ESTOP,
    MODE_IDLE,
    MODE_STAND,
    N_JOINTS,
    ST_CALIBRATED,
    TYPE_TEL_FAST,
    TYPE_TEL_STATUS,
)

VERBOSE = False
RESULTS: list = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    RESULTS.append((name, bool(cond), detail))
    if VERBOSE or not cond:
        print("  %s %s%s" % ("✅" if cond else "❌", name,
                             ("   [%s]" % detail) if detail else ""))
    return bool(cond)


def near(name: str, got: float, want: float, tol: float) -> bool:
    return check(name, abs(got - want) <= tol, "got %.6g want %.6g" % (got, want))


def wait_until(pred, timeout_s: float = 2.0, poll_s: float = 0.005) -> bool:
    t_end = time.perf_counter() + timeout_s
    while time.perf_counter() < t_end:
        if pred():
            return True
        time.sleep(poll_s)
    return pred()


class Rig:
    """一对 PTY + 假 STM32 + 真的 PiLink。"""

    def __init__(self, **fake_kw):
        # 测试里不需要等固件那 1000 ms 的上电写入门控；单独用 F 组测它
        fake_kw.setdefault("startup_gate_ms", 0)
        self.master, self.keep, self.path = create_pty()
        self.fake = FakeStm32(self.master, **fake_kw)
        self.fake.start()
        self.link = PiLink(self.path, baud=115200, verbose=False)
        self.link.open()

    def close(self) -> None:
        try:
            self.link.close()
        finally:
            self.fake.stop()
            os.close(self.keep)
            os.close(self.master)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # --- 便捷 ---
    def pump_for(self, seconds: float) -> None:
        t_end = time.perf_counter() + seconds
        while time.perf_counter() < t_end:
            self.link.pump()
            time.sleep(0.001)

    def drain_status(self, timeout_s: float = 2.0):
        return self.link.wait_status(timeout_ms=int(timeout_s * 1000))


# ============================================================================
# 测试组
# ============================================================================

def t_link_and_telemetry() -> None:
    print("\n[A] 链路与遥测")
    with Rig(initial_calibrated=True, voltage_mv=11750, temp_c=33) as r:
        # A1 收得到 TEL_FAST
        got = r.link.wait_tel_fast(timeout_ms=1500)
        check("A1 能收到 TEL_FAST", got is not None)

        # A2 帧率 ~100 Hz
        n0 = r.link.stats.rx_by_type.get(TYPE_TEL_FAST, 0)
        t0 = time.perf_counter()
        r.pump_for(1.0)
        n1 = r.link.stats.rx_by_type.get(TYPE_TEL_FAST, 0)
        hz = (n1 - n0) / (time.perf_counter() - t0)
        check("A2 TEL_FAST 帧率 ~100 Hz", 90.0 <= hz <= 110.0, "%.1f Hz" % hz)

        # A3 cycle_cnt 连续
        check("A3 cycle_cnt 连续无跳变", r.link.stats.cycle_gaps == 0,
              "gaps=%d lost=%d" % (r.link.stats.cycle_gaps, r.link.stats.cycle_lost))

        # A4 CRC 零错
        check("A4 CRC 零错", r.link.stats.crc_errors == 0,
              "crc_err=%d" % r.link.stats.crc_errors)

        # A5 TEL_FAST 里的电压是真的
        t = r.link.last_tel
        check("A5 TEL_FAST 电压可用", t is not None and t.voltage_mv == 11750,
              "v=%s" % (t.voltage_mv if t else None))

        # A6 步态相位在 3 Hz 附近转
        s0 = r.link.last_tel.gait_sin
        r.pump_for(0.05)
        s1 = r.link.last_tel.gait_sin
        check("A6 步态相位在推进", s0 != s1, "%.4f -> %.4f" % (s0, s1))

        # A7 TEL_STATUS 的固件怪癖：电压恒 0
        st = r.drain_status()
        check("A7 TEL_STATUS 电压为 0（固件怪癖，别用它）",
              st is not None and st.voltage_mv == 0,
              "v=%s" % (st.voltage_mv if st else None))

        # A8 TEL_STATUS 温度是原始 °C 而不是 0.1°C
        check("A8 TEL_STATUS 温度是原始 °C（不是 0.1°C）",
              st is not None and st.temperature[0] == 33,
              "t=%s" % (st.temperature[0] if st else None))

        # A8b TEL_STATUS 的 status 位域是真状态（不是写死的 0）
        check("A8b TEL_STATUS 带真实 status 位（CALIBRATED 已置位）",
              st is not None and bool(st.status & ST_CALIBRATED),
              "status=0x%02X" % (st.status if st else 0))

        # A9 中途混入垃圾字节（固件的 ASCII shell / 内核日志）后还能同步
        for _ in range(5):
            os.write(r.master, b"\r\nsome kernel log line\r\n> ")
            r.pump_for(0.05)
        n_before = r.link.stats.rx_by_type.get(TYPE_TEL_FAST, 0)
        os.write(r.master, b"garbage" * 20 + b"\x00\xff\xaa")
        r.pump_for(0.2)
        n_after = r.link.stats.rx_by_type.get(TYPE_TEL_FAST, 0)
        check("A9 垃圾字节后能重新同步", n_after > n_before,
              "%d -> %d" % (n_before, n_after))


def t_modes_and_motion() -> None:
    print("\n[B] 模式与运动")
    # B1 未标定 → ACTIVE 静默降级为 STAND
    #    ⚠️ 必须【持续】发指令：固件的失联判定基于"距上次收到任意帧"，
    #       只发一帧然后等 150 ms，早就超过 100 ms 又回 ESTOP 了。
    with Rig(initial_calibrated=False) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        for _ in range(15):
            r.link.send_cmd_joint([0.3] * N_JOINTS, mode=MODE_ACTIVE)
            r.pump_for(0.01)
        check("B1 未标定时 ACTIVE 降级为 STAND",
              r.fake.mode_eff == MODE_STAND, "mode_eff=%d" % r.fake.mode_eff)

    # B2/B3/B4 已标定 → ACTIVE 跟踪目标
    with Rig(initial_calibrated=True) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        # 目标定在 thigh(索引4) 0.6 rad
        target = [0.0] * N_JOINTS
        target[4] = 0.6
        r.link.send_cmd_joint(target, mode=MODE_ACTIVE)
        r.pump_for(0.03)
        near("B2 限速生效：第一个周期只走 0.15 rad",
             r.fake.q_issued[4], C.MAX_DELTA_PER_CYCLE, 1e-6)

        for _ in range(40):
            r.link.send_cmd_joint(target, mode=MODE_ACTIVE)
            r.pump_for(0.01)
        check("B3 ACTIVE 下关节跟到目标",
              abs(r.fake.q[4] - 0.6) < 0.05, "q4=%.4f" % r.fake.q[4])
        check("B4 力矩是打开的", r.fake.torque is True)

    # B5 限幅：反复给远超限位的值，q 不能越界
    with Rig(initial_calibrated=True) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        wild = [0.0] * N_JOINTS
        wild[4] = 10.0            # thigh 软上限约 1.4835 rad
        wild[0] = -10.0           # hip 软下限约 0.0873 rad
        for _ in range(60):
            r.link.send_cmd_joint(wild, mode=MODE_ACTIVE)
            r.pump_for(0.01)
        # ⚠️ B5 是【把野值直接塞给链路】，绕过了 Pi 侧的软限位钳位，
        #    所以这里验证的是【固件】按硬件限位夹紧这件事。
        #    Pi 侧的软限位钳位由 I3 与 config.py 自检负责。
        check("B5a thigh 被固件夹在硬件上限内",
              r.fake.q[4] <= C.Q_MAX[4] + 1e-6,
              "q4=%.4f 硬件上限=%.4f" % (r.fake.q[4], C.Q_MAX[4]))
        check("B5b hip 被固件夹在硬件下限内",
              r.fake.q[0] >= C.Q_MIN[0] - 1e-6,
              "q0=%.4f 硬件下限=%.4f" % (r.fake.q[0], C.Q_MIN[0]))

        # ⭐ B5c 回归用例：软限位【绝不能】把站姿排除掉。
        #    曾经按"统一留 5° 余量"算，hip 变成 [5°,35°]，而 q_default=0°
        #    正好在区间外 → 所有髋关节目标被抬到 5° → 机器人永远站不正。
        worst = max(abs(C.clamp_q(C.Q_DEFAULT[i], i) - C.Q_DEFAULT[i])
                    for i in range(N_JOINTS))
        check("B5c 站姿(全 0) 通过软限位钳位时不被改动", worst < 1e-12,
              "hip 软限位 = %.1f°~%.1f°，最大改动 %.3g"
              % (C.Q_SOFT_MIN[0] * 180.0 / math.pi,
                 C.Q_SOFT_MAX[0] * 180.0 / math.pi, worst))

    # B6 NaN 不能把关节搞乱
    with Rig(initial_calibrated=True) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        nanvec = [0.0] * N_JOINTS
        nanvec[4] = float("nan")
        for _ in range(20):
            r.link.send_cmd_joint(nanvec, mode=MODE_ACTIVE)
            r.pump_for(0.01)
        # ⚠️ 容差不能用 1e-6：上电那一段（还没收到帧时）固件在走趴下斜坡，
        #    残留会让 q4 偏离 0 约 1e-5 量级。判据是"不发散、不出 NaN"。
        check("B6 NaN 目标被当作默认值处理（不产生 NaN、不发散）",
              r.fake.q[4] == r.fake.q[4] and abs(r.fake.q[4]) < 0.02,
              "q4=%.6f" % r.fake.q[4])

    # B7 MODE_IDLE 卸力矩
    with Rig(initial_calibrated=True) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        r.link.send_cmd_mode(MODE_IDLE)
        r.pump_for(0.1)
        check("B7 MODE_IDLE 卸力矩", r.fake.torque is False)


def t_calibration() -> None:
    print("\n[C] 标定")
    # C1 START 会回 ACK(action, result=0)
    with Rig(initial_calibrated=False, calib_ms=300) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        r.link.send_cmd_calib(C.CALIB_START)
        ack = r.link.wait_ack(timeout_ms=1000)
        check("C1a CMD_CALIB(START) 收到 TEL_ACK", ack is not None, "ack=%r" % (ack,))
        check("C1b ACK 的 result 恒为 0（固件没有成功/失败信息）",
              ack == (C.CALIB_START, 0), "ack=%r" % (ack,))

        # C2 完成后 STAT_CALIBRATED 置位
        def calibrated_seen():
            r.link.pump()
            t = r.link.last_tel
            return t is not None and bool(t.status & ST_CALIBRATED)

        check("C2 标定完成后 STAT_CALIBRATED 置位",
              wait_until(calibrated_seen, timeout_s=2.0))
        check("C3 标定后假固件确实置了 calibrated", r.fake.calibrated is True)

    # C4 MANUAL_ZERO(5) 会被解码器拒收 —— 连 ACK 都没有
    with Rig(initial_calibrated=False) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        r.link.last_ack = None
        r.link.send_cmd_calib(C.CALIB_MANUAL_ZERO_UNREACHABLE)
        ack = r.link.wait_ack(timeout_ms=400)
        check("C4 CMD_CALIB(5) 被固件拒收（无 ACK）—— 树莓派不能依赖它",
              ack is None, "ack=%r" % (ack,))

    # C5 START 静默失败：仍然回 result=0，但流程根本没跑
    with Rig(initial_calibrated=False, calib_start_ok=False, calib_ms=100) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        r.link.send_cmd_calib(C.CALIB_START)
        ack = r.link.wait_ack(timeout_ms=800)
        r.pump_for(0.3)
        check("C5a 启动失败也回 result=0（静默失败陷阱）",
              ack == (C.CALIB_START, 0), "ack=%r" % (ack,))
        check("C5b 流程其实没启动", r.fake.calib_busy is False and
              r.fake.calibrated is False)

    # C6 失败注入：流程跑完但不置 CALIBRATED → 这是 Pi 唯一能看到的"失败"
    with Rig(initial_calibrated=False, calib_fail=True, calib_ms=200) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        r.link.send_cmd_calib(C.CALIB_START)
        r.link.wait_ack(timeout_ms=800)
        r.pump_for(0.6)
        check("C6 标定失败时 STAT_CALIBRATED 始终不置位（唯一的失败信号）",
              r.fake.calibrated is False and
              not bool(r.link.last_tel.status & ST_CALIBRATED))


def t_fault_injection() -> None:
    print("\n[D] 故障注入")
    # D1 CRC 错
    with Rig(initial_calibrated=True, crc_rate=0.3) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        r.pump_for(1.0)
        check("D1 CRC 错被计数", r.link.stats.crc_errors > 0,
              "crc_err=%d" % r.link.stats.crc_errors)

    # D2 丢帧
    with Rig(initial_calibrated=True, drop_rate=0.3) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        r.pump_for(1.0)
        check("D2 cycle_cnt 丢帧被检测", r.link.stats.cycle_gaps > 0,
              "gaps=%d lost=%d" % (r.link.stats.cycle_gaps, r.link.stats.cycle_lost))

    # D3 失联：不发任何指令 → 固件应进入 ESTOP
    with Rig(initial_calibrated=True) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        r.pump_for(0.5)              # 此时已经从没发过指令
        from rlcat_proto import ST_ESTOP
        t = r.link.last_tel
        check("D3 长时间不发帧 → 固件进入 ESTOP（它的安全设计）",
              t is not None and bool(t.status & ST_ESTOP),
              "status=0x%02X" % (t.status if t else 0))

    # D4 但只发心跳就能骗过它的失联判定（⚠️ 危险，必须知道）
    with Rig(initial_calibrated=True) as r:
        r.link.wait_tel_fast(timeout_ms=1000)
        t_end = time.perf_counter() + 0.5
        while time.perf_counter() < t_end:
            r.link.send_heartbeat()
            r.pump_for(0.02)
        t = r.link.last_tel
        from rlcat_proto import ST_LINK_OK, ST_ESTOP
        check("D4 只发 HEARTBEAT 就能让固件以为链路正常（所以它不能替代控制指令）",
              t is not None and bool(t.status & ST_LINK_OK)
              and not bool(t.status & ST_ESTOP),
              "status=0x%02X" % (t.status if t else 0))


def t_reply_latency() -> None:
    print("\n[E] 请求-应答耗时（模型 A 的 4 ms 窗口）")
    import stats as S
    with Rig(initial_calibrated=True) as r:
        lat = S.Percentiles(capacity=5000)
        r.pump_for(0.2)
        for _ in range(200):
            t = r.link.wait_tel_fast(timeout_ms=50)
            if t is None:
                continue
            r.link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE)
            lat.add(r.link.lost_ms() * 0 + (time.perf_counter() - r.link.t_rx_tel_fast) * 1e6)
        check("E1 采集到足够的样本", lat.n_total >= 150, "n=%d" % lat.n_total)
        check("E2 回复耗时 P99 < 4000 us", lat.p99 < 4000.0, lat.line(""))
        if VERBOSE:
            print("      %s" % lat.line(""))


# ============================================================================

def t_boot_behavior() -> None:
    """⭐ 上电行为：这是 Pi 程序必须知道、又最容易忽略的一条。"""
    print("\n[F] 上电与失联行为")
    from rlcat_proto import ST_ESTOP, ST_LINK_OK

    # F1 一帧都没收到过 → 固件按失联处理 → 直接 ESTOP
    with Rig(initial_calibrated=True) as r:
        t = r.link.wait_tel_fast(timeout_ms=1500)
        check("F1 上电后（还没收到任何 Pi 帧）固件就进 ESTOP",
              t is not None and bool(t.status & ST_ESTOP)
              and not bool(t.status & ST_LINK_OK),
              "status=0x%02X" % (t.status if t else 0))

    # F2 开始周期发帧后 ESTOP 自动解除（但模式不会自动回到 ACTIVE，要主动请求）
    with Rig(initial_calibrated=True) as r:
        r.link.wait_tel_fast(timeout_ms=1500)
        for _ in range(30):
            r.link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE)
            r.pump_for(0.01)
        t = r.link.last_tel
        check("F2 周期发帧后 ESTOP 解除、LINK_OK 置位",
              t is not None and not bool(t.status & ST_ESTOP)
              and bool(t.status & ST_LINK_OK),
              "status=0x%02X" % (t.status if t else 0))

    # F3 失联 30~100 ms 之间：降级但不急停
    with Rig(initial_calibrated=True) as r:
        r.link.wait_tel_fast(timeout_ms=1500)
        r.link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_ACTIVE)
        r.pump_for(0.05)          # 停在这里：距上次发帧 ~50 ms
        t = r.link.last_tel
        check("F3 失联 30~100 ms：LINK_OK 清掉但还没 ESTOP",
              t is not None and not bool(t.status & ST_LINK_OK)
              and not bool(t.status & ST_ESTOP),
              "status=0x%02X" % (t.status if t else 0))

    # F4 ⚠️ 上电 1000 ms 内固件不写舵机位置 —— 关节【纹丝不动】，
    #    即使你已经在发 ACTIVE + 目标角。上真机时"发了指令但不动"就是它。
    with Rig(initial_calibrated=True, startup_gate_ms=1000) as r:
        r.link.wait_tel_fast(timeout_ms=1500)
        tgt = [0.0] * N_JOINTS
        tgt[4] = 0.5
        for _ in range(30):        # 约 0.3 s，仍在门控期内
            r.link.send_cmd_joint(tgt, mode=MODE_ACTIVE)
            r.pump_for(0.01)
        check("F4 上电门控期内关节不动（虽然目标角已经在变）",
              abs(r.fake.q[4]) < 1e-9 and abs(r.fake.q_issued[4]) > 0.1,
              "q4=%.6g q_issued4=%.4f" % (r.fake.q[4], r.fake.q_issued[4]))


def t_telemetry_semantics() -> None:
    """[G] 遥测语义层（纯逻辑 + 一条端到端）。"""
    print("\n[G] 遥测语义（阶段 2）")
    import telemetry as T
    from rlcat_proto import (ERRF_BUS_TIMEOUT, ERRF_OVERHEAT, ST_IMU_OK as IMU_OK,
                             ST_READY as RDY, ST_CALIBRATED as CAL,
                             ST_LINK_OK as LNK, ST_ESTOP as EST)

    full = RDY | CAL | LNK | IMU_OK

    # G1 生效模式推断
    m, _ = T.derive_effective_mode(MODE_ACTIVE, full)
    check("G1 ACTIVE + 全部状态位 → 生效模式确为 ACTIVE", m == MODE_ACTIVE,
          "m=%d" % m)
    m, why = T.derive_effective_mode(MODE_ACTIVE, full & ~CAL)
    check("G1b 缺 CALIBRATED → 降级 STAND（这是最容易误解的一条）",
          m == MODE_STAND and "NOT_CALIB" in why, "%d %s" % (m, why))
    m, why = T.derive_effective_mode(MODE_ACTIVE, full & ~IMU_OK)
    check("G1c 缺 IMU_OK → 降级 STAND", m == MODE_STAND and "NO_IMU" in why,
          "%d %s" % (m, why))
    m, why = T.derive_effective_mode(MODE_ACTIVE, full & ~LNK)
    check("G1d 缺 LINK_OK → 降级 STAND", m == MODE_STAND and "NO_LINK" in why,
          "%d %s" % (m, why))
    m, _ = T.derive_effective_mode(MODE_STAND, full | EST)
    check("G1e ESTOP 位优先于一切", m == MODE_ESTOP, "m=%d" % m)
    m, _ = T.derive_effective_mode(MODE_STAND, full)
    check("G1f 非 ACTIVE 的请求原样返回", m == MODE_STAND, "m=%d" % m)

    # G2 电池等级（判据是 <=，与固件一致）
    cases = [(12000, 0), (10501, 0), (10500, 1), (9901, 1),
             (9900, 2), (9301, 2), (9300, 3), (0, -1)]
    bad = [(v, T.battery_level(v)[0], want) for v, want in cases
           if T.battery_level(v)[0] != want]
    check("G2 电池等级在 4 个阈值上正确（含 <= 语义与读数无效）",
          not bad, "错: %r" % bad)

    # G3 温度单位换算与等级
    check("G3a raw 单位原样", T.temp_to_c(31, "raw") == 31.0)
    check("G3b deci 单位除以 10", T.temp_to_c(310, "deci") == 31.0)
    lv = [T.temp_level(x) for x in (20, 59.9, 60, 70, 70.1)]
    check("G3c 温度等级（>=60 偏高；>70 超温）", lv == [0, 0, 1, 1, 2], "%r" % lv)

    # G4 位域解码
    check("G4a status 解码", T.status_str(RDY | IMU_OK) == "READY|IMU_OK",
          T.status_str(RDY | IMU_OK))
    check("G4b err=0 显示'无'", T.err_str(0) == "无", T.err_str(0))
    e = T.err_str(ERRF_OVERHEAT | ERRF_BUS_TIMEOUT)
    check("G4c err 多位置位都能列出来",
          "OVERHEAT" in e and "BUS_TIMEOUT" in e, e)

    # G5 端到端：假固件注入超温舵机 + latch 错误位 → 面板能读到并告警
    with Rig(initial_calibrated=True, hot_servo=2, err_flags=ERRF_BUS_TIMEOUT,
             bus_err=7, bus_timeout=3, rx_overflow=1) as r:
        st = r.drain_status(2.0)
        r.link.pump()
        tel = r.link.last_tel
        check("G5a 12 路温度是分别读到的（第 2 路被注入成 75）",
              st is not None and st.temperature[2] == 75
              and st.temperature[0] != st.temperature[2],
              "temps=%s" % (st.temperature[:4] if st else None))
        check("G5b 总线计数能读到", st is not None and st.bus_err == 7
              and st.bus_timeout == 3 and st.rx_overflow == 1,
              "bus_err=%s" % (st.bus_err if st else None))
        check("G5c latch 错误位能读到", tel is not None
              and tel.err_flags == ERRF_BUS_TIMEOUT,
              "err=0x%02X" % (tel.err_flags if tel else 0))

        import status_panel as SP
        lines = SP.render_panel(r.link, tel, st, temp_unit="raw", elapsed_s=1.0)
        text = "\n".join(lines)
        check("G5d 面板能渲染且含温度与告警",
              "75.0" in text and "超温" in text and "err_flags" in text)
        warns = SP.health_warnings(r.link, tel, st, "raw")
        check("G5e 健康检查把超温和 latch 错误都报出来",
              any("超温" in w for w in warns) and any("err_flags" in w for w in warns),
              "%d 条告警" % len(warns))
        if VERBOSE:
            print("\n".join("      " + x for x in lines))


def t_calibration_session() -> None:
    """[H] 阶段 3 的标定会话（pi/calib.py）—— 只用协议，不碰 shell。"""
    print("\n[H] 标定会话（阶段 3）")
    import calib as CAL
    from rlcat_proto import ST_ESTOP, ST_CALIBRATED

    # H1 未标定 → 启动 → STAT_CALIBRATED 由 0 变 1 → 成功
    with Rig(initial_calibrated=False, calib_ms=600) as r:
        sess = CAL.CalibSession(r.link, quiet=True)
        cont, notes = sess.precheck()
        check("H1a 前置检查通过", cont, "; ".join(notes))
        ok, why = sess.start()
        check("H1b START 收到 ACK", ok, why)
        res = sess.wait_done(timeout_s=4.0, was_calibrated=False)
        check("H1c 标定判定为成功（0→1 跳变）", res.ok, str(res))
        check("H1d 耗时接近模拟的 0.6 s", 0.5 <= res.seconds <= 2.0,
              "%.2f s" % res.seconds)

    # H2 失败注入 → 不会出现 0→1 → 判为超时/失败
    with Rig(initial_calibrated=False, calib_fail=True, calib_ms=300) as r:
        sess = CAL.CalibSession(r.link, quiet=True)
        sess.start()
        res = sess.wait_done(timeout_s=1.5, was_calibrated=False)
        check("H2 标定失败被判出来（协议里只有'没跳变'这一个信号）",
              not res.ok and res.verdict == "timeout", str(res))

    # H3 ⭐ 保活是必须的：不发帧的话，8~10 s 的标定中途必然被 ESTOP 打断
    with Rig(initial_calibrated=False, calib_ms=600) as r:
        sess = CAL.CalibSession(r.link, quiet=True)
        sess.keepalive = lambda: None          # 故意不发帧
        sess.start()
        res = sess.wait_done(timeout_s=2.0, was_calibrated=False)
        check("H3 不保活 → 标定被 ESTOP 打断（所以必须持续发帧）",
              res.verdict == "estop", str(res))

    # H4 ⚠️ 起始就已标定 → 失败的重标定无法用协议识别（已知局限）
    with Rig(initial_calibrated=True, calib_fail=True, calib_ms=300) as r:
        sess = CAL.CalibSession(r.link, quiet=True)
        sess.start()
        res = sess.wait_done(timeout_s=1.5, was_calibrated=True)
        check("H4 起始已标定时，判定里明确写出'无法区分成功/失败'",
              "无法区分" in res.detail or "无法判断" in res.detail, res.detail[:60])

    # H5 --show 的判定：已标定 → 0；未标定 → 1
    with Rig(initial_calibrated=True) as r:
        check("H5a 已标定时 --show 退出码 0", CAL.cmd_show(r.link) == 0)
    with Rig(initial_calibrated=False, calib_ms=100) as r:
        check("H5b 未标定时 --show 退出码 1", CAL.cmd_show(r.link) == 1)

    # H6 标定后自检能发现"零点明显偏了"
    with Rig(initial_calibrated=True) as r:
        r.fake.force_q = [0.9] * N_JOINTS    # 伪造：站姿下 q 竟然有 0.9 rad
        sess = CAL.CalibSession(r.link, quiet=True)
        ok, notes = sess.postcheck()
        check("H6 自检能报出关节角超限/零点可疑", not ok,
              "; ".join(n for n in notes if n.startswith(("❌", "⚠️"))))


def _jog_to(runner, state, idx: int, deg: float, dwell: float = 1.2,
            tol: float = 1.5):
    """把第 idx 个关节的目标设成 deg，等它收敛。"""
    state.sel = idx
    state.set_deg(idx, deg)
    ok, worst = runner.wait_settled(tol_deg=tol, timeout_s=dwell)
    return ok, worst


def t_jog() -> None:
    """[I] 阶段 4：单关节点动。"""
    print("\n[I] 单关节点动（阶段 4）")
    import jog as JOG

    DEG = 180.0 / 3.141592653589793

    # I0 纯逻辑：命令解析（不需要链路）
    st = JOG.JogState(sel=4)
    cont, msg = JOG.handle_command(st, "j 9")
    check("I0a j <idx> 能选关节", cont and st.sel == 9, msg)
    JOG.handle_command(st, "lf_thigh")
    check("I0b 用关节名也能选", st.sel == 4, "sel=%d" % st.sel)
    JOG.handle_command(st, "s 2.5")
    check("I0c s <度> 改步长", abs(st.step_deg - 2.5) < 1e-9, "%.2f" % st.step_deg)
    JOG.handle_command(st, "+")
    check("I0d + 按步长加", abs(st.q[4] * DEG - 2.5) < 1e-6, "%.3f°" % (st.q[4] * DEG))
    JOG.handle_command(st, "30")
    check("I0e 纯数字 = 绝对角度", abs(st.q[4] * DEG - 30.0) < 1e-6,
          "%.3f°" % (st.q[4] * DEG))
    cont, _ = JOG.handle_command(st, "q")
    check("I0f q 退出", cont is False)
    JOG.handle_command(st, "home")
    check("I0g home 回零", all(abs(v) < 1e-12 for v in st.q))

    # I1/I2 方向与幅度：给 10° 就应该动约 10°，方向要对
    with Rig(initial_calibrated=True) as r:
        st = JOG.JogState(sel=4)
        runner = JOG.JogRunner(r.link, st)
        r.link.wait_tel_fast(timeout_ms=1000)

        st.sel = 4
        st.nudge(+10.0)
        ok, worst = runner.wait_settled(tol_deg=1.5, timeout_s=1.5)
        q4 = r.fake.q[4] * DEG
        check("I1 正角度 → 关节往正方向动、幅度约 10°",
              ok and abs(q4 - 10.0) <= 1.5, "q4=%.2f° err=%.2f°" % (q4, worst))

        st.set_deg(4, 0.0)
        runner.wait_settled(tol_deg=1.5, timeout_s=1.5)
        st.nudge(-10.0)
        ok, worst = runner.wait_settled(tol_deg=1.5, timeout_s=1.5)
        q4 = r.fake.q[4] * DEG
        check("I2 负角度 → 反方向、幅度约 10°",
              ok and abs(q4 + 10.0) <= 1.5, "q4=%.2f°" % q4)

        # I3 限位：给一个远超限位的值，必须被截断（Pi 侧和固件侧都夹）
        before = st.clamp_events
        msg = st.set_deg(4, 200.0)
        check("I3a 超限目标在 Pi 侧被截断，并明确报告",
              st.clamp_events == before + 1 and "超出软限位" in msg, msg)
        check("I3b 截断后的值正好是软上限",
              abs(st.q[4] - C.Q_SOFT_MAX[4]) < 1e-9,
              "%.2f° vs %.2f°" % (st.q[4] * DEG, C.Q_SOFT_MAX[4] * DEG))
        runner.wait_settled(tol_deg=2.0, timeout_s=2.0)
        check("I3c 固件侧也没让关节越过硬件限位",
              r.fake.q[4] <= C.Q_MAX[4] + 1e-6,
              "q4=%.4f max=%.4f" % (r.fake.q[4], C.Q_MAX[4]))

    # I4 12 个关节逐个都能动
    with Rig(initial_calibrated=True) as r:
        st = JOG.JogState()
        runner = JOG.JogRunner(r.link, st)
        r.link.wait_tel_fast(timeout_ms=1000)
        fails = []
        for i in range(C.N_JOINTS):
            want_deg = 20.0 if C.JOINT_KIND[i] in ("hip", "calf") else 30.0
            st.home()
            runner.wait_settled(tol_deg=2.0, timeout_s=1.0)
            ok, worst = _jog_to(runner, st, i, want_deg, dwell=1.5, tol=2.0)
            got = r.fake.q[i] * DEG
            if not (ok and abs(got - want_deg) <= 2.0):
                fails.append((i, C.JOINT_NAMES[i], round(got, 2)))
        check("I4 12 个关节逐个都能动到目标", not fails, "失败: %r" % (fails,))

    # I5 未标定时必须提前拦住（否则固件静默降级，你会以为"发了指令没反应"）
    with Rig(initial_calibrated=False, calib_ms=100) as r:
        ok, notes = JOG.precheck(r.link)
        check("I5 未标定时拒绝点动，并说清原因",
              not ok and any("STAT_CALIBRATED" in n for n in notes),
              "; ".join(notes)[:80])

    # I7 点动期间链路保持活跃（否则固件 100 ms 后开始趴下）
    with Rig(initial_calibrated=True) as r:
        st = JOG.JogState(sel=4)
        runner = JOG.JogRunner(r.link, st)
        r.link.wait_tel_fast(timeout_ms=1000)
        runner.run_for(0.6)
        tel = r.link.last_tel
        from rlcat_proto import ST_LINK_OK, ST_ESTOP
        check("I7 点动期间链路活跃、没有 ESTOP",
              tel is not None and bool(tel.status & ST_LINK_OK)
              and not bool(tel.status & ST_ESTOP),
              "status=0x%02X" % (tel.status if tel else 0))


def t_pose() -> None:
    """[J] 阶段 5：全身姿态。"""
    print("\n[J] 全身姿态（阶段 5）")
    import pose as POSE

    DEG = 180.0 / 3.141592653589793

    # J0 姿态文件解析（纯逻辑）
    ok_pose, name, note = POSE.load_pose_file(
        os.path.join(_PI, "poses", "example.json"))
    check("J0a 能读姿态文件", len(ok_pose) == C.N_JOINTS, "%s" % name)
    bad = 0
    for txt in ("1,2,3", "0," * 13):
        try:
            POSE.parse_pose_str(txt, "deg")
        except ValueError:
            bad += 1
    check("J0b 个数不对时明确报错", bad == 2, "bad=%d" % bad)
    q = POSE.parse_pose_str("0,0,0,0,10,0,0,0,0,0,0,0", "deg")
    check("J0c 度→弧度换算正确", abs(q[4] * DEG - 10.0) < 1e-9, "%.4f°" % (q[4] * DEG))

    # J1 平滑过渡：限速真的生效（耗时不短于 Δmax/rate），并且收敛
    with Rig(initial_calibrated=True) as r:
        runner = POSE.PoseRunner(r.link)
        r.link.wait_tel_fast(timeout_ms=1000)
        target = [0.0] * C.N_JOINTS
        target[4] = 30.0 / DEG          # 30°
        rate = 30.0                     # °/s → 理论 1.0 s
        t0 = time.perf_counter()
        ok, why = runner.goto(target, rate_deg_s=rate, timeout_s=4.0, tol_deg=2.0)
        dt = time.perf_counter() - t0
        check("J1a 姿态到位并收敛", ok, why)
        check("J1b 限速生效（耗时 >= Δmax/rate×0.9）", dt >= 0.9,
              "耗时 %.2f s（理论 1.00 s）" % dt)
        check("J1c 最后确实停在目标附近",
              abs(r.fake.q[4] * DEG - 30.0) <= 2.0,
              "q4=%.2f°" % (r.fake.q[4] * DEG))

    # J2 目标超限 → 夹紧并如实报告
    with Rig(initial_calibrated=True) as r:
        runner = POSE.PoseRunner(r.link)
        r.link.wait_tel_fast(timeout_ms=1000)
        tgt = [0.0] * C.N_JOINTS
        tgt[4] = 500.0 / DEG            # 远超 thigh 上限
        ok, why = runner.goto(tgt, rate_deg_s=60.0, timeout_s=3.0, tol_deg=2.0)
        check("J2a 超限目标被夹紧且报告出来", "夹紧" in why, why[:90])
        check("J2b 仍能到达被夹紧后的目标", ok and
              abs(r.fake.q[4] - C.Q_SOFT_MAX[4]) <= 2.0 / DEG,
              "q4=%.2f° 目标=%.2f°" % (r.fake.q[4] * DEG, C.Q_SOFT_MAX[4] * DEG))

    # J3 NaN 目标 → 当作 q_default，不能把关节搞乱
    with Rig(initial_calibrated=True) as r:
        runner = POSE.PoseRunner(r.link)
        r.link.wait_tel_fast(timeout_ms=1000)
        tgt = [0.0] * C.N_JOINTS
        tgt[4] = float("nan")
        ok, why = runner.goto(tgt, rate_deg_s=60.0, timeout_s=3.0, tol_deg=2.0)
        check("J3 NaN 目标不产生 NaN、不发散",
              r.fake.q[4] == r.fake.q[4] and abs(r.fake.q[4]) < 0.05,
              "q4=%.6f" % r.fake.q[4])

    # J4 过程中进入 ESTOP → 立刻中止（不能假装成功）
    with Rig(initial_calibrated=True) as r:
        runner = POSE.PoseRunner(r.link)
        r.link.wait_tel_fast(timeout_ms=1000)
        r.link.send_cmd_joint = lambda *a, **k: True    # 假装发了，其实没发
        tgt = [0.0] * C.N_JOINTS
        tgt[4] = 30.0 / DEG
        ok, why = runner.goto(tgt, rate_deg_s=10.0, timeout_s=3.0, tol_deg=2.0)
        check("J4 过程中 ESTOP → 中止并说明", (not ok) and "ESTOP" in why, why[:90])

    # J5 未标定时拒绝
    with Rig(initial_calibrated=False, calib_ms=100) as r:
        ok, notes = POSE.precheck(r.link)
        check("J5 未标定时拒绝摆姿态", not ok and
              any("STAT_CALIBRATED" in n for n in notes), "; ".join(notes)[:80])

    # J6 print_pose 的"(deg)"那一行必须真的是度（曾经把弧度打在度那一行）
    import io
    import contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        POSE.print_pose("t", [10.0 / DEG] * 4 + [0.0] * 8)
    text = buf.getvalue()
    deg_line = [l for l in text.splitlines() if "(deg)" in l][0]
    check("J6 print_pose 的度/弧度两行不混", "+10.00" in deg_line,
          deg_line.strip())


def t_gait() -> None:
    """[L] 阶段 6：脚本化步态（纯逻辑，无需硬件）。"""
    print("\n[L] 脚本化步态（阶段 6）")
    import gait as G

    DEG = 180.0 / 3.141592653589793

    # L1 trot 的腿相位：lf 与 rb 同相，lb 与 rf 同相，两组差 π
    offs = G.leg_phase_offsets("trot")
    check("L1a trot 对角同相 (lf==rb)", abs(offs[0] - offs[3]) < 1e-9,
          "lf=%.3f rb=%.3f" % (offs[0], offs[3]))
    check("L1b trot 对角同相 (lb==rf)", abs(offs[1] - offs[2]) < 1e-9,
          "lb=%.3f rf=%.3f" % (offs[1], offs[2]))
    check("L1c 两组相差 π", abs(abs(offs[0] - offs[1]) - 3.14159265) < 1e-6,
          "delta=%.4f" % abs(offs[0] - offs[1]))

    # L2 hip 恒为 0（软下限就是 0°，外摆会把机器人往外撑）
    stride, lift = 12.0 / DEG, 18.0 / DEG
    hip_max = 0.0
    for k in range(72):
        q = G.gait_target(2 * 3.14159265 * k / 72.0, offs, stride, lift)
        hip_max = max(hip_max, max(abs(q[i]) for i in range(4)))
    check("L2 hip 在所有相位下恒为 0", hip_max < 1e-12, "max|hip|=%.3e" % hip_max)

    # L3 四条 thigh 之和≈0（重心不偏）
    worst = 0.0
    for k in range(72):
        q = G.gait_target(2 * 3.14159265 * k / 72.0, offs, stride, lift)
        worst = max(worst, abs(sum(q[4:8])))
    check("L3 四条 thigh 之和≈0", worst < 1e-9, "max sum=%.3e" % worst)

    # L4 calf 必须每个周期都回到 0（否则四条腿会"全体蹲着摇"）
    #    ⚠️ 这正是 (1-cos)/2 写成时的 bug：膝盖永远不回 0。
    calf_min, calf_max = 9.9, -9.9
    for k in range(72):
        q = G.gait_target(2 * 3.14159265 * k / 72.0, offs, stride, lift)
        for i in range(8, 12):
            calf_min = min(calf_min, q[i])
            calf_max = max(calf_max, q[i])
    check("L4a calf 抬腿方向为负（屈膝）", calf_min < -lift * 0.9,
          "min=%.3f deg" % (calf_min * DEG))
    check("L4b calf 必须能回到 0（不得恒蹲）", abs(calf_max) < 1e-9,
          "max=%.3e" % calf_max)

    # L5 抬腿与摆腿的协调：lf 抬腿时 thigh 应同号前进，lb 相反
    q90 = G.gait_target(3.14159265 / 2.0, offs, stride, lift)
    check("L5a 相位90° 时 lf 抬腿", q90[8] < -lift * 0.9, "lf_calf=%.2f" % (q90[8] * DEG))
    check("L5b 相位90° 时 lb 不抬腿", abs(q90[9]) < 1e-9, "lb_calf=%.2f" % (q90[9] * DEG))
    check("L5c 对角腿反向摆", abs(q90[4] + q90[5]) < 1e-9,
          "lf=%.2f lb=%.2f" % (q90[4] * DEG, q90[5] * DEG))

    # L6 渐入系数 scale=0 时输出必须全 0（不能一上来就满幅）
    q0 = G.gait_target(1.23, offs, stride, lift, scale=0.0)
    check("L6 scale=0 ⇒ 全 0（渐入有效）",
          max(abs(v) for v in q0) < 1e-12, "max=%.3e" % max(abs(v) for v in q0))

    # L7 软限位夹取：故意给超大摆幅，结果必须仍在软限位内
    q_big = G.clamp_soft(G.gait_target(1.0, offs, 200.0 / DEG, 200.0 / DEG))
    in_range = all(C.Q_SOFT_MIN[i] - 1e-9 <= q_big[i] <= C.Q_SOFT_MAX[i] + 1e-9
                   for i in range(C.N_JOINTS))
    check("L7 超大摆幅被软限位夹住", in_range)

    # L8 限速：一步最多走 MAX_DELTA_PER_CYCLE
    prev = [0.0] * C.N_JOINTS
    want = [1.0] * C.N_JOINTS
    limited = G.rate_limit(prev, want)
    worst_d = max(abs(limited[i] - prev[i]) for i in range(C.N_JOINTS))
    check("L8 限速生效", abs(worst_d - C.MAX_DELTA_PER_CYCLE) < 1e-12,
          "max delta=%.4f" % worst_d)

    # L9 theta 从 TEL_FAST 反算：sin/cos 必须还原成同一相位（含象限）
    #    ⚠️ 必须用 math.pi，别用 3.14159265 这种截断值 ——
    #       在 225°/315° 这种跨越 ±π 的角上，截断误差会被模运算放大。
    #       （本条最初就写错了，被它自己抓到。）
    import math as _m

    class _T:                      # 极简替身，只需要两个字段
        pass

    bad = 0
    for deg in (0, 45, 90, 135, 180, 225, 270, 315):
        th = _m.radians(deg)
        t = _T()
        t.gait_sin, t.gait_cos = _m.sin(th), _m.cos(th)
        got = G.theta_from_tel(t)
        # 相位等价（同余 2π）：
        #   atan2 的值域是 (-π, π]，而 th 可能落在 [0, 2π) ⇒ 必须先折算再比。
        d = abs((got - th + _m.pi) % (2.0 * _m.pi) - _m.pi)
        if d > 1e-6:
            bad += 1
            if VERBOSE:
                print("      deg=%d got=%.6f want=%.6f d=%.3e"
                      % (deg, got, th, d))
    check("L9a theta 还原（含象限）", bad == 0, "bad=%d" % bad)
    check("L9b tel 为 None 时返回 0", G.theta_from_tel(None) == 0.0)


def t_acceptance() -> None:
    """[K] 一键验收（pi/acceptance.py），含 ⭐ 安全回归。"""
    print("\n[K] 一键验收 + 安全回归（acceptance.py）")
    import acceptance as ACC
    import calib as CAL
    from rlcat_proto import MODE_ACTIVE, MODE_IDLE, MODE_STAND

    def run_all(acc):
        acc.phase_link()
        acc.phase_telemetry()
        acc.phase_calibration()
        acc.phase_jog()
        acc.phase_pose()

    # K1 已标定 + 动作项 → 自动项应全通过
    with Rig(initial_calibrated=True) as r:
        acc = ACC.Acceptance(r.link, allow_move=True, assume_yes=True, verbose=False)
        run_all(acc)
        n = acc.summary()
        fails = [(c.cid, c.title, c.detail) for c in acc.checks if c.status == ACC.FAIL]
        check("K1 已标定 + 动作项 → 自动项全部通过", n[ACC.FAIL] == 0,
              "失败 %r" % (fails[:3],))
        check("K1b 动作项确实执行了（没有跳过）", n[ACC.SKIP] == 0,
              "skip=%d，通过=%d" % (n[ACC.SKIP], n[ACC.PASS]))
        check("K1c 12 个关节各有一条检查项",
              sum(1 for c in acc.checks if c.cid.startswith("P4.")) >= 14,
              "P4 项数=%d" % sum(1 for c in acc.checks if c.cid.startswith("P4.")))

    # K2 ⭐⭐ 安全回归：未标定时【绝不能】发出 STAND / ACTIVE
    #    固件的 STAND 没有准入门；未标定（默认表 dir=+1/offset=0）时
    #    q_target=0 → 舵机 step 0 → 12 个关节全部冲到机械行程一端。
    with Rig(initial_calibrated=False, calib_ms=99999) as r:
        acc = ACC.Acceptance(r.link, allow_move=True, assume_yes=True, verbose=False)
        run_all(acc)
        seen = sorted(r.fake.modes_seen)
        check("K2a 未标定 → 标定检查项报失败",
              any(c.status == ACC.FAIL for c in acc.checks),
              "失败项 %r" % [c.cid for c in acc.checks if c.status == ACC.FAIL])
        check("K2b ⭐ 未标定时【从未】发出 STAND", MODE_STAND not in r.fake.modes_seen,
              "假固件见过的模式 = %r" % seen)
        check("K2c ⭐ 未标定时【从未】发出 ACTIVE", MODE_ACTIVE not in r.fake.modes_seen,
              "假固件见过的模式 = %r" % seen)
        check("K2d 动作项被【跳过】而不是硬跑",
              all(any(c.cid == p and c.status == ACC.SKIP for c in acc.checks)
                  for p in ("P4.0", "P5.0")),
              "P4/P5 = %r" % [(c.cid, c.status) for c in acc.checks
                              if c.cid in ("P4.0", "P5.0")])

    # K3 未标定时 CalibSession.postcheck 必须拒绝（它是唯一会主动发 STAND 的地方）
    with Rig(initial_calibrated=False, calib_ms=99999) as r:
        sess = CAL.CalibSession(r.link, quiet=True)
        ok, notes = sess.postcheck()
        check("K3a 未标定时 postcheck 拒绝执行",
              (not ok) and any("拒绝发 STAND" in n for n in notes),
              "; ".join(notes)[:70])
        check("K3b 拒绝时确实一个 STAND 都没发",
              MODE_STAND not in r.fake.modes_seen,
              "假固件见过的模式 = %r" % sorted(r.fake.modes_seen))

    # K4 报告结构
    d = acc.checks[0].as_dict()
    check("K4 报告条目字段完整",
          set(d.keys()) == {"id", "title", "status", "detail"}, "%r" % d)


# ============================================================================

def main() -> int:
    global VERBOSE
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    VERBOSE = args.verbose

    print("=" * 78)
    print(" RL-CAT 树莓派侧基础层：离线端到端测试（PTY + 假 STM32，无需硬件）")
    print("=" * 78)

    for fn in (t_link_and_telemetry, t_modes_and_motion, t_calibration,
               t_fault_injection, t_reply_latency, t_boot_behavior,
               t_telemetry_semantics, t_calibration_session,
               t_jog, t_pose, t_gait, t_acceptance):
        try:
            fn()
        except Exception as e:
            import traceback
            check("%s 抛异常" % fn.__name__, False, repr(e))
            traceback.print_exc()

    n_pass = sum(1 for _, ok, _ in RESULTS if ok)
    n_fail = len(RESULTS) - n_pass
    print("\n" + "=" * 78)
    print(" PASS %d   FAIL %d" % (n_pass, n_fail))
    if n_fail:
        print("\n 失败项：")
        for name, ok, detail in RESULTS:
            if not ok:
                print("   · %s   [%s]" % (name, detail))
    print("=" * 78)
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
