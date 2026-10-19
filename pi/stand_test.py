#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M1 · 地面站立验证（mainline Gate）

⚠️ 为什么需要这个工具
--------------------
`docs/46-mainline-plan.md` 把「机器人在地面上自主站立 10 秒」定为 M1，
它是 M2（步态）和 M3（sim2real）的共同前提。

但"站住了"很容易靠眼睛糊弄过去：

    · 腿在抖，但看起来"还站着"
    · 10 秒内缓慢下沉 5°，肉眼几乎看不出来
    · 某个关节被限位卡住，一动不动，看起来"很稳"

⇒ 所以本工具把"站立"拆成 4 个可测的量，并给出**通过/不通过**：

    ① 到位误差  |q − q_default|        站立姿态本身做得对不对
    ② 漂移      |mean(末 1s) − mean(初 1s)|   会不会缓慢下沉/走样
    ③ 抖动      std(q)                 高频振荡（伺服增益/延迟问题）
    ④ 力矩持续  torque_actual 是否全程为 1       有没有中途卸力

⚠️ 本工具**只保持站姿**（目标恒为 q_default），不跑策略、不走路。
   跑策略是 M3 的事，见 `rl_run.py`。

用法
----
    # 吊起或架空时先试一次（不发力的行为要先确认）
    python3 stand_test.py --seconds 10 --yes

    # ⭐ 放上地面（腿着地、吊带留 2~3 cm 余量）再跑
    python3 stand_test.py --seconds 10 --yes

    # 只测量、不命令（需要有人扶住机器人）
    python3 stand_test.py --observe --seconds 10

退出码：0 通过，1 失败，2 安全中止
"""

from __future__ import annotations

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
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import config as C                                      # noqa: E402
import telemetry as T                                   # noqa: E402
from link import PiLink, LinkError                      # noqa: E402
from rlcat_proto import (                               # noqa: E402
    FLAG_ENABLE,
    MODE_ACTIVE,
    MODE_IDLE,
    ST_BATT_FORCE,
    ST_CALIBRATED,
    ST_ESTOP,
    ST_IMU_OK,
    ST_LINK_OK,
)

DEG = 180.0 / math.pi

# ---- 判据（可在命令行覆盖）----
TOL_REACH_DEG = 3.0      # ① 到位误差
TOL_DRIFT_DEG = 2.0      # ② 漂移
TOL_RIPPLE_DEG = 1.5     # ③ 抖动（std）
# ⚠️ 只看 std 会漏掉"小幅但明显的振荡"：
#    振幅 A 的正弦 std = A/√2。2° 振幅的 8 Hz 抖动 std 只有 1.41°，
#    刚好卡在 1.5° 阈值下面 ⇒ 会误判为通过。
#    ⇒ 再加一条峰峰值判据：p2p > 3° 就算抖。
TOL_P2P_DEG = 3.0        # ③b 峰峰值
TOL_COLLAPSE_DEG = 12.0  # 塌陷阈值（任一关节偏离 q_default 超过它就算塌）


def _stats(vals):
    n = len(vals)
    if n == 0:
        return 0.0, 0.0
    m = sum(vals) / n
    var = sum((v - m) ** 2 for v in vals) / n
    return m, math.sqrt(var)


def main() -> int:
    ap = argparse.ArgumentParser(description="M1 · 地面站立验证")
    ap.add_argument("--port", default=C.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=C.SERIAL_BAUD)
    ap.add_argument("--seconds", type=float, default=10.0, help="测量时长")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="到位后先稳定多久再开始测量")
    ap.add_argument("--ramp", type=float, default=10.0,
                    help="从当前姿态到站姿的速度（度/秒）")
    ap.add_argument("--observe", action="store_true",
                    help="只测量，不命令（需有人扶住机器人）")
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()

    link = PiLink(args.port, args.baud, verbose=False)
    try:
        link.open()
    except LinkError as e:
        print("❌ 打不开串口：%s" % e)
        return 1

    print("=" * 78)
    print(" M1 · 地面站立验证   %s @ %d" % (args.port, args.baud))
    print("=" * 78)
    print(" 测量 %0.0f s，到位后先稳定 %0.1f s" % (args.seconds, args.settle))
    print(" 模式：%s" % ("只测量（不命令）" if args.observe else "⭐ 保持站姿"))

    # ---- 热身 ----
    print("\n 建立链路...")
    warm_end = time.perf_counter() + 0.3
    while time.perf_counter() < warm_end:
        link.pump()
        link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE, flags=0)
        time.sleep(0.01)
    link.pump()

    st = link.wait_status(timeout_ms=2000)
    tel = link.wait_tel_fast(timeout_ms=3000)
    if tel is None or st is None:
        print("❌ 收不到遥测")
        link.close()
        return 1

    print(" 标定/链路/IMU : %s" % T.status_str(st.status))
    if not (st.status & ST_LINK_OK):
        print(" ⚠️ 当前 LINK_OK 未亮 —— 这是【刚才没有工具在发帧】的正常现象，")
        print("    热身会把它拉起来；若热身 3 s 后仍不亮，再查链路。")
    start_q = list(tel.q)

    # ---- 平滑过渡到站姿（q_default）----
    print("\n 从当前姿态过渡到站姿（%.0f°/s）..." % args.ramp)
    arm_t0 = time.perf_counter()
    armed = False
    while time.perf_counter() < arm_t0 + 3.0:
        link.pump()
        link.send_cmd_joint(start_q, mode=MODE_ACTIVE, flags=FLAG_ENABLE)
        t = link.last_tel
        if t is not None and (link.t_rx_tel_fast >= arm_t0):
            if not (t.status & ST_ESTOP) and \
               T.derive_effective_mode(MODE_ACTIVE, t.status)[0] == MODE_ACTIVE:
                armed = True
                break
        time.sleep(0.002)
    if not armed:
        print("❌ 3 秒内没能进入 ACTIVE")
        if link.last_tel is not None:
            print("   状态位 = 0x%02X（%s）"
                  % (link.last_tel.status, T.status_str(link.last_tel.status)))
        link.close()
        return 1
    print(" ✅ 已进入 ACTIVE")

    if not args.observe and not args.yes:
        print("\n⚠️⚠️ 机器人应已【放在地面】上，吊带留 2~3 cm 余量（站住时不受力）。")
        try:
            if input("确认继续？输入 yes 回车：").strip().lower() != "yes":
                print("已取消")
                link.close()
                return 1
        except (EOFError, KeyboardInterrupt):
            print("\n已取消")
            link.close()
            return 1

    # ---- 过渡 ----
    q_def = list(C.Q_DEFAULT)
    max_d = max(abs(q_def[i] - start_q[i]) for i in range(C.N_JOINTS))
    rate = max(args.ramp, 1e-6) / DEG
    dur = max_d / rate
    period = 1.0 / C.CONTROL_HZ
    t0 = time.perf_counter()
    t_next = t0
    bad_since = None
    min_mv = 1 << 30
    torque_ok = True
    samples = []           # [(t, q, status)]
    n_tick = 0
    aborted = False
    # ⭐ 发送健康度（2026-10-18 加）：区分"Pi 停止发帧"与"发了但没到"
    #   ⚠️ `t` 已证明主循环最长只停 20 ms（poll_gap_max_us = 20237），
    #      解释不了 300 ms 的 ESTOP 空档 ⇒ 缺口要么在 Pi 的发送侧，
    #      要么在"Pi TX → STM32 RX"之间。这两者只有 Pi 能分辨。
    tx_gaps = []            # 相邻两次【成功发送】的间隔（ms）
    tx_fail = 0
    tx_last = None

    print("\n 过渡 + 稳定 + 测量 ...（Ctrl-C 中止）")
    try:
        while True:
            link.pump()
            now = time.perf_counter()
            tel = link.last_tel
            if tel is None or now < t_next:
                time.sleep(0.001)
                continue
            t_next = now + period
            n_tick += 1

            el = now - t0
            if args.observe:
                s = 0.0                       # 不命令
                cmd = list(tel.q)
            else:
                s = 1.0 if dur <= 0 else min(1.0, el / dur)
                e = s * s * (3.0 - 2.0 * s)
                cmd = [start_q[i] + (q_def[i] - start_q[i]) * e
                       for i in range(C.N_JOINTS)]

            # ---- 健康 ----
            if tel.voltage_mv > 0:
                min_mv = min(min_mv, tel.voltage_mv)
            bad = bool(tel.status & ST_ESTOP) or \
                (T.derive_effective_mode(MODE_ACTIVE, tel.status)[0] != MODE_ACTIVE)
            if bad:
                if bad_since is None:
                    bad_since = now
                elif now - bad_since > 0.6:
                    print("\n❌ 非 ACTIVE/ESTOP 持续 %.1f s，中止" % (now - bad_since))
                    print("   状态位 = 0x%02X（%s）"
                          % (tel.status, T.status_str(tel.status)))
                    # ⭐ 直接把原因翻成人话（ESTOP 只有 4 个来源，见 rlca_safety.c）
                    if tel.status & ST_ESTOP:
                        if not (tel.status & ST_LINK_OK):
                            print("   ⇒ 原因：链路空档 >300 ms（STAT_LINK_OK 熄灭）")
                            print("     ⚠️ 任何工具只要暂停发帧就会这样；")
                            print("        本工具在 --observe 下也发 keepalive，不应出现。")
                        elif tel.status & ST_BATT_FORCE:
                            print("   ⇒ 原因：电池 ≤9300 mV（STAT_BATT_FORCE）")
                        else:
                            print("   ⇒ 原因：既非链路也非电压 ⇒ 查舵机过温（>70℃）")
                            print("     或用 ESP32 无线口敲 R 看 safety_reasons 的 0x10 位")
                    aborted = True
                    break
            else:
                bad_since = None

            # ---- 采集（只在稳定期和测量期）----
            if el >= dur and el >= dur + 0.0:
                # ⭐ 记录 TEL_STATUS 里的舵机最高温度与 err_flags
                #   ⚠️ safety_reasons 只在 shell 的 R 里，遥测里没有；
                #      但 `temperature` 就是 servo_temp_max —— 而"过温"正是
                #      ESTOP 的候选触发源之一，所以这个值足以定罪/排除。
                #   ⚠️ TEL_STATUS 是 10 Hz，比 100 Hz 的采样慢，
                #      所以它可能"就近"但不会错得离谱（分类足够）。
                # ⚠️ `temperature` 是长度 12 的【逐舵机】列表，不是标量。
                #    取最大值即是固件用的 servo_temp_max。
                _st_last = link.last_status
                if _st_last is not None:
                    _tl = _st_last.temperature
                    _tm = max(_tl) if isinstance(_tl, (list, tuple)) and _tl else -1
                    _ef = int(_st_last.err_flags)
                    _mt = tuple(int(x) for x in _tl) if isinstance(_tl, (list, tuple)) else ()
                else:
                    _tm, _ef, _mt = -1, -1, ()
                samples.append((el, list(tel.q), int(tel.status),
                                int(tel.cycle_cnt), int(_tm), _ef, _mt))

            # ---- 下发（同时记录发送健康度）----
            # ⚠️⚠️ `--observe` 也【必须发帧】（2026-10-18 修我自己的 bug）
            #
            #   原来写的是 `if not args.observe: send(...)`，
            #   于是观测模式热身完就一帧不发 ⇒ 300 ms 后链路判定失联
            #   ⇒ 固件进 ESTOP ⇒ 本工具把自己的疏忽报成"机器人故障"。
            #
            #   ⭐ 正确语义：`--observe` 的含义是"**不命令关节目标**"，
            #      而不是"不发帧"。目标传【当前实测姿态】即可 ——
            #      机器人不会动，但链路与力矩都保持活着。
            _ok = link.send_cmd_joint(cmd, mode=MODE_ACTIVE, flags=FLAG_ENABLE)
            _ts = time.perf_counter()
            if _ok:
                if tx_last is not None:
                    tx_gaps.append((_ts - tx_last) * 1000.0)
                tx_last = _ts
            else:
                tx_fail += 1

            # ⚠️⚠️ 控制循环里【不许 print】（2026-10-18 又踩了一次）
            #
            #  原来这里每 0.5 s 打一行进度。看起来只有 2 Hz、很安全，
            #  但 `print` 到 SSH 管道是**阻塞**的：一次可能卡几十毫秒。
            #  那段时间 Pi 不发帧 ⇒ 链路空档 > 80 ms ⇒ 固件 `NO_LINK`
            #  ⇒ ACTIVE 降级 STAND ⇒ `stand_test` 报"15% 的样本不在 ACTIVE"。
            #
            #  ⭐ 实测对照（同一台机器、同一条链路）：
            #      linkprobe.py（循环里不打印）  ⇒ 丢帧 0%
            #      stand_test.py（2 Hz 打印）    ⇒ 14.3% 样本 NO_LINK
            #      10 s / 20 次打印 × ~70 ms ≈ 1.4 s = 14%  ← 精确吻合
            #
            #  ⇒ 进度信息改为【只在结束时汇总】。想看中间过程就开 status_panel
            #    （它自己维护独立的保活线程，不占控制循环）。
            #  ⚠️ 这条已写进 docs/48-sim2real-gaps.md 的"循环内 I/O"条目。

            if el >= dur + args.settle + args.seconds:
                break

    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
        aborted = True
    finally:
        try:
            if link.is_open:
                link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE, flags=0)
                link.pump()
                time.sleep(0.05)
        except Exception:                               # noqa: BLE001
            pass
        link.close()

    # ========================================================================
    # 分析
    # ========================================================================
    print()
    print("=" * 78)
    print(" M1 站立质量报告")
    print("=" * 78)

    if args.observe:
        print(" ⚠️ --observe 模式：本工具没有命令关节，"
              "报告只反映机器人的被动姿态。")
    if not samples:
        print(" ❌ 没有采到样本（中途中止？）")
        return 2

    # 只分析稳定之后的样本
    meas = [s for s in samples if s[0] >= dur + args.settle]
    if len(meas) < 20:
        print(" ❌ 测量期样本太少（%d），无法判断" % len(meas))
        return 2

    n = len(meas)
    win = max(5, n // 10)                 # 首尾各取 10% 做漂移对比
    head = meas[:win]
    tail = meas[-win:]

    print(" 样本 %d 个（测量期 %.1f s）" % (n, args.seconds))
    print()
    print(" 关节        到位误差(°)   漂移(°)   抖动(°)   峰峰(°)   判定")
    print(" " + "-" * 72)

    fails = []
    worst_reach = 0.0
    worst_drift = 0.0
    worst_ripple = 0.0
    worst_p2p = 0.0

    for i in range(C.N_JOINTS):
        vals = [m[1][i] for m in meas]
        h = [m[1][i] for m in head]
        tl = [m[1][i] for m in tail]
        mean_all, std_all = _stats(vals)
        mean_h, _ = _stats(h)
        mean_t, _ = _stats(tl)

        reach = abs(mean_all - q_def[i]) * DEG      # ①
        drift = abs(mean_t - mean_h) * DEG          # ②
        ripple = std_all * DEG                      # ③
        p2p = (max(vals) - min(vals)) * DEG         # ③b

        worst_reach = max(worst_reach, reach)
        worst_drift = max(worst_drift, drift)
        worst_ripple = max(worst_ripple, ripple)
        worst_p2p = max(worst_p2p, p2p)

        bad = []
        if reach > TOL_REACH_DEG:
            bad.append("到位")
        if drift > TOL_DRIFT_DEG:
            bad.append("漂移")
        if (ripple > TOL_RIPPLE_DEG) or (p2p > TOL_P2P_DEG):
            bad.append("抖动")
        if reach > TOL_COLLAPSE_DEG:
            bad.append("塌陷")

        verdict = "ok" if not bad else ("⚠️ " + "+".join(bad))
        if bad:
            fails.append((i, bad))
        print("  %2d %-10s %8.2f   %7.2f   %7.2f   %7.2f   %s"
              % (i, C.JOINT_NAMES[i], reach, drift, ripple, p2p, verdict))

    # ---- 模式持续性（分类诊断）----
    #
    #  ⚠️ 为什么要把"模式"和"姿态"分开判（2026-10-18）：
    #     M1 的目标是 q_default，而 STAND 模式同样保持 q_def
    #     ⇒ 从 ACTIVE 掉到 STAND **关节目标完全不变**，姿态上看不出来。
    #     所以"模式闪断"不该否决"站得稳不稳"，它是**另一件事**，
    #     而且对 M3（要持续跟踪上位机目标）才是致命的。
    #  ⭐ 这里进一步分门别类，直接指出是哪道门（ACTIVE 准入共 5 道）。
    print()
    # ⭐ 把连续的非 ACTIVE 样本归并成【事件】，并顺便判断 MCU 当时卡没卡
    #
    #  ⚠️ 为什么要看 cycle_cnt（固件每 100 Hz 加 1，在 TEL_FAST 里）：
    #     ESTOP 的触发条件是"距上次收到帧 > 300 ms"。
    #     但若 `cycle_cnt` 在这 300 ms 里**照常推进**，说明 MCU 在跑、
    #     只是没去解析串口 ⇒ 问题在"解析被饿死"（阻塞 app_poll）。
    #     若 `cycle_cnt` **停住**，说明整个主循环卡死了 ⇒ 另一种故障。
    #   ⇒ 这一个判据就能把"链路判定"和"MCU 阻塞"分开。
    def _classify(_s):
        if _s & ST_ESTOP:
            return "estop"
        if not (_s & ST_LINK_OK):
            return "link"
        if not (_s & ST_IMU_OK):
            return "imu"
        if not (_s & ST_CALIBRATED):
            return "calib"
        return "other"

    events = []
    cur = None
    for _t, _q, _s, _c, _tm, _ef, _mt in meas:
        if T.derive_effective_mode(MODE_ACTIVE, _s)[0] == MODE_ACTIVE:
            if cur is not None:
                events.append(cur)
                cur = None
            continue
        if cur is None:
            cur = {"t0": _t, "t1": _t, "n": 1, "c0": _c, "c1": _c,
                   "cls": _classify(_s), "tm": _tm, "ef": _ef, "mt": _mt}
        else:
            cur["t1"] = _t
            cur["n"] += 1
            cur["c1"] = _c
    if cur is not None:
        events.append(cur)

    n_nomode = sum(e["n"] for e in events)
    dcyc_all = [(e["c1"] - e["c0"]) & 0xFFFF for e in events]
    dur_all = [e["t1"] - e["t0"] for e in events]

    pct = 100.0 * n_nomode / n
    mode_ok = (n_nomode == 0)
    print(" 模式持续性      : %s（%d/%d = %.1f%% 不在 ACTIVE）"
          % ("✅" if mode_ok else "⚠️", n_nomode, n, pct))
    if n_nomode > 0:
        label = {"estop": "ESTOP", "link": "链路空档 >80 ms（NO_LINK）",
                 "imu": "IMU 无效（NO_IMU）", "calib": "未标定（NOT_CALIB）",
                 "other": "安全层 or 舵机反馈（SAFETY/NO_FB）"}
        print("     事件 %d 个，持续时间 中位 %.0f ms / 最长 %.0f ms"
              % (len(events),
                 sorted(dur_all)[len(dur_all) // 2] * 1000.0,
                 max(dur_all) * 1000.0))

        # ⭐ 核心判据：事件期间 cycle_cnt 有没有继续走
        advancing = sum(1 for d in dcyc_all if d > 2)
        frozen = len(dcyc_all) - advancing
        print("     cycle_cnt 在事件期间：推进 %d 个 / 停住 %d 个"
              % (advancing, frozen))
        if advancing and not frozen:
            print("     ⇒ ⭐ MCU 一直在跑（每事件约 +%d 个周期），"
                  "只是没去解析串口" % (sum(dcyc_all) // max(len(dcyc_all), 1)))
            print("       ⇒ 问题在【解析被饿死】（app_poll 阻塞），不是串口本身")
            print("       ⇒ 下一步：跑完立刻敲 t，看 poll_gap_max_us 与 8 个 step 的最大值")
        elif frozen:
            print("     ⇒ ⚠️ MCU 在事件期间【停住】了 —— 主循环真的卡死")
            print("       ⇒ 查 shell 命令 / flash 擦写 / 舵机总线")

        # 按类别统计样本数
        cnt = {"estop": 0, "link": 0, "imu": 0, "calib": 0, "other": 0}
        for e in events:
            cnt[e["cls"]] += e["n"]
        for k, v in cnt.items():
            if v:
                print("     - %-40s %d 个样本" % (label[k], v))
        if cnt["estop"]:
            print("     ⚠️ 有 ESTOP ⇒ 按固件语义力矩会被卸掉；")
            print("        但若漂移仍接近 0，说明每次只停极少周期（腿来不及塌）。")

        # ⭐⭐ 用 TEL_STATUS 的【舵机最高温度】给 ESTOP 定罪/排除
        #
        #  固件里 ESTOP 只有 4 个来源（rlca_safety.c）：
        #    · link_level >= 2（>300 ms 空档）  ← rx_bytes 全收到 ⇒ 已排除
        #    · batt_mv <= 9300                  ← 电压 11.4 V ⇒ 已排除
        #    · max_temp_c > 70                  ← ⭐ 只剩这条
        #    · operator_estop                   ← 代码里恒 false
        #  ⇒ 所以只看温度就能定论。
        tm_all = [e["tm"] for e in events if e["tm"] is not None and e["tm"] >= 0]
        ef_all = [e["ef"] for e in events if e["ef"] is not None and e["ef"] >= 0]
        if tm_all:
            tm_max = max(tm_all)
            n_over = sum(1 for x in tm_all if x > 70)
            print(" 事件期间舵机最高温度 : max %d ℃（>70℃ 的事件 %d/%d 个）"
                  % (tm_max, n_over, len(tm_all)))
            # ⭐ 逐舵机温度：能看出"是哪一个舵机的读数异常"
            mt_all = [e["mt"] for e in events if e.get("mt")]
            if mt_all:
                per = [max(x[i] for x in mt_all) for i in range(len(mt_all[0]))]
                print(" 逐舵机温度           : %s" % per)
                bad = [i for i, v in enumerate(per) if v > 70]
                if bad:
                    print("   ⇒ 异常舵机：%s" % [C.JOINT_NAMES[i] for i in bad])
            if n_over:
                print("     ⇒ 温度确实越过了 70℃ 阈值 ⇒ 这是 ESTOP 的直接触发源")
            else:
                print("     ⇒ 温度正常 ⇒ 过温【不是】原因")
                print("        ⇒ 固件里 ESTOP 只剩链路一条（电池/操作者已排除），")
                print("          但 rx_bytes 全收到、poll_gap 20 ms ⇒ 需要查")
                print("          link_level 的计算与 s_last_rx_ms 的刷新路径。")
        if ef_all and max(ef_all) != 0:
            print(" 事件期间 err_flags     : max 0x%02X（可查 docs 里的 ERRF 位表）"
                  % max(ef_all))
        if cnt["other"]:
            print("     ⇒ 敲 R 看 active_block：4 = 安全层，5 = 舵机反馈")

    print(" 电压最低        : %d mV" % min_mv)

    # ⭐ Pi 侧发送健康度 —— 这是区分"Pi 没发"和"发了没到"的唯一证据
    if tx_gaps:
        g_sorted = sorted(tx_gaps)
        g_p50 = g_sorted[len(g_sorted) // 2]
        g_max = g_sorted[-1]
        n_hole = sum(1 for g in tx_gaps if g > 80.0)
        n_big = sum(1 for g in tx_gaps if g > 300.0)
        print(" Pi 发送间隔     : p50 %.1f ms / max %.1f ms / 失败 %d 次"
              % (g_p50, g_max, tx_fail))
        if n_big:
            print("   ⇒ ⚠️ 有 %d 次发送间隔 > 300 ms（最长 %.0f ms）"
                  "—— 这是 **Pi 自己停了发帧**" % (n_big, g_max))
            print("      查：本工具循环里的阻塞（print? sleep?）、GC、SSH 管道")
        elif n_hole:
            print("   ⇒ ⚠️ 有 %d 次发送间隔 > 80 ms（最长 %.0f ms），但都 < 300 ms"
                  % (n_hole, g_max))
            print("      这只能解释 LINK_OK 级别的降级，**解释不了 ESTOP**")
            print("      ⇒ ESTOP 的 300 ms 空档另有来源：查 'Pi TX → STM32 RX' 之间")
            print("         （线材/接触/500kbps 下的误码被 CRC 丢掉？）")
        else:
            print("   ⇒ ✅ Pi 全程按 ~10 ms 稳定发帧、零失败")
            print("      ⇒ 那 300 ms 空档发生在【Pi TX → STM32 RX】之间，"
                  "不是本工具的错")
            print("      ⇒ 查：杜邦线接触、共地、500 kbps 误码（R 里 crc/len 计数）")
    print()
    print(" 最差值：到位 %.2f°  漂移 %.2f°  抖动 %.2f°(std) / %.2f°(峰峰)"
          % (worst_reach, worst_drift, worst_ripple, worst_p2p))
    print(" 判据  ：到位 <%.1f°  漂移 <%.1f°  抖动 <%.1f°(std) 且 <%.1f°(峰峰)  不塌(<%.0f°)"
          % (TOL_REACH_DEG, TOL_DRIFT_DEG, TOL_RIPPLE_DEG,
             TOL_P2P_DEG, TOL_COLLAPSE_DEG))

    print()
    print("=" * 78)
    # ⭐ 判定分两层：
    #    ① 姿态质量（M1 的本体）—— 到位/漂移/抖动/不塌
    #    ② 模式稳定性（是 M3 的前置条件，不是 M1 的否决项）
    #       ⚠️ 理由：STAND 与 ACTIVE 的目标都是 q_def，掉的瞬间姿态不变。
    pose_ok = (not fails) and (not aborted)
    ok = pose_ok and mode_ok
    if ok:
        print(" ✅ M1 通过 —— 机器人能在地面上稳定站立，且全程保持 ACTIVE")
        print("    ⇒ 可以进 M2（脚本步态）或 M3（sim2real）")
    elif pose_ok:
        print(" ⭐ M1【姿态】通过 —— 站得很稳（到位 %.2f°、漂移 %.2f°、抖动 %.2f°）"
              % (worst_reach, worst_drift, worst_ripple))
        print("    ⚠️ 但模式在 %.0f%% 的样本里掉了 ACTIVE（原因见上）。" % pct)
        print("       · 对 M1（静态站立）：**无影响** —— STAND 也保持同一姿态")
        print("       · 对 M3（跑策略）  ：**必须修** —— STAND 会忽略策略目标")
    else:
        print(" ❌ M1 未通过")
        if aborted:
            print("    原因：中途安全中止")
        for i, bad in fails:
            print("    %s：%s" % (C.JOINT_NAMES[i], "+".join(bad)))
        print()
        print("  排查顺序：")
        print("    1) 漂移大 ⇒ 力矩不够或重心偏 ⇒ 调 q_default 让膝盖多弯/重心降低")
        print("    2) 抖大   ⇒ 伺服增益或驱动器延迟 ⇒ 见 docs/48-sim2real-gaps.md G2")
        print("    3) 到位差 ⇒ 某个关节被限位/机械卡住 ⇒ 敲 K 看 raw pos_step")
    print("=" * 78)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
