#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 阶段 6：脚本化步态（硬件联调用的开环步态）

⚠️⚠️ 本工具是【硬件联调】用的开环步态，不是 RL 策略。
    它的目的是：在一个已知、可预测、可复现的轨迹下确认
      · 12 个关节都朝正确方向动
      · 步态相位与 STM32 产生的相位一致
      · 幅度/频率能调，用来找"不摔"的参数区间
    它【不】做平衡、不看姿态、不闭环。

⭐ 相位从哪来（这条是硬约束，别改）：
    相位由 **STM32** 产生（`rlca_phase`，每控制周期推进 0.03 圈 = 3 Hz），
    经 TEL_FAST 的 `gait_sin` / `gait_cos` 下发。
    本工具【只读】这两个值当主时钟，**绝不在 Pi 上自己算相位**。
    理由：两边各算一份必然漂移（docs/31-pi-runtime.md §6.2 第 2 条）。

步态形状（关节空间，不做逆运动学）：
    每条腿一个相位 φ = θ + leg_offset（θ 来自 STM32 的 sin/cos）
        thigh = +A_thigh * sin(φ)                    ← 前后摆
        calf  = -A_calf  * (1 - cos(φ)) / 2          ← 摆动相抬腿（负 = 屈膝）
        hip   = 0                                    ← 不外摆（hip 软下限是 0°）
    trot（对角小跑）：lf 与 rb 同相，lb 与 rf 反相。

⚠️ 符号约定（第一次上机必须确认，见 --sign-* 参数）：
    · calf 的负方向 = 屈膝（把脚抬起来）。依据：标定姿态 q_calib ≈ +1.571
      是"腿伸直"，而站立 q=0 是弯腿 ⇒ 从 0 往负是继续屈。
    · thigh 的正方向 = 向前摆还是向后摆，取决于 URDF 轴向；
      对称振荡下不影响"能不能动"，但影响四腿协调。用 --sign-thigh 翻转。

用法::

    # 不放硬件，先看轨迹长什么样（全程不发帧）
    python3 gait.py --preview

    # 真机（⚠️ 必须吊起）
    python3 gait.py --stride-deg 12 --lift-deg 18 --seconds 10 --yes

退出码：0 正常，1 失败/被拒，2 安全中止（ESTOP 等）
"""

from __future__ import annotations

# ⚠️ Windows 控制台默认 GBK，中文输出会乱码。在任何输出之前切成 UTF-8。
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
    MODE_ESTOP,
    MODE_IDLE,
    ST_CALIBRATED,
    ST_ESTOP,
    ST_IMU_OK,
    ST_LINK_OK,
)

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# ============================================================================
# 步态定义（纯逻辑，可离线单测）
# ============================================================================

# 逻辑索引 A：0-3 hip(lf,lb,rf,rb)  4-7 thigh  8-11 calf
# ⚠️ 12 维向量一律用这个顺序，不要按腿分组。
LEG_NAMES = ("lf", "lb", "rf", "rb")


def leg_phase_offsets(gait: str = "trot") -> tuple:
    """每条腿的相位偏移（弧度）。顺序 = (lf, lb, rf, rb)。

    trot（对角小跑）：lf+rb 一组、lb+rf 一组，两组相差 π。
    walk（四拍）：lf→rb→rf→lb 依次差 π/2。
    pace（同侧）：lf+lb 一组、rf+rb 一组 —— 同侧同相，小四足上通常很不稳，
                   列在这里只是为了让你能复现"错误步态"作对比。
    """
    pi = math.pi
    if gait == "trot":
        return (0.0, pi, pi, 0.0)
    if gait == "walk":
        return (0.0, -pi / 2.0, pi, pi / 2.0)
    if gait == "pace":
        return (0.0, 0.0, pi, pi)
    raise ValueError("unknown gait: %s" % gait)


def gait_target(theta: float,
                offsets: tuple,
                stride_rad: float,
                lift_rad: float,
                scale: float = 1.0,
                sign_thigh: float = 1.0,
                sign_calf: float = 1.0) -> list:
    """按当前相位算出 12 维目标角（逻辑索引 A）。

    @param theta       全局相位（弧度），来自 STM32 的 gait_sin/cos
    @param offsets     leg_phase_offsets() 的返回值
    @param stride_rad  thigh 摆幅（rad）
    @param lift_rad    calf 抬腿幅度（rad，取正值表示"抬"，内部会取负方向）
    @param scale       0..1 的渐入系数（启动时从 0 平滑到 1）
    """
    q = [0.0] * C.N_JOINTS
    for leg in range(4):
        phi = theta + offsets[leg]
        thigh = sign_thigh * stride_rad * math.sin(phi)
        # ⚠️ 抬腿必须用【半波 max(0, sin)】而不是 (1-cos)/2：
        #    后者在整个周期里都不回到 0（膝盖永远蹲着），
        #    四条腿各差半周期就变成"全体蹲着摇"，不是步态。
        #    半波则一段抬、一段放，两组对角腿正好互补。
        lift_wave = max(0.0, math.sin(phi))
        calf = sign_calf * (-lift_rad) * lift_wave
        q[4 + leg] = thigh * scale
        q[8 + leg] = calf * scale
        # hip 保持 0：软下限就是 0°，任何外摆都会把机器人往外撑
        q[leg] = 0.0
    return q


def clamp_soft(q: list) -> list:
    """夹到软限位（config.Q_SOFT_MIN/MAX）。

    ⚠️ 固件侧还有一道 `rlca_q_clamp`，这里夹是为了让【上报的意图】与
       实际下发一致，避免你以为发了 60° 其实被固件削到 55°。
    """
    return [min(max(q[i], C.Q_SOFT_MIN[i]), C.Q_SOFT_MAX[i])
            for i in range(C.N_JOINTS)]


def rate_limit(prev: list, want: list, max_delta: float = None) -> list:
    """按每周期最大变化量限速（与固件 RLCA_MAX_DELTA_PER_CYCLE_RAD 对齐）。

    ⚠️ 固件也会限速，但那是在"目标之间"限；这里先限一次可以把
       "第一条指令与当前位置的落差"也在 Pi 侧摊平，避免首帧猛冲。
    """
    lim = C.MAX_DELTA_PER_CYCLE if max_delta is None else max_delta
    out = [0.0] * C.N_JOINTS
    for i in range(C.N_JOINTS):
        d = want[i] - prev[i]
        if d > lim:
            d = lim
        elif d < -lim:
            d = -lim
        out[i] = prev[i] + d
    return out


def theta_from_tel(tel) -> float:
    """从 TEL_FAST 的 gait_sin/gait_cos 取出全局相位（弧度）。

    ⚠️ 用 atan2 而不是 asin：asin 无法区分象限，会让步态在一半的周期里反相。
    """
    if tel is None:
        return 0.0
    return math.atan2(tel.gait_sin, tel.gait_cos)


# ============================================================================
# 本地预览（不碰硬件）
# ============================================================================

def preview(stride_deg: float, lift_deg: float, gait: str, steps: int = 8) -> int:
    print("=" * 78)
    print(" 步态预览（不连接硬件、不发任何帧）")
    print("=" * 78)
    offs = leg_phase_offsets(gait)
    print(" 步态      : %s   腿相位偏移 = %s" % (
        gait, ["%.0f°" % (o * RAD2DEG) for o in offs]))
    print(" 摆幅      : thigh ±%.1f°   calf 抬 %.1f°" % (stride_deg, lift_deg))
    print(" 频率      : %.1f Hz（由 STM32 产生，本工具只读）" % C.GAIT_FREQ_HZ)
    print()
    hdr = " 相位   " + "  ".join("%-11s" % n for n in
                                 ("lf_thigh", "lb_thigh", "rf_thigh", "rb_thigh",
                                  "lf_calf", "lb_calf", "rf_calf", "rb_calf"))
    print(hdr)
    print("  " + "-" * (len(hdr) - 1))
    for k in range(steps):
        theta = 2.0 * math.pi * k / steps
        q = gait_target(theta, offs, stride_deg * DEG2RAD, lift_deg * DEG2RAD)
        vals = [q[4], q[5], q[6], q[7], q[8], q[9], q[10], q[11]]
        print(" %5.0f°  " % (theta * RAD2DEG) +
              "  ".join("%+10.1f" % (v * RAD2DEG) for v in vals))
    print()
    print(" 说明：thigh 是前后摆，calf 的负值 = 屈膝抬脚。")
    print("       四腿之和应接近 0（重心不偏），若明显偏向一侧就查 sign。")
    return 0


# ============================================================================
# 真机
# ============================================================================

def run(args) -> int:
    link = PiLink(args.port, args.baud, verbose=False)
    try:
        link.open()
    except LinkError as e:
        print("❌ 打不开串口 %s: %s" % (args.port, e))
        return 1

    offs = leg_phase_offsets(args.gait)
    stride = args.stride_deg * DEG2RAD
    lift = args.lift_deg * DEG2RAD

    print("=" * 78)
    print(" 阶段 6：脚本化步态   %s @ %d" % (args.port, args.baud))
    print("=" * 78)
    print(" 步态 %s  thigh ±%.1f°  calf 抬 %.1f°  频率 %.1f Hz（STM32 产生）"
          % (args.gait, args.stride_deg, args.lift_deg, C.GAIT_FREQ_HZ))

    # ---- ⭐⭐ 先"热身"再检查（2026-10-17 修的自己的 bug）
    #
    #  ⚠️ 原来是"一上来就 wait_status() 检查 STAT_LINK_OK"，
    #     而那时本工具【一帧都还没发】，固件自然把链路判为失活
    #     （它的判据是"距上次收到任意帧的间隔"）。
    #     ⇒ 必然误报"链路不活"，且报错文案还错位成"未标定时绝不能发 ACTIVE"。
    #
    #  ⭐ 正确顺序：先发约 300 ms 的 MODE_IDLE（不上力矩）把链路建立起来，
    #     再读状态。这和 status_panel.py --keepalive 的做法一致。
    print(" 正在建立链路（发 300 ms 的 MODE_IDLE，不上力矩）...")
    warm_end = time.perf_counter() + 0.3
    while time.perf_counter() < warm_end:
        link.pump()
        link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE, flags=0)
        time.sleep(0.01)
    link.pump()

    # ---- 前置检查：必须已标定、链路活、IMU 正常 ----
    st = link.wait_status(timeout_ms=2000)
    tel = link.wait_tel_fast(timeout_ms=3000)
    if tel is None:
        print("❌ 3 秒内没收到 TEL_FAST —— 检查接线 / 波特率 / 设备节点")
        return 1
    if st is None:
        print("❌ 没收到 TEL_STATUS")
        return 1

    missing = []
    if not (st.status & ST_CALIBRATED):
        missing.append("未标定(STAT_CALIBRATED)")
    if not (st.status & ST_LINK_OK):
        missing.append("链路不活(STAT_LINK_OK) —— 热身 300 ms 后仍未建立")
    if not (st.status & ST_IMU_OK):
        missing.append("IMU 无效(STAT_IMU_OK)")
    if missing:
        print("❌ 拒绝运行：%s" % " / ".join(missing))
        print("   ⚠️ 三个前置条件都必须满足：")
        print("      · STAT_CALIBRATED —— 未标定时角度换算没有意义，绝不能发 ACTIVE")
        print("        （见 docs/42-pi-setup.md §D2 第 8 条）")
        print("      · STAT_LINK_OK    —— 稳住链路才能保证 100 Hz 连续下发")
        print("      · STAT_IMU_OK     —— 固件用它拦【带假姿态跑策略】")
        return 1

    if tel.err_flags:
        print("⚠️ err_flags=0x%02X（%s）" % (tel.err_flags, T.err_str(tel.err_flags)))

    print()
    print(" 当前关节角(deg):")
    for grp, base in (("hip  ", 0), ("thigh", 4), ("calf ", 8)):
        print("   %s %s" % (grp, " ".join("%+7.1f" % (tel.q[base + k] * RAD2DEG)
                                         for k in range(4))))
    print()

    if not args.yes:
        print("⚠️⚠️ 机器人必须【吊起 / 架住】，腿悬空。")
        print("⚠️⚠️ 一旦开始，12 个舵机会按 %.1f Hz 持续运动。" % C.GAIT_FREQ_HZ)
        try:
            if input("确认继续？输入 yes 回车: ").strip().lower() != "yes":
                print("已取消")
                return 1
        except (EOFError, KeyboardInterrupt):
            print("\n已取消")
            return 1

    prev = list(tel.q)          # 从实测位置起步，避免首帧猛冲
    t0 = time.perf_counter()
    t_end = t0 + args.seconds if args.seconds > 0 else float("inf")
    period = 1.0 / C.CONTROL_HZ
    t_next = t0
    t_next_print = t0
    ramp_s = max(args.ramp_s, 1e-6)
    n_sent = 0

    print("\n开始（Ctrl-C 中止，中止时会发 MODE_IDLE 卸力矩）\n")

    try:
        while time.perf_counter() < t_end:
            link.pump()
            now = time.perf_counter()
            if now < t_next:
                time.sleep(0.001)
                continue
            t_next = now + period

            tel = link.last_tel
            st = link.last_status

            # ---- 安全中止 ----
            if st is not None and (st.status & ST_ESTOP):
                print("\n⚠️ 固件进入 ESTOP —— 中止（力矩已由固件卸掉）")
                return 2

            theta = theta_from_tel(tel)
            scale = min(1.0, (now - t0) / ramp_s)      # 渐入
            want = clamp_soft(gait_target(theta, offs, stride, lift, scale,
                                          args.sign_thigh, args.sign_calf))
            prev = rate_limit(prev, want)

            # ⭐ FLAG_ENABLE 必须带：固件自 2026-10-16 起"没明确指令不上力矩"。
            if not link.send_cmd_joint(prev, mode=MODE_ACTIVE, flags=FLAG_ENABLE):
                print("\n❌ 发帧失败（串口写错误）—— 中止")
                return 1
            n_sent += 1

            # ⚠️ 只 2 Hz 打印：打印会占 GIL，打多了会把发帧饿死
            if now >= t_next_print:
                t_next_print = now + 0.5
                amp = T.derive_effective_mode(MODE_ACTIVE, st.status)[0] \
                    if st is not None else -1
                print(" [%5.1fs] θ=%+6.1f°  scale=%.2f  sent=%d  生效=%s"
                      % (now - t0, theta * RAD2DEG, scale, n_sent,
                         T.mode_name(amp)))

    except KeyboardInterrupt:
        print("\n收到 Ctrl-C —— 中止")
    finally:
        # ⚠️ 退出前必须发 IDLE：否则固件会继续跟踪最后一帧目标。
        try:
            if link.is_open:
                link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE, flags=0)
                link.pump()
                time.sleep(0.05)
        except Exception:
            pass
        link.close()

    print("\n结束：共发 %d 帧（%.1f 秒，约 %.0f Hz）"
          % (n_sent, max(time.perf_counter() - t0, 1e-9),
             n_sent / max(time.perf_counter() - t0, 1e-9)))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="RL-CAT 阶段 6：脚本化步态（开环）")
    ap.add_argument("--port", default=C.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=C.SERIAL_BAUD)
    ap.add_argument("--gait", choices=("trot", "walk", "pace"), default="trot")
    ap.add_argument("--stride-deg", type=float, default=12.0,
                    help="thigh 摆幅（度，单侧），默认 12")
    ap.add_argument("--lift-deg", type=float, default=18.0,
                    help="calf 抬腿幅度（度，正值=抬），默认 18")
    ap.add_argument("--sign-thigh", type=float, default=1.0, choices=(1.0, -1.0))
    ap.add_argument("--sign-calf", type=float, default=1.0, choices=(1.0, -1.0))
    ap.add_argument("--ramp-s", type=float, default=1.5,
                    help="渐入时长（秒），默认 1.5")
    ap.add_argument("--seconds", type=float, default=0.0, help="跑多久（0=一直）")
    ap.add_argument("--preview", action="store_true",
                    help="只打印一个周期的轨迹表，不连硬件")
    ap.add_argument("--yes", action="store_true", help="跳过交互确认")
    args = ap.parse_args()

    if args.preview:
        return preview(args.stride_deg, args.lift_deg, args.gait)

    print("关节软限位（度）:")
    for i in range(C.N_JOINTS):
        print("  %2d %-10s [%+6.1f, %+6.1f]" % (
            i, C.JOINT_NAMES[i],
            C.Q_SOFT_MIN[i] * RAD2DEG, C.Q_SOFT_MAX[i] * RAD2DEG))
    print()
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
