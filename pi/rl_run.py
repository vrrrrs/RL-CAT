#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 RL 策略接到真机控制回路上。

⚠️ 本文件是"胶水"，纯计算在 `rl_policy.py` 里（那边有离线对拍测试）。

安全设计（按重要性排序）
------------------------
1. **`--observe` 模式**：只读遥测 + 推理 + 打印，**绝不把 action 发给舵机**。
   但**仍然发 keepalive 帧**（`MODE_IDLE` + `q_default`）——
   不发帧会导致链路失联 ⇒ 固件进 ESTOP，反而危险。
   ⇒ 上真机的**第一步永远是这个模式**：确认 obs 合理、action 不发散。

2. **前置硬拒**：未标定 / 链路不活 / IMU 无效 → 直接退出。

3. **热身**：先发 3 秒"目标=当前姿态"，等固件退出 ESTOP 并进入 ACTIVE。

4. **渐入**：action 从 0 线性放大到满，默认 2 秒。避免第一帧猛冲。

5. **动作限幅**：`clip(±100)` + `q_target` 夹软限位（`rl_policy` 内已做）。

6. **ESTOP 容忍**：瞬时 ESTOP 继续发帧等它恢复；**持续 >0.6 s 才中止**。

7. **Ctrl-C**：`finally` 里发 `MODE_IDLE` 卸力矩。

用法
----
    # ⭐ 第一步：只观察，不动舵机
    python3 rl_run.py --observe --cmd 0 0 0 --seconds 10

    # 第二步：真跑（吊起！）
    python3 rl_run.py --cmd 0 0 0 --seconds 10 --yes

    # 前进
    python3 rl_run.py --cmd 0.3 0 0 --seconds 15 --yes
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
from rl_policy import Policy, build_obs, ACT_DIM        # noqa: E402
from rlcat_proto import (                               # noqa: E402
    FLAG_ENABLE,
    MODE_ACTIVE,
    MODE_IDLE,
    ST_CALIBRATED,
    ST_ESTOP,
    ST_IMU_OK,
    ST_LINK_OK,
)

DEG = 180.0 / math.pi

# ⚠️⚠️ 【实机】URDF 的关节限位（hardware/mechanical/URDF/robot.urdf）
#
#   ⚠️ 为什么不直接用 config.Q_SOFT_MIN/MAX：那两个是跟着【仿真 URDF】写的，
#      thigh 上限是 90°，而实机只有 30°（仿真为了给摆腿留空间放宽了）。
#
#   ⚠️ 策略在仿真里会把 thigh 推到 46.9°。若不夹，实机舵机会顶到机械限位：
#      大电流、异响，长时间会伤舵机/齿轮。
#
#   ⭐ 所以第一次上真机（尤其是跑未见过的指令）建议加 --clamp-real：
#      先把目标夹到实机真的能到的范围，看策略还能不能站住；
#      确认安全后再放开，对比两者的差异 —— 这个差异就是 G1 的量化值。
REAL_Q_MIN = [0.0] * 4 + [-35.0] * 4 + [-60.0] * 4
REAL_Q_MAX = [40.0] * 4 + [30.0] * 4 + [60.0] * 4
REAL_Q_MIN_RAD = [v / DEG for v in REAL_Q_MIN]
REAL_Q_MAX_RAD = [v / DEG for v in REAL_Q_MAX]


def _status_line(bits: int) -> str:
    return T.status_str(bits)


def main() -> int:
    ap = argparse.ArgumentParser(description="RL-CAT 真机策略运行器")
    ap.add_argument("--port", default=C.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=C.SERIAL_BAUD)
    ap.add_argument("--policy", default=os.path.join(_HERE, "models",
                                                    "policy_rlcat_flat.npz"))
    # ⚠️ 速度指令：物理量（m/s, rad/s），不是观测里的缩放值
    ap.add_argument("--cmd", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                    metavar=("VX", "VY", "WZ"),
                    help="速度指令（m/s, m/s, rad/s），默认原地站")
    ap.add_argument("--seconds", type=float, default=10.0, help="跑多久（0=一直）")
    ap.add_argument("--ramp", type=float, default=2.0,
                    help="动作渐入时长（秒），默认 2")
    # ⭐ 动作整体缩放 —— 第一次上真机最该用的旋钮
    #
    #  ⚠️ 为什么需要：docs/47 的交付报告把"仿真没有驱动器延迟（真机 10~30 ms）"
    #     列为本项目最大的 sim2real 风险。仿真里舵机是理想的，真机慢 1~3 个周期，
    #     于是**同样的动作在真机上会震荡，而仿真里完全看不出来**。
    #
    #  ⇒ 最便宜、最有效的缓解手段就是先把动作幅度缩小：
    #     0.5 = 半幅，能显著降低对延迟的敏感度；
    #     确认不震荡后再逐步放大到 1.0。
    #
    #  ⚠️ 注意它与动作限幅（clip）的区别：这里是【等比例缩小】，
    #     保持策略输出的**相对形状**不变，只降增益。
    ap.add_argument("--action-scale", type=float, default=1.0,
                    help="动作整体缩放（0.3~1.0）。⚠️ 第一次上真机建议 0.5，"
                         "用于抵消仿真缺失的驱动器延迟")
    ap.add_argument("--observe", action="store_true",
                    help="只观察推理，不把动作下发给舵机（⭐ 上机第一步用这个）")
    ap.add_argument("--clamp-real", action="store_true",
                    help="把 q_target 夹到【实机 URDF】限位（thigh ≤30°）。"
                         "⭐ 第一次上真机建议开启，避免顶到机械限位")
    ap.add_argument("--yes", action="store_true", help="跳过交互确认")
    args = ap.parse_args()

    try:
        pol = Policy(args.policy)
    except Exception as e:                              # noqa: BLE001
        print("❌ 加载策略失败：%s" % e)
        return 1

    link = PiLink(args.port, args.baud, verbose=False)
    try:
        link.open()
    except LinkError as e:
        print("❌ 打不开串口：%s" % e)
        return 1

    print("=" * 78)
    print(" RL-CAT 真机策略运行器   %s @ %d" % (args.port, args.baud))
    print("=" * 78)
    print(" 策略     : %s（%d 参数）"
          % (os.path.basename(args.policy),
             sum(W.size + b.size for W, b in pol.layers)))
    print(" 速度指令 : vx=%.2f  vy=%.2f  wz=%.2f m/s,rad/s" % tuple(args.cmd))
    print(" 模式     : %s" % ("⭐ 只观察（不下发动作）" if args.observe else "⚠️ 真跑"))
    print(" 动作缩放 : %.2f%s" % (args.action_scale,
          "" if args.action_scale <= 1.0 else "  ⚠️ >1 会放大动作，慎用"))
    print(" 限位夹取 : %s" % ("实机 URDF（thigh ≤30°）" if args.clamp_real
                              else "仿真限位（thigh ≤85°）⚠️ 可能顶机械限位"))

    # ---- 热身 + 前置检查 ----
    print("\n 建立链路（发 300 ms MODE_IDLE）...")
    warm_end = time.perf_counter() + 0.3
    while time.perf_counter() < warm_end:
        link.pump()
        link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE, flags=0)
        time.sleep(0.01)
    link.pump()

    st = link.wait_status(timeout_ms=2000)
    tel = link.wait_tel_fast(timeout_ms=3000)
    if tel is None or st is None:
        print("❌ 收不到遥测 —— 检查接线 / 波特率 / 设备节点")
        link.close()
        return 1

    missing = []
    if not (st.status & ST_CALIBRATED):
        missing.append("未标定")
    if not (st.status & ST_LINK_OK):
        missing.append("链路不活")
    if not (st.status & ST_IMU_OK):
        missing.append("IMU 无效")
    if missing:
        print("❌ 拒绝运行：%s" % " / ".join(missing))
        print("   ⚠️ 这三项都必须满足（见 docs/46-mainline-plan.md §2）")
        link.close()
        return 1

    # ---- 等 ESTOP 自清并进入 ACTIVE（目标=当前姿态，不会动）----
    print(" 等固件退出急停并上力矩（目标=当前姿态）...")
    cur = list(tel.q)
    arm_t0 = time.perf_counter()
    armed = False
    while time.perf_counter() < arm_t0 + 3.0:
        link.pump()
        link.send_cmd_joint(cur, mode=MODE_ACTIVE, flags=FLAG_ENABLE)
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
                  % (link.last_tel.status, _status_line(link.last_tel.status)))
        link.close()
        return 1
    print(" ✅ 已进入 ACTIVE")

    if not args.observe and not args.yes:
        print("\n⚠️⚠️ 即将【真正下发动作】。机器人必须【吊起或架住】，腿悬空。")
        try:
            if input("确认继续？输入 yes 回车：").strip().lower() != "yes":
                print("已取消")
                link.close()
                return 1
        except (EOFError, KeyboardInterrupt):
            print("\n已取消")
            link.close()
            return 1

    print()
    print("=" * 78)
    cmd = [float(v) for v in args.cmd]
    last_action = [0.0] * ACT_DIM
    period = 1.0 / C.CONTROL_HZ
    t0 = time.perf_counter()
    t_end = t0 + args.seconds if args.seconds > 0 else float("inf")
    t_next = t0
    ramp_s = max(args.ramp, 1e-6)
    n_sent = n_tick = 0
    bad_since = None
    worst_status = 0
    min_mv = 1 << 30
    print(" 跑 %s   Ctrl-C 中止" % ("%.0f s" % args.seconds if args.seconds > 0 else "一直"))

    try:
        while time.perf_counter() < t_end:
            link.pump()
            now = time.perf_counter()
            tel = link.last_tel
            if tel is None:
                time.sleep(0.001)
                continue
            if now < t_next:
                time.sleep(0.001)
                continue
            t_next = now + period

            # ---- 构造 obs（状态来自上一帧遥测，与仿真 shift=+1 一致）----
            obs = build_obs(q=tel.q, qd=tel.qd, gyro=tel.gyro,
                            quat_wxyz=tel.quat, cmd_vel=cmd,
                            last_action=last_action,
                            gait_sin=tel.gait_sin, gait_cos=tel.gait_cos)
            action = pol.act(obs)

            # ---- 渐入 + 动作缩放 ----
            # ⚠️ 两者相乘：ramp 解决"起步猛"，action_scale 解决"整体增益过高"
            scale = min(1.0, (now - t0) / ramp_s) * float(args.action_scale)
            last_action = (action * scale).astype("float32")

            q_target = pol.action_to_q_target(last_action)

            # ⭐ --clamp-real：再夹一次【实机】限位（见文件顶部的 REAL_Q_* 说明）
            #   ⚠️ 这一层是"机械保护"，不是"控制策略"。
            #      它会让策略行为偏离仿真 —— 那个偏差正是 G1 的代价，
            #      第一次上真机时宁可偏一点，也别顶坏舵机。
            if args.clamp_real:
                for _i in range(ACT_DIM):
                    q_target[_i] = min(max(float(q_target[_i]),
                                           REAL_Q_MIN_RAD[_i]),
                                       REAL_Q_MAX_RAD[_i])

            # ---- 健康检查 ----
            if tel.voltage_mv > 0:
                min_mv = min(min_mv, tel.voltage_mv)
            bad = bool(tel.status & ST_ESTOP) or \
                (T.derive_effective_mode(MODE_ACTIVE, tel.status)[0] != MODE_ACTIVE)
            if bad:
                worst_status = tel.status
                if bad_since is None:
                    bad_since = now
                elif now - bad_since > 0.6:
                    print("\n❌ 非 ACTIVE/ESTOP 持续 %.1f 秒，中止" % (now - bad_since))
                    print("   状态位 = 0x%02X（%s）" % (tel.status, _status_line(tel.status)))
                    print("   电压   = 最低 %d mV" % min_mv)
                    break
            else:
                bad_since = None

            # ---- 下发 ----
            if args.observe:
                # ⚠️ 只发 keepalive（目标=当前姿态），绝不发 action
                link.send_cmd_joint(list(tel.q), mode=MODE_ACTIVE, flags=FLAG_ENABLE)
            else:
                link.send_cmd_joint(q_target, mode=MODE_ACTIVE, flags=FLAG_ENABLE)
                n_sent += 1

            # ⚠️⚠️ 控制循环里【不许 print】（2026-10-18 修）
            #
            #  原来这里 2 Hz 打一行状态。看着无害，但 `print` 到 SSH 管道会
            #  阻塞几十毫秒，期间不发帧 ⇒ 链路空档 > 80 ms ⇒ `NO_LINK`
            #  ⇒ 策略在"ACTIVE/STAND"之间被反复打断 ⇒ 观测与控制双重失真。
            #
            #  ⭐ 实测：stand_test.py 的 2 Hz 打印造成 14.3% 的样本掉 ACTIVE。
            #  ⇒ 所有实时循环里的进度输出一律改为"只在结束时汇总"。
            n_tick += 1

    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
    finally:
        try:
            if link.is_open:
                link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE, flags=0)
                link.pump()
                time.sleep(0.05)
        except Exception:                               # noqa: BLE001
            pass
        link.close()

    el = max(time.perf_counter() - t0, 1e-9)
    print("\n" + "=" * 78)
    print(" 结束：%.1f s，%d 个控制拍%s，约 %.0f Hz"
          % (el, n_tick, "（其中下发 %d 帧）" % n_sent if not args.observe else "（只观察）",
             n_tick / el))
    print(" 电压最低 %d mV" % min_mv)
    print("=" * 78)
    return 0


if __name__ == "__main__":
    sys.exit(main())
