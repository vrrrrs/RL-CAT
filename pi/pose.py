#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 树莓派侧：阶段 5 全身姿态

把 12 个关节角一次性下发，让机器人摆到指定姿态；从当前姿态到目标姿态
用 smoothstep（缓入缓出）按限速过渡，绝不跳变。

验收判据（docs/42-pi-setup.md §A2 阶段 5）：
    □ 能摆出站立姿态并保持
    □ 能缓慢改变姿态（如缓慢下蹲/起身）而不失稳

⚠️⚠️ 关于"姿态"的诚实说明：
   · 唯一【被实测确认过】的姿态是 **全 0 = 弯腿站姿**（q_default）。
   · 本脚本【不发明】任何姿态。下蹲/抬腿这类姿态需要真实运动学与限位配合，
     应由你（或 IK）给出具体角度，再写进 --pose / --pose-file。
     poses/example.json 是一个模板，里面的数值【未经真机验证】。

⚠️⚠️ 必须先把机器人【吊起或垫高】。落地跑本脚本没有意义，也可能摔。

用法：
    python3 pose.py --pose zero --yes                    # 摆回站姿
    python3 pose.py --pose "0,0,0,0,10,10,10,10,0,0,0,0" --yes --ramp 10
    python3 pose.py --pose-file poses/example.json --yes
    python3 pose.py --pose zero --from "0,0,0,0,20,20,20,20,0,0,0,0" --cycles 3 --yes
    python3 pose.py --list

退出码：0 = 到达目标并收敛；1 = 未收敛/被拒/有告警；2 = 打不开串口
"""

from __future__ import annotations

# ⚠️ Windows 控制台默认是 GBK，中文输出会乱码。在任何输出之前切成 UTF-8。
import sys as _sys
for _s in (_sys.stdout, _sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import argparse
import json
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
    MODE_STAND,
    ST_BATT_FORCE,
    ST_CALIBRATED,
    ST_ESTOP,
    ST_IMU_OK,
    ST_LINK_OK,
    ST_READY,
)

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

# 预设姿态。⚠️ 只放"已验证"的：全 0 就是实测确认过的弯腿站姿。
PRESETS = {
    "zero": {
        "q_deg": [0.0] * C.N_JOINTS,
        "note": "全 0 = q_default = 实测确认的弯腿站姿（唯一已验证的姿态）",
    },
}


def parse_pose_str(s: str, unit: str) -> list:
    parts = [p for p in s.replace(";", ",").split(",") if p.strip() != ""]
    if len(parts) != C.N_JOINTS:
        raise ValueError("需要 %d 个角度，给了 %d 个" % (C.N_JOINTS, len(parts)))
    vals = [float(p) for p in parts]
    if unit == "deg":
        return [v * DEG2RAD for v in vals]
    return vals


def load_pose_file(path: str) -> tuple:
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    unit = d.get("unit", "deg")
    if "q_deg" in d:
        vals = d["q_deg"]
        unit = "deg"
    elif "q_rad" in d:
        vals = d["q_rad"]
        unit = "rad"
    elif "q" in d:
        vals = d["q"]
    else:
        raise ValueError("文件里要有 q_deg / q_rad / q 之一")
    if len(vals) != C.N_JOINTS:
        raise ValueError("文件里的角度个数是 %d，应为 %d" % (len(vals), C.N_JOINTS))
    q = [float(v) * (DEG2RAD if unit == "deg" else 1.0) for v in vals]
    return q, d.get("name", os.path.basename(path)), d.get("note", "")


class PoseRunner:
    """姿态过渡：smoothstep 插值 + 限速 + 100 Hz 发帧 + 保活。"""

    def __init__(self, link: PiLink, hz: float = 100.0):
        self.link = link
        self.period = 1.0 / hz
        self._t_next = 0.0

    def _tick(self, cmd: list) -> None:
        now = time.perf_counter()
        if now >= self._t_next:
            self._t_next = now + self.period
            # ⭐⭐ 必须带 FLAG_ENABLE（2026-10-17 修的 showstopper）
            #
            #  ⚠️ 固件自 2026-10-16 起有一条"上锁"规则：
            #     没有明确指令（CMD_JOINT 带 FLAG_ENABLE）之前，
            #     **不准上力矩、不准写位置**（见 app_main.c 的 ⑥ 与 ⑦）。
            #
            #  ⚠️ 而本层所有工具原来都发 `flags=0`
            #     （`rlcat_proto.make_cmd_joint` 的默认值才是 FLAG_ENABLE，
            #      但每个调用点都显式传了 0）——
            #     ⇒ 固件的 s_armed 永远是 false
            #     ⇒ 力矩被强制关、位置永远不写
            #     ⇒ **机器人永远不会动**（表现为 pose.py 一直报"未收敛"）。
            #
            #  ⭐ 语义：FLAG_ENABLE = "我现在要求你接管并输出力矩"。
            #     所以只有**真的要动**的工具才带它：
            #       ✅ pose.py / jog.py / acceptance.py --move
            #       ❌ status_panel.py / calib.py（观察与静态量测，不该上力矩）
            self.link.send_cmd_joint(cmd, mode=MODE_ACTIVE, flags=FLAG_ENABLE)
        self.link.pump()

    def measured(self) -> list:
        tel = self.link.last_tel
        return list(tel.q) if tel is not None else list(C.Q_DEFAULT)

    def goto(self, target: list, *, rate_deg_s: float, timeout_s: float,
             tol_deg: float, label: str = "") -> tuple:
        """从【当前实测姿态】平滑过渡到 target。返回 (是否收敛, 说明)。"""
        self.link.wait_tel_fast(timeout_ms=1000)
        start = self.measured()

        # 目标按软限位夹紧（并如实报告）
        clamped, over = [], []
        for i, v in enumerate(target):
            c = C.clamp_q(v, i)
            if abs(c - v) > 1e-9:
                over.append((i, v * RAD2DEG, c * RAD2DEG))
            clamped.append(c)

        max_delta = max(abs(clamped[i] - start[i]) for i in range(C.N_JOINTS))
        rate = rate_deg_s * DEG2RAD
        dur = (max_delta / rate) if rate > 0 else 0.0

        note = "%s：Δmax %.2f°，按 %.1f°/s 预计 %.2f s" % (
            label or "姿态", max_delta * RAD2DEG, rate_deg_s, dur)
        if over:
            note += "；⚠️ %d 个关节被软限位夹紧" % len(over)

        # ⭐⭐⭐ 运动前先"等急停自清"（2026-10-17 修的关键 bug）
        #
        #  ⚠️⚠️ 现场症状：`--ramp 15` 报"过程中进入 ESTOP，已中止"，
        #      而状态位是 0x9F（LINK_OK 亮、无 BATT_FORCE）+ 电压 min==max。
        #      那说明【只采到一个样本】⇒ 第一帧反馈就已经是 ESTOP 了。
        #
        #  ⭐ 真因：
        #      ① 上一个工具退出 / 本工具打印姿态表期间，一个帧都没发
        #      ② 空档 > RLCA_LINK_ESTOP_MS → 固件进 ESTOP（卸力矩）
        #      ③ 本工具刚开始发帧，固件要过 MODE_MIN_HOLD_MS(100 ms) 才退出 ESTOP
        #      ④ 于是"第一帧新鲜反馈"里 ST_ESTOP 仍然亮着
        #      ⑤ 运动循环一看到就判"过程中进入 ESTOP"并退出
        #      ⇒ 表现成"有时成功、有时失败"（取决于打印耗时是否刚好让 ESTOP 自清）
        #
        #  ⭐ 修法：运动开始前，用【当前实测姿态】当目标持续发帧（不会动），
        #     等到 ESTOP 真的熄灭再开始计时。这样 loop_t0 之后的帧一定是干净的。
        print(" 正在等待固件退出急停并上力矩（目标=当前姿态，不会动）...")
        arm_t0 = time.perf_counter()
        arm_deadline = arm_t0 + 3.0
        armed = False
        last_bits = 0
        while time.perf_counter() < arm_deadline:
            self._tick(start)
            t = self.link.last_tel
            if (t is not None) and (self.link.t_rx_tel_fast >= arm_t0):
                last_bits = t.status
                if not (t.status & ST_ESTOP):
                    # ESTOP 已灭 ⇒ 再确认模式真的进了 ACTIVE
                    if T.derive_effective_mode(MODE_ACTIVE, t.status)[0] == MODE_ACTIVE:
                        armed = True
                        break
            time.sleep(0.002)
        if not armed:
            return False, note + (
                "；❌ 3 秒内没能进入 ACTIVE（等急停自清失败）"
                "\n     最后状态位 = 0x%02X（%s）"
                "\n     ⇒ 若含 ESTOP 且含 LINK_OK：查 safety_reasons（过温/电压）"
                "\n     ⇒ 若含 ESTOP 但不含 LINK_OK：链路空档 >%d ms"
                % (last_bits, T.status_str(last_bits), C.LINK_ESTOP_MS))

        t0 = time.perf_counter()
        settle_deadline = None
        # ⚠️ 只信【本循环开始之后】收到的帧。否则上电那段的 ESTOP 帧会被当成
        #    "过程中进入 ESTOP"，姿态一发出就立刻中止（这个坑踩过一次）。
        loop_t0 = time.perf_counter()

        # ⭐⭐ 诊断用：记录运动过程中的极值（2026-10-17 加）
        #   ⚠️ 背景：15°/s 会 ESTOP、5°/s 不会，而 uptime 复位=0、周期正常、
        #      单独跑不丢帧 ⇒ 只剩"电池电压在浪涌下跌破 9300 mV"这一条路。
        #      ESTOP 是瞬态的，敲 R 时早就恢复了，所以必须在循环里就地采样。
        min_mv = 1 << 30
        max_mv = 0
        worst_status = 0

        # ⭐⭐ 容忍瞬时降级（2026-10-17 修）
        #
        #  ⚠️ 背景：热身循环明明刚成功进了 ACTIVE，运动循环的第一帧却报
        #      "生效模式=STAND（状态位 0x1B，LINK_OK 熄灭）"。
        #      0x1B 说明空档落在 80~300 ms 的【降级区】—— ESTOP 不触发，
        #      但 ACTIVE 准入的第 ② 道门过不去。
        #
        #  ⭐ 关键认识：**单帧抖动不该中止一整趟运动。**
        #      固件在 STAND 下会忽略 pi_q_target、保持 q_def，
        #      所以此时继续发帧【是安全的】—— 机器人不动而已。
        #      等链路恢复、模式回到 ACTIVE，它会自然接着跟踪。
        #
        #  ⇒ 规则：只有"非 ACTIVE"或"ESTOP"**持续**超过 BAD_ABORT_S 才中止。
        #     ESTOP 也走同一条规则：瞬态 ESTOP 之后固件自己会恢复。
        BAD_ABORT_S = 0.6
        bad_since = None
        n_bad_frames = 0
        was_estop = False
        events = []          # 每次"降级→恢复"的时长（ms），循环里只累计不打印

        while True:
            now = time.perf_counter()
            s = 1.0 if dur <= 0 else min(1.0, (now - t0) / dur)
            e = s * s * (3.0 - 2.0 * s)                    # smoothstep
            cmd = [start[i] + (clamped[i] - start[i]) * e
                   for i in range(C.N_JOINTS)]
            self._tick(cmd)

            tel = self.link.last_tel
            fresh = (tel is not None) and (self.link.t_rx_tel_fast >= loop_t0)
            if fresh:
                if tel.voltage_mv > 0:
                    min_mv = min(min_mv, tel.voltage_mv)
                    max_mv = max(max_mv, tel.voltage_mv)

                bad = bool(tel.status & ST_ESTOP)
                eff = T.derive_effective_mode(MODE_ACTIVE, tel.status)[0]
                if eff != MODE_ACTIVE:
                    bad = True
                if tel.status & ST_ESTOP:
                    was_estop = True
                    worst_status = tel.status

                if bad:
                    n_bad_frames += 1
                    if bad_since is None:
                        bad_since = now
                    elif (now - bad_since) >= BAD_ABORT_S:
                        # 持续失效 ⇒ 真的中止
                        diag = ("；❌ %s 持续 %.1f 秒，已中止"
                                "\n     状态位 = 0x%02X（%s）"
                                "\n     生效模式 = %s"
                                "\n     电压   = 最低 %s / 最高 %s"
                                % ("ESTOP" if was_estop else "非 ACTIVE",
                                   now - bad_since,
                                   tel.status, T.status_str(tel.status),
                                   T.mode_name(eff),
                                   ("%d mV" % min_mv) if min_mv < (1 << 30) else "无",
                                   ("%d mV" % max_mv) if max_mv else "无"))
                        if tel.status & ST_BATT_FORCE:
                            diag += ("\n     ⇒ 🎯 STAT_BATT_FORCE：电压跌到 ≤9300 mV。")
                        elif not (tel.status & ST_LINK_OK):
                            diag += ("\n     ⇒ ⚠️ STAT_LINK_OK 熄灭：链路空档 >%d ms。\n"
                                     "        ⇒ 先确认没有别的工具占用串口"
                                     "（现在有 flock 会拦住）" % C.LINK_LOST_MS)
                        elif tel.status & ST_ESTOP:
                            diag += ("\n     ⇒ 链路正常却持续 ESTOP ⇒ 查 safety_reasons"
                                     "（过温 0x10 / 电压 0x01）")
                        return False, note + diag
                else:
                    # 恢复正常（含"一直是好的"）
                    #
                    # ⚠️⚠️ 绝对不要在运动循环里 print（2026-10-17 踩到）：
                    #     `print` 到 SSH 管道可能阻塞几十毫秒，而它恰好发生在
                    #     "降级刚恢复"的瞬间 ⇒ 这几十毫秒又没发帧 ⇒ 又超阈值
                    #     ⇒ 又降级 ⇒ 又打印 … 形成正反馈。
                    #     实测：42 次打印把降级次数从个位数推到 750 帧。
                    #  ⇒ 只累计，循环结束后统一汇报。
                    if bad_since is not None:
                        events.append((now - bad_since) * 1000.0)
                    bad_since = None
                    was_estop = False

            if s >= 1.0:
                if settle_deadline is None:
                    settle_deadline = now + timeout_s
                err = max(abs(tel.q[i] - clamped[i]) for i in range(C.N_JOINTS)) \
                    if fresh else 9.9
                if err * RAD2DEG <= tol_deg:
                    extra = ""
                    if events:
                        ev = sorted(events)
                        extra = ("（过程降级 %d 次，最长 %.0f ms，中位 %.0f ms）"
                                 % (len(ev), ev[-1], ev[len(ev) // 2]))
                    elif n_bad_frames:
                        extra = "（过程降级 %d 帧，已全部恢复）" % n_bad_frames
                    return True, note + "；✅ 收敛（残差 %.2f°）%s" \
                        % (err * RAD2DEG, extra)
                if now >= settle_deadline:
                    bad = max(range(C.N_JOINTS),
                              key=lambda i: abs(tel.q[i] - clamped[i])) \
                        if tel is not None else -1
                    return False, (note + "；❌ 未收敛：关节 %s 残差 %.2f°（判据 %.2f°）"
                                   % (C.JOINT_NAMES[bad] if bad >= 0 else "?",
                                      err * RAD2DEG, tol_deg))

            time.sleep(0.002)


def print_pose(title: str, q: list) -> None:
    """打印 12 个关节角（度 + 弧度两行）。⚠️ 传入的 q 是【弧度】。"""
    if title:
        print("  %s" % title)
    for r, kind in ((0, "hip  "), (4, "thigh"), (8, "calf ")):
        deg = [v * RAD2DEG for v in q[r:r + 4]]
        print("      %s %s  (deg)" % (kind, T.fmt_joints(deg, 0, 4, fmt="%+7.2f")))
        print("      %s %s  (rad)" % (" " * len(kind),
                                      T.fmt_joints(q, r, 4, fmt="%+7.4f")))


def precheck(link: PiLink) -> tuple:
    notes = []
    tel = link.wait_tel_fast(timeout_ms=2000)
    if tel is None:
        return False, ["2 秒内没收到 TEL_FAST —— 链路不通"]
    fatal = False
    if not (tel.status & ST_CALIBRATED):
        notes.append("❌ STAT_CALIBRATED 未置位 —— ACTIVE 会被静默降级成 STAND，"
                     "姿态不会生效。先跑 calib.py --start")
        fatal = True
    if not (tel.status & ST_READY):
        notes.append("⚠️ STAT_READY 未置位")
    if not (tel.status & ST_IMU_OK):
        notes.append("⚠️ STAT_IMU_OK 未置位")
    if tel.err_flags:
        notes.append("⚠️ err_flags=0x%02X（%s）—— latch；安全层会把幅度压到 75%%，"
                     "即姿态只能到目标的 3/4" % (tel.err_flags, T.err_str(tel.err_flags)))
    return (not fatal), notes


def main() -> int:
    ap = argparse.ArgumentParser(description="RL-CAT 阶段 5：全身姿态")
    ap.add_argument("--port", default=C.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=C.SERIAL_BAUD)
    ap.add_argument("--pose", default=None, help='12 个角度，如 "0,0,0,0,10,..."')
    ap.add_argument("--pose-file", default=None, help="JSON 姿态文件")
    ap.add_argument("--unit", choices=("deg", "rad"), default="deg",
                    help="--pose 的单位（默认度）")
    ap.add_argument("--from", dest="from_pose", default=None,
                    help="先摆到这个起始姿态（度），用于 --cycles")
    ap.add_argument("--cycles", type=int, default=1,
                    help="在 from 与目标之间来回几趟（演示'缓慢改变姿态'）")
    ap.add_argument("--ramp", type=float, default=15.0, help="最大关节速度 度/秒")
    ap.add_argument("--hold", type=float, default=1.0, help="到位后保持几秒")
    ap.add_argument("--tol", type=float, default=2.0, help="收敛判据（度）")
    ap.add_argument("--timeout", type=float, default=8.0, help="收敛等待上限（秒）")
    ap.add_argument("--list", action="store_true", help="打印预设与关节表")
    ap.add_argument("--yes", action="store_true", help="跳过交互确认")
    args = ap.parse_args()

    if args.list:
        print("可用预设：")
        for k, v in PRESETS.items():
            print("  %-8s %s" % (k, v["note"]))
        print("\n关节表（逻辑索引 A）：")
        print("  索引  关节名      类型    软限位(度)")
        for i in range(C.N_JOINTS):
            print("  %2d    %-10s  %-6s  %6.1f ~ %+6.1f"
                  % (i, C.JOINT_NAMES[i], C.JOINT_KIND[i],
                     C.Q_SOFT_MIN[i] * RAD2DEG, C.Q_SOFT_MAX[i] * RAD2DEG))
        print("\n⚠️ 本脚本不发明姿态。下蹲/抬腿这类姿态请自己算好角度后放进 "
              "--pose / --pose-file。")
        return 0

    # ---- 解析目标姿态 ----
    if args.pose_file:
        try:
            target, name, note = load_pose_file(args.pose_file)
        except Exception as e:
            print("!! 读姿态文件失败：%s" % e)
            return 1
        print("姿态文件：%s（%s）" % (args.pose_file, name))
        if note:
            print("  说明：%s" % note)
    elif args.pose:
        key = args.pose.strip().lower()
        if key in PRESETS:
            target = list(PRESETS[key]["q_deg"])
            target = [v * DEG2RAD for v in target]
            print("预设姿态：%s（%s）" % (key, PRESETS[key]["note"]))
        else:
            try:
                target = parse_pose_str(args.pose, args.unit)
            except Exception as e:
                print("!! --pose 解析失败：%s" % e)
                return 1
    else:
        print("!! 必须给 --pose 或 --pose-file（或 --list 看可选项）")
        return 1

    start_pose = None
    if args.from_pose:
        try:
            start_pose = parse_pose_str(args.from_pose, args.unit)
        except Exception as e:
            print("!! --from 解析失败：%s" % e)
            return 1

    try:
        link = PiLink(args.port, args.baud, verbose=False)
        link.open()
    except LinkError as e:
        print("!! %s" % e)
        return 2

    rc = 0
    runner = PoseRunner(link)
    cal_ok = False          # ⚠️ 只有确认已标定，退出时才敢发 STAND（见 finally）
    try:
        print("=" * 78)
        print(" 阶段 5：全身姿态   %s @ %d" % (args.port, args.baud))
        print("=" * 78)
        print("⚠️⚠️ 请确认机器人已【吊起或垫高】，腿部悬空、周围无人。")

        ok, notes = precheck(link)
        for n in notes:
            print("  " + n)
        tel = link.last_tel
        cal_ok = bool(tel.status & ST_CALIBRATED) if tel is not None else False
        if not ok:
            return 1

        print()
        print_pose("当前实测姿态：", runner.measured())
        print_pose("目标姿态：", target)
        print("  限速 %.1f°/s   收敛判据 %.1f°   保持 %.1f s"
              % (args.ramp, args.tol, args.hold))

        if not args.yes:
            try:
                r = input("确认开始？输入 yes 继续：").strip().lower()
            except EOFError:
                r = ""
            if r not in ("y", "yes"):
                print("已取消")
                return 1

        # ---- 需要的话先摆到起始姿态 ----
        if start_pose is not None:
            print("\n[起始] 先摆到 --from 姿态")
            ok, why = runner.goto(start_pose, rate_deg_s=args.ramp,
                                  timeout_s=args.timeout, tol_deg=args.tol,
                                  label="起始")
            print("      " + why)
            if not ok:
                return 1

        # ---- 来回若干趟 ----
        for c in range(max(1, args.cycles)):
            print("\n[第 %d/%d 趟] → 目标姿态" % (c + 1, max(1, args.cycles)))
            ok, why = runner.goto(target, rate_deg_s=args.ramp,
                                  timeout_s=args.timeout, tol_deg=args.tol,
                                  label="去程")
            print("      " + why)
            if not ok:
                rc = 1
                break

            if args.hold > 0:
                t_end = time.perf_counter() + args.hold
                while time.perf_counter() < t_end:
                    runner._tick(target)
                    time.sleep(0.002)
                tel = link.last_tel
                if tel is not None:
                    err = max(abs(tel.q[i] - target[i]) for i in range(C.N_JOINTS))
                    print("      保持 %.1f s 后残差 %.2f°%s"
                          % (args.hold, err * RAD2DEG,
                             "  ⚠️ 有下垂（可能是幅度被安全层缩放或力矩不足）"
                             if err * RAD2DEG > 2 * args.tol else ""))

            if args.cycles > 1 and start_pose is not None and c < args.cycles - 1:
                print("[第 %d/%d 趟] ← 回起始姿态" % (c + 1, max(1, args.cycles)))
                ok, why = runner.goto(start_pose, rate_deg_s=args.ramp,
                                      timeout_s=args.timeout, tol_deg=args.tol,
                                      label="回程")
                print("      " + why)
                if not ok:
                    rc = 1
                    break

        print("\n最终实测姿态：")
        print_pose("", runner.measured())
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
        rc = 1
    finally:
        try:
            if link.is_open:
                # ⚠️⚠️ 只有【已标定】时才发 STAND。
                #    固件的 STAND 没有任何准入门：未标定（默认标定表 dir=+1/offset=0）时，
                #    q_target=0 会换算成舵机 step 0，把 12 个关节全部驱到机械行程一端。
                #    所以未标定时只发 MODE_IDLE（卸力），不发任何带目标角的模式。
                if cal_ok:
                    link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_STAND, flags=0)
                    time.sleep(0.3)
                link.send_cmd_mode(MODE_IDLE)
                time.sleep(0.2)
        except Exception:
            pass
        link.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
