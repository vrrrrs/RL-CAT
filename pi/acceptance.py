#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 树莓派侧：阶段 1~5 一键验收

一条命令跑完五个阶段的验收项，输出带勾的清单（可选 JSON 报告）。

⚠️⚠️ 默认【只读】，不会让舵机动。要让机器人动，必须显式加 `--move`。

⚠️⚠️ 关于"保活发什么"这一条安全设计（读固件源码得出的结论，很重要）：

    · 已标定  → 发 MODE_STAND：力矩使能，握住站姿，机器人纹丝不动
    · 未标定  → 只发 MODE_IDLE：卸载力矩

    为什么未标定时【绝不能】发 STAND/ACTIVE：
      固件只给 ACTIVE 加了准入门（app_mode.c），STAND **没有任何准入检查**。
      而默认标定表是 dir=+1 / offset=0（rlca_calib_load_defaults 里 memset 出来的），
      q_target = 0（正是站姿）按 rlca_q_to_step 换算出来是【舵机 step 0】——
      于是 12 个舵机全部冲向绝对位置 0，也就是机械行程的一端。
      固件作者在 app_main.c:695-709 记录过这个撞机事故。
      也就是说："未标定 → ACTIVE 降级成 STAND" 这个保护，对未标定的机器人
      本身就是危险的路径。

用法：
    python3 acceptance.py                        # 只读验收（安全）
    python3 acceptance.py --json report.json     # 同时写 JSON 报告
    python3 acceptance.py --move --yes           # 含动作项（⚠️ 必须吊起）
    python3 acceptance.py --move --calibrate --yes   # 连标定一起跑
    python3 acceptance.py --port /dev/pts/0 --move --yes   # 对着假 STM32 跑

退出码：0 = 没有失败项；1 = 有失败项；2 = 打不开串口
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
import datetime
import json
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import calib as CAL                                     # noqa: E402
import config as C                                      # noqa: E402
import jog as JOG                                       # noqa: E402
import pose as POSE                                     # noqa: E402
import stats as S                                       # noqa: E402
import telemetry as T                                   # noqa: E402
from link import PiLink, LinkError                      # noqa: E402
from rlcat_proto import (                               # noqa: E402
    MODE_IDLE,
    MODE_STAND,
    ST_CALIBRATED,
    ST_ESTOP,
    ST_IMU_OK,
    ST_LINK_OK,
    ST_READY,
    quat_to_gravity,
)

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

PASS, FAIL, SKIP, MANUAL = "pass", "fail", "skip", "manual"
ICON = {PASS: "✅", FAIL: "❌", SKIP: "⏭ ", MANUAL: "⏸ "}

W = 80


class Check:
    __slots__ = ("cid", "title", "status", "detail")

    def __init__(self, cid: str, title: str, status: str, detail: str = ""):
        self.cid = cid
        self.title = title
        self.status = status
        self.detail = detail

    def as_dict(self) -> dict:
        return {"id": self.cid, "title": self.title,
                "status": self.status, "detail": self.detail}


class Acceptance:
    #: 点动验收用的角度（对所有关节都在软限位内）
    JOG_DEG = 15.0
    JOG_TOL_DEG = 2.5
    JOG_OTHER_MAX_DEG = 3.0        # 动一个关节时，其余关节允许的最大漂移

    def __init__(self, link: PiLink, *, allow_move: bool = False,
                 assume_yes: bool = False, do_calibrate: bool = False,
                 temp_unit: str = "raw", verbose: bool = True):
        self.link = link
        self.allow_move = allow_move
        self.assume_yes = assume_yes
        self.do_calibrate = do_calibrate
        self.temp_unit = temp_unit
        self.verbose = verbose
        self.checks: list = []
        self.notes: list = []
        self.cal_ok = False
        self.estop_seen = False

    # ---------------------------------------------------------------- 工具
    def add(self, cid: str, title: str, status: str, detail: str = "") -> None:
        self.checks.append(Check(cid, title, status, detail))
        if self.verbose:
            line = "  %s %-7s %s" % (ICON[status], cid, title)
            if detail:
                line += "\n              %s" % detail
            print(line)

    def note(self, text: str) -> None:
        self.notes.append(text)
        if self.verbose:
            print("  ℹ️  %s" % text)

    def phase(self, title: str) -> None:
        if self.verbose:
            print("\n[%s]" % title)

    def keepalive(self) -> None:
        """链路保活。⚠️ 未标定时只发 IDLE，绝不发 STAND（见模块 docstring）。"""
        if self.cal_ok:
            self.link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_STAND, flags=0)
        else:
            self.link.send_cmd_mode(MODE_IDLE)

    def refresh(self) -> None:
        self.link.pump()
        tel = self.link.last_tel
        if tel is not None:
            self.cal_ok = bool(tel.status & ST_CALIBRATED)
            if tel.status & ST_ESTOP:
                self.estop_seen = True

    def pump_for(self, seconds: float) -> None:
        t_end = time.perf_counter() + seconds
        while time.perf_counter() < t_end:
            self.refresh()
            time.sleep(0.001)

    def wait_status_alive(self, timeout_s: float = 3.0):
        """等一帧【新的】TEL_STATUS，同时持续发帧保活。

        ⚠️ 不能像 link.wait_status() 那样"只等不发"：固件的失联判定基于
           "距上次收到任意帧"，超过 30 ms 就会把 STAT_LINK_OK 清掉，
           于是随后的检查会读到"链路不活跃"这种假失败。
        """
        t0 = time.perf_counter()
        t_end = t0 + timeout_s
        while time.perf_counter() < t_end:
            self.keepalive()
            self.link.pump()
            if (self.link.last_status is not None
                    and self.link.t_rx_status >= t0):
                return self.link.last_status
            time.sleep(0.002)
        return self.link.last_status

    def settle(self, runner, tol_deg: float, timeout_s: float) -> tuple:
        """等 runner 的目标收敛到 tol_deg 以内；每圈都保活并刷新状态。

        返回 (是否达到 tol_deg, 最终最大误差度)。
        ⚠️ 调用方应当用返回的"最大误差"再按自己的判据判定 ——
           不要"一碰到宽松判据就返回"，那样验收会变成擦边过。
        """
        t_end = time.perf_counter() + timeout_s
        worst = 1e9
        while time.perf_counter() < t_end:
            runner.tick()
            self.refresh()                 # ← 这里才能记录"点动期间有没有 ESTOP"
            tel = self.link.last_tel
            if tel is not None:
                worst = max(abs(tel.q[i] - runner.state.q[i])
                            for i in range(C.N_JOINTS))
                if worst * RAD2DEG <= tol_deg:
                    return True, worst * RAD2DEG
            time.sleep(0.002)
        return False, (worst * RAD2DEG if worst < 1e8 else float("inf"))

    # ---------------------------------------------------------------- P1
    def phase_link(self) -> bool:
        self.phase("阶段 1 · 链路")

        tel = self.link.wait_tel_fast(timeout_ms=3000)
        if tel is None:
            self.add("P1.1", "能收到 TEL_FAST", FAIL,
                     "3 秒内一帧都没收到 —— 查接线（交叉）/ 共地 / 波特率 / 设备节点")
            return False
        self.cal_ok = bool(tel.status & ST_CALIBRATED)
        self.add("P1.1", "能收到 TEL_FAST", PASS, "首帧 cycle_cnt=%d" % tel.cycle_cnt)

        # 1 秒窗口测帧率 / CRC / 丢帧
        n0 = self.link.stats.rx_by_type.get(0x02, 0)
        c0 = self.link.stats.crc_errors
        g0 = self.link.stats.cycle_gaps
        t0 = time.perf_counter()
        self.pump_for(1.0)
        dt = time.perf_counter() - t0
        n1 = self.link.stats.rx_by_type.get(0x02, 0)
        hz = (n1 - n0) / dt

        self.add("P1.2", "TEL_FAST 帧率 ~100 Hz", PASS if 90 <= hz <= 110 else FAIL,
                 "实测 %.1f Hz" % hz)
        crc = self.link.stats.crc_errors - c0
        self.add("P1.3", "CRC 零错", PASS if crc == 0 else FAIL,
                 "这 1 秒里 CRC 错 %d 次（累计 %d）" % (crc, self.link.stats.crc_errors))
        gaps = self.link.stats.cycle_gaps - g0
        self.add("P1.4", "cycle_cnt 连续（不丢周期）", PASS if gaps == 0 else FAIL,
                 "这 1 秒里跳变 %d 次（累计丢 %d 个周期）"
                 % (gaps, self.link.stats.cycle_lost))
        sf = self.link.stats.frame_timeouts
        self.add("P1.5", "无半帧超时", PASS if sf == 0 else FAIL,
                 "半帧超时 %d 次" % sf)

        # 回复耗时：收到帧 → 发出指令（对齐固件 4 ms 等待窗口）
        lat = S.Percentiles(capacity=2000)
        for _ in range(200):
            t = self.link.wait_tel_fast(timeout_ms=50)
            if t is None:
                continue
            self.keepalive()
            lat.add((time.perf_counter() - self.link.t_rx_tel_fast) * 1e6)
        if lat.n_total >= 50:
            self.add("P1.6", "请求-应答耗时 P99 < 4 ms",
                     PASS if lat.p99 < 4000 else FAIL,
                     "均值 %.0f  P95 %.0f  P99 %.0f  最大 %.0f us（n=%d）"
                     % (lat.mean, lat.p95, lat.p99, lat.maximum, lat.n_total))
        else:
            self.add("P1.6", "请求-应答耗时", SKIP, "样本不足（%d）" % lat.n_total)

        # 人工项
        self.add("P1.7", "【人工】用手掰某条腿 → 打印的 q 对应分量应跟着变", MANUAL)
        self.add("P1.8", "【人工】倾斜机身 → 重力向量应指向地面（用 --tilt / 面板看）", MANUAL)
        v = self.link.last_tel.voltage_mv if self.link.last_tel else 0
        self.add("P1.9", "【人工】万用表比对电压，误差 < 50 mV", MANUAL,
                 "当前 TEL_FAST 读数 %d mV（可用 status_panel.py 实时看）" % v)
        return True

    # ---------------------------------------------------------------- P2
    def phase_telemetry(self) -> None:
        self.phase("阶段 2 · 遥测")

        st = self.wait_status_alive(timeout_s=3.0)
        if st is None:
            self.add("P2.0", "能收到 TEL_STATUS", FAIL,
                     "3 秒内没收到 —— 固件只在每 100 ms 或 CMD_JOINT 带 "
                     "FLAG_REQ_STATUS 时发它")
            return
        tel = self.link.last_tel

        self.add("P2.1", "STAT_READY 置位（72 MHz 时钟 + DWT 自检）",
                 PASS if (tel.status & ST_READY) else FAIL,
                 "status = 0x%02X [%s]" % (tel.status, T.status_str(tel.status)))
        self.add("P2.2", "STAT_IMU_OK 置位",
                 PASS if (tel.status & ST_IMU_OK) else FAIL,
                 "未置位时 ACTIVE 过不了准入")
        self.add("P2.3", "STAT_LINK_OK 置位", PASS if (tel.status & ST_LINK_OK) else FAIL)

        temps = [T.temp_to_c(x, self.temp_unit) for x in st.temperature]
        sane = all(-20.0 <= x <= 120.0 for x in temps)
        self.add("P2.4", "12 路舵机温度都能读到且合理",
                 PASS if sane else FAIL,
                 " ".join("%.1f" % x for x in temps)
                 + "  (单位按 %s 解释)" % self.temp_unit)

        lvl, lvl_name = T.battery_level(tel.voltage_mv)
        self.add("P2.5", "电池读数有效且不低于警告线",
                 PASS if lvl == 0 else FAIL,
                 "TEL_FAST 电压 %d mV → %s" % (tel.voltage_mv, lvl_name))
        if st.voltage_mv == 0:
            self.note("TEL_STATUS.voltage_mv = 0 —— 这是固件写死的（真电压只在 TEL_FAST 里），"
                      "已在本层按预期处理")
        if st.temperature and 0 < st.temperature[0] < 120:
            self.note("TEL_STATUS 温度看起来是【原始 °C】而不是协议写的 0.1 °C"
                      "（用手捂一个舵机可以一次分辨：变几度=原始，变几十=0.1°C）")

        self.add("P2.6", "err_flags 为 0",
                 PASS if tel.err_flags == 0 else FAIL,
                 "0x%02X（%s）—— ⚠️ 这是 latch，只有重启才清；"
                 "且非 0 时安全层会把动作幅度压到 75%%"
                 % (tel.err_flags, T.err_str(tel.err_flags)) if tel.err_flags else "0")
        req = st.mode
        eff, why = T.derive_effective_mode(req, tel.status)
        self.add("P2.7", "模式状态自洽", PASS,
                 "请求 %s / 生效 %s%s"
                 % (T.mode_name(req), T.mode_name(eff), ("  ← " + why) if why else ""))
        self.add("P2.8", "【人工】mode / status 位与实际状态一致", MANUAL)

    # ---------------------------------------------------------------- P3
    def phase_calibration(self) -> None:
        self.phase("阶段 3 · 标定")
        tel = self.link.last_tel
        cal = bool(tel.status & ST_CALIBRATED) if tel else False

        if cal:
            self.add("P3.1", "STAT_CALIBRATED 已置位", PASS)
        elif self.do_calibrate and self.allow_move:
            self.add("P3.1", "STAT_CALIBRATED 已置位", SKIP, "下面会跑一次标定")
            print("\n  ▶ 启动标定（约 8.4 s，12 个关节各动约 0.15 rad）...")
            sess = CAL.CalibSession(self.link, quiet=True)
            ok, why = sess.start()
            self.add("P3.2", "CMD_CALIB(START) 收到 ACK", PASS if ok else FAIL, why)
            if ok:
                was = bool(self.link.last_tel.status & ST_CALIBRATED) if self.link.last_tel else False
                res = sess.wait_done(timeout_s=25.0, was_calibrated=was)
                self.add("P3.3", "标定完成（STAT_CALIBRATED 0→1）",
                         PASS if res.ok else FAIL, str(res))
                self.cal_ok = bool(self.link.last_tel.status & ST_CALIBRATED) \
                    if self.link.last_tel else False
        else:
            self.add("P3.1", "STAT_CALIBRATED 已置位", FAIL,
                     "未标定 → ACTIVE 会被静默降级成 STAND，关节不会跟踪目标。"
                     "跑 calib.py --start（机器人须吊起），或加 --calibrate --move")

        if not self.cal_ok:
            self.add("P3.4", "未标定时【不发】STAND/ACTIVE（安全规则）", PASS,
                     "已按规则只发 MODE_IDLE。⚠️ 固件的 STAND 无准入门，未标定时 "
                     "q_target=0 会换算成舵机 step 0，把 12 个关节驱到行程一端")
        self.add("P3.5", "【人工】标定后 q=0 时机器人呈站立姿态", MANUAL,
                 "用 calib.py --check 或 pose.py --pose zero 观察")
        self.add("P3.6", "【人工】断电重启后 STAT_CALIBRATED 仍为 1（offset 已入 Flash）",
                 MANUAL)

    # ---------------------------------------------------------------- P4
    def phase_jog(self) -> None:
        self.phase("阶段 4 · 单关节点动")
        if not self.allow_move:
            self.add("P4.0", "12 个关节逐个点动", SKIP, "需要 --move（会真的驱动舵机）")
            return
        if not self.cal_ok:
            self.add("P4.0", "12 个关节逐个点动", SKIP,
                     "未标定 → ACTIVE 会降级成 STAND，关节不会动")
            return

        state = JOG.JogState()
        runner = JOG.JogRunner(self.link, state)

        # ⭐ estop_seen 只统计【点动期间】的 ESTOP。
        #    之前没重置，结果把"还没开始发帧时的上电 ESTOP"也算进来了。
        self.estop_seen = False

        # 先回到全 0，给一个确定的起点
        state.home()
        self.settle(runner, tol_deg=1.5, timeout_s=2.5)

        bad = []
        for i in range(C.N_JOINTS):
            state.home()
            self.settle(runner, tol_deg=1.5, timeout_s=1.5)

            state.sel = i
            state.set_deg(i, self.JOG_DEG)
            # ⚠️ 先等它【真的收敛】（1.0°），再按宽松判据判定 ——
            #    否则验收会变成"擦边过"（在 2.5° 那一刻就返回）。
            reached, worst = self.settle(runner, tol_deg=1.0, timeout_s=3.0)

            tel = self.link.last_tel
            got = tel.q[i] * RAD2DEG if tel else float("nan")
            others = [abs(tel.q[j] - state.q[j]) * RAD2DEG
                      for j in range(C.N_JOINTS) if j != i] if tel else [99.9]
            drift = max(others) if others else 0.0

            reached_ok = (worst <= self.JOG_TOL_DEG)
            good = reached_ok and abs(got - self.JOG_DEG) <= self.JOG_TOL_DEG \
                and drift <= self.JOG_OTHER_MAX_DEG
            if not good:
                bad.append(i)
            self.add("P4.%02d" % (i + 1),
                     "关节 %d %s 能动到 +%.0f° 且【只有它动】" % (i, C.JOINT_NAMES[i],
                                                              self.JOG_DEG),
                     PASS if good else FAIL,
                     "实测 %+.2f°（误差 %.2f°，限 %.1f°）；其余关节最大漂移 %.2f°；%s"
                     % (got, abs(got - self.JOG_DEG), self.JOG_TOL_DEG, drift,
                        "已收敛到 1.0° 内" if reached else "未收敛到 1.0° 内"))

        # 限位截断
        n_before = state.clamp_events
        msg = state.set_deg(4, 200.0)
        self.add("P4.13", "超限目标被截断（Pi 侧软限位）",
                 PASS if state.clamp_events == n_before + 1 else FAIL, msg)
        for _ in range(20):
            runner.tick()
            time.sleep(0.01)
        tel = self.link.last_tel
        self.add("P4.14", "固件侧也没让关节越过硬件限位",
                 PASS if tel and tel.q[4] <= C.Q_MAX[4] + 1e-6 else FAIL,
                 "q4=%.4f  硬件上限=%.4f" % (tel.q[4] if tel else float("nan"), C.Q_MAX[4]))

        # 收尾：回零
        state.home()
        self.settle(runner, tol_deg=2.0, timeout_s=2.0)
        self.add("P4.15", "点动全程链路保持活跃（没有 ESTOP）",
                 PASS if not self.estop_seen else FAIL,
                 "estop_seen=%s" % self.estop_seen)
        if bad:
            self.note("有 %d 个关节没通过：%s" % (len(bad), bad))

    # ---------------------------------------------------------------- P5
    def phase_pose(self) -> None:
        self.phase("阶段 5 · 全身姿态")
        if not self.allow_move:
            self.add("P5.0", "摆站姿 / 缓慢变姿态", SKIP, "需要 --move")
            return
        if not self.cal_ok:
            self.add("P5.0", "摆站姿 / 缓慢变姿态", SKIP, "未标定")
            return

        runner = POSE.PoseRunner(self.link)

        # 摆站姿（全 0）
        ok, why = runner.goto(list(C.Q_DEFAULT), rate_deg_s=20.0,
                              timeout_s=6.0, tol_deg=2.0, label="站姿")
        self.add("P5.1", "能摆出站立姿态并收敛", PASS if ok else FAIL, why)

        # 保持 2 秒，看有没有下垂
        target = list(C.Q_DEFAULT)
        t_end = time.perf_counter() + 2.0
        while time.perf_counter() < t_end:
            runner._tick(target)
            time.sleep(0.002)
        tel = self.link.last_tel
        err = max(abs(tel.q[i] - target[i]) for i in range(C.N_JOINTS)) * RAD2DEG \
            if tel else 99.9
        self.add("P5.2", "保持 2 s 后残差 < 3°（没有明显下垂）",
                 PASS if err < 3.0 else FAIL, "残差 %.2f°" % err)

        # 缓慢来回（演示"缓慢改变姿态"）
        demo = list(C.Q_DEFAULT)
        for i in range(4, 8):
            demo[i] = 12.0 * DEG2RAD
        ok1, why1 = runner.goto(demo, rate_deg_s=12.0, timeout_s=6.0,
                                tol_deg=2.0, label="下蹲一档")
        ok2, why2 = runner.goto(list(C.Q_DEFAULT), rate_deg_s=12.0, timeout_s=6.0,
                                tol_deg=2.0, label="回站姿")
        self.add("P5.3", "能缓慢改变姿态并回到站姿（限速 12°/s）",
                 PASS if (ok1 and ok2) else FAIL,
                 (why1 + "；" + why2)[:160])

        self.add("P5.4", "【人工】变姿态过程中不失稳（腿不打架、机身不晃到失稳）", MANUAL)

    # ---------------------------------------------------------------- 汇总
    def summary(self) -> dict:
        n = {PASS: 0, FAIL: 0, SKIP: 0, MANUAL: 0}
        for c in self.checks:
            n[c.status] = n.get(c.status, 0) + 1
        return n


# ============================================================================

def main() -> int:
    ap = argparse.ArgumentParser(description="RL-CAT 阶段 1~5 一键验收")
    ap.add_argument("--port", default=C.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=C.SERIAL_BAUD)
    ap.add_argument("--move", action="store_true",
                    help="包含动作项（点动 / 姿态）—— ⚠️ 会真的驱动舵机，必须先吊起")
    ap.add_argument("--calibrate", action="store_true",
                    help="未标定时顺便跑一次标定（需 --move）")
    ap.add_argument("--yes", action="store_true", help="跳过交互确认")
    ap.add_argument("--temp-unit", choices=T.TEMP_UNITS, default="raw")
    ap.add_argument("--json", default=None, help="把报告写到这个 JSON 文件")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    try:
        link = PiLink(args.port, args.baud, verbose=False)
        link.open()
    except LinkError as e:
        print("!! %s" % e)
        return 2

    t_start = time.perf_counter()
    now = datetime.datetime.now()
    acc = Acceptance(link, allow_move=args.move, assume_yes=args.yes,
                     do_calibrate=args.calibrate, temp_unit=args.temp_unit,
                     verbose=not args.quiet)

    print("=" * W)
    print(" RL-CAT 树莓派侧 阶段 1~5 验收")
    print(" 端口 %s @ %d" % (args.port, args.baud))
    print(" 时间 %s" % now.strftime("%Y-%m-%d %H:%M:%S"))
    print(" 模式 %s" % ("含动作项（会驱动舵机）" if args.move else "只读（不动舵机）"))
    print("=" * W)
    if args.move:
        print("⚠️⚠️ 动作项会真的驱动 12 个舵机。请确认机器人已【吊起或垫高】。")

    rc = 1
    try:
        if args.move and not args.yes:
            try:
                r = input("确认开始（含动作项）？输入 yes 继续：").strip().lower()
            except EOFError:
                r = ""
            if r not in ("y", "yes"):
                print("已取消")
                link.close()
                return 1

        if not acc.phase_link():
            pass
        else:
            acc.phase_telemetry()
            acc.phase_calibration()
            acc.phase_jog()
            acc.phase_pose()

        n = acc.summary()
        print("\n" + "=" * W)
        print(" 通过 %d   失败 %d   跳过 %d   人工待确认 %d   用时 %.1f s"
              % (n[PASS], n[FAIL], n[SKIP], n[MANUAL], time.perf_counter() - t_start))
        if n[FAIL]:
            print("\n 失败项：")
            for c in acc.checks:
                if c.status == FAIL:
                    print("   · %s %s  [%s]" % (c.cid, c.title, c.detail))
        if n[MANUAL]:
            print("\n 需要人工确认的项：")
            for c in acc.checks:
                if c.status == MANUAL:
                    print("   · %s %s" % (c.cid, c.title))
        if not args.move:
            print("\n 提示：动作项（阶段 4/5）本次被跳过。加 --move --yes 才会跑。")
        print(" 链路统计：%d 帧  CRC 错 %d  丢 %d 周期"
              % (link.stats.rx_frames, link.stats.crc_errors, link.stats.cycle_lost))
        print("=" * W)
        rc = 0 if n[FAIL] == 0 else 1

        if args.json:
            report = {
                "time": now.isoformat(timespec="seconds"),
                "port": args.port,
                "baud": args.baud,
                "mode": "move" if args.move else "readonly",
                "summary": acc.summary(),
                "checks": [c.as_dict() for c in acc.checks],
                "notes": acc.notes,
                "link_stats": {
                    "rx_frames": link.stats.rx_frames,
                    "crc_errors": link.stats.crc_errors,
                    "cycle_lost": link.stats.cycle_lost,
                    "frame_timeouts": link.stats.frame_timeouts,
                },
                "duration_s": round(time.perf_counter() - t_start, 2),
            }
            try:
                with open(args.json, "w", encoding="utf-8") as f:
                    json.dump(report, f, ensure_ascii=False, indent=2)
                print(" 报告已写入 %s" % args.json)
            except OSError as e:
                print(" !! 写 JSON 报告失败：%s" % e)

    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
        rc = 1
    finally:
        try:
            if link.is_open:
                # ⚠️ 与 keepalive 同一条规则：未标定不发 STAND
                if acc.cal_ok:
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
