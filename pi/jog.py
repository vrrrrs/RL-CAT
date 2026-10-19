#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 树莓派侧：阶段 4 单关节点动

一次只动一个关节，其余保持在 q_default(=0 = 站姿)。用来验证：
    □ 方向正确（正角度让关节往预期方向动）
    □ 幅度正确（给 10° 就动约 10°）
    □ 限位生效（超限被拒绝或截断）
    □ 12 个关节逐个都能动

⚠️⚠️ 必须先把机器人【吊起或垫高】，腿悬空。这是第一次真正驱动舵机。

⚠️⚠️ 交互式等键盘输入时，主循环仍然在 100 Hz 发帧。
   这不是多此一举：固件的失联判定基于"距上次收到任意帧"，一旦超过 100 ms
   没收到帧，安全层就会开始趴下斜坡。所以键盘读在独立线程，主循环只从队列取。

⚠️ 关节必须已标定。未标定时固件会把 ACTIVE 静默降级成 STAND，关节根本不动 ——
   那种"发了指令没反应"的情况本脚本会提前拦住并说清楚。

用法：
    python3 jog.py                                  # 交互式（吊起后）
    python3 jog.py --sequence "4:+10;4:+10;4:-20;all:0" --dwell 1.2
    python3 jog.py --port /dev/pts/0 --list         # 看关节表

交互命令：
    j <0-11>      选关节（也可直接敲关节名，如 lf_thigh）
    + / -         按当前步长加/减
    <数字>        直接给绝对角度（度）
    s <度>        改步长
    list          打印 12 个关节与当前目标角
    home          全部回到 q_default
    q             退出（会先发 MODE_IDLE 卸力）

退出码：0 = 正常结束；1 = 有告警/拒绝；2 = 打不开串口
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
import math
import os
import queue
import sys
import threading
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
    ST_CALIBRATED,
    ST_ESTOP,
    ST_IMU_OK,
    ST_LINK_OK,
    ST_READY,
)

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0


# ============================================================================
# 状态（与链路解耦，方便离线单测）
# ============================================================================

class JogState:
    """点动状态：12 维目标角 + 当前选中的关节 + 步长。全部用【逻辑索引 A】。"""

    def __init__(self, q=None, sel: int = 4, step_deg: float = 5.0):
        self.q = list(q) if q is not None else list(C.Q_DEFAULT)
        if len(self.q) != C.N_JOINTS:
            raise ValueError("q 必须是 %d 维" % C.N_JOINTS)
        self.sel = sel
        self.step_deg = step_deg
        self.clamp_events = 0          # 被限位截断过几次（验收要看这个）
        self.last_note = ""

    # ---------------------------------------------------------------- 操作
    def clamp_index(self, i: int) -> float:
        """把 self.q[i] 夹进软限位，返回被夹掉多少（rad）。"""
        lo, hi = C.Q_SOFT_MIN[i], C.Q_SOFT_MAX[i]
        v = self.q[i]
        if v != v:                                   # NaN
            self.q[i] = C.Q_DEFAULT[i]
            return 0.0
        if v < lo:
            self.q[i] = lo
            return v - lo
        if v > hi:
            self.q[i] = hi
            return v - hi
        return 0.0

    def select(self, i: int) -> str:
        if not (0 <= i < C.N_JOINTS):
            return "关节索引必须在 0..11"
        self.sel = i
        return "选中 %d %s（当前目标 %+.1f°）" % (
            i, C.JOINT_NAMES[i], self.q[i] * RAD2DEG)

    def select_by_name(self, name: str) -> str:
        try:
            return self.select(C.JOINT_NAMES.index(name))
        except ValueError:
            return "没有叫 %s 的关节" % name

    def nudge(self, delta_deg: float) -> str:
        want = self.q[self.sel] + delta_deg * DEG2RAD
        self.q[self.sel] = want
        over = self.clamp_index(self.sel)
        if over:
            self.clamp_events += 1
            return ("⚠️ %s 目标 %.1f° 超出软限位，已截断到 %.1f°（限位 %.1f°~%.1f°）"
                    % (C.JOINT_NAMES[self.sel], want * RAD2DEG,
                       self.q[self.sel] * RAD2DEG,
                       C.Q_SOFT_MIN[self.sel] * RAD2DEG,
                       C.Q_SOFT_MAX[self.sel] * RAD2DEG))
        return "%s → %+.2f°" % (C.JOINT_NAMES[self.sel], self.q[self.sel] * RAD2DEG)

    def set_deg(self, i: int, deg: float) -> str:
        if not (0 <= i < C.N_JOINTS):
            return "关节索引必须在 0..11"
        self.sel = i
        self.q[i] = deg * DEG2RAD
        over = self.clamp_index(i)
        if over:
            self.clamp_events += 1
            return ("⚠️ %s 目标 %.1f° 超出软限位，已截断到 %.1f°"
                    % (C.JOINT_NAMES[i], deg, self.q[i] * RAD2DEG))
        return "%s → %+.2f°" % (C.JOINT_NAMES[i], deg)

    def home(self) -> str:
        self.q = list(C.Q_DEFAULT)
        return "全部回到 q_default(0°)"

    def as_deg(self) -> list:
        return [v * RAD2DEG for v in self.q]


# ============================================================================
# 运行（100 Hz 发帧 + 非阻塞取用户输入）
# ============================================================================

class JogRunner:
    """负责"把目标角按 100 Hz 发出去，同时保持链路活跃"。"""

    def __init__(self, link: PiLink, state: JogState, *, hz: float = 100.0):
        self.link = link
        self.state = state
        self.period = 1.0 / hz
        self._t_next = 0.0
        self.sent = 0

    def tick(self) -> None:
        """到点就发一帧；每次调用都顺带收一下。永不阻塞。"""
        now = time.perf_counter()
        if now >= self._t_next:
            self._t_next = now + self.period
            # ⭐ 必须带 FLAG_ENABLE —— 否则固件的 s_armed 永远为 false，
            #    力矩被强制关掉、位置一条都不写，点动完全没反应。
            #    详见 pose.py 里 _tick() 的说明。
            if self.link.send_cmd_joint(self.state.q, mode=MODE_ACTIVE,
                                        flags=FLAG_ENABLE):
                self.sent += 1
        self.link.pump()

    def run_for(self, seconds: float) -> None:
        t_end = time.perf_counter() + seconds
        while time.perf_counter() < t_end:
            self.tick()
            time.sleep(0.002)

    def wait_settled(self, tol_deg: float = 1.5, timeout_s: float = 5.0) -> tuple:
        """等实际关节角收敛到目标。返回 (是否收敛, 最大误差度)。"""
        t_end = time.perf_counter() + timeout_s
        worst = 1e9
        while time.perf_counter() < t_end:
            self.tick()
            tel = self.link.last_tel
            if tel is not None:
                worst = max(abs(tel.q[i] - self.state.q[i]) for i in range(C.N_JOINTS))
                if worst * RAD2DEG <= tol_deg:
                    return True, worst * RAD2DEG
            time.sleep(0.002)
        return False, (worst * RAD2DEG if worst < 1e8 else float("inf"))


def precheck(link: PiLink) -> tuple:
    """返回 (能否动关节, 说明列表)。"""
    notes = []
    tel = link.wait_tel_fast(timeout_ms=2000)
    if tel is None:
        return False, ["2 秒内没收到 TEL_FAST —— 链路不通"]

    fatal = False
    if not (tel.status & ST_CALIBRATED):
        notes.append("❌ STAT_CALIBRATED 未置位 —— 固件会把 ACTIVE 静默降级成 "
                     "STAND，关节【根本不会动】。先跑 calib.py --start")
        fatal = True
    if not (tel.status & ST_READY):
        notes.append("⚠️ STAT_READY 未置位（时钟/DWT 自检没过）")
    if not (tel.status & ST_IMU_OK):
        notes.append("⚠️ STAT_IMU_OK 未置位（不影响点动，但之后 ACTIVE 过不了准入）")
    if tel.err_flags:
        notes.append("⚠️ err_flags=0x%02X（%s）—— latch；非 0 时安全层把幅度压到 75%%"
                     % (tel.err_flags, T.err_str(tel.err_flags)))
    return (not fatal), notes


def print_joint_table(state: JogState) -> None:
    print("  索引  关节名      类型    软限位(度)        当前目标")
    for i in range(C.N_JOINTS):
        mark = "→" if i == state.sel else " "
        print("  %s %2d   %-10s  %-6s  %6.1f ~ %+6.1f   %+7.2f°"
              % (mark, i, C.JOINT_NAMES[i], C.JOINT_KIND[i],
                 C.Q_SOFT_MIN[i] * RAD2DEG, C.Q_SOFT_MAX[i] * RAD2DEG,
                 state.q[i] * RAD2DEG))


def handle_command(state: JogState, line: str) -> tuple:
    """处理一条交互命令。返回 (是否继续, 回复文本)。"""
    s = line.strip()
    if not s:
        return True, ""
    low = s.lower()

    if low in ("q", "quit", "exit"):
        return False, "退出"
    if low in ("?", "h", "help"):
        return True, ("命令：j <0-11> | <关节名> | + | - | <绝对角度> | s <步长> | "
                      "list | home | q")
    if low == "list":
        print_joint_table(state)
        return True, ""
    if low == "home":
        return True, state.home()
    if low.startswith("j "):
        try:
            return True, state.select(int(low[2:].strip()))
        except ValueError:
            return True, "用法：j <0-11>"
    if low.startswith("s "):
        try:
            v = float(low[2:].strip())
            if v <= 0:
                return True, "步长必须为正"
            state.step_deg = v
            return True, "步长 = %.2f°" % v
        except ValueError:
            return True, "用法：s <度>"
    if low == "+":
        return True, state.nudge(+state.step_deg)
    if low == "-":
        return True, state.nudge(-state.step_deg)

    # 关节名
    if low in C.JOINT_NAMES:
        return True, state.select_by_name(low)

    # 纯数字 → 绝对角度（给当前选中的关节）
    try:
        v = float(low)
        return True, state.set_deg(state.sel, v)
    except ValueError:
        pass

    # "关节名 角度" / "索引 角度"
    parts = low.split()
    if len(parts) == 2:
        try:
            deg = float(parts[1])
            if parts[0] in C.JOINT_NAMES:
                return True, state.set_deg(C.JOINT_NAMES.index(parts[0]), deg)
            return True, state.set_deg(int(parts[0]), deg)
        except (ValueError, IndexError):
            pass

    return True, "看不懂：%r（敲 ? 看帮助）" % s


def run_interactive(link: PiLink, state: JogState, runner: JogRunner,
                    print_hz: float = 2.0) -> int:
    inbox: queue.Queue = queue.Queue()
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            try:
                line = sys.stdin.readline()
            except Exception:
                return
            if not line:
                return
            inbox.put(line)

    th = threading.Thread(target=reader, daemon=True)
    th.start()

    print("\n交互模式。敲 ? 看命令，q 退出。")
    print("⚠️ 机器人必须已吊起。")
    t_next_print = 0.0
    rc = 0

    try:
        while not stop.is_set():
            # 1) 非阻塞取用户输入（主循环绝不被键盘阻塞）
            while True:
                try:
                    line = inbox.get_nowait()
                except queue.Empty:
                    break
                cont, reply = handle_command(state, line)
                if reply:
                    print("  " + reply)
                if not cont:
                    stop.set()

            # 2) 100 Hz 发帧（保持固件不失联）
            runner.tick()

            # 3) 定期打印实际关节角
            now = time.perf_counter()
            if now >= t_next_print and not stop.is_set():
                t_next_print = now + 1.0 / print_hz
                tel = link.last_tel
                if tel is None:
                    print("  (收不到 TEL_FAST)")
                else:
                    eff = T.derive_effective_mode(MODE_ACTIVE, tel.status)[0]
                    tag = "" if eff == MODE_ACTIVE else ("  ⚠️ 生效模式=%s（没在跟踪！）"
                                                         % T.mode_name(eff))
                    print("  目标 %+7.2f°  实际 %+7.2f°  (%s)  |q|max=%.3f%s"
                          % (state.q[state.sel] * RAD2DEG,
                             tel.q[state.sel] * RAD2DEG,
                             C.JOINT_NAMES[state.sel],
                             max(abs(v) for v in tel.q), tag))
            time.sleep(0.002)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
    finally:
        stop.set()
        print("\n卸力并退出 ...")
        link.send_cmd_mode(MODE_IDLE)
        time.sleep(0.2)
    return rc


def run_sequence(link: PiLink, state: JogState, runner: JogRunner,
                 sequence: str, dwell: float, tol_deg: float) -> int:
    """脚本模式。

    语法：`关节:增量度` 用分号隔开。
        · 关节可以是逻辑索引 0..11，也可以是关节名（lf_thigh）
        · `all:0` 表示全部回零（回到 q_default）
    例：`--sequence "4:+10;4:+10;4:-20;all:0"`
    """
    steps = [s.strip() for s in sequence.split(";") if s.strip()]
    print("\n脚本模式：%d 步，每步最多等 %.2f s，收敛判据 %.2f°"
          % (len(steps), dwell, tol_deg))
    rc = 0

    for k, step in enumerate(steps):
        if ":" not in step:
            print("  第 %d 步 %r 格式不对，应为 '关节:增量度'" % (k + 1, step))
            rc = 1
            continue
        who, val_s = step.split(":", 1)
        who = who.strip().lower()
        try:
            val = float(val_s)
        except ValueError:
            print("  第 %d 步角度 %r 不是数字" % (k + 1, val_s))
            rc = 1
            continue

        if who == "all":
            if abs(val) > 1e-9:
                print("  第 %d 步：all 只支持 all:0（全部回零）" % (k + 1))
                rc = 1
                continue
            msg = state.home()
        else:
            if who in C.JOINT_NAMES:
                idx = C.JOINT_NAMES.index(who)
            else:
                try:
                    idx = int(who)
                except ValueError:
                    print("  第 %d 步：认不出关节 %r" % (k + 1, who))
                    rc = 1
                    continue
            if not (0 <= idx < C.N_JOINTS):
                print("  第 %d 步：关节索引 %d 越界" % (k + 1, idx))
                rc = 1
                continue
            state.sel = idx
            msg = state.nudge(val)

        print("  [%d/%d] %s" % (k + 1, len(steps), msg))

        ok, worst = runner.wait_settled(tol_deg=tol_deg, timeout_s=dwell)
        tel = link.last_tel
        if tel is None:
            print("         ❌ 收不到 TEL_FAST")
            rc = 1
            continue
        eff = T.derive_effective_mode(MODE_ACTIVE, tel.status)[0]
        if eff != MODE_ACTIVE:
            print("         ⚠️ 生效模式=%s —— 关节没有在跟踪目标！"
                  % T.mode_name(eff))
            rc = 1
        print("         收敛 %s  最大误差 %.2f°  实际 %+7.2f°"
              % ("✅" if ok else "❌未收敛", worst, tel.q[state.sel] * RAD2DEG))
        if not ok:
            rc = 1

    print("\n结果：限位截断发生 %d 次" % state.clamp_events)
    return rc


def main() -> int:
    ap = argparse.ArgumentParser(description="RL-CAT 阶段 4：单关节点动")
    ap.add_argument("--port", default=C.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=C.SERIAL_BAUD)
    ap.add_argument("--sequence", default=None,
                    help='脚本模式，例如 "4:+10;4:+10;4:-20;all:0"')
    ap.add_argument("--dwell", type=float, default=1.5,
                    help="脚本模式每步最长等待秒数")
    ap.add_argument("--tol", type=float, default=1.5, help="收敛判据（度）")
    ap.add_argument("--step", type=float, default=5.0, help="交互步长（度）")
    ap.add_argument("--joint", type=int, default=4, help="初始选中的关节")
    ap.add_argument("--list", action="store_true", help="打印关节表就退出")
    ap.add_argument("--yes", action="store_true", help="跳过交互确认")
    args = ap.parse_args()

    state = JogState(sel=args.joint, step_deg=args.step)
    if args.list:
        print_joint_table(state)
        return 0

    try:
        link = PiLink(args.port, args.baud, verbose=False)
        link.open()
    except LinkError as e:
        print("!! %s" % e)
        return 2

    rc = 0
    try:
        print("=" * 78)
        print(" 阶段 4：单关节点动   %s @ %d" % (args.port, args.baud))
        print("=" * 78)
        print("⚠️⚠️ 请确认机器人已【吊起或垫高】，腿部悬空、周围无人。")

        ok, notes = precheck(link)
        for n in notes:
            print("  " + n)
        if not ok:
            return 1

        if not args.yes and args.sequence is None:
            try:
                r = input("确认开始？输入 yes 继续：").strip().lower()
            except EOFError:
                r = ""
            if r not in ("y", "yes"):
                print("已取消")
                return 1

        runner = JogRunner(link, state)
        if args.sequence:
            rc = run_sequence(link, state, runner, args.sequence, args.dwell, args.tol)
        else:
            rc = run_interactive(link, state, runner)
    finally:
        try:
            if link.is_open:
                link.send_cmd_mode(MODE_IDLE)
                time.sleep(0.2)
        except Exception:
            pass
        link.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
