#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 树莓派侧：阶段 2 遥测/状态面板

一个"看一眼就知道机器人现在什么状态"的工具：
    链路健康 / 模式（请求 vs 生效）/ 状态位 / 错误位 / 电池等级 /
    12 个舵机温度 / 总线计数 / 12 个关节角 / 步态相位

⚠️ 两个必须记住的坑（详见 pi/README.md §5）：
    · 电压必须从 TEL_FAST 取 —— TEL_STATUS.voltage_mv 固件写死为 0
    · TEL_STATUS.mode 是【请求的模式】；本面板会推算【生效模式】并标出降级原因

三种输出模式：
    panel  原地刷新（默认，仅当 stdout 是终端；否则自动退化成 log）
    log    每 N 秒一行紧凑日志（适合 nohup / 重定向到文件）
    once   打印一次完整快照就退出（适合脚本与验收；有告警时退出码 1）

用法：
    python3 status_panel.py                              # 面板
    python3 status_panel.py --mode log --interval 1      # 日志
    python3 status_panel.py --mode once                  # 快照（脚本用）
    python3 status_panel.py --port /dev/pts/0 --mode once --temp-unit raw
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
import os
import sys
import threading
import time
from typing import List

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import config as C                                      # noqa: E402
import telemetry as T                                   # noqa: E402
from link import PiLink, LinkError                      # noqa: E402
from rlcat_proto import (                               # noqa: E402
    MODE_ACTIVE,
    MODE_ESTOP,
    MODE_IDLE,
    ST_BATT_FORCE,
    ST_CALIBRATED,
    ST_ESTOP,
    ST_LINK_OK,
    TYPE_TEL_FAST,
    TYPE_TEL_STATUS,
)

W = 78


def render_panel(link: PiLink, tel, st, *, temp_unit: str, elapsed_s: float) -> List[str]:
    L: List[str] = []
    s = link.stats

    hz = link_panel_hz(link, elapsed_s)

    L.append("┌─ RL-CAT 遥测面板 " + "─" * (W - 22) + "┐")

    # 链路
    L.append("│ 链路  %-28s 帧率 %6.1f Hz   运行 %s"
             % ("%s @ %d" % (link.port, link.baud), hz, fmt_hms(elapsed_s)))
    L.append("│       收 %d 帧   CRC 错 %d   半帧超时 %d   丢 %d 周期   发出 %d 帧"
             % (s.rx_frames, s.crc_errors, s.frame_timeouts, s.cycle_lost, s.tx_frames))
    by = "  ".join("%s %d" % (s.type_name(t), n) for t, n in sorted(s.rx_by_type.items()))
    L.append("│       分类 %s" % (by if by else "(还没有帧)"))

    if tel is None:
        L.append("│")
        L.append("│ ⚠️ 还没有收到 TEL_FAST —— 检查接线 / 波特率 / 设备节点")
        L.append("└" + "─" * (W - 2) + "┘")
        return L

    # 模式
    req = st.mode if st is not None else -1
    eff, why = T.derive_effective_mode(req, tel.status)
    downgraded = (req == MODE_ACTIVE and eff != MODE_ACTIVE)
    L.append("│ 模式  请求 %-8s 生效 %-8s %s"
             % (T.mode_name(req), T.mode_name(eff),
                ("⚠️ 被降级：" + why) if downgraded else ""))

    # 状态位 / 错误位
    L.append("│ 状态  %s" % T.status_str(tel.status))
    if tel.err_flags:
        L.append("│ 错误  ⚠️ %s   ← ⚠️ 固件里这是 latch，只有重启才清；"
                 % T.err_str(tel.err_flags))
        L.append("│              且非 0 时安全层会把动作幅度压到 75%")
    else:
        L.append("│ 错误  无")

    # 电池（⚠️ 从 TEL_FAST 取）
    lvl, lvl_name = T.battery_level(tel.voltage_mv)
    mark = {0: "✅", 1: "⚠️", 2: "🟠", 3: "🔴", -1: "❓"}.get(lvl, "?")
    L.append("│ 电池  %s %5d mV  %s" % (mark, tel.voltage_mv, lvl_name))

    # 舵机温度
    if st is not None:
        unit_txt = "原始 °C（固件实际行为）" if temp_unit == "raw" else "0.1 °C（协议文档写的）"
        L.append("│ 温度  (%s)" % unit_txt)
        tc = [T.temp_to_c(v, temp_unit) for v in st.temperature]
        for r, kind in ((0, "hip  "), (1, "thigh"), (2, "calf ")):
            row = tc[r * 4:r * 4 + 4]
            cells = []
            for v in row:
                lv = T.temp_level(v)
                cells.append(("%5.1f%s" % (v, "!" if lv == 2 else ("?" if lv == 1 else " "))))
            L.append("│       %s  [%s]" % (kind, " ".join(cells)))

        # 总线计数
        L.append("│ 总线  rx_overflow %d   bus_err %d   bus_timeout %d   "
                 "seq_lost %d   uptime %.1f s"
                 % (st.rx_overflow, st.bus_err, st.bus_timeout, st.seq_lost,
                    st.uptime_ms / 1000.0))

    # 关节
    L.append("│ 关节  索引 A（髋外摆×4 → 髋俯仰×4 → 膝×4）")
    for r, kind in ((0, "hip  "), (4, "thigh"), (8, "calf ")):
        L.append("│       %s %s" % (kind, T.fmt_joints(tel.q, r, 4)))

    # 相位
    L.append("│ 相位  sin %+.3f  cos %+.3f   cycle_cnt %d"
             % (tel.gait_sin, tel.gait_cos, tel.cycle_cnt))

    # 告警汇总
    warns = health_warnings(link, tel, st, temp_unit)
    if warns:
        L.append("├" + "─" * (W - 2) + "┤")
        for w in warns:
            L.append("│ ⚠️  %s" % w)

    L.append("└" + "─" * (W - 2) + "┘")
    return L


def link_panel_hz(link: PiLink, elapsed_s: float) -> float:
    n = link.stats.rx_by_type.get(TYPE_TEL_FAST, 0)
    return n / elapsed_s if elapsed_s > 0 else 0.0


def fmt_hms(sec: float) -> str:
    sec = int(sec)
    return "%02d:%02d:%02d" % (sec // 3600, (sec % 3600) // 60, sec % 60)


def health_warnings(link: PiLink, tel, st, temp_unit: str) -> List[str]:
    w: List[str] = []
    s = link.stats
    if s.rx_frames == 0:
        w.append("一帧都没收到")
        return w
    if s.crc_errors:
        w.append("CRC 错 %d 次 —— 链路质量有问题" % s.crc_errors)
    if s.frame_timeouts:
        w.append("半帧超时 %d 次 —— 可能有字节丢失" % s.frame_timeouts)
    if s.cycle_gaps:
        w.append("cycle_cnt 跳变 %d 次（丢 %d 个周期）" % (s.cycle_gaps, s.cycle_lost))
    if s.unknown_type:
        w.append("未知消息类型 %d 个" % s.unknown_type)

    if tel is not None:
        lvl, name = T.battery_level(tel.voltage_mv)
        if lvl >= 2:
            w.append("电池 %d mV（%s）—— 别再跑了" % (tel.voltage_mv, name))
        elif lvl == 1:
            w.append("电池 %d mV（%s）" % (tel.voltage_mv, name))
        elif lvl == -1:
            w.append("电池读数无效（TEL_FAST.voltage_mv = 0）→ 固件 battery_is_valid() 为假")
        if tel.err_flags:
            w.append("err_flags=0x%02X（%s）—— 2 秒无新错误会自动清除；"
                     "持续非 0 时安全层会把动作幅度压到 75%%"
                     % (tel.err_flags, T.err_str(tel.err_flags)))
        if tel.status & ST_ESTOP:
            if link.stats.tx_frames == 0:
                # 只观察、一帧都没发过 → 固件按"失联"处理是必然的，这不算故障
                w.append("ℹ️ 处于 ESTOP —— 本工具只观察、没发过帧，固件自然把链路判为"
                         "失联（它的失联判定基于'距上次收到任意帧'）。加 --keepalive 可解除")
            else:
                # ⭐⭐ 直接说出原因（2026-10-17 加）
                #
                #  ⚠️ 固件的 safety_reasons 位域【不在 TEL_STATUS 里】，
                #     协议没有这个字段，所以 Pi 侧看不到原始位。
                #     但 STATUS 的几位足以把原因区分开：
                #
                #       ESTOP 只能由四种条件触发（rlca_safety.c）：
                #         ① link_level >= 2（>C.LINK_ESTOP_MS 无有效帧）→ STAT_LINK_OK 会熄灭
                #         ② 电池 ≤ 9.3 V                       → STAT_BATT_FORCE 置位
                #         ③ 舵机 > 70 °C                       → 温度看得到
                #         ④ operator_estop                     → 代码里恒 false，未实现
                #
                #  ⇒ 按优先级逐条排除，就能给出唯一答案，不用再连 shell 敲 R。
                cause = None
                if not (tel.status & ST_LINK_OK):
                    cause = ("链路失联：>%d ms 没收到有效帧"
                             "（STAT_LINK_OK 已熄灭）" % C.LINK_ESTOP_MS)
                elif tel.status & ST_BATT_FORCE:
                    cause = "电池电压 ≤ 9.3 V（STAT_BATT_FORCE）"
                elif st is not None:
                    tcmax = max(T.temp_to_c(v, temp_unit) for v in st.temperature)
                    if tcmax > C.TEMP_OVERHEAT_C:
                        cause = "舵机过温 %.0f°C > %d°C" % (tcmax, C.TEMP_OVERHEAT_C)
                w.append("⚠️ 处于 ESTOP —— 原因：%s"
                         % (cause if cause else
                            "不明（STAT 位都没有异常）→ 用 ESP32 无线口敲 R 看 safety_reasons"))
        if st is not None:
            req, eff = st.mode, T.derive_effective_mode(st.mode, tel.status)[0]
            if req == MODE_ACTIVE and eff != MODE_ACTIVE:
                w.append("ACTIVE 被静默降级成 %s —— 你以为在跑，其实只是站着"
                         % T.mode_name(eff))
            tc = [T.temp_to_c(v, temp_unit) for v in st.temperature]
            hot = [i for i, v in enumerate(tc) if T.temp_level(v) >= 2]
            if hot:
                w.append("舵机 %s 超温（>%d°C）—— 固件会强制趴下"
                         % (hot, C.TEMP_OVERHEAT_C))
            warm = [i for i, v in enumerate(tc) if T.temp_level(v) == 1]
            if warm:
                w.append("舵机 %s 温度偏高（>=%d°C）" % (warm, C.TEMP_WARN_C))
    return w


class _KeepAlive(threading.Thread):
    """⭐ 后台线程：固定周期发 CMD_JOINT(MODE_IDLE) 保活。

    ⚠️⚠️ 为什么必须独立线程（2026-10-17 实测踩到）：
       固件的链路判据是"距上次收到任意帧的间隔"：
           > 30 ms  → 降级（LINK_OK 熄灭，ACTIVE 进不去）
           > 100 ms → 急停（ESTOP，安全层把关节往趴下姿态推）
       而主循环里要做 render_panel() + print()，输出到 SSH 管道时
       一次可能阻塞几十毫秒。把保活塞在主循环里 ⇒
       一旦渲染慢了，保活就被饿死 ⇒ 链路断 ⇒ 机器人自己进 ESTOP。

       现场表现就是这个自相矛盾的组合：
           状态  READY|CALIBRATED|LINK_OK|IMU_OK|ESTOP
           ↑ LINK_OK 是亮的，却同时处于 ESTOP
       即：观察窗口那一刻链路是好的，但之前被渲染卡出过 >100 ms 的空档。

    ⚠️ 线程里只发不读（读仍在主线程的 link.pump()），避免两边抢串口读缓冲。

    ⚠️⚠️ 2026-10-17 第二次校正：周期从 20 ms（50 Hz）缩到 **10 ms（100 Hz）**。
       现场实测：50 Hz 运行时 `safety_reasons` 仍在 0 和 0x08(LINK_LOST) 之间跳，
       偶尔进 ESTOP。原因是**余量只有 10 ms**：
           保活周期 20 ms  vs  固件 LOST 阈值 30 ms
       而 Python 的单线程调度抖动（GIL 默认 5 ms 切换、GC、往 SSH 管道 print）
       经常超过 10 ms ⇒ 越过 30 ms 阈值 ⇒ LINK_LOST；
       偶发超过 80 ms ⇒ 越过 100 ms 阈值 ⇒ ESTOP。
       ⇒ 周期取 10 ms 后余量变成 20 ms，抖动吸收能力翻倍。
       带宽代价：38 B × 100 Hz = 3.8 kB/s，1 Mbps 的 3%，可忽略。
    """

    def __init__(self, link, period_s: float = 0.01) -> None:
        super().__init__(daemon=True, name="rlcat-keepalive")
        self._link = link
        self._period = period_s
        self._stop = threading.Event()

    def run(self) -> None:
        # ⚠️ Event.wait() 而不是 time.sleep()：退出时能立刻停下，
        #    不会在 Ctrl-C 之后又多撑一个周期。
        while not self._stop.wait(self._period):
            try:
                self._link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE, flags=0)
            except Exception:
                # ⚠️ 串口拔掉/写失败不该把面板打崩；下一轮继续尝试
                pass

    def stop(self) -> None:
        self._stop.set()


def main() -> int:
    ap = argparse.ArgumentParser(description="RL-CAT 阶段 2：遥测面板")
    ap.add_argument("--port", default=C.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=C.SERIAL_BAUD)
    ap.add_argument("--mode", choices=("auto", "panel", "log", "once"), default="auto")
    ap.add_argument("--rate", type=float, default=2.0, help="面板刷新 Hz")
    ap.add_argument("--interval", type=float, default=1.0, help="log 模式间隔 秒")
    ap.add_argument("--seconds", type=float, default=0.0, help="跑多久（0=一直）")
    ap.add_argument("--temp-unit", choices=T.TEMP_UNITS, default="raw",
                    help="舵机温度单位；raw = 固件实际行为（默认）")
    ap.add_argument("--keepalive", action="store_true",
                    help="周期性发 MODE_IDLE，避免固件因'没收到帧'而进 ESTOP")
    args = ap.parse_args()

    mode = args.mode
    if mode == "auto":
        mode = "panel" if sys.stdout.isatty() else "log"

    try:
        link = PiLink(args.port, args.baud, verbose=False)
        link.open()
    except LinkError as e:
        print("!! %s" % e)
        return 2

    t0 = time.perf_counter()

    # ---------------------------------------------------------------- once
    if mode == "once":
        try:
            if args.keepalive:
                for _ in range(12):               # 先发几帧把 ESTOP 解掉
                    link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE, flags=0)
                    link.pump()
                    time.sleep(0.01)
            tel = link.wait_tel_fast(timeout_ms=5000)
            st = link.wait_status(timeout_ms=2000) if tel is not None else None
            elapsed = max(time.perf_counter() - t0, 1e-9)
            print("\n".join(render_panel(link, tel, st,
                                         temp_unit=args.temp_unit,
                                         elapsed_s=elapsed)))
            print()
            if tel is None:
                print("!! 5 秒内没收到 TEL_FAST —— 检查接线 / 波特率 / 设备节点")
                return 1
            warns = health_warnings(link, tel, st, args.temp_unit)
            hard = [w for w in warns if not w.startswith("ℹ️")]
            for w in warns:
                print(w if w.startswith("ℹ️") else "⚠️  " + w)
            if not hard:
                print("✅ 无告警")
            return 1 if hard else 0
        finally:
            link.close()

    # ------------------------------------------------------------ panel/log
    t_end = t0 + args.seconds if args.seconds > 0 else float("inf")
    t_next = 0.0
    last_warn_key = None

    # ⭐ 保活放到后台线程（见 _KeepAlive 的说明）。
    #  ⚠️ 原来在主循环里 `t_next_keep = now + 0.02`，一旦 render/print 阻塞
    #     就会饿死保活 ⇒ 固件判失联 ⇒ 急停。观察工具不能自己制造故障。
    ka = None
    if args.keepalive:
        # 先补几帧把可能已经存在的 ESTOP 解掉，再交给线程维持
        for _ in range(12):
            link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE, flags=0)
            link.pump()
            time.sleep(0.01)
        ka = _KeepAlive(link)
        ka.start()

    try:
        while time.perf_counter() < t_end:
            link.pump()
            now = time.perf_counter()
            if now < t_next:
                time.sleep(0.002)
                continue
            t_next = now + (1.0 / args.rate if mode == "panel" else args.interval)

            tel = link.last_tel
            st = link.last_status
            elapsed = now - t0

            if mode == "panel":
                lines = render_panel(link, tel, st, temp_unit=args.temp_unit,
                                     elapsed_s=elapsed)
                sys.stdout.write("\033[H\033[J")      # home + clear
                sys.stdout.write("\n".join(lines) + "\n")
                sys.stdout.flush()
            else:   # log
                if tel is None:
                    print("[%7.2fs] 还没有 TEL_FAST (alive=%s)"
                          % (elapsed, link.is_alive()))
                else:
                    print("[%7.2fs] %s"
                          % (elapsed, T.one_line_summary(tel, st, args.temp_unit)))
                warns = health_warnings(link, tel, st, args.temp_unit)
                key = tuple(warns)
                if key != last_warn_key:          # 只在告警变化时打，避免刷屏
                    last_warn_key = key
                    for w in warns:
                        print("           %s"
                              % (w if w.startswith("ℹ️") else "⚠️  " + w))
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
    finally:
        # ⚠️ 先停保活线程，再关串口 —— 否则线程可能在 close() 之后还写一次
        if ka is not None:
            ka.stop()
            ka.join(timeout=1.0)
        link.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
