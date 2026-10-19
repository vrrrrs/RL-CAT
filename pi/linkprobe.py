#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 链路探针：判定"链路空档"是【树莓派发帧晚】还是【STM32 卡住】

⚠️⚠️ 为什么要这个工具：
    现场现象是 ESTOP 每几秒跳一次，根因是"距上次收到有效帧 > 阈值"。
    但那条判据有两个完全不同的成因：
      ① 树莓派侧没能按时发帧（GIL / GC / 打印 / 非 RT 调度）
      ② STM32 侧卡住了，收不到/解析不了帧（长阻塞、Flash 擦写、shell 打印）
    只敲 shell 的 R 看到的是【结果】（link_max_gap_ms），分不出是①还是②。

本工具用一条纯软件的判据把两者分开：
    · 树莓派自己记录【每次发帧的间隔】          → 描述①
    · 树莓派从 TEL_FAST 读【STM32 的 cycle_cnt】→ 描述②
      （cycle_cnt 由 STM32 每 100 Hz 控制周期加 1；它停住就说明 STM32 卡了）

判读表：
    Pi 发帧间隔正常（max < 30 ms）+ cycle_cnt 停住  ⇒ ② STM32 卡住，查固件
    Pi 发帧间隔出现 300+ ms 空档                     ⇒ ① 树莓派卡住，查 Python
    两者都正常                                       ⇒ 帧在物理层丢了，查线路

⚠️ 安全：本工具只发 `MODE_IDLE` 且 `flags=0`，
    ⇒ 不会置 FLAG_ENABLE、不会上力矩、机器人不会动。
    ⇒ 可以在机器人站立时放心跑。

用法::

    python3 linkprobe.py --seconds 20
    python3 linkprobe.py --seconds 20 --hz 100
"""

from __future__ import annotations

import sys as _sys
for _s in (_sys.stdout, _sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import argparse
import os
import statistics
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import config as C                                      # noqa: E402
from link import PiLink, LinkError                      # noqa: E402
from rlcat_proto import MODE_IDLE, TYPE_TEL_FAST        # noqa: E402


def pct(vals: list, p: float) -> float:
    """第 p 百分位（p ∈ 0..100）。空列表返回 0。"""
    if not vals:
        return 0.0
    s = sorted(vals)
    k = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


def main() -> int:
    ap = argparse.ArgumentParser(description="RL-CAT 链路探针（只发 IDLE，不动机器人）")
    ap.add_argument("--port", default=C.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=C.SERIAL_BAUD)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--hz", type=float, default=100.0)
    args = ap.parse_args()

    link = PiLink(args.port, args.baud, verbose=False)
    try:
        link.open()
    except LinkError as e:
        print("❌ 打不开串口 %s: %s" % (args.port, e))
        return 1

    print("=" * 78)
    print(" 链路探针   %s @ %d   目标 %.0f Hz   %.0f 秒"
          % (args.port, args.baud, args.hz, args.seconds))
    print("=" * 78)
    print(" ⚠️ 全程只发 MODE_IDLE + flags=0 ⇒ 不上力矩，机器人不会动\n")

    tel = link.wait_tel_fast(timeout_ms=3000)
    if tel is None:
        print("❌ 3 秒内没收到 TEL_FAST —— 检查接线 / 波特率 / 设备节点")
        link.close()
        return 1

    period = 1.0 / args.hz
    t0 = time.perf_counter()
    t_end = t0 + args.seconds
    t_next = t0

    send_gaps = []           # 相邻两次【成功发帧】的间隔（ms）
    last_send = None
    sent = 0
    send_fail = 0

    # cycle_cnt 观测
    cyc_hist = []            # (perf_counter, cycle_cnt)
    last_cyc = tel.cycle_cnt

    # ⭐⭐ 额外观测（2026-10-17 加）：区分"链路问题"和"掉电/复位"
    #   ⚠️ 起因：探针已证明 Pi 发帧（max 11.8 ms）和 STM32 周期（max 15.1 ms）
    #      都正常，却仍有 ESTOP。那就不是"发得晚"，而是别的东西：
    #        · uptime 归零      ⇒ STM32 复位了（最可能是 12V 被大电流拉塌）
    #        · ESTOP 帧出现     ⇒ 固件确实进了急停
    #        · voltage 明显下陷 ⇒ 电源被拖动
    n_estop = 0
    uptime_resets = 0

    # ⭐⭐ 用 link 自己的计数器，而不是轮询 last_tel（2026-10-17 修自己的 bug）
    #   ⚠️ 原因：探针原来只读 link.last_tel 看 cycle_cnt 变化。
    #      若两次 pump() 之间到来了 2 帧，中间那帧就被吞掉，
    #      于是算出来的"丢帧率"会虚高（实测虚报成 18.4%）。
    #      ⇒ 权威数据是 LinkStats：
    #          rx_by_type[TEL_FAST] = 真正解析到的 TEL_FAST 帧数
    #          cycle_lost           = link.py 检测到的周期不连续累计数
    n_tf0 = link.stats.rx_by_type.get(TYPE_TEL_FAST, 0)
    lost0 = link.stats.cycle_lost
    gaps0 = link.stats.cycle_gaps
    crc0  = link.stats.crc_errors
    len0  = link.stats.len_errors
    fto0  = link.stats.frame_timeouts
    unk0  = link.stats.unknown_type
    rxb0  = link.stats.rx_bytes
    txe0  = link.stats.tx_errors
    min_mv = 1 << 30
    last_uptime = None

    while time.perf_counter() < t_end:
        link.pump()
        now = time.perf_counter()

        # ---- 记录 STM32 的 cycle_cnt（每个新帧都记一次）----
        t = link.last_tel
        if t is not None and t.cycle_cnt != last_cyc:
            last_cyc = t.cycle_cnt
            cyc_hist.append((now, t.cycle_cnt))
            if t.status & 0x80:              # ST_ESTOP
                n_estop += 1
            if t.voltage_mv > 0:
                min_mv = min(min_mv, t.voltage_mv)

        # ---- TEL_STATUS：uptime 变小 = STM32 重启（掉电 / 看门狗）----
        s = link.last_status
        if s is not None and s.uptime_ms != last_uptime:
            if last_uptime is not None and s.uptime_ms < last_uptime:
                uptime_resets += 1
            last_uptime = s.uptime_ms

        if now < t_next:
            time.sleep(0.0005)
            continue
        t_next = now + period

        if not link.send_cmd_joint(C.Q_DEFAULT, mode=MODE_IDLE, flags=0):
            send_fail += 1
            continue

        ts = time.perf_counter()
        if last_send is not None:
            send_gaps.append((ts - last_send) * 1000.0)
        last_send = ts
        sent += 1

    link.close()
    elapsed = time.perf_counter() - t0

    # ---- 报告：树莓派侧的发送间隔 ----
    print("─" * 78)
    print(" ① 树莓派侧：发帧间隔（只在【成功写串口】时统计）")
    print("─" * 78)
    print("   实际发出      : %d 帧 / %.1f s  = %.1f Hz（失败 %d）"
          % (sent, elapsed, sent / max(elapsed, 1e-9), send_fail))
    if send_gaps:
        print("   间隔 p50/p95/p99/max : %.1f / %.1f / %.1f / %.1f ms"
              % (pct(send_gaps, 50), pct(send_gaps, 95),
                 pct(send_gaps, 99), max(send_gaps)))
        for thr in (C.LINK_LOST_MS, 100.0, C.LINK_ESTOP_MS):
            n = sum(1 for g in send_gaps if g > thr)
            print("   间隔 > %5.0f ms      : %d 次%s"
                  % (thr, n, "   ← ⚠️ 越过固件 ESTOP 阈值" if thr >= C.LINK_ESTOP_MS else ""))
    else:
        print("   ⚠️ 没有采集到间隔样本")

    # ---- 报告：STM32 的 cycle_cnt 推进 ----
    print()
    print("─" * 78)
    print(" ② STM32 侧：cycle_cnt 推进（正常应恒为 100 Hz）")
    print("─" * 78)
    if len(cyc_hist) >= 3:
        gaps = []                      # (dt_ms, dcyc) 相邻两个观测点之间
        for i in range(1, len(cyc_hist)):
            dt = (cyc_hist[i][0] - cyc_hist[i - 1][0]) * 1000.0
            dcyc = (cyc_hist[i][1] - cyc_hist[i - 1][1]) & 0xFFFF
            if dcyc > 0 and dt > 0:
                gaps.append((dt, dcyc))
        dts = [g[0] for g in gaps]

        # 相邻 TEL_FAST 的正常间隔 ≈ 10 ms；把它当基线，找长间隔
        print("   TEL_FAST 相邻间隔 p50/p99/max : %.1f / %.1f / %.1f ms"
              % (pct(dts, 50), pct(dts, 99), max(dts) if dts else 0.0))
        big = [g for g in gaps if g[0] > 100.0]
        print("   间隔 > 100 ms       : %d 次" % len(big))

        # ⭐⭐ 关键：把长空档期间的 cycle_cnt 跳变打出来。
        #    ⚠️ 这是区分"STM32 真的没跑"和"跑了但帧在线上丢了"的唯一判据：
        #       dcyc ≈ dt/10  ⇒ STM32 一直在跑，是 TEL_FAST 丢了
        #       dcyc ≈ 0~1    ⇒ STM32 真的没跑（被阻塞）
        if big:
            print()
            print("   ⚠️ 长空档明细（判据：正常应 dcyc ≈ dt/10）")
            print("      %-10s %-8s %-10s %s" % ("dt(ms)", "dcyc", "期望dcyc", "判定"))
            for dt, dcyc in sorted(big, key=lambda x: -x[0])[:5]:
                expect = dt / 10.0
                if dcyc < expect * 0.5:
                    verdict = "STM32 没跑（被阻塞）"
                elif dcyc > expect * 1.5:
                    verdict = "异常"
                else:
                    verdict = "帧丢了，32 一直在跑"
                print("      %-10.0f %-8d %-10.1f %s" % (dt, dcyc, expect, verdict))

        # ⭐ 整段帧丢失率
        #   ⚠️ 算清楚：
        #      · 相邻两帧之间 cycle_cnt 推进 dcyc ⇒ 那段时间 STM32 跑了 dcyc 个周期
        #      · 每个周期都会发一帧 TEL_FAST
        #      ⇒ 应有帧数 = sum(dcyc)，实收 = 区间个数 = len(gaps)
        #      ⇒ 丢的帧数 = sum(dcyc) − len(gaps)
        # ⭐ 权威的帧丢失统计（来自 LinkStats，不是轮询推算）
        n_tf = link.stats.rx_by_type.get(TYPE_TEL_FAST, 0) - n_tf0
        c_lost = link.stats.cycle_lost - lost0
        c_gaps = link.stats.cycle_gaps - gaps0
        print()
        print("   ── 权威统计（LinkStats，不是轮询推算）──")
        print("   实收 TEL_FAST       : %d 帧" % n_tf)
        print("   周期不连续          : %d 次，累计丢 %d 个周期"
              "（%.1f%%）" % (c_gaps, c_lost,
                              100.0 * c_lost / max(1, n_tf + c_lost)))
        # ⭐⭐ 丢帧的定性分解（这几个数决定往哪儿修）
        d_crc = link.stats.crc_errors - crc0
        d_len = link.stats.len_errors - len0
        d_fto = link.stats.frame_timeouts - fto0
        d_unk = link.stats.unknown_type - unk0
        d_rxb = link.stats.rx_bytes - rxb0
        d_txe = link.stats.tx_errors - txe0
        print("   ── 丢帧定性分解 ──")
        print("   收字节数        : %d B（%.0f B/s，理论 100 Hz × 103 B ≈ 10300）"
              % (d_rxb, d_rxb / max(elapsed, 1e-9)))
        print("   CRC 错          : %d 次" % d_crc)
        print("   帧长错          : %d 次" % d_len)
        print("   半帧超时        : %d 次   ← 一帧中途断掉（字节丢失）" % d_fto)
        print("   未知类型        : %d 次" % d_unk)
        print("   Pi 侧发送错误   : %d 次" % d_txe)
        print()
        if d_crc or d_len or d_fto or d_unk:
            print("   ⇒ ⚠️ 有解析错误 ⇒ 字节在【线上】被破坏/丢失。")
            print("      首要怀疑：共地不良、线太长、1 Mbps 无屏蔽、"
                  "或两端波特率有偏差。")
        elif d_rxb < 9000 * (elapsed / 1.0):
            print("   ⇒ ⚠️ 收到的字节数明显偏少，而【没有】解析错误 ⇒")
            print("      帧根本没到 Pi。可能是 STM32 侧的发送失败（静默丢帧）")
            print("      ⇒ 用 ESP32 无线口敲 K 或 R，看 link tx_errors。")
        else:
            print("   ⇒ 字节量正常且无解析错误，但帧数偏少 —— 需进一步查。")
    else:
        print("   ⚠️ TEL_FAST 太少（%d 帧），无法判断" % len(cyc_hist))

    # ---- 报告：电源 / 复位 / ESTOP（区分"链路"与"掉电"）----
    print()
    print("─" * 78)
    print(" ③ 电源 / 复位 / 急停（用来区分【链路问题】和【掉电复位】）")
    print("─" * 78)
    print("   TEL_STATUS.uptime 复位 : %d 次%s"
          % (uptime_resets,
             "   ← ⚠️⚠️ STM32 重启过！不是链路问题" if uptime_resets else "   ✅"))
    print("   收到 ESTOP 的帧        : %d 帧%s"
          % (n_estop, "   ← ⚠️ 固件确实进了急停" if n_estop else "   ✅"))
    print("   最低电压（TEL_FAST）   : %s"
          % (("%d mV" % min_mv) if min_mv < (1 << 30) else "无有效读数（voltage_mv=0）"))

    # ---- 结论 ----
    print()
    print("=" * 78)
    print(" 结论")
    print("=" * 78)
    pi_bad = [g for g in send_gaps if g > C.LINK_ESTOP_MS]
    stm_bad = max([(cyc_hist[i][0] - cyc_hist[i - 1][0]) * 1000.0
                   for i in range(1, len(cyc_hist))], default=0.0)
    if uptime_resets:
        print(" ⇒ ⚠️⚠️ STM32 在本次探测中【重启了 %d 次】" % uptime_resets)
        print("    这是**掉电/看门狗复位**，不是链路问题。")
        print("    查：12V 电源余量、电池内阻、共地、舵机堵转/大电流、")
        print("        以及加了力矩之后 12 个舵机同时启动的浪涌。")
    elif pi_bad:
        print(" ⇒ ① 树莓派侧有问题：有 %d 次发帧间隔 > %d ms（最长 %.0f ms）"
              % (len(pi_bad), C.LINK_ESTOP_MS, max(pi_bad)))
        print("    查：GC、print 到 SSH、GIL、CPU governor、是否同时跑了别的工具")
    elif stm_bad > 100.0:
        print(" ⇒ ② STM32 侧有问题：cycle_cnt 最长停更 %.0f ms，而 Pi 发帧正常"
              % stm_bad)
        print("    查：shell 命令、Flash 擦写、舵机总线阻塞")
    elif n_estop:
        print(" ⇒ ③ 链路和周期都正常，但固件【进过 ESTOP %d 帧】" % n_estop)
        print("    ⇒ 触发原因不在链路。用 ESP32 无线口敲 R 看 safety_reasons：")
        print("       0x01 电压 ≤9.3V / 0x10 过温 / 0x80 操作者急停")
    else:
        print(" ⇒ 两边都正常（Pi 最长 %.1f ms / 32 最长 %.1f ms）"
              % (max(send_gaps) if send_gaps else 0.0, stm_bad))
        print("    ⇒ 帧可能在物理层丢了：查串口线、共地、屏蔽、波特率")
    return 0


if __name__ == "__main__":
    sys.exit(main())
