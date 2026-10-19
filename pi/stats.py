#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 树莓派侧：统计小工具

⚠️ 为什么要单独一个文件：
   100 Hz 主循环要连续跑几分钟到几十分钟，任何"每周期 append 到一个 list"
   的写法都会无界增长（最终 OOM 或 GC 停顿）。这里全部用【有界】结构。

⚠️ 周期抖动（jitter）是判断"到底是不是实时"的唯一指标 —— 见
   docs/31-pi-runtime.md §4.1：判据是 P99 < 4 ms（固件等指令的窗口）。
"""

from __future__ import annotations

import time
from collections import deque
from typing import Dict, Iterable, List, Optional


class Percentiles:
    """有界的耗时统计：均值 / P50 / P95 / P99 / 最大值。

    ⚠️ 用 deque(maxlen) 而不是 list —— 满了自动丢最旧的，内存恒定。
       max 单独累计，不会因为淘汰而丢失（reset 时一起清）。
    """

    def __init__(self, capacity: int = 20_000, unit: str = "us"):
        self.capacity = capacity
        self.unit = unit
        self._buf: deque = deque(maxlen=capacity)
        self.n_total = 0
        self._max = 0.0
        self._min = float("inf")
        self._sum = 0.0

    def add(self, v: float) -> None:
        self._buf.append(v)
        self.n_total += 1
        # ⚠️ 总和与极值只在"当前窗口"里维护：窗口滚动时无法精确扣减，
        #    所以这里用【累计】语义，平均值取累计，P 值取当前窗口。
        #    对本项目够用：判据是 P99，且窗口 >= 2 万个周期（200 秒）。
        self._sum += v
        if v > self._max:
            self._max = v
        if v < self._min:
            self._min = v

    def _pct(self, p: float) -> float:
        if not self._buf:
            return 0.0
        s = sorted(self._buf)
        k = int(round(p / 100.0 * (len(s) - 1)))
        k = 0 if k < 0 else (len(s) - 1 if k >= len(s) else k)
        return s[k]

    def reset(self) -> None:
        self._buf.clear()
        self.n_total = 0
        self._max = 0.0
        self._min = float("inf")
        self._sum = 0.0

    @property
    def n(self) -> int:
        return len(self._buf)

    @property
    def mean(self) -> float:
        return (self._sum / self.n_total) if self.n_total else 0.0

    @property
    def p50(self) -> float:
        return self._pct(50)

    @property
    def p95(self) -> float:
        return self._pct(95)

    @property
    def p99(self) -> float:
        return self._pct(99)

    @property
    def maximum(self) -> float:
        return self._max

    @property
    def minimum(self) -> float:
        return 0.0 if self._min == float("inf") else self._min

    def line(self, label: str = "") -> str:
        if self.n_total == 0:
            return "%s(无样本)" % (label + " " if label else "")
        return ("%s均值 %.0f  P50 %.0f  P95 %.0f  P99 %.0f  最大 %.0f  (n=%d) %s"
                % (label + " " if label else "",
                   self.mean, self.p50, self.p95, self.p99, self.maximum,
                   self.n_total, self.unit))


class RateMeter:
    """滑动窗口测速率（Hz）。用于确认"真的在 100 Hz"。"""

    def __init__(self, window_s: float = 2.0):
        self.window_s = window_s
        self._t: deque = deque()

    def tick(self, t: Optional[float] = None) -> None:
        now = time.perf_counter() if t is None else t
        self._t.append(now)
        cutoff = now - self.window_s
        while self._t and self._t[0] < cutoff:
            self._t.popleft()

    @property
    def hz(self) -> float:
        if len(self._t) < 2:
            return 0.0
        span = self._t[-1] - self._t[0]
        return (len(self._t) - 1) / span if span > 0 else 0.0


class Counters:
    """具名计数器 + 可读报告。"""

    def __init__(self) -> None:
        self._c: Dict[str, int] = {}

    def bump(self, name: str, n: int = 1) -> None:
        self._c[name] = self._c.get(name, 0) + n

    def get(self, name: str) -> int:
        return self._c.get(name, 0)

    def as_dict(self) -> Dict[str, int]:
        return dict(self._c)

    def reset(self) -> None:
        self._c.clear()

    def line(self, keys: Optional[Iterable[str]] = None) -> str:
        items = self._c.items() if keys is None else ((k, self.get(k)) for k in keys)
        return "  ".join("%s %d" % (k, v) for k, v in items)


def fmt_ms(us: float) -> str:
    return "%.2f ms" % (us / 1000.0)


def _self_test() -> int:
    errs: List[str] = []
    p = Percentiles(capacity=100)
    for i in range(1000):
        p.add(float(i))
    if p.n != 100:
        errs.append("窗口未生效：n=%d" % p.n)
    if p.maximum != 999.0:
        errs.append("max 未跨窗口累计：%r" % p.maximum)
    if not (0 <= p.p50 <= p.p95 <= p.p99 <= p.maximum):
        errs.append("分位数没单调：%r %r %r %r" % (p.p50, p.p95, p.p99, p.maximum))

    p2 = Percentiles(capacity=5)
    for v in (1.0, 2.0, 3.0, 4.0, 5.0):
        p2.add(v)
    if abs(p2.p50 - 3.0) > 1e-9:
        errs.append("p50 不对：%r" % p2.p50)
    if abs(p2.p99 - 5.0) > 1e-9:
        errs.append("p99 不对：%r" % p2.p99)

    r = RateMeter(window_s=1.0)
    t0 = 1000.0
    for i in range(101):
        r.tick(t0 + i * 0.01)          # 100 Hz
    if abs(r.hz - 100.0) > 0.5:
        errs.append("RateMeter 不准：%.3f" % r.hz)

    c = Counters()
    c.bump("a"); c.bump("a"); c.bump("b", 3)
    if c.get("a") != 2 or c.get("b") != 3:
        errs.append("Counters 计数错")

    if errs:
        for e in errs:
            print("  !! " + e)
        return 1
    print("  stats.py 自检通过（有界分位统计 / 速率 / 计数器）")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(_self_test())
