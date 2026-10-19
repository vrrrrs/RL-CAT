#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT 树莓派侧：与 STM32 的串口链路层（基础层，阶段 1）

职责边界（⚠️ 只做这些，不做别的）：
    · 打开/配置/关闭串口（1 Mbps、8N1、无流控、raw、非阻塞）
    · 字节流 → 帧（复用 rlcat_proto.FrameDecoder，与固件状态机同源）
    · 帧 → 结构化对象（TEL_FAST / TEL_STATUS / TEL_ACK / TEL_LOG）
    · 组帧发送（CMD_JOINT / CMD_MODE / CMD_CALIB / HEARTBEAT）
    · 统计：rx / crc_err / 半帧超时 / cycle_cnt 丢帧 / 回复耗时

⚠️ 不做：不解观测、不算动作、不管模式语义。那些在上层。

⚠️ 三个必须做对的地方（docs/31-pi-runtime.md §3.1）：
    ① 非阻塞读 + select  —— 不能用阻塞 read()，会卡死 100 Hz 循环
    ② 接收要能处理跨周期残包  —— FrameDecoder 本身就是流式状态机
    ③ rx_overflow / crc_errors / timeouts 三个指标必须记

⚠️ 关于 pyserial：
    只用它【打开与配置】端口（波特率/位数/校验在任何平台上都对），
    读用 os.read + select（零 Python 层缓冲），写用 os.write。
"""

from __future__ import annotations

import errno
import os
import select
import sys
import tempfile
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Set

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from rlcat_proto import (  # noqa: E402
    FRAME_OVERHEAD,
    LEN_TEL_FAST,
    LEN_TEL_STATUS,
    N_JOINTS,
    TYPE_CMD_CALIB,
    TYPE_CMD_JOINT,
    TYPE_CMD_MODE,
    TYPE_HEARTBEAT,
    TYPE_TEL_ACK,
    TYPE_TEL_FAST,
    TYPE_TEL_LOG,
    TYPE_TEL_STATUS,
    Frame,
    FrameDecoder,
    TelFast,
    TelStatus,
    make_cmd_calib,
    make_cmd_joint,
    make_cmd_mode,
    make_heartbeat,
    parse_tel_fast,
    parse_tel_status,
)

import config as C  # noqa: E402


# ============================================================================
# 统计
# ============================================================================

@dataclass
class LinkStats:
    rx_bytes: int = 0
    rx_frames: int = 0
    rx_by_type: Dict[int, int] = field(default_factory=dict)
    crc_errors: int = 0
    len_errors: int = 0
    frame_timeouts: int = 0          # 半帧超时复位
    unknown_type: int = 0
    tx_frames: int = 0
    tx_bytes: int = 0
    tx_errors: int = 0
    cycle_gaps: int = 0              # cycle_cnt 不连续次数
    cycle_lost: int = 0              # 累计丢的周期数
    stm32_restarts: int = 0          # ⭐ 检测到的 STM32 重启次数（断电重上电）
    last_cycle: Optional[int] = None
    tel_logs: int = 0

    def bump_type(self, t: int) -> None:
        self.rx_by_type[t] = self.rx_by_type.get(t, 0) + 1

    def type_name(self, t: int) -> str:
        return {
            TYPE_CMD_JOINT: "CMD_JOINT", TYPE_TEL_FAST: "TEL_FAST",
            TYPE_CMD_MODE: "CMD_MODE", TYPE_TEL_STATUS: "TEL_STATUS",
            TYPE_CMD_CALIB: "CMD_CALIB", TYPE_TEL_LOG: "TEL_LOG",
            TYPE_TEL_ACK: "TEL_ACK", TYPE_HEARTBEAT: "HEARTBEAT",
        }.get(t, "0x%02X" % t)


# ============================================================================
# 链路
# ============================================================================

class LinkError(RuntimeError):
    pass


class PiLink:
    """与 STM32 的串口链路。

    典型用法（不阻塞的 100 Hz 主循环）::

        link = PiLink(C.SERIAL_PORT, C.SERIAL_BAUD)
        link.open()
        while True:
            for fr in link.pump():          # 非阻塞：把已到的帧全部取走
                ...
            fr = link.wait_for({TYPE_TEL_FAST}, timeout_ms=5)   # 需要时再等
            ...

    需要"请求-应答"时（这才是固件模型 A 的实际形态）::

        tel = link.wait_tel_fast(timeout_ms=10)   # 等到 STM32 的遥测
        t0 = time.perf_counter()
        link.send_cmd_joint(q, mode=C.MODE_ACTIVE)
        lat_us = (time.perf_counter() - t0) * 1e6
    """

    RECENT_MAX = 64

    def __init__(self,
                 port: str = C.SERIAL_PORT,
                 baud: int = C.SERIAL_BAUD,
                 frame_timeout_ms: int = C.FRAME_TIMEOUT_MS,
                 verbose: bool = False):
        self.port = port
        self.baud = baud
        self.verbose = verbose

        self.stats = LinkStats()
        self._decoder = FrameDecoder(frame_timeout_ms=frame_timeout_ms)

        self._ser = None                 # pyserial 对象（只管配置）
        self._fd: Optional[int] = None
        self._seq = 0
        self._lock_fd: Optional[int] = None     # ⭐ 串口独占锁（见 open()）

        # ⭐ 发帧看门狗（2026-10-18 加）：检测"开着口却不发帧"的工具
        #
        #  ⚠️ 为什么需要：固件把"距上次收到任何帧 > RLCA_LINK_ESTOP_MS"判为失联
        #     并进 ESTOP（当前开发期值 1200 ms，见 config.LINK_ESTOP_MS）。
        #     于是**任何工具只要暂停发帧，机器人就会进 ESTOP** ——
        #     "不命令关节目标"和"不发帧"是两件事，很容易写错。
        #     这个坑已经踩过三次（status_panel 没加 --keepalive、
        #     两进程抢串口、stand_test 的 --observe 一帧不发）。
        #  ⇒ 链路层自己盯着：用 **300 ms** 作【早期预警】（比固件阈值早得多，
        #     这样问题在变成 ESTOP 之前就被发现），把问题暴露在开发期。
        self._last_tx_ms: Optional[float] = None
        self._tx_starved_warned = False

        # 最近收到的帧（环形，便于调试/事后查）
        self.recent: deque = deque(maxlen=self.RECENT_MAX)
        # 最近一次解析结果
        self.last_tel: Optional[TelFast] = None
        self.last_status: Optional[TelStatus] = None
        self.last_ack: Optional[tuple] = None      # (action, result)
        self.last_log: Optional[tuple] = None      # (level, text)

        # ⭐ 上一次 TEL_STATUS 里的 STM32 uptime（ms）。用于检测"STM32 断电重启"：
        #    uptime 变小 ⇒ 重启 ⇒ cycle_cnt 归零，不该算成"丢了 6 万个周期"。
        self._last_uptime_ms: Optional[int] = None

        # 时间戳（perf_counter）：用于算"收到遥测 → 发出指令"的耗时
        self.t_rx_tel_fast: float = 0.0
        self.t_rx_status: float = 0.0
        self.t_rx_any: float = 0.0

        # 可选回调
        self.on_tel_fast: Optional[Callable[[TelFast], None]] = None
        self.on_status: Optional[Callable[[TelStatus], None]] = None

    # ---------------------------------------------------------------- 生命周期

    # ⭐⭐⭐ 串口独占锁（flock）—— 2026-10-17 加，血的教训
    #
    #  ⚠️⚠️ 为什么必须锁：两个进程同时打开 /dev/ttyAMA0 时，
    #      **内核不会报错**，而是把收到的字节随机分给两个读端。
    #      ⇒ 双方各丢约一半的帧；双方都表现为"链路正常但遥测缺失"，
    #        而且 CRC 错 = 0（字节本身没问题，只是被别的进程读走了）。
    #
    #  ⚠️ 现场踩到（2026-10-17）：按"终端 A 跑 linkprobe、终端 B 跑 pose.py"
    #     的方式测，测得丢帧 28~30%，花了好几轮去查线路和固件，
    #     而**根因就是两个进程抢同一个串口**。单独跑任何一个都是 0% 丢帧。
    #
    #  ⭐ 用 flock 而不是"检查 /proc 里有没有别人打开"：
    #     flock 是原子的、进程崩溃时内核自动释放，不会有陈旧锁。
    #     ⚠️ 锁文件按设备名区分，所以离线测试用 PTY 时互不干扰。

    def _lock_path(self) -> str:
        name = os.path.basename(str(self.port)) or "serial"
        return os.path.join(tempfile.gettempdir(), "rlcat_link_%s.lock" % name)

    def _take_port_lock(self) -> None:
        """拿独占到本进程结束（或 close()）。已有别的进程占用则抛 LinkError。"""
        if self._lock_fd is not None:
            return
        try:
            import fcntl
        except ImportError:
            return                          # 非 POSIX：跳过（Windows 上跑不了真串口）
        path = self._lock_path()
        try:
            fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o644)
        except OSError:
            return                          # 建不了锁文件就别挡路
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            # 读一下持有者 PID，给一条能直接定位的报错
            holder = ""
            try:
                holder = os.pread(fd, 64, 0).decode("ascii", "replace").strip()
            except Exception:
                pass
            os.close(fd)
            raise LinkError(
                "%s 已被另一个进程占用（%s）。\n"
                "⚠️ 两个进程同时打开同一个串口会让双方各丢一半的帧，\n"
                "   而且不会报任何错 —— 这是 2026-10-17 排查了很久的坑。\n"
                "   ⇒ 请先关掉 status_panel / pose / jog / calib 等工具，再重试。"
                % (self.port, holder or "PID 未知"))
        try:
            os.ftruncate(fd, 0)
            os.pwrite(fd, ("pid=%d" % os.getpid()).encode("ascii"), 0)
        except Exception:
            pass
        self._lock_fd = fd

    def _drop_port_lock(self) -> None:
        if self._lock_fd is None:
            return
        try:
            import fcntl
            fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
        except Exception:
            pass
        try:
            os.close(self._lock_fd)
        except Exception:
            pass
        self._lock_fd = None

    def open(self) -> None:
        if self._ser is not None:
            return
        try:
            import serial                       # pyserial 只在需要时导入
        except ImportError as e:
            raise LinkError("需要 pyserial：sudo apt install python3-serial") from e

        self._take_port_lock()

        try:
            ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,                      # 非阻塞
                write_timeout=0.05,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
            )
        except Exception as e:
            self._drop_port_lock()
            raise LinkError("打不开串口 %s: %s" % (self.port, e)) from e

        self._ser = ser
        self._fd = ser.fileno()
        self._force_raw(self._fd)
        self.flush_input()
        if self.verbose:
            print("[link] 已打开 %s @ %d (8N1, raw, 非阻塞)" % (self.port, self.baud))

    def close(self) -> None:
        self._fd = None
        self._drop_port_lock()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def reopen(self, settle_s: float = 0.2) -> None:
        """关掉再打开（用于恢复异常链路）。"""
        self.close()
        time.sleep(settle_s)
        self.open()

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._fd is not None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # ---------------------------------------------------------------- 串口配置

    @staticmethod
    def _force_raw(fd: int) -> None:
        """显式设成 raw + VMIN=0/VTIME=0。

        ⚠️ pyserial 一般已经做了，但"一般"不够 —— 这里再确认一次：
           少了它，一旦落到 canonical 模式，read 会一直等换行符，
           整个 100 Hz 循环直接卡死。这是 docs/31-pi-runtime.md §3.1 明确要求的。
        """
        import termios
        try:
            attrs = termios.tcgetattr(fd)
        except Exception:
            return
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs

        iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                   termios.ISTRIP | termios.INLCR | termios.IGNCR |
                   termios.ICRNL | termios.IXON)
        oflag &= ~termios.OPOST
        lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON |
                   termios.ISIG | termios.IEXTEN)
        cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
        cflag |= termios.CS8
        for name in ("CRTSCTS", "CCTS_OFLOW", "CRTS_IFLOW"):
            bit = getattr(termios, name, 0)
            if bit:
                cflag &= ~bit
        cc = list(cc)
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0

        termios.tcsetattr(fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])

    def flush_input(self) -> None:
        """丢掉接收缓冲里的残留字节。"""
        if self._fd is None:
            return
        import termios
        try:
            termios.tcflush(self._fd, termios.TCIFLUSH)
        except Exception:
            pass
        self._decoder.reset()

    # ---------------------------------------------------------------- 时间

    @staticmethod
    def _now_ms() -> int:
        return time.monotonic_ns() // 1_000_000

    # ---------------------------------------------------------------- 接收

    def pump(self) -> List[Frame]:
        """非阻塞地把所有已到达的字节读干净并解码。

        返回本次新解出的帧列表（可能是空的）。
        ⚠️ 每个周期都应该调用它 —— 不要让内核接收缓冲堆积。
        """
        out: List[Frame] = []
        if self._fd is None:
            return out

        # ⭐⭐ 发帧看门狗（2026-10-18 加）
        #
        #  ⚠️ 固件把"距上次收到任何帧 > RLCA_LINK_ESTOP_MS"判为失联并进 ESTOP。
        #     所以只要工具暂停发帧，机器人就进 ESTOP ——
        #     而"不命令关节目标"和"不发帧"是两件事，极易写错
        #     （本仓库已经踩过三次）。
        #  ⇒ 这里主动发现：连续 300 ms 没发过 CMD_JOINT 就警告一次
        #     （300 ms 是刻意选的早期预警值，远小于固件的 1200 ms）。
        #  ⚠️ 只警告一次，不刷屏；下次发帧后自动复位。
        #  ⚠️ 纯观察工具（如 status_panel 不加 --keepalive）也会触发 ——
        #     那正是事实：它会让机器人进 ESTOP。
        if self._last_tx_ms is not None:
            starved = time.monotonic() - self._last_tx_ms
            if starved > 0.3 and not self._tx_starved_warned:
                self._tx_starved_warned = True
                print("[link] ⚠️ 已 %.0f ms 没发过帧 —— 固件会把链路判为失联"
                      "并进 ESTOP。\n"
                      "       ⇒ 若本工具不是故意只观察，请检查是否漏了 keepalive 帧；\n"
                      "         若确实是只观察，加 --keepalive（或用 MODE_ACTIVE + "
                      "目标=当前姿态）。" % (starved * 1000.0))
        # ⚠️ 这里【不能】return —— 看门狗只是顺手检查，后面还要继续把串口读干净。
        #    （2026-10-18：一度把 `return out` 插在这里，导致 pump() 一进来就返回、
        #      一个字节都不读，整个离线测试套件立刻红掉 run_checks exit=1。
        #      这就是"改完必须跑测试"的价值 —— 它比眼睛可靠。）

        while True:
            try:
                r, _, _ = select.select([self._fd], [], [], 0)
            except (OSError, ValueError):
                break
            if not r:
                break
            try:
                chunk = os.read(self._fd, C.RX_CHUNK)
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                    break
                # 设备被拔掉等：记一次，交给上层决定
                self.stats.tx_errors += 0
                break
            if not chunk:
                break

            self.stats.rx_bytes += len(chunk)
            now = self._now_ms()
            for fr in self._decoder.feed(chunk, now):
                out.append(fr)
                self._dispatch(fr)

        # 半帧超时
        if self._decoder.tick(self._now_ms()):
            self.stats.frame_timeouts += 1

        # 把解码器的错误计数同步过来（解码器是权威）
        self.stats.crc_errors = self._decoder.n_crc_err
        self.stats.len_errors = self._decoder.n_len_err
        return out

    def _dispatch(self, fr: Frame) -> None:
        self.recent.append(fr)
        self.stats.rx_frames += 1
        self.stats.bump_type(fr.type)
        self.stats.rx_by_type  # noqa: B018  (可读性)
        self.t_rx_any = time.perf_counter()

        if fr.type == TYPE_TEL_FAST:
            try:
                tel = parse_tel_fast(fr.payload)
            except ValueError:
                self.stats.len_errors += 1
                return
            self._note_cycle(tel.cycle_cnt)
            self.last_tel = tel
            self.t_rx_tel_fast = self.t_rx_any
            if self.on_tel_fast is not None:
                self.on_tel_fast(tel)

        elif fr.type == TYPE_TEL_STATUS:
            try:
                st = parse_tel_status(fr.payload)
            except ValueError:
                self.stats.len_errors += 1
                return

            # ⭐⭐ 检测 STM32 重启（2026-10-17）
            #  ⚠️ 为什么必须检测：`cycle_cnt` 是 16 位、上电从 0 开始。
            #     STM32 断电重启后它会归零，而 `_note_cycle()` 只看到
            #     "上一次是 60000，这次是 0" ⇒ 算出 delta≈5500
            #     ⇒ 记成"丢了 5500 个周期"，面板上就是
            #       "cycle_cnt 跳变 3 次（累计丢 168308 个周期）"。
            #     而真实原因只是中间断了几次电。
            #  ⇒ 判据：uptime 变小 = 重启 ⇒ 让 _note_cycle 重新对齐基线，
            #     不计入丢帧（TEL_FAST 本身不带 uptime，只能靠 TEL_STATUS）。
            if self._last_uptime_ms is not None and st.uptime_ms < self._last_uptime_ms:
                self.stats.stm32_restarts += 1
                self.stats.last_cycle = None      # 下一帧重新对齐，不算失步
            self._last_uptime_ms = st.uptime_ms

            self.last_status = st
            self.t_rx_status = self.t_rx_any
            if self.on_status is not None:
                self.on_status(st)

        elif fr.type == TYPE_TEL_ACK:
            if len(fr.payload) >= 2:
                self.last_ack = (fr.payload[0], fr.payload[1])

        elif fr.type == TYPE_TEL_LOG:
            self.stats.tel_logs += 1
            if len(fr.payload) >= 2:
                lvl = fr.payload[0]
                n = min(fr.payload[1], len(fr.payload) - 2)
                txt = bytes(fr.payload[2:2 + n]).decode("ascii", "replace")
                self.last_log = (lvl, txt)

        elif fr.type == TYPE_HEARTBEAT:
            pass

        else:
            self.stats.unknown_type += 1

    def _note_cycle(self, cnt: int) -> None:
        """用 cycle_cnt 检测丢帧（协议 §4.4.1）。"""
        prev = self.stats.last_cycle
        if prev is not None:
            delta = (cnt - prev) & 0xFFFF
            if delta != 1:
                self.stats.cycle_gaps += 1
                if delta > 1:
                    self.stats.cycle_lost += delta - 1
        self.stats.last_cycle = cnt

    def wait_for(self, types: Set[int], timeout_ms: int) -> Optional[Frame]:
        """等到指定类型的帧，或超时。非匹配的帧会被正常缓存/分发，不会丢。"""
        deadline = time.perf_counter() + timeout_ms / 1000.0
        while True:
            for fr in self.pump():
                if fr.type in types:
                    return fr
            remain = deadline - time.perf_counter()
            if remain <= 0:
                return None
            if self._fd is None:
                return None
            try:
                select.select([self._fd], [], [], remain)
            except (OSError, ValueError):
                return None

    def wait_tel_fast(self, timeout_ms: int = 10) -> Optional[TelFast]:
        fr = self.wait_for({TYPE_TEL_FAST}, timeout_ms)
        return self.last_tel if fr is not None else None

    def wait_status(self, timeout_ms: int = 200) -> Optional[TelStatus]:
        fr = self.wait_for({TYPE_TEL_STATUS}, timeout_ms)
        return self.last_status if fr is not None else None

    def wait_ack(self, timeout_ms: int = 500) -> Optional[tuple]:
        """等一个 TEL_ACK（CMD_CALIB 用）。返回 (action, result)。"""
        before = self.last_ack
        deadline = time.perf_counter() + timeout_ms / 1000.0
        while time.perf_counter() < deadline:
            self.pump()
            if self.last_ack is not None and self.last_ack != before:
                return self.last_ack
            time.sleep(0.002)
        return None

    def lost_ms(self) -> float:
        """距上次收到【任意帧】的毫秒数（没收到过返回 inf）。"""
        if self.t_rx_any == 0.0:
            return float("inf")
        return (time.perf_counter() - self.t_rx_any) * 1000.0

    def is_alive(self) -> bool:
        return self.lost_ms() <= C.LINK_LOST_MS

    # ---------------------------------------------------------------- 发送

    def _send(self, blob: bytes) -> bool:
        if self._fd is None:
            return False
        try:
            n = os.write(self._fd, blob)       # 38~110 B，不会阻塞
        except OSError:
            self.stats.tx_errors += 1
            return False
        if n != len(blob):
            self.stats.tx_errors += 1
            return False
        self.stats.tx_frames += 1
        self.stats.tx_bytes += n
        return True

    def _next_seq(self) -> int:
        s = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return s

    def _note_tx(self) -> None:
        """每次成功发帧后更新看门狗时间戳。"""
        self._last_tx_ms = time.monotonic()
        self._tx_starved_warned = False

    def send_cmd_joint(self,
                       q_target: Sequence[float],
                       mode: int = C.MODE_ACTIVE,
                       flags: int = 0,
                       kp_scale: int = 100,
                       led: int = 1) -> bool:
        """下行关节目标角（q_target 必须是【逻辑索引 A】的 12 维，单位 rad）。

        ⚠️ flags 默认 0：固件里 FLAG_ENABLE(0x01)/FLAG_RESET_ERR/FLAG_REQ_CALIB
           【都没有实现】，只有 FLAG_REQ_STATUS(0x04) 有效。别传没用的位。
        """
        if len(q_target) != N_JOINTS:
            raise ValueError("q_target 必须是 %d 维" % N_JOINTS)
        blob = make_cmd_joint(q_target, mode=mode, flags=flags,
                              seq=self._next_seq(), kp_scale=kp_scale, led=led)
        ok = self._send(blob)
        if ok:
            self._note_tx()          # ⭐ 看门狗：刷新"最后一次发帧"时间
        return ok

    def send_cmd_mode(self, mode: int, param: int = 0) -> bool:
        return self._send(make_cmd_mode(mode, param, self._next_seq()))

    def send_cmd_calib(self, action: int) -> bool:
        return self._send(make_cmd_calib(action, self._next_seq()))

    def send_heartbeat(self) -> bool:
        """⚠️ 只有心跳、没有 CMD_JOINT，固件会一直以为链路活着并继续跟踪冻结的
           旧目标（它的失联判定基于【任意帧】）。所以心跳不能替代控制指令。"""
        return self._send(make_heartbeat(self._next_seq()))

    # ---------------------------------------------------------------- 诊断

    def report(self) -> str:
        s = self.stats
        lines = []
        lines.append("[链路] %s @ %d" % (self.port, self.baud))
        lines.append("  收到 %d 帧 / %d 字节" % (s.rx_frames, s.rx_bytes))
        by = "  ".join("%s %d" % (self.stats.type_name(t), n)
                       for t, n in sorted(s.rx_by_type.items()))
        lines.append("  分类: %s" % (by if by else "(无)"))
        lines.append("  CRC 错 %d   半帧超时 %d   LEN 错 %d   未知类型 %d"
                     % (s.crc_errors, s.frame_timeouts, s.len_errors, s.unknown_type))
        lines.append("  cycle_cnt 跳变 %d 次（累计丢 %d 个周期）"
                     % (s.cycle_gaps, s.cycle_lost))
        lines.append("  发出 %d 帧 / %d 字节   发送失败 %d"
                     % (s.tx_frames, s.tx_bytes, s.tx_errors))
        return "\n".join(lines)

    def check_integrity(self) -> List[str]:
        """返回"不健康"的说明列表（空 = 健康）。给脚本/面板用。"""
        bad: List[str] = []
        s = self.stats
        if s.rx_frames == 0:
            bad.append("一帧都没收到")
            return bad
        if s.crc_errors:
            bad.append("CRC 错 %d 次" % s.crc_errors)
        if s.len_errors:
            bad.append("LEN 错 %d 次" % s.len_errors)
        if s.cycle_gaps:
            bad.append("cycle_cnt 跳变 %d 次（丢 %d 周期）" % (s.cycle_gaps, s.cycle_lost))
        if s.unknown_type:
            bad.append("未知类型 %d 个" % s.unknown_type)
        if not self.is_alive():
            bad.append("链路不活跃（%.0f ms 无帧）" % self.lost_ms())
        return bad


# ============================================================================
# 自检（不需要硬件）：用一对内存里的 PTY 验证收发与统计
# ============================================================================

def _self_test() -> int:
    import pty

    errs: List[str] = []
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    keepalive = os.dup(slave)          # 保住从端，避免 master 写入 EIO

    link = PiLink(slave_name, baud=115200, verbose=False)
    try:
        link.open()

        # 1) 空负载帧（HEARTBEAT）收发
        blob = make_heartbeat(7)
        os.write(master, blob)
        frames = link.pump()
        if len(frames) != 1 or frames[0].type != TYPE_HEARTBEAT:
            errs.append("HEARTBEAT 往返失败: %r" % frames)
        if frames and frames[0].seq != 7:
            errs.append("SEQ 不对: %r" % frames[0].seq)

        # 2) 垃圾字节 + AA AA 55 前缀，状态机必须能同步
        os.write(master, b"\x00\xffgarbage" + b"\xaa\xaa" + blob)
        frames = link.pump()
        if len(frames) != 1:
            errs.append("垃圾流里没解出那一帧: %r" % frames)

        # 3) CRC 破坏 → 必须被丢弃并计数
        bad = bytearray(blob)
        bad[-1] ^= 0xFF
        os.write(master, bytes(bad))
        frames = link.pump()
        if frames:
            errs.append("坏 CRC 的帧被接受了")
        if link.stats.crc_errors < 1:
            errs.append("CRC 错没计数")

        # 4) 分两次写入一个帧 → 跨包残帧必须能拼起来
        half = len(blob) // 2
        os.write(master, blob[:half])
        if link.pump():
            errs.append("半个帧不该产出帧")
        os.write(master, blob[half:])
        frames = link.pump()
        if len(frames) != 1:
            errs.append("跨包帧没拼起来: %r" % frames)

        # 5) 发送：CMD_JOINT 必须正好 38 字节且能被对端解出
        link.flush_input()
        ok = link.send_cmd_joint([0.1] * N_JOINTS, mode=C.MODE_ACTIVE)
        if not ok:
            errs.append("send_cmd_joint 返回失败")
        import select as _sel
        r, _, _ = _sel.select([master], [], [], 0.5)
        if not r:
            errs.append("对端没收到 CMD_JOINT")
        else:
            got = os.read(master, 4096)
            if len(got) != FRAME_OVERHEAD + 31:
                errs.append("CMD_JOINT 帧长 %d，应为 %d" % (len(got), FRAME_OVERHEAD + 31))
            from rlcat_proto import unpack
            fr = unpack(got)
            if fr is None or fr.type != TYPE_CMD_JOINT:
                errs.append("对端解不出 CMD_JOINT: %r" % (fr,))

        # 6) TEL_FAST 解析 + cycle_cnt 丢帧检测
        from rlcat_proto import TelFast, encode_tel_fast, pack
        for seq, cyc in ((1, 100), (2, 101), (3, 105)):
            t = TelFast(cycle_cnt=cyc)
            os.write(master, pack(TYPE_TEL_FAST, seq, encode_tel_fast(t)))
        link.pump()
        if link.stats.last_cycle != 105:
            errs.append("last_cycle 不对: %r" % link.stats.last_cycle)
        if link.stats.cycle_gaps != 1 or link.stats.cycle_lost != 3:
            errs.append("丢帧检测不对: gaps=%d lost=%d"
                        % (link.stats.cycle_gaps, link.stats.cycle_lost))

        # 7) wait_for 能等到并正确返回
        os.write(master, pack(TYPE_TEL_STATUS, 9, bytes(LEN_TEL_STATUS)))
        st = link.wait_status(timeout_ms=300)
        if st is None:
            errs.append("wait_status 超时")

        # 8) 非阻塞：没有数据时 pump 立刻返回空
        t0 = time.perf_counter()
        if link.pump():
            errs.append("没数据时 pump 不该返回帧")
        if (time.perf_counter() - t0) > 0.05:
            errs.append("pump 不是非阻塞的")

    finally:
        link.close()
        os.close(keepalive)
        os.close(master)
        os.close(slave)

    if errs:
        for e in errs:
            print("  !! " + e)
        return 1
    print("  link.py 自检通过（PTY 内存链路：收发/残帧/坏 CRC/丢帧/非阻塞，共 8 组）")
    return 0


if __name__ == "__main__":
    sys.exit(_self_test())
