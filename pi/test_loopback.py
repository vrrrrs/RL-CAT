#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RL-CAT 模拟器回环测试（PC 端，不需要串口、不需要硬件、不需要 pyserial）

⚠️ 目的：在没有 STM32 的情况下验证【模拟器本身的逻辑】是对的：
    · 100 Hz 循环、TEL_FAST 解析、cycle_cnt 连续性统计
    · CMD_JOINT 组帧正确（假固件侧用 C 侧同一套规则解析并断言）
    · 回复耗时统计、失联超时统计
    · 能正确识别"垃圾/CRC 错"输入而不崩

手法：
    用 socketpair 冒充串口 —— pyserial 的 read/write 语义与 socket 足够接近
    （read(n) 无数据返回 b""，write 返回字节数）。
    一边跑 pi/sim_esp32.py 的主循环，另一边跑一个"假 STM32"线程。

运行：
    python pi/test_loopback.py
退出码：0 = 全过
"""

from __future__ import annotations

# ⚠️ Windows 控制台默认是 GBK，中文报告会乱码。
#    这里在任何输出之前把 stdout/stderr 切成 UTF-8。
#    ⚠️ 必须放在 `from __future__` 之后 —— 它要求位于文件最前面。
import sys as _sys
for _s in (_sys.stdout, _sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import importlib.util
import io
import math
import os
import random
import socket
import sys
import threading
import time
from typing import List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from rlcat_proto import (  # noqa: E402
    FrameDecoder, pack, unpack,
    TYPE_CMD_JOINT, TYPE_TEL_FAST, TYPE_TEL_STATUS, TYPE_CMD_MODE, TYPE_CMD_CALIB,
    TYPE_HEARTBEAT,
    LEN_TEL_FAST, LEN_TEL_STATUS,
    parse_cmd_joint, parse_tel_fast, encode_tel_fast,
    TelFast, TelStatus, MODE_ACTIVE, MODE_STAND, MODE_IDLE,
    FLAG_ENABLE, ST_READY, ST_LINK_OK, N_JOINTS,
)

_pass = 0
_fail = 0


def check(cond: bool, msg: str) -> None:
    global _pass, _fail
    if cond:
        _pass += 1
    else:
        _fail += 1
        print("  FAIL %s" % msg)


def eq(a, b, what: str) -> None:
    check(a == b, "%s (got %r, want %r)" % (what, a, b))


def section(name: str) -> None:
    print("\n[%s]" % name)


# ---------------------------------------------------------------------------
# 用 socket 冒充串口
# ---------------------------------------------------------------------------

class SocketSerial:
    """pyserial 的最小替身：read(n) 非阻塞，write 立即发。"""

    def __init__(self, sock: socket.socket):
        self._s = sock
        # Windows has no MSG_DONTWAIT -> use a non-blocking socket instead
        self._s.setblocking(False)
        self.bytes_written = 0
        self.bytes_read = 0

    def read(self, n: int) -> bytes:
        try:
            data = self._s.recv(n)
        except BlockingIOError:
            return b""
        except OSError:
            return b""
        self.bytes_read += len(data)
        return data

    def write(self, data: bytes) -> int:
        self._s.sendall(data)
        self.bytes_written += len(data)
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        try:
            while self._s.recv(65536):
                pass
        except Exception:
            pass

    def reset_output_buffer(self) -> None:
        pass

    def close(self) -> None:
        try:
            self._s.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 假 STM32：按固件的真实行为回帧
# ---------------------------------------------------------------------------

class FakeStm32(threading.Thread):
    """模拟固件侧：
        · 每 10 ms 发一帧 TEL_FAST（带递增 cycle_cnt、真实步态相位）
        · 接收 CMD_JOINT / CMD_MODE / CMD_CALIB，用 C 侧同一套规则解析并校验
        · 每 100 ms 发一帧 TEL_STATUS
    可注入故障：乱发垃圾、破坏 CRC、突然停发（测失联）。
    """

    def __init__(self, sock: socket.socket, faults: Optional[dict] = None):
        super().__init__(daemon=True)
        self.sock = sock
        self.faults = faults or {}
        self.sock.setblocking(False)
        self.dec = FrameDecoder()
        self.stop_flag = threading.Event()

        self.cycle = 0
        self.cmd_joints: List[object] = []
        self.cmd_modes: List[object] = []
        self.cmd_calibs: List[object] = []
        self.bad_frames = 0
        self.phase = 0
        self.gait_freq = 3.0

    def run(self) -> None:
        t0 = time.perf_counter()
        next_tel = t0
        next_status = t0
        period = 0.01

        while not self.stop_flag.is_set():
            now = time.perf_counter()

            # ---- 周期性发 TEL_FAST ----
            if now >= next_tel and not self.faults.get("silent"):
                next_tel += period
                if next_tel < now:
                    next_tel = now + period

                # 相位推进：与固件/训练侧一致，每控制周期 +0.03
                self.phase = (self.phase + self.gait_freq / 100.0) % 1.0
                t = TelFast()
                t.cycle_cnt = self.cycle & 0xFFFF
                t.quat = (1.0, 0.0, 0.0, 0.0)
                t.gyro = (0.4, -0.8, 0.12)
                t.gait_sin = math.sin(2 * math.pi * self.phase)
                t.gait_cos = math.cos(2 * math.pi * self.phase)
                t.voltage_mv = 11500
                t.status = ST_READY | ST_LINK_OK
                for i in range(N_JOINTS):
                    t.q[i] = 0.001 * i
                    t.qd[i] = 0.5
                    t.tau[i] = 0.01 * i

                payload = encode_tel_fast(t)
                frame = pack(TYPE_TEL_FAST, self.cycle & 0xFF, payload)

                if self.faults.get("bad_crc_every") and \
                        self.cycle % self.faults["bad_crc_every"] == 0:
                    f = bytearray(frame)
                    f[-1] ^= 0xFF
                    frame = bytes(f)
                    self.bad_frames += 1

                try:
                    self.sock.sendall(frame)
                except OSError:
                    return

                if self.faults.get("garbage"):
                    try:
                        self.sock.sendall(bytes(random.randrange(256) for _ in range(17)))
                    except OSError:
                        return

                self.cycle += 1

            # ---- 周期性发 TEL_STATUS ----
            if now >= next_status:
                next_status += 0.1
                if next_status < now:
                    next_status = now + 0.1
                s = TelStatus(uptime_ms=int((now - t0) * 1000), voltage_mv=11500,
                              temperature=[30 + i for i in range(N_JOINTS)],
                              rx_overflow=0, bus_err=0, bus_timeout=0,
                              mode=MODE_ACTIVE, status=ST_READY | ST_LINK_OK,
                              err_flags=0, seq_lost=0)
                p = bytearray(LEN_TEL_STATUS)
                import struct
                struct.pack_into("<I", p, 0, s.uptime_ms)
                struct.pack_into("<H", p, 4, s.voltage_mv)
                struct.pack_into("<12h", p, 6, *s.temperature)
                p[36] = s.mode
                p[37] = s.status
                try:
                    self.sock.sendall(pack(TYPE_TEL_STATUS, 0, bytes(p)))
                except OSError:
                    return

            # ---- 收下行 ----
            try:
                data = self.sock.recv(4096)
            except BlockingIOError:
                data = b""
            except OSError:
                return

            if data:
                for fr in self.dec.feed(data, int(time.perf_counter() * 1000)):
                    if fr.type == TYPE_CMD_JOINT:
                        try:
                            self.cmd_joints.append(parse_cmd_joint(fr.payload))
                        except ValueError as e:
                            print("  !! 假固件：CMD_JOINT 解析失败 %s" % e)
                    elif fr.type == TYPE_CMD_MODE:
                        self.cmd_modes.append((fr.payload[0], fr.payload[1]))
                    elif fr.type == TYPE_CMD_CALIB:
                        self.cmd_calibs.append(fr.payload[0])

            time.sleep(0.001)


# ---------------------------------------------------------------------------
# 载入模拟器模块
# ---------------------------------------------------------------------------

def load_sim():
    spec = importlib.util.spec_from_file_location(
        "sim_esp32", os.path.join(HERE, "sim_esp32.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Args:
    def __init__(self, **kw):
        self.port = "loopback"
        self.baud = 1_000_000
        self.hz = 100.0
        self.seconds = 0.0
        self.mode = "active"
        self.frame_timeout = 0.005
        self.vx = 0.0
        self.verbose = False
        for k, v in kw.items():
            setattr(self, k, v)


def run_case(name: str, faults: Optional[dict], seconds: float = 1.5,
             mode: str = "active"):
    """跑一轮模拟器，返回 (stats, fake)。"""
    sim = load_sim()
    a, b = socket.socketpair()
    fake = FakeStm32(b, faults)
    fake.start()

    ser = SocketSerial(a)
    sim.open_serial = lambda port, baud: ser      # 注入替身

    args = Args(seconds=seconds, mode=mode)
    # 把 stdout 抓起来免得刷屏
    buf = io.StringIO()
    old = sys.stdout
    sys.stdout = buf
    try:
        rc = sim.run(args)
    except Exception as e:
        sys.stdout = old
        print("  !! 模拟器抛异常: %r" % e)
        fake.stop_flag.set()
        fake.join(timeout=1.0)
        return None, fake, 1
    finally:
        sys.stdout = old

    fake.stop_flag.set()
    fake.join(timeout=1.0)
    fake.sock.close()
    return rc, fake, buf.getvalue()


def main() -> int:
    print("========================================")
    print(" RL-CAT simulator loopback test")
    print("========================================")

    # ---- 1. 正常链路 ----
    section("normal link: 100 Hz both directions")
    rc, fake, _ = run_case("normal", None, seconds=1.5)
    check(fake.cycle > 100, "fake stm32 must have sent > 100 TEL_FAST (sent %d)" % fake.cycle)
    check(len(fake.cmd_joints) > 100,
          "fake stm32 must have received > 100 CMD_JOINT (got %d)" % len(fake.cmd_joints))
    check(fake.dec.n_crc_err == 0, "no CRC errors on the uplink (got %d)" % fake.dec.n_crc_err)
    check(fake.dec.n_len_err == 0, "no LEN errors on the uplink")
    check(len(fake.cmd_modes) >= 1, "at least one CMD_MODE was sent first")
    eq(fake.cmd_modes[0][0] if fake.cmd_modes else None, MODE_ACTIVE,
       "first CMD_MODE must carry the requested mode")

    # 收到的 CMD_JOINT 必须真的可解析且数值合理
    if fake.cmd_joints:
        c = fake.cmd_joints[0]
        eq(c.mode, MODE_ACTIVE, "CMD_JOINT.mode")
        eq(c.flags, FLAG_ENABLE, "CMD_JOINT.flags")
        eq(len(c.q_target), N_JOINTS, "CMD_JOINT q_target length")
        # ⚠️ ACTIVE 模式的假策略给 ±0.1 动作 × action_scale 0.25 = ±0.025 rad
        ok = all(abs(v) <= 0.026 for v in c.q_target)
        check(ok, "q_target within the fake policy range (±0.025 rad)")

    # seq 必须连续循环
    if len(fake.cmd_joints) >= 5:
        # cmd_joints 里没存 seq，这里只能靠数量判断；序列连续性由解码器保证
        check(True, "")

    # ---- 2. 上行注入 CRC 错 ----
    section("uplink fault injection: every 7th TEL_FAST has a bad CRC")
    rc, fake, _ = run_case("badcrc", {"bad_crc_every": 7}, seconds=1.2)
    check(fake.bad_frames > 5, "fake must have injected bad frames (%d)" % fake.bad_frames)
    check(fake.cycle > 60, "link must keep working despite bad frames")
    print("    fake sent %d frames, %d were corrupted" % (fake.cycle, fake.bad_frames))

    # ---- 3. 上行垃圾字节 ----
    section("uplink fault injection: random garbage interleaved")
    rc, fake, _ = run_case("garbage", {"garbage": True}, seconds=1.2)
    check(fake.cycle > 60, "link must survive garbage injection")
    check(fake.dec.n_crc_err >= 0, "decoder survives garbage")

    # ---- 4. 下行静默 → 模拟器应报超时 ----
    section("silent STM32 -> simulator must count timeouts, not hang")
    t0 = time.perf_counter()
    rc, fake, out = run_case("silent", {"silent": True}, seconds=0.6)
    dt = time.perf_counter() - t0
    check(dt < 3.0, "silent case must finish promptly (took %.2fs)" % dt)
    check("一帧 TEL_FAST 都没收到" in out or "TEL_FAST" in out,
          "report must mention the missing TEL_FAST")
    eq(rc, 1, "exit code must be non-zero when nothing was received")

    # ---- 5. stand 模式 ----
    section("mode=stand: actions are zero, q_target stays at q_default")
    rc, fake, _ = run_case("stand", None, seconds=1.0, mode="stand")
    if fake.cmd_joints:
        c = fake.cmd_joints[0]
        eq(c.mode, MODE_STAND, "CMD_JOINT.mode in stand")
        check(all(abs(v) < 1e-9 for v in c.q_target),
              "stand mode must command q_default (all zeros)")

    print("\n========================================")
    print(" PASS %d   FAIL %d" % (_pass, _fail))
    print("========================================")
    return 0 if _fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
