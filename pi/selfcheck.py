#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RL-CAT 协议实现自检（PC 端，不需要硬件）

⚠️ 目的：证明【Python 侧】的协议实现与【固件侧】的 C 实现是同一套东西。
   固件侧的单测在 firmware/tests/host/test_link_proto.c 里逐一断言字节偏移；
   这里用同一批期望值再断言一遍。两端都过，才说明协议真的对齐了。

运行：
    python pi/selfcheck.py
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

import math
import struct
import sys
from typing import List

sys.path.insert(0, __file__.rsplit("\\", 1)[0].rsplit("/", 1)[0])

from rlcat_proto import (  # noqa: E402
    crc16, pack, unpack, FrameDecoder,
    make_cmd_joint, parse_cmd_joint, make_cmd_mode, make_cmd_calib,
    make_heartbeat, encode_tel_fast, parse_tel_fast, parse_tel_status,
    build_observation, action_to_q_target, quat_to_gravity,
    FRAME_OVERHEAD, MAX_PAYLOAD, LEN_CMD_JOINT, LEN_TEL_FAST, LEN_TEL_STATUS,
    TYPE_CMD_JOINT, TYPE_TEL_FAST, TYPE_TEL_STATUS, TYPE_HEARTBEAT,
    MODE_ACTIVE, MODE_IDLE, MODE_ESTOP, FLAG_ENABLE,
    Q_UNIT, ACTION_SCALE, OBS_DIM, N_JOINTS, GAIT_FREQ_HZ, GAIT_UNIT,
    TelFast, TelStatus,
)

# ---------------------------------------------------------------------------
# 测试框架
# ---------------------------------------------------------------------------

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


def near(a: float, b: float, tol: float, what: str) -> None:
    check(abs(a - b) <= tol, "%s (got %.6f, want %.6f +- %g)" % (what, a, b, tol))


def section(name: str) -> None:
    print("\n[%s]" % name)


# ---------------------------------------------------------------------------
# 1. CRC（协议 §10 测试 1）
# ---------------------------------------------------------------------------

def test_crc() -> None:
    section("CRC-16/CCITT-FALSE")
    eq(crc16(b"123456789"), 0x29B1, "standard vector")
    eq(crc16(b""), 0xFFFF, "empty input = init value")
    check(crc16(b"\x00") != crc16(b"\x01"), "different data -> different crc")


# ---------------------------------------------------------------------------
# 2. 帧往返 + 边界（协议 §10 测试 2~8）
# ---------------------------------------------------------------------------

def test_frame() -> None:
    section("frame codec")

    # 测试 2：空负载
    f = pack(TYPE_HEARTBEAT, 7)
    eq(len(f), 7, "HEARTBEAT frame length")
    eq(list(f[:2]), [0xAA, 0x55], "header bytes")
    fr = unpack(f)
    check(fr is not None and fr.type == TYPE_HEARTBEAT and fr.seq == 7
          and len(fr.payload) == 0, "HEARTBEAT roundtrip")

    # 测试 3：最大负载
    payload = bytes((i * 7 + 3) & 0xFF for i in range(MAX_PAYLOAD))
    f = pack(0x42, 200, payload)
    eq(len(f), FRAME_OVERHEAD + MAX_PAYLOAD, "max frame length")
    fr = unpack(f)
    check(fr is not None and fr.payload == payload, "max payload roundtrip")

    # 实际尺寸
    eq(len(make_cmd_joint([0.0] * N_JOINTS)), 38, "CMD_JOINT frame length")
    eq(len(pack(TYPE_TEL_FAST, 0, bytes(LEN_TEL_FAST))), 103, "TEL_FAST frame length")
    eq(len(pack(TYPE_TEL_STATUS, 0, bytes(LEN_TEL_STATUS))), 47, "TEL_STATUS frame length")

    # 测试 4：CRC 错
    bad = bytearray(f)
    bad[-1] ^= 0xFF
    check(unpack(bytes(bad)) is None, "corrupted CRC must be rejected")

    # 测试 5：截断
    check(unpack(f[:5]) is None, "truncated frame must be rejected")

    # 测试 8：LEN 超限
    over = bytes([0xAA, 0x55, 0xFF, 0, 0]) + bytes(10)
    check(unpack(over) is None, "LEN=255 must be rejected")

    # 容量检查
    try:
        pack(0x01, 0, bytes(MAX_PAYLOAD + 1))
        check(False, "oversized payload must raise")
    except ValueError:
        check(True, "")


def test_stream_decoder() -> None:
    section("stream decoder state machine")

    frames = [pack(0x11, i, bytes([1, 2, 3, 4, 5, 6, 7, 8])) for i in range(10)]
    stream = b"".join(frames)

    # 逐字节喂
    d = FrameDecoder()
    got = []
    for b in stream:
        got += d.feed(bytes([b]), 1000)
    eq(len(got), 10, "byte-wise frame count")
    eq(d.n_ok, 10, "decoder ok counter")
    eq(d.n_crc_err, 0, "decoder crc counter")

    # 测试 6：AA AA 55
    d = FrameDecoder()
    got = d.feed(b"\xAA\xAA\x55" + stream[2:], 1000)
    eq(len(got), 10, "AA AA 55 must still sync")

    # 测试 7：前导垃圾
    d = FrameDecoder()
    got = d.feed(bytes([0x00, 0x01, 0xFF, 0xAA, 0x00, 0x55, 0x13, 0x99, 0xAA]) + stream, 1000)
    eq(len(got), 10, "must resync after garbage")

    # 测试 8：LEN=255 被拒 + 状态机复位
    d = FrameDecoder()
    got = d.feed(b"\xAA\x55\xFF" + stream, 1000)
    eq(d.n_len_err, 1, "LEN error counted")

    # 测试 10：随机字节流不崩
    import random
    rnd = random.Random(0x12345678)
    d = FrameDecoder()
    blob = bytes(rnd.randrange(256) for _ in range(1 << 17))   # 128 KB
    n = len(d.feed(blob, 1000))
    print("    random 128 KB -> %d spurious frames accepted" % n)
    check(d.n_bytes == len(blob), "all bytes counted")

    # 整帧超时
    d = FrameDecoder(frame_timeout_ms=5)
    d.feed(b"\xAA\x55\x08\x00", 1000)
    check(not d.tick(1005), "no timeout exactly at limit")
    check(d.tick(1006), "timeout past the limit")
    eq(d.n_timeout, 1, "timeout counter")
    check(not d.tick(999999), "idle decoder must not report timeout")


# ---------------------------------------------------------------------------
# 3. CMD_JOINT（协议 §4.1）—— 与 C 侧同一批期望字节
# ---------------------------------------------------------------------------

def test_cmd_joint() -> None:
    section("CMD_JOINT bytes (must match the C side)")

    q = [0.0] * N_JOINTS
    q[0] = 0.25
    q[1] = -0.25
    f = make_cmd_joint(q, mode=MODE_ACTIVE, flags=FLAG_ENABLE, seq=0)
    p = f[5:5 + LEN_CMD_JOINT]

    eq(p[0], MODE_ACTIVE, "mode byte")
    eq(p[1], FLAG_ENABLE, "flags byte")
    # 2500 = 0x09C4 → 小端 C4 09
    eq(p[2], 0xC4, "q[0] low byte")
    eq(p[3], 0x09, "q[0] high byte")
    # -2500 = 0xF63C → 小端 3C F6
    eq(p[4], 0x3C, "q[1] low byte")
    eq(p[5], 0xF6, "q[1] high byte")
    eq(p[26], 0x00, "reserved")
    eq(p[27], 0x64, "kp_scale low")
    eq(p[28], 0x00, "kp_scale high")
    eq(p[29], 0x01, "led")

    # 往返
    c = parse_cmd_joint(p)
    eq(c.mode, MODE_ACTIVE, "parsed mode")
    near(c.q_target[0], 0.25, 1e-6, "parsed q[0]")
    near(c.q_target[1], -0.25, 1e-6, "parsed q[1]")

    # 量程
    eq(struct.unpack_from("<h", p, 2)[0], 2500, "q[0] encoded value")

    # CMD_MODE / CMD_CALIB 尺寸
    eq(len(make_cmd_mode(MODE_IDLE)), 9, "CMD_MODE frame length")
    eq(len(make_cmd_calib(0)), 8, "CMD_CALIB frame length")


# ---------------------------------------------------------------------------
# 4. TEL_FAST 96 字节布局（与 C 侧 RLCA_TF_OFF_* 一一对应）
# ---------------------------------------------------------------------------

def test_tel_fast() -> None:
    section("TEL_FAST layout (must match C RLCA_TF_OFF_*)")

    t = TelFast()
    for i in range(N_JOINTS):
        t.q[i] = (i + 1) * 0.0100          # -> 100,200,...,1200
        t.qd[i] = (i + 1) * 0.1000         # -> 1000,2000,...
        t.tau[i] = (i + 1) * 0.0010        # -> 1,2,...,12
    t.quat = (1.0, -0.5, 0.25, 0.125)      # -> 10000, -5000, 2500, 1250
    t.gyro = (1.0, -0.5, 0.125)            # -> 1000, -500, 125
    t.gait_sin = 32767 * GAIT_UNIT
    t.gait_cos = -16384 * GAIT_UNIT
    t.cycle_cnt = 0x1234
    t.voltage_mv = 11500
    t.status = 0x0F
    t.err_flags = 0xA5

    p = encode_tel_fast(t)
    eq(len(p), LEN_TEL_FAST, "payload length")
    eq(len(p), 96, "payload length literal")

    # 逐偏移核对 —— 与 test_link_proto.c 完全相同的期望值
    eq(struct.unpack_from("<12h", p, 0)[0], 100, "offset 0 q[0]")
    eq(struct.unpack_from("<12h", p, 0)[11], 1200, "offset 22 q[11]")
    eq(struct.unpack_from("<12h", p, 24)[0], 1000, "offset 24 qd[0]")
    eq(struct.unpack_from("<12h", p, 48)[0], 1, "offset 48 tau[0]")
    eq(struct.unpack_from("<4h", p, 72), (10000, -5000, 2500, 1250), "offset 72 quat")
    eq(struct.unpack_from("<3h", p, 80), (1000, -500, 125), "offset 80 gyro")
    eq(struct.unpack_from("<h", p, 86)[0], 32767, "offset 86 gait_sin")
    eq(struct.unpack_from("<h", p, 88)[0], -16384, "offset 88 gait_cos")
    eq(struct.unpack_from("<H", p, 90)[0], 0x1234, "offset 90 cycle_cnt")
    eq(struct.unpack_from("<H", p, 92)[0], 11500, "offset 92 voltage")
    eq(p[94], 0x0F, "offset 94 status")
    eq(p[95], 0xA5, "offset 95 err_flags")

    # 往返
    back = parse_tel_fast(p)
    near(back.q[0], 0.01, 1e-6, "roundtrip q[0]")
    near(back.qd[0], 0.1, 1e-6, "roundtrip qd[0]")
    near(back.tau[0], 0.001, 1e-6, "roundtrip tau[0]")
    eq(back.cycle_cnt, 0x1234, "roundtrip cycle_cnt")
    eq(back.voltage_mv, 11500, "roundtrip voltage")
    eq(back.status, 0x0F, "roundtrip status")
    eq(back.err_flags, 0xA5, "roundtrip err_flags")
    near(back.gait_sin, 1.0, 1e-3, "roundtrip gait_sin")

    # 饱和
    t2 = TelFast()
    t2.tau[0] = 100.0          # 100/0.001 = 100000 -> 饱和
    t2.tau[1] = -100.0
    p2 = encode_tel_fast(t2)
    eq(struct.unpack_from("<h", p2, 48)[0], 32767, "tau saturation high")
    eq(struct.unpack_from("<h", p2, 48 + 2)[0], -32768, "tau saturation low")

    # NaN -> 0
    t3 = TelFast()
    t3.q[0] = float("nan")
    p3 = encode_tel_fast(t3)
    eq(struct.unpack_from("<h", p3, 0)[0], 0, "NaN must encode to 0")


def test_tel_status() -> None:
    section("TEL_STATUS layout")
    temps = [30 + i for i in range(N_JOINTS)]
    p = bytearray(LEN_TEL_STATUS)
    struct.pack_into("<I", p, 0, 123456)
    struct.pack_into("<H", p, 4, 11234)
    struct.pack_into("<12h", p, 6, *temps)
    struct.pack_into("<H", p, 30, 3)
    struct.pack_into("<H", p, 32, 7)
    struct.pack_into("<H", p, 34, 11)
    p[36] = MODE_ACTIVE
    p[37] = 0x0F
    p[38] = 0x01
    p[39] = 42

    s = parse_tel_status(bytes(p))
    eq(s.uptime_ms, 123456, "uptime")
    eq(s.voltage_mv, 11234, "voltage")
    eq(s.temperature, temps, "temperatures")
    eq(s.rx_overflow, 3, "rx_overflow")
    eq(s.bus_err, 7, "bus_err")
    eq(s.bus_timeout, 11, "bus_timeout")
    eq(s.mode, MODE_ACTIVE, "mode")
    eq(s.status, 0x0F, "status")
    eq(s.err_flags, 0x01, "err_flags")
    eq(s.seq_lost, 42, "seq_lost")


# ---------------------------------------------------------------------------
# 5. 47 维观测组装（与 rlcat_env.py 对齐）
# ---------------------------------------------------------------------------

def test_observation() -> None:
    section("47-dim observation assembly")

    t = TelFast()
    t.quat = (1.0, 0.0, 0.0, 0.0)
    t.gyro = (0.4, -0.8, 0.12)
    for i in range(N_JOINTS):
        t.q[i] = 0.01 * i
        t.qd[i] = 1.0
    t.gait_sin = 0.5
    t.gait_cos = -0.5

    last_action = [0.1 * i for i in range(N_JOINTS)]
    obs = build_observation(t, 0.3, -0.1, 0.2, last_action)

    eq(len(obs), OBS_DIM, "observation dimension")
    eq(len(obs), 47, "observation dimension literal")

    # 0-2：单位四元数 → 重力应为 (0,0,-1)
    near(obs[0], 0.0, 1e-6, "gravity x")
    near(obs[1], 0.0, 1e-6, "gravity y")
    near(obs[2], -1.0, 1e-6, "gravity z")

    # 3-5：角速度 ×0.25
    near(obs[3], 0.1, 1e-9, "ang vel x scaled")
    near(obs[4], -0.2, 1e-9, "ang vel y scaled")
    near(obs[5], 0.03, 1e-9, "ang vel z scaled")

    # 6-8：速度指令原样
    near(obs[6], 0.3, 1e-9, "cmd vx")
    near(obs[7], -0.1, 1e-9, "cmd vy")
    near(obs[8], 0.2, 1e-9, "cmd wz")

    # 9-20：关节位置（相对 q_default = 0）
    for i in range(N_JOINTS):
        near(obs[9 + i], t.q[i], 1e-9, "q[%d]" % i)

    # 21-32：关节速度 ×0.1（⚠️ upstream 是 0.05，本项目 0.1）
    for i in range(N_JOINTS):
        near(obs[21 + i], 0.1, 1e-9, "qd[%d] scaled" % i)

    # 33-44：上一帧动作 ×4.0
    for i in range(N_JOINTS):
        near(obs[33 + i], last_action[i] * 4.0, 1e-9, "action[%d] scaled" % i)

    # 45-46：步态相位原样
    near(obs[45], 0.5, 1e-9, "gait sin")
    near(obs[46], -0.5, 1e-9, "gait cos")

    # 动作 → 目标角
    qt = action_to_q_target([1.0] * N_JOINTS)
    for i in range(N_JOINTS):
        near(qt[i], ACTION_SCALE, 1e-9, "q_target[%d]" % i)

    # 相位步长必须与训练侧一致
    step = GAIT_FREQ_HZ / 100.0
    near(step, 0.03, 1e-9, "phase step per control cycle")

    # 四元数→重力：绕 Y 轴 90° 应把重力转到 X 方向
    import math
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    g = quat_to_gravity((c, 0.0, s, 0.0))
    near(abs(g[2]), 0.0, 1e-6, "gravity z after 90deg pitch")
    near(abs(g[0]), 1.0, 1e-6, "gravity tilted into x")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> int:
    print("========================================")
    print(" RL-CAT Python protocol self-check")
    print("========================================")

    test_crc()
    test_frame()
    test_stream_decoder()
    test_cmd_joint()
    test_tel_fast()
    test_tel_status()
    test_observation()

    print("\n========================================")
    print(" PASS %d   FAIL %d" % (_pass, _fail))
    print("========================================")
    return 0 if _fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
