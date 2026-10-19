#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RL-CAT 协议实现（树莓派侧 / PC 侧通用）

⚠️ 这是【两端共用】的协议实现：树莓派运行时（docs/31-pi-runtime.md）与
   PC 端模拟器都 import 它。好处是"协议只有一份实现"，不会出现
   Pi 侧和模拟器各自一套、各自有 bug 的情况。

本文件严格对齐 firmware/doc/协议规范_树莓派通信.md：
  §2   帧格式      0xAA 0x55 | LEN | SEQ | TYPE | PAYLOAD | CRC16(LE)
  §4.1 CMD_JOINT   LEN=31
  §4.4 TEL_FAST    LEN=96
  §4.8 TEL_STATUS  LEN=40
  §5   CRC-16/CCITT-FALSE，测试向量 "123456789" -> 0x29B1

⚠️ CRC 覆盖范围：从 LEN（偏移 2）到 PAYLOAD 末尾，共 LEN + 3 字节。
   不包含两个头字节。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ============================================================================
# 常量
# ============================================================================

HEAD0 = 0xAA
HEAD1 = 0x55
FRAME_OVERHEAD = 7          # 头2 + LEN + SEQ + TYPE + CRC2
MAX_PAYLOAD = 128

# 消息类型
TYPE_CMD_JOINT = 0x01
TYPE_TEL_FAST = 0x02
TYPE_CMD_MODE = 0x03
TYPE_TEL_STATUS = 0x04
TYPE_CMD_CALIB = 0x05
TYPE_TEL_LOG = 0x06
TYPE_TEL_ACK = 0x07
TYPE_HEARTBEAT = 0x7F

# 负载长度
LEN_CMD_JOINT = 31
LEN_TEL_FAST = 96
LEN_CMD_MODE = 2
LEN_TEL_STATUS = 40
LEN_CMD_CALIB = 1
LEN_TEL_ACK = 2

# 关节
N_JOINTS = 12

# 运行模式（协议 §4.2）
MODE_IDLE = 0
MODE_STAND = 1
MODE_ACTIVE = 2
MODE_DAMP = 3
MODE_ESTOP = 4

# CMD_JOINT.flags（协议 §4.3）
FLAG_ENABLE = 0x01
FLAG_RESET_ERR = 0x02
FLAG_REQ_STATUS = 0x04
FLAG_REQ_CALIB = 0x08

# status 位域（协议 §4.5）
ST_READY = 0x01
ST_CALIBRATED = 0x02
ST_LINK_OK = 0x04
ST_IMU_OK = 0x08
ST_BATT_WARN = 0x10
ST_BATT_STOP = 0x20
ST_BATT_FORCE = 0x40
ST_ESTOP = 0x80

# err_flags 位域（协议 §4.6）
ERRF_VOLTAGE = 0x01
ERRF_ANGLE = 0x02
ERRF_OVERHEAT = 0x04
ERRF_OVERCURRENT = 0x08
ERRF_ANGLE_LIMIT = 0x10
ERRF_OVERLOAD = 0x20
ERRF_BUS_TIMEOUT = 0x40
ERRF_CALIB_INVALID = 0x80

# 编码单位（协议 §4.1 / §4.4）
Q_UNIT = 0.0001          # rad
QD_UNIT = 0.0001         # rad/s
TAU_UNIT = 0.001         # N·m
QUAT_UNIT = 0.0001
GYRO_UNIT = 0.001        # rad/s
GAIT_UNIT = 1.0 / 32767.0

# 观测缩放（docs/30-firmware.md §5.1，⚠️ 必须与训练侧一致）
OBS_SCALE_ANG_VEL = 0.25
OBS_SCALE_DOF_VEL = 0.1
OBS_SCALE_ACTION = 4.0
OBS_DIM = 47
ACTION_SCALE = 0.25
GAIT_FREQ_HZ = 3.0       # rlcat_config.py 的 control.gait_freq


# ============================================================================
# CRC-16/CCITT-FALSE（协议 §5）
# ============================================================================

def _build_crc_table() -> List[int]:
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _build_crc_table()


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE。测试向量：crc16(b"123456789") == 0x29B1"""
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


# ============================================================================
# 组帧 / 解帧（协议 §2）
# ============================================================================

def pack(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    """组一个完整帧。"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload %d > %d" % (len(payload), MAX_PAYLOAD))
    body = bytes([len(payload), seq & 0xFF, msg_type & 0xFF]) + payload
    return bytes([HEAD0, HEAD1]) + body + struct.pack("<H", crc16(body))


@dataclass
class Frame:
    type: int
    seq: int
    payload: bytes


def unpack(buf: bytes) -> Optional[Frame]:
    """解一个完整帧；失败返回 None。

    ⚠️ 这是"单帧"接口。字节流要用 FrameDecoder。
    """
    if len(buf) < FRAME_OVERHEAD:
        return None
    if buf[0] != HEAD0 or buf[1] != HEAD1:
        return None
    ln = buf[2]
    if ln > MAX_PAYLOAD:
        return None
    total = FRAME_OVERHEAD + ln
    if len(buf) < total:
        return None
    body = buf[2:5 + ln]
    crc_rx = struct.unpack_from("<H", buf, 5 + ln)[0]
    if crc16(body) != crc_rx:
        return None
    return Frame(type=buf[4], seq=buf[3], payload=buf[5:5 + ln])


class FrameDecoder:
    """字节流 → 帧的接收状态机。

    ⚠️ 必须与固件侧（rlca_frame.c）的行为一致，三处容错：
       ① WAIT_HEAD1 收到 0xAA 时留在本状态（处理 AA AA 55）
       ② LEN > 128 直接丢弃并复位
       ③ 收半个帧后超时复位

    用法：
        d = FrameDecoder()
        for fr in d.feed(chunk):        # 产出 0..n 个完整帧
            ...
        d.tick(now_ms)                  # 主循环里调，处理超时
    """

    WAIT_HEAD0, WAIT_HEAD1, HEADER, PAYLOAD, CRC0, CRC1 = range(6)

    def __init__(self, frame_timeout_ms: int = 5):
        self.frame_timeout_ms = frame_timeout_ms
        self.reset()
        # 诊断计数（与固件 TEL_STATUS 的字段对应）
        self.n_ok = 0
        self.n_crc_err = 0
        self.n_len_err = 0
        self.n_timeout = 0
        self.n_bytes = 0

    def reset(self) -> None:
        self.state = self.WAIT_HEAD0
        self._len = 0
        self._seq = 0
        self._type = 0
        self._idx = 0
        self._payload = bytearray()
        self._crc_lo = 0
        self._last_byte_ms = 0

    def feed(self, data: bytes, now_ms: int = 0) -> List[Frame]:
        out: List[Frame] = []
        for b in data:
            r = self._feed_byte(b, now_ms)
            if r is not None:
                out.append(r)
        return out

    def tick(self, now_ms: int) -> bool:
        """整帧超时检查。返回 True = 发生了超时复位。"""
        if self.state == self.WAIT_HEAD0:
            return False
        if now_ms - self._last_byte_ms > self.frame_timeout_ms:
            self.n_timeout += 1
            self.reset()
            return True
        return False

    def _feed_byte(self, b: int, now_ms: int) -> Optional[Frame]:
        self.n_bytes += 1
        self._last_byte_ms = now_ms

        if self.state == self.WAIT_HEAD0:
            if b == HEAD0:
                self.state = self.WAIT_HEAD1

        elif self.state == self.WAIT_HEAD1:
            if b == HEAD1:
                self.state = self.HEADER
                self._idx = 0
            elif b == HEAD0:
                pass                       # ⚠️ 容错①：AA AA 55
            else:
                self.state = self.WAIT_HEAD0

        elif self.state == self.HEADER:
            if self._idx == 0:
                if b > MAX_PAYLOAD:        # ⚠️ 容错②
                    self.n_len_err += 1
                    self.reset()
                    return None
                self._len = b
            elif self._idx == 1:
                self._seq = b
            else:
                self._type = b
            self._idx += 1
            if self._idx >= 3:
                self._idx = 0
                self._payload = bytearray()
                self.state = self.PAYLOAD if self._len > 0 else self.CRC0

        elif self.state == self.PAYLOAD:
            self._payload.append(b)
            self._idx += 1
            if self._idx >= self._len:
                self.state = self.CRC0

        elif self.state == self.CRC0:
            self._crc_lo = b
            self.state = self.CRC1

        elif self.state == self.CRC1:
            crc_rx = self._crc_lo | (b << 8)
            body = bytes([self._len, self._seq, self._type]) + bytes(self._payload)
            if crc16(body) == crc_rx:
                self.n_ok += 1
                fr = Frame(type=self._type, seq=self._seq, payload=bytes(self._payload))
                self.reset()
                return fr
            self.n_crc_err += 1
            self.reset()

        return None


# ============================================================================
# CMD_JOINT（协议 §4.1）
# ============================================================================

def encode_q(q_rad: float) -> int:
    """关节角 → int16（0.0001 rad/单位），带饱和。"""
    if q_rad != q_rad:                     # NaN
        return 0
    v = int(round(q_rad / Q_UNIT))
    return max(-32768, min(32767, v))


def make_cmd_joint(q_target: List[float], mode: int = MODE_ACTIVE,
                   flags: int = FLAG_ENABLE, seq: int = 0,
                   kp_scale: int = 100, led: int = 1) -> bytes:
    """组 CMD_JOINT 帧（38 字节）。"""
    if len(q_target) != N_JOINTS:
        raise ValueError("q_target must have %d entries" % N_JOINTS)

    payload = struct.pack("<BB", mode & 0xFF, flags & 0xFF)
    payload += b"".join(struct.pack("<h", encode_q(q)) for q in q_target)
    payload += struct.pack("<B", 0)                    # 26 reserved
    payload += struct.pack("<h", int(kp_scale))        # 27..28
    payload += struct.pack("<B", led & 0xFF)           # 29
    payload += struct.pack("<B", 0)                    # 30 reserved2
    assert len(payload) == LEN_CMD_JOINT, len(payload)
    return pack(TYPE_CMD_JOINT, seq, payload)


@dataclass
class CmdJoint:
    mode: int
    flags: int
    q_target: List[float]
    kp_scale: int
    led: int


def parse_cmd_joint(payload: bytes) -> CmdJoint:
    if len(payload) != LEN_CMD_JOINT:
        raise ValueError("CMD_JOINT payload must be %d bytes" % LEN_CMD_JOINT)
    mode, flags = struct.unpack_from("<BB", payload, 0)
    q = [v * Q_UNIT for v in struct.unpack_from("<12h", payload, 2)]
    kp = struct.unpack_from("<h", payload, 27)[0]
    led = payload[29]
    return CmdJoint(mode=mode, flags=flags, q_target=q, kp_scale=kp, led=led)


def make_cmd_mode(mode: int, param: int = 0, seq: int = 0) -> bytes:
    return pack(TYPE_CMD_MODE, seq, struct.pack("<BB", mode & 0xFF, param & 0xFF))


def make_cmd_calib(action: int, seq: int = 0) -> bytes:
    return pack(TYPE_CMD_CALIB, seq, struct.pack("<B", action & 0xFF))


def make_heartbeat(seq: int = 0) -> bytes:
    return pack(TYPE_HEARTBEAT, seq, b"")


# ============================================================================
# TEL_FAST（协议 §4.4，LEN=96）
# ============================================================================

@dataclass
class TelFast:
    q: List[float] = field(default_factory=lambda: [0.0] * N_JOINTS)
    qd: List[float] = field(default_factory=lambda: [0.0] * N_JOINTS)
    tau: List[float] = field(default_factory=lambda: [0.0] * N_JOINTS)
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)   # w,x,y,z
    gyro: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    gait_sin: float = 0.0
    gait_cos: float = 1.0
    cycle_cnt: int = 0
    voltage_mv: int = 0
    status: int = 0
    err_flags: int = 0


def parse_tel_fast(payload: bytes) -> TelFast:
    """解析 TEL_FAST。

    ⚠️ 这里的偏移【必须】与 firmware 的 RLCA_TF_OFF_* 一致。
       固件侧有单测逐一断言这些偏移（test_link_proto.c），
       本文件有对应的字节级写测（tools/test-python-proto.ps1）。
    """
    if len(payload) != LEN_TEL_FAST:
        raise ValueError("TEL_FAST payload must be %d bytes, got %d"
                         % (LEN_TEL_FAST, len(payload)))

    q = [v * Q_UNIT for v in struct.unpack_from("<12h", payload, 0)]
    qd = [v * QD_UNIT for v in struct.unpack_from("<12h", payload, 24)]
    tau = [v * TAU_UNIT for v in struct.unpack_from("<12h", payload, 48)]
    quat = tuple(v * QUAT_UNIT for v in struct.unpack_from("<4h", payload, 72))
    gyro = tuple(v * GYRO_UNIT for v in struct.unpack_from("<3h", payload, 80))
    gait_sin = struct.unpack_from("<h", payload, 86)[0] * GAIT_UNIT
    gait_cos = struct.unpack_from("<h", payload, 88)[0] * GAIT_UNIT
    cycle_cnt = struct.unpack_from("<H", payload, 90)[0]
    voltage_mv = struct.unpack_from("<H", payload, 92)[0]
    status = payload[94]
    err_flags = payload[95]

    return TelFast(q=q, qd=qd, tau=tau, quat=quat, gyro=gyro,
                   gait_sin=gait_sin, gait_cos=gait_cos,
                   cycle_cnt=cycle_cnt, voltage_mv=voltage_mv,
                   status=status, err_flags=err_flags)


def encode_tel_fast(t: TelFast) -> bytes:
    """组 TEL_FAST 负载（96 字节）。⚠️ 供固件侧对比测试用。"""
    def enc(v: float, unit: float) -> int:
        if v != v:
            return 0
        x = int(round(v / unit))
        return max(-32768, min(32767, x))

    p = bytearray(LEN_TEL_FAST)
    struct.pack_into("<12h", p, 0, *[enc(v, Q_UNIT) for v in t.q])
    struct.pack_into("<12h", p, 24, *[enc(v, QD_UNIT) for v in t.qd])
    struct.pack_into("<12h", p, 48, *[enc(v, TAU_UNIT) for v in t.tau])
    struct.pack_into("<4h", p, 72, *[enc(v, QUAT_UNIT) for v in t.quat])
    struct.pack_into("<3h", p, 80, *[enc(v, GYRO_UNIT) for v in t.gyro])
    struct.pack_into("<h", p, 86, enc(t.gait_sin, GAIT_UNIT))
    struct.pack_into("<h", p, 88, enc(t.gait_cos, GAIT_UNIT))
    struct.pack_into("<H", p, 90, t.cycle_cnt & 0xFFFF)
    struct.pack_into("<H", p, 92, t.voltage_mv & 0xFFFF)
    p[94] = t.status & 0xFF
    p[95] = t.err_flags & 0xFF
    return bytes(p)


# ============================================================================
# TEL_STATUS（协议 §4.8，LEN=40）
# ============================================================================

@dataclass
class TelStatus:
    uptime_ms: int = 0
    voltage_mv: int = 0
    temperature: List[int] = field(default_factory=lambda: [0] * N_JOINTS)  # 0.1 °C
    rx_overflow: int = 0
    bus_err: int = 0
    bus_timeout: int = 0
    mode: int = 0
    status: int = 0
    err_flags: int = 0
    seq_lost: int = 0


def parse_tel_status(payload: bytes) -> TelStatus:
    if len(payload) != LEN_TEL_STATUS:
        raise ValueError("TEL_STATUS payload must be %d bytes" % LEN_TEL_STATUS)
    uptime = struct.unpack_from("<I", payload, 0)[0]
    volt = struct.unpack_from("<H", payload, 4)[0]
    temps = list(struct.unpack_from("<12h", payload, 6))
    rx_ovf = struct.unpack_from("<H", payload, 30)[0]
    bus_err = struct.unpack_from("<H", payload, 32)[0]
    bus_to = struct.unpack_from("<H", payload, 34)[0]
    return TelStatus(uptime_ms=uptime, voltage_mv=volt, temperature=temps,
                     rx_overflow=rx_ovf, bus_err=bus_err, bus_timeout=bus_to,
                     mode=payload[36], status=payload[37],
                     err_flags=payload[38], seq_lost=payload[39])


# ============================================================================
# 47 维观测组装（docs/31-pi-runtime.md §3.2）
# ============================================================================

def quat_to_gravity(quat) -> List[float]:
    """四元数 (w,x,y,z) → 机身系下的重力方向单位向量（3 维）。

    ⚠️ 必须与训练侧的 projected_gravity 导出量一致。
       在 Isaac Gym 里 base 系下重力为 R^T · (0,0,-1)，
       其第三列取负号即 R[2][:] 的负值。
    """
    import math
    w, x, y, z = quat
    gx = -2.0 * (x * z - w * y)
    gy = -2.0 * (y * z + w * x)
    gz = -(1.0 - 2.0 * (x * x + y * y))
    n = math.sqrt(gx * gx + gy * gy + gz * gz)
    if n <= 1e-12:
        return [0.0, 0.0, -1.0]
    return [gx / n, gy / n, gz / n]


def build_observation(tel: TelFast, cmd_vx: float, cmd_vy: float, cmd_wz: float,
                      last_action: List[float],
                      q_default: Optional[List[float]] = None) -> List[float]:
    """组装 47 维观测。

    ⚠️ 每一维的位置必须与 Sim/legged_gym/.../rlcat_env.py 的
       compute_observations 严格一致。
    ⚠️ last_action 是【策略原始输出】（tanh 之后、缩放之前），
       不是缩放后的关节角（docs/31-pi-runtime.md §3.2）。
    """
    if q_default is None:
        q_default = [0.0] * N_JOINTS
    if len(last_action) != N_JOINTS:
        raise ValueError("last_action must have %d entries" % N_JOINTS)

    obs: List[float] = []
    obs += quat_to_gravity(tel.quat)                                  #  0- 2
    obs += [v * OBS_SCALE_ANG_VEL for v in tel.gyro]                  #  3- 5
    obs += [cmd_vx, cmd_vy, cmd_wz]                                   #  6- 8
    obs += [tel.q[i] - q_default[i] for i in range(N_JOINTS)]         #  9-20
    obs += [v * OBS_SCALE_DOF_VEL for v in tel.qd]                    # 21-32
    obs += [v * OBS_SCALE_ACTION for v in last_action]                # 33-44
    obs += [tel.gait_sin, tel.gait_cos]                               # 45-46

    assert len(obs) == OBS_DIM, "observation must be %d dims, got %d" % (OBS_DIM, len(obs))
    return obs


def action_to_q_target(action: List[float],
                       q_default: Optional[List[float]] = None) -> List[float]:
    """动作 → 关节目标角：q_target = q_default + ACTION_SCALE × action"""
    if q_default is None:
        q_default = [0.0] * N_JOINTS
    if len(action) != N_JOINTS:
        raise ValueError("action must have %d entries" % N_JOINTS)
    return [q_default[i] + ACTION_SCALE * action[i] for i in range(N_JOINTS)]
