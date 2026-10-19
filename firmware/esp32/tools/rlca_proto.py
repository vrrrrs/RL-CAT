#!/usr/bin/env python3
"""RL-CAT frame codec for the PC-side bench tools.

This mirrors firmware/esp32/src/protocol.c (which is the implementation under
test).  The independent oracle for the C code is
firmware/esp32/test/reference_frames.py (taken from the protocol spec §9) --
keep that file untouched; this module is the convenience layer the bench tools
use (build + parse + pretty print).

Conventions (firmware/doc/协议规范_树莓派通信.md):
  frame = AA 55 | LEN | SEQ | TYPE | PAYLOAD[LEN] | CRC16(LE over LEN..PAYLOAD)
  joint angles are logical index A (hip x4, thigh x4, calf x4), 0.0001 rad units
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import Iterator, List, Optional, Tuple

HEAD0 = 0xAA
HEAD1 = 0x55
FRAME_MAX_PAYLOAD = 128

TYPE_CMD_JOINT = 0x01
TYPE_TEL_FAST = 0x02
TYPE_CMD_MODE = 0x03
TYPE_TEL_STATUS = 0x04
TYPE_CMD_CALIB = 0x05
TYPE_TEL_LOG = 0x06
TYPE_TEL_ACK = 0x07
TYPE_HEARTBEAT = 0x7F

TYPE_NAMES = {
    TYPE_CMD_JOINT: "CMD_JOINT",
    TYPE_TEL_FAST: "TEL_FAST",
    TYPE_CMD_MODE: "CMD_MODE",
    TYPE_TEL_STATUS: "TEL_STATUS",
    TYPE_CMD_CALIB: "CMD_CALIB",
    TYPE_TEL_LOG: "TEL_LOG",
    TYPE_TEL_ACK: "TEL_ACK",
    TYPE_HEARTBEAT: "HEARTBEAT",
}

MODE_IDLE, MODE_STAND, MODE_ACTIVE, MODE_DAMP, MODE_ESTOP = range(5)
MODE_NAMES = {0: "IDLE", 1: "STAND", 2: "ACTIVE", 3: "DAMP", 4: "ESTOP"}

FLAG_ENABLE = 1 << 0
FLAG_RESET_ERR = 1 << 1
FLAG_REQ_STATUS = 1 << 2
FLAG_REQ_CALIB = 1 << 3

STAT_READY = 1 << 0
STAT_CALIBRATED = 1 << 1
STAT_LINK_OK = 1 << 2
STAT_IMU_OK = 1 << 3
STAT_BATT_WARN = 1 << 4
STAT_BATT_STOP = 1 << 5
STAT_BATT_FORCE = 1 << 6
STAT_ESTOP = 1 << 7

NUM_JOINTS = 12
Q_SCALE = 0.0001


# --------------------------------------------------------------------------
# CRC + framing
# --------------------------------------------------------------------------
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pack(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    assert len(payload) <= FRAME_MAX_PAYLOAD, len(payload)
    body = bytes([len(payload), seq & 0xFF, msg_type & 0xFF]) + payload
    return bytes([HEAD0, HEAD1]) + body + struct.pack("<H", crc16(body))


@dataclass
class Frame:
    type: int
    seq: int
    payload: bytes = b""

    @property
    def name(self) -> str:
        return TYPE_NAMES.get(self.type, "0x%02X" % self.type)


class FrameParser:
    """Streaming receiver, mirrors the C state machine (protocol.c)."""

    def __init__(self) -> None:
        self.crc_errors = 0
        self.len_errors = 0
        self.frames_ok = 0
        self._state = 0
        self._hdr = bytearray()
        self._pay = bytearray()
        self._crc = bytearray()

    def feed(self, data: bytes) -> List[Frame]:
        out: List[Frame] = []
        for b in data:
            if self._state == 0:
                if b == HEAD0:
                    self._state = 1
            elif self._state == 1:
                if b == HEAD1:
                    self._state, self._hdr = 2, bytearray()
                elif b != HEAD0:
                    self._state = 0
            elif self._state == 2:
                if not self._hdr and b > FRAME_MAX_PAYLOAD:
                    self.len_errors += 1
                    self._reset()
                    continue
                self._hdr.append(b)
                if len(self._hdr) == 3:
                    self._pay = bytearray()
                    self._state = 4 if self._hdr[0] == 0 else 3
            elif self._state == 3:
                self._pay.append(b)
                if len(self._pay) >= self._hdr[0]:
                    self._state = 4
            else:
                self._crc.append(b)
                if len(self._crc) < 2:
                    continue
                body = bytes(self._hdr) + bytes(self._pay)
                rx = struct.unpack("<H", bytes(self._crc))[0]
                if crc16(body) == rx:
                    self.frames_ok += 1
                    out.append(Frame(self._hdr[2], self._hdr[1], bytes(self._pay)))
                else:
                    self.crc_errors += 1
                self._reset()
        return out

    def _reset(self) -> None:
        self._state = 0
        self._hdr = bytearray()
        self._pay = bytearray()
        self._crc = bytearray()


# --------------------------------------------------------------------------
# message builders
# --------------------------------------------------------------------------
def build_heartbeat(seq: int = 0) -> bytes:
    return pack(TYPE_HEARTBEAT, seq)


def build_cmd_mode(mode: int, param: int = 0, seq: int = 0) -> bytes:
    return pack(TYPE_CMD_MODE, seq, struct.pack("<BB", mode, param))


def build_cmd_calib(action: int, seq: int = 0) -> bytes:
    return pack(TYPE_CMD_CALIB, seq, struct.pack("<B", action))


def build_cmd_joint(q_rad, mode: int = MODE_ACTIVE, flags: int = FLAG_ENABLE,
                    seq: int = 0, kp_scale: int = 100, led: int = 1) -> bytes:
    assert len(q_rad) == NUM_JOINTS, len(q_rad)
    payload = struct.pack("<BB", mode, flags)
    payload += b"".join(struct.pack("<h", int(round(q / Q_SCALE))) for q in q_rad)
    payload += struct.pack("<B", 0)             # 26     reserved
    payload += struct.pack("<h", kp_scale)      # 27..28
    payload += struct.pack("<B", led)           # 29
    payload += struct.pack("<B", 0)             # 30     reserved2
    assert len(payload) == 31, len(payload)
    return pack(TYPE_CMD_JOINT, seq, payload)


def build_tel_fast(q=None, qd=None, tau=None, quat=(10000, 0, 0, 0), gyro=(0, 0, 0),
                   gait=(0, 0), cycle: int = 0, voltage_mv: int = 11500,
                   status: int = 0, err_flags: int = 0, seq: int = 0) -> bytes:
    q = q or [0.0] * NUM_JOINTS
    qd = qd or [0.0] * NUM_JOINTS
    tau = tau or [0.0] * NUM_JOINTS
    payload = b"".join(struct.pack("<h", int(round(v / Q_SCALE))) for v in q)
    payload += b"".join(struct.pack("<h", int(round(v / Q_SCALE))) for v in qd)
    payload += b"".join(struct.pack("<h", int(round(v / 0.001))) for v in tau)
    payload += b"".join(struct.pack("<h", int(v)) for v in quat)
    payload += b"".join(struct.pack("<h", int(v)) for v in gyro)
    payload += struct.pack("<hh", int(gait[0]), int(gait[1]))
    payload += struct.pack("<H", cycle & 0xFFFF)
    payload += struct.pack("<H", voltage_mv & 0xFFFF)
    payload += struct.pack("<BB", status & 0xFF, err_flags & 0xFF)
    assert len(payload) == 96, len(payload)
    return pack(TYPE_TEL_FAST, seq, payload)


def build_tel_status(uptime_ms: int = 0, voltage_mv: int = 11500, temperatures=None,
                     rx_overflow: int = 0, bus_err: int = 0, bus_timeout: int = 0,
                     mode: int = MODE_IDLE, status: int = 0, err_flags: int = 0,
                     seq_lost: int = 0, seq: int = 0) -> bytes:
    temperatures = temperatures or [250] * NUM_JOINTS
    payload = struct.pack("<IH", uptime_ms & 0xFFFFFFFF, voltage_mv & 0xFFFF)
    payload += b"".join(struct.pack("<h", t) for t in temperatures)
    payload += struct.pack("<HHH", rx_overflow & 0xFFFF, bus_err & 0xFFFF, bus_timeout & 0xFFFF)
    payload += struct.pack("<BBBB", mode & 0xFF, status & 0xFF, err_flags & 0xFF, seq_lost & 0xFF)
    assert len(payload) == 40, len(payload)
    return pack(TYPE_TEL_STATUS, seq, payload)


def build_tel_ack(acked_type: int, result: int, seq: int = 0) -> bytes:
    return pack(TYPE_TEL_ACK, seq, struct.pack("<BB", acked_type, result))


# --------------------------------------------------------------------------
# parsers / pretty printers
# --------------------------------------------------------------------------
def parse_tel_fast(payload: bytes) -> dict:
    assert len(payload) == 96, len(payload)
    q = struct.unpack_from("<12h", payload, 0)
    qd = struct.unpack_from("<12h", payload, 24)
    tau = struct.unpack_from("<12h", payload, 48)
    quat = struct.unpack_from("<4h", payload, 72)
    gyro = struct.unpack_from("<3h", payload, 80)
    gait_sin, gait_cos = struct.unpack_from("<hh", payload, 86)
    cycle, volt = struct.unpack_from("<HH", payload, 90)
    return dict(
        q=[v * Q_SCALE for v in q],
        qd=[v * Q_SCALE for v in qd],
        tau=[v * 0.001 for v in tau],
        quat=[v * Q_SCALE for v in quat],
        gyro=[v * 0.001 for v in gyro],
        gait_sin=gait_sin / 32767.0,
        gait_cos=gait_cos / 32767.0,
        cycle_cnt=cycle,
        voltage_mv=volt,
        status=payload[94],
        err_flags=payload[95],
    )


def parse_tel_status(payload: bytes) -> dict:
    assert len(payload) == 40, len(payload)
    uptime, volt = struct.unpack_from("<IH", payload, 0)
    temps = struct.unpack_from("<12h", payload, 6)
    rx_ovf, bus_err, bus_to = struct.unpack_from("<HHH", payload, 30)
    mode, status, err, lost = struct.unpack_from("<BBBB", payload, 36)
    return dict(uptime_ms=uptime, voltage_mv=volt, temperature=[t * 0.1 for t in temps],
                rx_overflow=rx_ovf, bus_err=bus_err, bus_timeout=bus_to,
                mode=mode, status=status, err_flags=err, seq_lost=lost)


def parse_cmd_joint(payload: bytes) -> dict:
    assert len(payload) == 31, len(payload)
    mode, flags = struct.unpack_from("<BB", payload, 0)
    q = [v * Q_SCALE for v in struct.unpack_from("<12h", payload, 2)]
    kp = struct.unpack_from("<h", payload, 27)[0]
    return dict(mode=mode, flags=flags, q_target=q, kp_scale=kp, led=payload[29])


def status_str(status: int) -> str:
    bits = [(STAT_READY, "READY"), (STAT_CALIBRATED, "CALIB"), (STAT_LINK_OK, "LINK_OK"),
            (STAT_IMU_OK, "IMU_OK"), (STAT_BATT_WARN, "BATT_WARN"),
            (STAT_BATT_STOP, "BATT_STOP"), (STAT_BATT_FORCE, "BATT_FORCE"),
            (STAT_ESTOP, "ESTOP")]
    on = [n for m, n in bits if status & m]
    return "|".join(on) if on else "-"


def quat_to_gravity(quat):
    """Gravity direction in the body frame -- the same formula the Pi side uses
    (protocol spec §9) and that the ESP32 firmware implements in
    rlca_quat_to_gravity()."""
    w, x, y, z = quat
    gx = -2.0 * (x * z - w * y)
    gy = -2.0 * (y * z + w * x)
    gz = -(1.0 - 2.0 * (x * x + y * y))
    n = math.sqrt(gx * gx + gy * gy + gz * gz)
    if n < 1e-6:
        return [0.0, 0.0, -1.0]
    return [gx / n, gy / n, gz / n]


def fmt_frame(f: Frame, verbose: bool = False) -> str:
    if f.type == TYPE_TEL_FAST:
        d = parse_tel_fast(f.payload)
        g = quat_to_gravity(d["quat"])
        s = ("TEL_FAST seq=%3d cyc=%5d %5.2fV %-22s q0..3=[%s] gait=(%+.2f,%+.2f)\n"
             "           quat=(%+.3f,%+.3f,%+.3f,%+.3f) gyro=(%+.2f,%+.2f,%+.2f) g_body=(%+.3f,%+.3f,%+.3f)"
             % (f.seq, d["cycle_cnt"], d["voltage_mv"] / 1000.0, status_str(d["status"]),
                " ".join("%+.3f" % v for v in d["q"][:4]), d["gait_sin"], d["gait_cos"],
                d["quat"][0], d["quat"][1], d["quat"][2], d["quat"][3],
                d["gyro"][0], d["gyro"][1], d["gyro"][2], g[0], g[1], g[2]))
        return s
    if f.type == TYPE_TEL_STATUS:
        d = parse_tel_status(f.payload)
        return ("TEL_STATUS seq=%3d up=%dms %5.2fV mode=%s %-22s ovf=%d lost=%d"
                % (f.seq, d["uptime_ms"], d["voltage_mv"] / 1000.0,
                   MODE_NAMES.get(d["mode"], "?"), status_str(d["status"]),
                   d["rx_overflow"], d["seq_lost"]))
    if f.type == TYPE_CMD_JOINT:
        d = parse_cmd_joint(f.payload)
        return ("CMD_JOINT seq=%3d mode=%s flags=0x%02X q=[%s]"
                % (f.seq, MODE_NAMES.get(d["mode"], "?"), d["flags"],
                   " ".join("%+.3f" % v for v in d["q_target"])))
    if f.type == TYPE_CMD_MODE:
        return "CMD_MODE seq=%3d mode=%s param=%d" % (
            f.seq, MODE_NAMES.get(f.payload[0], "?"), f.payload[1])
    if f.type == TYPE_CMD_CALIB:
        return "CMD_CALIB seq=%3d action=%d" % (f.seq, f.payload[0])
    if f.type == TYPE_TEL_LOG:
        return "TEL_LOG seq=%3d level=%d text=%r" % (
            f.seq, f.payload[0], f.payload[2:].decode("ascii", "replace"))
    if f.type == TYPE_TEL_ACK:
        return "TEL_ACK seq=%3d type=0x%02X result=%d" % (f.seq, f.payload[0], f.payload[1])
    if f.type == TYPE_HEARTBEAT:
        return "HEARTBEAT seq=%3d" % f.seq
    extra = f.payload.hex() if verbose else ("%d bytes" % len(f.payload))
    return "%s seq=%3d %s" % (f.name, f.seq, extra)
