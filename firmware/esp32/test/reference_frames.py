#!/usr/bin/env python3
"""Reference frame encoder for cross-checking the firmware's protocol layer.

This is the implementation from firmware/doc/协议规范_树莓派通信.md §9 (the Pi
side), used as an independent oracle: `test/test_protocol.c` prints the frames it
builds, this script prints the same frames, and firmware/tools/test_esp32_host.ps1
diffs the two.  If they ever disagree, the firmware and the Pi are incompatible.

Usage:  python reference_frames.py
"""
import struct
import sys

HEAD = b"\xAA\x55"
TYPE_CMD_JOINT = 0x01
TYPE_CMD_MODE = 0x03
TYPE_CMD_CALIB = 0x05
TYPE_TEL_FAST = 0x02
TYPE_TEL_LOG = 0x06
TYPE_HEARTBEAT = 0x7F


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pack(msg_type: int, seq: int, payload: bytes) -> bytes:
    assert len(payload) <= 128, len(payload)
    body = bytes([len(payload), seq, msg_type]) + payload
    return HEAD + body + struct.pack("<H", crc16(body))


def make_cmd_joint(q_target, mode=2, flags=1, seq=0, kp_scale=100, led=1) -> bytes:
    assert len(q_target) == 12, len(q_target)
    payload = struct.pack("<BB", mode, flags)
    payload += b"".join(struct.pack("<h", int(round(q / 0.0001))) for q in q_target)
    payload += struct.pack("<B", 0)          # 26     reserved
    payload += struct.pack("<h", kp_scale)   # 27..28
    payload += struct.pack("<B", led)        # 29
    payload += struct.pack("<B", 0)          # 30     reserved2
    assert len(payload) == 31, len(payload)
    return pack(TYPE_CMD_JOINT, seq, payload)


def emit(label: str, frame: bytes) -> None:
    print("FRAME %s %s" % (label, frame.hex()))


def main() -> int:
    # order must match test/test_protocol.c
    emit("HEARTBEAT", pack(TYPE_HEARTBEAT, 7, b""))

    max_payload = bytes((i * 7 + 3) & 0xFF for i in range(128))
    emit("MAXPAYLOAD", pack(TYPE_TEL_LOG, 200, max_payload))

    q = [(i - 5) * 0.1 for i in range(12)]
    emit("CMD_JOINT", make_cmd_joint(q, mode=2, flags=1, seq=7, kp_scale=100, led=1))

    emit("CMD_MODE", pack(TYPE_CMD_MODE, 8, struct.pack("<BB", 0, 0)))
    emit("CMD_CALIB", pack(TYPE_CMD_CALIB, 9, struct.pack("<B", 0)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
