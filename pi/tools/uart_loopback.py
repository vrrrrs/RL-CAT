#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL-CAT UART 1 Mbps 回环误码测试

用途：把 GPIO14(TXD0) 和 GPIO15(RXD0) 用一根杜邦线短接，
      然后跑本脚本，验证 /dev/ttyAMA0 在 1 Mbps(8N1) 下的链路质量。

判据（docs/42-pi-setup.md §C3）：连续发送零误码、零丢字节。

⚠️ 为什么必须做这一步：
   1 Mbps 下只要时钟有偏差就会随机误码，而"能收到数据"≠"零误码"。
   不做这一步，后面与 STM32 联调时你会分不清是 UART、是协议还是接线的问题。

用法：
    python3 uart_loopback.py                        # 默认 /dev/ttyAMA0 @ 1000000
    python3 uart_loopback.py /dev/ttyAMA0 1000000   # 位置参数写法
    python3 uart_loopback.py --port /dev/pts/3 --flat 20000 --prbs 50000

退出码：0 = 零误码；1 = 有误码/丢字节；2 = 一个字节都没回来（跳线没接好）
"""
import argparse
import sys
import threading
import time

import serial

_ap = argparse.ArgumentParser(
    description="RL-CAT UART 1 Mbps 回环误码测试（需短接 GPIO14↔GPIO15）")
_ap.add_argument("port_pos", nargs="?", default=None, help="串口设备（位置参数写法）")
_ap.add_argument("baud_pos", nargs="?", type=int, default=None, help="波特率（位置参数写法）")
_ap.add_argument("--port", default=None, help="串口设备，默认 /dev/ttyAMA0")
_ap.add_argument("--baud", type=int, default=None, help="波特率，默认 1000000")
_ap.add_argument("--flat", type=int, default=100_000,
                 help="每种固定图形的字节数（0x55/0xAA/0xFF/0x00），默认 100000")
_ap.add_argument("--prbs", type=int, default=500_000,
                 help="伪随机序列字节数，默认 500000")
_args = _ap.parse_args()

PORT = _args.port or _args.port_pos or "/dev/ttyAMA0"
BAUD = _args.baud or _args.baud_pos or 1000000

# 每种图形的长度（字节）
N_FLAT = _args.flat
N_PRBS = _args.prbs


def prbs(n: int, seed: int = 0x12345678) -> bytes:
    """xorshift32 伪随机序列 —— 确定性、可复现，便于和 PC 侧对拍。"""
    b = bytearray(n)
    x = seed & 0xFFFFFFFF
    for i in range(n):
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        b[i] = x & 0xFF
    return bytes(b)


class Reader(threading.Thread):
    """后台持续读，防止内核 RX 缓冲(约 4KB)溢出导致假丢包。"""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.buf = bytearray()
        self.stop = threading.Event()
        self.err = None

    def run(self):
        while not self.stop.is_set():
            try:
                d = self.ser.read(65536)
            except Exception as e:          # 端口被拔掉/关闭
                self.err = e
                return
            if d:
                self.buf.extend(d)


def one_pattern(ser, name, data):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    r = Reader(ser)
    r.start()

    t0 = time.perf_counter()
    chunk = 4096
    for off in range(0, len(data), chunk):
        ser.write(data[off:off + chunk])
    ser.flush()                              # 等 TX 真正排空

    deadline = t0 + 5.0 + len(data) / 50_000.0
    while len(r.buf) < len(data) and time.perf_counter() < deadline:
        time.sleep(0.002)
    t1 = time.perf_counter()
    time.sleep(0.05)                         # 让最后几个字节到齐
    r.stop.set()
    r.join(timeout=1.0)

    got = bytes(r.buf[:len(data)])
    nbad = 0
    first = []
    for i in range(min(len(got), len(data))):
        if got[i] != data[i]:
            nbad += 1
            if len(first) < 5:
                first.append(i)

    dt = t1 - t0
    kbps = (len(data) * 10 / 1000.0) / dt if dt > 0 else 0.0
    ok = (nbad == 0 and len(got) == len(data))
    print("  %-6s 发 %7d  收 %7d  误码 %6d  丢 %6d  %6.3fs  %6.0f kbit/s  %s"
          % (name, len(data), len(got), nbad, len(data) - len(got), dt, kbps,
             "OK" if ok else "FAIL"))
    if first:
        print("         首个误码偏移: %s" % first)
    return ok, nbad, len(got), len(data)


def main() -> int:
    print("=" * 74)
    print(" RL-CAT UART 回环误码测试   %s @ %d bps  (8N1, 无流控)" % (PORT, BAUD))
    print("=" * 74)

    ser = serial.Serial(PORT, BAUD,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=0.05,
                        write_timeout=2.0)
    ser.rtscts = False
    ser.dsrdtr = False
    ser.xonxoff = False
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.2)

    # ---- 冒烟：先确认跳线真的接通 ----
    probe = b"\x55\xaa" * 8
    ser.write(probe)
    ser.flush()
    time.sleep(0.1)
    back = ser.read(len(probe) + 64)
    same = back[:len(probe)] == probe
    print("冒烟测试: 发 %d 收 %d -> %s"
          % (len(probe), len(back), "一致" if same else "不一致"))
    if len(back) == 0:
        print("\n!! 一个字节都没回来。")
        print("!! 请检查：杜邦线是否短接了【物理引脚 8 (GPIO14)】和【物理引脚 10 (GPIO15)】")
        ser.close()
        return 2

    total_bad = 0
    allok = True
    for name, data in (("0x55", b"\x55" * N_FLAT),
                       ("0xAA", b"\xaa" * N_FLAT),
                       ("0xFF", b"\xff" * N_FLAT),
                       ("0x00", b"\x00" * N_FLAT),
                       ("PRBS", prbs(N_PRBS))):
        ok, nbad, ngot, nsent = one_pattern(ser, name, data)
        total_bad += nbad
        allok = allok and ok and (ngot == nsent)

    ser.close()

    print("-" * 74)
    print("累计误码: %d" % total_bad)
    if allok:
        print("结论: ✅ 零误码、零丢字节 —— 这一路可以接 STM32 了")
    else:
        print("结论: ❌ 有误码或丢字节 —— 先不要接 STM32，先排查")
    return 0 if allok else 1


if __name__ == "__main__":
    sys.exit(main())
