#!/usr/bin/env python3
"""RL-CAT ESP32 bridge bench tool.

Acceptance tests for docs/36-esp32-plan.md phases P1..P3 that do not need the
firmware source tree, and that work whether or not the STM32 exists yet:

  selftest  prove this tool + the codec are correct (no hardware needed)
  loop      TCP throughput/loss test against the ESP32 bridge port.
            With a jumper between ESP32 GPIO6 and GPIO7 (UART1 TX<->RX) the
            bridge echoes everything back, which validates the complete
            TCP -> UART -> TCP path with no STM32 attached.
  mon       decode a protocol byte stream from a serial port (e.g. the STM32
            link through a USB-TTL adapter)
  sim       pretend to be the STM32: send TEL_FAST at 100 Hz, decode the
            commands coming back (the ESP32's phase-1 peer)
  tcp       laptop-side client for the bridge port (raw or decoded)
  con       laptop-side client for the ESP32 console port (text commands)

Run it with a Python that has pyserial, e.g. through ../tools/bench.ps1:
    powershell -ExecutionPolicy Bypass -File firmware/tools/bench.ps1 selftest
"""
from __future__ import annotations

import argparse
import os
import random
import select
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rlca_proto as P  # noqa: E402


def _serial():
    try:
        import serial  # noqa: F401
    except ImportError:
        sys.exit("pyserial is missing -- run this via firmware/tools/bench.ps1")
    import serial
    return serial


def _open_serial(port: str, baud: int):
    serial = _serial()
    return serial.Serial(port, baud, timeout=0.05)


# --------------------------------------------------------------------------
# selftest
# --------------------------------------------------------------------------
def _load_reference():
    """Import the protocol-spec oracle (test/reference_frames.py)."""
    import importlib.util
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "test",
                        "reference_frames.py")
    spec = importlib.util.spec_from_file_location("reference_frames", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def cmd_selftest(args) -> int:
    fails = []

    def check(cond, what):
        if not cond:
            fails.append(what)
        print("  %s %s" % ("ok  " if cond else "FAIL", what))

    print("[1] CRC vector")
    check(P.crc16(b"123456789") == 0x29B1, "crc16('123456789') == 0x29B1")

    print("[2] parser robustness")
    good = P.build_tel_status(seq=5)
    par = P.FrameParser()
    frames = par.feed(b"\xAA\xAA" + good)            # `AA AA 55 ...`: sync on the 2nd AA
    check(len(frames) == 1 and frames[0].seq == 5, "AA AA 55 sync")
    par = P.FrameParser()
    junk = bytes([0x00, 0xFF, 0x55, 0xAA, 0x12, 0xAA])   # same junk as the C test
    frames = par.feed(junk + good)
    check(len(frames) == 1 and frames[0].seq == 5, "garbage prefix skipped")
    par = P.FrameParser()
    check(len(par.feed(good[:6])) == 0, "truncated frame is not delivered")
    par = P.FrameParser()
    check(len(par.feed(b"\xAA\x55\xFF" + b"\x00" * 8 + good)) == 1 and par.len_errors == 1,
          "LEN=255 rejected and state machine recovers")
    par = P.FrameParser()
    bad = bytearray(good)
    bad[-1] ^= 0xFF
    check(len(par.feed(bytes(bad))) == 0 and par.crc_errors == 1, "bad CRC rejected")

    print("[3] TEL_FAST round trip")
    q = [0.1 * i - 0.5 for i in range(12)]
    raw = P.build_tel_fast(q=q, cycle=4660, voltage_mv=11536,
                           status=P.STAT_READY | P.STAT_LINK_OK, seq=42)
    f = P.FrameParser().feed(raw)[0]
    d = P.parse_tel_fast(f.payload)
    check(len(raw) == 103, "TEL_FAST frame is 103 bytes")
    check(d["cycle_cnt"] == 4660 and d["voltage_mv"] == 11536, "cycle/voltage decode")
    check(d["status"] == (P.STAT_READY | P.STAT_LINK_OK), "status bits decode")
    check(max(abs(a - b) for a, b in zip(d["q"], q)) < 1e-4, "q decode")

    print("[4] cross-check against the protocol-spec oracle")
    ref = _load_reference()
    qq = [(i - 5) * 0.1 for i in range(12)]
    mine = P.build_cmd_joint(qq, mode=P.MODE_ACTIVE, flags=P.FLAG_ENABLE, seq=7,
                             kp_scale=100, led=1)
    theirs = ref.make_cmd_joint(qq, mode=2, flags=1, seq=7, kp_scale=100, led=1)
    check(mine == theirs, "CMD_JOINT matches reference_frames.py")
    check(P.build_heartbeat(7) == ref.pack(ref.TYPE_HEARTBEAT, 7, b""), "HEARTBEAT matches")
    check(P.build_cmd_mode(0, 0, 8) == ref.pack(3, 8, b"\x00\x00"), "CMD_MODE matches")
    check(P.build_cmd_calib(0, 9) == ref.pack(5, 9, b"\x00"), "CMD_CALIB matches")

    print("[5] loopback logic against a local echo server")
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def echo():
        c, _ = srv.accept()
        while True:
            data = c.recv(4096)
            if not data:
                break
            c.sendall(data)
        c.close()

    threading.Thread(target=echo, daemon=True).start()
    r = run_loop("127.0.0.1", port, total=64 * 1024, chunk=1024, quiet=True)
    check(r["sent"] == r["received"] == 64 * 1024, "echo server returned everything")
    check(r["mismatch"] == 0, "no byte mismatches")
    srv.close()

    print()
    if fails:
        print("SELFTEST FAILED: %d problem(s)" % len(fails))
        return 1
    print("SELFTEST OK")
    return 0


# --------------------------------------------------------------------------
# loop: throughput / loss over the bridge port
# --------------------------------------------------------------------------
def run_loop(host: str, port: int, total: int, chunk: int, quiet: bool = False,
             deadline_s: float = 30.0) -> dict:
    pattern = bytes(range(256))
    sock = socket.create_connection((host, port), timeout=5)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setblocking(False)

    sent = 0
    received = bytearray()
    mismatch = 0
    t0 = time.time()

    while sent < total and (time.time() - t0) < deadline_s:
        n = min(chunk, total - sent)
        payload = (pattern * (n // 256 + 1))[:n]
        try:
            sock.sendall(payload)
        except BlockingIOError:
            pass
        else:
            sent += n

        # drain whatever came back
        want = sent - len(received)
        end = time.time() + 0.05
        while len(received) < sent and time.time() < end:
            r, _, _ = select.select([sock], [], [], 0.02)
            if not r:
                break
            try:
                data = sock.recv(65536)
            except BlockingIOError:
                continue
            if not data:
                break
            received += data

        if want <= 0 and sent == total:
            break

    # final drain
    end = time.time() + 0.5
    while len(received) < sent and time.time() < end:
        r, _, _ = select.select([sock], [], [], 0.05)
        if not r:
            break
        try:
            data = sock.recv(65536)
        except BlockingIOError:
            continue
        if not data:
            break
        received += data

    dt = time.time() - t0
    cmp_len = min(len(received), sent)
    for i in range(cmp_len):
        if received[i] != pattern[i % 256]:
            mismatch += 1
    sock.close()

    result = dict(sent=sent, received=len(received), mismatch=mismatch,
                  seconds=dt, kbps=(sent * 8 / 1000.0 / dt) if dt else 0.0)
    if not quiet:
        print("loop: sent=%d received=%d mismatch=%d  %.2fs  %.1f kbit/s"
              % (result["sent"], result["received"], result["mismatch"],
                 result["seconds"], result["kbps"]))
        if result["received"] < result["sent"]:
            print("  ⚠️  %d bytes never came back -- is GPIO6 jumpered to GPIO7 "
                  "(UART loopback) or is a peer echoing?" % (result["sent"] - result["received"]))
        elif result["mismatch"]:
            print("  ⚠️  byte mismatches: the UART path corrupted data")
        else:
            print("  ✅ full round trip, zero loss, zero corruption")
    return result


def cmd_loop(args) -> int:
    r = run_loop(args.host, args.port, args.bytes, args.chunk)
    return 0 if (r["mismatch"] == 0 and r["received"] == r["sent"]) else 1


# --------------------------------------------------------------------------
# mon: decode a serial byte stream
# --------------------------------------------------------------------------
def cmd_mon(args) -> int:
    ser = _open_serial(args.port, args.baud)
    print("monitoring %s @ %d (ctrl-c to stop)" % (args.port, args.baud))
    parser = P.FrameParser()
    last = time.time()
    try:
        while True:
            data = ser.read(4096)
            if not data:
                continue
            if args.raw:
                print(data.hex(" "), flush=True)
                continue
            for f in parser.feed(data):
                print("  %s" % P.fmt_frame(f), flush=True)
            if time.time() - last > 5:
                last = time.time()
                print("  -- frames=%d crc_err=%d len_err=%d --"
                      % (parser.frames_ok, parser.crc_errors, parser.len_errors))
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    return 0


# --------------------------------------------------------------------------
# sim: pretend to be the STM32
# --------------------------------------------------------------------------
def cmd_sim(args) -> int:
    ser = _open_serial(args.port, args.baud)
    parser = P.FrameParser()
    seq = 0
    cycle = 0
    stop = threading.Event()

    status = 0
    if args.link_ok:
        status |= P.STAT_LINK_OK
    if args.ready:
        status |= P.STAT_READY
    if args.calibrated:
        status |= P.STAT_CALIBRATED
    if args.imu_ok:
        status |= P.STAT_IMU_OK
    status |= args.status_extra

    def tx_loop():
        nonlocal seq, cycle
        period = 1.0 / args.hz
        next_t = time.time()
        while not stop.is_set():
            q = [args.amplitude * __import__("math").sin(2 * 3.14159265 * ((cycle % 100) / 100.0))
                 for _ in range(12)]
            gait_phase = (cycle % 34) / 34.0
            import math
            frame = P.build_tel_fast(
                q=q,
                gait=(32767 * math.sin(2 * math.pi * gait_phase),
                      32767 * math.cos(2 * math.pi * gait_phase)),
                cycle=cycle, voltage_mv=args.voltage, status=status, seq=seq)
            try:
                ser.write(frame)
            except Exception as e:      # noqa: BLE001
                print("serial write failed: %s" % e)
                stop.set()
                return
            seq = (seq + 1) & 0xFF
            cycle = (cycle + 1) & 0xFFFF
            next_t += period
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()

    print("simulating the STM32 on %s @ %d" % (args.port, args.baud))
    print("  status=0x%02X (%s)  %.2f V  %d Hz  link_ok=%d"
          % (status, P.status_str(status), args.voltage / 1000.0, args.hz, args.link_ok))
    t = threading.Thread(target=tx_loop, daemon=True)
    t.start()

    deadline = time.time() + args.duration if args.duration else None
    try:
        while not stop.is_set():
            if deadline and time.time() > deadline:
                break
            data = ser.read(4096)
            if not data:
                continue
            for f in parser.feed(data):
                print("  <- %s" % P.fmt_frame(f), flush=True)
                if args.serve and f.type == P.TYPE_CMD_JOINT and (f.payload[1] & P.FLAG_REQ_STATUS):
                    ser.write(P.build_tel_status(voltage_mv=args.voltage, status=status, mode=f.payload[0]))
                if args.serve and f.type == P.TYPE_HEARTBEAT:
                    ser.write(P.build_heartbeat(seq))
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        time.sleep(0.05)
        ser.close()
    print("sim done: sent=%d frames, received=%d frames, crc_err=%d"
          % (cycle, parser.frames_ok, parser.crc_errors))
    return 0


# --------------------------------------------------------------------------
# tcp / con: laptop-side clients
# --------------------------------------------------------------------------
def cmd_tcp(args) -> int:
    sock = socket.create_connection((args.host, args.port), timeout=5)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("connected to %s:%d" % (args.host, args.port))
    if args.send_hex:
        payload = bytes.fromhex(args.send_hex.replace(" ", ""))
        sock.sendall(payload)
        print("  -> %s" % payload.hex(" "))
    if args.send_text:
        sock.sendall(args.send_text.encode())
        print("  -> %r" % args.send_text)

    parser = P.FrameParser()
    deadline = time.time() + args.duration
    count = 0
    try:
        while time.time() < deadline:
            r, _, _ = select.select([sock], [], [], 0.2)
            if not r:
                continue
            data = sock.recv(65536)
            if not data:
                print("peer closed")
                break
            if args.raw:
                print(data.hex(" "), flush=True)
                continue
            frames = parser.feed(data)
            for f in frames:
                print("  %s" % P.fmt_frame(f), flush=True)
                count += 1
                if args.frames and count >= args.frames:
                    raise KeyboardInterrupt
            if not frames and not args.raw:
                print("  (unparsed bytes: %s)" % data.hex(" "))
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    print("done: %d frames, crc_err=%d len_err=%d" % (count, parser.crc_errors, parser.len_errors))
    return 0


def cmd_con(args) -> int:
    sock = socket.create_connection((args.host, args.port), timeout=5)
    print("connected to console %s:%d" % (args.host, args.port))
    if args.cmd:
        for line in args.cmd:
            sock.sendall((line + "\r\n").encode())
            time.sleep(0.3)
            r, _, _ = select.select([sock], [], [], 1.0)
            if r:
                print(sock.recv(65536).decode("utf-8", "replace").rstrip())
        sock.close()
        return 0

    print("interactive: type commands ('?' for help, 'q' to quit)")
    sock.setblocking(False)
    try:
        while True:
            r, _, _ = select.select([sock, sys.stdin], [], [], 0.2)
            if sock in r:
                data = sock.recv(65536)
                if not data:
                    print("peer closed")
                    break
                sys.stdout.write(data.decode("utf-8", "replace"))
                sys.stdout.flush()
            if sys.stdin in r:
                line = sys.stdin.readline()
                if not line:
                    break
                sock.sendall(line.encode())
                if line.strip() == "q":
                    break
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return 0


# --------------------------------------------------------------------------
# verify: drive the acceptance sheet (docs/37-esp32-acceptance.md) with one command
# --------------------------------------------------------------------------
# Trick that makes this possible without a STM32 and without a USB-TTL adapter:
#   with a single jumper between ESP32 GPIO6 (UART1 TX) and GPIO7 (UART1 RX), every
#   byte the bridge transmits comes straight back into its own UART receiver.
#   The console command `raw <hex>` therefore lets us *feed the bridge a
#   TEL_FAST frame* -- exactly what the STM32 would send -- so telemetry,
#   arbitration and injection can all be verified on the real device.
NUM_JOINTS_RAW = P.NUM_JOINTS
Q_REF = [(i + 1) * 1000 for i in range(NUM_JOINTS_RAW)]   # raw units = 0.0001 rad


class Result:
    def __init__(self) -> None:
        self.rows = []

    def check(self, step: str, desc: str, ok: bool, detail: str = "") -> bool:
        self.rows.append(dict(step=step, desc=desc, ok=bool(ok), detail=detail))
        print("  %s %-5s %s%s" % ("[ok]  " if ok else "[FAIL]", step, desc,
                                  ("   -- " + detail) if detail else ""))
        return bool(ok)

    @property
    def failed(self):
        return [r for r in self.rows if not r["ok"]]


def _connect(host: str, port: int, timeout: float = 3.0):
    s = socket.create_connection((host, port), timeout=timeout)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.setblocking(False)
    return s


def _read_for(sock, seconds: float) -> bytes:
    out = b""
    end = time.time() + seconds
    while time.time() < end:
        r, _, _ = select.select([sock], [], [], 0.05)
        if not r:
            continue
        try:
            d = sock.recv(65536)
        except BlockingIOError:
            continue
        if not d:
            break
        out += d
    return out


def _ask(sock, line: str, wait: float = 0.6) -> str:
    sock.sendall((line + "\r\n").encode())
    return _read_for(sock, wait).decode("utf-8", "replace")


class Observer:
    """Collects every frame the ESP32 transmits (seen back through the jumper)."""

    def __init__(self, host: str, port: int):
        self.sock = _connect(host, port)
        self.parser = P.FrameParser()
        self.frames = []

    def pump(self, seconds: float) -> None:
        self.frames.extend(self.parser.feed(_read_for(self.sock, seconds)))

    def count(self, ftype: int) -> int:
        return sum(1 for f in self.frames if f.type == ftype)

    def last(self, ftype: int):
        for f in reversed(self.frames):
            if f.type == ftype:
                return f
        return None


def _inject_tel_fast(con, status: int, cycle: int = 0, q_raw=None, voltage: int = 11500) -> str:
    q = [v * P.Q_SCALE for v in (q_raw or Q_REF)]
    frame = P.build_tel_fast(q=q, cycle=cycle, voltage_mv=voltage, status=status, seq=0)
    return _ask(con, "raw " + frame.hex(), wait=0.25)


def cmd_verify(args) -> int:
    res = Result()
    print("== RL-CAT ESP32 acceptance run ==")
    print("target %s  bridge=%d  console=%d  loopback=%s"
          % (args.host, args.port_bridge, args.port_console, "on" if args.loopback else "off"))
    if not args.loopback:
        print("NOTE: without --loopback the telemetry/arbitration steps cannot run;")
        print("      put a jumper between GPIO6 and GPIO7 and re-run with --loopback.")

    try:
        con = _connect(args.host, args.port_console)
    except OSError as exc:
        print("ERROR: cannot reach %s:%d (%s)" % (args.host, args.port_console, exc))
        print("       Is this laptop connected to the RL-CAT-XXXX access point?")
        print("       (ssid from the boot log, e.g. RL-CAT-E959, password rlcat1234)")
        return 2
    _read_for(con, 0.3)          # drain the mirrored log tail

    r = _ask(con, "?")
    res.check("A4", "console '?' returns the help text",
              ("s <j>" in r) and ("stats" in r), r.strip().splitlines()[0][:70] if r.strip() else "no reply")
    r = _ask(con, "k")
    res.check("A4", "console 'k' replies ok", "ok" in r)
    r = _ask(con, "stats")
    res.check("A4", "console 'stats' shows tcp+uart counters", ("tcp" in r) and ("uart:" in r))

    r = _ask(con, "selftest")
    res.check("A18", "on-target self test 6/6", "6/6" in r, r.strip()[:70])
    r = _ask(con, "raw ZZ")
    res.check("A13", "raw rejects bad hex", "bad hex" in r, r.strip()[:60])
    r = _ask(con, "s 12 5")
    res.check("A12", "out-of-range joint rejected", "usage" in r, r.strip()[:50])
    r = _ask(con, "s 4 99")
    res.check("A12", "out-of-range delta rejected", "30 deg" in r, r.strip()[:50])
    r = _ask(con, "s 4 0")
    res.check("A12", "zero delta rejected", "nothing to do" in r, r.strip()[:50])
    r = _ask(con, "cfg")
    res.check("A19", "cfg shows ports and password source", "bridge port" in r, r.strip()[:70])
    r = _ask(con, "cfg pw abc")
    res.check("A19", "cfg pw validates length", "8..63" in r, r.strip()[:60])
    r = _ask(con, "i")
    res.check("A10", "i reports link state without injecting", "inject=" in r, r.replace("\r\n", " | ")[:100])

    if args.loopback:
        lr = run_loop(args.host, args.port_bridge, args.bytes, args.chunk, quiet=True)
        res.check("A5", "UART loopback: TCP->UART->TCP, zero loss",
                  (lr["received"] == lr["sent"]) and (lr["mismatch"] == 0),
                  "sent=%d received=%d mismatch=%d %.1fs" % (lr["sent"], lr["received"],
                                                             lr["mismatch"], lr["seconds"]))
        obs = Observer(args.host, args.port_bridge)
        obs.pump(0.3)

        # A10/A14 -- telemetry arrives, Pi considered offline -> injection allowed
        _inject_tel_fast(con, P.STAT_READY | P.STAT_CALIBRATED | P.STAT_IMU_OK, cycle=1)
        r = _ask(con, "i")
        res.check("A10", "with telemetry: q=yes, pi_online=0, inject=allowed",
                  ("q=yes" in r) and ("pi_online=0" in r) and ("allowed" in r),
                  r.replace("\r\n", " | ")[:110])

        # A11 -- single joint move, other joints held at the reported positions
        _inject_tel_fast(con, P.STAT_READY | P.STAT_CALIBRATED | P.STAT_IMU_OK, cycle=2)
        before = obs.count(P.TYPE_CMD_JOINT)
        r = _ask(con, "s 4 10", wait=0.5)
        obs.pump(0.4)
        f = obs.last(P.TYPE_CMD_JOINT)
        ok = ("sent CMD_JOINT" in r) and (obs.count(P.TYPE_CMD_JOINT) > before) and (f is not None)
        detail = r.strip()[:60]
        if ok:
            d = P.parse_cmd_joint(f.payload)
            want = Q_REF[4] * P.Q_SCALE + 10 * 3.14159265 / 180.0
            held = all(abs(d["q_target"][i] - Q_REF[i] * P.Q_SCALE) < 2e-4
                       for i in range(NUM_JOINTS_RAW) if i != 4)
            ok = held and (abs(d["q_target"][4] - want) < 5e-4) and (d["mode"] == P.MODE_ACTIVE)
            detail = "q4=%.4f (want %.4f) others=%s mode=%s" % (
                d["q_target"][4], want, "held" if held else "CHANGED",
                P.MODE_NAMES.get(d["mode"], "?"))
        res.check("A11", "s 4 10 moves joint 4 by 10 deg, others held", ok, detail)

        # A15 -- Pi online -> refused, and nothing is transmitted
        _inject_tel_fast(con, P.STAT_READY | P.STAT_CALIBRATED | P.STAT_IMU_OK | P.STAT_LINK_OK, cycle=3)
        before = obs.count(P.TYPE_CMD_JOINT)
        r = _ask(con, "s 4 10", wait=0.4)
        obs.pump(0.3)
        res.check("A15", "Pi online: s refused and nothing sent",
                  ("blocked" in r) and (obs.count(P.TYPE_CMD_JOINT) == before), r.strip()[:80])
        r = _ask(con, "z", wait=0.4)
        res.check("A15", "Pi online: z refused", "blocked" in r, r.strip()[:70])

        # A16 -- telemetry goes stale -> fail safe
        _inject_tel_fast(con, P.STAT_READY | P.STAT_CALIBRATED | P.STAT_IMU_OK, cycle=4)
        time.sleep(1.2)
        r = _ask(con, "s 4 10", wait=0.4)
        res.check("A16", "stale telemetry: s refused (fail safe)",
                  ("blocked" in r) and ("stale" in r), r.strip()[:90])

        # A14 -- Pi offline again -> z really transmits
        _inject_tel_fast(con, P.STAT_READY | P.STAT_CALIBRATED | P.STAT_IMU_OK, cycle=5)
        before = obs.count(P.TYPE_CMD_MODE)
        r = _ask(con, "z", wait=0.4)
        obs.pump(0.3)
        res.check("A14", "Pi offline: z sends CMD_MODE",
                  ("sent CMD_MODE" in r) and (obs.count(P.TYPE_CMD_MODE) > before), r.strip()[:70])
        obs.sock.close()
    else:
        for step, desc in (("A5", "UART loopback loss test"),
                           ("A10", "telemetry state"),
                           ("A11", "single joint move"),
                           ("A14", "Pi offline injection"),
                           ("A15", "Pi online refusal"),
                           ("A16", "stale telemetry refusal")):
            res.check(step, desc + " (skipped: needs --loopback)", False, "jumper GPIO6<->GPIO7")

    con.close()

    print()
    print("== summary: %d passed, %d failed ==" % (len(res.rows) - len(res.failed), len(res.failed)))
    for r in res.failed:
        print("   FAIL %s  %s  %s" % (r["step"], r["desc"], r["detail"]))

    if args.report:
        import json
        with open(args.report, "w", encoding="utf-8") as fh:
            json.dump(dict(host=args.host, loopback=args.loopback, rows=res.rows), fh,
                      ensure_ascii=False, indent=2)
        print("report written to %s" % args.report)
    return 1 if res.failed else 0


# --------------------------------------------------------------------------
# doctor: one command that says what is wrong
# --------------------------------------------------------------------------
def _wlan_ssid():
    """Best effort: the SSID this PC is associated with (Windows netsh)."""
    import subprocess
    try:
        out = subprocess.run(["netsh", "wlan", "show", "interfaces"],
                             capture_output=True, timeout=10)
    except Exception:
        return None
    text = (out.stdout or b"").decode("utf-8", "replace")
    for line in text.splitlines():
        s = line.strip()
        u = s.upper()
        if u.startswith("SSID") and not u.startswith("BSSID"):
            return s.split(":", 1)[1].strip() if ":" in s else s
    return None


def _try_connect(host: str, port: int, timeout: float = 2.0):
    try:
        s = socket.create_connection((host, port), timeout=timeout)
        s.close()
        return True, ""
    except Exception as exc:                                    # noqa: BLE001
        return False, str(exc)


def cmd_doctor(args) -> int:
    print("== RL-CAT ESP32 bridge doctor ==")
    ok = True

    ssid = _wlan_ssid()
    if ssid is None:
        print("  ?    could not read the WiFi SSID (not Windows?) -- check manually")
    elif ssid.upper().startswith("RL-CAT"):
        print("  [ok]  this PC is on the bridge AP: %s" % ssid)
    else:
        ok = False
        print("  [!!]  this PC is on '%s', NOT on the bridge AP." % ssid)
        print("        Connect to the AP shown in the ESP32 boot log, e.g. RL-CAT-E959")
        print("        (password rlcat1234). Nothing else can work until you do.")

    reach_console, err_c = _try_connect(args.host, args.port_console)
    reach_bridge, err_b = _try_connect(args.host, args.port_bridge)
    print("  %s  TCP %s:%d (console) %s" % ("[ok]" if reach_console else "[!!]",
                                            args.host, args.port_console,
                                            "" if reach_console else "-- " + err_c))
    print("  %s  TCP %s:%d (bridge)  %s" % ("[ok]" if reach_bridge else "[!!]",
                                            args.host, args.port_bridge,
                                            "" if reach_bridge else "-- " + err_b))
    if not (reach_console and reach_bridge):
        ok = False
        print("        If the SSID above is right but the ports are closed, reboot the")
        print("        board (or re-flash) and look at the boot log for 'TCP servers listening'.")

    if reach_console:
        con = _connect(args.host, args.port_console)
        _read_for(con, 0.4)
        r = _ask(con, "stats", wait=0.8)
        print("  -- console probe --")
        for line in r.replace("\r\n", "\n").splitlines():
            if line.strip() and not line.startswith("I (") and not line.startswith("W ("):
                print("     %s" % line.strip()[:120])
        live = [l for l in r.splitlines() if "alive:" in l]
        if live:
            print("  [ok]  the ESP32 is alive and its logs reach this port")

        r2 = _ask(con, "i", wait=0.6)
        for line in r2.replace("\r\n", "\n").splitlines():
            if line.strip().startswith(("link:", "      ")):
                print("     %s" % line.strip()[:120])

        # ---- interpret the counters: the usual source of "no reaction" -------
        import re
        uart_rx = uart_tx = baud = None
        for line in r.splitlines():
            if "uart:" in line:
                m = re.search(r"@(\d+)\s+baud\s+rx=(\d+)\s+tx=(\d+)", line)
                if m:
                    baud, uart_rx, uart_tx = int(m.group(1)), int(m.group(2)), int(m.group(3))
        frames = crc = None
        m = re.search(r"frames=(\d+)\s*(?:\([^)]*\))?.*?crc_err=(\d+)", r2)
        if m:
            frames, crc = int(m.group(1)), int(m.group(2))
        elif "frames=" in r2:
            m = re.search(r"frames=(\d+)", r2)
            frames = int(m.group(1)) if m else None

        print("  -- UART verdict (ESP32 side) --")
        if uart_rx is not None:
            print("     ESP32 UART: %s baud, received %d bytes, sent %d bytes"
                  % (baud, uart_rx, uart_tx))
            if uart_rx == 0:
                print("     [!!] NOTHING arrived on GPIO7 -> this is electrical, not protocol:")
                print("          - STM32 powered on? (heartbeat LED must blink)")
                print("          - wires crossed? ESP32 GPIO6 -> STM32 RX, ESP32 GPIO7 <- STM32 TX")
                print("          - common GND connected?")
                print("          - if you wired USART3 (PB10/PB11): the STM32 sends NOTHING there")
                print("            (known gap, see docs/36-esp32-plan.md §2) -> use USART1 instead")
            elif frames == 0:
                print("     [!!] bytes arrive but no valid frame -> almost always a BAUDRATE")
                print("          mismatch. ESP32 is at %d baud; STM32 USART1 is 1000000 and" % baud)
                print("          USART3 is 921600. Fix with:  cfg baud <n>   then reboot.")
            else:
                print("     [ok]  %d valid frames decoded -> link and baud are correct" % frames)
                if crc:
                    print("          (%d CRC errors seen -- check grounding / wire length)" % crc)
        con.close()

    if reach_bridge:
        obs = Observer(args.host, args.port_bridge)
        obs.pump(1.5)
        n = len(obs.frames)
        raw = sum(1 for _ in obs.frames)
        if n:
            print("  [ok]  bridge port is streaming %d valid frames in 1.5 s" % n)
        else:
            print("  [  ]  bridge port is silent (no frames in 1.5 s)")
            print("        - nothing wired to ESP32 GPIO6/GPIO7?")
            print("        - baud mismatch? check the boot log: 'UART1 ready ... <baud> 8N1'")
            print("        - STM32 may still be off / not sending")
        obs.sock.close()

    print()
    if ok:
        print("doctor: network path to the ESP32 looks healthy")
        print("next: open TCP %d and type '?' + Enter (STM32 shell), or TCP %d for the ESP32 console"
              % (args.port_bridge, args.port_console))
        return 0
    print("doctor: problems found above -- fix them before using PuTTY / the serial tool")
    return 1


# --------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("selftest", help="validate the tool + codec (no hardware)").set_defaults(
        func=cmd_selftest)

    p = sub.add_parser("loop", help="TCP loss/throughput test (needs UART loopback jumper)")
    p.add_argument("--host", default="192.168.4.1")
    p.add_argument("--port", type=int, default=3333)
    p.add_argument("--bytes", type=int, default=256 * 1024)
    p.add_argument("--chunk", type=int, default=1024)
    p.set_defaults(func=cmd_loop)

    p = sub.add_parser("mon", help="decode frames from a serial port")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--raw", action="store_true", help="hex dump instead of decoding")
    p.set_defaults(func=cmd_mon)

    p = sub.add_parser("sim", help="be the STM32: send TEL_FAST, decode commands")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--hz", type=float, default=100.0)
    p.add_argument("--voltage", type=int, default=11500, help="battery mV")
    p.add_argument("--link-ok", type=int, default=0, help="set STAT_LINK_OK (1 = pretend the Pi is online)")
    p.add_argument("--ready", type=int, default=1)
    p.add_argument("--calibrated", type=int, default=1)
    p.add_argument("--imu-ok", type=int, default=1)
    p.add_argument("--status-extra", type=lambda s: int(s, 0), default=0)
    p.add_argument("--amplitude", type=float, default=0.25, help="synthetic joint motion (rad)")
    p.add_argument("--serve", action="store_true", help="answer HEARTBEAT / REQ_STATUS")
    p.add_argument("--duration", type=float, default=0, help="seconds (0 = until ctrl-c)")
    p.set_defaults(func=cmd_sim)

    p = sub.add_parser("tcp", help="bridge-port client (raw or decoded)")
    p.add_argument("--host", default="192.168.4.1")
    p.add_argument("--port", type=int, default=3333)
    p.add_argument("--raw", action="store_true")
    p.add_argument("--send-hex")
    p.add_argument("--send-text")
    p.add_argument("--frames", type=int, default=0, help="stop after N frames")
    p.add_argument("--duration", type=float, default=5.0)
    p.set_defaults(func=cmd_tcp)

    p = sub.add_parser("con", help="ESP32 console client (port 3334)")
    p.add_argument("--host", default="192.168.4.1")
    p.add_argument("--port", type=int, default=3334)
    p.add_argument("--cmd", action="append", help="send one command and print the reply (repeatable)")
    p.set_defaults(func=cmd_con)

    p = sub.add_parser("doctor", help="check WiFi/ports/UART and say what is wrong")
    p.add_argument("--host", default="192.168.4.1")
    p.add_argument("--port-bridge", type=int, default=3333)
    p.add_argument("--port-console", type=int, default=3334)
    p.set_defaults(func=cmd_doctor)

    p = sub.add_parser("verify",
                       help="run the acceptance sheet against a board (see docs/37-esp32-acceptance.md)")
    p.add_argument("--host", default="192.168.4.1")
    p.add_argument("--port-bridge", type=int, default=3333)
    p.add_argument("--port-console", type=int, default=3334)
    p.add_argument("--bytes", type=int, default=256 * 1024)
    p.add_argument("--chunk", type=int, default=1024)
    p.add_argument("--loopback", action="store_true",
                   help="a jumper is in place between GPIO6 and GPIO7 (enables telemetry/arbitration steps)")
    p.add_argument("--report", help="write a JSON report to this path")
    p.set_defaults(func=cmd_verify)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
