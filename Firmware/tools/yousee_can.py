#!/usr/bin/env python3
"""yousee USB-CAN (AT mode) side-channel tool — independent bus observer.

Recovered from ``legacy/yourcee_usb_to_can.py`` (the YourCeeSerialBus
python-can driver, CH340 CDC serial @ 921600, AT-framed data mode; the
adapter speaks CyberGear's own wit-motion protocol family). Used to verify
and ISOLATE the can0/MCP2515 fault: it can watch/send on the same physical
bus while the HAT is quiesced, so "the bus is dead" vs "the HAT is deaf"
finally become separately testable.

Subcommands
  init            configure the adapter (AT+CAN_BAUD=<bitrate>, AT+AT mode)
  probe           send CyberGear register READs via the adapter, print replies
  sniff [secs]    listen, print frames with host timestamps (default 10 s)
  watch [secs]    sniff PLUS a periodic probe read of loc_kp on 0x64/0x65:
                  a continuous "are the drives answering ME" verdict while
                  other things happen on the bus
  send <id> <hex> transmit one frame (id may be hex, e.g. 0x11000064)

Frame protocol (from the legacy driver / wit-motion docs):
  TX/RX: 'A' 'T' | id_be32 (id<<3 | ext<<2) | dlc | data | CRLF
"""
import struct
import sys
import time

import serial

DEFAULT_PORT = "/dev/ttyUSB0"
UART_BAUD = 921600


class Yousee:
    def __init__(self, port=DEFAULT_PORT, bitrate=1_000_000):
        self.s = serial.Serial(port, UART_BAUD, timeout=0.3)
        self.s.reset_input_buffer()
        self._at("AT+CG\r\n", "OK\r\n")
        self._at(f"AT+CAN_BAUD={bitrate}\r\n", "OK\r\n")
        self._at("AT+CAN_BAUD=?\r\n", f"+CAN_BAUD:{bitrate}\r\n")
        self._at("AT+AT\r\n", "OK\r\n")  # command (data) mode

    def _at(self, cmd, expect):
        self.s.write(cmd.encode())
        got = self.s.readline().decode(errors="replace")
        if got != expect:
            raise RuntimeError(f"AT init failed: {cmd!r} -> {got!r}")

    def send_frame(self, can_id, data, extended=True):
        cid = (can_id << 3) | (1 << 2 if extended else 0)
        buf = bytearray(b"AT")
        buf += struct.pack(">I", cid)
        buf.append(len(data))
        buf += bytes(data)
        buf += b"\r\n"
        self.s.write(buf)

    def recv_frame(self, timeout=0.3):
        self.s.timeout = timeout
        pre = self.s.read(2)
        if len(pre) != 2 or pre != b"AT":
            return None
        cid = struct.unpack(">I", self.s.read(4))[0]
        dlc = ord(self.s.read(1))
        payload = self.s.read(dlc)
        post = self.s.read(2)  # CRLF
        return {"id": cid >> 3, "ext": bool(cid >> 2 & 1), "dlc": dlc,
                "data": payload, "post_ok": post == b"\r\n"}

    def drain(self):
        n = 0
        while self.recv_frame(0.05):
            n += 1
        return n

    def close(self):
        self.s.close()


def read_reg(ads, motor, reg):
    return struct.pack("<H", reg) + b"\x00" * 6


def cmd_init(port, bitrate):
    y = Yousee(port, bitrate)
    print(f"OK: {port} configured, CAN bitrate {bitrate} (verified)")
    y.close()


def cmd_probe(port, regs=(0x701E,)):
    y = Yousee(port)
    y.drain()
    names = {0x701E: "loc_kp", 0x701F: "spd_kp", 0x7020: "spd_ki", 0x7018: "limit_cur"}
    for m in (0x64, 0x65):
        for reg in regs:
            y.send_frame((0x11 << 24) | m, read_reg(None, m, reg))
            t0 = time.time()
            got = False
            while time.time() - t0 < 1.0:
                f = y.recv_frame(0.2)
                if f and (f["id"] >> 24) & 0x1F == 0x11 and (f["id"] >> 8) & 0xFF == m:
                    val = struct.unpack("<f", f["data"][4:8])[0]
                    print(f"  0x{m:02X} {names.get(reg, hex(reg)):9s} = {val}  "
                          f"(reply in {(time.time()-t0)*1000:.0f} ms)")
                    got = True
                    break
            if not got:
                print(f"  0x{m:02X} {names.get(reg, hex(reg)):9s} : NO REPLY")
    y.close()


def cmd_sniff(port, secs, quiet=False):
    y = Yousee(port)
    t0 = time.time()
    n = 0
    while time.time() - t0 < secs:
        f = y.recv_frame(0.2)
        if f:
            n += 1
            if not quiet:
                print(f"{time.time():.6f} id=0x{f['id']:08X} ext={int(f['ext'])} "
                      f"[{f['dlc']}] {f['data'].hex()}")
    print(f"({n} frames in {secs} s)", file=sys.stderr)
    y.close()


def cmd_watch(port, secs):
    """sniff + periodic drive probe: continuous independent liveness verdict"""
    y = Yousee(port)
    end = time.time() + secs
    n = probes = ok = 0
    next_probe = 0.0
    while time.time() < end:
        if time.time() >= next_probe:
            y.send_frame((0x11 << 24) | 0x64, read_reg(None, 0x64, 0x701E))
            probes += 1
            next_probe = time.time() + 2.0
            pending_since = time.time()
            probing = True
        else:
            probing = False
        f = y.recv_frame(0.1)
        if f:
            n += 1
            if (f["id"] >> 24) & 0x1F == 0x11 and (f["id"] >> 8) & 0xFF == 0x64:
                if probing and time.time() - pending_since < 2.0:
                    ok += 1
                    print(f"{time.time():.1f} probe OK", flush=True)
    print(f"watch done: {n} frames, {probes} probes, {ok} drive replies",
          flush=True)
    y.close()


def cmd_send(port, can_id, hexdata):
    y = Yousee(port)
    y.send_frame(int(can_id, 16), bytes.fromhex(hexdata))
    print(f"sent id={int(can_id,16):#010x} data={hexdata}")
    y.close()


if __name__ == "__main__":
    c = sys.argv[1] if len(sys.argv) > 1 else "sniff"
    if c == "init":
        cmd_init(DEFAULT_PORT, int(sys.argv[2]) if len(sys.argv) > 2 else 1_000_000)
    elif c == "probe":
        regs = tuple(int(x, 16) for x in sys.argv[2:]) or (0x701E, 0x701F, 0x7020)
        cmd_probe(DEFAULT_PORT, regs)
    elif c == "sniff":
        cmd_sniff(DEFAULT_PORT, int(sys.argv[2]) if len(sys.argv) > 2 else 10)
    elif c == "watch":
        cmd_watch(DEFAULT_PORT, int(sys.argv[2]) if len(sys.argv) > 2 else 60)
    elif c == "send":
        cmd_send(DEFAULT_PORT, sys.argv[2], sys.argv[3])
    else:
        print(__doc__)
        sys.exit(1)
