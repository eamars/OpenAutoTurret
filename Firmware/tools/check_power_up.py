#!/usr/bin/env python3
"""Read-only power-up check for the yaw axis: is the drive actually alive before anything moves?

Why this exists: the yaw slip ring uses screw terminals with no net labels, so a reversed pair is
undetectable by looking. A swapped CAN pair shows up as *no feedback at all*; a swapped supply shows
up as no feedback too, and the drive's guide documents over-temperature and over-voltage protection
but says nothing about reverse polarity. So the first question after reassembly is not "does it
move" -- it is "is it even talking", and that question must be answerable without commanding motion.

Sends nothing. Reads `0x204 + motor_id` for a moment and reports frames/sec, the temperature byte,
and the interface's own error counters. Exit 0 alive, 2 no feedback, 3 bus is erroring.
"""
import argparse
import socket
import struct
import subprocess
import sys
import time

FRAME = struct.Struct("@IBBBB8s")   # id, dlc, flags, reserved, len8, data[8]


def bus_errors(interface: str) -> dict:
    """The kernel's own tally, so a claim about the bus does not depend on my counting."""
    out = subprocess.run(["ip", "-s", "-details", "link", "show", interface],
                         capture_output=True, text=True).stdout
    wanted = ("bus_error", "error_passive", "bus_off", "retransmit", "quereqfull")
    return {k: out.lower().count(k) for k in wanted}


def main() -> int:
    ap = argparse.ArgumentParser(description="passive GM6020 power-up check")
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--motor-id", type=int, default=1)
    ap.add_argument("--seconds", type=float, default=2.0)
    args = ap.parse_args()
    if not 1 <= args.motor_id <= 4:
        print("用法错误: motor id 只能是 1..4")
        return 2
    expected = 0x204 + args.motor_id

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.interface,))          # 1-tuple: AF_CAN addresses carry no protocol field
    sock.settimeout(0.05)
    errors_before = bus_errors(args.interface)

    frames = 0
    temps = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        try:
            cid, dlc, _pad, _res, _len8, data = FRAME.unpack(sock.recv(16))
        except socket.timeout:
            continue
        if cid != expected or dlc < 7:
            continue
        frames += 1
        temps.append(data[6])            # guide printed p.8: byte 6 is the temperature field
    sock.close()
    hz = frames / max(time.monotonic() - t0, 1e-9)
    errors_after = bus_errors(args.interface)
    grew = {k: errors_after[k] - errors_before[k] for k in errors_after
            if errors_after[k] - errors_before[k] > 0}

    print("feedback 0x%03X: %d 帧，%.0f Hz（标称 1 kHz）；温度字节 max=%s；总线错误增量 %s"
          % (expected, frames, hz, max(temps) if temps else "无", grew or "无"))
    if grew:
        print("结论：总线在出错 —— 先别动，看线")
        return 3
    if hz < 500:
        print("结论：不见反馈（或远低于 1 kHz）—— 这个轴没在说话，不许发运动命令")
        return 2
    print("结论：驱动活着。允许继续，但运动命令仍然是另一道决定")
    return 0


if __name__ == "__main__":
    sys.exit(main())
