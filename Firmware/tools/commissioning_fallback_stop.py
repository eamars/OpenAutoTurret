"""One last CAN output request after a commissioning child has exited.

The launcher owns the station motion lock while this runs. This closes the
child-crash gap on a live Pi; it cannot protect against host power or OS loss.
"""
import socket
import struct
import sys
import time


def main() -> None:
    if len(sys.argv) != 2 or sys.argv[1] not in {"yaw", "pitch"}:
        raise SystemExit("usage: commissioning_fallback_stop.py yaw|pitch")
    axis = sys.argv[1]
    if axis == "yaw":
        interface, can_id = "can0", 0x1FF  # GM6020 group 0, slot 1: zero voltage.
    else:
        interface, can_id = "can1", socket.CAN_EFF_FLAG | 0x0400007F  # CyberGear STOP, ID 0x7f.
    frame = struct.pack("=IB3x8s", can_id, 8, bytes(8))
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.bind((interface,))
        for _ in range(20):
            bus.send(frame)
            time.sleep(0.005)
    print(f"Commissioning fallback {axis} stop requested over {interface}; no disable confirmation")


if __name__ == "__main__":
    main()
