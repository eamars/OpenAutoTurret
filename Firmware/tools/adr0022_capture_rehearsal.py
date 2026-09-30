"""Local process/I/O rehearsal. Emits SYNTHETIC CAN over loopback and IMU over a pipe.

Never opens SocketCAN, I2C, SSH, or motor outputs. Runs the actual commissiond
acquisition process, including its writer, clock checks and sensor decoders.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import selectors
import socket
import struct
import subprocess
import time


def rehearse(binary: Path, root: Path, *, duration: float = 3.0, fault: str = "none",
             runner: tuple[str, ...] = (), native_settings: bool = False) -> dict:
    root.mkdir(parents=True, exist_ok=False)
    ports = []
    reservations = []
    for _ in range(2):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        ports.append(sock.getsockname()[1])
        reservations.append(sock)
    read_fd, write_fd = os.pipe()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.bind(("127.0.0.1", 0))
    sender.setblocking(False)
    manifest = {
        "schema": "adr0022.capture/2", "provenance": "SYNTHETIC", "transport": "loopback_udp",
        "output": str((root / "capture.jsonl").resolve()), "imu_fd": read_fd,
        "yaw": {"port": ports[0]}, "pitch": {"port": ports[1], "peer_port": sender.getsockname()[1]},
        "register_reads": True,
        "pitch_stop_poll": True, "pitch_supported_when_disabled": True, "expected_pitch_uid": "7216313130333105",
        "limits": {"clock_uncertainty_s": .001, "dequeue_age_s": .08,
                   "can_gap_s": .10, "imu_gap_s": .12, "startup_s": .3,
                   "duration_s": duration, "minimum_imu_status": 0,
                   "read_timeout_s": .15, "read_period_s": .01, "stop_period_s": .02},
    }
    if native_settings:
        manifest["additional_startup_registers"] = [0x701E,0x701F,0x7020,0x7017]
    config = root / "manifest.json"
    config.write_text(json.dumps(manifest, indent=2) + "\n")
    for sock in reservations:
        sock.close()
    # A local user-mode emulator may prefix the actual target executable. Keep
    # argv structured and record both identities; never disguise a shell wrapper
    # as the target binary or count emulation as station execution.
    command = [*runner, str(binary.resolve()), "--capture-baseline", str(config.resolve())]
    executable_sha256=hashlib.sha256(binary.read_bytes()).hexdigest()
    child = subprocess.Popen(command,
                             pass_fds=(read_fd,), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    os.close(read_fd)
    selector = selectors.DefaultSelector()
    selector.register(child.stdout, selectors.EVENT_READ)
    first = ""
    counts = [0, 0, 0]
    replies = 0
    discoveries = 0
    try:
        if not selector.select(10):
            raise RuntimeError("capture did not become ready")
        first = child.stdout.readline()
        if not first:
            raise RuntimeError("capture exited before readiness; see failure.json for process diagnostics")
        if json.loads(first).get("kind") != "capture_ready":
            raise RuntimeError("capture refused startup: " + first)
        start = time.monotonic()
        due = [start, start, start]
        periods = [.001, .01, .02]
        while child.poll() is None and time.monotonic() - start < duration + 2:
            now = time.monotonic()
            elapsed = now - start
            try:
                request, address = sender.recvfrom(1024)
            except BlockingIOError:
                pass
            else:
                can_id, dlc, data = struct.unpack("=IB3x8s", request)
                kind = (can_id & 0x1F000000) >> 24
                assert kind in (0, 4, 17), "acquisition attempted enable or excitation"
                assert can_id & 255 == 127 and dlc == 8
                if kind == 0:
                    assert data == bytes(8)
                    response_id = 0x80000000 | (127 << 8) | 0xFE
                    uid = "7216313130333105" if fault != "wrong_uid" else "7216313130333106"
                    sender.sendto(struct.pack("=IB3x8s", response_id, 8, bytes.fromhex(uid)), address)
                    discoveries += 1
                elif kind == 4:
                    assert data == bytes(8), "STOP must not clear faults"
                    if fault != "stop_timeout" or elapsed <= .6:
                        counts[1] += 1
                        response_id = 0x80000000 | (2 << 24) | (127 << 8)
                        if fault == "reenabled" and elapsed > .6:
                            response_id |= 2 << 22
                        data = struct.pack(">HHHH", 32768 + counts[1] % 1000, 32768, 32768, 345)
                        sender.sendto(struct.pack("=IB3x8s", response_id, 8, data), address)
                else:
                    index = struct.unpack_from("<H", data)[0]
                    value = {0x7005: 1, 0x7014: .1, 0x7018: 5., 0x7010: .2, 0x7011: .3,
                         0x7019: 0., 0x701C: 24., 0x701A: .25,
                         0x701E:30.,0x701F:1.,0x7020:.002,0x7017:.175}[index]
                    response = struct.pack("<H2x", index) + (struct.pack("<B3x", value) if index == 0x7005
                                                         else struct.pack("<f", value))
                    response_id = 0x80000000 | (17 << 24) | (127 << 8)
                    if fault == "read_echo" and elapsed > .6:
                        response_id = 0x80000000 | (18 << 24) | (127 << 8)
                    if fault == "read_source" and elapsed > .6:
                        response_id = 0x80000000 | (17 << 24) | (126 << 8)
                    if fault == "read_rejected" and index in (0x7019, 0x701A):
                        response_id = 0x80000000 | 0x11017F00
                        # Existing captured factory NACK layout; the last four
                        # bytes are stale data and must never become a value.
                        response = struct.pack("<H", index) + b"\x00\x00\x30\x33\x31\x05"
                    if not (fault == "read_timeout" and elapsed > .6):
                        sender.sendto(struct.pack("=IB3x8s", response_id, 8, response), address)
                        replies += 1
            for axis in range(2):
                if axis == 1:
                    continue  # CyberGear need not send unsolicited type-2 feedback.
                if now < due[axis]:
                    continue
                for _ in range(min(16, 1 + int((now-due[axis])/periods[axis]))):
                    # Preserve the nominal producer rate instead of adding scheduler
                    # overshoot to every period. Actual kernel receipt timestamps
                    # still expose late/bunched delivery; none are manufactured.
                    due[axis] += periods[axis]
                    if fault == "can_stale" and axis == 0 and elapsed > .6:
                        continue
                    counts[axis] += 1
                    if axis == 0:
                        data = struct.pack(">HhhBB", counts[axis] % 8192, -1, 1365, 37, 0)
                        can_id = 0x205
                    else:
                        data = struct.pack(">HHHH", 32768 + counts[axis] % 1000, 32768, 32768, 345)
                        can_id = 0x80000000 | (2 << 24) | (2 << 22) | (127 << 8)
                    if fault == "can_error" and elapsed > .6 and axis == 0:
                        can_id = 0x20000004
                    wire = struct.pack("=IB3x8s", can_id, 8, data)
                    if fault == "can_truncated" and elapsed > .6 and axis == 0:
                        wire += b"EXTRA"
                    sender.sendto(wire, ("127.0.0.1", ports[axis]))
            if now >= due[2]:
                due[2] += periods[2]
                counts[2] += 1
                sequence = counts[2] & 255
                if fault == "imu_sequence" and elapsed > .6:
                    sequence = (sequence + 1) & 255
                generation = int(fault == "imu_reset" and elapsed > .6)
                if fault == "imu_eof" and elapsed > .6:
                    os.close(write_fd)
                    write_fd = -1
                    break
                stamp = time.monotonic_ns()
                for sensor, values in (("accel", [0, 0, 9.81]), ("gyro", [0, 0, .01]),
                                       ("rv", [0, 0, 0, 1]), ("game_rv", [0, 0, 0, 1])):
                    sample = {"kind": "sample", "sensor": sensor, "sample_ns": stamp - 100000,
                              "rx_ns": stamp, "sh2_us": stamp // 1000, "sequence": sequence,
                              "generation": generation, "status": 3, "values": values}
                    try:
                        os.write(write_fd, (json.dumps(sample) + "\n").encode())
                    except BrokenPipeError:
                        break
            time.sleep(.0004)
        stdout, stderr = child.communicate(timeout=5)
        records = [json.loads(line) for line in (root / "capture.jsonl").read_text().splitlines()]
        complete = records[-1].get("status") == "COMPLETE"
        offered_load = None
        if fault in ("none", "read_rejected"):
            assert child.returncode == 0 and complete, stderr
            assert records[0]["provenance"] == "SYNTHETIC"
            can = [row for row in records if row["kind"] == "can_rx"]
            for axis, count in zip(("yaw", "pitch"), (counts[0], counts[1] + replies + discoveries)):
                rows = [row for row in can if row["axis"] == axis]
                assert len(rows) == records[-1][axis + "_frames"]
                # Exclude traffic sent after the capture interval, including
                # while the child flushes its file. Prove no internal hole from
                # independent wire counters, not the child's own receive count.
                assert 0 < len(rows) <= count
                assert [row["sequence"] for row in rows] == list(range(1, len(rows) + 1))
                assert all(row["drop_delta"] == 0 and row["clock_uncertainty_ns"] <= 1000000 for row in rows)
                feedback = [row for row in rows if "angle_raw" in row]
                origin, modulus = (0, 8192) if axis == "yaw" else (32768, 1000)
                assert [row["angle_raw"] for row in feedback] == [origin + i % modulus for i in range(1, len(feedback) + 1)]
            assert any(row.get("temperature_C") == 34.5 for row in can)
            assert all(row["temperature_C"] is None for row in can if row["axis"] == "yaw")
            assert all(row["current_A"] is None for row in can if row["axis"] == "yaw")
            imu = [json.loads(r["raw_json"]) for r in records if r["kind"] == "imu_raw"]
            assert imu and all(sum(r["sensor"] == sensor for r in imu) > 0 for sensor in ("gyro", "accel", "rv", "game_rv"))
            for sensor in ("gyro", "accel", "rv", "game_rv"):
                sequence = [r["sequence"] for r in imu if r["sensor"] == sensor]
                assert sequence == [i & 255 for i in range(1, len(sequence) + 1)]
            reads = [r for r in records if r["kind"] == "register_read"]
            rejected = [r for r in records if r["kind"] == "register_rejected"]
            assert len(reads) == records[-1]["register_reads"]
            assert len(rejected) == records[-1]["register_rejections"]
            assert len(reads) + len(rejected) == replies
            if fault == "read_rejected":
                assert {r["index"] for r in rejected} == {0x7019, 0x701A} and len(rejected) == 2
                assert all(r["value"] is None for r in rejected)
                assert not any(r["index"] in (0x7019, 0x701A) for r in reads)
            else:
                assert not rejected and any(r["index"] == 0x701A and r["value"] == .25 for r in reads)
            assert all(r["device_sample_ns"] is None for r in reads)
            yaw_times = [r["kernel_monotonic_ns"] for r in can if r["axis"] == "yaw"]
            yaw_hz = (len(yaw_times)-1)*1e9/(yaw_times[-1]-yaw_times[0])
            offered_load = {"nominal_yaw_hz": 1/periods[0], "observed_yaw_hz": yaw_hz,
                            "minimum_load_fraction": .99}
            assert yaw_hz >= .99/periods[0], "test generator did not supply the required yaw load"
        else:
            assert child.returncode != 0 and not complete, "fault capture was accepted"
            expected = {"can_stale": "feedback absent/stale", "can_error": "CAN error frame",
                        "can_truncated": "truncated", "imu_sequence": "sequence loss",
                        "imu_reset": "generation changed", "imu_eof": "producer EOF",
                        "read_echo": "uncorrelated", "read_source": "uncorrelated",
                        "read_timeout": "read timeout", "wrong_uid": "discovery identity",
                        "reenabled": "became enabled", "stop_timeout": "feedback"}[fault]
            assert expected in records[-1]["detail"], records[-1]
        result = {"provenance": "SYNTHETIC", "fault": fault, "returncode": child.returncode,
                  "executable_sha256": executable_sha256,
                  "command": command, "emulated": bool(runner),
                  "offered_load": offered_load,
                  "records": len(records), "result": records[-1], "stdout": first + stdout,
                  "stderr": stderr, "hardware_accessed": False}
        assert hashlib.sha256(binary.read_bytes()).hexdigest()==executable_sha256, "executable changed during rehearsal"
        (root / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        return result
    except Exception as exc:
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)
        stdout, stderr = child.communicate(timeout=5)
        failure = {"provenance": "SYNTHETIC", "hardware_accessed": False,
                   "command": command, "returncode": child.returncode,
                   "executable_sha256": executable_sha256,
                   "detail": str(exc), "stdout": first + stdout, "stderr": stderr}
        (root / "failure.json").write_text(json.dumps(failure, indent=2) + "\n")
        raise
    finally:
        selector.close()
        sender.close()
        if write_fd >= 0:
            os.close(write_fd)
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=3.)
    parser.add_argument("--native-settings",action="store_true",help="read native homing settings while disabled")
    parser.add_argument("--runner", action="append", default=[], metavar="ARG",
                        help="local emulator argv prefix; repeat per token, e.g. --runner=qemu-aarch64 --runner=-L --runner=SYSROOT")
    parser.add_argument("--fault", choices=("none", "read_rejected", "can_stale", "can_error", "can_truncated",
                                           "imu_sequence", "imu_reset", "imu_eof", "read_echo",
                                           "read_source", "read_timeout", "wrong_uid", "reenabled", "stop_timeout"), default="none")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 1:
        parser.error("duration must exceed one second")
    print(json.dumps(rehearse(args.binary, args.output, duration=args.duration, fault=args.fault,
                             runner=tuple(args.runner),native_settings=args.native_settings), indent=2))
