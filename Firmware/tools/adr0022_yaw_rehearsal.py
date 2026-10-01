"""One normal-path local rehearsal of the real commissiond yaw acquisition.

CAN devices and IMU hardware are substituted by loopback UDP and a raw JSON
pipe. This provides process/I/O evidence; it does not qualify the physical plant.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import selectors
import shlex
import socket
import struct
import subprocess
import time

from adr0022_yaw_data import summarize

WIRE = struct.Struct("=IB3x8s")
AMPS_PER_RAW = 3.0 / 16384


def rehearse(binary: Path, output: Path, runner=(), source_manifest: Path | None = None) -> dict:
    output.mkdir(parents=True, exist_ok=False)
    reservations = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2)]
    for sock in reservations:
        sock.bind(("127.0.0.1", 0))
    ports = [sock.getsockname()[1] for sock in reservations]
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(("127.0.0.1", 0))
    peer.setblocking(False)
    read_fd, write_fd = os.pipe()
    config = {
        "schema": "adr0022.yaw-acquisition/1", "purpose": "yaw_current_identification",
        "provenance": "SYNTHETIC", "transport": "loopback_udp",
        "expected_pitch_uid": "7216313130333105", "pitch_supported_when_disabled": True,
        "yaw": {"port": ports[0], "peer_port": peer.getsockname()[1]},
        "pitch": {"port": ports[1], "peer_port": peer.getsockname()[1]},
        "imu_fd": read_fd, "output": str((output / "capture.jsonl").resolve()),
        "baseline_s": 2.0, "stop_observation_s": 2.0,
        "yaw_current_bound_A": 1.62, "pitch_maximum_temperature_C": 60.0,
        "current_segments": [
            {"duration_s": .4, "start_A": 0.0, "end_A": .75},
            {"duration_s": .4, "start_A": .75, "end_A": 0.0},
            {"duration_s": .4, "start_A": 0.0, "end_A": -.75},
            {"duration_s": .4, "start_A": -.75, "end_A": 0.0},
        ],
        "limits": {"clock_uncertainty_s": .001, "dequeue_age_s": .08,
                   "can_gap_s": .1, "imu_gap_s": .12, "startup_s": .3,
                   "duration_s": 10.0, "minimum_imu_status": 0,
                   "read_timeout_s": .15, "read_period_s": .01, "stop_period_s": .02},
    }
    if source_manifest:
        requested = json.loads(source_manifest.read_text())
        local = {key: config[key] for key in ("provenance", "transport", "yaw", "pitch", "imu_fd", "output")}
        local_limits = config["limits"]
        config.update(requested)
        config.update(local)
        config["limits"] = {**requested.get("limits", {}), **local_limits}
        config["limits"]["duration_s"] = max(10.0, float(config["baseline_s"])+
            sum(float(row["duration_s"]) for row in config["current_segments"])+
            float(config["stop_observation_s"])+2.0)
    synthetic_acceleration_gain = 2.0 if config.get("yaw_displacement_target_deg") is not None else .35
    manifest = output / "manifest.json"
    manifest.write_text(json.dumps(config, indent=2) + "\n")
    for sock in reservations:
        sock.close()
    command = [*runner, str(binary.resolve()), "--acquire-yaw", str(manifest.resolve())]
    child = subprocess.Popen(command, pass_fds=(read_fd,), stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True)
    os.close(read_fd)
    selector = selectors.DefaultSelector()
    selector.register(child.stdout, selectors.EVENT_READ)
    first = ""
    events = []
    sent_yaw = 0
    sent_imu = 0
    position = 0.0
    speed = 0.0
    current = 0.0
    try:
        if not selector.select(10):
            raise RuntimeError("process did not report readiness")
        first = child.stdout.readline()
        if not first or json.loads(first).get("kind") != "capture_ready":
            raise RuntimeError("process exited before acquisition readiness: " + first)
        started = time.monotonic()
        previous = started
        yaw_due = started
        imu_due = started
        while child.poll() is None and time.monotonic() - started < config["limits"]["duration_s"]+2:
            now = time.monotonic()
            dt = now - previous
            previous = now
            speed += (synthetic_acceleration_gain * current - 4.0 * speed) * dt
            position += speed * dt
            for _ in range(64):
                try:
                    request, address = peer.recvfrom(1024)
                except BlockingIOError:
                    break
                can_id, dlc, data = WIRE.unpack(request)
                if dlc != 8:
                    raise RuntimeError("unexpected command DLC")
                if not can_id & 0x80000000:
                    if can_id != 0x1FE or data[2:] != bytes(6):
                        raise RuntimeError("unexpected yaw command")
                    raw = struct.unpack_from(">h", data)[0]
                    current = raw * AMPS_PER_RAW
                    events.append({"axis": "yaw", "received_ns": time.monotonic_ns(),
                                   "current_raw": raw, "current_A": current})
                    continue
                kind = (can_id >> 24) & 31
                if can_id & 255 != 127 or kind not in (0, 4, 17):
                    raise RuntimeError("unexpected pitch motion command")
                events.append({"axis": "pitch", "received_ns": time.monotonic_ns(), "type": kind})
                if kind == 0:
                    response_id = 0x80007FFE
                    response = bytes.fromhex("7216313130333105")
                elif kind == 4:
                    if data != bytes(8):
                        raise RuntimeError("pitch STOP cleared faults")
                    response_id = 0x82007F00
                    response = struct.pack(">HHHH", 32768, 32768, 32768, 280)
                else:
                    index = struct.unpack_from("<H", data)[0]
                    value = {0x7005: 2, 0x7019: 0.0, 0x701A: 0.0}[index]
                    response_id = 0x91007F00
                    response = struct.pack("<H2x", index) + (
                        struct.pack("<B3x", value) if index == 0x7005 else struct.pack("<f", value))
                peer.sendto(WIRE.pack(response_id, 8, response), address)
            if now >= yaw_due:
                for _ in range(min(16, 1 + int((now-yaw_due) / .001))):
                    yaw_due += .001
                    sent_yaw += 1
                    encoder = (5773 + round(position * 8192 / (2 * math.pi))) % 8192
                    rpm = round(speed * 60 / (2 * math.pi))
                    raw_current = round(current / AMPS_PER_RAW)
                    data = struct.pack(">HhhBB", encoder, rpm, raw_current, 28, 0)
                    peer.sendto(WIRE.pack(0x205, 8, data), ("127.0.0.1", ports[0]))
            if now >= imu_due:
                imu_due += .02
                sent_imu += 1
                stamp = time.monotonic_ns()
                orientation = [0, 0, math.sin(position / 2), math.cos(position / 2)]
                for sensor, values in (("gyro", [0, 0, speed]), ("accel", [0, 0, 9.81]),
                                       ("rv", orientation), ("game_rv", orientation)):
                    sample = {"kind": "sample", "sensor": sensor,
                              "sample_ns": stamp-100000, "rx_ns": stamp,
                              "sh2_us": stamp // 1000, "sequence": sent_imu & 255,
                              "generation": 0, "status": 0 if sensor == "gyro" else 3,
                              "values": values}
                    try:
                        os.write(write_fd, (json.dumps(sample) + "\n").encode())
                    except BrokenPipeError:
                        # The recorder closes its IMU descriptor on completion.
                        # Collect the actual exit/footer below before judging it.
                        break
            time.sleep(.0004)
        stdout, stderr = child.communicate(timeout=3)
        summary = summarize(output / "capture.jsonl", manifest)
        records = [json.loads(line) for line in (output / "capture.jsonl").read_text().splitlines()]
        stop_begin = next(row["time_ns"] for row in records if row.get("kind") == "stop_observation_begin")
        stop_elapsed_s = (summary["footer"]["end_ns"]-stop_begin)/1e9
        target_requested = config.get("yaw_displacement_target_deg") is not None
        requires_positive = any(max(float(row["start_A"]), float(row["end_A"])) > 0 for row in config["current_segments"])
        requires_negative = any(min(float(row["start_A"]), float(row["end_A"])) < 0 for row in config["current_segments"])
        yaw_events = [row for row in events if row["axis"] == "yaw"]
        normal_path = {
            "positive_current_transmissions": sum(row["current_raw"] > 0 for row in yaw_events),
            "negative_current_transmissions": sum(row["current_raw"] < 0 for row in yaw_events),
            "zero_current_transmissions": sum(row["current_raw"] == 0 for row in yaw_events),
            "last_current_A": yaw_events[-1]["current_A"] if yaw_events else None,
            "raw_yaw_retained": summary["record_counts"].get("yaw_feedback", 0) > 0,
            "raw_can_retained": summary["record_counts"].get("can_rx", 0) > 0,
            "raw_imu_sensors_retained": all(summary["imu_sample_counts"].get(sensor, 0) > 0
                                            for sensor in ("gyro", "accel", "rv", "game_rv")),
            "stop_observation_elapsed_s": stop_elapsed_s,
            "full_zero_observation": stop_elapsed_s >= 2.0 and summary["footer"]["zero_request_completed"] and
                all(row["successful_tx_A"] == 0 for row in records if row.get("kind") == "yaw_current_tx" and row["phase"] == "stop" and row["success"]),
            "target_requested": target_requested,
            "target_observed": summary["footer"].get("yaw_target_reached", False) and
                any(row.get("kind") == "yaw_displacement_target_observed" for row in records),
        }
        result = {"provenance": "SYNTHETIC", "hardware_accessed": False, "command": command,
                  "returncode": child.returncode, "normal_path": normal_path,
                  "summary": summary, "peer_events": events, "stdout": first+stdout,
                  "stderr": stderr, "offered_yaw_frames": sent_yaw, "offered_imu_samples_per_sensor": sent_imu,
                  "source_manifest": str(source_manifest.resolve()) if source_manifest else None,
                  "synthetic_plant": {"acceleration_gain_rad_s2_per_A": synthetic_acceleration_gain,
                      "velocity_decay_per_s": 4.0, "physical_model_qualified": False}}
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        if not (child.returncode == 0 and summary["capture_complete"] and
                (not requires_positive or normal_path["positive_current_transmissions"] > 0) and
                (not requires_negative or normal_path["negative_current_transmissions"] > 0) and
                normal_path["zero_current_transmissions"] > 0 and
                normal_path["last_current_A"] == 0 and normal_path["raw_yaw_retained"] and
                normal_path["raw_can_retained"] and normal_path["raw_imu_sensors_retained"] and
                normal_path["full_zero_observation"] and
                (not target_requested or normal_path["target_observed"])):
            raise RuntimeError("normal acquisition path did not complete; see result.json")
        return result
    except Exception as exc:
        if child.poll() is None:
            child.terminate()
        try:
            stdout, stderr = child.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            child.kill()
            stdout, stderr = child.communicate(timeout=3)
        (output / "failure.json").write_text(json.dumps({"detail": str(exc), "command": command,
            "returncode": child.returncode, "stdout": first+stdout, "stderr": stderr,
            "peer_events": events}, indent=2) + "\n")
        raise
    finally:
        selector.close()
        peer.close()
        os.close(write_fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="new unused output directory")
    parser.add_argument("--runner", default="", help="optional user-mode emulator command prefix")
    parser.add_argument("--manifest", type=Path, help="requested waveform/target shape; transport and runtime remain synthetic")
    args = parser.parse_args()
    result = rehearse(args.binary, args.output, tuple(shlex.split(args.runner)), args.manifest)
    print(json.dumps({"provenance": result["provenance"], "returncode": result["returncode"],
                      "normal_path": result["normal_path"], "summary": result["summary"]}, indent=2))


if __name__ == "__main__":
    main()
