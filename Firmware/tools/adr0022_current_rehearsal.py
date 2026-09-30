"""Run the real neutral current-mode entry against local protocol peers only."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import selectors
import signal
import socket
import struct
import subprocess
import time

FAULTS = ("none", "mode_ignored", "motion", "overcurrent", "yaw_motion",
          "zero_ignored", "read_rejected", "stop_ignored", "write_echo",
          "unexpected_disable", "disabled_reenable", "abort_stop_truncated",
          "can_error", "can_stale", "imu_eof", "interrupt", "overheat", "slow_read")


def rehearse(binary: Path, output: Path, *, fault="none", runner=(), characterize=False, original_mode=2, observation_s=2.):
    if original_mode not in (1,2,3):
        raise ValueError("unsupported original mode fixture")
    if not math.isfinite(observation_s) or not 0 < observation_s <= 60 or (not characterize and observation_s != 2):
        raise ValueError("observation duration must match the requested operation")
    output.mkdir(parents=True, exist_ok=False)
    reservations = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2)]
    for s in reservations:
        s.bind(("127.0.0.1", 0))
    ports = [s.getsockname()[1] for s in reservations]
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(("127.0.0.1", 0)); peer.setblocking(False)
    rd, wr = os.pipe()
    config = {
        "schema": "adr0022.neutral-characterization/1" if characterize else "adr0022.current-preparation/1",
        "provenance": "SYNTHETIC", "transport": "loopback_udp",
        "purpose": "neutral_current_measurement_characterization" if characterize else "neutral_current_mode_verification",
        "expected_pitch_uid": "7216313130333105",
        "pitch_supported_when_disabled": True, "neutral_current_bound_A": .1,
        "transition_displacement_bound_rad": .01, "pitch_maximum_temperature_C": 60.,
        "yaw": {"port": ports[0], "peer_port": peer.getsockname()[1]},
        "pitch": {"port": ports[1], "peer_port": peer.getsockname()[1]},
        "imu_fd": rd, "output": str((output/"capture.jsonl").resolve()),
        "limits": {"clock_uncertainty_s": .001, "dequeue_age_s": .08, "can_gap_s": .1,
                   "imu_gap_s": .12, "startup_s": .3, "duration_s": 10., "minimum_imu_status": 0,
                   "read_timeout_s": .15, "read_period_s": .01, "stop_period_s": .02}}
    if characterize:
        config["protection_current_bound_A"] = 6.5  # explicit synthetic fixture; physical manifest binds its rating source
        config["neutral_observation_s"] = observation_s
        config["limits"]["duration_s"] = max(10.,observation_s+config["limits"]["startup_s"]+4.)
    manifest = output/"manifest.json"
    manifest.write_text(json.dumps(config, indent=2)+"\n")
    for s in reservations:
        s.close()
    command = [*runner, str(binary.resolve()), "--characterize-current" if characterize else "--prepare-current", str(manifest.resolve())]
    executable_sha256=hashlib.sha256(binary.read_bytes()).hexdigest()
    child = subprocess.Popen(command, pass_fds=(rd,), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    os.close(rd)
    selector = selectors.DefaultSelector(); selector.register(child.stdout, selectors.EVENT_READ)
    first = ""; events = []; enabled = False; mode = original_mode; iq = 1.; seq = 0; yaw_count = 0
    injected = False; imu_closed = False; stop_count = 0; delayed_replies = []
    try:
        if not selector.select(10):
            raise RuntimeError("process readiness deadline")
        first = child.stdout.readline()
        assert first and json.loads(first)["kind"] == "capture_ready", first
        started = time.monotonic(); yaw_due = started; imu_due = started
        while child.poll() is None and time.monotonic()-started < config["limits"]["duration_s"]+2:
            now = time.monotonic()
            for due, payload, destination in delayed_replies[:]:
                if now >= due:
                    peer.sendto(payload, destination)
                    delayed_replies.remove((due, payload, destination))
            for _ in range(32):
                try:
                    request, address = peer.recvfrom(1024)
                except BlockingIOError:
                    break
                cid, dlc, data = struct.unpack("=IB3x8s", request)
                assert dlc == 8
                if not cid & 0x80000000:
                    assert cid == 0x1FE and data == bytes(8), "nonneutral yaw output"
                    continue
                kind = (cid >> 24) & 31
                assert cid & 255 == 127 and (cid >> 8) & 65535 == 0
                events.append({"type": kind, "data": data.hex(), "time": now-started})
                response = None
                if kind == 0:
                    assert data == bytes(8)
                    response = struct.pack("=IB3x8s", 0x80007FFE, 8, bytes.fromhex("7216313130333105"))
                elif kind in (3, 4, 18):
                    if kind == 3:
                        assert mode == 3 and iq == 0, "enable preceded verified neutral current mode"
                        enabled = True
                    elif kind == 4:
                        stop_count += 1
                        assert data == bytes(8), "cleared faults during STOP"
                        if not (fault == "stop_ignored" and enabled):
                            enabled = False
                        if fault == "disabled_reenable" and stop_count == 2:
                            enabled = True
                    else:
                        reg = struct.unpack_from("<H", data)[0]
                        assert reg in (0x7005, 0x7006), "unrelated register write"
                        if reg == 0x7005:
                            assert not enabled, "mode changed while enabled"
                            if fault != "mode_ignored":
                                mode = data[4]
                        else:
                            requested = struct.unpack_from("<f", data, 4)[0]
                            assert requested == 0., "nonneutral current"
                            if fault != "zero_ignored":
                                iq = requested
                        if fault == "write_echo":
                            peer.sendto(struct.pack("=IB3x8s", 0x92007F00, 8, data), address)
                    response_id = 0x82007F00 | ((2 if enabled else 0) << 22)
                    angle = 35000 if fault == "motion" and enabled else 32000
                    temperature = 700 if fault == "overheat" and enabled else 225
                    length = 0 if fault == "abort_stop_truncated" and kind == 4 and injected else 8
                    response = struct.pack("=IB3x8s", response_id, length, struct.pack(">HHHH", angle, 32768, 32768, temperature))
                elif kind == 17:
                    reg = struct.unpack_from("<H", data)[0]
                    assert reg in (0x7005, 0x7006, 0x701A)
                    sensed_current = 7. if characterize and fault in ("overcurrent", "abort_stop_truncated") else .3 if fault in ("overcurrent", "abort_stop_truncated") else 0.
                    if characterize and fault == "neutral_noise":
                        sensed_current = .2515
                    value = mode if reg == 0x7005 else iq if reg == 0x7006 else sensed_current
                    if reg == 0x701A and fault == "abort_stop_truncated":
                        injected = True
                    if reg == 0x701A and fault == "unexpected_disable":
                        enabled = False
                        peer.sendto(struct.pack("=IB3x8s", 0x82007F00, 8, struct.pack(">HHHH", 32000, 32768, 32768, 225)), address)
                    value_bytes = struct.pack("<B3x", value) if reg == 0x7005 else struct.pack("<f", value)
                    response_id = 0x91017F00 if fault == "read_rejected" and reg == 0x7006 else 0x91007F00
                    response = struct.pack("=IB3x8s", response_id, 8, struct.pack("<H2x", reg)+value_bytes)
                else:
                    raise AssertionError("unexpected command")
                if response:
                    if fault == "slow_read" and kind == 17 and reg == 0x701A:
                        delayed_replies.append((now+.03,response,address))
                    else:
                        peer.sendto(response, address)
            if enabled and not injected and fault in ("can_error", "can_stale", "imu_eof", "interrupt"):
                injected = True
                if fault == "can_error":
                    peer.sendto(struct.pack("=IB3x8s", 0x20000001, 8, bytes(8)), ("127.0.0.1", ports[0]))
                elif fault == "imu_eof":
                    os.close(wr); imu_closed = True
                elif fault == "interrupt":
                    child.send_signal(signal.SIGTERM)
            if now >= yaw_due and not (fault == "can_stale" and injected):
                for _ in range(min(16, 1+int((now-yaw_due)/.001))):
                    yaw_due += .001; yaw_count += 1
                    yaw_angle = 5773 + min(40,int(100*(now-started-2.1))) if fault == "yaw_motion" and now-started>2.1 else 5773
                    peer.sendto(struct.pack("=IB3x8s", 0x205, 8, struct.pack(">HhhBB", yaw_angle, 0, 0, 28, 0)),
                                ("127.0.0.1", ports[0]))
            if now >= imu_due and not imu_closed:
                imu_due += .02; seq += 1
                stamp = time.monotonic_ns()
                for sensor, values in (("gyro", [0,0,0]), ("accel", [0,0,9.81]), ("rv", [0,0,0,1]), ("game_rv", [0,0,0,1])):
                    row = {"kind":"sample", "sensor":sensor, "sample_ns":stamp-100000,
                           "rx_ns":stamp, "sh2_us":stamp//1000, "sequence":seq & 255,
                           "generation":0, "status":3, "values":values}
                    try:
                        os.write(wr, (json.dumps(row)+"\n").encode())
                    except BrokenPipeError:
                        break
            time.sleep(.0004)
        stdout, stderr = child.communicate(timeout=3)
        records = [json.loads(line) for line in (output/"capture.jsonl").read_text().splitlines()]
        final = records[-1]
        if fault in ("none", "write_echo", "slow_read") or (characterize and fault == "neutral_noise"):
            assert child.returncode == 0 and final["status"] == "COMPLETE", final
            assert not enabled and mode == original_mode and iq == 0, "exit did not restore original disabled mode"
            reads = [r for r in records if r["kind"] == "register_read"]
            assert [r["value"] for r in reads if r["index"] == 0x7005] == [original_mode,3,3,original_mode]
            assert sum(r["index"] == 0x7006 and r["value"] == 0 for r in reads) == 2
            assert any(r["index"] == 0x701A for r in reads)
            if characterize:
                assert final["characterization_only"] and not final["neutral_current_qualified"]
                assert final["neutral_current_criterion_satisfied"] == (fault != "neutral_noise")
        else:
            assert child.returncode != 0 and final["status"] == "INVALID", final
            expected = {"mode_ignored": "readback differs", "motion": "displacement",
                        "yaw_motion": "yaw displacement", "zero_ignored": "readback differs",
                        "read_rejected": "register rejected", "stop_ignored": "STOP confirmation timeout",
                        "overcurrent": "manufacturer current protection" if characterize else "nonneutral measured",
                        "abort_stop_truncated": "manufacturer current protection" if characterize else "nonneutral measured",
                        "unexpected_disable": "enabled current mode was lost",
                        "disabled_reenable": "re-enabled during disabled transition",
                        "can_error": "CAN error frame", "can_stale": "CAN feedback absent/stale",
                        "imu_eof": "IMU producer EOF", "interrupt": "interrupted",
                        "overheat": "pitch fault or temperature"}[fault]
            if fault == "stop_ignored":
                assert expected in final["detail"] or "CAN feedback absent/stale" in final["detail"], final
            else:
                assert expected in final["detail"], final
            if fault in ("mode_ignored", "zero_ignored", "read_rejected"):
                assert not any(e["type"] == 3 for e in events), "enabled despite failed mode readback"
            if fault in ("stop_ignored", "abort_stop_truncated"):
                assert not final["abort_stop_confirmed"]
            else:
                assert final["abort_stop_confirmed"], "abort did not confirm disabled status"
        result = {"provenance":"SYNTHETIC", "hardware_accessed":False, "fault":fault,
                  "command":command, "executable_sha256":executable_sha256,
                  "returncode":child.returncode, "result":final, "protocol_events":events,
                  "stdout":first+stdout, "stderr":stderr}
        assert hashlib.sha256(binary.read_bytes()).hexdigest()==executable_sha256, "executable changed during rehearsal"
        (output/"result.json").write_text(json.dumps(result, indent=2)+"\n")
        return result
    except Exception as exc:
        if child.poll() is None:
            child.terminate()
        try:
            stdout, stderr = child.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            child.kill(); stdout, stderr = child.communicate(timeout=3)
        (output/"failure.json").write_text(json.dumps({"detail":str(exc), "stdout":first+stdout,
            "stderr":stderr, "returncode":child.returncode, "protocol_events":events}, indent=2)+"\n")
        raise
    finally:
        selector.close(); peer.close()
        if not imu_closed:
            os.close(wr)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fault", choices=(*FAULTS,"neutral_noise"), default="none")
    parser.add_argument("--characterize", action="store_true", help="distinct neutral current measurement acquisition")
    parser.add_argument("--original-mode", type=int, choices=(1,2,3), default=2,
                        help="synthetic initial disabled native mode")
    parser.add_argument("--observation-s",type=float,default=2.,help="characterization observation duration fixture")
    parser.add_argument("--matrix", action="store_true", help="fresh local process attempt for each specified fault")
    args = parser.parse_args()
    if args.matrix:
        args.output.mkdir(parents=True, exist_ok=False)
        checks = []
        faults=(*FAULTS,"neutral_noise") if args.characterize else FAULTS
        for fault in faults:
            result = rehearse(args.binary, args.output/fault, fault=fault,characterize=args.characterize,
                              original_mode=args.original_mode,observation_s=args.observation_s)
            summary = {k:result[k] for k in ("fault", "returncode", "result", "hardware_accessed")}
            checks.append(summary); print(json.dumps(summary), flush=True)
        (args.output/"summary.json").write_text(json.dumps({"status":"LOCAL_CURRENT_PROCESS_PASS",
            "provenance":"SYNTHETIC", "physical_capabilities_qualified":False, "checks":checks}, indent=2)+"\n")
    else:
        result = rehearse(args.binary, args.output, fault=args.fault,characterize=args.characterize,
                          original_mode=args.original_mode,observation_s=args.observation_s)
        print(json.dumps({k:result[k] for k in ("fault", "returncode", "result", "hardware_accessed")}))
