"""Exercise the real pitch sensorless executor through local UDP/IMU peers.

Fixture geometry, gains, current and timing are synthetic. This tool never opens
hardware and never sends CAN; the production C++ entry is the sole sender.
"""
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

FAULTS = ("none", "delayed_reads", "write_echo", "original_zero_gain", "native_position_offset", "continuous_current_guard",
          "sensor_disagreement", "mode_ignored", "gain_ignored", "read_rejected",
          "read_timeout", "overcurrent", "overheat", "over_torque", "stop_ignored",
          "abort_stop_truncated", "transition_motion", "unexpected_disable", "disabled_reenable",
          "can_stale", "can_error", "can_truncated", "imu_eof", "imu_reset", "imu_stale",
          "interrupt", "wrong_uid", "encoder_jump", "no_contact", "wrong_span", "repeatability",
          "midpoint_timeout")
PASS_FAULTS = ("none", "delayed_reads", "write_echo", "original_zero_gain", "native_position_offset", "continuous_current_guard")


def fixture(ports, peer_port, imu_fd, output):
    d = math.pi / 180
    return {
        "schema": "adr0022.sensorless-homing/1", "purpose": "pitch_sensorless_homing",
        "provenance": "SYNTHETIC", "transport": "loopback_udp",
        "pitch_supported_when_disabled": True, "expected_pitch_uid": "7216313130333105",
        "protection_limit_basis": {"kind": "manufacturer_continuous_current_rating",
            "document": "docs/references/cybergear/CyberGear微电机使用说明书.pdf",
            "sha256": "4fe8727a690193953e62438c04abd25f8e8be232e02b4eddf3aa1f99610da495", "continuous_current_A": 6.5},
        "serialization_probe": {"utf8_text": "制造商手册/电机使用说明书.pdf", "pending_value": None},
        "yaw": {"port": ports[0], "peer_port": peer_port},
        "pitch": {"port": ports[1], "peer_port": peer_port}, "imu_fd": imu_fd,
        "output": str(output.resolve()),
        "expected_span": {"operator_reported_deg": 60, "minimum_deg": 55, "maximum_deg": 65},
        "native_settings": {"expected_original_mode": 3, "original_limit_cur_A": 5,
            "original_speed_kp": 1, "original_speed_ki": .002, "original_position_kp": 30,
            "homing_limit_cur_A": 5, "homing_speed_kp": 4, "homing_speed_ki": .05,
            "homing_position_kp": 30, "position_reference_readback_tolerance_rad": .0004},
        "guards": {"current_bound_A": 5, "torque_bound_Nm": 3, "pitch_temperature_C": 60,
            "yaw_displacement_rad": .01, "total_displacement_rad": 70*d,
            "maximum_encoder_speed_rad_s": .8, "mode_transition_displacement_rad": .01,
            "encoder_mechpos_agreement_bound_rad": .0004,
            "midpoint_tolerance_rad": .004, "midpoint_speed_rad_s": 10*d,
            "midpoint_dwell_s": .5, "midpoint_timeout_s": 15},
        "homing": {"motion_checks_abort": True, "coarse_speed_rad_s": 10*d,
            "fine_speed_rad_s": 20*d, "backoff_speed_rad_s": 10*d,
            "backoff_rad": 5*d, "small_backoff_rad": 2*d, "repeatability_rad": .5*d,
            "repeatability_retries": 2, "settle_time_s": .5, "approach_timeout_s": 30,
            "max_travel_rad": 70*d, "arrival_tol_rad": .01, "backoff_timeout_s": 15,
            "backoff_arrival_tol_rad": .25*d, "backoff_arrive_vel_rad_s": .1,
            "rearm_before_start": True, "limit_cur_initial_a": 5, "limit_cur_step_a": 0,
            "limit_cur_max_a": 5, "max_rotation_rad": 70*d, "torque_safety_nm": 3,
            "dir_endpoint_a": 1, "dir_endpoint_b": -1,
            "contact": {"v_stall_threshold_rad_s": .2, "q_stall_threshold_rad": .001,
                "progress_window_s": .2, "effort_contact_threshold_nm": .05,
                "effort_hard_contact_nm": .4, "motion_history_vel_rad_s": .05,
                "effort_hard_abort_nm": 3, "contact_dwell_ms": 200,
                "min_command_active_ms": 100, "jitter_window_ms": 250,
                "v_move_threshold_rad_s": .10, "a_peak_rad_s2": 8}},
        "limits": {"clock_uncertainty_s": .001, "dequeue_age_s": .08,
            "can_gap_s": .15, "imu_gap_s": .12, "startup_s": .3,
            "duration_s": 80, "minimum_imu_status": 0, "read_timeout_s": .2,
            "read_period_s": .01, "stop_period_s": .02}}


def rehearse(binary: Path, output: Path, *, fault="none", runner=()):
    output.mkdir(parents=True, exist_ok=False)
    reservations = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2)]
    for reserved in reservations:
        reserved.bind(("127.0.0.1", 0))
    ports = [reserved.getsockname()[1] for reserved in reservations]
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(("127.0.0.1", 0)); peer.setblocking(False)
    rd, wr = os.pipe()
    config = fixture(ports, peer.getsockname()[1], rd, output / "capture.jsonl")
    if fault in ("continuous_current_guard", "overcurrent", "abort_stop_truncated"):
        config["guards"]["current_bound_A"] = 6.5
    if fault == "original_zero_gain":
        config["native_settings"]["original_speed_ki"] = 0
    if fault == "midpoint_timeout":
        config["guards"]["midpoint_timeout_s"] = 3
    native_offset = .00067 if fault == "native_position_offset" else (.002 if fault == "sensor_disagreement" else 0.)
    if native_offset:
        config["guards"]["encoder_mechpos_agreement_bound_rad"] = .001
    manifest = output / "manifest.json"
    manifest.write_text(json.dumps(config, indent=2) + "\n")
    for reserved in reservations:
        reserved.close()
    command = [*runner, str(binary.resolve()), "--establish-homing", str(manifest.resolve())]
    executable_sha256 = hashlib.sha256(binary.read_bytes()).hexdigest()
    child = subprocess.Popen(command, pass_fds=(rd,), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    os.close(rd)
    selector = selectors.DefaultSelector(); selector.register(child.stdout, selectors.EVENT_READ)
    enabled = False; first = ""; events = []; imu_closed = False; injected = False
    registers = {0x7005: 3., 0x7006: 0., 0x700A: 0., 0x7016: -1., 0x7017: 0.175,
                 0x7018: 5., 0x701E: 30., 0x701F: 1., 0x7020: .002}
    originals = dict(registers)
    if fault == "original_zero_gain":
        registers[0x7020] = originals[0x7020] = 0
    q = -1.; low = q - math.pi / 6; high = q + math.pi / 6; velocity = 0.; torque = 0.
    stop_count = 0; enable_count = 0; sequence = 0; delayed = []; positive_approaches = 0; echo_counts = {}
    if fault == "wrong_span":
        low, high = -1.-math.pi/9, -1.+math.pi/9

    def feedback(address, *, truncated=False):
        if fault == "can_stale" and injected:
            return
        count = max(0, min(65535, round((q + 12.5) * 65535 / 25)))
        effort = max(0, min(65535, round((torque + 12) * 65535 / 24)))
        temp = 700 if fault == "overheat" and injected else 225
        cid = 0x82007F00 | ((2 if enabled else 0) << 22)
        peer.sendto(struct.pack("=IB3x8s", cid, 0 if truncated else 8, struct.pack(">HHHH", count, 32768, effort, temp)), address)

    try:
        if not selector.select(10):
            raise RuntimeError("process readiness deadline")
        first = child.stdout.readline()
        assert first and json.loads(first)["kind"] == "capture_ready", first
        start = time.monotonic(); previous = start; yaw_due = start; imu_due = start
        while child.poll() is None and time.monotonic() - start < 85:
            now = time.monotonic(); dt = now - previous; previous = now
            mode = int(registers[0x7005]); desired_velocity = 0.
            if enabled:
                if mode == 2:
                    desired_velocity = registers[0x700A]
                elif mode == 1:
                    error = registers[0x7016] - (q+native_offset)
                    desired_velocity = math.copysign(min(registers[0x7017], abs(error)/dt), error)
                    if fault == "midpoint_timeout" and abs(error) > .3:
                        desired_velocity = 0
                elif fault != "disabled_reenable":
                    raise AssertionError("C++ enabled unqualified current/motion mode")
            proposed = q + desired_velocity * dt
            actual_high = high-.02 if fault == "repeatability" and positive_approaches >= 3 else high
            q_next = proposed if fault == "no_contact" else max(low, min(actual_high, proposed))
            velocity = (q_next - q) / dt; q = q_next
            if not enabled and enable_count and fault == "transition_motion":
                injected = True; q -= .3*dt
            torque = math.copysign(.6, desired_velocity) if enabled and abs(desired_velocity) > .001 and abs(velocity) < .001 else .0
            if fault == "over_torque" and injected:
                torque = 4
            for due, response, address in delayed[:]:
                if now >= due:
                    peer.sendto(response, address); delayed.remove((due, response, address))
            for _ in range(32):
                try:
                    request, address = peer.recvfrom(1024)
                except BlockingIOError:
                    break
                cid, dlc, data = struct.unpack("=IB3x8s", request); assert dlc == 8
                if not cid & 0x80000000:
                    assert cid == 0x1FE and data == bytes(8), "nonzero yaw homing command"
                    continue
                kind = (cid >> 24) & 31
                assert cid & 255 == 127 and (cid >> 8) & 65535 == 0
                events.append({"type": kind, "data": data.hex(), "time_s": now-start, "q_rad": q, "enabled": enabled})
                if kind == 0:
                    uid = "7216313130333104" if fault == "wrong_uid" else "7216313130333105"
                    peer.sendto(struct.pack("=IB3x8s", 0x80007FFE, 8, bytes.fromhex(uid)), address)
                elif kind == 4:
                    assert data == bytes(8), "STOP cleared faults"
                    stop_count += 1
                    if not (fault == "stop_ignored" and enabled):
                        enabled = False
                    if fault == "disabled_reenable" and stop_count == 2:
                        enabled = True
                    feedback(address, truncated=fault == "abort_stop_truncated" and injected)
                elif kind == 3:
                    assert mode in (1, 2) and registers[0x700A] == 0 and registers[0x7006] == 0, "enable without neutral native mode"
                    assert registers[0x7018] == 5 and registers[0x701F] == 4 and abs(registers[0x7020]-.05) < 1e-7
                    if mode == 1:
                        assert abs(registers[0x7016]-(q+native_offset)) <= .0004, "position enable wasn't pinned to actual native MechPos"
                        assert registers[0x7017] == 0, "position enable allowed a nonzero speed limit"
                        # Actual firmware overwrites the disabled LocRef pin on
                        # enable; the executor must obtain and pin enabled pose.
                        registers[0x7016] = round((q+native_offset)/.00003)*.00003
                    if fault == "delayed_reads":
                        # Replay a trailing STOP acknowledgement while enable
                        # is in flight, before its actual Motor acknowledgement.
                        assert not enabled
                        feedback(address)
                    enabled = True; enable_count += 1; feedback(address)
                elif kind == 18:
                    index = struct.unpack_from("<H", data)[0]
                    assert index in registers, "unrelated or persistent register write"
                    value = float(data[4]) if index == 0x7005 else struct.unpack_from("<f", data, 4)[0]
                    if index in (0x7005, 0x7018, 0x701E, 0x701F, 0x7020):
                        assert not enabled, "settings write while enabled"
                    if index == 0x7006:
                        assert value == 0, "nonzero current command"
                    if index == 0x7016:
                        assert low-.01 <= value <= high+.01 and value != 0, "position hold used zero or left mount corridor"
                        value = round(value/.00003)*.00003
                    if index == 0x700A and value > 0 and registers[index] <= 0:
                        positive_approaches += 1
                    if not (fault == "mode_ignored" and index == 0x7005) and not (fault == "gain_ignored" and index == 0x701F):
                        registers[index] = value
                    if fault == "write_echo":
                        echo_counts[index] = echo_counts.get(index, 0)+1
                        # Alternate latency so valid older writes are echoed
                        # after later writes to the same register, including
                        # changed targets. Echoes never replace type17 reads.
                        delay = .05 if echo_counts[index] % 2 else .001
                        delayed.append((now+delay, struct.pack("=IB3x8s", 0x92007F00, 8, data), address))
                    feedback(address)
                elif kind == 17:
                    index = struct.unpack_from("<H", data)[0]
                    assert index in registers or index in (0x7019, 0x701A), "unexpected register read"
                    if index == 0x7019:
                        value = q+native_offset
                    elif index == 0x701A:
                        value = (7. if fault in ("overcurrent", "abort_stop_truncated") and enabled else
                            5.2 if fault == "continuous_current_guard" and enabled else abs(torque)*2)
                    else:
                        value = registers[index]
                    payload = bytearray(data); struct.pack_into("<f", payload, 4, value)
                    if index == 0x7005:
                        payload[4:] = bytes((int(value), 0, 0, 0))
                    response_id = 0x91017F00 if fault == "read_rejected" and index == 0x701F else 0x91007F00
                    response = struct.pack("=IB3x8s", response_id, 8, payload)
                    if not (fault == "read_timeout" and index == 0x701F):
                        if fault in ("delayed_reads", "transition_motion"):
                            delayed.append((now+.03, response, address))
                        else:
                            peer.sendto(response, address)
                else:
                    raise AssertionError("unrelated C++ command")
            if enabled and not injected and fault in ("imu_eof", "imu_reset", "imu_stale", "interrupt", "overheat",
                    "over_torque", "unexpected_disable", "encoder_jump", "can_stale", "can_error", "can_truncated", "abort_stop_truncated"):
                injected = True
                if fault == "imu_eof":
                    os.close(wr); imu_closed = True
                elif fault == "interrupt":
                    child.send_signal(signal.SIGTERM)
                elif fault == "unexpected_disable":
                    enabled = False
                elif fault == "encoder_jump":
                    q += .1
                elif fault == "can_error":
                    peer.sendto(struct.pack("=IB3x8s", 0x20000001, 8, bytes(8)), ("127.0.0.1", ports[0]))
                elif fault == "can_truncated":
                    feedback(("127.0.0.1", ports[1]), truncated=True)
                elif fault == "imu_reset":
                    os.write(wr, b'{"kind":"trace_reset"}\n')
                if fault == "unexpected_disable":
                    feedback(("127.0.0.1", ports[1]))
            if now >= yaw_due:
                for _ in range(min(16, 1+int((now-yaw_due)/.001))):
                    yaw_due += .001
                    peer.sendto(struct.pack("=IB3x8s", 0x205, 8, struct.pack(">HhhBB", 5773, 0, 0, 28, 0)), ("127.0.0.1", ports[0]))
            if now >= imu_due and not imu_closed and not (fault == "imu_stale" and injected):
                imu_due += .02; sequence += 1; stamp = time.monotonic_ns()
                for sensor, values in (("gyro", [0,0,0]), ("accel", [0,0,9.81]), ("rv", [0,0,0,1]), ("game_rv", [0,0,0,1])):
                    row = {"kind":"sample", "sensor":sensor, "sample_ns":stamp-100000,
                           "rx_ns":stamp, "sh2_us":stamp//1000, "sequence":sequence & 255,
                           "generation":0, "status":3, "values":values}
                    try:
                        os.write(wr, (json.dumps(row)+"\n").encode())
                    except BrokenPipeError:
                        break
            time.sleep(.0005)
        stdout, stderr = child.communicate(timeout=3)
        assert hashlib.sha256(binary.read_bytes()).hexdigest() == executable_sha256, "executable changed during process probe"
        records = [json.loads(line) for line in (output/"capture.jsonl").read_text().splitlines()]
        recorded_manifest = json.loads(records[0]["manifest_yaml"])
        assert recorded_manifest["serialization_probe"]["utf8_text"] == config["serialization_probe"]["utf8_text"], "UTF-8 manifest field changed in journal"
        assert recorded_manifest["serialization_probe"]["pending_value"] == "null", "pending manifest value lost its canonical binding"
        final = records[-1]
        if fault in PASS_FAULTS:
            assert child.returncode == 0 and final["status"] == "COMPLETE", final
            assert not enabled and final["normal_stop_confirmed"]
            for index in (0x7005, 0x7018, 0x701E, 0x701F, 0x7020):
                assert abs(registers[index]-originals[index]) < 1e-6, "original setting not restored"
            assert 59.9 < final["measured_travel_deg"] < 60.1 and abs(final["midpoint_rad"]+1) < .001
            assert abs(q+1) < config["guards"]["midpoint_tolerance_rad"]
            assert any(row["kind"] == "homing_midpoint_dwell" for row in records)
            assert enable_count >= 7, "one-shot mode/rearm sequence didn't traverse both endpoints"
            native_reads = [row for row in records if row["kind"] == "register_read" and row["index"] == 0x7019]
            for row in records:
                if row.get("operation") in ("transition_pin_measured_pose", "enabled_repin_measured_pose", "hold_measured_pose"):
                    preceding = [read for read in native_reads if read["receive_ns"] < row["begin_ns"]]
                    assert preceding, "pin had no fresh native MechPos read"
                    fresh = preceding[-1]; value = struct.unpack_from("<f", bytes.fromhex(row["data_hex"]), 4)[0]
                    assert row["begin_ns"]-fresh["receive_ns"] <= config["limits"]["read_timeout_s"]*1e9
                    assert abs(value-fresh["value"]) <= config["native_settings"]["position_reference_readback_tolerance_rad"], "pin substituted the biased type2 pose"
            if fault == "native_position_offset":
                residuals = [row["register_minus_type2_rad"] for row in records if row["kind"] == "homing_position_observation"]
                assert residuals and max(abs(value) for value in residuals) > .0004
                assert max(abs(value) for value in residuals) <= config["guards"]["encoder_mechpos_agreement_bound_rad"]
            if fault == "continuous_current_guard":
                currents = [row["value"] for row in records if row["kind"] == "register_read" and row["index"] == 0x701A]
                assert currents and max(currents) > 5 and max(currents) <= 6.5, "probe did not exercise measured current above command cap within continuous protection"
                caps = [struct.unpack_from("<f", bytes.fromhex(row["data_hex"]), 4)[0] for row in records
                    if row["kind"] == "homing_tx" and row["axis"] == "pitch" and (row["id"] >> 24) & 31 == 18 and
                    int.from_bytes(bytes.fromhex(row["data_hex"])[:2], "little") == 0x7018]
                assert caps and all(value == 5 for value in caps), "current protection change widened homing command cap"
            if fault == "write_echo":
                prior_writes = {}; replaced_echoes = 0
                for row in records:
                    if row["kind"] == "homing_tx" and row["axis"] == "pitch" and (row["id"] >> 24) & 31 == 18:
                        index = int.from_bytes(bytes.fromhex(row["data_hex"])[:2], "little")
                        prior_writes.setdefault(index, []).append(row)
                    elif row["kind"] == "can_rx" and row["axis"] == "pitch" and (row["id"] >> 24) & 31 == 18:
                        payload = bytes(row["bytes"]); index = int.from_bytes(payload[:2], "little")
                        writes = prior_writes.get(index, [])
                        assert any(write["data_hex"] == payload.hex() and
                            0 <= row["kernel_monotonic_ns"]-write["begin_ns"] < config["limits"]["read_timeout_s"]*1e9
                            for write in writes), "echo did not match an actually accepted write in the bounded receipt window"
                        replaced_echoes += bool(writes and writes[-1]["data_hex"] != payload.hex())
                assert replaced_echoes, "fixture never echoed an older value after a later same-register write"
                assert all(row["readback_verified"] is False for row in records if row["kind"] == "write_echo")
        else:
            assert child.returncode != 0 and final["status"] == "INVALID", final
            assert not final["homing_observed"]
            expected = {"mode_ignored":"readback differs", "gain_ignored":"readback differs",
                "sensor_disagreement":"measured encoder agreement bound",
                "read_rejected":"register rejected", "read_timeout":"register read timeout",
                "overcurrent":"current exceeds guard", "abort_stop_truncated":"current exceeds guard",
                "overheat":"fault/temperature/torque guard", "over_torque":"fault/temperature/torque guard",
                "stop_ignored":"STOP confirmation timeout", "transition_motion":"mode transition displacement",
                "unexpected_disable":"enabled homing mode was lost", "disabled_reenable":"re-enabled during disabled transition",
                "can_stale":"CAN feedback absent/stale", "can_error":"CAN error frame", "can_truncated":"invalid pitch frame",
                "imu_eof":"IMU producer EOF", "imu_reset":"IMU stream ended", "imu_stale":"IMU stream stale",
                "interrupt":"interrupted", "wrong_uid":"UID/discovery correlation", "encoder_jump":"encoder speed guard",
                "no_contact":"total displacement guard", "wrong_span":"measured contacts outside approximate operator span", "repeatability":"repeatability exceeded",
                "midpoint_timeout":"measured midpoint timeout"}[fault]
            assert expected in final["detail"], final
            assert final["abort_stop_confirmed"] == (fault not in ("stop_ignored", "wrong_uid", "can_stale", "abort_stop_truncated")), final
            abort = next((i for i, row in enumerate(records) if row.get("operation") == "abort_stop"), None)
            if abort is not None:
                assert not any(row.get("operation") == "enable_homing_neutral" for row in records[abort:]), "automatic retry after abort"
        result = {"provenance":"SYNTHETIC", "hardware_accessed":False, "fault":fault,
                  "command":command, "executable_sha256":executable_sha256,
                  "returncode":child.returncode, "result":final, "protocol_events":events,
                  "stdout":first+stdout, "stderr":stderr}
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
            "executable_sha256_before":executable_sha256, "executable_sha256_after":hashlib.sha256(binary.read_bytes()).hexdigest(),
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
    parser.add_argument("--fault", choices=FAULTS, default="none")
    parser.add_argument("--matrix", action="store_true")
    args = parser.parse_args()
    if args.matrix:
        args.output.mkdir(parents=True, exist_ok=False); checks = []
        for fault in FAULTS:
            result = rehearse(args.binary, args.output/fault, fault=fault)
            summary = {key:result[key] for key in ("fault", "returncode", "result", "hardware_accessed")}
            checks.append(summary); print(json.dumps(summary), flush=True)
        (args.output/"summary.json").write_text(json.dumps({"status":"LOCAL_HOMING_PROCESS_PASS",
            "provenance":"SYNTHETIC", "physical_capabilities_qualified":False, "checks":checks}, indent=2)+"\n")
    else:
        result = rehearse(args.binary, args.output, fault=args.fault)
        print(json.dumps({key:result[key] for key in ("fault", "returncode", "result", "hardware_accessed")}))
