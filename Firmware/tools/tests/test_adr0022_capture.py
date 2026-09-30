"""Local Linux boundary and corruption tests. No station access or motor I/O."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import struct
import subprocess
import sys
import textwrap
import time
import pytest
pytest.importorskip("fcntl", reason="Linux capture supervision")

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
from adr0022_capture_rehearsal import rehearse
from adr0022_capture_review import review
from adr0022_capture_launch import (preflight, source_identity, SOURCE_FILES, SOURCE_DIRECTORIES,
    validate_current_contract, canonical_sha, PROTECTION_DOCUMENT, PROTECTION_DOCUMENT_SHA256)
from adr0022_baseline_assets import derive, write_asset

pytestmark = pytest.mark.skipif(sys.platform != "linux", reason="real Linux recvmsg/process boundary")
ROOT = TOOLS.parents[1]
BINARY = Path(os.environ.get("OTA_STAGE1_BUILD", ROOT / "run/adr0022-local/firmware")) / "axis_control_core/commissiond"


@pytest.fixture(scope="module")
def successful(tmp_path_factory):
    directory = tmp_path_factory.mktemp("capture") / "success"
    result = rehearse(BINARY, directory, duration=6.)
    return directory, result


def test_full_process_capture_and_independent_review(successful):
    directory, result = successful
    report = review(directory / "capture.jsonl")
    assert result["returncode"] == 0 and report["capture_complete"]
    assert report["provenance"] == "SYNTHETIC"
    assert not report["motion_authorized"] and not report["physical_parameters_qualified"]
    assert report["streams"]["gyro"]["count"] > 256  # SH-2 source sequence wrapped
    assert report["streams"]["gyro"]["observed_hz"] < 60  # never sum four sensors as gyro Hz
    assert report["register_values_first_observed"][str(0x7005)] == 1
    assert report["temperatures"]["pitch"]["max_C"] == 34.5
    assert report["current_units"]["yaw"]["scale_A_per_count"] is None
    assert report["current_units"]["yaw"]["legacy_derived_ampere_fields_ignored"] == 0
    assert report["streams"]["gyro"]["status_counts"]["3"] == report["streams"]["gyro"]["count"]


def test_saved_legacy_ampere_values_cannot_become_calibrated_current(tmp_path, successful):
    rows = [json.loads(line) for line in (successful[0] / "capture.jsonl").read_text().splitlines()]
    yaw = [r for r in rows if r["kind"] == "can_rx" and r["axis"] == "yaw"]
    for row in yaw:
        row["current_A"] = row["current_raw"] * 3 / 16384
    path = tmp_path / "legacy.jsonl"
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    report = review(path)
    assert report["capture_complete"]  # original raw evidence remains usable
    assert report["current_units"]["yaw"]["legacy_derived_ampere_fields_ignored"] == len(yaw)
    assert report["current_units"]["yaw"]["scale_A_per_count"] is None
    assert not report["physical_parameters_qualified"]


def test_capability_assets_bind_evidence_without_fabricating_a_plant(tmp_path, successful):
    rows = [json.loads(line) for line in (successful[0] / "capture.jsonl").read_text().splitlines()]
    bound = json.loads((successful[0] / "manifest.json").read_text())
    bound["expected_binaries"] = {"commissiond": hashlib.sha256(BINARY.read_bytes()).hexdigest(), "imu": "a" * 64}
    original = {k: v for k, v in bound.items() if k != "imu_fd"}
    def scalar_strings(value):
        if isinstance(value, dict): return {k: scalar_strings(v) for k, v in value.items()}
        if type(value) is bool: return str(value).lower()
        return str(value)
    rows[0]["manifest_yaml"] = json.dumps(scalar_strings(bound))
    capture = tmp_path / "capture.jsonl"
    capture.write_text("".join(json.dumps(r) + "\n" for r in rows))
    manifest, bound_path, attempt = (tmp_path / name for name in ("manifest.json", "bound.json", "attempt.json"))
    manifest.write_text(json.dumps(original))
    bound_path.write_text(json.dumps(bound))
    receipt = {"schema": "adr0022.capture_attempt/1", "provenance": "SYNTHETIC", "motion_requested": False,
               "binaries": bound["expected_binaries"], "manifest_sha256": hashlib.sha256(manifest.read_bytes()).hexdigest()}
    attempt.write_text(json.dumps(receipt))
    asset = derive(capture, attempt, manifest, bound_path)
    assert asset["capture_integrity"] == "PASS" and asset["provenance"] == "SYNTHETIC"
    assert asset["plant_snapshot"] is None and asset["controller_candidate"] is None
    assert not asset["physical_parameters_qualified"] and all(v is None for v in asset["pending_parameters"].values())
    path = write_asset(tmp_path / "assets", asset)
    assert path == write_asset(tmp_path / "assets", asset)
    receipt["binaries"]["commissiond"] = "b" * 64
    attempt.write_text(json.dumps(receipt))
    with pytest.raises(ValueError, match="binding differs"):
        derive(capture, attempt, manifest, bound_path)
    assert len(list((tmp_path / "assets").glob("*.json"))) == 1


def test_rejected_capability_preserves_other_measurements_without_inventing_values(tmp_path):
    directory = tmp_path / "limited"
    result = rehearse(BINARY, directory, fault="read_rejected")
    report = review(directory / "capture.jsonl")
    assert result["returncode"] == 0 and report["capture_complete"]
    assert {r["index"] for r in report["measurement_limitations"]} == {0x7019, 0x701A}
    assert all(r["reason"] == "MEASUREMENT_LIMITED" for r in report["measurement_limitations"])
    assert "pitch_iqf" not in report["streams"]
    assert str(0x7019) not in report["register_values_first_observed"]
    assert str(0x701A) not in report["register_values_first_observed"]
    assert not report["physical_parameters_qualified"] and not report["motion_authorized"]
    rows = [json.loads(line) for line in (directory / "capture.jsonl").read_text().splitlines()]
    for index in (0x7019, 0x701A):
        assert sum(r["kind"] == "register_request" and r["index"] == index for r in rows) == 1
    next(r for r in rows if r["kind"] == "register_rejected")["value"] = .25
    altered = tmp_path / "invented-value.jsonl"
    altered.write_text("".join(json.dumps(r) + "\n" for r in rows))
    with pytest.raises(ValueError, match="fabricated value"):
        review(altered)


@pytest.mark.parametrize("fault", ["can_stale", "can_error", "can_truncated", "imu_sequence", "imu_reset",
                                   "imu_eof", "read_echo", "read_source", "read_timeout", "wrong_uid", "reenabled", "stop_timeout"])
def test_full_process_rejects_injected_fault(tmp_path, fault):
    result = rehearse(BINARY, tmp_path / fault, fault=fault)
    assert result["returncode"] != 0
    with pytest.raises(ValueError):
        review(tmp_path / fault / "capture.jsonl")


@pytest.mark.parametrize("corruption", ["truncate", "remove_can", "remove_imu", "duplicate", "temperature", "register", "final_loss", "missing_loss"])
def test_review_rejects_corrupt_evidence(tmp_path, successful, corruption):
    source = (successful[0] / "capture.jsonl").read_text()
    rows = [json.loads(r) for r in source.splitlines()]
    if corruption == "truncate":
        rows.pop()
    elif corruption in ("remove_can", "remove_imu"):
        kind = "can_rx" if corruption == "remove_can" else "imu_raw"
        at = [i for i, r in enumerate(rows) if r["kind"] == kind][20]
        del rows[at]
    elif corruption == "duplicate":
        rows.insert(10, rows[10])
    elif corruption == "temperature":
        next(r for r in rows if r["kind"] == "can_rx" and r["axis"] == "yaw")["temperature_C"] = 37.
    elif corruption == "final_loss":
        rows[-1]["socket_drops"]["yaw"] = 1
    elif corruption == "missing_loss":
        del rows[-1]["socket_drops"]
    else:
        next(r for r in rows if r["kind"] == "register_read")["source"] = "type18_echo"
    path = tmp_path / "altered.jsonl"
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    with pytest.raises(ValueError):
        review(path)


def prepare_launcher_fixture(tmp_path):
    firmware = tmp_path / "Firmware"
    for part in ("tools", "scripts", "config", "build/axis_control_core"):
        (firmware / part).mkdir(parents=True)
    for name in ("adr0022_capture_launch.py", "adr0022_capture_review.py"):
        shutil.copy(TOOLS / name, firmware / "tools" / name)
    shutil.copy(TOOLS.parent / "scripts/run_application.sh", firmware / "scripts/run_application.sh")
    (firmware / "config/turret.yaml").write_text("{}\n")
    (firmware / "build/axis_control_core/commissiond").symlink_to(BINARY.resolve())
    imu = firmware / "build/imu-bno085"
    imu.write_text(f"#!{sys.executable}\n" + textwrap.dedent('''\
        import json, os, signal, sys, time
        assert sys.argv[1:] == ['--commissioning']
        stopping = False
        def stop(*_):
            global stopping
            stopping = True
        signal.signal(signal.SIGTERM, stop)
        seq = 0
        while not stopping:
            seq = (seq + 1) & 255
            stamp = time.monotonic_ns()
            for name, values in [('accel',[0,0,9.81]),('gyro',[0,0,0]),('rv',[0,0,0,1]),('game_rv',[0,0,0,1])]:
                print(json.dumps(dict(kind='sample', sensor=name, values=values, generation=0,
                      sequence=seq, status=3, sample_ns=stamp-10000, rx_ns=stamp)), flush=True)
            time.sleep(.02)
        '''))
    imu.chmod(0o755)
    emitter = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    emitter.bind(("127.0.0.1", 0))
    emitter.setblocking(False)
    reservations = []
    for _ in range(2):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        reservations.append(sock)
    ports = [s.getsockname()[1] for s in reservations]
    manifest = {"schema": "adr0022.capture/2", "provenance": "SYNTHETIC", "transport": "loopback_udp",
                "output": str(tmp_path / "baseline.jsonl"), "register_reads": False,
                "yaw": {"port": ports[0]}, "pitch": {"port": ports[1], "peer_port": emitter.getsockname()[1]},
                "pitch_stop_poll": True, "pitch_supported_when_disabled": True, "expected_pitch_uid": "7216313130333105",
                "limits": {"clock_uncertainty_s": .001, "dequeue_age_s": .1, "can_gap_s": .1,
                           "imu_gap_s": .12, "startup_s": .5, "duration_s": 2., "minimum_imu_status": 0,
                           "read_timeout_s": .15, "read_period_s": .01, "stop_period_s": .02},
                "expected_binaries": {"commissiond": hashlib.sha256(BINARY.read_bytes()).hexdigest(),
                                      "imu": hashlib.sha256(imu.read_bytes()).hexdigest()}}
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(manifest))
    for sock in reservations:
        sock.close()
    return firmware, path, ports, emitter


def test_real_launcher_single_attempt_supervision(tmp_path):
    firmware, manifest, ports, emitter = prepare_launcher_fixture(tmp_path)
    env = os.environ.copy()
    env.update(OTA_PYTHON=sys.executable, OTA_RUN_DIR=str(tmp_path / "runtime"))
    launcher = firmware / "scripts/run_application.sh"
    child = subprocess.Popen(["bash", str(launcher), "run", "--capture-baseline", str(manifest)],
                             env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        start = time.monotonic()
        while child.poll() is None and time.monotonic() - start < 10:
            try:
                request, address = emitter.recvfrom(1024)
            except BlockingIOError:
                pass
            else:
                can_id, _, data = struct.unpack("=IB3x8s", request)
                kind = (can_id >> 24) & 31
                assert kind in (0, 4) and data == bytes(8)
                if kind == 0:
                    response_id, data = 0x80000000 | (127 << 8) | 0xFE, bytes.fromhex("7216313130333105")
                else:
                    response_id, data = 0x80000000 | (2 << 24) | (127 << 8), struct.pack(">HHHH", 32768, 32768, 32768, 345)
                emitter.sendto(struct.pack("=IB3x8s", response_id, 8, data), address)
            for axis, port in enumerate(ports[:1]):
                can_id = 0x205 if axis == 0 else 0x80000000 | (2 << 24) | (127 << 8)
                data = struct.pack(">HhhBB", 100, 0, 0, 37, 0) if axis == 0 else struct.pack(">HHHH", 32768, 32768, 32768, 345)
                emitter.sendto(struct.pack("=IB3x8s", can_id, 8, data), ("127.0.0.1", port))
            time.sleep(.01)
        stdout, stderr = child.communicate(timeout=10)
        log = (tmp_path / "runtime/controller.log").read_text() if (tmp_path / "runtime/controller.log").exists() else ""
        assert child.returncode == 0, stdout + stderr + log
        result = json.loads((tmp_path / "baseline.result.json").read_text())
        assert result["status"] == "COMPLETE" and not result["motion_authorized"]
        assert not (tmp_path / "runtime/launcher.pid").exists()
        # No second sensor start when the same evidence identity is requested.
        second = subprocess.run(["bash", str(launcher), "run", "--capture-baseline", str(manifest)],
                                env=env, capture_output=True, text=True, timeout=10)
        assert second.returncode != 0 and "evidence reuse" in second.stderr
    finally:
        emitter.close()
        if child.poll() is None:
            child.terminate()
            child.wait(timeout=15)


def test_preflight_refuses_changed_binary_before_sensor_start(tmp_path):
    firmware, manifest, _, emitter = prepare_launcher_fixture(tmp_path)
    emitter.close()
    config = json.loads(manifest.read_text())
    config["expected_binaries"]["imu"] = "0" * 64
    manifest.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="changed imu"):
        preflight(manifest, firmware)
    assert not (tmp_path / "baseline.attempt.json").exists()


def test_rehearsal_preserves_process_failure_before_ready(tmp_path):
    binary = tmp_path / "refuse"
    binary.write_text("#!/bin/sh\necho 'loader fixture rejected' >&2\nexit 7\n")
    binary.chmod(0o755)
    output = tmp_path / "failed"
    with pytest.raises(RuntimeError, match="before readiness"):
        rehearse(binary, output)
    failure = json.loads((output / "failure.json").read_text())
    assert failure["returncode"] == 7 and failure["stderr"] == "loader fixture rejected\n"
    assert failure["executable_sha256"] == hashlib.sha256(binary.read_bytes()).hexdigest()
    assert not failure["hardware_accessed"] and not (output / "result.json").exists()


def test_preflight_rejects_timing_that_rounds_to_zero_before_sensor_start(tmp_path):
    firmware, manifest, _, emitter = prepare_launcher_fixture(tmp_path)
    emitter.close()
    config = json.loads(manifest.read_text())
    config["limits"]["read_period_s"] = 1e-12
    manifest.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="at least one nanosecond"):
        preflight(manifest, firmware)
    assert not (tmp_path / "baseline.attempt.json").exists()


def neutral_launcher_probe(tmp_path, cancel=False, lose_collector=False, freeze_collector=False, characterize_current=False,
                           protection_violation=False):
    """Actual shell, Python supervisor, C++ owner, UDP peers and IMU pipe."""
    firmware, manifest, ports, emitter = prepare_launcher_fixture(tmp_path)
    current_review = TOOLS / "adr0022_current_review.py"
    if current_review.exists():
        shutil.copy(current_review, firmware / "tools")
    for name in SOURCE_FILES:
        destination = firmware / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(TOOLS.parent / name, destination)
    for name in SOURCE_DIRECTORIES:
        shutil.copytree(TOOLS.parent / name, firmware / name)
    config = json.loads(manifest.read_text())
    config.update(schema="adr0022.current-preparation/1", purpose="neutral_current_mode_verification",
                  neutral_current_bound_A=.1, transition_displacement_bound_rad=.01,
                  pitch_maximum_temperature_C=60., expected_source_sha256=source_identity(firmware)["source_sha256"])
    if characterize_current:
        config.update(schema="adr0022.neutral-characterization/1", purpose="neutral_current_measurement_characterization",
                      protection_current_bound_A=6.5, neutral_observation_s=2.,
                      pending_calibration=None,
                      protection_limit_basis={"kind": "manufacturer_continuous_current_rating", "document": PROTECTION_DOCUMENT,
                                              "sha256": PROTECTION_DOCUMENT_SHA256, "continuous_current_A": 6.5})
    config["yaw"]["peer_port"] = emitter.getsockname()[1]
    config["limits"]["duration_s"] = 10.
    manifest.write_text(json.dumps(config))
    env = dict(os.environ, OTA_PYTHON=sys.executable, OTA_RUN_DIR=str(tmp_path / "runtime"))
    launcher = firmware / "scripts/run_application.sh"
    option = "--characterize-current" if characterize_current else "--prepare-current"
    child = subprocess.Popen(["bash", str(launcher), "run", option, str(manifest)],
                             env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    enabled, mode, iq, cancelled = False, 2, 1., False
    started = time.monotonic()
    try:
        while child.poll() is None and time.monotonic() - started < 15:
            for _ in range(64):
                try:
                    request, address = emitter.recvfrom(1024)
                except BlockingIOError:
                    break
                cid, dlc, data = struct.unpack("=IB3x8s", request)
                assert dlc == 8
                if not cid & 0x80000000:
                    assert cid == 0x1fe and data == bytes(8)
                    continue
                kind = (cid >> 24) & 31
                assert cid & 255 == 127 and (cid >> 8) & 65535 == 0
                if kind == 0:
                    response_id, payload = 0x80007ffe, bytes.fromhex("7216313130333105")
                elif kind == 17:
                    reg = struct.unpack_from("<H", data)[0]
                    assert reg in (0x7005, 0x7006, 0x701a)
                    observed_current = 7. if protection_violation else .3 if characterize_current else 0.
                    value = struct.pack("<B3x", mode) if reg == 0x7005 else struct.pack("<f", iq if reg == 0x7006 else observed_current)
                    response_id, payload = 0x91007f00, struct.pack("<H2x", reg) + value
                else:
                    assert kind in (3, 4, 18)
                    if kind == 3:
                        assert mode == 3 and iq == 0
                        enabled = True
                    elif kind == 4:
                        assert data == bytes(8)
                        enabled = False
                    else:
                        reg = struct.unpack_from("<H", data)[0]
                        assert reg in (0x7005, 0x7006)
                        if reg == 0x7005:
                            assert not enabled
                            mode = data[4]
                        else:
                            iq = struct.unpack_from("<f", data, 4)[0]
                            assert iq == 0
                    response_id = 0x82007f00 | ((2 if enabled else 0) << 22)
                    payload = struct.pack(">HHHH", 32000, 32768, 32768, 225)
                emitter.sendto(struct.pack("=IB3x8s", response_id, 8, payload), address)
            emitter.sendto(struct.pack("=IB3x8s", 0x205, 8, struct.pack(">HhhBB", 100, 0, 0, 37, 0)),
                           ("127.0.0.1", ports[0]))
            if cancel and enabled and not cancelled:
                if lose_collector or freeze_collector:
                    processes = json.loads((tmp_path / "baseline.processes.json").read_text())
                    os.kill(processes["collector_pid"], signal.SIGSTOP if freeze_collector else signal.SIGKILL)
                child.terminate()
                cancelled = True
            time.sleep(.001)
        stdout, stderr = child.communicate(timeout=15)
        log = (tmp_path / "runtime/controller.log").read_text()
        result = json.loads((tmp_path / "baseline.result.json").read_text())
        if lose_collector or freeze_collector:
            assert enabled  # Loss cannot be repaired or called safe by Python CAN output.
            assert result["status"] == "INVALID" and not result["pitch_stop_confirmed"]
            assert result["process_loss"] and "STOP unconfirmed" in result["stop_detail"]
            assert result["forced_termination"] == freeze_collector
        elif protection_violation:
            assert child.returncode != 0 and result["status"] == "INVALID"
            assert not enabled and result["pitch_stop_confirmed"]
            footer = json.loads((tmp_path / "baseline.jsonl").read_text().splitlines()[-1])
            assert "manufacturer current protection bound exceeded" in footer["detail"]
            assert footer["abort_stop_confirmed"]
        else:
            assert not enabled, stdout + stderr + log
            assert result["pitch_stop_confirmed"], result
        assert not result["yaw_stop_confirmed"] and not result["independent_cutoff_qualified"]
        assert not result["physical_parameters_qualified"]
        assert not (tmp_path / "runtime/launcher.pid").exists()
        if cancel:
            assert cancelled and child.returncode != 0 and result["status"] == "INVALID"
            if not (lose_collector or freeze_collector):
                footer = json.loads((tmp_path / "baseline.jsonl").read_text().splitlines()[-1])
                assert footer["abort_stop_confirmed"]
        elif not protection_violation:
            assert child.returncode == 0 and result["status"] == "COMPLETE", stdout + stderr + log
            assert mode == 2 and iq == 0
        if characterize_current:
            assert result["neutral_current_qualified"] is False and result["current_mode_qualified"] is False
            header = json.loads((tmp_path / "baseline.jsonl").read_text().splitlines()[0])
            recorded_manifest = json.loads(header["manifest_yaml"])
            assert recorded_manifest["protection_limit_basis"]["document"] == config["protection_limit_basis"]["document"]
            assert recorded_manifest["pending_calibration"] == "null"
            if result["status"] == "COMPLETE":
                assert result["neutral_current_criterion_satisfied"] is False
            attempt = json.loads((tmp_path / "baseline.attempt.json").read_text())
            assert attempt["schema"] == "adr0022.neutral_characterization_attempt/1"
            assert attempt["automatic_retries"] == 0 and attempt["nonzero_current_requested"] is False
            assert attempt["historical_diagnostic_bound_A"] == config["neutral_current_bound_A"]
        again = subprocess.run(["bash", str(launcher), "run", option, str(manifest)],
                               env=env, text=True, capture_output=True, timeout=10)
        assert again.returncode != 0 and "evidence reuse" in again.stderr
    finally:
        emitter.close()
        if child.poll() is None:
            child.terminate()
            child.wait(timeout=15)


def test_neutral_launcher_cancellation_allows_cpp_stop(tmp_path):
    neutral_launcher_probe(tmp_path, cancel=True)


def test_neutral_launcher_complete_and_immutable_attempt(tmp_path):
    neutral_launcher_probe(tmp_path)


def test_neutral_launcher_lost_owner_preserves_unconfirmed_stop(tmp_path):
    neutral_launcher_probe(tmp_path, cancel=True, lose_collector=True)


def test_neutral_launcher_forced_termination_preserves_unconfirmed_stop(tmp_path):
    neutral_launcher_probe(tmp_path, cancel=True, freeze_collector=True)


def test_characterization_launcher_records_above_quality_without_qualification(tmp_path):
    neutral_launcher_probe(tmp_path, characterize_current=True)


def test_characterization_launcher_keeps_manufacturer_protection_abort(tmp_path):
    neutral_launcher_probe(tmp_path, characterize_current=True, protection_violation=True)


def homing_launcher_probe(tmp_path, monkeypatch, fault="none"):
    """Reuse the existing protocol plant, with the actual launcher as process owner."""
    from adr0022_homing_rehearsal import rehearse as homing_rehearse, PASS_FAULTS
    firmware, _, _, emitter = prepare_launcher_fixture(tmp_path)
    emitter.close()
    for name in SOURCE_FILES:
        destination = firmware / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(TOOLS.parent / name, destination)
    for name in SOURCE_DIRECTORIES:
        shutil.copytree(TOOLS.parent / name, firmware / name)
    wrapper = tmp_path / "homing-launcher-wrapper"
    wrapper.write_text(f"#!{sys.executable}\n" + textwrap.dedent('''\
        import hashlib, json, os, signal, subprocess, sys, time
        from pathlib import Path
        firmware = Path(os.environ['HOMING_LAUNCHER_FIRMWARE'])
        sys.path.insert(0, str(firmware / 'tools'))
        from adr0022_capture_launch import source_identity
        manifest = Path(sys.argv[-1])
        config = json.loads(manifest.read_text())
        os.close(config.pop('imu_fd'))
        config['expected_source_sha256'] = source_identity(firmware)['source_sha256']
        config['expected_binaries'] = {
            'commissiond': hashlib.sha256((firmware / 'build/axis_control_core/commissiond').read_bytes()).hexdigest(),
            'imu': hashlib.sha256((firmware / 'build/imu-bno085').read_bytes()).hexdigest()}
        manifest.write_text(json.dumps(config))
        env = dict(os.environ, OTA_PYTHON=sys.executable, OTA_RUN_DIR=str(firmware.parent / 'runtime'))
        child = subprocess.Popen(['bash', str(firmware / 'scripts/run_application.sh'), 'run',
                                  '--establish-homing', str(manifest)], env=env,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        def cancel(*_):
            if child.poll() is None: child.terminate()
        signal.signal(signal.SIGTERM, cancel)
        signal.signal(signal.SIGINT, cancel)
        log = firmware.parent / 'runtime/controller.log'
        deadline = time.monotonic() + 10
        while child.poll() is None and time.monotonic() < deadline:
            if log.is_file():
                ready = next((line for line in log.read_text().splitlines()
                              if line.startswith('{"kind":"capture_ready"')), None)
                if ready:
                    print(ready, flush=True)
                    break
            time.sleep(.01)
        stdout, stderr = child.communicate(timeout=120)
        print(stdout, end='')
        print(stderr, file=sys.stderr, end='')
        if log.is_file(): print(log.read_text(), file=sys.stderr)
        raise SystemExit(child.returncode)
        '''))
    wrapper.chmod(0o755)
    monkeypatch.setenv("HOMING_LAUNCHER_FIRMWARE", str(firmware))
    directory = tmp_path / "homing"
    raw = homing_rehearse(BINARY, directory, fault=fault, runner=(str(wrapper),))
    result = json.loads((directory / "capture.result.json").read_text())
    attempt = json.loads((directory / "capture.attempt.json").read_text())
    succeeded = fault in PASS_FAULTS
    assert result["status"] == ("COMPLETE" if succeeded else "INVALID"), result
    assert result["pitch_stop_confirmed"] and not result["yaw_stop_confirmed"]
    assert not result["current_mode_qualified"] and not result["physical_parameters_qualified"]
    assert result["retained_calibration_modified"] is False
    assert attempt["schema"] == "adr0022.sensorless_homing_attempt/1"
    assert attempt["motion_requested"] is True and attempt["automatic_retries"] == 0
    assert result["homing_observed"] == succeeded
    assert not (tmp_path / "runtime/launcher.pid").exists()
    assert raw["hardware_accessed"] is False
    again = subprocess.run(["bash", str(firmware / "scripts/run_application.sh"), "run", "--establish-homing",
                            str(directory / "manifest.json")],
                           env=dict(os.environ, OTA_PYTHON=sys.executable, OTA_RUN_DIR=str(tmp_path / "runtime")),
                           text=True, capture_output=True, timeout=10)
    assert again.returncode != 0 and "evidence reuse" in again.stderr


def test_homing_launcher_complete(tmp_path, monkeypatch):
    homing_launcher_probe(tmp_path, monkeypatch)


def test_homing_launcher_cancellation_allows_cpp_stop(tmp_path, monkeypatch):
    homing_launcher_probe(tmp_path, monkeypatch, fault="interrupt")


def test_homing_launcher_measured_native_offset_preserves_observations(tmp_path, monkeypatch):
    homing_launcher_probe(tmp_path, monkeypatch, fault="native_position_offset")


@pytest.mark.parametrize("invalid", ["missing_parameter", "unsafe_current"])
def test_homing_preflight_uses_exact_runtime_numeric_validation_before_sensor(tmp_path, invalid):
    from adr0022_homing_rehearsal import fixture
    firmware, baseline, ports, emitter = prepare_launcher_fixture(tmp_path)
    emitter.close()
    for name in SOURCE_FILES:
        destination = firmware / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(TOOLS.parent / name, destination)
    for name in SOURCE_DIRECTORIES:
        shutil.copytree(TOOLS.parent / name, firmware / name)
    binaries = json.loads(baseline.read_text())["expected_binaries"]
    config = fixture(ports, 31003, -1, tmp_path / "homing.jsonl")
    config.pop("imu_fd")
    config.update(expected_binaries=binaries, expected_source_sha256=source_identity(firmware)["source_sha256"])
    if invalid == "missing_parameter":
        del config["homing"]["arrival_tol_rad"]
    else:
        config["guards"]["current_bound_A"] = 6.
    manifest = tmp_path / "homing.json"
    manifest.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="runtime homing parameter validation failed"):
        preflight(manifest, firmware, establish_homing=True)
    assert not (tmp_path / "homing.jsonl").exists()
    assert not (tmp_path / "homing.attempt.json").exists()
    assert not (tmp_path / "homing.imu.log").exists()
