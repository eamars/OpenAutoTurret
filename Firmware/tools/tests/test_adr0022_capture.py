"""Local Linux boundary and corruption tests. No station access or motor I/O."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
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
from adr0022_capture_launch import preflight

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
