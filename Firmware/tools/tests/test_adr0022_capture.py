"""Local Linux launcher/supervisor boundary tests. No station access or motor I/O."""
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import textwrap
import pytest
pytest.importorskip("fcntl", reason="Linux capture supervision")

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
from adr0022_capture_launch import preflight

pytestmark = pytest.mark.skipif(sys.platform != "linux", reason="real Linux recvmsg/process boundary")
ROOT = TOOLS.parents[1]
BINARY = Path(os.environ.get("OTA_STAGE1_BUILD", ROOT / "run/adr0022-local/firmware")) / "axis_control_core/commissiond"
LAUNCHER_TOOLS = ("adr0022_capture_launch.py", "adr0022_homing_review.py", "adr0022_yaw_data.py",
                  "adr0022_yaw_vibration.py", "station_preflight.py")


def prepare_launcher_fixture(tmp_path):
    """A release-shaped Firmware tree: launcher, supervisor tools, commissiond and a fake IMU producer."""
    firmware = tmp_path / "Firmware"
    for part in ("tools", "scripts", "config", "build/axis_control_core"):
        (firmware / part).mkdir(parents=True)
    for name in LAUNCHER_TOOLS:
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
    return firmware


def free_ports(count):
    reservations = []
    for _ in range(count):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        reservations.append(sock)
    ports = [s.getsockname()[1] for s in reservations]
    for sock in reservations:
        sock.close()
    return ports


def yaw_control_manifest(tmp_path):
    ports = free_ports(3)
    return {"schema": "adr0022.yaw-control/1", "purpose": "yaw_shared_core_3a", "provenance": "SYNTHETIC",
            "transport": "loopback_udp", "output": str(tmp_path / "yaw-control.jsonl"),
            "yaw": {"port": ports[0], "peer_port": ports[2]}, "pitch": {"port": ports[1], "peer_port": ports[2]},
            "expected_pitch_uid": "7216313130333105", "pitch_supported_when_disabled": True,
            "baseline_s": 2., "stop_observation_s": 2., "pitch_maximum_temperature_C": 45., "yaw_current_bound_A": 3.,
            "candidate_label": "fixture", "servo_parameters": {"use_gyro": 0}, "gyro_calibration": {"yaw_column": [0, 0, 1]},
            "other_axis_posture_rad": 0., "reference_segments": [{"duration_s": 1., "target_position_rad": .1}],
            "limits": {"clock_uncertainty_s": .001, "dequeue_age_s": .1, "can_gap_s": .1, "imu_gap_s": .2,
                       "startup_s": .5, "duration_s": 20., "minimum_imu_status": 0, "read_timeout_s": .2,
                       "read_period_s": .01, "stop_period_s": .02}}


def launcher_env(tmp_path):
    return dict(os.environ, OTA_PYTHON=sys.executable, OTA_RUN_DIR=str(tmp_path / "runtime"))


def test_launcher_check_control_yaw_runs_preflight_only(tmp_path):
    firmware = prepare_launcher_fixture(tmp_path)
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps(yaw_control_manifest(tmp_path)))
    checked = subprocess.run(["bash", str(firmware / "scripts/run_application.sh"), "check", "--control-yaw", str(manifest)],
                             env=launcher_env(tmp_path), capture_output=True, text=True, timeout=30)
    assert checked.returncode == 0, checked.stdout + checked.stderr
    assert "Yaw shared-core control manifest and binaries checked; devices unopened" in checked.stdout
    assert "Preflight passed" in checked.stdout
    assert not (tmp_path / "yaw-control.attempt.json").exists() and not (tmp_path / "yaw-control.jsonl").exists()


@pytest.mark.parametrize("option", ["--capture-baseline", "--prepare-current", "--characterize-current", "--acquire-yaw"])
def test_launcher_refuses_removed_acquisition_modes(tmp_path, option):
    firmware = prepare_launcher_fixture(tmp_path)
    manifest = tmp_path / "manifest.json"
    manifest.write_text("{}\n")
    refused = subprocess.run(["bash", str(firmware / "scripts/run_application.sh"), "check", option, str(manifest)],
                             env=launcher_env(tmp_path), capture_output=True, text=True, timeout=30)
    assert refused.returncode == 2 and f"unknown option: {option}" in refused.stderr


def test_preflight_requires_exactly_one_session(tmp_path):
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps(yaw_control_manifest(tmp_path)))
    for modes in ({}, {"control_yaw": True, "establish_homing": True}):
        with pytest.raises(ValueError, match="choose a single session"):
            preflight(manifest, prepare_launcher_fixture(tmp_path / str(len(modes))), **modes)


def test_preflight_rejects_timing_that_rounds_to_zero_before_sensor_start(tmp_path):
    firmware = prepare_launcher_fixture(tmp_path)
    config = yaw_control_manifest(tmp_path)
    config["limits"]["read_period_s"] = 1e-12
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="at least one nanosecond"):
        preflight(manifest, firmware, control_yaw=True)
    assert not (tmp_path / "yaw-control.attempt.json").exists()


def homing_launcher_probe(tmp_path, monkeypatch, fault="none"):
    """Reuse the existing protocol plant, with the actual launcher as process owner."""
    from adr0022_homing_rehearsal import rehearse as homing_rehearse, PASS_FAULTS
    firmware = prepare_launcher_fixture(tmp_path)
    wrapper = tmp_path / "homing-launcher-wrapper"
    wrapper.write_text(f"#!{sys.executable}\n" + textwrap.dedent('''\
        import json, os, signal, subprocess, sys, time
        from pathlib import Path
        firmware = Path(os.environ['HOMING_LAUNCHER_FIRMWARE'])
        manifest = Path(sys.argv[-1])
        config = json.loads(manifest.read_text())
        os.close(config.pop('imu_fd'))
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
                           env=launcher_env(tmp_path), text=True, capture_output=True, timeout=10)
    assert again.returncode != 0 and "evidence reuse" in again.stderr


def test_homing_launcher_complete(tmp_path, monkeypatch):
    homing_launcher_probe(tmp_path, monkeypatch)


def test_homing_launcher_cancellation_allows_cpp_stop(tmp_path, monkeypatch):
    homing_launcher_probe(tmp_path, monkeypatch, fault="interrupt")


def test_homing_launcher_measured_native_offset_preserves_observations(tmp_path, monkeypatch):
    homing_launcher_probe(tmp_path, monkeypatch, fault="native_position_offset")


@pytest.mark.parametrize("invalid", ["missing_parameter", "unsafe_command_cap"])
def test_homing_preflight_uses_exact_runtime_numeric_validation_before_sensor(tmp_path, invalid):
    from adr0022_homing_rehearsal import fixture
    firmware = prepare_launcher_fixture(tmp_path)
    config = fixture(free_ports(2), 31003, -1, tmp_path / "homing.jsonl")
    config.pop("imu_fd")
    if invalid == "missing_parameter":
        del config["homing"]["arrival_tol_rad"]
    else:
        config["homing"]["limit_cur_max_a"] = 6.
    manifest = tmp_path / "homing.json"
    manifest.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="runtime homing parameter validation failed"):
        preflight(manifest, firmware, establish_homing=True)
    assert not (tmp_path / "homing.jsonl").exists()
    assert not (tmp_path / "homing.attempt.json").exists()
    assert not (tmp_path / "homing.imu.log").exists()
