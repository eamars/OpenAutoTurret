"""Launcher-owned, single-attempt supervision of a commissiond session.

Python starts processes and reviews files; only commissiond can open CAN. Two
sessions remain: --control-yaw (yaw position servo, pitch disabled) and
--establish-homing (bounded pitch sensorless homing, optionally a servo trial).
A yaw session whose manifest has `camera_capture: {seconds}` also records the
tracking camera beside it (tools/tracking/camera_record.py, ADR-003 stage 2
timing calibration): same lease, same lifetime, no motor or CAN access.
"""
from __future__ import annotations
import argparse
try:
    import fcntl
except ImportError:
    fcntl = None  # Portable bundle validation; launching remains Linux-only.
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from adr0022_homing_review import strict_json

HOMING_SCHEMA = "adr0022.sensorless-homing/1"
HOMING_PURPOSE = "pitch_sensorless_homing"
YAW_CONTROL_SCHEMA = "adr0022.yaw-control/1"
YAW_CONTROL_PURPOSE = "yaw_shared_core_3a"
PROTECTION_DOCUMENT = "docs/references/cybergear/CyberGear微电机使用说明书.pdf"
def validate_current_contract(config, establish_homing=True):
    """Validate portable sensorless-homing session fields without accessing a device."""
    if not establish_homing:
        raise ValueError("only the sensorless homing current contract is supported")
    if config.get("schema") != HOMING_SCHEMA or config.get("purpose") != HOMING_PURPOSE:
        raise ValueError("neutral current preparation schema/purpose required")
    if config.get("pitch_supported_when_disabled") is not True:
        raise ValueError("pitch must be physically supported for current preparation")
    expected_basis = {"kind": "manufacturer_continuous_current_rating", "document": PROTECTION_DOCUMENT,
                      "continuous_current_A": 6.5}
    basis = config.get("protection_limit_basis", {})
    basis_matches = isinstance(basis, dict) and all(basis.get(key) == value for key, value in expected_basis.items())
    guards, settings = config.get("guards"), config.get("native_settings")
    if not isinstance(guards, dict) or not isinstance(settings, dict):
        raise ValueError("explicit homing current protection and native command cap required")
    protection, command_cap = guards.get("current_bound_A"), settings.get("homing_limit_cur_A")
    if type(protection) not in (int, float) or not math.isfinite(protection) or not 0 < protection <= 6.5:
        raise ValueError("homing measured current protection must be explicit, positive and <=6.5 A")
    if type(command_cap) not in (int, float) or not math.isfinite(command_cap) or not 0 < command_cap <= 5:
        raise ValueError("homing native current command cap must be explicit, positive and <=5 A")
    if config.get("provenance") == "MEASURED" and not basis_matches:
        raise ValueError("homing requires manufacturer 6.5 A rated protection basis")
    if config.get("provenance") == "MEASURED":
        attendance = config.get("operator_attendance", {})
        authorization = config.get("session_authorization", {})
        if not isinstance(attendance, dict) or not isinstance(authorization, dict):
            raise ValueError("explicit attendance and session authorization objects required")
        if type(attendance.get("present_at_manual_cutoff")) is not bool or any(
                not isinstance(attendance.get(k), str) or not attendance[k].strip()
                for k in ("operator_identity", "manual_cutoff_evidence_identity")):
            raise ValueError("explicit operator attendance fact and manual cutoff evidence identity required")
        if authorization.get("purpose") != HOMING_PURPOSE or authorization.get("sensorless_homing_authorized") is not True or not isinstance(
                authorization.get("authorization_identity"), str) or not authorization["authorization_identity"].strip():
            raise ValueError("explicit current-mode session authorization identity required")
        if attendance["present_at_manual_cutoff"] is False and (authorization.get("unattended_operation_authorized") is not True or authorization.get("presence_required") is not False):
            raise ValueError("unattended operation requires explicit authorization without a presence requirement")
        # Original native parameters are verified by fresh disabled readbacks
        # in commissiond; a local simulation certificate is not an entry gate.


def validate_yaw_control_contract(config):
    """Check finite session fields; commissiond applies and reads back the core."""
    if config.get("schema") != YAW_CONTROL_SCHEMA or config.get("purpose") != YAW_CONTROL_PURPOSE:
        raise ValueError("yaw shared-core control schema/purpose required")
    if config.get("pitch_supported_when_disabled") is not True:
        raise ValueError("yaw control requires supported disabled pitch")
    for key in ("baseline_s", "stop_observation_s", "yaw_current_bound_A", "pitch_maximum_temperature_C"):
        value = config.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"finite positive {key} required")
    # The servo may peak to the 3 A full scale under its own RMS budget (<=1.62 A, checked natively).
    authority = 3.0 if "servo_parameters" in config else .9
    if config["yaw_current_bound_A"] > authority or config["baseline_s"] < 2 or config["stop_observation_s"] != 2:
        raise ValueError("yaw control requires bounded current authority and complete baseline/stop windows")
    if not isinstance(config.get("candidate_label"), str) or not config["candidate_label"].strip():
        raise ValueError("descriptive yaw candidate label required")
    control_key = "servo_parameters" if "servo_parameters" in config else "controller_parameters"
    for key in (control_key, "gyro_calibration"):
        if not isinstance(config.get(key), dict):
            raise ValueError(f"explicit {key} required for runtime application")
    for key in ("other_axis_posture_rad", "yaw_position_offset_rad"):
        value = config.get(key, 0.) if key == "yaw_position_offset_rad" else config.get(key)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"finite {key} required")
    samples = config.get("reference_samples")
    if samples is not None:
        if config.get("reference_segments") is not None:
            raise ValueError("choose yaw reference samples or position segments")
        if not isinstance(samples, list) or len(samples) < 2:
            raise ValueError("finite yaw reference sample table required")
        previous = -1.
        for index, sample in enumerate(samples):
            if not isinstance(sample, dict) or any(
                    type(sample.get(key)) not in (int, float) or not math.isfinite(sample[key])
                    for key in ("time_s", "position_rad", "velocity_rad_s", "acceleration_rad_s2")):
                raise ValueError("finite yaw reference time and q/v/a required")
            if (index == 0 and sample["time_s"] != 0) or sample["time_s"] <= previous:
                raise ValueError("yaw reference samples must start at zero and increase in time")
            previous = sample["time_s"]
        total = previous
    else:
        segments = config.get("reference_segments")
        if not isinstance(segments, list) or not segments:
            raise ValueError("finite yaw reference segments required")
        total = 0.
        for segment in segments:
            if not isinstance(segment, dict) or any(
                    type(segment.get(key)) not in (int, float) or not math.isfinite(segment[key])
                    for key in ("duration_s", "target_position_rad")) or segment["duration_s"] <= 0:
                raise ValueError("finite yaw reference duration and target required")
            total += segment["duration_s"]
    limits = config.get("limits", {})
    if limits.get("duration_s", 0) <= limits.get("startup_s", 0) + config["baseline_s"] + total + config["stop_observation_s"]:
        raise ValueError("yaw references and stop observation must fit the finite session")


def verify_protection_basis(config, firmware):
    """Require the declared manufacturer source file without computing a digest."""
    if not (Path(firmware) / PROTECTION_DOCUMENT).is_file():
        raise ValueError("manufacturer protection document unavailable")


def exclusive_json(path, value):
    with Path(path).open("x", encoding="utf-8") as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())


def preflight(manifest: Path, firmware: Path, *, establish_homing=False, control_yaw=False):
    if establish_homing == control_yaw:
        raise ValueError("choose a single session: control_yaw or establish_homing")
    config = strict_json(manifest.read_text(encoding="utf-8"))
    expected_schema = YAW_CONTROL_SCHEMA if control_yaw else HOMING_SCHEMA
    if config.get("schema") != expected_schema or config.get("provenance") not in ("SYNTHETIC", "MEASURED"):
        raise ValueError("capture schema/provenance required")
    if control_yaw:
        validate_yaw_control_contract(config)
    else:
        validate_current_contract(config, establish_homing=True)
    synthetic = config["provenance"] == "SYNTHETIC"
    if config.get("transport") != ("loopback_udp" if synthetic else "socketcan"):
        raise ValueError("transport does not match provenance")
    uid = config.get("expected_pitch_uid", "")
    if not isinstance(uid, str) or len(uid) != 16 or any(c not in "0123456789abcdef" for c in uid):
        raise ValueError("expected pitch UID required")
    limits = config["limits"]
    for key in ("clock_uncertainty_s", "dequeue_age_s", "can_gap_s", "imu_gap_s", "startup_s",
                "duration_s", "read_timeout_s", "read_period_s", "stop_period_s"):
        value = limits[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value < 1e-9:
            raise ValueError(f"explicit {key} of at least one nanosecond required")
    if limits["duration_s"] <= limits["startup_s"] or limits["startup_s"] < (0 if synthetic else 3):
        raise ValueError("capture must include complete sensor startup")
    if type(limits["minimum_imu_status"]) is not int or not 0 <= limits["minimum_imu_status"] <= 3:
        raise ValueError("IMU quality requirement must be explicit")
    if limits["read_timeout_s"] > 5:
        raise ValueError("neutral preparation requires bounded <=5 s read/STOP deadlines")
    output = Path(config["output"])
    if not output.is_absolute() or not output.parent.is_dir():
        raise ValueError("existing absolute capture directory required")
    for path in (output, output.with_suffix(".attempt.json"), output.with_suffix(".bound.json"),
                 output.with_suffix(".review.json"), output.with_suffix(".result.json"),
                 output.with_suffix(".imu.log"), output.with_suffix(".processes.json")):
        if path.exists() or path.is_symlink():
            raise ValueError(f"refusing capture/evidence reuse: {path}")
    binaries = {"commissiond": firmware / "build/axis_control_core/commissiond", "imu": firmware / "build/imu-bno085"}
    for key, path in binaries.items():
        if not path.is_file() or not os.access(path, os.X_OK):
            raise ValueError(f"missing or non-executable {key}")
    if establish_homing:
        checked = subprocess.run([str(binaries["commissiond"]), "--validate-homing", str(manifest)],
                                 capture_output=True, text=True, timeout=5)
        if checked.returncode:
            raise ValueError("runtime homing parameter validation failed: " + checked.stderr.strip())
    if not synthetic:
        if uid != "7216313130333105":
            raise ValueError("station baseline requires known pitch UID and STOP feedback polling")
        if establish_homing:
            verify_protection_basis(config, firmware)
        from station_preflight import _validate_can_spi_mapping
        for axis, iface, spi in (("yaw", "can0", "spi0.0"), ("pitch", "can1", "spi1.0")):
            if config[axis] != {"interface": iface}:
                raise ValueError(f"unsupported {axis} endpoint")
            _validate_can_spi_mapping({"interface": iface, "spi_parent": spi}, axis, require_up=True)
        if not os.access("/dev/i2c-1", os.R_OK | os.W_OK):
            raise ValueError("BNO085 device access unavailable")
    return config, binaries


def capture(manifest: Path, firmware: Path, *, establish_homing=False, control_yaw=False):
    yaw_motion = control_yaw
    purpose = YAW_CONTROL_PURPOSE if control_yaw else HOMING_PURPOSE
    config, binaries = preflight(manifest, firmware, establish_homing=establish_homing, control_yaw=control_yaw)
    output = Path(config["output"])
    synthetic = config["provenance"] == "SYNTHETIC"
    if not synthetic:
        inherited = os.fstat(8)
        named = Path(f"/tmp/ota-motion-{os.getuid()}.lock").stat()
        if (inherited.st_dev, inherited.st_ino) != (named.st_dev, named.st_ino):
            raise ValueError("launcher motion lease required before sensor reset")
        fcntl.flock(8, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Check uncooperative old binaries as well as cooperating launchers.
        for proc in Path("/proc").iterdir():
            if not proc.name.isdigit():
                continue
            try:
                name = (proc / "comm").read_text().strip()
            except FileNotFoundError:
                continue
            if name in ("controld", "commissiond", "imu_main", "imu-bno085"):
                raise ValueError(f"existing device consumer {proc.name}: {name}")
    attempt_schema = "adr0022.yaw_control_attempt/1" if control_yaw else "adr0022.sensorless_homing_attempt/1"
    attempt = {"schema": attempt_schema, "provenance": config["provenance"],
               "manifest_path": str(manifest), "binary_paths": {key: str(path) for key, path in binaries.items()},
               "started_ns": time.monotonic_ns(), "automatic_retries": 0, "motion_requested": establish_homing or yaw_motion,
               "pitch_stop_requests": True}
    attempt.update(purpose=purpose, mode_transition_requested=not yaw_motion, nonzero_current_requested=yaw_motion,
                   session_label=config.get("session_label"), source_description=config.get("source_description"),
                   session_authorization=config.get("session_authorization"), operator_attendance=config.get("operator_attendance"))
    if establish_homing:
        attempt.update(native_settings_source=config.get("native_settings_source"),
                       protection_limit_basis=config["protection_limit_basis"])
    if control_yaw:
        attempt["candidate_label"] = config["candidate_label"]
    exclusive_json(output.with_suffix(".attempt.json"), attempt)
    children = []
    cancelled = False
    def cancel(_signum, _frame):
        nonlocal cancelled
        cancelled = True
    old_handlers = {sig: signal.signal(sig, cancel) for sig in (signal.SIGINT, signal.SIGTERM)}
    result = {"status": "INVALID", "provenance": config["provenance"], "motion_authorized": False,
              "physical_parameters_qualified": False}
    result.update(purpose=purpose, mode_transition_requested=not yaw_motion, nonzero_current_requested=yaw_motion,
                  pitch_stop_confirmed=False, yaw_stop_confirmed=False, independent_cutoff_qualified=False,
                  forced_termination=False, process_loss=False)
    if establish_homing:
        result.update(homing_observed=False, current_mode_qualified=False, dynamics_qualified=False,
                      retained_calibration_modified=False, motion_authorized=not synthetic)
    if yaw_motion:
        result.update(current_mode_qualified=False, dynamics_qualified=False,
                      retained_calibration_modified=False, motion_authorized=not synthetic)
    if control_yaw:
        result.update(candidate_label=config["candidate_label"], shared_core_control_requested=True,
                      stage3a_qualified=False)
    imu_log = None
    imu = collector = camera = None
    producer_lost = False
    try:
        if control_yaw and config.get("camera_capture"):
            seconds = float(config["camera_capture"]["seconds"])
            if not 0 < seconds <= config["limits"]["duration_s"]:
                raise ValueError("camera_capture seconds must be positive and within the session deadline")
            camera_log = output.with_suffix(".camera.log").open("xb")
            fixed = config["camera_capture"].get("exposure_us")
            extra = [str(int(fixed)), str(float(config["camera_capture"]["analogue_gain"]))] if fixed else []
            camera = subprocess.Popen([sys.executable, str(firmware / "tools/tracking/camera_record.py"), str(output), str(seconds), *extra],
                                      stdout=camera_log, stderr=subprocess.STDOUT)
            children.append(camera)
            # The control must start after the camera delivers, or the motion would be half unseen.
            frames = output.with_suffix(".camera.jsonl")
            ready = time.monotonic() + 15
            while not (frames.is_file() and frames.read_text().count('"kind": "frame"') >= 3):
                if camera.poll() is not None or time.monotonic() > ready:
                    raise RuntimeError("tracking camera did not start delivering frames")
                time.sleep(.05)
        imu_log = output.with_suffix(".imu.log").open("xb")
        imu = subprocess.Popen([str(binaries["imu"]), "--commissioning"], stdout=subprocess.PIPE, stderr=imu_log)
        children.append(imu)
        config["imu_fd"] = imu.stdout.fileno()
        bound = output.with_suffix(".bound.json")
        exclusive_json(bound, config)
        descriptors = (imu.stdout.fileno(),) if synthetic else (imu.stdout.fileno(), 8)
        option = "--control-yaw" if control_yaw else "--establish-homing"
        collector = subprocess.Popen([str(binaries["commissiond"]), option, str(bound)], pass_fds=descriptors)
        children.append(collector)
        exclusive_json(output.with_suffix(".processes.json"), {
            "schema": "adr0022.acquisition_processes/1", "supervisor_pid": os.getpid(),
            "imu_pid": imu.pid, "collector_pid": collector.pid, "camera_pid": camera.pid if camera else None,
            "recorded_ns": time.monotonic_ns(), "automatic_retries": 0})
        # The collector exclusively drains this pipe. Keeping the read end here
        # would conceal collector death from the producer's broken-pipe signal.
        imu.stdout.close()
        deadline = time.monotonic() + config["limits"]["duration_s"] + 15
        while collector.poll() is None:
            if cancelled or imu.poll() is not None or time.monotonic() >= deadline:
                producer_lost = imu.poll() is not None
                collector.terminate()
                # Let the C++ owner request/observe STOP with the IMU still alive.
                try:
                    collector.wait(timeout=max(5, config["limits"]["read_timeout_s"] + 2))
                except subprocess.TimeoutExpired:
                    collector.kill()
                    collector.wait(timeout=5)
                    result.update(forced_termination=True)
                raise RuntimeError("capture cancelled, producer exited, or supervised deadline expired")
            time.sleep(.01)
        if collector.returncode:
            raise RuntimeError(f"commissiond rejected acquisition (exit {collector.returncode})")
        if yaw_motion:
            from adr0022_yaw_data import summarize
            report = summarize(output)
        else:
            from adr0022_homing_review import review as homing_review
            report = homing_review(output)
            result["homing_observed"] = report["homing_observed"]
        exclusive_json(output.with_suffix(".review.json"), report)
        complete = report.get("capture_complete") is True and report.get("capture_footer_status") == "COMPLETE"
        result.update(status="COMPLETE" if complete else "INVALID", capture_path=str(output),
                      capture_footer_status=report.get("capture_footer_status"),
                      capture_footer_detail=report.get("capture_footer_detail"))
        if not complete:
            result["detail"] = "Raw review did not confirm complete acquisition"
    except Exception as exc:
        result["detail"] = str(exc)
    finally:
        # Exact children only; no station-wide process kill or automatic restart.
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
            try:
                child.wait(timeout=max(5, config["limits"]["read_timeout_s"] + 2) if child is collector else 5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)
                result.update(status="INVALID", detail="capture child required forced termination", forced_termination=True)
        result["process_loss"] = producer_lost or collector is None or collector.returncode is None or collector.returncode < 0 or imu is None
        if not result["forced_termination"] and not result["process_loss"] and output.is_file():
            try:
                with output.open("rb") as evidence:
                    evidence.seek(max(0, output.stat().st_size - 65536))
                    footer = strict_json(evidence.read().splitlines()[-1].decode())
                result["pitch_stop_confirmed"] = footer.get("kind") == "footer" and (footer.get("normal_stop_confirmed") is True or footer.get("abort_stop_confirmed") is True)
            except (OSError, ValueError, IndexError, UnicodeError):
                pass
        result["stop_detail"] = "Pitch disabled feedback observed; yaw zero is a request without stop qualification" if result["pitch_stop_confirmed"] else "STOP unconfirmed; manual cutoff remains operator responsibility"
        if imu_log:
            imu_log.close()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        exclusive_json(output.with_suffix(".result.json"), result)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == "COMPLETE" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--firmware", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--preflight-only", action="store_true")
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--establish-homing", action="store_true")
    modes.add_argument("--control-yaw", action="store_true")
    args = parser.parse_args()
    if args.preflight_only:
        preflight(args.manifest, args.firmware, establish_homing=args.establish_homing, control_yaw=args.control_yaw)
        print("Yaw shared-core control" if args.control_yaw else "Sensorless homing", "manifest and binaries checked; devices unopened")
    else:
        raise SystemExit(capture(args.manifest, args.firmware, establish_homing=args.establish_homing, control_yaw=args.control_yaw))
