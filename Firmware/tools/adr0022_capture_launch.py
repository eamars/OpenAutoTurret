"""Launcher-owned, single-attempt supervision of C++ acquisition.

Python starts processes and reviews files; only commissiond can open CAN, and
Baseline permits discovery, STOP and reads. Neutral preparation additionally
permits its explicitly authorized zero-current mode transition in commissiond.
"""
from __future__ import annotations
import argparse
try:
    import fcntl
except ImportError:
    fcntl = None  # Portable bundle validation; launching remains Linux-only.
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from adr0022_capture_review import review, strict_json

CURRENT_SCHEMA = "adr0022.current-preparation/1"
CURRENT_PURPOSE = "neutral_current_mode_verification"
CHARACTERIZATION_SCHEMA = "adr0022.neutral-characterization/1"
CHARACTERIZATION_PURPOSE = "neutral_current_measurement_characterization"
HOMING_SCHEMA = "adr0022.sensorless-homing/1"
HOMING_PURPOSE = "pitch_sensorless_homing"
PROTECTION_DOCUMENT = "docs/references/cybergear/CyberGear微电机使用说明书.pdf"
PROTECTION_DOCUMENT_SHA256 = "4fe8727a690193953e62438c04abd25f8e8be232e02b4eddf3aa1f99610da495"
SOURCE_DIRECTORIES = ("axis_control_core", "commission_runtime", "control/src", "third_party/sh2")
SOURCE_FILES = ("CMakeLists.txt", "control/CMakeLists.txt", "scripts/run_application.sh",
                "tools/imu_bno085.c", "tools/adr0022_capture_launch.py", "tools/adr0022_capture_review.py",
                "tools/adr0022_current_review.py", "tools/adr0022_homing_review.py",
                "tools/adr0022_baseline_bundle.py", "tools/station_preflight.py")


def source_identity(firmware):
    """Hash an explicit acquisition dependency set identically before/after shipping."""
    firmware = Path(firmware)
    paths = set(SOURCE_FILES)
    for relative in SOURCE_DIRECTORIES:
        directory = firmware / relative
        if not directory.is_dir():
            raise ValueError(f"missing acquisition source directory: {relative}")
        paths.update(p.relative_to(firmware).as_posix() for p in directory.rglob("*")
                     if p.is_file() and (p.suffix in (".cpp", ".c", ".hpp", ".h") or p.name == "CMakeLists.txt"))
    files = {name: hashlib.sha256((firmware / name).read_bytes().replace(b"\r\n", b"\n")).hexdigest()
             for name in sorted(paths)}
    return {"source_sha256": canonical_sha(files), "normalization": "CRLF_TO_LF", "files": files}


def canonical_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def validate_current_contract(config, characterize_current=False, establish_homing=False):
    """Validate portable neutral-session fields without accessing a device."""
    schema = HOMING_SCHEMA if establish_homing else CHARACTERIZATION_SCHEMA if characterize_current else CURRENT_SCHEMA
    purpose = HOMING_PURPOSE if establish_homing else CHARACTERIZATION_PURPOSE if characterize_current else CURRENT_PURPOSE
    if config.get("schema") != schema or config.get("purpose") != purpose:
        raise ValueError("neutral current preparation schema/purpose required")
    if config.get("pitch_supported_when_disabled") is not True:
        raise ValueError("pitch must be physically supported for current preparation")
    for key in (() if establish_homing else ("neutral_current_bound_A", "transition_displacement_bound_rad", "pitch_maximum_temperature_C")):
        value = config.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"explicit finite positive {key} required")
    if characterize_current:
        observation = config.get("neutral_observation_s")
        timing = config.get("limits", {})
        if type(observation) not in (int, float) or not math.isfinite(observation) or not 0 < observation <= 60:
            raise ValueError("explicit finite positive characterization neutral_observation_s <=60 s required")
        if not isinstance(timing, dict) or any(type(timing.get(key)) not in (int, float) or not math.isfinite(timing[key]) or timing[key] <= 0 for key in ("startup_s", "duration_s")) or timing["duration_s"] <= timing["startup_s"] + observation:
            raise ValueError("characterization duration must cover startup and the explicit neutral observation")
        expected_basis = {"kind": "manufacturer_continuous_current_rating", "document": PROTECTION_DOCUMENT,
                          "sha256": PROTECTION_DOCUMENT_SHA256, "continuous_current_A": 6.5}
        if config.get("protection_current_bound_A") != 6.5 or config.get("protection_limit_basis") != expected_basis:
            raise ValueError("characterization requires manufacturer 6.5 A rated protection basis")
    if config.get("provenance") == "MEASURED":
        attendance = config.get("operator_attendance", {})
        authorization = config.get("session_authorization", {})
        if not isinstance(attendance, dict) or not isinstance(authorization, dict):
            raise ValueError("explicit attendance and session authorization objects required")
        if type(attendance.get("present_at_manual_cutoff")) is not bool or any(
                not isinstance(attendance.get(k), str) or not attendance[k].strip()
                for k in ("operator_identity", "manual_cutoff_evidence_identity")):
            raise ValueError("explicit operator attendance fact and manual cutoff evidence identity required")
        authorization_flag = "sensorless_homing_authorized" if establish_homing else "current_mode_enable_authorized"
        if authorization.get("purpose") != purpose or authorization.get(authorization_flag) is not True or not isinstance(
                authorization.get("authorization_identity"), str) or not authorization["authorization_identity"].strip():
            raise ValueError("explicit current-mode session authorization identity required")
        if attendance["present_at_manual_cutoff"] is False and (authorization.get("unattended_operation_authorized") is not True or authorization.get("presence_required") is not False):
            raise ValueError("unattended operation requires explicit authorization without a presence requirement")
        revision = config.get("expected_revision", "")
        source = config.get("expected_source_sha256", "")
        if not isinstance(revision, str) or len(revision) != 40 or any(c not in "0123456789abcdef" for c in revision):
            raise ValueError("full committed source revision required")
        if not isinstance(source, str) or len(source) != 64 or any(c not in "0123456789abcdef" for c in source):
            raise ValueError("expected acquisition source SHA-256 required")
        qualification = config.get("local_qualification", {})
        if not isinstance(qualification, dict) or not isinstance(qualification.get("report"), dict):
            raise ValueError("embedded local current preparation qualification required")
        report = qualification.get("report", {})
        expected_status = "LOCAL_SENSORLESS_HOMING_PASS" if establish_homing else "LOCAL_CURRENT_CHARACTERIZATION_PASS" if characterize_current else "LOCAL_CURRENT_PREPARATION_PASS"
        if qualification.get("sha256") != canonical_sha(report) or report.get("status") != expected_status or (
                report.get("expected_binaries") != config.get("expected_binaries") or report.get("source_sha256") != source or
                report.get("revision") != revision or report.get("hardware_accessed") is not False or report.get("provenance") != "SYNTHETIC"):
            raise ValueError("matching successful local current preparation qualification required")
        if establish_homing:
            validate_native_settings_evidence(config)


def validate_native_settings_evidence(config):
    """Bind expected originals to the existing measured disabled capability asset."""
    evidence = config.get("native_settings_evidence", {})
    if not isinstance(evidence, dict) or not isinstance(evidence.get("asset"), dict):
        raise ValueError("measured disabled native-setting capability asset identity required")
    asset = evidence["asset"]
    if evidence.get("sha256") != canonical_sha(asset) or (
            asset.get("schema") != "adr0022.baseline_capabilities/1" or asset.get("provenance") != "MEASURED" or
            asset.get("capture_integrity") != "PASS" or asset.get("pitch_uid") != config.get("expected_pitch_uid")):
        raise ValueError("measured disabled native-setting capability asset identity required")
    source, registers, settings = asset.get("source"), asset.get("registers"), config.get("native_settings")
    if not isinstance(source, dict) or not isinstance(registers, dict) or not isinstance(settings, dict):
        raise ValueError("native-setting measurement source, registers and declared originals required")
    capture_hash = source.get("capture_sha256", "")
    if not isinstance(capture_hash, str) or len(capture_hash) != 64 or any(c not in "0123456789abcdef" for c in capture_hash):
        raise ValueError("native-setting measurement capture SHA-256 required")
    for name, index in (("expected_original_mode", "0x7005"), ("original_limit_cur_A", "0x7018"),
                        ("original_position_kp", "0x701e"), ("original_speed_kp", "0x701f"), ("original_speed_ki", "0x7020")):
        register = registers.get(index, {})
        observed = register.get("observed", {}) if isinstance(register, dict) else {}
        expected = settings.get(name)
        if not isinstance(register, dict) or not isinstance(observed, dict):
            raise ValueError(f"measured disabled native setting unavailable: {name}")
        if register.get("context") != "PITCH_DISABLED_BASELINE" or type(observed.get("count")) is not int or observed["count"] <= 0 or (
                type(expected) not in (int, float) or not math.isfinite(expected) or
                any(type(observed.get(k)) not in (int, float) or not math.isfinite(observed[k]) or observed[k] != expected for k in ("min", "max"))):
            raise ValueError(f"measured disabled native setting differs or is unavailable: {name}")


def verify_protection_basis(config, firmware):
    if sha(Path(firmware) / PROTECTION_DOCUMENT) != config["protection_limit_basis"]["sha256"]:
        raise ValueError("changed manufacturer protection document identity")


def validate_additional_registers(config):
    if "additional_startup_registers" not in config:
        return
    registers = config["additional_startup_registers"]
    if not isinstance(registers, list) or any(type(index) is not int or index not in (0x701e, 0x701f, 0x7020, 0x7017) for index in registers) or len(set(registers)) != len(registers):
        raise ValueError("additional startup registers must be unique documented native-setting indices")
    if config.get("register_reads") is not True or config.get("pitch_stop_poll") is not True:
        raise ValueError("additional native-setting reads require disabled pitch STOP polling and register reads")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def exclusive_json(path, value):
    with Path(path).open("x", encoding="utf-8") as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())


def preflight(manifest: Path, firmware: Path, prepare_current=False, characterize_current=False, establish_homing=False):
    if sum((prepare_current, characterize_current, establish_homing)) > 1:
        raise ValueError("choose a single neutral session purpose")
    active = prepare_current or characterize_current or establish_homing
    config = strict_json(manifest.read_text(encoding="utf-8"))
    expected_schema = HOMING_SCHEMA if establish_homing else CHARACTERIZATION_SCHEMA if characterize_current else CURRENT_SCHEMA if prepare_current else "adr0022.capture/2"
    if config.get("schema") != expected_schema or config.get("provenance") not in ("SYNTHETIC", "MEASURED"):
        raise ValueError("capture schema/provenance required")
    if active:
        validate_current_contract(config, characterize_current, establish_homing)
        if config.get("expected_source_sha256") != source_identity(firmware)["source_sha256"]:
            raise ValueError("changed acquisition source identity")
    synthetic = config["provenance"] == "SYNTHETIC"
    if config.get("transport") != ("loopback_udp" if synthetic else "socketcan"):
        raise ValueError("transport does not match provenance")
    if not active:
        validate_additional_registers(config)
        if type(config.get("register_reads")) is not bool or type(config.get("pitch_stop_poll")) is not bool:
            raise ValueError("register_reads and pitch_stop_poll must be explicit")
        if config["pitch_stop_poll"] and config.get("pitch_supported_when_disabled") is not True:
            raise ValueError("pitch must be supported before STOP polling")
    uid = config.get("expected_pitch_uid", "")
    if not isinstance(uid, str) or len(uid) != 16 or any(c not in "0123456789abcdef" for c in uid):
        raise ValueError("expected pitch UID required")
    limits = config["limits"]
    for key in ("clock_uncertainty_s", "dequeue_age_s", "can_gap_s", "imu_gap_s", "startup_s",
                "duration_s", "read_timeout_s", "read_period_s", "stop_period_s"):
        value = limits[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not 1e-9 <= value <= 3600:
            raise ValueError(f"explicit {key} of at least one nanosecond required")
    if limits["duration_s"] <= limits["startup_s"] or limits["startup_s"] < (0 if synthetic else 3):
        raise ValueError("capture must include complete sensor startup")
    if type(limits["minimum_imu_status"]) is not int or not 0 <= limits["minimum_imu_status"] <= 3:
        raise ValueError("IMU quality requirement must be explicit")
    if active and (limits["read_timeout_s"] > 5 or limits["duration_s"] > 120):
        raise ValueError("neutral preparation requires bounded <=5 s read/STOP and <=120 s session deadlines")
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
        if not os.access(path, os.X_OK) or sha(path) != config["expected_binaries"][key]:
            raise ValueError(f"missing or changed {key} executable")
    if establish_homing:
        checked = subprocess.run([str(binaries["commissiond"]), "--validate-homing", str(manifest)],
                                 capture_output=True, text=True, timeout=5)
        if checked.returncode:
            raise ValueError("runtime homing parameter validation failed: " + checked.stderr.strip())
    if not synthetic:
        if (not active and not config["pitch_stop_poll"]) or uid != "7216313130333105":
            raise ValueError("station baseline requires known pitch UID and STOP feedback polling")
        if active:
            identity = source_identity(firmware)
            revision_path = firmware.parent / "REVISION"
            if not revision_path.is_file() or revision_path.read_text().strip() != config["expected_revision"] or identity["source_sha256"] != config["expected_source_sha256"]:
                raise ValueError("changed acquisition source/release identity")
            if characterize_current:
                verify_protection_basis(config, firmware)
        from station_preflight import _validate_can_spi_mapping
        for axis, iface, spi in (("yaw", "can0", "spi0.0"), ("pitch", "can1", "spi1.0")):
            if config[axis] != {"interface": iface}:
                raise ValueError(f"unsupported {axis} endpoint")
            _validate_can_spi_mapping({"interface": iface, "spi_parent": spi}, axis, require_up=True)
        if not os.access("/dev/i2c-1", os.R_OK | os.W_OK):
            raise ValueError("BNO085 device access unavailable")
    return config, binaries


def capture(manifest: Path, firmware: Path, prepare_current=False, characterize_current=False, establish_homing=False):
    active = prepare_current or characterize_current or establish_homing
    purpose = HOMING_PURPOSE if establish_homing else CHARACTERIZATION_PURPOSE if characterize_current else CURRENT_PURPOSE
    config, binaries = preflight(manifest, firmware, prepare_current, characterize_current, establish_homing)
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
    attempt_schema = "adr0022.sensorless_homing_attempt/1" if establish_homing else "adr0022.neutral_characterization_attempt/1" if characterize_current else "adr0022.current_preparation_attempt/1" if prepare_current else "adr0022.capture_attempt/1"
    attempt = {"schema": attempt_schema, "provenance": config["provenance"],
               "manifest_sha256": sha(manifest), "binaries": config["expected_binaries"],
               "started_ns": time.monotonic_ns(), "automatic_retries": 0, "motion_requested": establish_homing,
               "pitch_stop_requests": True if active else config["pitch_stop_poll"]}
    if active:
        attempt.update(purpose=purpose, mode_transition_requested=True, nonzero_current_requested=False,
                       expected_revision=config.get("expected_revision"), source_sha256=config.get("expected_source_sha256"),
                       session_authorization=config.get("session_authorization"), operator_attendance=config.get("operator_attendance"),
                       local_qualification_sha256=config.get("local_qualification", {}).get("sha256"))
    if characterize_current:
        attempt.update(neutral_current_qualified=False, protection_limit_basis=config["protection_limit_basis"],
                       historical_diagnostic_bound_A=config["neutral_current_bound_A"], protection_current_bound_A=config["protection_current_bound_A"])
    if establish_homing:
        attempt.update(native_settings_evidence_sha256=config.get("native_settings_evidence", {}).get("sha256"))
    exclusive_json(output.with_suffix(".attempt.json"), attempt)
    children = []
    cancelled = False
    def cancel(_signum, _frame):
        nonlocal cancelled
        cancelled = True
    old_handlers = {sig: signal.signal(sig, cancel) for sig in (signal.SIGINT, signal.SIGTERM)}
    result = {"status": "INVALID", "provenance": config["provenance"], "motion_authorized": False,
              "physical_parameters_qualified": False}
    if active:
        result.update(purpose=purpose, mode_transition_requested=True, nonzero_current_requested=False,
                      pitch_stop_confirmed=False, yaw_stop_confirmed=False, independent_cutoff_qualified=False,
                      forced_termination=False, process_loss=False)
    if characterize_current:
        result.update(neutral_current_qualified=False, current_mode_qualified=False, dynamics_qualified=False)
    if establish_homing:
        result.update(homing_observed=False, current_mode_qualified=False, dynamics_qualified=False,
                      retained_calibration_modified=False, motion_authorized=not synthetic)
    imu_log = None
    imu = collector = None
    producer_lost = False
    try:
        imu_log = output.with_suffix(".imu.log").open("xb")
        imu = subprocess.Popen([str(binaries["imu"]), "--commissioning"], stdout=subprocess.PIPE, stderr=imu_log)
        children.append(imu)
        config["imu_fd"] = imu.stdout.fileno()
        bound = output.with_suffix(".bound.json")
        exclusive_json(bound, config)
        descriptors = (imu.stdout.fileno(),) if synthetic else (imu.stdout.fileno(), 8)
        option = "--establish-homing" if establish_homing else "--characterize-current" if characterize_current else "--prepare-current" if prepare_current else "--capture-baseline"
        collector = subprocess.Popen([str(binaries["commissiond"]), option, str(bound)], pass_fds=descriptors)
        children.append(collector)
        exclusive_json(output.with_suffix(".processes.json"), {
            "schema": "adr0022.acquisition_processes/1", "supervisor_pid": os.getpid(),
            "imu_pid": imu.pid, "collector_pid": collector.pid,
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
                    collector.wait(timeout=max(5, config["limits"]["read_timeout_s"] + 2) if active else 5)
                except subprocess.TimeoutExpired:
                    collector.kill()
                    collector.wait(timeout=5)
                    result.update(forced_termination=True)
                raise RuntimeError("capture cancelled, producer exited, or supervised deadline expired")
            time.sleep(.01)
        if collector.returncode:
            raise RuntimeError(f"commissiond rejected acquisition (exit {collector.returncode})")
        if establish_homing:
            from adr0022_homing_review import review as homing_review
            report = homing_review(output)
            result["homing_observed"] = report["homing_observed"]
        elif characterize_current:
            from adr0022_current_review import review_characterization
            report = review_characterization(output)
            result["neutral_current_criterion_satisfied"] = report["neutral_current_criterion_satisfied"]
        elif prepare_current:
            from adr0022_current_review import review as current_review
            report = current_review(output)
        else:
            report = review(output)
        exclusive_json(output.with_suffix(".review.json"), report)
        result.update(status="COMPLETE", capture_sha256=report["capture_sha256"])
    except Exception as exc:
        result["detail"] = str(exc)
    finally:
        # Exact children only; no station-wide process kill or automatic restart.
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
            try:
                child.wait(timeout=max(5, config["limits"]["read_timeout_s"] + 2) if active and child is collector else 5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)
                result.update(status="INVALID", detail="capture child required forced termination", forced_termination=True)
        if active:
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
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--prepare-current", action="store_true")
    modes.add_argument("--characterize-current", action="store_true")
    modes.add_argument("--establish-homing", action="store_true")
    args = parser.parse_args()
    if args.preflight_only:
        preflight(args.manifest, args.firmware, args.prepare_current, args.characterize_current, args.establish_homing)
        print("Sensorless homing" if args.establish_homing else "Neutral current characterization" if args.characterize_current else "Current preparation" if args.prepare_current else "Baseline capture", "manifest and binaries checked; devices unopened")
    else:
        raise SystemExit(capture(args.manifest, args.firmware, args.prepare_current, args.characterize_current, args.establish_homing))
