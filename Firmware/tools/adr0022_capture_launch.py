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
SOURCE_DIRECTORIES = ("axis_control_core", "commission_runtime", "control/src", "third_party/sh2")
SOURCE_FILES = ("CMakeLists.txt", "control/CMakeLists.txt", "scripts/run_application.sh",
                "tools/imu_bno085.c", "tools/adr0022_capture_launch.py", "tools/adr0022_capture_review.py",
                "tools/adr0022_current_review.py", "tools/adr0022_baseline_bundle.py", "tools/station_preflight.py")


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


def validate_current_contract(config):
    """Validate portable neutral-session fields without accessing a device."""
    if config.get("schema") != CURRENT_SCHEMA or config.get("purpose") != CURRENT_PURPOSE:
        raise ValueError("neutral current preparation schema/purpose required")
    if config.get("pitch_supported_when_disabled") is not True:
        raise ValueError("pitch must be physically supported for current preparation")
    for key in ("neutral_current_bound_A", "transition_displacement_bound_rad", "pitch_maximum_temperature_C"):
        value = config.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"explicit finite positive {key} required")
    if config.get("provenance") == "MEASURED":
        attendance = config.get("operator_attendance", {})
        authorization = config.get("session_authorization", {})
        if not isinstance(attendance, dict) or not isinstance(authorization, dict):
            raise ValueError("explicit attendance and session authorization objects required")
        if type(attendance.get("present_at_manual_cutoff")) is not bool or any(
                not isinstance(attendance.get(k), str) or not attendance[k].strip()
                for k in ("operator_identity", "manual_cutoff_evidence_identity")):
            raise ValueError("explicit operator attendance fact and manual cutoff evidence identity required")
        if authorization.get("purpose") != CURRENT_PURPOSE or authorization.get("current_mode_enable_authorized") is not True or not isinstance(
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
        if qualification.get("sha256") != canonical_sha(report) or report.get("status") != "LOCAL_CURRENT_PREPARATION_PASS" or (
                report.get("expected_binaries") != config.get("expected_binaries") or report.get("source_sha256") != source or
                report.get("revision") != revision or report.get("hardware_accessed") is not False or report.get("provenance") != "SYNTHETIC"):
            raise ValueError("matching successful local current preparation qualification required")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def exclusive_json(path, value):
    with Path(path).open("x", encoding="utf-8") as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())


def preflight(manifest: Path, firmware: Path, prepare_current=False):
    config = strict_json(manifest.read_text(encoding="utf-8"))
    if config.get("schema") != (CURRENT_SCHEMA if prepare_current else "adr0022.capture/2") or config.get("provenance") not in ("SYNTHETIC", "MEASURED"):
        raise ValueError("capture schema/provenance required")
    if prepare_current:
        validate_current_contract(config)
        if config.get("expected_source_sha256") != source_identity(firmware)["source_sha256"]:
            raise ValueError("changed acquisition source identity")
    synthetic = config["provenance"] == "SYNTHETIC"
    if config.get("transport") != ("loopback_udp" if synthetic else "socketcan"):
        raise ValueError("transport does not match provenance")
    if not prepare_current:
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
    if prepare_current and (limits["read_timeout_s"] > 5 or limits["duration_s"] > 120):
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
    if not synthetic:
        if (not prepare_current and not config["pitch_stop_poll"]) or uid != "7216313130333105":
            raise ValueError("station baseline requires known pitch UID and STOP feedback polling")
        if prepare_current:
            identity = source_identity(firmware)
            revision_path = firmware.parent / "REVISION"
            if not revision_path.is_file() or revision_path.read_text().strip() != config["expected_revision"] or identity["source_sha256"] != config["expected_source_sha256"]:
                raise ValueError("changed acquisition source/release identity")
        from station_preflight import _validate_can_spi_mapping
        for axis, iface, spi in (("yaw", "can0", "spi0.0"), ("pitch", "can1", "spi1.0")):
            if config[axis] != {"interface": iface}:
                raise ValueError(f"unsupported {axis} endpoint")
            _validate_can_spi_mapping({"interface": iface, "spi_parent": spi}, axis, require_up=True)
        if not os.access("/dev/i2c-1", os.R_OK | os.W_OK):
            raise ValueError("BNO085 device access unavailable")
    return config, binaries


def capture(manifest: Path, firmware: Path, prepare_current=False):
    config, binaries = preflight(manifest, firmware, prepare_current)
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
    attempt = {"schema": "adr0022.current_preparation_attempt/1" if prepare_current else "adr0022.capture_attempt/1", "provenance": config["provenance"],
               "manifest_sha256": sha(manifest), "binaries": config["expected_binaries"],
               "started_ns": time.monotonic_ns(), "automatic_retries": 0, "motion_requested": False,
               "pitch_stop_requests": True if prepare_current else config["pitch_stop_poll"]}
    if prepare_current:
        attempt.update(purpose=CURRENT_PURPOSE, mode_transition_requested=True, nonzero_current_requested=False,
                       expected_revision=config.get("expected_revision"), source_sha256=config.get("expected_source_sha256"),
                       session_authorization=config.get("session_authorization"), operator_attendance=config.get("operator_attendance"),
                       local_qualification_sha256=config.get("local_qualification", {}).get("sha256"))
    exclusive_json(output.with_suffix(".attempt.json"), attempt)
    children = []
    cancelled = False
    def cancel(_signum, _frame):
        nonlocal cancelled
        cancelled = True
    old_handlers = {sig: signal.signal(sig, cancel) for sig in (signal.SIGINT, signal.SIGTERM)}
    result = {"status": "INVALID", "provenance": config["provenance"], "motion_authorized": False,
              "physical_parameters_qualified": False}
    if prepare_current:
        result.update(purpose=CURRENT_PURPOSE, mode_transition_requested=True, nonzero_current_requested=False,
                      pitch_stop_confirmed=False, yaw_stop_confirmed=False, independent_cutoff_qualified=False,
                      forced_termination=False, process_loss=False)
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
        collector = subprocess.Popen([str(binaries["commissiond"]), "--prepare-current" if prepare_current else "--capture-baseline", str(bound)], pass_fds=descriptors)
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
                    collector.wait(timeout=max(5, config["limits"]["read_timeout_s"] + 2) if prepare_current else 5)
                except subprocess.TimeoutExpired:
                    collector.kill()
                    collector.wait(timeout=5)
                    result.update(forced_termination=True)
                raise RuntimeError("capture cancelled, producer exited, or supervised deadline expired")
            time.sleep(.01)
        if collector.returncode:
            raise RuntimeError(f"commissiond rejected acquisition (exit {collector.returncode})")
        if prepare_current:
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
                child.wait(timeout=max(5, config["limits"]["read_timeout_s"] + 2) if prepare_current and child is collector else 5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)
                result.update(status="INVALID", detail="capture child required forced termination", forced_termination=True)
        if prepare_current:
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
    parser.add_argument("--prepare-current", action="store_true")
    args = parser.parse_args()
    if args.preflight_only:
        preflight(args.manifest, args.firmware, args.prepare_current)
        print("Current preparation" if args.prepare_current else "Baseline capture", "manifest and binaries checked; devices unopened")
    else:
        raise SystemExit(capture(args.manifest, args.firmware, args.prepare_current))
