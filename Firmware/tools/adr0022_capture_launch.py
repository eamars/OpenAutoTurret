"""Launcher-owned, single-attempt supervision of baseline C++ acquisition.

Python starts processes and reviews files; only commissiond can open CAN, and
this mode permits discovery, normal STOP and reads. No mode writes or enable.
"""
from __future__ import annotations
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from adr0022_capture_review import review, strict_json


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def exclusive_json(path, value):
    with Path(path).open("x", encoding="utf-8") as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())


def preflight(manifest: Path, firmware: Path):
    config = strict_json(manifest.read_text(encoding="utf-8"))
    if config.get("schema") != "adr0022.capture/1" or config.get("provenance") not in ("SYNTHETIC", "MEASURED"):
        raise ValueError("capture schema/provenance required")
    synthetic = config["provenance"] == "SYNTHETIC"
    if config.get("transport") != ("loopback_udp" if synthetic else "socketcan"):
        raise ValueError("transport does not match provenance")
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
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 3600:
            raise ValueError(f"explicit positive {key} required")
    if limits["duration_s"] <= limits["startup_s"] or limits["startup_s"] < (0 if synthetic else 3):
        raise ValueError("capture must include complete sensor startup")
    if type(limits["minimum_imu_status"]) is not int or not 0 <= limits["minimum_imu_status"] <= 3:
        raise ValueError("IMU quality requirement must be explicit")
    output = Path(config["output"])
    if not output.is_absolute() or not output.parent.is_dir():
        raise ValueError("existing absolute capture directory required")
    for path in (output, output.with_suffix(".attempt.json"), output.with_suffix(".bound.json"),
                 output.with_suffix(".review.json"), output.with_suffix(".result.json"),
                 output.with_suffix(".imu.log")):
        if path.exists() or path.is_symlink():
            raise ValueError(f"refusing capture/evidence reuse: {path}")
    binaries = {"commissiond": firmware / "build/axis_control_core/commissiond", "imu": firmware / "build/imu-bno085"}
    for key, path in binaries.items():
        if not os.access(path, os.X_OK) or sha(path) != config["expected_binaries"][key]:
            raise ValueError(f"missing or changed {key} executable")
    if not synthetic:
        if not config["pitch_stop_poll"] or uid != "7216313130333105":
            raise ValueError("station baseline requires known pitch UID and STOP feedback polling")
        from station_preflight import _validate_can_spi_mapping
        for axis, iface, spi in (("yaw", "can0", "spi0.0"), ("pitch", "can1", "spi1.0")):
            if config[axis] != {"interface": iface}:
                raise ValueError(f"unsupported {axis} endpoint")
            _validate_can_spi_mapping({"interface": iface, "spi_parent": spi}, axis, require_up=True)
        if not os.access("/dev/i2c-1", os.R_OK | os.W_OK):
            raise ValueError("BNO085 device access unavailable")
    return config, binaries


def capture(manifest: Path, firmware: Path):
    config, binaries = preflight(manifest, firmware)
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
    attempt = {"schema": "adr0022.capture_attempt/1", "provenance": config["provenance"],
               "manifest_sha256": sha(manifest), "binaries": config["expected_binaries"],
               "started_ns": time.monotonic_ns(), "automatic_retries": 0, "motion_requested": False,
               "pitch_stop_requests": config["pitch_stop_poll"]}
    exclusive_json(output.with_suffix(".attempt.json"), attempt)
    children = []
    cancelled = False
    def cancel(_signum, _frame):
        nonlocal cancelled
        cancelled = True
    old_handlers = {sig: signal.signal(sig, cancel) for sig in (signal.SIGINT, signal.SIGTERM)}
    result = {"status": "INVALID", "provenance": config["provenance"], "motion_authorized": False,
              "physical_parameters_qualified": False}
    imu_log = None
    imu = collector = None
    try:
        imu_log = output.with_suffix(".imu.log").open("xb")
        imu = subprocess.Popen([str(binaries["imu"]), "--commissioning"], stdout=subprocess.PIPE, stderr=imu_log)
        children.append(imu)
        config["imu_fd"] = imu.stdout.fileno()
        bound = output.with_suffix(".bound.json")
        exclusive_json(bound, config)
        descriptors = (imu.stdout.fileno(),) if synthetic else (imu.stdout.fileno(), 8)
        collector = subprocess.Popen([str(binaries["commissiond"]), "--capture-baseline", str(bound)], pass_fds=descriptors)
        children.append(collector)
        # The collector exclusively drains this pipe. Keeping the read end here
        # would conceal collector death from the producer's broken-pipe signal.
        imu.stdout.close()
        deadline = time.monotonic() + config["limits"]["duration_s"] + 15
        while collector.poll() is None:
            if cancelled or imu.poll() is not None or time.monotonic() >= deadline:
                collector.terminate()
                raise RuntimeError("capture cancelled, producer exited, or supervised deadline expired")
            time.sleep(.01)
        if collector.returncode:
            raise RuntimeError(f"commissiond rejected acquisition (exit {collector.returncode})")
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
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)
                result.update(status="INVALID", detail="capture child required forced termination")
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
    args = parser.parse_args()
    if args.preflight_only:
        preflight(args.manifest, args.firmware)
        print("Baseline capture manifest and binaries checked; devices unopened")
    else:
        raise SystemExit(capture(args.manifest, args.firmware))
