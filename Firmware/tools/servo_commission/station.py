"""Where sessions run: the real station (rpi-turret) or the simulator standing in for it.

Station: cross-build commissiond once, deploy one acquisition release with
deploy_station.py (Windows Python, this machine's ssh identity), then run every
session of the commissioning in that release: upload its manifest, run it through
run_application.sh, fetch the journal. One release per commissioning run, not per
session (each release is ~140 MB on the Pi).

SimStation: the same interface, answered by servo-sim against a "true" plant the
pipeline never sees -- the offline proof that identification and design recover
a plant from data alone.
"""
import json
import math
import os
import re
import shlex
import subprocess
from pathlib import Path

import numpy as np

import journal
import sim

REPO = Path(__file__).resolve().parents[3]
HOST = os.environ.get("OTA_STATION", "rpi-turret")
XBUILD = os.environ.get("OTA_XBUILD", str(REPO / "run/adr0022-debian13/firmware-make"))
XLIB = os.environ.get("OTA_XLIB", str(REPO / "run/adr0022-debian13/cross-host-lib"))
WINDOWS_PYTHON = os.environ.get("OTA_WINDOWS_PYTHON", "python.exe")
LAUNCH = {"yaw": ("--control-yaw", "yaw-control.jsonl"), "pitch": ("--establish-homing", "sensorless-homing.jsonl")}


def _run(command, **kwargs):
    kwargs.setdefault("check", True)
    kwargs.setdefault("cwd", REPO)
    return subprocess.run(command, **kwargs)


def _relative(path):
    """Paths handed to Windows programs: relative to the repository (the shared cwd)."""
    return os.path.relpath(Path(path).resolve(), REPO).replace(os.sep, "/")


class Station:
    def __init__(self, out_dir, log=print, build=True):
        self.out = Path(out_dir)
        self.log = log
        self.build = build
        self.release = None

    def _ssh(self, command, **kwargs):
        return _run(["ssh.exe", "-o", "ConnectTimeout=15", HOST, command], **kwargs)

    def _prepare(self, manifest, label):
        if self.build:
            self.log("  cross-building commissiond (ARM64)")
            log = self.out / "xbuild.log"
            with open(log, "w") as f:
                _run(["make", f"-j{os.cpu_count()}", "commissiond", "imu-bno085"], cwd=XBUILD, stdout=f, stderr=subprocess.STDOUT,
                     env=dict(os.environ, LD_LIBRARY_PATH=XLIB))
        bundle_manifest = dict(manifest, session_label=label)
        bundle_manifest.pop("output", None)
        first = self.out / "release-manifest.json"
        first.write_text(json.dumps(bundle_manifest, indent=1), encoding="utf-8")
        bundle = self.out / "bundle.tar"
        if bundle.exists():
            bundle.unlink()
        _run([WINDOWS_PYTHON, "Firmware/tools/adr0022_baseline_bundle.py", "pack", "--build", _relative(XBUILD),
              "--manifest", _relative(first), "--session-label", label, "--output", _relative(bundle)],
             stdout=subprocess.DEVNULL)
        self.log("  deploying one acquisition release for this commissioning run")
        done = _run([WINDOWS_PYTHON, "Firmware/tools/deploy_station.py", "--baseline-bundle", _relative(bundle)],
                    capture_output=True, text=True, check=False)
        (self.out / "deploy.log").write_text(done.stdout + done.stderr, encoding="utf-8")
        found = re.search(r"Acquisition release prepared; devices unopened: (\S+)", done.stdout)
        if done.returncode or not found:
            raise RuntimeError("deploy failed; see " + str(self.out / "deploy.log"))
        self.release = found.group(1)
        self.log(f"  release {self.release}")

    def run(self, axis, manifest, label):
        """Run one session; returns the parsed journal (journal.yaw / journal.pitch)."""
        if self.release is None:
            self._prepare(manifest, label)
        option, name = LAUNCH[axis]
        remote = f"{self.release}/run/servo/{label}"
        local = self.out / label
        local.mkdir(parents=True, exist_ok=True)
        bound = dict(manifest, session_label=label)
        bound["output"] = f"{remote}/{name}"
        (local / "manifest.json").write_text(json.dumps(bound, indent=1), encoding="utf-8")
        q = shlex.quote
        self._ssh(f"mkdir -p {q(remote)}")
        _run(["scp.exe", "-q", _relative(local / "manifest.json"), f"{HOST}:{remote}/manifest.json"])
        script = f"{self.release}/Firmware/scripts/run_application.sh"
        done = self._ssh(f"OTA_RUN_DIR={q(self.release + '/run/stack')} bash {q(script)} run {option} {q(remote + '/manifest.json')}",
                         capture_output=True, text=True, check=False)
        (local / "session.log").write_text(done.stdout + done.stderr, encoding="utf-8")
        footer = next((line for line in done.stdout.splitlines() if '"kind":"footer"' in line), None)
        fetched = _run(["scp.exe", "-q", f"{HOST}:{remote}/{name}", _relative(local / name)], check=False)
        if fetched.returncode:
            raise RuntimeError(f"{label}: no journal (session log: {local / 'session.log'})")
        j = (journal.yaw if axis == "yaw" else journal.pitch)(local / name)
        if footer and j.get("footer") is None:
            j["footer"] = json.loads(footer[footer.index("{"):])
        return j


class SimStation:
    """Answers sessions from a true plant (asset-format "plant" + pitch facts) with servo-sim."""

    def __init__(self, truth, log=print):
        self.truth = truth
        self.log = log
        self.yaw_position = truth.get("yaw_start_rad", 1.0)

    def run(self, axis, manifest, label):
        return self._yaw(manifest) if axis == "yaw" else self._pitch(manifest)

    def _yaw(self, m):
        request = {"axis": "yaw", "servo": m["servo_parameters"], "plant": self.truth["yaw"],
                   "reference": sim.reference_arrays(m["reference_samples"]), "start_position_rad": self.yaw_position,
                   "hold_after_s": m["servo_hold_after_s"], "speed_limit_rad_s": m["servo_speed_limit_rad_s"],
                   "oscillation_limit_A": m.get("servo_oscillation_limit_A", 0.0)}
        if m.get("servo_excitation"):
            request["excitation"] = m["servo_excitation"]
        if m.get("servo_gain_schedule"):
            request["gain_schedule"] = m["servo_gain_schedule"]
        r, status = sim.run(request)
        self.yaw_position = float(r["q_true"][-1])
        t = r["t"]
        return {"manifest": m, "footer": {"status": "COMPLETE" if status == "COMPLETE" else "INVALID", "detail": "" if status == "COMPLETE" else status},
                "learned": r["learned"],
                "feedback": {"t": t, "q": r["q_meas"], "current": r["current"], "temperature": np.full(len(t), 35.0)},
                # commissiond transmits ~0.2 ms after the encoder receipt that triggered the step
                "tx": {"t": t + 0.0002, "u": r["u"], "excitation": r["excitation"] != 0},
                "cycles": {"t": t, "qr": r["qr"], "vr": r["vr"], "ar": r["ar"], "q": r["q_hat"], "v": r["v_hat"], "u": r["u"] - r["excitation"],
                           "req": r["requested"], "ff": r["friction"], "fr": r["friction"], "i": r["integral"], "rock": r["rocking"],
                           "stalls": r["stalls"], "sat": r["saturated"], "rms": r["rms"], "cap": r["cap"]},
                "gains": [(g["begin_s"], g["kq"], g["kv"], g["ki"]) for g in m.get("servo_gain_schedule", [])],
                "truth": {"t": t, "q": r["q_true"]}}

    def _pitch(self, m):
        trial = m["servo_trial"]
        p = self.truth["pitch"]
        margin = trial["window_margin_rad"]
        if "window_min_rad" in trial:  # the window given by production's homing
            low, high = trial["window_min_rad"] - margin, trial["window_max_rad"] + margin
            centre = trial["center_rad"]
        else:
            low, high = p["endpoint_low_rad"], p["endpoint_high_rad"]
            centre = 0.5 * (low + high)
        loop = {k: trial[k] for k in ("kp_per_s", "ki_per_s2", "integral_clamp_rad_s", "speed_limit_rad_s", "command_period_s",
                                      "following_error_rad")}
        request = {"axis": "pitch", "loop": loop, "plant": p, "reference": sim.reference_arrays(trial["reference_samples"]),
                   "start_position_rad": centre, "hold_after_s": trial.get("hold_after_s", 1.0)}
        if trial.get("excitation"):
            request["excitation"] = trial["excitation"]
        if trial.get("gain_schedule"):
            request["gain_schedule"] = trial["gain_schedule"]
        r, status = sim.run(request)
        if not (r["q_true"].min() >= low + margin - 1e-9 and r["q_true"].max() <= high - margin + 1e-9):
            status = "HARD_ABORT: pitch outside servo trial window"
        t = r["t"]
        return {"manifest": m, "footer": {"status": "COMPLETE" if status == "COMPLETE" else "INVALID", "detail": "" if status == "COMPLETE" else status},
                "window": {"endpoint_low_rad": low, "endpoint_high_rad": high, "window_min_rad": low + margin,
                           "window_max_rad": high - margin, "center_rad": centre},
                "endpoints": {"endpoint_a_rad": high, "endpoint_b_rad": low},
                "trial": {"t": t, "qr": r["qr"], "vr": r["vr"], "q": r["q_meas"], "cmd": r["cmd"], "x": r["excitation"],
                          "i": r["integral"], "torque": np.zeros(len(t)), "status_t": t},
                "centre": centre,
                "gains": [(g["begin_s"], g["kp"], g["ki"]) for g in trial.get("gain_schedule", [])]}
