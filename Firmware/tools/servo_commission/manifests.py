"""Build commissiond manifests for the yaw servo and the pitch speed-mode servo trial.

    python manifests.py yaw   LABEL OUT.json [--script usecase] [--asset ../../config/servo/yaw_servo.json] [--opts JSON]
    python manifests.py pitch LABEL OUT.json [--script pitch] [--asset ../../config/servo/pitch_servo.json] [--opts JSON]

Scripts are ADR-003-shaped references (usecase.level1). Yaw references are relative
to the position where control begins; pitch first moves to its absolute centre.
Only manifests are produced; packing/deploying is done by run.sh.
"""
import argparse, json, math
from pathlib import Path
import numpy as np
import usecase

DEG = math.pi / 180
HERE = Path(__file__).resolve().parent
CONFIG = HERE.parents[1] / "config" / "servo"


def probe():
    S = [(2.0, "hold", 0, "hold")]
    for d in (0.5, 2.0):
        S += [(2.0, "step", d * DEG, f"step+{d}"), (2.0, "step", -d * DEG, f"step-{d}")]
    for s in (5, 20):
        S += [(4.0, "ramp", s * DEG, f"ramp+{s}"), (2.0, "hold", 0, f"stop+{s}"),
              (4.0, "ramp", -s * DEG, f"ramp-{s}"), (2.0, "hold", 0, f"stop-{s}")]
    return S + [(5.0, "step", 45 * DEG, "long+45"), (5.0, "step", -45 * DEG, "long-45"), (2.0, "hold", 0, "final")]


def ladder(n=8):
    out = []
    for i in range(n):
        tag = f"@{i}"
        out += [(1.0, "hold", 0, f"hold{tag}"), (1.5, "step", 0.5 * DEG, f"step+{tag}"), (1.5, "step", -0.5 * DEG, f"step-{tag}"),
                (2.0, "ramp", 5 * DEG, f"ramp+5{tag}"), (1.0, "hold", 0, f"stop+{tag}"), (2.0, "ramp", -5 * DEG, f"ramp-5{tag}"),
                (1.0, "hold", 0, f"stop-{tag}")]
    return out


def circle():
    """A stop every 45 degrees around the full circle in both directions (angle-dependent stability)."""
    out = []
    for sign in (1, -1):
        for i in range(8):
            out += [(4.5, "ramp", sign * 10 * DEG, f"go{sign:+d}@{i}"), (1.5, "hold", 0, f"hold{sign:+d}@{i}"),
                    (1.0, "step", 0.3 * DEG, f"s+{sign:+d}@{i}"), (1.0, "step", -0.3 * DEG, f"s-{sign:+d}@{i}")]
    return out


def crosstalk_scan():
    """Warm-up rotation, then 375 degrees at 3 deg/s for the 80 Hz probe-tone crosstalk scan."""
    return [(2.0, "hold", 0, "hold"), (30.0, "ramp", 30 * DEG, "warm+"), (1.0, "hold", 0, "warm-stop"),
            (30.0, "ramp", -30 * DEG, "warm-"), (2.0, "hold", 0, "warm-stop2"), (125.0, "ramp", 3 * DEG, "scan"),
            (2.0, "hold", 0, "final")]


def pitch():
    S = [(2.0, "hold", 0, "hold")]
    for d in (0.5, 2.0):
        S += [(2.0, "step", d * DEG, f"step+{d}"), (2.0, "step", -d * DEG, f"step-{d}")]
    for s, dur in ((2, 4.0), (5, 3.0), (10, 2.0), (20, 1.0)):
        S += [(dur, "ramp", s * DEG, f"ramp+{s}"), (1.5, "hold", 0, f"stop+{s}"),
              (dur, "ramp", -s * DEG, f"ramp-{s}"), (1.5, "hold", 0, f"stop-{s}")]
    return S + [(12.0, "sine", ((8 * DEG, 2 * DEG), (0.5, 1.7)), "walker"), (2.0, "hold", 0, "final")]


SCRIPTS = {"probe": probe, "usecase": usecase.script, "ladder": ladder, "circle": circle,
           "crosstalk_scan": crosstalk_scan, "pitch": pitch,
           "hold": lambda: [(24.0, "hold", 0, "hold")]}


def samples(script, vmax=20, amax=60, jmax=300, lam=4.0):
    t, q, v, a, segs, _ = usecase.level1(SCRIPTS[script](), dt=0.001, lam=lam, vmax=vmax * DEG, amax=amax * DEG, jmax=jmax * DEG)
    idx = np.arange(0, len(t), 20)  # 50 Hz table: small enough for yaml-cpp on the Pi, ~0.003 deg interpolation error
    table = [dict(time_s=round(float(t[i] - t[0]), 4), position_rad=round(float(q[i]), 7),
                  velocity_rad_s=round(float(v[i]), 6), acceleration_rad_s2=round(float(a[i]), 5)) for i in idx]
    labels = [dict(begin_s=s[0], end_s=s[1], kind=s[2], value=s[3], label=s[4]) for s in segs]
    return table, labels


AUTHORIZATION = "Owner granted full station authority on 2026-10-02 (speed below 100 RPM)"


def yaw(label, asset, script="usecase", speed_limit=1.75, hold_after=1.5, gain_schedule=None, excitation=None, vmax=20):
    m = json.load(open(HERE / "templates" / "yaw_session.json", encoding="utf-8"))
    servo = json.load(open(asset, encoding="utf-8"))["servo_parameters"]
    table, labels = samples(script, vmax=vmax)
    m.update(session_label=label, candidate_label=label, servo_parameters=servo, reference_samples=table,
             reference_segments_labels=labels, servo_speed_limit_rad_s=speed_limit, servo_hold_after_s=hold_after,
             servo_event_control=True, servo_motor_temperature_limit_C=55.0, yaw_current_bound_A=servo["current_cap"],
             yaw_position_offset_rad=0.0,
             source_description="Position servo (PID + reference friction/inertia FF + encoder crosstalk compensation), 1 kHz event-driven",
             session_authorization={"purpose": "yaw_servo_commissioning", "yaw_acquisition_authorized": True,
                                    "authorization_identity": AUTHORIZATION, "unattended_operation_authorized": True,
                                    "presence_required": False},
             controlled_stop_scope={"mode": "servo holds the final reference for servo_hold_after_s, then zero current",
                                    "zero_observation_s": 2.0})
    if gain_schedule: m["servo_gain_schedule"] = gain_schedule
    if excitation: m["servo_excitation"] = excitation
    total = table[-1]["time_s"]
    m["limits"]["duration_s"] = int(m["limits"]["startup_s"] + m["baseline_s"] + total + hold_after + m["stop_observation_s"] + 30)
    return m


def pitch_trial(label, asset, script="pitch"):
    m = json.load(open(HERE / "templates" / "pitch_session.json", encoding="utf-8"))
    a = json.load(open(asset, encoding="utf-8"))
    table, labels = samples(script)
    m["session_label"] = label
    m["native_settings"].update(a["native_settings"])
    trial = dict(a["servo_trial"]); trial["reference_samples"] = table
    m["servo_trial"] = trial; m["reference_segments_labels"] = labels
    m["operator_attendance"] = {"present_at_manual_cutoff": False, "operator_identity": "owner",
                                "manual_cutoff_evidence_identity": "unattended operation authorized 2026-10-02"}
    m["session_authorization"] = {"purpose": "pitch_sensorless_homing", "sensorless_homing_authorized": True,
                                  "authorization_identity": AUTHORIZATION, "unattended_operation_authorized": True,
                                  "presence_required": False}
    m["limits"]["duration_s"] = int(20 + 25 + table[-1]["time_s"] + trial.get("hold_after_s", 1) + 40)
    return m


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("axis", choices=("yaw", "pitch")); p.add_argument("label"); p.add_argument("out")
    p.add_argument("--script"); p.add_argument("--asset"); p.add_argument("--opts", default="{}")
    a = p.parse_args()
    opts = json.loads(a.opts)
    if a.axis == "yaw":
        m = yaw(a.label, a.asset or CONFIG / "yaw_servo.json", a.script or "usecase", **opts)
        n = len(m["reference_samples"]); end = m["reference_samples"][-1]["time_s"]
    else:
        m = pitch_trial(a.label, a.asset or CONFIG / "pitch_servo.json", a.script or "pitch")
        n = len(m["servo_trial"]["reference_samples"]); end = m["servo_trial"]["reference_samples"][-1]["time_s"]
    json.dump(m, open(a.out, "w", encoding="utf-8", newline="\n"), indent=1, ensure_ascii=False)
    print(f"{a.out}: {n} reference samples, {end:.1f} s")
