"""Session scripts and commissiond manifests for the yaw servo and the pitch speed-mode servo.

Every script is an ADR-003-shaped reference (usecase.level1) stored as a 50 Hz
table in the manifest (small enough for yaml-cpp on the Pi; the session
interpolates). Yaw references are relative to where control begins; pitch first
homes, then moves to the centre of its measured window.
"""
import json
import math
from pathlib import Path

import numpy as np

import usecase

DEG = math.pi / 180
HERE = Path(__file__).resolve().parent
AUTHORIZATION = "Owner granted full station authority on 2026-10-02 (speed below 100 RPM)"


# ------------------------------------------------------------------ yaw scripts
def survey():
    """Friction survey: slow plateaus (creep region) and full revolutions at speed, both ways."""
    S = [(2.0, "hold", 0, "hold")]
    for s in (0.25, 0.5, 1, 2):
        S += [(8.0, "ramp", s * DEG, f"slow+{s}"), (1.0, "hold", 0, "h"), (8.0, "ramp", -s * DEG, f"slow-{s}"), (1.0, "hold", 0, "h")]
    for s in (5, 10, 20, 40, 60):
        d = 360.0 / s if s >= 10 else 20.0
        S += [(d, "ramp", s * DEG, f"fast+{s}"), (1.5, "hold", 0, "h"), (d, "ramp", -s * DEG, f"fast-{s}"), (1.5, "hold", 0, "h")]
    return S


def crosstalk_scan(speed=10.0, span=375.0):
    """A turn each way at 10 deg/s under an 80 Hz probe tone (0.25 s windows span 2.5 deg)."""
    d = span / speed
    return [(2.0, "hold", 0, "hold"), (d, "ramp", speed * DEG, "scan+"), (1.0, "hold", 0, "turn"),
            (d, "ramp", -speed * DEG, "scan-"), (2.0, "hold", 0, "final")]


CROSSTALK_PROBE = {"amplitude_A": 0.15, "f0_hz": 80.0, "f1_hz": 80.0001}


def inertia_sweep(speed=30.0, duration=12.0):
    """Steady sliding each way while a current sweep is added: the rigid-body response."""
    return [(2.0, "hold", 0, "hold"), (duration, "ramp", speed * DEG, "sweep+"), (1.5, "hold", 0, "turn"),
            (duration, "ramp", -speed * DEG, "sweep-"), (2.0, "hold", 0, "final")]


INERTIA_SWEEP = {"amplitude_A": 0.3, "f0_hz": 8.0, "f1_hz": 30.0}


def ladder_step(tag):
    """One gain step: hold, small steps, slow and fast ramps each way. The limit cycle
    starts on slow turnarounds or while sliding at speed (friction no longer damps)."""
    return [(1.0, "hold", 0, f"hold@{tag}"), (1.0, "step", 0.3 * DEG, f"s+@{tag}"), (1.0, "step", -0.3 * DEG, f"s-@{tag}"),
            (1.5, "ramp", 5 * DEG, f"r5+@{tag}"), (0.5, "hold", 0, f"t5+@{tag}"), (2.0, "ramp", 20 * DEG, f"r20+@{tag}"),
            (1.0, "hold", 0, f"t20+@{tag}"), (2.0, "ramp", -20 * DEG, f"r20-@{tag}"), (0.5, "hold", 0, f"t20-@{tag}"),
            (1.5, "ramp", -5 * DEG, f"r5-@{tag}"), (1.0, "hold", 0, f"stop@{tag}")]


LADDER_STEP_S = 13.0


def ladder(moves, steps):
    """Gain ladder at several angles: `moves` are relative moves (rad) to each test angle,
    `steps` the number of gain steps there. Returns the script and each step's begin time."""
    S, begins, t = [], [], 0.0
    for move in moves:
        if move:
            dur = max(2.0, abs(move) / (20 * DEG) + 1.5)
            S.append((dur, "step", move, "move")); t += dur
        for k in range(steps):
            begins.append(t)
            S += ladder_step(f"{len(begins) - 1}"); t += LADDER_STEP_S
    S.append((1.0, "hold", 0, "final"))
    return S, begins


def circle():
    """A stop every 45 degrees around the full circle in both directions (angle-dependent stability)."""
    out = []
    for sign in (1, -1):
        for i in range(8):
            out += [(4.5, "ramp", sign * 10 * DEG, f"go{sign:+d}@{i}"), (1.5, "hold", 0, f"hold{sign:+d}@{i}"),
                    (1.0, "step", 0.3 * DEG, f"s+{sign:+d}@{i}"), (1.0, "step", -0.3 * DEG, f"s-{sign:+d}@{i}")]
    return out


def probe():
    """A short check of the main behaviours (about 60 s)."""
    S = [(2.0, "hold", 0, "hold")]
    for d in (0.5, 2.0):
        S += [(2.0, "step", d * DEG, f"step+{d}"), (2.0, "step", -d * DEG, f"step-{d}")]
    for s in (2, 5, 20):
        S += [(4.0, "ramp", s * DEG, f"ramp+{s}"), (1.5, "hold", 0, f"stop+{s}"),
              (4.0, "ramp", -s * DEG, f"ramp-{s}"), (1.5, "hold", 0, f"stop-{s}")]
    return S + [(5.0, "step", 45 * DEG, "long+45"), (5.0, "step", -45 * DEG, "long-45"), (2.0, "hold", 0, "final")]


# ---------------------------------------------------------------- pitch scripts
def pitch_usecase():
    S = [(2.0, "hold", 0, "hold")]
    for d in (0.5, 2.0):
        S += [(2.0, "step", d * DEG, f"step+{d}"), (2.0, "step", -d * DEG, f"step-{d}")]
    for s, dur in ((2, 4.0), (5, 3.0), (10, 2.0), (20, 1.0)):
        S += [(dur, "ramp", s * DEG, f"ramp+{s}"), (1.5, "hold", 0, f"stop+{s}"),
              (dur, "ramp", -s * DEG, f"ramp-{s}"), (1.5, "hold", 0, f"stop-{s}")]
    return S + [(12.0, "sine", ((8 * DEG, 2 * DEG), (0.5, 1.7)), "walker"), (2.0, "hold", 0, "final")]


def pitch_identify():
    """Hold under a speed-reference sweep (the drive's speed loop), then small steps."""
    S = [(2.0, "hold", 0, "hold"), (24.0, "hold", 0, "sweep")]
    for d in (1.0, 2.0):
        S += [(2.0, "step", d * DEG, f"step+{d}"), (2.0, "step", -d * DEG, f"step-{d}")]
    return S + [(1.0, "hold", 0, "final")]


PITCH_SWEEP = {"amplitude_rad_s": 0.06, "f0_hz": 0.5, "f1_hz": 25.0, "begin_s": 2.5, "duration_s": 23.0}


def pitch_ladder_step(tag):
    return [(1.0, "hold", 0, f"hold@{tag}"), (1.5, "step", 1.0 * DEG, f"s+@{tag}"), (1.5, "step", -1.0 * DEG, f"s-@{tag}"),
            (1.5, "ramp", 5 * DEG, f"r+@{tag}"), (0.5, "hold", 0, f"t@{tag}"), (1.5, "ramp", -5 * DEG, f"r-@{tag}"),
            (0.5, "hold", 0, f"stop@{tag}")]


PITCH_LADDER_STEP_S = 8.0


def pitch_ladder(steps):
    S, begins = [], []
    for k in range(steps):
        begins.append(k * PITCH_LADDER_STEP_S)
        S += pitch_ladder_step(str(k))
    return S + [(1.0, "hold", 0, "final")], begins


def speed_sweep():
    """Full turns each way at rising speed with the commissioned servo: where it stays quiet at the
    speeds production tracking asks for (2026-10-02: a 15 Hz limit cycle at ~1 rad/s turning positive
    through motor angles 85-101 deg, twice, which the identified model does not reproduce).
    Run with --opts '{"vmax": 70}': the session reference is otherwise capped at 20 deg/s."""
    S = [(2.0, "hold", 0, "hold")]
    for s in (20, 30, 40, 50, 60):
        d = 360.0 / s + 1.0
        S += [(d, "ramp", s * DEG, f"turn+{s}"), (1.5, "hold", 0, "h"), (d, "ramp", -s * DEG, f"turn-{s}"), (1.5, "hold", 0, "h")]
    return S


SCRIPTS = {"survey": survey, "speed_sweep": speed_sweep, "crosstalk": crosstalk_scan, "inertia": inertia_sweep, "circle": circle, "probe": probe,
           "usecase": usecase.script, "pitch_usecase": pitch_usecase, "pitch_identify": pitch_identify,
           "hold": lambda: [(24.0, "hold", 0, "hold")]}


def samples(script, vmax=20.0, amax=60.0, jmax=300.0, lam=4.0):
    """Script -> (50 Hz reference table, segment labels, 1 kHz arrays for scoring)."""
    S = SCRIPTS[script]() if isinstance(script, str) else script
    t, q, v, a, segs, _ = usecase.level1(S, dt=0.001, lam=lam, vmax=vmax * DEG, amax=amax * DEG, jmax=jmax * DEG)
    idx = np.arange(0, len(t), 20)
    table = [dict(time_s=round(float(t[i] - t[0]), 4), position_rad=round(float(q[i]), 7),
                  velocity_rad_s=round(float(v[i]), 6), acceleration_rad_s2=round(float(a[i]), 5)) for i in idx]
    labels = [dict(begin_s=s[0], end_s=s[1], kind=s[2], value=s[3], label=s[4]) for s in segs]
    return table, labels


# -------------------------------------------------------------------- manifests
def yaw(label, servo, script, vmax=20.0, speed_limit=1.75, hold_after=1.5, gain_schedule=None, excitation=None,
        oscillation_limit=0.3, temperature_limit=55.0):
    with open(HERE / "templates" / "yaw_session.json", encoding="utf-8") as f:
        m = json.load(f)
    table, labels = samples(script, vmax=vmax)
    m.update(session_label=label, candidate_label=label, servo_parameters=servo, reference_samples=table,
             reference_segments_labels=labels, servo_speed_limit_rad_s=speed_limit, servo_hold_after_s=hold_after,
             servo_motor_temperature_limit_C=temperature_limit, yaw_current_bound_A=servo["current_cap"],
             servo_oscillation_limit_A=oscillation_limit,
             source_description="Yaw position servo (tools/servo_commission), event-driven at 1 kHz",
             session_authorization={"purpose": "yaw_servo_commissioning", "yaw_acquisition_authorized": True,
                                    "authorization_identity": AUTHORIZATION, "unattended_operation_authorized": True,
                                    "presence_required": False},
             controlled_stop_scope={"mode": "servo holds the final reference for servo_hold_after_s, then zero current",
                                    "zero_observation_s": 2.0})
    if gain_schedule:
        m["servo_gain_schedule"] = gain_schedule
    if excitation:
        m["servo_excitation"] = excitation
    total = table[-1]["time_s"]
    m["limits"]["duration_s"] = int(m["limits"]["startup_s"] + m["baseline_s"] + total + hold_after + m["stop_observation_s"] + 30)
    return m


def pitch(label, native_settings, trial, script, gain_schedule=None, excitation=None):
    with open(HERE / "templates" / "pitch_session.json", encoding="utf-8") as f:
        m = json.load(f)
    table, labels = samples(script)
    m["session_label"] = label
    m["native_settings"].update(native_settings)
    trial = dict(trial)
    trial["reference_samples"] = table
    if gain_schedule:
        trial["gain_schedule"] = gain_schedule
    if excitation:
        trial["excitation"] = excitation
    m["servo_trial"] = trial
    m["reference_segments_labels"] = labels
    m["operator_attendance"] = {"present_at_manual_cutoff": False, "operator_identity": "owner",
                                "manual_cutoff_evidence_identity": "unattended operation authorized 2026-10-02"}
    m["session_authorization"] = {"purpose": "pitch_sensorless_homing", "sensorless_homing_authorized": True,
                                  "authorization_identity": AUTHORIZATION, "unattended_operation_authorized": True,
                                  "presence_required": False}
    homing = 180 if trial.get("home_first") else 0  # full-range sensorless homing at 3-5 deg/s
    m["limits"]["duration_s"] = int(20 + 25 + homing + table[-1]["time_s"] + trial.get("hold_after_s", 1) + 40)
    return m
