"""Offline synthetic closed-loop estimator falsification; never operates the station.

The actual shared C++ Controller receives noisy, sampled measurements and acknowledges
bounded successful current commands. A separate piecewise analytic oracle generates
motion; no commissioning/model_family integrator generates the observations.
All fixture parameters and limits here are SYNTHETIC, never calibration or gains.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.contracts import ModelSpec
from Firmware.commissioning.measurement import ObserverSpec
from Firmware.commissioning.native import CObservation, CReference, Controller, Native, parameters


FIXTURE = {
    "provenance": "SYNTHETIC",
    "a": 0.10, "viscous": 0.06,
    "coulomb_negative": 0.12, "coulomb_positive": 0.12,
    "static_negative": 0.16, "static_positive": 0.16,
    "load_offset": 0.02, "transport_delay": 0.008,
    "gyro_tau": 0.015, "gyro_delay": 0.004,
    "encoder_quantum": 2 * math.pi / 8192,
    "encoder_noise": 0.00015, "gyro_noise": 0.005, "current_noise": 0.002,
    "oracle_dt": 0.001, "control_dt": 0.005, "gyro_dt": 0.020,
}
LIMITS = {
    "kp": 0.6, "ki": 0.5, "kpos": 3.0, "kaw": 3.0,
    "current_cap": 0.35, "slew": 2.0, "integral_cap": 0.3,
    "velocity_cap": 0.8, "dt_min": 0.004, "dt_max": 0.006,
    "intent_threshold": 0.015, "rest_speed": 0.020,
    "sustained_s": 0.060, "start_timeout_s": 2.0,
}
SEEDS = (17, 41, 83)
NOISE_SOURCES = ("encoder_noise", "encoder_quantization", "gyro_noise", "current_noise")
FREE_BOUNDS = {"a": (0.06, 0.15), "viscous": (0.015, 0.12),
               "coulomb_negative": (0.07, 0.15), "coulomb_positive": (0.07, 0.15)}
RECOVERY_GATES = {"pristine_relative": 0.02, "noisy_relative": 0.05,
                  "sensor_sigma_multiple": 3.0, "numerical_q_rms_rad": 1e-5,
                  "numerical_gyro_rms_rad_s": 1e-4}


def save(path: Path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def moving_interval(q, v, gyro, u, duration, direction):
    """Exact solution for constant current, viscous/Coulomb load and gyro filter."""
    a, b, tau = FIXTURE["a"], FIXTURE["viscous"], FIXTURE["gyro_tau"]
    fc = FIXTURE["coulomb_positive" if direction > 0 else "coulomb_negative"]
    rate = b / a
    equilibrium = (u - FIXTURE["load_offset"] - direction * fc) / b
    ev = math.exp(-rate * duration)
    eg = math.exp(-duration / tau)
    difference = v - equilibrium
    new_v = equilibrium + difference * ev
    new_q = q + equilibrium * duration + difference * (-math.expm1(-rate * duration)) / rate
    new_gyro = equilibrium + (gyro - equilibrium) * eg + difference * (ev - eg) / (1 - rate * tau)
    return new_q, new_v, new_gyro


def analytic_step(q, v, gyro, u, duration):
    """Resolve exact zero-speed crossings and the directional static holding interval."""
    remaining = duration
    while remaining > 1e-13:
        net = u - FIXTURE["load_offset"]
        if abs(v) < 1e-12 and -FIXTURE["static_negative"] <= net <= FIXTURE["static_positive"]:
            return q, 0.0, gyro * math.exp(-remaining / FIXTURE["gyro_tau"]), True
        direction = (1 if v > 0 else -1) if abs(v) >= 1e-12 else (1 if net > 0 else -1)
        fc = FIXTURE["coulomb_positive" if direction > 0 else "coulomb_negative"]
        equilibrium = (net - direction * fc) / FIXTURE["viscous"]
        crossing = math.inf
        if abs(v) >= 1e-12 and v * equilibrium < 0:
            crossing = -math.log(-equilibrium / (v - equilibrium)) / (FIXTURE["viscous"] / FIXTURE["a"])
        step = min(remaining, crossing)
        q, v, gyro = moving_interval(q, v, gyro, u, step, direction)
        remaining -= step
        if crossing <= step + 1e-13:
            v = 0.0
    return q, v, gyro, abs(v) < 1e-12


def reference(t):
    if t <= 0.5:
        return np.zeros(4)
    x = t - 0.5
    # Diverse frequencies and directions; the reference is fixed before noise generation.
    frequencies = 2 * math.pi * np.array([0.18, 0.55, 1.1])
    amplitudes = np.array([0.30, 0.07, 0.015])
    q = float(amplitudes @ np.sin(frequencies * x))
    v = float((amplitudes * frequencies) @ np.cos(frequencies * x))
    acceleration = float((-amplitudes * frequencies**2) @ np.sin(frequencies * x))
    return np.array([q, v, acceleration, 0.0])


def controller_parameters():
    spec = ModelSpec("yaw", (-2.0, -1.0, 0.0, 1.0, 2.0), (-0.3, 0.0, 0.3))
    h = np.empty((2, 3, 5))
    h[0] = FIXTURE["load_offset"] - FIXTURE["coulomb_negative"]
    h[1] = FIXTURE["load_offset"] + FIXTURE["coulomb_positive"]
    theta = np.r_[np.full(3, FIXTURE["a"]), np.full(3, FIXTURE["viscous"]), h.ravel(), FIXTURE["transport_delay"]]
    starts = np.empty_like(h)
    starts[0] = FIXTURE["load_offset"] - FIXTURE["static_negative"]
    starts[1] = FIXTURE["load_offset"] + FIXTURE["static_positive"]
    # Fixed declared synthetic covariance; no hashes/identities are computed.
    observer = ObserverSpec(2e-7, 2.5e-5, 0.5, 0.02, 0.060,
                            2e-7, 2.5e-5, False, "synthetic-fixture-no-digest", "SYNTHETIC")
    return parameters(spec, theta, observer, LIMITS, starts, np.zeros_like(starts, dtype=bool))


def generate(native, duration, seed, noisy, *, reference_function=reference, noise_sources=None):
    enabled = set(NOISE_SOURCES if noisy else ()) if noise_sources is None else set(noise_sources)
    if not enabled <= set(NOISE_SOURCES) or (enabled and not noisy):
        raise ValueError("Unknown noise source, or noise enabled on a pristine case")
    dt = FIXTURE["oracle_dt"]
    n = int(round(duration / dt)) + 1
    t = np.arange(n) * dt
    rng = np.random.default_rng(seed)
    truth = np.zeros((n, 5))  # q, v, effective current, gyro filter, stick
    truth[:, 4] = 1
    measured_q, measured_v, measured_current = np.zeros(n), np.zeros(n), np.zeros(n)
    q_new = np.ones(n, dtype=bool)
    v_new = np.arange(n) % 20 == 0
    current_new = np.ones(n, dtype=bool)
    tx_t, tx_A = [-0.10], [0.0]
    command_rows = []
    command_index = 0
    q = v = gyro = 0.0
    gyro_index = 0
    with Controller(native, controller_parameters()) as controller:
        controller.reset(0.0, 0.0, 0.0, 0.0, accepted_time=-0.10)
        for k in range(n):
            now = float(t[k])
            if k:
                # Accepted events and the fixed 8 ms delay align with this 1 ms grid.
                while command_index + 1 < len(tx_t) and tx_t[command_index + 1] + FIXTURE["transport_delay"] <= t[k - 1] + 1e-12:
                    command_index += 1
                q, v, gyro, stick = analytic_step(q, v, gyro, tx_A[command_index], dt)
            else:
                stick = True
            while command_index + 1 < len(tx_t) and tx_t[command_index + 1] + FIXTURE["transport_delay"] <= now + 1e-12:
                command_index += 1
            effective = tx_A[command_index]
            truth[k] = q, v, effective, gyro, int(stick)
            # Consume the same draws in every noisy ablation to preserve the
            # original random realization and isolate each enabled source.
            q_noise = rng.normal(0, FIXTURE["encoder_noise"]) if noisy else 0
            measured_q[k] = q + (q_noise if "encoder_noise" in enabled else 0)
            if "encoder_quantization" in enabled:
                measured_q[k] = np.round(measured_q[k] / FIXTURE["encoder_quantum"]) * FIXTURE["encoder_quantum"]
            current_noise = rng.normal(0, FIXTURE["current_noise"]) if noisy else 0
            measured_current[k] = effective + (current_noise if "current_noise" in enabled else 0)
            if v_new[k]:
                gyro_index = k
                delayed = max(0, k - int(round(FIXTURE["gyro_delay"] / dt)))
                gyro_noise = rng.normal(0, FIXTURE["gyro_noise"]) if noisy else 0
                measured_v[k] = truth[delayed, 3] + (gyro_noise if "gyro_noise" in enabled else 0)
            else:
                measured_v[k] = measured_v[gyro_index]
            if k and k % 5 == 0:
                observation = CObservation(now, now, float(t[gyro_index]), float(measured_q[k]), float(measured_v[k]),
                    k + 1, gyro_index + 1, 1, True, True)
                ref = reference_function(now)
                output = controller.step(observation, CReference(*ref))
                if output.status not in (0, 3):
                    raise RuntimeError(f"Synthetic native controller rejected seed={seed} noisy={noisy} t={now}: status={output.status}")
                if not controller.ack(output, accepted_time=now):
                    raise RuntimeError("Synthetic successful current acknowledgement rejected")
                tx_t.append(now)
                tx_A.append(float(output.limited))
                command_rows.append([now, *ref, output.requested, output.limited, output.position,
                                     output.velocity, output.motion, output.status])
    return {"t": t, "q": measured_q, "v": measured_v, "current": measured_current,
            "q_new": q_new, "v_new": v_new, "current_new": current_new,
            "tx_t": np.asarray(tx_t), "tx_A": np.asarray(tx_A),
            "truth": truth, "commands": np.asarray(command_rows), "seed": seed, "noisy": noisy,
            "noise_sources": np.asarray(sorted(enabled), dtype="U32")}


def fixture_report(data):
    commands = data["commands"]
    clipped = np.minimum(np.maximum(commands[:, 5], -LIMITS["current_cap"]), LIMITS["current_cap"])
    slew = np.abs(np.diff(data["tx_A"][1:])) / FIXTURE["control_dt"]
    return {
        "seed": data["seed"], "noisy": data["noisy"],
        "encoder_samples": int(data["q_new"].sum()), "gyro_samples": int(data["v_new"].sum()),
        "current_samples": int(data["current_new"].sum()), "successful_tx_events_including_prehistory": len(data["tx_t"]),
        "native_control_cycles": len(commands),
        "saturation_requested_cycles": int(np.sum(np.abs(commands[:, 5]) > LIMITS["current_cap"])),
        "slew_limited_cycles": int(np.sum(np.abs(clipped - commands[:, 6]) > 1e-10)),
        "max_successful_current_A": float(np.max(np.abs(data["tx_A"]))),
        "max_successful_slew_A_s": float(slew.max()),
        "position_domain_rad": [float(data["truth"][:, 0].min()), float(data["truth"][:, 0].max())],
        "velocity_domain_rad_s": [float(data["truth"][:, 1].min()), float(data["truth"][:, 1].max())],
        "oracle_sticking_samples": int(data["truth"][:, 4].sum()),
        "current_bound_respected": bool(np.max(np.abs(data["tx_A"])) <= LIMITS["current_cap"] + 1e-12),
        "slew_bound_respected": bool(slew.max() <= LIMITS["slew"] + 1e-10),
    }


def estimator_model():
    from Firmware.commissioning.model_family import FamilyModel
    return FamilyModel(a=FIXTURE["a"], viscous=FIXTURE["viscous"],
        coulomb_negative=FIXTURE["coulomb_negative"], coulomb_positive=FIXTURE["coulomb_positive"],
        static_negative=FIXTURE["static_negative"], static_positive=FIXTURE["static_positive"],
        load_offset=FIXTURE["load_offset"], q_min=-2.0, q_max=2.0,
        actuator_gain=1.0, actuator_bias=0.0, transport_delay=FIXTURE["transport_delay"],
        gyro_bias=0.0, gyro_tau=FIXTURE["gyro_tau"], gyro_delay=FIXTURE["gyro_delay"],
        current_gain=1.0, current_bias=0.0, current_tau=0.0, current_delay=0.0, max_step=0.00025)


def sensor_scales(data):
    enabled = set(data.get("noise_sources", NOISE_SOURCES if data["noisy"] else ()))
    q_variance = (FIXTURE["encoder_noise"]**2 if "encoder_noise" in enabled else 0) + \
                 (FIXTURE["encoder_quantum"]**2 / 12 if "encoder_quantization" in enabled else 0)
    return {"sigma_q": math.sqrt(q_variance) if q_variance else 1e-5,
            "sigma_v": FIXTURE["gyro_noise"] if "gyro_noise" in enabled else 1e-4,
            "sigma_current": FIXTURE["current_noise"] if "current_noise" in enabled else 1e-4}


def measurement_errors(data, prediction):
    results = {}
    for channel, index, mask in (("q", 0, "q_new"), ("v", 3, "v_new"), ("current", 4, "current_new")):
        residual = data[channel][data[mask]] - prediction[data[mask], index]
        results[channel] = {"rms": float(np.sqrt(np.mean(residual**2))),
            "mean": float(residual.mean()), "max_abs": float(np.abs(residual).max()),
            "native_samples": int(data[mask].sum())}
    return results


def trajectory_gate(data, errors):
    sigma = sensor_scales(data)
    thresholds = {"q": max(3 * sigma["sigma_q"], RECOVERY_GATES["numerical_q_rms_rad"]),
                  "v": max(3 * sigma["sigma_v"], RECOVERY_GATES["numerical_gyro_rms_rad_s"]),
                  "current": 3 * sigma["sigma_current"]}
    return {"thresholds": thresholds, "passed": all(errors[k]["rms"] <= bound for k, bound in thresholds.items())}


def numerical_oracle_errors(data, prediction):
    """Compare against independent latent truth, excluding measurement noise."""
    gyro_truth = np.interp(data["t"] - FIXTURE["gyro_delay"], data["t"], data["truth"][:, 3])
    return {
        "q_rms_rad": float(np.sqrt(np.mean((prediction[data["q_new"], 0] - data["truth"][data["q_new"], 0])**2))),
        "gyro_rms_rad_s": float(np.sqrt(np.mean((prediction[data["v_new"], 3] - gyro_truth[data["v_new"]])**2))),
        "current_rms_A": float(np.sqrt(np.mean((prediction[data["current_new"], 4] - data["truth"][data["current_new"], 2])**2))),
    }


def case_gates(data, optimizer, relative, recovery_limit, at_bounds, predictions, oracle):
    """Keep independent predicates distinct; unperformed physical gates stay NOT_RUN."""
    finite = all(np.isfinite(data[k]).all() for k in ("t", "q", "v", "current", "tx_t", "tx_A", "truth"))
    integrity = (finite and np.all(np.diff(data["t"]) > 0) and
                 np.all(np.diff(data["tx_t"]) >= 0) and data["tx_t"][0] <= data["t"][0] and
                 all(data[k].shape == data["t"].shape and data[k].dtype == np.bool_
                     for k in ("q_new", "v_new", "current_new")))
    numerical = (oracle["q_rms_rad"] <= RECOVERY_GATES["numerical_q_rms_rad"] and
                 oracle["gyro_rms_rad_s"] <= RECOVERY_GATES["numerical_gyro_rms_rad_s"] and
                 oracle["current_rms_A"] <= 1e-9)
    return {
        "data_integrity": "PASS" if integrity else "FAIL",
        "forward_numerics": "PASS" if numerical else "FAIL",
        "optimizer_termination_reason": optimizer["message"],
        "optimizer_converged": bool(optimizer["success"]),
        "synthetic_parameter_recovery": "PASS" if not at_bounds and all(abs(e) <= recovery_limit for e in relative.values()) else "FAIL",
        "training_trajectory": "PASS" if all(p["trajectory_gate"]["passed"] for p in predictions if p["role"] == "training_whole_run") else "FAIL",
        "selection_trajectory": "PASS" if all(p["trajectory_gate"]["passed"] for p in predictions if p["role"] != "training_whole_run") else "FAIL",
        "historical_regression": "NOT_RUN",
        "prospective_prediction": "NOT_RUN",
        "physical_stage3a": "NOT_RUN",
        "physical_stage3b": "NOT_RUN",
        "deployment_authorized": False,
    }


def fit_cases(library, output_dir, datasets, max_nfev):
    from Firmware.commissioning.model_family import FamilyNative, FamilyRun, fit_family
    family_native = FamilyNative(library)
    true_model = estimator_model()
    initial_model = replace(true_model, a=0.085, viscous=0.075,
                            coulomb_negative=0.105, coulomb_positive=0.13)
    all_cases = []
    for label, data in datasets.items():
        sigma = sensor_scales(data)
        run = FamilyRun(run_id=label, source_id=f"independent-analytic-oracle/{label}",
            t=data["t"], q=data["q"], v=data["v"], current=data["current"],
            q_new=data["q_new"], v_new=data["v_new"], current_new=data["current_new"],
            tx_t=data["tx_t"], tx_A=data["tx_A"], initial=np.zeros(5),
            provenance="SYNTHETIC", configuration_id="declared-coulomb-analytic-fixture",
            calibration_revision="known-synthetic-sensors",
            encoder_quantum=FIXTURE["encoder_quantum"] if "encoder_quantization" in
                set(data.get("noise_sources", NOISE_SOURCES if data["noisy"] else ())) else 0.0, **sigma)
        oracle_prediction = family_native.rollout(true_model, data["t"], data["tx_t"], data["tx_A"], np.zeros(5))
        oracle_errors = measurement_errors(data, oracle_prediction)
        oracle_numerics = numerical_oracle_errors(data, oracle_prediction)
        fit = fit_family(family_native, initial_model, [run], bounds=FREE_BOUNDS, max_nfev=max_nfev)
        save(output_dir / f"{label}-fit.json", {**fit, "model": fit["model"].document()})
        model = fit["model"]
        relative = {key: (getattr(model, key) - FIXTURE[key]) / FIXTURE[key] for key in FREE_BOUNDS}
        recovery_limit = RECOVERY_GATES["noisy_relative" if data["noisy"] else "pristine_relative"]
        at_bounds = [key for key, (lo, hi) in FREE_BOUNDS.items()
                     if min(abs(getattr(model, key) - lo), abs(getattr(model, key) - hi)) <= 1e-6 * (hi - lo)]
        predictions = []
        for predicted_label, predicted_data in datasets.items():
            if predicted_data["noisy"] != data["noisy"]:
                continue
            prediction = family_native.rollout(model, predicted_data["t"], predicted_data["tx_t"],
                                               predicted_data["tx_A"], np.zeros(5))
            np.savez_compressed(output_dir / f"{label}-predict-{predicted_label}.npz",
                t=predicted_data["t"], prediction=prediction,
                q_new=predicted_data["q_new"], v_new=predicted_data["v_new"], current_new=predicted_data["current_new"])
            errors = measurement_errors(predicted_data, prediction)
            predictions.append({"source": predicted_label,
                "role": "training_whole_run" if predicted_label == label else "blocked_whole_seeded_run_holdout",
                "prediction_errors": errors, "trajectory_gate": trajectory_gate(predicted_data, errors)})
        optimizer = fit["optimizer"]
        generation = fixture_report(data)
        generation_passed = generation["current_bound_respected"] and generation["slew_bound_respected"]
        gates = case_gates(data, optimizer, relative, recovery_limit, at_bounds, predictions, oracle_numerics)
        passed = (generation_passed and gates["data_integrity"] == "PASS" and gates["forward_numerics"] == "PASS" and
                  bool(optimizer["success"]) and not at_bounds and
                  all(abs(error) <= recovery_limit for error in relative.values()) and
                  all(p["trajectory_gate"]["passed"] for p in predictions))
        case = {"case": label, "generation": generation,
            "true_parameters": {key: FIXTURE[key] for key in FREE_BOUNDS},
            "fitted_parameters": {key: getattr(model, key) for key in FREE_BOUNDS},
            "relative_parameter_error": relative, "recovery_limit": recovery_limit,
            "parameter_bounds_hit": at_bounds, "optimizer": optimizer,
            "true_model_oracle_comparison": oracle_errors,
            "independent_truth_forward_numerics": oracle_numerics,
            "gates": gates,
            "predictions": predictions, "synthetic_case_gate_passed": passed}
        all_cases.append(case)
        save(output_dir / "estimator_cases.json", all_cases)
        print(json.dumps({"case": label, "fit": case["fitted_parameters"], "relative_error": relative,
                          "passed": passed, "optimizer": optimizer}), flush=True)
    statistics = {}
    for noisy in (False, True):
        cases = [case for case in all_cases if case["generation"]["noisy"] == noisy]
        statistics["noisy" if noisy else "pristine"] = {}
        for key in FREE_BOUNDS:
            values = np.array([case["fitted_parameters"][key] for case in cases])
            statistics["noisy" if noisy else "pristine"][key] = {
                "known_true": FIXTURE[key], "estimate_mean": float(values.mean()),
                "mean_relative_bias": float((values.mean() - FIXTURE[key]) / FIXTURE[key]),
                "estimate_std_across_seeds": float(values.std(ddof=1)) if len(values) > 1 else None,
                "worst_abs_relative_error": float(np.max(np.abs(values / FIXTURE[key] - 1)))}
    passed = all(case["synthetic_case_gate_passed"] for case in all_cases)
    result = {"schema": "adr0022.synthetic-closed-loop-estimator-probe/1", "provenance": "SYNTHETIC",
        "design_hypothesis": "Bounded whole-run native-sample output-error fit recovers observable mechanics under feedback-generated successful current, saturation/slew, and declared sensor errors",
        "oracle": "Independent exact piecewise analytic affine mechanics/Coulomb zero crossings and gyro-filter propagation; actual native Controller supplies feedback-dependent acknowledged TX",
        "predeclared_recovery_gates": RECOVERY_GATES, "free_parameter_bounds": FREE_BOUNDS,
        "fixed_known_synthetic_nuisances": {key: value for key, value in asdict(true_model).items() if key not in FREE_BOUNDS},
        "initialization": "One known zero latent initial state per full held fixture; never resets at later measured states",
        "sampling": "Encoder/current 1 kHz, control 200 Hz consuming latest encoder, gyro 50 Hz masks; no interpolated gyro residuals",
        "statistics": statistics, "cases": all_cases,
        "synthetic_recovery_gate_passed": passed,
        "estimator_status": "PARTIALLY_VERIFIED_SYNTHETIC_SUBSPACE" if passed else "UNVERIFIED_SYNTHETIC_RECOVERY_FAILED",
        "promotion_blocked": True, "yaw_qualification": "UNQUALIFIED",
        "limitations": ["All parameters/limits are declared synthetic; no station accessed and no deployment gains generated",
            "Pristine seeds repeat one deterministic run; noisy seeds change both measurement errors and feedback-generated currents",
            "Three noise seeds provide a small falsification probe, not a proof of asymptotic unbiasedness",
            "Fixed true nuisance parameters omit real clock, filter, friction and physical current semantic uncertainties",
            "Only algebraic actuator/Coulomb/local constant-load four-coordinate subspace exercised; no structure selection qualification"]}
    save(output_dir / "estimator_summary.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=8.0)
    parser.add_argument("--generate-only", action="store_true")
    parser.add_argument("--seeds", type=int, nargs="+", default=list(SEEDS))
    parser.add_argument("--max-nfev", type=int, default=120)
    args = parser.parse_args()
    if not math.isfinite(args.duration_s) or args.duration_s < 2.0 or len(set(args.seeds)) != len(args.seeds):
        parser.error("finite duration of at least 2 seconds and unique fixed seeds required")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error("Use a fresh output directory; retained evidence must not be overwritten")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    native = Native(args.library)
    reports = []
    datasets = {}
    save(args.output_dir / "predeclared_probe_contract.json", {"fixture": FIXTURE, "limits": LIMITS,
        "seeds": args.seeds, "duration_s": args.duration_s, "free_parameter_bounds": FREE_BOUNDS,
        "recovery_gates": RECOVERY_GATES, "maximum_optimizer_evaluations": args.max_nfev,
        "python": sys.version, "numpy": np.__version__, "library": str(args.library), "argv": sys.argv,
        "promotion_blocked": True, "provenance": "SYNTHETIC"})
    for noisy in (False, True):
        for seed in args.seeds:
            data = generate(native, args.duration_s, seed, noisy)
            label = f"{'noisy' if noisy else 'pristine'}-seed-{seed}"
            np.savez_compressed(args.output_dir / f"{label}.npz", **data)
            datasets[label] = data
            report = {"case": label, **fixture_report(data)}
            reports.append(report)
            save(args.output_dir / "generation.json", {"fixture": FIXTURE, "limits": LIMITS, "cases": reports,
                "oracle": "independent exact piecewise affine mechanical and causal gyro-filter solution; actual native Controller/observer/limits/acknowledgement",
                "sampling": "1 kHz encoder/current grid; 200 Hz native control consuming latest encoder; 50 Hz gyro with native sample masks; fixed 4 ms gyro signal delay",
                "qualification": "SYNTHETIC_ONLY; not a station model or deployable controller"})
            print(json.dumps(report), flush=True)
    if not args.generate_only:
        result = fit_cases(args.library, args.output_dir, datasets, args.max_nfev)
        if not result["synthetic_recovery_gate_passed"]:
            raise SystemExit(2)


if __name__ == "__main__":
    main()
