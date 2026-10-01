"""Native local-perturbation probe; synthetic/offline only, no hardware or gains."""
from __future__ import annotations

import argparse
import csv
from dataclasses import replace
import itertools
import json
from pathlib import Path
import sys

import numpy as np
from scipy.linalg import expm

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.family_analysis import SlidingPoint, SlidingSupport
from Firmware.commissioning.model_family import FamilyModel, FamilyNative
from Firmware.commissioning.synthesis import selected_family_linearization as linearize_sliding_family


def first_probe(library):
    model = FamilyModel(a=.1, viscous=.01, coulomb_negative=.1, coulomb_positive=.1,
        static_negative=.2, static_positive=.2, q_min=-2., q_max=2., actuator_gain=1.2,
        actuator_bias=.013, transport_delay=0., gyro_bias=.002, gyro_tau=.02, gyro_delay=0.,
        current_gain=.9, current_bias=.004, current_tau=.015, current_delay=0.,
        actuator="first_order", actuator_tau=.015, friction="stribeck",
        stribeck_negative=.1, stribeck_positive=.1, max_step=1e-6)
    point = SlidingPoint(q_rad=0., v_rad_s=.1, configuration_id="synthetic-local", frame="output_shaft_rad")
    support = SlidingSupport(configuration_id=point.configuration_id, frame=point.frame,
        q_min_rad=-1., q_max_rad=1., v_min_rad_s=.02, v_max_rad_s=.2)
    linear = linearize_sliding_family(model, point, support)
    effective = .01*.1 + .1 + .1*np.exp(-1.)
    initial = np.array([0., .1, effective, .1, effective])
    command = (effective-model.actuator_bias)/model.actuator_gain
    native, t, eps = FamilyNative(library), np.array([0., 1e-4]), 1e-6
    columns = []
    for j in range(5):
        delta = np.eye(5)[j]*eps
        plus = native.rollout(model, t, [-1.], [command], initial+delta)
        minus = native.rollout(model, t, [-1.], [command], initial-delta)
        columns.append((plus[-1, :5]-minus[-1, :5])/(2*eps))
    observed = np.column_stack(columns)
    observed[4] /= model.current_gain
    expected = expm(linear.A*t[-1])
    error = float(np.max(np.abs(observed-expected)))
    assert linear.incremental_damping_A_s_rad < 0, "velocity weakening was clipped"
    assert error < 1e-7, f"native perturbation mismatch {error}"
    return {"scope": "SYNTHETIC_OFFLINE", "pass": True,
        "negative_incremental_damping_A_s_rad": linear.incremental_damping_A_s_rad,
        "max_native_transition_matrix_error": error, "gate": 1e-7}


def model_and_context(direction, actuator, friction, load, filters):
    model = FamilyModel(a=.13, viscous=.027, coulomb_negative=.09, coulomb_positive=.12,
        static_negative=.19, static_positive=.22, q_min=-2., q_max=2., actuator_gain=1.17,
        actuator_bias=.014, transport_delay=0., gyro_bias=.006,
        gyro_tau=.018 if filters[0] else 0., gyro_delay=0., current_gain=.87,
        current_bias=-.009, current_tau=.013 if filters[1] else 0., current_delay=0.,
        actuator=actuator, actuator_tau=.023 if actuator == "first_order" else 0.,
        friction=friction, stribeck_negative=.07, stribeck_positive=.11,
        stribeck_power=1.6, load=load, load_offset=.031, load_slope=.21 if load == "affine" else 0.,
        q_origin=-.14, max_step=1e-6)
    v = direction*(.075 if direction < 0 else .105)
    point = SlidingPoint(q_rad=.12, v_rad_s=v, configuration_id="synthetic-local", frame="output_shaft_rad")
    support = SlidingSupport(configuration_id=point.configuration_id, frame=point.frame,
        q_min_rad=-1., q_max_rad=1., v_min_rad_s=.02 if direction > 0 else -.2,
        v_max_rad_s=.2 if direction > 0 else -.02)
    fc = model.coulomb_positive if direction > 0 else model.coulomb_negative
    fs = model.static_positive if direction > 0 else model.static_negative
    vs = model.stribeck_positive if direction > 0 else model.stribeck_negative
    friction_A = direction*(fc+(fs-fc)*np.exp(-(abs(v)/vs)**model.stribeck_power)
                            if friction == "stribeck" else fc)
    effective = model.load_offset+model.load_slope*(point.q_rad-model.q_origin)+model.viscous*v+friction_A
    command = (effective-model.actuator_bias)/model.actuator_gain
    initial = np.array([point.q_rad, v, effective, v, effective])
    return model, point, support, initial, command


def integrated_drive(linear, duration):
    n = len(linear.A)
    augmented = np.zeros((n+1, n+1))
    augmented[:n, :n], augmented[:n, n:] = linear.A, linear.B
    transition = expm(augmented*duration)
    return transition[:n, :n], transition[:n, n]


def selected_initial_indices(linear):
    source = {"q_rad": 0, "v_rad_s": 1, "effective_current_A": 2,
              "gyro_filter_rad_s": 3, "current_filter_A": 4}
    return [source[name] for name in linear.state_names]


def recover_states(trace, linear):
    values = trace[-1, :5].copy()
    values[3] -= linear.model.gyro_bias
    values[4] = (values[4]-linear.model.current_bias)/linear.model.current_gain
    return values[selected_initial_indices(linear)]


def local_matrix_case(native, direction, actuator, friction, load, filters):
    model, point, support, initial, command = model_and_context(direction, actuator, friction, load, filters)
    linear = linearize_sliding_family(model, point, support)
    t, eps = np.array([0., 1e-4]), 1e-6
    observed_states, observed_outputs = [], []
    for j in selected_initial_indices(linear):
        delta = np.eye(5)[j]*eps
        plus = native.rollout(model, t, [-1.], [command], initial+delta)
        minus = native.rollout(model, t, [-1.], [command], initial-delta)
        observed_states.append((recover_states(plus, linear)-recover_states(minus, linear))/(2*eps))
        observed_outputs.append((plus[-1, [0, 3, 4]]-minus[-1, [0, 3, 4]])/(2*eps))
    plus = native.rollout(model, t, [-1.], [command+eps], initial)
    minus = native.rollout(model, t, [-1.], [command-eps], initial)
    observed_input = (recover_states(plus, linear)-recover_states(minus, linear))/(2*eps)
    observed_feedthrough = (plus[-1, [0, 3, 4]]-minus[-1, [0, 3, 4]])/(2*eps)
    transition, drive = integrated_drive(linear, t[-1])
    errors = {"transition_error": float(np.max(np.abs(np.column_stack(observed_states)-transition))),
        "input_error": float(np.max(np.abs(observed_input-drive))),
        "output_error": float(np.max(np.abs(np.column_stack(observed_outputs)-linear.C@transition))),
        "feedthrough_error": float(np.max(np.abs(observed_feedthrough-(linear.C@drive+linear.D[:, 0]))))}
    assert max(errors.values()) < 1e-7, (direction, actuator, friction, load, filters, errors)
    # These are declared signs/units, checked against observed native responses.
    assert linear.A[1, 0] == -model.load_slope/model.a
    assert linear.input_delay_s == 0 and linear.output_delays_s == (0., 0., 0.)
    linear.assert_trajectory(plus[:, 0], plus[:, 1], model=model, support=support,
        frame=point.frame, configuration_id=point.configuration_id, friction_state="SLIDING")
    return {"direction": direction, "actuator": actuator, "friction": friction, "load": load,
        "gyro_filter": filters[0], "current_filter": filters[1], "dynamic_states": len(linear.A),
        "friction_derivative_A_s_rad": linear.friction_derivative_A_s_rad,
        "incremental_damping_A_s_rad": linear.incremental_damping_A_s_rad,
        "load_gradient_A_rad": linear.load_gradient_A_rad, **errors, "pass": True}


def delayed_step_case(native, direction, actuator, friction, filters):
    # Constant-load steady sliding baseline makes the frozen tangent exact at
    # the baseline. One causal TX step is perturbed, not future state samples.
    model, point, support, initial, command = model_and_context(direction, actuator, friction, "constant", filters)
    model = replace(model, transport_delay=.00137, gyro_delay=.00213, current_delay=.00307)
    linear = linearize_sliding_family(model, point, support)
    t = np.unique(np.r_[0., np.linspace(.00043, .025, 37), .00473, .0061, .00823, .00917])
    event, eps = .00473, 1e-6
    plus = native.rollout(model, t, [-1., event], [command, command+eps], initial)
    minus = native.rollout(model, t, [-1., event], [command, command-eps], initial)
    observed = (plus[:, [0, 3, 4]]-minus[:, [0, 3, 4]])/(2*eps)
    expected = np.zeros_like(observed)
    for k, at in enumerate(t):
        for channel, delay in enumerate(linear.output_delays_s):
            duration = at-event-linear.input_delay_s-delay
            if duration >= -1e-13:
                _, drive = integrated_drive(linear, max(0., duration))
                expected[k, channel] = linear.C[channel]@drive+linear.D[channel, 0]
    error = float(np.max(np.abs(observed-expected)))
    assert error < 1e-7, (direction, actuator, friction, filters, error)
    assert np.max(np.abs(observed[t < event+model.transport_delay])) < 1e-9, "actuator delay disappeared"
    assert np.max(np.abs(observed[t < event+model.transport_delay+model.current_delay, 2])) < 1e-9
    linear.assert_trajectory(plus[:, 0], plus[:, 1], model=model, support=support,
        frame=point.frame, configuration_id=point.configuration_id, friction_state="SLIDING")
    undelayed = linearize_sliding_family(replace(model, transport_delay=0., gyro_delay=0., current_delay=0.), point, support)
    omega = np.array([.3, 7., 41., 193.])
    expected_transfer = undelayed.frequency_response(omega)*np.exp(-1j*omega[:, None]*
        (model.transport_delay+np.array([0., model.gyro_delay, model.current_delay])))
    assert np.max(np.abs(linear.frequency_response(omega)-expected_transfer)) < 1e-12
    return {"direction": direction, "actuator": actuator, "friction": friction,
        "gyro_filter": filters[0], "current_filter": filters[1], "step_error": error, "pass": True}


def comprehensive_probe(library, output):
    output.mkdir(parents=True, exist_ok=True)
    native = FamilyNative(library)
    cases = [local_matrix_case(native, *case) for case in itertools.product((-1, 1),
        ("algebraic", "first_order"), ("coulomb", "stribeck"), ("constant", "affine"),
        ((False, False), (False, True), (True, False), (True, True)))]
    delay_cases = [delayed_step_case(native, *case) for case in itertools.product((-1, 1),
        ("algebraic", "first_order"), ("coulomb", "stribeck"),
        ((False, False), (False, True), (True, False), (True, True)))]
    for name, rows in (("native_matrix_cases.csv", cases), ("native_delay_cases.csv", delay_cases)):
        with (output/name).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
    summary = {"scope": "SYNTHETIC_OFFLINE_UNQUALIFIED", "pass": True,
        "matrix_cases": len(cases), "delayed_step_cases": len(delay_cases), "absolute_error_gate": 1e-7,
        "matrix_max_error": max(row[key] for row in cases for key in
            ("transition_error", "input_error", "output_error", "feedthrough_error")),
        "delayed_step_max_error": max(row["step_error"] for row in delay_cases),
        "first_probe": first_probe(library), "delay_representation": "EXACT; NO_PADE_OR_INVERSION",
        "flags": {"native_local_tangent_verified": True, "both_directions_verified": True,
            "negative_damping_preserved": True, "native_fractional_delays_preserved": True,
            "physical_qualified": False, "controller_gains_supplied": False,
            "sampled_observer_controller_margins_verified": False,
            "rest_reversal_nonlinear_manoeuvres_covered": False}}
    (output/"summary.json").write_text(json.dumps(summary, indent=2)+"\n", encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    print(json.dumps(comprehensive_probe(args.library, args.output) if args.output else first_probe(args.library), indent=2))


if __name__ == "__main__": main()
