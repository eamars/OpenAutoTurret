"""Early executable probe of the native uninterrupted stick/start/reverse/coast path."""
from __future__ import annotations
from dataclasses import replace
import argparse
import json
import numpy as np
from .model_family import FamilyModel, FamilyNative


def probe(native):
    model = FamilyModel(a=.2, viscous=.03, coulomb_negative=.12, coulomb_positive=.1,
        static_negative=.2, static_positive=.18, q_min=-5., q_max=5.,
        actuator_gain=1., actuator_bias=0., transport_delay=.013,
        gyro_bias=.002, gyro_tau=.02, gyro_delay=.007,
        current_gain=1., current_bias=0., current_tau=0., current_delay=0.)
    t = np.arange(0., 3.2001, .005)
    tx_t = np.array([-.1, .3, .8, 1.3, 1.8])
    tx = np.array([.15, .4, 0., -.4, 0.])
    trace = native.rollout(model, t, tx_t, tx, [0., 0., .15, 0., .15])
    stalled = np.max(np.abs(trace[t < .313, 0])) == 0
    positive = np.max(trace[(t > .4) & (t < .8), 1]) > .2
    negative = np.min(trace[(t > 1.4) & (t < 1.8), 1]) < -.2
    stopped = abs(trace[-1, 1]) < 1e-12 and trace[-1, 5] == 1
    fast = native.rollout(replace(model, max_step=.00025), t, tx_t, tx, [0., 0., .15, 0., .15])
    numerical_angle = float(np.max(np.abs(trace[:, 0] - fast[:, 0])))
    numerical_speed = float(np.max(np.abs(trace[:, 1] - fast[:, 1])))
    slow_model = replace(model, actuator="first_order", actuator_tau=.025,
                         friction="stribeck", stribeck_negative=.1, stribeck_positive=.1)
    slow = native.rollout(slow_model, t, tx_t, tx, [0., 0., .15, 0., .15])
    slow_fine = native.rollout(replace(slow_model, max_step=.00025), t, tx_t, tx, [0., 0., .15, 0., .15])
    first_order_angle = float(np.max(np.abs(slow[:, 0] - slow_fine[:, 0])))
    first_order_speed = float(np.max(np.abs(slow[:, 1] - slow_fine[:, 1])))
    # Analytical first-order current during a known held step, independent of mechanics.
    use = (t >= .32) & (t < .8)
    exact = .4 + (.15 - .4) * np.exp(-(t[use] - .313) / .025)
    current_error = float(np.max(np.abs(slow[use, 2] - exact)))
    result = {"schema": "adr0022.family-probe/1", "provenance": "SYNTHETIC",
        "stick_before_threshold": bool(stalled), "positive_start": bool(positive),
        "predicted_reversal": bool(negative), "coast_and_reattach": bool(stopped),
        "integration_angle_difference_rad": numerical_angle,
        "integration_speed_difference_rad_s": numerical_speed,
        "first_order_current_analytical_error_A": current_error,
        "first_order_angle_resolution_difference_rad": first_order_angle,
        "first_order_speed_resolution_difference_rad_s": first_order_speed,
        "qualification": "OFFLINE_DIAGNOSTIC_UNQUALIFIED"}
    result["passed"] = bool(stalled and positive and negative and stopped and numerical_angle < 1e-5
                            and numerical_speed < 1e-4 and current_error < 1e-10
                            and first_order_angle < 1e-5 and first_order_speed < 1e-4)
    if not result["passed"]:
        raise AssertionError(result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native-library", required=True)
    args = parser.parse_args()
    print(json.dumps(probe(FamilyNative(args.native_library)), indent=2))


if __name__ == "__main__":
    main()
