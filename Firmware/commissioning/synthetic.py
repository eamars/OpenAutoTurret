"""Analytic inverse-dynamics oracle, independent of the C++ rollout integrator.

Every number in this module is a declared synthetic fixture, never a station value.
"""
from __future__ import annotations
import numpy as np
from .contracts import Identity, ModelSpec, Reason, digest, require
from .identification import Run
from .model import coefficients


CONDITIONS = ("BASELINE", "PAYLOAD_UP", "PAYLOAD_DOWN", "FRICTION_UP", "FRICTION_DOWN",
              "RETURN_BASELINE", "UNDECLARED_CHANGE", "CENTRE_OF_MASS", "TEMPERATURE_SUPPLY")


def fixture(axis="yaw", condition="BASELINE"):
    require(condition in CONDITIONS,Reason.OPERATING_POINT_CHANGED,"unsupported synthetic condition")
    spec = ModelSpec(axis, (-1., -.5, 0., .5, 1.), (-.3, 0., .3))
    a = np.array([.08, .10, .12]) * (1.3 if axis == "pitch" else 1.)
    b = np.array([.05, .06, .07])
    h = np.empty((2, 3, 5))
    for j, d in enumerate((-1, 1)):
        for iz, z in enumerate(spec.z_nodes):
            h[j, iz] = .12*d + .04*np.asarray(spec.q_nodes) + .025*z
            if axis == "pitch":
                h[j, iz] += .18*np.cos(np.asarray(spec.q_nodes))
    if condition == "PAYLOAD_UP": a *= 1.5; h += .08
    if condition == "PAYLOAD_DOWN": a *= .7; h -= .035
    if condition in ("FRICTION_UP", "UNDECLARED_CHANGE"):
        b *= 1.8; h[0] -= .06; h[1] += .06
    if condition == "FRICTION_DOWN": b *= .6; h[0] += .04; h[1] -= .04
    if condition == "CENTRE_OF_MASS": h += .1*np.asarray(spec.q_nodes)[None, None, :]
    if condition == "TEMPERATURE_SUPPLY": b *= 1.2
    theta = np.r_[a, b, h.ravel(), .008]
    identity = Identity(digest({"fixture": axis}), digest("synthetic-calibration-v2"),
                        digest({"condition": condition, "theta": theta.tolist()}), "SYNTHETIC")
    return spec, theta, identity


def runs(spec, theta, identity, *, repetitions=4, seed=22, noise=True, mismatch=False):
    rng = np.random.default_rng(seed)
    result = []
    for iz, z in enumerate(spec.z_nodes):
        for d in (-1, 1):
            for repeat in range(repetitions):
                duration = 5.+repeat*.55
                dt = .01
                t = np.arange(0, duration+dt/2, dt)
                duration = t[-1]
                phase = .3*repeat
                # Different acceleration/velocity profiles across repeated whole runs remove
                # position/velocity confounding; all stay inside the spatial model boundary.
                w = 2*np.pi*(repeat%3+1)/duration
                def trajectory(at):
                    span = 2*np.pi if spec.periodic else 1.7
                    centre = spec.q_nodes[0]+np.pi if spec.periodic else 0.
                    base = span/duration
                    q = centre-d*span/2+d*base*(at+.32*(np.sin(w*at+phase)-np.sin(phase))/w)
                    v = d*base*(1+.32*np.cos(w*at+phase))
                    acc = -d*base*.32*w*np.sin(w*at+phase)
                    return q, v, acc
                q, v, acc = trajectory(t)
                # A sampled ZOH command approximates the smooth analytic input at interval
                # midpoint. It is advanced by the true delay, which the fitter must recover.
                qi, vi, ai = trajectory(t+theta[-1]+dt/2)
                a, b, h = coefficients(spec, theta, qi, z, np.full(len(t), d))
                tx = a*ai+b*vi+h
                if mismatch:
                    q = q+.015*np.sin(2*np.pi*3*t)
                    v = v+.015*2*np.pi*3*np.cos(2*np.pi*3*t)
                sigma_q, sigma_v = 2e-5, 8e-5
                if noise:
                    q = q+rng.normal(0, sigma_q, len(t))
                    v = v+rng.normal(0, sigma_v, len(t))
                    # The initial state is explicitly calibrated for this fixture.
                    q[0], v[0], _ = trajectory(0.)
                result.append(Run(f"{seed}-{iz}-{d}-{repeat}", identity, t, q, v, tx,
                                  np.full(len(t), z), np.full(len(t), d),
                                  np.ones(len(t), dtype=bool), np.ones(len(t), dtype=bool),
                                  sigma_q, sigma_v, 15.,gyro_filter_tau_s=0.))
    return result


def probe(native):
    from .identification import identify
    spec, theta, identity = fixture()
    train, holdout = runs(spec, theta, identity, seed=22), runs(spec, theta, identity, seed=23, repetitions=2)
    fit = identify(native, spec, train, holdout, delay_bound_s=.025, bootstrap=False)
    error = np.abs(fit["theta"]-theta)
    return {"mode": "SYNTHETIC_LOCAL_ONLY", "hardware_accessed": False,
            "max_parameter_error": float(error.max()), "delay_error_s": float(error[-1]),
            "a_relative_error": float(np.max(error[:3]/theta[:3])),
            "fit": fit["report"]}


if __name__ == "__main__":
    import json
    from .native import Native
    print(json.dumps(probe(Native()), indent=2))
