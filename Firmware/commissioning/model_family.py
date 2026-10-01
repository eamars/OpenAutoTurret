"""Predeclared offline fixed-pitch yaw families; successful TX drives whole runs.

Numerical success is a diagnostic candidate, never physical qualification or gains.
Physical nuisance parameters must be supplied explicitly. Defaults below are model
structure or numerical integration conventions, not measurements of the station.
"""
from __future__ import annotations

import ctypes as ct
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
import numpy as np

from .contracts import Reason, Rejected, require
from .applicability import configuration_pool
from scipy.optimize import least_squares, lsq_linear
from scipy.signal import savgol_filter


MODEL_FIELDS = ("a", "viscous", "coulomb_negative", "coulomb_positive", "static_negative",
    "static_positive", "stribeck_negative", "stribeck_positive", "stribeck_power",
    "load_offset", "load_slope", "q_origin", "actuator_gain", "actuator_bias", "actuator_tau",
    "transport_delay", "gyro_bias", "gyro_tau", "gyro_delay", "current_gain", "current_bias",
    "current_tau", "current_delay", "q_min", "q_max", "max_step")


@dataclass(frozen=True, kw_only=True)
class FamilyModel:
    a: float
    viscous: float
    coulomb_negative: float
    coulomb_positive: float
    static_negative: float
    static_positive: float
    q_min: float
    q_max: float
    actuator_gain: float
    actuator_bias: float
    transport_delay: float
    gyro_bias: float
    gyro_tau: float
    gyro_delay: float
    current_gain: float
    current_bias: float
    current_tau: float
    current_delay: float
    actuator: str = "algebraic"
    friction: str = "coulomb"
    load: str = "constant"
    actuator_tau: float = 0.
    stribeck_negative: float = 0.
    stribeck_positive: float = 0.
    stribeck_power: float = 2.
    load_offset: float = 0.
    load_slope: float = 0.
    q_origin: float = 0.
    max_step: float = .001

    def validate(self):
        require(self.actuator in ("algebraic", "first_order") and self.friction in ("coulomb", "stribeck")
                and self.load in ("constant", "affine"), Reason.DATA_INVALID,
                "unsupported family: winding/history/compliance need discriminating evidence")
        require(np.isfinite([getattr(self, k) for k in MODEL_FIELDS]).all(), Reason.DATA_INVALID,
                "all family parameters must be finite and explicit")
        require(self.a > 0 and self.viscous >= 0 and self.coulomb_negative >= 0 and self.coulomb_positive >= 0
                and self.static_negative >= self.coulomb_negative and self.static_positive >= self.coulomb_positive,
                Reason.DATA_INVALID, "A-equivalent inertia positive; B>=0 and Fs>=Fc>=0 required")
        require(self.actuator_gain > 0 and self.current_gain > 0 and self.q_max > self.q_min
                and 0 < self.max_step <= .01, Reason.DATA_INVALID, "invalid scales/domain/integration resolution")
        require(min(self.actuator_tau, self.transport_delay, self.gyro_tau, self.gyro_delay,
                    self.current_tau, self.current_delay) >= 0, Reason.DATA_INVALID, "negative timing parameter")
        require(self.actuator != "first_order" or self.actuator_tau > 0, Reason.DATA_INVALID,
                "first-order actuator requires positive tau")
        require(self.friction != "stribeck" or (self.stribeck_negative > 0 and self.stribeck_positive > 0
                and self.stribeck_power >= 1), Reason.DATA_INVALID, "invalid Stribeck speed/power")
        require(self.load != "constant" or self.load_slope == 0, Reason.DATA_INVALID,
                "constant load family cannot conceal a spatial slope")
        return self

    @property
    def structure(self):
        return f"rigid-yaw/{self.actuator}/{self.friction}/{self.load}"

    def document(self):
        return {"schema": "adr0022.identification-family/1", **asdict(self),
                "structure": self.structure, "parameter_units": "A-equivalent, rad, s",
                "q_domain_role": "SUPPLIED_NUMERICAL_ROLLOUT_DOMAIN; NOT_PHYSICAL_TRAVEL_OR_CERTIFIED_SUPPORT",
                "qualification": "OFFLINE_DIAGNOSTIC_UNQUALIFIED"}


class CFamilyModel(ct.Structure):
    _fields_ = [("actuator", ct.c_int), ("friction", ct.c_int), ("load", ct.c_int)] + [
        (k, ct.c_double) for k in MODEL_FIELDS]


class FamilyNative:
    def __init__(self, library_path):
        path = Path(library_path)
        require(path.is_file(), Reason.INTEGRATION_MISMATCH, "locally built axis shared library required")
        self.lib = ct.CDLL(str(path.resolve()))
        self.function = getattr(self.lib, "ota_identification_rollout", None)
        require(self.function is not None, Reason.INTEGRATION_MISMATCH, "rebuild native identification family core")
        p = ct.POINTER(ct.c_double)
        self.function.argtypes = [ct.POINTER(CFamilyModel), ct.c_int, p, ct.c_int, p, p, p, p]
        self.function.restype = ct.c_int

    def rollout(self, model, t, tx_t, tx_A, initial):
        model.validate()
        vectors = [np.ascontiguousarray(x, dtype=np.float64) for x in (t, tx_t, tx_A, initial)]
        require(vectors[0].ndim == 1 and len(vectors[0]) >= 2 and np.all(np.diff(vectors[0]) > 0)
                and vectors[1].ndim == 1 and len(vectors[1]) > 0 and np.all(np.diff(vectors[1]) > 0)
                and vectors[2].shape == vectors[1].shape and vectors[3].shape == (5,)
                and all(np.isfinite(x).all() for x in vectors), Reason.DATA_INVALID,
                "whole-run time/TX history/one latent initial state invalid")
        cmodel = CFamilyModel(int(model.actuator == "first_order"), int(model.friction == "stribeck"),
            int(model.load == "affine"), *(getattr(model, k) for k in MODEL_FIELDS))
        out = np.empty((len(t), 6), dtype=np.float64)
        pointers = [x.ctypes.data_as(ct.POINTER(ct.c_double)) for x in vectors]
        status = self.function(ct.byref(cmodel), len(t), pointers[0], len(tx_t), *pointers[1:],
                              out.ctypes.data_as(ct.POINTER(ct.c_double)))
        require(status == 0, Reason.MODEL_INADEQUATE if status == 2 else Reason.DATA_INVALID,
                "predicted state outside declared domain" if status == 2 else
                "accepted-TX history missing before run start minus delay" if status == 3 else
                f"native family rollout rejected contract ({status})")
        return out


@dataclass
class FamilyRun:
    run_id: str
    source_id: str
    t: np.ndarray
    q: np.ndarray
    v: np.ndarray
    current: np.ndarray
    q_new: np.ndarray
    v_new: np.ndarray
    current_new: np.ndarray
    tx_t: np.ndarray
    tx_A: np.ndarray
    initial: np.ndarray
    sigma_q: float
    sigma_v: float
    sigma_current: float
    physical_run_id: str | None = None
    configuration_id: str = ""
    calibration_revision: str = ""
    provenance: str = "MEASURED"
    initial_bounds: tuple | None = None
    encoder_quantum: float = 0.
    calibration_support: dict | None = None
    configuration_facts: object | None = None

    def validate(self):
        self.physical_run_id = self.physical_run_id or self.run_id
        require(bool(self.run_id) and bool(self.source_id) and bool(self.physical_run_id)
                and self.provenance in ("MEASURED", "SYNTHETIC"), Reason.DATA_INVALID,
                "physical acquisition/source identity and provenance required")
        self.t = np.asarray(self.t, dtype=float)
        n = len(self.t)
        require(n >= 3 and np.isfinite(self.t).all() and np.all(np.diff(self.t) > 0),
                Reason.DATA_INVALID, "native observation union times must increase")
        for channel, mask_name in (("q", "q_new"), ("v", "v_new"), ("current", "current_new")):
            values = np.asarray(getattr(self, channel), dtype=float)
            mask = np.asarray(getattr(self, mask_name))
            require(values.shape == mask.shape == (n,) and mask.dtype == bool and mask.sum() >= 2
                    and np.isfinite(values[mask]).all(), Reason.DATA_INVALID,
                    f"{channel}: finite native observations with explicit freshness mask required")
            setattr(self, channel, values); setattr(self, mask_name, mask)
        for channel in ("tx_t", "tx_A", "initial"):
            setattr(self, channel, np.asarray(getattr(self, channel), dtype=float))
        require(self.tx_t.ndim == 1 and len(self.tx_t) >= 1 and self.tx_A.shape == self.tx_t.shape
                and np.all(np.diff(self.tx_t) > 0) and self.initial.shape == (5,)
                and all(np.isfinite(getattr(self, k)).all() for k in ("tx_t", "tx_A", "initial")),
                Reason.DATA_INVALID, "accepted input history/one initial latent state invalid")
        require(np.isfinite([self.sigma_q, self.sigma_v, self.sigma_current, self.encoder_quantum]).all()
                and min(self.sigma_q, self.sigma_v, self.sigma_current) > 0 and self.encoder_quantum >= 0,
                Reason.DATA_INVALID, "observation noise/quantization must be explicit")
        if self.initial_bounds is not None:
            lo, hi = (np.asarray(x, dtype=float) for x in self.initial_bounds)
            require(lo.shape == hi.shape == (5,) and np.isfinite([lo, hi]).all()
                    and np.all(lo <= self.initial) and np.all(self.initial <= hi), Reason.DATA_INVALID,
                    "bounded whole-run initial latent state invalid")
            self.initial_bounds = (lo, hi)
        return self

    @classmethod
    def from_union(cls, observations, **noise_and_constraints):
        return cls(run_id=observations["physical_run_id"], source_id=observations["source_journal"],
            physical_run_id=observations["physical_run_id"], configuration_id=observations["configuration_id"],
            calibration_revision=observations["calibration_revision"], t=observations["t"],
            q=observations["q_obs"], v=observations["v_obs"], current=observations["current_obs"],
            q_new=observations["q_new"], v_new=observations["v_new"], current_new=observations["current_new"],
            tx_t=observations["tx_t"], tx_A=observations["tx_A"], initial=observations["initial"],
            calibration_support=observations.get("calibration_support"), **noise_and_constraints).validate()


def check_family_split(train, selection, holdout):
    require(all(len(group) > 0 for group in (train, selection, holdout)), Reason.INSUFFICIENT_EXCITATION,
            "complete physical train, selection and final validation blocks required")
    seen_runs, seen_sources = set(), set()
    for name, group in (("train", train), ("selection", selection), ("holdout", holdout)):
        ids, sources = set(), set()
        for run in group:
            run.validate()
            require(run.physical_run_id not in ids and run.source_id not in sources, Reason.DATA_INVALID,
                    f"{name}: repeated windows from one physical acquisition are not independent runs")
            ids.add(run.physical_run_id); sources.add(run.source_id)
        require(not ids & seen_runs and not sources & seen_sources, Reason.DATA_INVALID,
                "physical acquisition leaked across train/selection/holdout")
        seen_runs.update(ids); seen_sources.update(sources)


def _errors(run, prediction):
    position = prediction[run.q_new, 0] - run.q[run.q_new]
    # Interval likelihood surrogate: a quantized encoder observation identifies a
    # bin, not an exact point. Noise outside that bin remains scaled by sigma_q.
    position = np.sign(position) * np.maximum(np.abs(position) - run.encoder_quantum / 2, 0.)
    return position, prediction[run.v_new, 3] - run.v[run.v_new], \
        prediction[run.current_new, 4] - run.current[run.current_new]


def _bounded_central_jacobian(objective, x, lower, upper, absolute_steps):
    """Differentiate raw residuals at fixed physical steps, respecting bounds.

    Robustification belongs to least_squares. Relative forward differences can
    span several hybrid/encoder-bin transitions and stall an otherwise viable
    fit. These steps do not shrink to zero with a zero-valued coordinate.
    """
    columns = []
    for k, step in enumerate(absolute_steps):
        plus, minus = x.copy(), x.copy()
        plus[k], minus[k] = min(x[k] + step, upper[k]), max(x[k] - step, lower[k])
        columns.append((objective(plus) - objective(minus)) / (plus[k] - minus[k]))
    return np.column_stack(columns)


def moving_integral_initializer(model, runs, bounds):
    """Bounded moving-equation seed; actual uninterrupted hybrid OE still fits.

    Scope: four mechanics coordinates, algebraic current/Coulomb/constant load,
    supplied fixed sensor and input nuisances, sufficiently uniform native gyro.
    Equation fitting chooses an initialization, never a model acceptance gate.
    """
    fields = ("a", "viscous", "coulomb_negative", "coulomb_positive")
    if set(bounds) != set(fields) or model.actuator != "algebraic" or model.friction != "coulomb" \
            or model.load != "constant":
        return None, {"status":"NOT_APPLICABLE", "scope":"four mechanics with fixed supplied nuisances"}
    matrix, rhs, contributions = [], [], []
    for run in runs:
        times = run.t[run.v_new]-model.gyro_delay
        gyro = run.v[run.v_new]-model.gyro_bias
        if len(times)<5:
            return None, {"status":"INSUFFICIENT_NATIVE_GYRO", "run_id":run.run_id}
        spacing = float(np.median(np.diff(times)))
        if not np.isfinite(spacing) or spacing<=0 or \
                np.max(np.abs(np.diff(times)-spacing)) > max(1e-8, spacing*.01):
            return None, {"status":"INSUFFICIENT_UNIFORM_NATIVE_GYRO", "run_id":run.run_id}
        window = max(5, int(round(.22/spacing)) | 1)
        if window > len(times):
            return None, {"status":"INSUFFICIENT_NATIVE_GYRO", "run_id":run.run_id}
        smooth = savgol_filter(gyro, window, 3, mode="interp")
        derivative = savgol_filter(gyro, window, 3, deriv=1, delta=spacing, mode="interp")
        velocity = smooth+model.gyro_tau*derivative
        command_times = run.tx_t+model.transport_delay
        def input_integral(start, end):
            inside = command_times[(command_times>start) & (command_times<end)]
            edges = np.r_[start, inside, end]
            held = np.searchsorted(command_times, edges[:-1]+1e-13, side="right")-1
            return float(np.diff(edges) @ (model.actuator_gain*run.tx_A[held]+model.actuator_bias))
        q_times, q_values = run.t[run.q_new], run.q[run.q_new]
        first_row = len(rhs)
        minimum_speed = max(.05, 10*run.sigma_v)
        for count in (max(3, int(round(duration/spacing))) for duration in (.12,.2,.4)):
            for start in range(0,len(times)-count,3):
                end = start+count
                section = velocity[start:end+1]
                if times[start] < max(run.t[0],command_times[0]) or not (
                        np.all(section>minimum_speed) or np.all(section<-minimum_speed)):
                    continue
                direction = 1 if section[0]>0 else -1
                duration = times[end]-times[start]
                delta_q = np.interp(times[end],q_times,q_values)-np.interp(times[start],q_times,q_values)
                matrix.append([velocity[end]-velocity[start],delta_q,
                    -duration if direction<0 else 0.,duration if direction>0 else 0.])
                rhs.append(input_integral(times[start],times[end])-model.load_offset*duration)
        contributions.append({"run_id":run.run_id,"equations":len(rhs)-first_row,
                              "minimum_abs_velocity_rad_s":minimum_speed})
    if len(rhs)<4 or np.linalg.matrix_rank(matrix)<4:
        return None, {"status":"INSUFFICIENT_MOVING_EQUATIONS", "equations":len(rhs)}
    matrix, rhs = np.asarray(matrix),np.asarray(rhs)
    result = lsq_linear(matrix,rhs,bounds=([bounds[key][0] for key in fields],
                                         [bounds[key][1] for key in fields]))
    if not result.success:
        return None, {"status":"BOUNDED_EQUATION_SOLVER_FAILED", "message":str(result.message)}
    values = dict(zip(fields,map(float,result.x)))
    return replace(model,**values), {"status":"COMPUTED_TRAINING_ONLY", "parameters":values,
        "equations":len(rhs),"per_run":contributions,"matrix_rank":int(np.linalg.matrix_rank(matrix)),
        "matrix_condition":float(np.linalg.cond(matrix)),
        "equation_residual_rms_A_s":float(np.sqrt(np.mean((matrix @ result.x-rhs)**2))),
        "smoothing_window_s":.22,"moving_interval_s":[.12,.2,.4],
        "selection_or_holdout_used":False,"physical_identifiability":False,
        "scope":"four mechanics with fixed supplied nuisances; final native observations and hybrid transitions unchanged"}


def family_reports(native, model, runs):
    reports = []
    for run in runs:
        run.validate()
        try:
            pred = native.rollout(model, run.t, run.tx_t, run.tx_A, run.initial)
        except Rejected as exc:
            reports.append({"run_id": run.run_id, "physical_run_id": run.physical_run_id,
                "passed": False, "reason": str(exc), "prediction_kind": "INPUT_DRIVEN_WHOLE_RUN",
                "qualification": "OFFLINE_DIAGNOSTIC_UNQUALIFIED"})
            continue
        eq, ev, ei = _errors(run, pred)
        # Report both centre error and quantization-aware error; acceptance stays
        # on the original 0.15 degree centre-error requirement.
        raw_eq = pred[run.q_new, 0] - run.q[run.q_new]
        q_rms = float(np.sqrt(np.mean(raw_eq ** 2)))
        v_rms = float(np.sqrt(np.mean(ev ** 2)))
        current_rms = float(np.sqrt(np.mean(ei ** 2)))
        limit_v = max(np.deg2rad(.5), float(np.mean(np.abs(run.v[run.v_new]))) * .1)
        autocorrelation = float(np.corrcoef(ev[:-1], ev[1:])[0, 1]) if np.std(ev) > 1e-12 else 0.
        failures = []
        if q_rms > np.deg2rad(.15): failures.append("angle_rms")
        if v_rms > limit_v: failures.append("velocity_rms")
        if current_rms > 3 * run.sigma_current: failures.append("current_observation_rms")
        if abs(autocorrelation) > .8 and v_rms > 3 * run.sigma_v:
            failures.append("structured_velocity_residual")
        passed = not failures
        # All horizons extend from the one allowed acquisition initialization.
        horizons = {}
        for horizon in (.05, .1, .2, .5, 1.):
            mask = run.q_new & (run.t <= run.t[0] + horizon)
            horizons[str(horizon)] = float(np.sqrt(np.mean((pred[mask, 0] - run.q[mask]) ** 2))) if mask.any() else None
        reports.append({"run_id": run.run_id, "physical_run_id": run.physical_run_id,
            "source_id": run.source_id, "duration_s": float(run.t[-1] - run.t[0]),
            "q_rms_rad": q_rms, "q_rms_deg": float(np.rad2deg(q_rms)),
            "quantization_interval_q_rms_rad": float(np.sqrt(np.mean(eq ** 2))),
            "v_rms_rad_s": v_rms, "v_rms_deg_s": float(np.rad2deg(v_rms)),
            "current_rms_A": current_rms, "q_endpoint_error_rad": float(raw_eq[-1]),
            "v_endpoint_error_rad_s": float(ev[-1]), "velocity_limit_rad_s": float(limit_v),
            "velocity_residual_lag1": autocorrelation, "initial_horizon_q_rms_rad": horizons,
            "metric_failures": failures,
            "native_observation_counts": {"encoder": int(run.q_new.sum()), "gyro": int(run.v_new.sum()),
                                          "current": int(run.current_new.sum())},
            "passed": bool(passed), "prediction_kind": "INPUT_DRIVEN_WHOLE_RUN",
            "state_resets": 0, "qualification": "OFFLINE_DIAGNOSTIC_UNQUALIFIED"})
    return reports


def fit_family(native, model, train, *, bounds, max_nfev=200, progress=None, configuration_support=None):
    """Bounded diagnostic OE, training only; no closed-loop unbiasedness claim.

    Use static_excess_negative/positive coordinates: Fs=Fc+excess. Initial latent
    states may move only inside supplied bounds, once per complete physical run.
    """
    model.validate()
    for run in train: run.validate()
    configuration = configuration_pool(train, support=configuration_support)
    require(bool(train) and bool(bounds) and 1 <= max_nfev <= 2000, Reason.DATA_INVALID,
            "training runs, bounded coordinates and finite optimizer budget required")
    aliases = ("static_excess_negative", "static_excess_positive")
    allowed = set(MODEL_FIELDS) - {"q_min", "q_max", "max_step", "static_negative", "static_positive", "stribeck_power"}
    require(set(bounds) <= allowed | set(aliases), Reason.DATA_INVALID,
            "unsupported fit coordinate; use static_excess directional coordinates for Fs>=Fc")
    require(not ("actuator_gain" in bounds and "a" in bounds), Reason.INSUFFICIENT_EXCITATION,
            "actuator torque/current gain and A-equivalent inertia need an independent scale datum")
    fields = list(bounds)
    seed = {**asdict(model), **{f"static_excess_{d}": getattr(model, f"static_{d}") -
        getattr(model, f"coulomb_{d}") for d in ("negative", "positive")}}
    coordinates = [seed[k] for k in fields]
    lower, upper = [bounds[k][0] for k in fields], [bounds[k][1] for k in fields]
    latent = []
    for index, run in enumerate(train):
        if run.initial_bounds is None: continue
        lo, hi = run.initial_bounds
        active = (0, 1) + ((2,) if model.actuator == "first_order" else ()) + \
            ((3,) if model.gyro_tau > 0 else ()) + ((4,) if model.current_tau > 0 else ())
        for j in active:
            if lo[j] < hi[j]:
                latent.append((index, j)); coordinates.append(run.initial[j]); lower.append(lo[j]); upper.append(hi[j])
    coordinates, lower, upper = (np.asarray(x, dtype=float) for x in (coordinates, lower, upper))
    require(np.isfinite([coordinates, lower, upper]).all() and np.all(lower < upper)
            and np.all((coordinates >= lower) & (coordinates <= upper)), Reason.DATA_INVALID,
            "finite physical fit bounds must contain initializer")
    for k, name in enumerate(fields):
        if name in aliases or name in ("viscous", "coulomb_negative", "coulomb_positive", "transport_delay",
            "gyro_tau", "gyro_delay", "current_tau", "current_delay"):
            require(lower[k] >= 0, Reason.DATA_INVALID, "nonnegative physical coordinate bound required")
        if name in ("a", "actuator_gain", "current_gain", "stribeck_negative", "stribeck_positive"):
            require(lower[k] > 0, Reason.DATA_INVALID, "positive physical coordinate bound required")
        if name.startswith("coulomb_"):
            d = name.removeprefix("coulomb_")
            require(f"static_excess_{d}" in bounds or upper[k] <= getattr(model, f"static_{d}"),
                    Reason.DATA_INVALID, "fixed Fs requires Fc bounds below Fs; otherwise fit static_excess")
    def unpack(x):
        values = dict(zip(fields, x[:len(fields)]))
        changes = {k: v for k, v in values.items() if k not in aliases}
        for d in ("negative", "positive"):
            if f"static_excess_{d}" in values:
                changes[f"static_{d}"] = changes.get(f"coulomb_{d}", getattr(model, f"coulomb_{d}")) + \
                    values[f"static_excess_{d}"]
        candidate = replace(model, **changes)
        initials = [run.initial.copy() for run in train]
        for value, (index, j) in zip(x[len(fields):], latent): initials[index][j] = value
        return candidate, initials
    count = sum(int(r.q_new.sum() + r.v_new.sum() + r.current_new.sum()) for r in train)
    calls = 0
    began = time.monotonic()
    def evaluate(x):
        nonlocal calls
        calls += 1
        if progress and calls % 25 == 0:
            progress("output_error", {"structure": model.structure, "residual_evaluations": calls,
                                      "elapsed_s": time.monotonic() - began})
        candidate, initials = unpack(x)
        residual, invalid = [], []
        for run, initial in zip(train, initials):
            try:
                pred = native.rollout(candidate, run.t, run.tx_t, run.tx_A, initial)
                eq, ev, ei = _errors(run, pred)
                residual.extend((eq / run.sigma_q, ev / run.sigma_v, ei / run.sigma_current))
            except Rejected as exc:
                if exc.reason != Reason.MODEL_INADEQUATE: raise
                invalid.append({"run_id": run.run_id, "reason": str(exc)})
                # Preserve the actual residuals of every other physical run.
                # This block is an infeasibility penalty, never observation or
                # parameter-support evidence.
                samples = int(run.q_new.sum() + run.v_new.sum() + run.current_new.sum())
                residual.append(np.full(samples, 1e6 + np.linalg.norm(x - coordinates)))
        return np.concatenate(residual), invalid
    def objective(x):
        return evaluate(x)[0]
    initial_coordinates = coordinates.copy()
    _, original_invalid = evaluate(coordinates)
    feasibility_trials = 1
    if original_invalid:
        # Deterministic training-only feasibility restoration. Increase inertia
        # then drag inside the existing declared fit bounds. No extrapolated
        # trajectory, selection/holdout value, or controller gain is consulted.
        def toward_upper(name):
            if name not in fields: return [None]
            index = fields.index(name)
            start, end = coordinates[index], upper[index]
            values = [start]
            if start > 0:
                values.extend(min(start * factor, end) for factor in (2., 4., 8., 16.))
            else:
                values.extend(end * fraction for fraction in (.0625, .125, .25, .5))
            values.append(end)
            return list(dict.fromkeys(values))
        feasible = None
        for drag in toward_upper("viscous"):
            for inertia in toward_upper("a"):
                trial = initial_coordinates.copy()
                if drag is not None: trial[fields.index("viscous")] = drag
                if inertia is not None: trial[fields.index("a")] = inertia
                _, invalid = evaluate(trial)
                feasibility_trials += 1
                if not invalid:
                    feasible = trial
                    break
            if feasible is not None: break
        require(feasible is not None, Reason.MODEL_INADEQUATE,
                "FEASIBLE_INITIALIZATION_NOT_FOUND inside declared bounds; no structural rejection inferred")
        coordinates = feasible
    initialization = {"policy": "training-only inertia/drag backtracking within supplied bounds",
        "original_coordinates": initial_coordinates[:len(fields)].tolist(),
        "feasible_coordinates": coordinates[:len(fields)].tolist(),
        "original_invalid_runs": original_invalid, "trials": feasibility_trials,
        "feasible": True, "selection_or_holdout_used": False}
    integral_seed, integral_metadata = moving_integral_initializer(model,train,bounds)
    if integral_seed is not None:
        trial = coordinates.copy()
        trial[:len(fields)] = [getattr(integral_seed,key) for key in fields]
        current_values, _ = evaluate(coordinates)
        trial_values, trial_invalid = evaluate(trial)
        def huber_cost(values):
            absolute = np.abs(values)
            return float(np.sum(np.where(absolute<=1,.5*values**2,absolute-.5)))
        existing_cost, integral_cost = huber_cost(current_values),huber_cost(trial_values)
        # The moving equations provide an independent basin entry. A supplied
        # point can have a lower initial cost while grazing a hybrid transition
        # and trap the local solver; comparing unoptimized seed costs would
        # defeat this recovery (confirmed by quintic hold/reverse regression).
        # Final acceptance remains the actual hybrid whole-run fit and gates.
        selected = not trial_invalid
        if selected: coordinates = trial
        integral_metadata.update(selected=bool(selected),existing_initializer_cost=existing_cost,
            integral_initializer_cost=integral_cost,invalid_runs=trial_invalid,
            selection_rule="use valid bounded moving-equation basin entry; do not compare unoptimized seed costs")
        initialization["feasible_coordinates"] = coordinates[:len(fields)].tolist()
    initialization["moving_integral_initializer"] = integral_metadata
    if progress: progress("feasible_initialization", {"structure": model.structure, **initialization})
    # Characteristic physical coordinate magnitudes (or 5% of their declared
    # fit interval for zero seeds), never the numerical rollout angle guard.
    # A fixed symmetric physical perturbation avoids the old relative forward
    # step spanning hybrid transitions and many encoder-interval residual kinks.
    # The four-mechanics synthetic fixture verifies this procedure; unknown
    # delays and direct discontinuous current observations need separate probes.
    scales = np.maximum(np.abs(coordinates), (upper - lower) * .05)
    derivative_steps = scales * 1e-6
    accepted_coordinate_steps = []
    previous_jacobian_point = None
    def jacobian(x):
        nonlocal previous_jacobian_point
        accepted_coordinate_steps.append({"coordinates": x.tolist(),
            "characteristic_step_norm": None if previous_jacobian_point is None else
                float(np.linalg.norm((x-previous_jacobian_point)/scales))})
        previous_jacobian_point = x.copy()
        return _bounded_central_jacobian(objective, x, lower, upper, derivative_steps)
    result = least_squares(objective, coordinates, bounds=(lower, upper), loss="huber", f_scale=1.,
        method="trf", max_nfev=max_nfev, x_scale=scales, jac=jacobian)
    fitted, initials = unpack(result.x)
    fitted_train = [replace(run, initial=initial) for run, initial in zip(train, initials)]
    base, final_invalid = evaluate(result.x)
    successful = bool(result.success and np.isfinite(result.x).all() and not final_invalid)
    # Unmodified noise-scaled objective sensitivity, not Huber's downweighted J.
    # This numerical diagnostic does not establish physical identifiability or
    # calibrate a confidence ensemble from correlated receipts.
    columns = []
    invalid_perturbations = []
    if not final_invalid:
        for k in range(len(result.x)):
            step = derivative_steps[k]
            plus, minus = result.x.copy(), result.x.copy()
            plus[k], minus[k] = min(result.x[k]+step, upper[k]), max(result.x[k]-step, lower[k])
            positive, positive_invalid = evaluate(plus)
            negative, negative_invalid = evaluate(minus)
            if positive_invalid and negative_invalid:
                invalid_perturbations.append(k); columns.append(np.zeros(count))
            elif positive_invalid:
                columns.append((negative-base)/(minus[k]-result.x[k]))
            elif negative_invalid:
                columns.append((positive-base)/(plus[k]-result.x[k]))
            else:
                columns.append((positive-negative)/(plus[k]-minus[k]))
        jacobian = np.column_stack(columns)
        norms = np.linalg.norm(jacobian, axis=0)
        observed = norms > 1e-10
        singular = np.linalg.svd(jacobian[:, observed] / norms[observed], compute_uv=False) if observed.any() else np.array([])
        rank = int(np.sum(singular > singular[0] / 1e6)) if len(singular) else 0
        condition = float(singular[0] / singular[-1]) if len(singular) and singular[-1] > 0 else None
        sensitivity = {"status": "PARTIAL_INFEASIBLE_PERTURBATIONS" if invalid_perturbations else "COMPUTED_FROM_OBSERVATIONS",
            "column_norms": [None if k in invalid_perturbations else float(v) for k, v in enumerate(norms)],
            "observed_columns": np.flatnonzero(observed).tolist(), "normalized_rank": rank,
            "coordinate_count": len(result.x), "normalized_condition": condition,
            "invalid_perturbation_columns": invalid_perturbations, "physical_identifiability": False, "covariance": None}
        sensitivity["difference_policy"] = "bounded central physical steps; valid one-sided fallback for infeasible side"
    else:
        sensitivity = {"status": "NOT_COMPUTED_INVALID_TRAJECTORY", "invalid_runs": final_invalid,
            "column_norms": None, "observed_columns": [], "normalized_rank": None,
            "coordinate_count": len(result.x), "normalized_condition": None,
            "physical_identifiability": False, "covariance": None}
    bound_hits = [fields[k] if k < len(fields) else f"initial:{latent[k - len(fields)]}"
                  for k in range(len(result.x)) if min(result.x[k] - lower[k], upper[k] - result.x[k]) <=
                  1e-5 * (upper[k] - lower[k])]
    derived = [f"static_{d}" for d in ("negative", "positive") if f"static_excess_{d}" in fields]
    return {"model": fitted, "training": family_reports(native, fitted, fitted_train),
        "configuration_assessment": configuration,
        "optimizer": {"success": successful, "evaluations": int(result.nfev), "message": str(result.message),
            "termination_status": int(result.status), "jacobian_evaluations": int(result.njev),
            "residual_evaluations_including_jacobian_and_diagnostics": calls,
            "cost": float(result.cost), "optimality": float(result.optimality),
            "coordinate_scales": scales.tolist(), "absolute_derivative_steps": derivative_steps.tolist(),
            "derivative_method": "bounded central differences of raw noise-scaled residuals",
            "derivative_scale_policy": "1e-6 times fixed characteristic coordinate scale; zero seeds use 5% of fit interval",
            "accepted_coordinate_steps": accepted_coordinate_steps,
            "coordinates": fields, "coordinate_values": result.x[:len(fields)].tolist(),
            "initial_latent_states": [x.tolist() for x in initials], "initial_state_count_per_run": 1,
            "bounds": {k: list(v) for k, v in bounds.items()}, "normalized_residual_rms": float(np.sqrt(np.mean(result.fun ** 2))),
            "initialization": initialization, "final_infeasible_runs": final_invalid,
            "loss": "huber", "closed_loop_bias_qualification": "UNQUALIFIED",
            "parameter_status": "ESTIMATED_DIAGNOSTIC_NOT_PHYSICALLY_IDENTIFIED",
            "derived_estimated_coordinates": derived,
            "derived_coordinate_formulas": {k: k.replace("static_", "coulomb_") + " + " +
                k.replace("static_", "static_excess_") for k in derived},
            "prior_only_fixed_coordinates": [k for k in MODEL_FIELDS if k not in fields and k not in derived],
            "unmodified_objective_sensitivity": sensitivity,
            "parameter_bound_hits": bound_hits, "elapsed_s": time.monotonic() - began},
        "qualification": "OFFLINE_DIAGNOSTIC_UNQUALIFIED", "deployable": False}


def compare_families(native, candidates, train, selection, holdout, *, max_nfev=200, progress=None,
                     configuration_support=None):
    """Fit train, choose least complex passing selection, evaluate final holdout once.

    candidates: (label, FamilyModel, bounds) tuples. Final holdout never changes
    structure or parameters. All historical outcomes remain nondeployable.
    """
    check_family_split(train, selection, holdout)
    # Context metadata is part of the predeclared split, not future motion data.
    configuration = configuration_pool([*train, *selection, *holdout], support=configuration_support)
    comparisons = []
    fitted_models = []
    for label, model, bounds in candidates:
        if progress: progress("family_started", {"label": label, "structure": model.structure})
        try:
            fit = fit_family(native, model, train, bounds=bounds, max_nfev=max_nfev, progress=progress,
                             configuration_support=configuration_support)
            chosen_model = fit["model"]
            reports = family_reports(native, chosen_model, selection)
            training_passed = all(r["passed"] for r in fit["training"])
            accepted = fit["optimizer"]["success"] and training_passed and all(r["passed"] for r in reports)
            complexity = len(bounds) + int(model.actuator == "first_order") + 2 * int(model.friction == "stribeck") + int(model.load == "affine")
            score = sum(r.get("q_rms_rad", 1e6) ** 2 + r.get("v_rms_rad_s", 1e6) ** 2 for r in reports)
            comparisons.append({"label": label, "model": chosen_model.document(), "optimizer": fit["optimizer"],
                "training": fit["training"], "training_passed": bool(training_passed),
                "selection": reports, "selection_passed": bool(accepted),
                "complexity": complexity, "selection_score": float(score)})
            fitted_models.append(chosen_model)
        except Rejected as exc:
            comparisons.append({"label": label, "selection_passed": False, "reason": str(exc)})
            fitted_models.append(None)
        if progress: progress("family_completed", comparisons[-1])
    eligible = [i for i, r in enumerate(comparisons) if r["selection_passed"]]
    selected = min(eligible, key=lambda i: (comparisons[i]["complexity"], comparisons[i]["selection_score"],
                                          comparisons[i]["label"])) if eligible else None
    final = family_reports(native, fitted_models[selected], holdout) if selected is not None else []
    outcome = "MODEL_DATA_FAILURE" if selected is None else "HISTORICAL_HOLDOUT_PASSED" if all(r["passed"] for r in final) else "MODEL_DATA_FAILURE"
    return {"schema": "adr0022.whole-run-family-comparison/1", "comparisons": comparisons,
        "configuration_assessment": configuration,
        "selected_label": comparisons[selected]["label"] if selected is not None else None,
        "selected_model": comparisons[selected]["model"] if selected is not None else None,
        "final_holdout": final, "outcome": outcome, "deployable": False,
        "qualification": "OFFLINE_DIAGNOSTIC_UNQUALIFIED",
        "selection_rule": "least complex structure passing training and separate physical selection; score then label tie-break",
        "holdout_evaluated_for_rejected_families": False,
        "required_next_gate": "frozen calibration/identifiability and prospective predictions, physical 3a and 3b"}
