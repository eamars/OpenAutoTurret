"""Whole-run constrained initialization, output-error fit and joint bootstrap."""
from __future__ import annotations
from dataclasses import dataclass, replace
from typing import Sequence
import numpy as np
from scipy.optimize import least_squares, lsq_linear
from scipy.signal import lfilter

from .contracts import Identity, ModelSpec, Reason, Rejected, array, digest, require
from .model import features
from .native import Native


@dataclass
class Run:
    run_id: str
    identity: Identity
    t: np.ndarray
    q: np.ndarray
    v: np.ndarray
    tx: np.ndarray
    z: np.ndarray
    direction: np.ndarray
    q_new: np.ndarray
    v_new: np.ndarray
    sigma_q: float
    sigma_v: float
    bandwidth_hz: float
    generation: int = 1
    acquisition_verified: bool = True
    closed_loop_identification: bool = False
    support_controller_hash: str | None = None
    gyro_filter_tau_s: float | None = None
    # Actual successful command events, in the same time coordinate as t.
    # They may precede the measured state window; no state is integrated there.
    tx_history_t: np.ndarray | None = None
    tx_history_A: np.ndarray | None = None

    def validate(self):
        n = len(self.t)
        require(n >= 30 and type(self.generation) is int and self.generation > 0,
                Reason.DATA_INVALID, "run length/generation invalid")
        for name in ("t", "q", "v", "tx", "z", "direction"):
            setattr(self, name, array(getattr(self, name), (n,), name))
        require(np.all(np.diff(self.t) > 0), Reason.DATA_INVALID, "time must increase uniquely")
        history_t, history_A = getattr(self, "tx_history_t", None), getattr(self, "tx_history_A", None)
        require((history_t is None) == (history_A is None), Reason.DATA_INVALID,
                "successful TX history requires both timestamps and currents")
        if history_t is not None:
            self.tx_history_t = array(history_t, (len(history_t),), "successful TX history time")
            self.tx_history_A = array(history_A, (len(history_t),), "successful TX history current")
            require(len(history_t) > 0 and np.all(np.diff(self.tx_history_t) > 0),
                    Reason.DATA_INVALID, "successful TX history must increase uniquely")
        require(np.isin(self.direction, [-1, 0, 1]).all(), Reason.DATA_INVALID, "invalid direction")
        require(self.acquisition_verified is True, Reason.DATA_INVALID, "acquisition not verified")
        require(self.gyro_filter_tau_s is not None and np.isfinite(self.gyro_filter_tau_s) and
                self.gyro_filter_tau_s>=0,Reason.DATA_INVALID,"gyro observation filter must be declared, not assumed zero")
        for name in ("q_new", "v_new"):
            mask = np.asarray(getattr(self, name))
            require(mask.shape == (n,) and mask.dtype == bool and mask.sum() >= 2,
                    Reason.DATA_INVALID, "missing independent RX observations")
            setattr(self, name, mask)
        require(self.sigma_q > 0 and self.sigma_v > 0 and
                np.isfinite([self.sigma_q, self.sigma_v, self.bandwidth_hz]).all(),
                Reason.DATA_INVALID, "calibrated positive noise and bandwidth required")
        require(self.sigma_q <= np.deg2rad(.15)/3 and self.sigma_v <= np.deg2rad(.5)/3,
                Reason.MEASUREMENT_LIMITED, "measurement noise exceeds a third of quality target")
        for mask in (self.q_new, self.v_new):
            gaps = np.diff(self.t[mask])
            require(self.bandwidth_hz >= .5 and 1/np.max(gaps) >= 5*self.bandwidth_hz,
                    Reason.MEASUREMENT_LIMITED, "RX gaps/rate cannot resolve declared bandwidth")
        require(not self.closed_loop_identification or bool(self.support_controller_hash),
                Reason.DATA_INVALID, "closed-loop identification requires support-law identity")
        return self

    @property
    def hash(self):
        return digest({"id": self.run_id, "identity": self.identity.__dict__,
                       **({"gyro_filter_tau_s":self.gyro_filter_tau_s} if self.gyro_filter_tau_s else {}),
                       **{k: getattr(self, k).tolist() for k in
                          ("t", "q", "v", "tx", "z", "direction", "q_new", "v_new")},
                       "sigma_q": self.sigma_q, "sigma_v": self.sigma_v,
                       "bandwidth": self.bandwidth_hz, "generation": self.generation,
                       "support": self.support_controller_hash,
                       "closed_loop": self.closed_loop_identification})

    @property
    def content_hash(self):
        # A second identity excludes the label specifically for leakage detection.
        return replace(self,run_id="").hash


def check_split(train: Sequence[Run], holdout: Sequence[Run]):
    require(len(train) >= 4 and len(holdout) >= 2, Reason.INSUFFICIENT_EXCITATION,
            "independent complete training and validation runs required")
    identities = {tuple(r.identity.__dict__.values()) for r in [*train, *holdout]}
    require(len(identities) == 1, Reason.OPERATING_POINT_CHANGED,
            "different hardware/calibration/operating points must not be pooled")
    require(not {r.run_id for r in train} & {r.run_id for r in holdout} and
            not {r.content_hash for r in train} & {r.content_hash for r in holdout}, Reason.DATA_INVALID,
            "training/holdout leakage: split complete runs, never adjacent samples")
    for r in [*train, *holdout]:
        r.validate()


def integral_design(spec: ModelSpec, runs: Sequence[Run], window_s: float = .15):
    rows, targets, groups = [], [], []
    for group, run in enumerate(runs):
        run.validate()
        # Initialization only: interpolate within real RX coverage, never count held values as RX.
        q = np.interp(run.t, run.t[run.q_new], run.q[run.q_new])
        v = np.interp(run.t, run.t[run.v_new], run.v[run.v_new])
        start = 0
        while start < len(run.t)-2:
            end = int(np.searchsorted(run.t, run.t[start]+window_s))
            if end >= len(run.t):
                break
            sl = slice(start, end+1)
            dirs = run.direction[sl]
            # Do not span reversal/rest/contact. Use actual observed posture
            # weights rather than silently discarding normal posture drift.
            if dirs[0] and np.all(dirs == dirs[0]):
                s, h = features(spec, q[sl], run.z[sl], dirs)
                average_s = (s[:-1]+s[1:])/2
                rows.append(np.r_[np.sum(average_s*np.diff(v[sl])[:,None], axis=0),
                                  np.sum(average_s*np.diff(q[sl])[:,None], axis=0),
                                  np.trapezoid(h, run.t[sl], axis=0)])
                # Successful command is ZOH, not an invented 200Hz measured current stream.
                targets.append(np.sum(run.tx[start:end]*np.diff(run.t[sl])))
                groups.append(group)
            start = end
    require(bool(rows), Reason.INSUFFICIENT_EXCITATION, "no homogeneous running windows")
    return np.asarray(rows), np.asarray(targets), np.asarray(groups)


def bounded_initializer(X, y, *, lower_bounds=None):
    X, y = np.asarray(X), np.asarray(y)
    require(X.ndim == 2 and len(y) >= 2*X.shape[1], Reason.INSUFFICIENT_EXCITATION,
            "too few independent equations")
    scale = np.linalg.norm(X, axis=0)
    require(np.all(scale > 1e-12), Reason.INSUFFICIENT_EXCITATION, "unobserved parameter columns")
    singular = np.linalg.svd(X/scale, compute_uv=False)
    require(singular[-1] > singular[0]/1e6, Reason.INSUFFICIENT_EXCITATION,
            "rank deficient or normalized condition exceeds 1e6")
    lower = (np.r_[np.full(3, 1e-10), np.zeros(3), np.full(X.shape[1]-6, -np.inf)]
             if lower_bounds is None else np.asarray(lower_bounds, dtype=float))
    require(lower.shape == (X.shape[1],), Reason.DATA_INVALID, "initializer bound shape differs")
    fit = lsq_linear(X/scale, y, bounds=(lower*scale, np.full(X.shape[1], np.inf)),
                     method="trf", tol=1e-11)
    require(fit.success and np.isfinite(fit.x).all(), Reason.MODEL_INADEQUATE,
            "constrained initializer failed")
    return fit.x/scale, float(singular[0]/singular[-1])


def residuals(native: Native, spec, theta, runs):
    result = []
    for run in runs:
        predicted = native.rollout(spec, theta, run.t, run.tx, run.z, run.direction,
                                   (run.q[0], run.v[0]),
                                   tx_history_t=getattr(run, "tx_history_t", None),
                                   tx_history_A=getattr(run, "tx_history_A", None))
        gyro=observe_gyro(predicted[:,1],run.t,run.gyro_filter_tau_s)
        result.extend(((predicted[run.q_new, 0]-run.q[run.q_new])/run.sigma_q,
                       (gyro[run.v_new]-run.v[run.v_new])/run.sigma_v))
    return np.concatenate(result)


def observe_gyro(velocity,time,tau):
    require(tau is not None and np.isfinite(tau) and tau>=0,Reason.DATA_INVALID,"unknown gyro filter")
    if tau==0:return velocity
    gaps=np.diff(time)
    if np.allclose(gaps,gaps[0],rtol=1e-8,atol=1e-12):
        weight=gaps[0]/(tau+gaps[0])
        return lfilter([weight],[1.,weight-1],velocity,zi=[(1-weight)*velocity[0]])[0]
    out=np.empty(len(velocity));out[0]=velocity[0]
    for k,dt in enumerate(gaps,1):out[k]=out[k-1]+dt/(tau+dt)*(velocity[k]-out[k-1])
    return out


def output_error(native, spec, initial, runs, delay_bound_s, *, parameter_map=None,
                 reduced_initial=None, reduced_bounds=None, parameter_offset=None):
    require(np.isfinite(delay_bound_s) and delay_bound_s > 0, Reason.DATA_INVALID,
            "known positive delay search bound required; unknown is not zero")
    initial = array(initial, (spec.size,), "initializer")
    lower = np.r_[np.full(3, 1e-8), np.zeros(3), np.full(spec.size-7, -np.inf), 0.]
    upper = np.r_[np.full(spec.size-1, np.inf), delay_bound_s]
    if parameter_map is None:
        projection = np.eye(spec.size)
        coordinates = initial.copy()
        offset = np.zeros(spec.size)
        require(parameter_offset is None, Reason.DATA_INVALID,
                "a fixed parameter offset needs an explicit reduced map")
    else:
        projection = np.asarray(parameter_map, dtype=float)
        coordinates = np.asarray(reduced_initial, dtype=float)
        offset = (np.zeros(spec.size) if parameter_offset is None else
                  array(parameter_offset, (spec.size,), "fixed parameter offset"))
        require(projection.shape == (spec.size, len(coordinates)) and
                np.isfinite(projection).all() and np.isfinite(coordinates).all() and
                np.allclose(offset + projection @ coordinates, initial), Reason.DATA_INVALID,
                "parameter tying must reproduce the native initializer")
        require(reduced_bounds is not None, Reason.DATA_INVALID, "explicit reduced physical bounds required")
        lower, upper = (np.asarray(bound, dtype=float) for bound in reduced_bounds)
        require(lower.shape == upper.shape == coordinates.shape, Reason.DATA_INVALID,
                "reduced bound shape differs")
    count = sum(int(r.q_new.sum()+r.v_new.sum()) for r in runs)
    def objective(coordinates):
        theta = offset + projection @ coordinates
        try:
            return residuals(native, spec, theta, runs)
        except Rejected as exc:
            if exc.reason != Reason.MODEL_INADEQUATE:
                raise
            # A trial outside the declared spatial model is infeasible, never silently extrapolated.
            return np.full(count, 1e9 + np.linalg.norm(theta-initial))
    # Huber's modified Jacobian can have almost zero columns when every
    # observation is initially an outlier. Updating trust-region scales from
    # that Jacobian then permits enormous steps and false xtol convergence.
    # Freeze scales from the UNMODIFIED, noise-normalized observation Jacobian.
    base = objective(coordinates)
    sensitivity=[]
    for k in range(len(coordinates)):
        delta=max(abs(coordinates[k])*1e-5,1e-8)
        trial=coordinates.copy();trial[k]+=delta
        if trial[k]>upper[k]:trial[k]=coordinates[k]-delta
        sensitivity.append(np.linalg.norm((objective(trial)-base)/(trial[k]-coordinates[k])))
    sensitivity=np.asarray(sensitivity)
    require(np.all(sensitivity>1e-10) and np.isfinite(sensitivity).all(),
            Reason.INSUFFICIENT_EXCITATION,"output-error sensitivity has an unobserved parameter")
    scales=1/sensitivity
    result = least_squares(objective, coordinates, bounds=(lower, upper), method="trf",
                           loss="huber", f_scale=1., max_nfev=2000, x_scale=scales,
                           ftol=1e-8, xtol=1e-8, gtol=1e-8,
                           diff_step=1e-5)
    require(result.success and np.isfinite(result.x).all() and
            np.max(np.abs(objective(result.x))) < 1e8, Reason.MODEL_INADEQUATE,
            "output-error optimizer failed or left the model domain")
    if parameter_map is not None:
        result.reduced_x = result.x.copy()
        result.parameter_map = projection
        result.parameter_offset = offset
        result.x = offset + projection @ result.x
    return result


def validation_report(native, spec, theta, runs):
    reports = []
    for r in runs:
        pred = native.rollout(spec, theta, r.t, r.tx, r.z, r.direction, (r.q[0], r.v[0]),
                              tx_history_t=getattr(r, "tx_history_t", None),
                              tx_history_A=getattr(r, "tx_history_A", None))
        rq = pred[r.q_new, 0]-r.q[r.q_new]
        rv = observe_gyro(pred[:,1],r.t,r.gyro_filter_tau_s)[r.v_new]-r.v[r.v_new]
        qrms, vrms = float(np.sqrt(np.mean(rq**2))), float(np.sqrt(np.mean(rv**2)))
        baseline = r.q[0]+r.v[0]*(r.t-r.t[0])
        base = min(float(np.sqrt(np.mean((r.q-r.q[0])**2))),
                   float(np.sqrt(np.mean((r.q-baseline)**2))))
        limit_v = max(np.deg2rad(.5), float(np.mean(np.abs(r.v)))*.1)
        normalized = rv/r.sigma_v
        # Systematic structure is relevant only above the calibrated noise floor.
        ac = float(np.corrcoef(normalized[:-1], normalized[1:])[0, 1]) if np.std(normalized)>1e-9 else 0.
        bins = []
        edges=np.linspace(spec.q_nodes[0],spec.q_nodes[-1],6)
        for lo,hi in zip(edges[:-1],edges[1:]):
            mask = (r.q[r.q_new]>=lo)&(r.q[r.q_new]<=hi)
            if mask.sum() >= 5:
                bins.append(float(np.mean(rq[mask])))
        # Measurement-anchored one-observation prediction: carry the preceding
        # innovation forward, rather than counting a long free-run drift twice.
        one_q=np.diff(pred[r.q_new,0])-np.diff(r.q[r.q_new])
        one_v=np.diff(observe_gyro(pred[:,1],r.t,r.gyro_filter_tau_s)[r.v_new])-np.diff(r.v[r.v_new])
        one_q_rms=float(np.sqrt(np.mean(one_q**2)));one_v_rms=float(np.sqrt(np.mean(one_v**2)))
        passed = (qrms <= np.deg2rad(.15) and vrms <= limit_v and
                  one_q_rms<=np.deg2rad(.15) and one_v_rms<=limit_v and
                  (base <= np.deg2rad(.15) or qrms < .8*base) and
                  not (abs(ac) > .8 and vrms > 3*r.sigma_v))
        reports.append({"run_hash": r.hash, "q_rms_rad": qrms, "v_rms_rad_s": vrms,
                        "one_observation_q_rms_rad":one_q_rms,"one_observation_v_rms_rad_s":one_v_rms,
                        "baseline_q_rms_rad": base, "residual_lag1": ac,
                        "position_bin_residual_rad": bins, "passed": bool(passed)})
    return reports


def identify(native, spec, train, holdout, *, delay_bound_s, bootstrap=True, progress=None):
    check_split(train, holdout)
    X, y, groups = integral_design(spec, train)
    guess, condition = bounded_initializer(X, y)
    # A strictly interior initial delay avoids a zero numerical derivative at a ZOH boundary.
    initial = np.r_[guess, delay_bound_s/4]
    fitted = output_error(native, spec, initial, train, delay_bound_s)
    if progress: progress("output_error_fit", 1)
    hold = validation_report(native, spec, fitted.x, holdout)
    require(all(r["passed"] for r in hold), Reason.MODEL_INADEQUATE,
            f"independent holdout failed: {hold}")
    before = validation_report(native, spec, initial, holdout)
    require(sum(r["q_rms_rad"]**2+r["v_rms_rad_s"]**2 for r in hold) <=
            sum(r["q_rms_rad"]**2+r["v_rms_rad_s"]**2 for r in before) + 1e-12,
            Reason.MODEL_INADEQUATE, "output-error fit is worse than initialization on holdout")
    samples = []
    if bootstrap:
        # Whole-run, posture/direction-stratified block bootstrap. Correlated theta stays intact.
        strata = {}
        for k, r in enumerate(train):
            strata.setdefault((float(r.z[0]), int(r.direction[0])), []).append(k)
        require(all(len(v) >= 3 for v in strata.values()), Reason.INSUFFICIENT_EXCITATION,
                "at least three independent repeated runs per posture/direction for uncertainty")
        rng = np.random.default_rng(2202)
        for _ in range(128):
            indices = np.concatenate([rng.choice(v, len(v), replace=True) for v in strata.values()])
            chosen = [train[int(k)] for k in indices]
            result = output_error(native, spec, fitted.x, chosen, delay_bound_s)
            samples.append(result.x)
            if progress and len(samples)%8==0:progress("whole_run_bootstrap",len(samples))
    return {"theta": fitted.x, "uncertainty": np.asarray(samples),
            "report": {"normalized_condition": condition, "optimizer_evaluations": fitted.nfev,
                       "holdout": hold, "bootstrap_seed": 2202, "bootstrap_runs": len(samples),
                       "uncertainty_reliable": len(samples) == 128,
                       "initial_theta": initial.tolist(), "loss": "huber", "method": "trf"}}
