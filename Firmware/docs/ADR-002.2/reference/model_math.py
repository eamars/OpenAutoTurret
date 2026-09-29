"""Offline mathematical INITIALIZERS, not a hardware tuner.

The regression below is the constant-load special case of docs/02.
It does not replace the required output-error estimator, spatial tables,
uncertainty analysis, shared C++ controller, or physical validation.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Iterable
import math
import numpy as np
from scipy.optimize import lsq_linear


class IdentificationError(ValueError):
    """Invalid measurements or insufficiently informative data."""


def _positive(value: float, name: str, *, zero_ok: bool = False) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f'{name}: boolean is not a number')
    value = float(value)
    if not math.isfinite(value) or (value < 0 if zero_ok else value <= 0):
        raise ValueError(f'{name}: invalid finite range')
    return value


def integral_rows(t: Iterable[float], q: Iterable[float], velocity: Iterable[float],
                  current: Iterable[float], direction: Iterable[int],
                  *, window_samples: int = 50) -> tuple[np.ndarray, np.ndarray]:
    """Generate [delta_v, delta_q, time_positive, time_negative] equations.

    Call once per homogeneous operating-point/run. Units are s, rad, rad/s, A.
    Excludes windows crossing a direction change or an explicit direction=0.
    No re-timestamping, interpolation of missing feedback, or second derivatives.
    """
    if type(window_samples) is not int or window_samples < 2:
        raise IdentificationError('window_samples must be an integer >= 2')
    values = [np.asarray(list(x), dtype=float) for x in (t, q, velocity, current, direction)]
    if any(x.ndim != 1 for x in values):
        raise IdentificationError('measurements must be one-dimensional')
    n = len(values[0])
    if n <= window_samples or any(len(x) != n for x in values):
        raise IdentificationError('length mismatch or insufficient samples')
    if any(not np.all(np.isfinite(x)) for x in values):
        raise IdentificationError('non-finite measurement')
    ts, pos, vel, iq, dirs = values
    if np.any(np.diff(ts) <= 0):
        raise IdentificationError('timestamps must be strictly increasing unique RX times')
    if not np.all(np.isin(dirs, [-1, 0, 1])):
        raise IdentificationError('directions must be -1/0/+1')
    rows, targets = [], []
    for first in range(0, n - window_samples, window_samples):
        last = first + window_samples
        d = dirs[first:last + 1]
        if d[0] == 0 or np.any(d != d[0]):
            continue
        elapsed = ts[last] - ts[first]
        rows.append([vel[last] - vel[first], pos[last] - pos[first],
                     elapsed if d[0] == 1 else 0., elapsed if d[0] == -1 else 0.])
        targets.append(float(np.trapezoid(iq[first:last + 1], ts[first:last + 1])))
    if not rows:
        raise IdentificationError('no valid homogeneous-direction windows')
    return np.asarray(rows), np.asarray(targets)


@dataclass(frozen=True)
class ConstantLoadInitializer:
    a: float
    b: float
    load_positive: float
    load_negative: float
    rms_integral_residual: float
    normalized_condition: float
    rows: int

    def as_vector(self) -> np.ndarray:
        return np.array([self.a, self.b, self.load_positive, self.load_negative])


def fit_initializer(X: np.ndarray, y: np.ndarray, *, condition_limit: float = 1e6
                    ) -> ConstantLoadInitializer:
    """Bounded least squares, after rank/column-scale checks.

    Reports no confidence interval: errors-in-variables, closed-loop bias and
    temporal dependence are NOT removed by this initializer.
    """
    condition_limit = _positive(condition_limit, 'condition_limit')
    X, y = np.asarray(X, dtype=float), np.asarray(y, dtype=float)
    if X.ndim != 2 or X.shape[1] != 4 or y.shape != (X.shape[0],) or len(y) < 8:
        raise IdentificationError('require >=8 rows, four columns, matching scalar response')
    if not np.all(np.isfinite(X)) or not np.all(np.isfinite(y)):
        raise IdentificationError('non-finite design matrix or response')
    scale = np.linalg.norm(X, axis=0)
    if np.any(scale <= np.finfo(float).eps):
        raise IdentificationError('INSUFFICIENT_EXCITATION: a feature was never excited')
    normalized = X / scale
    singular = np.linalg.svd(normalized, compute_uv=False)
    if singular[-1] <= np.finfo(float).eps * max(normalized.shape) * singular[0]:
        raise IdentificationError('INSUFFICIENT_EXCITATION: rank deficiency')
    condition = float(singular[0] / singular[-1])
    if condition > condition_limit:
        raise IdentificationError('INSUFFICIENT_EXCITATION: ill-conditioned normalized design')
    low = np.array([1e-10, 0., -np.inf, -np.inf]) * scale
    high = np.full(4, np.inf)
    result = lsq_linear(normalized, y, bounds=(low, high), method='trf', tol=1e-12)
    if not result.success or not np.all(np.isfinite(result.x)):
        raise IdentificationError('SOLVER_FAILED')
    theta = result.x / scale
    residual = X @ theta - y
    return ConstantLoadInitializer(*map(float, theta), float(np.sqrt(np.mean(residual**2))),
                                   condition, len(y))


def ideal_pi(a: float, b: float, omega_n: float, *, zeta: float = 1.) -> dict[str, float]:
    """Analytic starting point; no stability/performance qualification implied."""
    a = _positive(a, 'a')
    b = _positive(b, 'b', zero_ok=True)
    wn = _positive(omega_n, 'omega_n')
    zeta = _positive(zeta, 'zeta')
    kp, ki = 2 * zeta * wn * a - b, a * wn**2
    if kp <= 0:
        raise ValueError('INFEASIBLE_PI: proportional coefficient is nonpositive; do not clamp')
    return {'kp_A_s_per_rad': kp, 'ki_A_per_rad': ki, 'omega_n_rad_s': wn, 'zeta': zeta}


def linear_discrete_matrix(a: float, b: float, kp: float, ki: float, dt: float,
                           *, transport_delay_s: float = 0.) -> np.ndarray:
    """Exact scalar ZOH plant + the documented trapezoidal integrator update.

    State=[velocity, integral_current, previous_error, newest_cmd,...oldest_cmd].
    Fractional transport delay is rounded UP to sample periods, not a proof of
    worst-case robustness for an arbitrary delayed/filtered system.
    This omits the actual observer, spatial load, saturation and reference chain;
    it is a unit-testable local mathematical check, not the full solver.
    """
    a, b = _positive(a, 'a'), _positive(b, 'b', zero_ok=True)
    kp, ki = _positive(kp, 'kp'), _positive(ki, 'ki', zero_ok=True)
    dt = _positive(dt, 'dt')
    delay = _positive(transport_delay_s, 'transport_delay_s', zero_ok=True)
    delay_steps = int(math.ceil(delay / dt - 1e-12))
    if delay_steps > 10000:
        raise ValueError('delay too large for this reference linear check')
    phi = math.exp(-b * dt / a)
    gamma = -math.expm1(-b * dt / a) / b if b > 1e-12 else dt / a
    A = np.zeros((3 + delay_steps, 3 + delay_steps))
    A[0, 0] = phi
    if delay_steps:
        A[0, -1] = gamma
        A[3, 0], A[3, 1] = -kp, 1.
        for k in range(1, delay_steps):
            A[3 + k, 3 + k - 1] = 1.
    else:
        A[0, 0] -= gamma * kp
        A[0, 1] = gamma
    A[1, 0], A[1, 1], A[1, 2] = -ki * dt / 2, 1., ki * dt / 2
    A[2, 0] = -1.
    return A


def spectral_radius(A: np.ndarray) -> float:
    A = np.asarray(A, dtype=float)
    if A.ndim != 2 or A.shape[0] != A.shape[1] or not np.all(np.isfinite(A)):
        raise ValueError('finite square matrix required')
    return float(np.max(np.abs(np.linalg.eigvals(A))))
