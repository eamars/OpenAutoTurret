#!/usr/bin/env python3
"""Offline arithmetic only. No hardware API, control thread or deployable controller."""
from __future__ import annotations
import json
import math
from typing import Iterable


def _finite(*values: float) -> None:
    if not all(math.isfinite(v) for v in values):
        raise ValueError('all numeric inputs must be finite')


def blocked_axis_time(speed_deg_s: float, kp: float, ki: float,
                      breakaway_a: float, ceiling_a: float) -> float | None:
    """Idealized constant-reference, stationary-plant PI ramp. None = unreachable.

    Uses magnitudes, zero initial integral, no reference shaping, no friction FF.
    This is NOT a measured motor response or an identification of static friction.
    """
    _finite(speed_deg_s, kp, ki, breakaway_a, ceiling_a)
    if kp < 0 or ki < 0 or breakaway_a < 0 or ceiling_a <= 0:
        raise ValueError('gains/threshold must be nonnegative and ceiling positive')
    if breakaway_a > ceiling_a:
        return None
    speed = abs(math.radians(speed_deg_s))
    p = kp * speed
    if breakaway_a <= p:
        return 0.0
    if ki == 0 or speed == 0:
        return None
    return (breakaway_a - p) / (ki * speed)


def current_aw_step(integral_a: float, ki_a_per_rad: float, velocity_error_rad_s: float,
                    dt_s: float, kaw_per_s: float, requested_a: float,
                    actually_limited_a: float, integral_limit_a: float) -> float:
    """One mathematical back-calculation step; caller supplies FINAL limited output.

    Not a complete controller. Actual TX failure, watchdog, breakaway sequencing,
    estimator validity and motor current limits must be handled by integration.
    """
    _finite(integral_a, ki_a_per_rad, velocity_error_rad_s, dt_s,
            kaw_per_s, requested_a, actually_limited_a, integral_limit_a)
    if dt_s <= 0 or integral_limit_a <= 0 or ki_a_per_rad < 0 or kaw_per_s < 0:
        raise ValueError('invalid timestep, gain or integral bound')
    result = (integral_a + ki_a_per_rad * velocity_error_rad_s * dt_s
              + kaw_per_s * (actually_limited_a - requested_a) * dt_s)
    return min(integral_limit_a, max(-integral_limit_a, result))


def rx_window_velocity(samples: Iterable[tuple[int, float]]) -> float | None:
    """Unwrapped position slope from fresh RX timestamps; radians/second.

    Identical duplicate timestamps/positions are ignored. Conflicting duplicate
    or reversed timestamps are rejected. At least two unique samples are needed.
    The caller controls the window; this function makes no bandwidth promise.
    """
    rows: list[tuple[int, float]] = []
    for t, q in samples:
        if isinstance(t, bool) or not isinstance(t, int) or t < 0:
            raise ValueError('RX timestamp must be a nonnegative integer in ns')
        _finite(q)
        if rows and t < rows[-1][0]:
            raise ValueError('RX time reversed')
        if rows and t == rows[-1][0]:
            if q != rows[-1][1]:
                raise ValueError('same RX timestamp with conflicting position')
            continue
        rows.append((t, q))
    if len(rows) < 2:
        return None
    # Subtract integer origin BEFORE float conversion (large monotonic clocks).
    origin = rows[0][0]
    times = [(t - origin) * 1e-9 for t, _ in rows]
    mt = sum(times) / len(times)
    mq = sum(q for _, q in rows) / len(rows)
    denominator = sum((t - mt) ** 2 for t in times)
    if denominator <= 0:
        return None
    return sum((t - mt) * (q - mq) for t, (_, q) in zip(times, rows)) / denominator


def encoder_speed_quantum_deg_s(period_s: float, counts_per_rev: int = 8192) -> float:
    _finite(period_s)
    if period_s <= 0 or counts_per_rev <= 0:
        raise ValueError('positive period and encoder resolution required')
    return 360.0 / counts_per_rev / period_s


def main() -> None:
    print(json.dumps({
        'kind': 'ILLUSTRATIVE_ARITHMETIC_NOT_HARDWARE',
        'assumption': 'stationary plant; constant reference; zero I; breakaway=0.40 A',
        'breakaway_a_is_not_identified': True,
        'kp': 1.0, 'ki': 0.6, 'ceiling_a': 0.8,
        'blocked_axis_time_s': {str(v): blocked_axis_time(v, 1, .6, .4, .8)
                                for v in (3, 5, 10)},
        'gm_encoder_deg_per_count': 360 / 8192,
        'single_count_at_5ms_deg_s': encoder_speed_quantum_deg_s(.005),
    }, ensure_ascii=False, indent=2, allow_nan=False))

if __name__ == '__main__':
    main()
