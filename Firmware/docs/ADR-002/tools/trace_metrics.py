#!/usr/bin/env python3
"""Read-only statistics on an ADR-002 CSV trace subset. Python 3.10+, stdlib.

Does not infer vendor current scaling, motor safety, thermal qualification or
plant dynamics. A 200 Hz sampled log cannot prove the total CAN RX frame rate.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

REQUIRED = ('t_ns', 'rx_ns', 'axis', 'trial_id', 'phase', 'drive_mode',
            'q_ref_rad', 'v_ref_rad_s', 'q_rad', 'v_est_rad_s', 'clamped', 'guard_action')
REQUIRED_FLOATS = ('q_ref_rad', 'v_ref_rad_s', 'q_rad', 'v_est_rad_s')
OPTIONAL_FLOATS = ('u_applied_a', 'iq_a', 'temperature_c', 'bus_voltage_v')


def percentile(values: list[float], p: float) -> float | None:
    if not 0 <= p <= 100 or not math.isfinite(p):
        raise ValueError('percentile must be 0..100')
    if not values:
        return None
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite observation')
    s = sorted(values)
    x = (len(s) - 1) * p / 100.0
    low = int(math.floor(x))
    high = int(math.ceil(x))
    return s[low] + (s[high] - s[low]) * (x - low)


def distribution(values: list[float]) -> dict[str, float | None]:
    return {f'p{p}': percentile(values, p) for p in (50, 95, 99)}


def _number(text: str | None, key: str, line: int, optional: bool = False) -> float | None:
    if text is None or not text.strip():
        if optional:
            return None
        raise ValueError(f'line {line}: missing {key}')
    try:
        value = float(text)
    except ValueError as exc:
        raise ValueError(f'line {line}: invalid {key}') from exc
    if not math.isfinite(value):
        raise ValueError(f'line {line}: {key} must be finite or blank if optional')
    return value


def load_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    last_t: dict[str, int] = {}
    last_rx: dict[str, int] = {}
    with path.open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError('CSV has no header')
        if len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise ValueError('duplicate CSV columns')
        missing = set(REQUIRED) - set(reader.fieldnames)
        if missing:
            raise ValueError('missing columns: ' + ', '.join(sorted(missing)))
        for line, raw in enumerate(reader, start=2):
            if None in raw:
                raise ValueError(f'line {line}: excess CSV values')
            row: dict[str, Any] = dict(raw)
            for key in ('t_ns', 'rx_ns'):
                text = raw.get(key)
                try:
                    val = int(text or '')
                except ValueError as exc:
                    raise ValueError(f'line {line}: {key} must be integer ns') from exc
                if val <= 0:
                    raise ValueError(f'line {line}: {key} must be positive')
                row[key] = val
            axis = row['axis']
            if axis not in ('yaw', 'pitch'):
                raise ValueError(f'line {line}: unknown axis')
            if row['drive_mode'] not in ('current', 'speed', 'position', 'voltage'):
                raise ValueError(f'line {line}: unsupported drive_mode label')
            if not row['trial_id'] or not row['phase'] or not row['guard_action']:
                raise ValueError(f'line {line}: empty phase/trial/guard label')
            if axis in last_t and row['t_ns'] <= last_t[axis]:
                raise ValueError(f'line {line}: control time not strictly increasing per axis')
            if axis in last_rx and row['rx_ns'] < last_rx[axis]:
                raise ValueError(f'line {line}: RX time reversed; split trace at generation reset')
            last_t[axis], last_rx[axis] = row['t_ns'], row['rx_ns']
            for key in REQUIRED_FLOATS:
                row[key] = _number(raw.get(key), key, line)
            for key in OPTIONAL_FLOATS:
                row[key] = _number(raw.get(key), key, line, optional=True)
            if raw.get('clamped') not in ('0', '1'):
                raise ValueError(f'line {line}: clamped must be 0 or 1')
            row['clamped'] = int(raw['clamped'])
            if row['drive_mode'] in ('speed', 'position') and row['u_applied_a'] is not None:
                raise ValueError(f'line {line}: speed/position host command is not current; leave u_applied_a blank')
            if row['drive_mode'] == 'voltage' and row['u_applied_a'] is not None:
                raise ValueError(f'line {line}: voltage command cannot be labeled amperes')
            rows.append(row)
    if not rows:
        raise ValueError('CSV has no samples')
    return rows


def weighted_rms(rows: list[dict[str, Any]], values: list[float | None]) -> dict[str, float | None]:
    """ZOH time-weighted RMS; missing values excluded, with coverage exposed.

    No extrapolation after the last sample. No RMS value for a one-row trace.
    """
    if len(values) != len(rows):
        raise ValueError('sample/value length mismatch')
    total = valid = numerator = 0.0
    for i in range(len(rows) - 1):
        dt = (rows[i + 1]['t_ns'] - rows[i]['t_ns']) * 1e-9
        if dt <= 0:
            raise ValueError('nonpositive sample interval')
        total += dt
        if values[i] is not None:
            value = values[i]
            if not math.isfinite(value):
                raise ValueError('nonfinite RMS input')
            numerator += value * value * dt
            valid += dt
    return {'rms': math.sqrt(numerator / valid) if valid > 0 else None,
            'coverage_fraction': valid / total if total > 0 else 0.0}


def temperature_slope(rows: list[dict[str, Any]]) -> float | None:
    points = [(r['t_ns'], r['temperature_c']) for r in rows if r['temperature_c'] is not None]
    if len(points) < 2:
        return None
    origin = points[0][0]
    t = [(ts - origin) * 1e-9 / 60 for ts, _ in points]
    mt = sum(t) / len(t)
    mv = sum(v for _, v in points) / len(points)
    den = sum((x - mt) ** 2 for x in t)
    return (sum((x - mt) * (v - mv) for x, (_, v) in zip(t, points)) / den
            if den > 0 else None)


def summarize_group(rows: list[dict[str, Any]], motion_threshold_rad: float) -> dict[str, Any]:
    if not rows or not math.isfinite(motion_threshold_rad) or motion_threshold_rad <= 0:
        raise ValueError('rows and positive motion threshold required')
    duration = (rows[-1]['t_ns'] - rows[0]['t_ns']) * 1e-9
    periods = [(b['t_ns'] - a['t_ns']) / 1e6 for a, b in zip(rows, rows[1:])]
    rx = sorted({r['rx_ns'] for r in rows})
    rx_duration = (rx[-1] - rx[0]) * 1e-9 if len(rx) > 1 else 0.0
    ages = [(r['t_ns'] - r['rx_ns']) / 1e6 for r in rows]
    q_error = [r['q_ref_rad'] - r['q_rad'] for r in rows]
    v_error = [r['v_ref_rad_s'] - r['v_est_rad_s'] for r in rows]
    # Only first crossing, NOT sustained onset detection and NOT a safety verdict.
    first_demand = next((i for i, r in enumerate(rows) if abs(r['v_ref_rad_s']) > 1e-9
                         or abs(r['q_ref_rad'] - r['q_rad']) >= motion_threshold_rad), None)
    latency = None
    if first_demand is not None:
        start = rows[first_demand]
        request = start['v_ref_rad_s'] or (start['q_ref_rad'] - start['q_rad'])
        direction = 1 if request > 0 else -1
        for r in rows[first_demand:]:
            if direction * (r['q_rad'] - start['q_rad']) >= motion_threshold_rad:
                latency = (r['t_ns'] - start['t_ns']) / 1e6
                break
    actions: dict[str, int] = defaultdict(int)
    prev = None
    for r in rows:
        if r['guard_action'] != prev:
            actions[r['guard_action']] += 1
        prev = r['guard_action']
    # A squared 0/1 RMS is the ZOH time fraction.
    clamp_rms = weighted_rms(rows, [float(r['clamped']) for r in rows])['rms']
    voltages = [r['bus_voltage_v'] for r in rows if r['bus_voltage_v'] is not None]
    return {
        'axis': rows[0]['axis'], 'trial_id': rows[0]['trial_id'], 'phase': rows[0]['phase'],
        'drive_modes': sorted({r['drive_mode'] for r in rows}),
        'samples': len(rows), 'duration_s': duration,
        'capture_hz': (len(rows) - 1) / duration if duration > 0 else None,
        'capture_period_ms': distribution(periods),
        'observed_distinct_feedback_hz_NOT_total_CAN_hz': (len(rx) - 1) / rx_duration if rx_duration > 0 else None,
        'feedback_age_ms': distribution(ages),
        'rx_later_than_cycle_start_rows': sum(age < 0 for age in ages),
        'position_error_rad': weighted_rms(rows, q_error),
        'velocity_error_rad_s': weighted_rms(rows, v_error),
        'peak_abs_position_error_rad': max(abs(x) for x in q_error),
        'current_command_a': weighted_rms(rows, [r['u_applied_a'] for r in rows]),
        'measured_current_a': weighted_rms(rows, [r['iq_a'] for r in rows]),
        'clamped_time_fraction': clamp_rms ** 2 if clamp_rms is not None else None,
        'first_motion_threshold_crossing_ms_NOT_confirmed_onset': latency,
        'motion_threshold_rad': motion_threshold_rad,
        'guard_label_episode_counts': dict(actions),
        'temperature_linear_slope_c_per_min_NOT_equilibrium_proof': temperature_slope(rows),
        'bus_voltage_min_v': min(voltages) if voltages else None,
        'bus_voltage_max_v': max(voltages) if voltages else None,
    }


def analyze(rows: list[dict[str, Any]], threshold: float) -> dict[str, Any]:
    # Split consecutive runs, not all equal labels: joining disjoint phases would
    # integrate missing time as if it had been observed.
    by_axis: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        by_axis[row['axis']].append(row)
    groups: list[list[dict[str, Any]]] = []
    for axis_rows in by_axis.values():
        current: list[dict[str, Any]] = []
        key = None
        for row in axis_rows:
            next_key = (row['trial_id'], row['phase'])
            if current and next_key != key:
                groups.append(current)
                current = []
            current.append(row)
            key = next_key
        if current:
            groups.append(current)
    return {'kind': 'OFFLINE_TRACE_STATISTICS_NOT_QUALIFICATION',
            'hardware_qualified_by_this_tool': False,
            'notes': ['Current engineering units must be independently validated before populating iq_a.',
                      'Blank current/temperature is unknown, not zero.',
                      'Observed distinct feedback rate is limited by logging rate; use CAN counters for actual bus rate.',
                      'No automatic pass/fail, thermal equilibrium certification, or step settling-time claim.'],
            'groups': [summarize_group(group, threshold) for group in groups]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--motion-threshold-rad', type=float, default=3 * 2 * math.pi / 8192,
                        help='First-crossing threshold; default three GM counts, not a pitch calibration')
    args = parser.parse_args()
    try:
        result = analyze(load_rows(args.csv), args.motion_threshold_rad)
        text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text, encoding='utf-8')
        else:
            print(text, end='')
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
