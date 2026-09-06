"""Summarize commissioning CSVs without confusing encoder velocity noise with motion.

Run with a project venv containing numpy and PyYAML. No hardware access.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import yaml


def spectral_peak(t, values):
    dt = float(np.median(np.diff(t)))
    uniform_t = np.arange(t[0], t[-1], dt)
    if len(uniform_t) < 16 or np.ptp(values) == 0:
        return None
    uniform = np.interp(uniform_t, t, values)
    uniform -= np.mean(uniform)
    window = np.hanning(len(uniform))
    amplitude = 2 * np.abs(np.fft.rfft(uniform * window)) / window.sum()
    amplitude[0] = 0
    index = int(np.argmax(amplitude))
    return {'hz': float(np.fft.rfftfreq(len(uniform), dt)[index]),
            'amplitude': float(amplitude[index]),
            'resolution_hz': 1 / (len(uniform) * dt)}


def analyze(directory, discard_s=1):
    directory = Path(directory)
    with (directory / 'samples.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) < 2:
        raise ValueError(f'{directory}: insufficient samples')
    data = {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}
    t = (data['t_ns'] - data['t_ns'][0]) * 1e-9
    keep = t >= discard_s
    if np.count_nonzero(keep) < 2:
        raise ValueError('discard interval leaves insufficient samples')
    steady_t, q, iq = t[keep], data['q_rad'][keep], data['iq_a'][keep]
    rad_to_deg = 180 / math.pi
    result = {
        'run': directory.name,
        'samples': len(rows),
        'sample_rate_hz': (len(t)-1) / t[-1],
        'max_sample_gap_ms': float(np.max(np.diff(t))*1000),
        'discard_startup_s': discard_s,
        'position_rms_deg': float(np.std(q)*rad_to_deg),
        'position_p2p_deg': float(np.ptp(q)*rad_to_deg),
        'position_drift_deg': float((np.median(q[-10:])-np.median(q[:10]))*rad_to_deg),
        'reported_velocity_rms_deg_s': float(np.sqrt(np.mean(data['v_rad_s'][keep]**2))*rad_to_deg),
        'iq_rms_a': float(np.sqrt(np.mean(iq**2))),
        'iq_peak_a': float(np.max(np.abs(iq))),
        'temperature_start_c': float(data['temperature_c'][0]),
        'temperature_end_c': float(data['temperature_c'][-1]),
        'position_spectral_peak': spectral_peak(steady_t, q*rad_to_deg),
        'iq_spectral_peak': spectral_peak(steady_t, iq),
        'fault_bits_observed': sorted(set(int(v) for v in data['fault_bits'])),
    }
    if (directory / 'result.yaml').exists():
        outcome = yaml.safe_load((directory / 'result.yaml').read_text())
        result.update({key: outcome.get(key) for key in
                       ('axis', 'kind', 'ok', 'restoration_verified', 'ended_disabled', 'error', 'interrupted')})
        result['hold_error_rms_deg'] = float(np.sqrt(np.mean((q-outcome['q_hold_rad'])**2))*rad_to_deg)
    if 'loc_ref_rad' in data:
        result['loc_ref_p2p_deg'] = float(np.ptp(data['loc_ref_rad'][keep])*rad_to_deg)
        result['reference_error_rms_deg'] = float(np.sqrt(np.mean(
            (q-data['q_ref_rad'][keep])**2))*rad_to_deg)
    if (directory / 'commands.csv').exists():
        with (directory / 'commands.csv').open() as handle:
            commands = list(csv.DictReader(handle))
        stamps = np.array([int(row['t_ns']) for row in commands], dtype=np.int64)
        result['command_request_rate_hz'] = (len(stamps)-1)*1e9 / (stamps[-1]-stamps[0])
        result['command_request_gap_p99_ms'] = float(np.percentile(np.diff(stamps)*1e-6, 99))
        result['command_rate_note'] = 'backend calls before caching; not actual CAN writes'
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--discard-startup-s', type=float, default=1)
    args = parser.parse_args()
    output = [analyze(path, args.discard_startup_s) for path in args.directories]
    text = json.dumps(output, indent=2, allow_nan=False)
    if args.output:
        args.output.write_text(text + '\n', encoding='utf-8')
    print(text)
