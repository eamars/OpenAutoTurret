#!/usr/bin/env python3
"""Read-only audit of ADR-002.2-model-evidence-20261001.zip.

Requires Python 3.10+ and NumPy. Does not load project code, simulate a plant,
fit a model, access a network, or communicate with hardware. Recomputes errors
from the exported native observations and predictions; checks reported claims.
A successful audit means internally reproducible evidence, NOT qualified motion.
"""
from __future__ import annotations
import argparse
import hashlib
import io
import json
import math
import sys
import zipfile
from pathlib import Path
from typing import Any
import numpy as np


class AuditError(RuntimeError):
    pass


def require(test: bool, message: str) -> None:
    if not test:
        raise AuditError(message)


def rms(x: np.ndarray) -> float:
    require(x.size > 0, "Cannot compute RMS of an empty channel")
    return float(np.sqrt(np.mean(np.square(x))))


def npz(z: zipfile.ZipFile, member: str) -> dict[str, np.ndarray]:
    with np.load(io.BytesIO(z.read(member)), allow_pickle=False) as f:
        return {k: f[k] for k in f.files}


def js(z: zipfile.ZipFile, member: str) -> Any:
    return json.loads(z.read(member))


def validate_observations(o: dict[str, np.ndarray], name: str,
                          channels: tuple[str, str, str]) -> None:
    t = o['t']
    require(t.ndim == 1 and t.size > 0 and bool(np.isfinite(t).all()), name + ': invalid times')
    require(bool((np.diff(t) > 0).all()), name + ': union timestamps not strictly increasing')
    for ch, mask in zip(channels, ('q_new', 'v_new', 'current_new')):
        require(o[ch].shape == t.shape and o[mask].shape == t.shape, name + ': shape mismatch')
        require(o[mask].dtype == np.bool_, name + ': mask is not boolean')
        require(bool(o[mask].any()) and bool(np.isfinite(o[ch][o[mask]]).all()), name + ': invalid native channel')
    require(o['tx_t'].ndim == 1 and o['tx_t'].size > 0, name + ': no TX history')
    require(o['tx_t'].shape == o['tx_A'].shape, name + ': TX shape mismatch')
    require(bool(np.isfinite(o['tx_t']).all()) and bool(np.isfinite(o['tx_A']).all()), name + ': invalid TX')
    require(bool((np.diff(o['tx_t']) >= 0).all()), name + ': unordered TX')
    require(float(o['tx_t'][0]) <= float(t[0]), name + ': initial TX history missing')


def compare(o: dict[str, np.ndarray], p: dict[str, np.ndarray],
            channels: tuple[str, str, str]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for tag, ch, mask, pred in zip(
            ('q', 'v', 'current'), channels, ('q_new', 'v_new', 'current_new'),
            ('encoder_prediction_rad', 'gyro_prediction_rad_s', 'reported_current_prediction_A')):
        y = o[ch][o[mask]]
        yh = p[pred]
        require(y.shape == yh.shape, f'{ch}: prediction shape mismatch')
        require(bool(np.isfinite(yh).all()), f'{ch}: non-finite prediction')
        e = yh - y
        out[tag] = {'rms': rms(e), 'mean': float(np.mean(e)),
                    'max_abs': float(np.max(np.abs(e))), 'endpoint_error': float(e[-1]),
                    'native_samples': int(y.size)}
    q = o[channels[0]][o['q_new']]
    qhat = p['encoder_prediction_rad']
    out['observed_q_span_deg'] = math.degrees(float(np.ptp(q)))
    out['predicted_q_span_deg'] = math.degrees(float(np.ptp(qhat)))
    out['initial_observation_constant_q_rms_deg'] = math.degrees(rms(q[0] - q))
    out['prediction_initial_constant_q_rms_deg'] = math.degrees(rms(qhat[0] - q))
    out['q_rms_deg'] = math.degrees(out['q']['rms'])
    out['v_rms_deg_s'] = math.degrees(out['v']['rms'])
    return out


def close(actual: float, expected: float, name: str, differences: list[float]) -> None:
    differences.append(abs(actual - expected))
    require(math.isclose(actual, expected, rel_tol=1e-10, abs_tol=1e-12),
            f'{name}: recomputed {actual} != reported {expected}')


def run(archive: Path, output: Path) -> dict[str, Any]:
    require(archive.is_file(), 'Evidence ZIP does not exist')
    output.mkdir(parents=True, exist_ok=True)
    differences: list[float] = []
    rows: list[dict[str, Any]] = []
    opt_rows: list[dict[str, Any]] = []
    obs_rows: list[dict[str, Any]] = []
    with zipfile.ZipFile(archive) as z:
        require(len(z.namelist()) == len(set(z.namelist())), 'Duplicate ZIP members')
        fmt = js(z, 'DATA_FORMAT.json')
        physical = fmt['physical_runs']
        observed: dict[str, dict[str, np.ndarray]] = {}
        for name, meta in physical.items():
            o = npz(z, meta['observations'])
            validate_observations(o, name, ('encoder_q', 'gyro_v', 'decoded_current'))
            observed[name] = o
            qq, vv = o['encoder_q'][o['q_new']], o['gyro_v'][o['v_new']]
            # Descriptive threshold, not an excitation-onset classification or safety limit.
            changed = np.flatnonzero(np.abs(o['tx_A'] - o['tx_A'][0]) > 0.01)
            obs_rows.append({'run_id': name, 'role': meta['role'],
                'duration_s': float(o['t'][-1]-o['t'][0]),
                'encoder_samples': int(o['q_new'].sum()), 'gyro_samples': int(o['v_new'].sum()),
                'current_samples': int(o['current_new'].sum()), 'tx_events': int(o['tx_t'].size),
                'q_span_deg': math.degrees(float(np.ptp(qq))),
                'q_min_deg': math.degrees(float(np.min(qq))), 'q_max_deg': math.degrees(float(np.max(qq))),
                'gyro_min_deg_s': math.degrees(float(np.min(vv))),
                'gyro_max_deg_s': math.degrees(float(np.max(vv))),
                'first_tx_departure_gt_0p01_A_s': float(o['tx_t'][changed[0]]) if changed.size else None})
        results = {mode: js(z, f'results/{mode}.json') for mode in ('constant', 'affine')}
        for mode, result in results.items():
            require(result['selected_model'] is None and not result['deployable'], 'Unexpected promoted model')
            require(result['final_holdout'] == [], 'Final holdout unexpectedly evaluated')
            for candidate in result['comparisons']:
                op = candidate['optimizer']
                sensitivity = op['unmodified_objective_sensitivity']
                sv = dict(zip(op['coordinates'], sensitivity['column_norms']))
                opt_rows.append({'label': candidate['label'], 'reported_solver_success': op['success'],
                    'reported_evaluations': op['evaluations'], 'reported_message': op['message'],
                    'estimated_coordinates': op['coordinates'],
                    'static_excess_sensitivity_norms': {k: v for k,v in sv.items() if 'static_excess' in k},
                    'load_offset': candidate['model']['load_offset'],
                    'q_min_numerical_only': candidate['model']['q_min'],
                    'q_max_numerical_only': candidate['model']['q_max']})
                for role, key in (('train', 'training'), ('selection', 'selection')):
                    for reported in candidate[key]:
                        name = reported['run_id']
                        require(physical[name]['role'] == role, 'Split role mismatch')
                        member = f'predictions/{mode}/{candidate["label"]}/{name}.npz'
                        pred = npz(z, member)
                        metrics = compare(observed[name], pred, ('encoder_q', 'gyro_v', 'decoded_current'))
                        for tag, key2 in (('q', 'q_rms_rad'), ('v','v_rms_rad_s'), ('current','current_rms_A')):
                            close(metrics[tag]['rms'], reported[key2], member + ':' + key2, differences)
                        for k in ('q_rms_deg','v_rms_deg_s'):
                            close(metrics[k], reported[k], member + ':' + k, differences)
                        require(reported['state_resets'] == 0, 'Reported physical run has state resets')
                        rows.append({'label': candidate['label'], 'run_id': name, 'role': role,
                            'source_prediction_member': member, 'reported_passed': reported['passed'],
                            'reported_failures': reported['metric_failures'], **metrics})
        synth = js(z, 'estimator/summary.json')
        synth_rows = []
        for c in synth['cases']:
            errors = {k: (c['fitted_parameters'][k]-v)/v for k,v in c['true_parameters'].items()}
            for k, e in errors.items():
                close(e, c['relative_parameter_error'][k], c['case'] + ':' + k, differences)
            synth_rows.append({'case': c['case'], 'max_abs_parameter_relative_error': max(map(abs,errors.values())),
                'max_abs_parameter_percent_error': 100*max(map(abs,errors.values())),
                'parameter_gate_passed': all(abs(e) <= c['recovery_limit'] for e in errors.values()),
                'parameter_gate_limit': c['recovery_limit'], 'reported_solver_success': c['optimizer']['success'],
                'reported_evaluations': c['optimizer']['evaluations'],
                'reported_trajectory_passes': sum(p['trajectory_gate']['passed'] for p in c['predictions']),
                'reported_trajectory_count': len(c['predictions']),
                'reported_case_passed': c['synthetic_case_gate_passed']})
        case41 = next(c for c in synth['cases'] if c['case']=='noisy-seed-41')
        cross_rows = []
        for seed in (17,83):
            member = f'estimator/seed41-predict-seed{seed}.npz'
            o = npz(z, f'estimator/noisy-seed-{seed}.npz')
            validate_observations(o, member, ('q','v','current'))
            m = compare(o, npz(z, member), ('q','v','current'))
            reported = next(p for p in case41['predictions'] if p['source']==f'noisy-seed-{seed}')
            for ch in ('q','v','current'):
                close(m[ch]['rms'], reported['prediction_errors'][ch]['rms'], member+':'+ch, differences)
            limits = reported['trajectory_gate']['thresholds']
            passed = {ch: m[ch]['rms'] <= limits[ch] for ch in limits}
            cross_rows.append({'source_member': member, 'target_seed':seed,
                               'thresholds':limits, 'channel_passed':passed, **m})
        no_motion = [r for r in rows if r['run_id']=='yaw-descended-03' and 'stribeck-affine' in r['label']]
        algebraic = results['constant']['comparisons'][0]['model']
        coulomb_threshold_example = {'source': 'results/constant.json#/comparisons/0/model',
            'static_negative_A_equivalent': algebraic['static_negative'],
            'static_positive_A_equivalent': algebraic['static_positive'],
            'probe_min_tx_A': float(np.min(observed['yaw-physical-probe-01']['tx_A'])),
            'probe_max_tx_A': float(np.max(observed['yaw-physical-probe-01']['tx_A']))}
        noisy = [x for x in synth_rows if x['case'].startswith('noisy')]
        summary = {'audit_scope': 'RECOMPUTED_EXPORTED_TRACE_ERRORS_AND_INSPECTED_REPORTED_METADATA',
            'does_not_rerun_original_optimizer_or_simulator': True,
            'source_archive_name': archive.name,
            'source_archive_sha256_generated_by_this_audit': hashlib.sha256(archive.read_bytes()).hexdigest(),
            'source_member_count': len(z.namelist()),
            'physical_observation_run_count': len(observed),
            'physical_prediction_count': len(rows),
            'physical_reported_passes': sum(r['reported_passed'] for r in rows),
            'metrics_comparison_count': len(differences),
            'max_absolute_difference_recomputed_vs_reported_numeric_metrics': max(differences),
            'reported_solver_success_count': sum(o['reported_solver_success'] for o in opt_rows),
            'reported_physical_solver_budget_exhaustions': sum(not o['reported_solver_success'] and o['reported_evaluations']==200 for o in opt_rows),
            'noisy_synthetic_parameter_gate_passes': sum(r['parameter_gate_passed'] for r in noisy),
            'noisy_synthetic_solver_successes': sum(r['reported_solver_success'] for r in noisy),
            'noisy_synthetic_reported_trajectory_gate_passes': sum(r['reported_trajectory_passes'] for r in noisy),
            'noisy_synthetic_reported_trajectory_count': sum(r['reported_trajectory_count'] for r in noisy),
            'no_motion_selection_cases': no_motion,
            'coulomb_threshold_example': coulomb_threshold_example,
            'all_checked_evidence_consistency_tests_passed': True,
            'qualification': 'UNQUALIFIED', 'deployment_authorized': False,
            'limitations': ['No physical experiments or new fitting performed.',
                'Original source, event transitions and actual actuator current semantics are not independently verified.',
                'Solver flags and sensitivity values are reported metadata, not rerun results.',
                'A local archive digest checks reproducibility of this input, not equivalence to unseen repository originals.']}
    files = {'audit_summary.json': summary, 'physical_predictions.json': rows,
             'observation_inventory.json': obs_rows, 'optimizer_metadata.json':opt_rows,
             'synthetic_gate_breakdown.json':synth_rows, 'synthetic_cross_predictions.json': cross_rows}
    for name,data in files.items():
        (output/name).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n', encoding='utf-8')
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    try:
        result = run(args.archive,args.output)
    except (OSError, ValueError, KeyError, zipfile.BadZipFile, AuditError) as exc:
        print(f'AUDIT_FAILED: {exc}',file=sys.stderr)
        return 2
    print(json.dumps({k:result[k] for k in ('physical_observation_run_count','physical_prediction_count',
        'metrics_comparison_count','all_checked_evidence_consistency_tests_passed','qualification',
        'deployment_authorized')},indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
