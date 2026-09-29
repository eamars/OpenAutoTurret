"""Evidence and reuse rules. OFFLINE validator, NOT a hardware attestation service."""
from __future__ import annotations
import hashlib
import json
import math
from datetime import datetime
from typing import Any

DIGEST_KEYS = (
    'hardware_signature', 'measurement_signature', 'core_build_hash', 'observer_hash',
    'model_hash', 'parameters_hash', 'metrics_hash', 'test_spec_hash', 'envelope_hash',
)


def canonical_hash(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False,
                         allow_nan=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def is_digest(value: Any) -> bool:
    return (isinstance(value, str) and len(value) == 64
            and all(ch in '0123456789abcdef' for ch in value))


def reuse_action(*, same_hardware: bool, same_measurement: bool,
                 covered_operating_point: bool, prediction_check_passed: bool) -> str:
    values = (same_hardware, same_measurement, covered_operating_point, prediction_check_passed)
    if any(type(v) is not bool for v in values):
        raise ValueError('reuse inputs must be measured boolean decisions, not truthy strings')
    if not same_hardware or not same_measurement:
        return 'INVALIDATE_AFFECTED_ASSETS'
    if covered_operating_point and prediction_check_passed:
        return 'REUSE_EXACT'
    return 'UPDATE_PARAMETERS'


def can_start(expected_parameters_hash: str, expected_revision: int,
              receipt: dict[str, Any], *, acquisition_ready: bool) -> bool:
    """Typed, positive acknowledgement required; a directory label is irrelevant."""
    return (is_digest(expected_parameters_hash)
            and type(expected_revision) is int and expected_revision >= 0
            and receipt.get('applied') is True and receipt.get('readback_verified') is True
            and receipt.get('parameters_hash') == expected_parameters_hash
            and type(receipt.get('revision')) is int
            and receipt['revision'] == expected_revision
            and acquisition_ready is True)


def bundle_reasons(profile: dict[str, Any], evidence: list[dict[str, Any]]) -> list[str]:
    """Check 3a+3b metadata bindings and case coverage for ONE candidate profile.

    Does not prove provenance or physical truth of user-supplied JSON. Production
    must bind these declarations to raw data, signed/session receipts and launcher.
    """
    reasons: list[str] = []
    if not isinstance(profile, dict) or not isinstance(evidence, list) or any(not isinstance(e, dict) for e in evidence):
        return ['INVALID_BUNDLE_TYPE']
    for name in DIGEST_KEYS:
        if not is_digest(profile.get(name)):
            reasons.append(f'PROFILE_INVALID_{name}')
    candidate = profile.get('candidate_id')
    if not isinstance(candidate, str) or not candidate.strip():
        reasons.append('PROFILE_INVALID_candidate_id')
    conditions = profile.get('conditions')
    axes = profile.get('axes')
    cases = profile.get('required_cases')
    for name, data in [('conditions', conditions), ('axes', axes), ('required_cases', cases)]:
        if (not isinstance(data, list) or not data or any(not isinstance(x, str) or not x for x in data)
                or len(data) != len(set(data))):
            reasons.append(f'PROFILE_INVALID_{name}')
    if reasons:
        return reasons
    if set(axes) != {'yaw', 'pitch'}:
        reasons.append('PROFILE_REQUIRES_BOTH_AXES')
    stages = {stage: [e for e in evidence if e.get('stage') == stage] for stage in ('3a', '3b')}
    for stage, certs in stages.items():
        if len(certs) != 1:
            reasons.append(f'{stage}_REQUIRES_EXACTLY_ONE_CERTIFICATE')
            continue
        cert = certs[0]
        prefix = f'{stage}_'
        expected_program = 'commissiond' if stage == '3a' else 'production'
        expected_route = 'independent_shared_core' if stage == '3a' else 'normal_production_chain'
        if cert.get('program') != expected_program or cert.get('route') != expected_route:
            reasons.append(prefix + 'WRONG_EXECUTION_ROUTE')
        if cert.get('execution') != 'physical' or cert.get('shadow') is not False:
            reasons.append(prefix + 'NOT_PHYSICAL_EXECUTION')
        try:
            start = datetime.fromisoformat(cert.get('started_at', '').replace('Z', '+00:00'))
            end = datetime.fromisoformat(cert.get('finished_at', '').replace('Z', '+00:00'))
            if start.tzinfo is None or end.tzinfo is None or end <= start:
                raise ValueError('invalid interval')
        except (ValueError, TypeError, AttributeError):
            reasons.append(prefix + 'INVALID_TIME_INTERVAL')
        if cert.get('status') != 'PASS':
            reasons.append(prefix + 'NOT_PASS')
        if cert.get('candidate_id') != candidate:
            reasons.append(prefix + 'CANDIDATE_MISMATCH')
        for name in DIGEST_KEYS:
            if cert.get(name) != profile[name]:
                reasons.append(prefix + 'MISMATCH_' + name)
        if cert.get('actual_parameters_hash') != profile['parameters_hash']:
            reasons.append(prefix + 'READBACK_MISMATCH')
        traces = cert.get('trace_hashes')
        if not isinstance(traces, list) or not traces or any(not is_digest(x) for x in traces):
            reasons.append(prefix + 'MISSING_RAW_TRACE')
        if not is_digest(cert.get('binary_hash')):
            reasons.append(prefix + 'MISSING_BINARY')
        if cert.get('unresolved_abort') is not False:
            reasons.append(prefix + 'UNRESOLVED_ABORT')
        if stage == '3b' and (cert.get('production_load_verified') is not True
                             or cert.get('lifecycle_verified') is not True
                             or cert.get('parity_passed') is not True):
            reasons.append(prefix + 'PRODUCTION_INCOMPLETE')
        coverage = cert.get('coverage')
        wanted = {(a, c, s) for a in axes for c in conditions for s in cases}
        found: set[tuple[str, str, str]] = set()
        if not isinstance(coverage, list):
            reasons.append(prefix + 'MISSING_COVERAGE')
        else:
            for row in coverage:
                if not isinstance(row, dict):
                    reasons.append(prefix + 'INVALID_COVERAGE_ROW'); continue
                key = (row.get('axis'), row.get('condition'), row.get('case'))
                if any(not isinstance(x, str) for x in key):
                    reasons.append(prefix + 'INVALID_COVERAGE_ROW'); continue
                if key in found:
                    reasons.append(prefix + 'DUPLICATE_COVERAGE')
                found.add(key)
                if row.get('status') != 'PASS' or row.get('valid_data') is not True:
                    reasons.append(prefix + 'CASE_NOT_PASS')
                if type(row.get('repetitions')) is not int or row.get('repetitions', 0) < 3:
                    reasons.append(prefix + 'INSUFFICIENT_REPETITIONS')
                if not is_digest(row.get('operating_point_hash')):
                    reasons.append(prefix + 'MISSING_OPERATING_POINT')
            if wanted - found:
                reasons.append(prefix + 'MISSING_CASES')
            if found - wanted:
                reasons.append(prefix + 'UNEXPECTED_CASES')
    # Matching labels do not suffice: the physical condition identity must match too.
    if len(stages['3a']) == len(stages['3b']) == 1:
        maps = []
        for stage in ('3a', '3b'):
            rows = stages[stage][0].get('coverage', [])
            maps.append({(r.get('axis'), r.get('condition'), r.get('case')):
                         r.get('operating_point_hash') for r in (rows if isinstance(rows, list) else [])
                         if isinstance(r, dict) and all(isinstance(r.get(k), str) for k in ('axis','condition','case'))})
        if maps[0] != maps[1]:
            reasons.append('CROSS_ENVIRONMENT_OPERATING_POINT_MISMATCH')
        bench, prod = stages['3a'][0], stages['3b'][0]
        try:
            expected_bench_hash = canonical_hash(bench)
        except (ValueError, TypeError):
            expected_bench_hash = None
            reasons.append('BENCH_CERTIFICATE_NOT_CANONICAL')
        if expected_bench_hash is None or prod.get('bench_certificate_hash') != expected_bench_hash:
            reasons.append('PRODUCTION_NOT_BOUND_TO_THIS_BENCH_CERTIFICATE')
        if bench.get('bench_certificate_hash') is not None:
            reasons.append('BENCH_MUST_NOT_REFERENCE_ANOTHER_BENCH')
        try:
            bench_end = datetime.fromisoformat(bench['finished_at'].replace('Z','+00:00'))
            prod_start = datetime.fromisoformat(prod['started_at'].replace('Z','+00:00'))
            if bench_end.tzinfo is None or prod_start.tzinfo is None or prod_start < bench_end:
                reasons.append('PRODUCTION_PRECEDES_BENCH_COMPLETION')
        except (ValueError, TypeError, KeyError, AttributeError):
            reasons.append('DUAL_VALIDATION_TIME_UNVERIFIABLE')
    return sorted(set(reasons))


def completion_reasons(requirements: list[dict[str, Any]], results: list[dict[str, Any]],
                       all_profile_bundles_pass: bool) -> list[str]:
    """No MVP completion: each mandatory requirement must have nonempty evidence."""
    reasons = []
    wanted = {r['id']: r for r in requirements if r.get('mandatory') is True}
    recorded: dict[str, dict[str, Any]] = {}
    for row in results:
        rid = row.get('id')
        if rid in recorded:
            reasons.append('DUPLICATE_REQUIREMENT_RESULT')
        recorded[rid] = row
    for rid, req in wanted.items():
        row = recorded.get(rid, {})
        if row.get('status') != 'PASS' or not row.get('evidence_ids'):
            reasons.append('UNMET_' + rid)
        if req.get('evidence_kind') == 'physical' and row.get('execution') != 'physical':
            reasons.append('NOT_PHYSICAL_' + rid)
    if all_profile_bundles_pass is not True:
        reasons.append('PROFILE_DUAL_VALIDATION_INCOMPLETE')
    return sorted(set(reasons))


def stage1_completion_reasons(requirements: list[dict[str, Any]], results: list[dict[str, Any]]) -> list[str]:
    """Architect override: mathematical completion has no physical prerequisites.

    This cannot grant hardware, calibration, Stage 2, 3a/3b or full ADR qualification.
    Historical S1-05/S1-06 IDs deliberately remain traceable but now belong to Stage 2.
    """
    required = [r for r in requirements if r.get('mandatory') is True and
                r.get('evidence_kind') == 'software' and r.get('stage') in ('1', 'all')]
    rows = {}
    reasons = []
    for row in results:
        rid = row.get('id')
        if rid in rows:
            reasons.append('DUPLICATE_REQUIREMENT_RESULT')
        rows[rid] = row
    for requirement in required:
        rid = requirement['id']; row = rows.get(rid, {})
        if (row.get('status') != 'PASS' or row.get('execution') not in ('software', 'synthetic')
                or not row.get('evidence_ids')):
            reasons.append('UNMET_' + rid)
    return sorted(set(reasons))
