#!/usr/bin/env python3
"""Offline reference checks for the N1 design, not production control code.

No camera, network, CAN, I2C, SSH, subprocess, model, or deployment access.
The schemas/fixtures use a proposed semantic format, not the repository ABI.
"""
from __future__ import annotations
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Iterable

VERSION = 'ota.n1.draft.selected/1'
NS_RE = re.compile(r'^(0|[1-9][0-9]*)$')
MAX_NS = (1 << 63) - 1
ANCHORS = {'face_box', 'head_box', 'body_upper', 'torso', 'marker_center'}


def ns(value: Any) -> int:
    """Strict decimal string -> signed-int64 nonnegative nanoseconds/sequence."""
    if not isinstance(value, str) or not NS_RE.fullmatch(value):
        raise ValueError('nanoseconds/sequence must be an unsigned decimal string')
    result = int(value)
    if result > MAX_NS:
        raise ValueError('nanoseconds/sequence exceeds int64')
    return result


def finite(value: Any) -> bool:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def decode_throttled(value: str | int) -> dict[str, Any]:
    """Decode the documented low/history flags, preserving unknown bits."""
    if isinstance(value, str):
        text = value.strip()
        match = re.fullmatch(r'(?:throttled=)?(0[xX][0-9a-fA-F]+)', text)
        if not match:
            raise ValueError('expected 0x... or throttled=0x...')
        raw = int(match.group(1), 16)
    elif isinstance(value, int) and not isinstance(value, bool):
        raw = value
    else:
        raise ValueError('invalid throttled value type')
    if not 0 <= raw <= 0xFFFFFFFF:
        raise ValueError('throttled value must fit uint32')
    labels = ('undervoltage', 'frequency_capped', 'throttled', 'soft_temperature_limit')
    current = {name: bool(raw & (1 << i)) for i, name in enumerate(labels)}
    history = {name: bool(raw & (1 << (16+i))) for i, name in enumerate(labels)}
    unknown = raw & ~0x000F000F & 0xFFFFFFFF
    state = ('current_fault' if any(current.values()) else
             'unknown_bits' if unknown else 'history_only' if any(history.values()) else 'clear')
    return {'raw_hex': f'0x{raw:x}', 'current': current, 'history': history,
            'unknown_bits_hex': f'0x{unknown:x}', 'state': state}


def invert_center_letterbox_yxyx(
    bbox: Iterable[float], *, source_width: int = 640, source_height: int = 480,
    model_width: int = 640, model_height: int = 640,
) -> tuple[float, float, float, float] | None:
    """Normalized model yxyx -> source xyxy for centered isotropic letterbox.

    This helper assumes only the declared center-letterbox operation, no lens
    distortion or additional crop/orientation. It does not replace calibration.
    """
    dims = (source_width, source_height, model_width, model_height)
    if any(not isinstance(x, int) or isinstance(x, bool) or x <= 0 for x in dims):
        raise ValueError('positive integer dimensions required')
    values = tuple(bbox)
    if len(values) != 4 or any(not finite(v) for v in values):
        raise ValueError('four finite box coordinates required')
    ymin, xmin, ymax, xmax = values
    if ymin >= ymax or xmin >= xmax:
        return None
    scale = min(model_width/source_width, model_height/source_height)
    pad_x = (model_width-source_width*scale)/2
    pad_y = (model_height-source_height*scale)/2
    x1 = max(0.0, min(float(source_width), (xmin*model_width-pad_x)/scale))
    x2 = max(0.0, min(float(source_width), (xmax*model_width-pad_x)/scale))
    y1 = max(0.0, min(float(source_height), (ymin*model_height-pad_y)/scale))
    y2 = max(0.0, min(float(source_height), (ymax*model_height-pad_y)/scale))
    return (x1, y1, x2, y2) if x1 < x2 and y1 < y2 else None


def rotate_180_pixel(u: float, v: float, width: int, height: int) -> tuple[float, float]:
    """Pixel-center convention: centers occupy 0..width-1 and 0..height-1."""
    if any(not isinstance(d, int) or isinstance(d, bool) or d <= 0 for d in (width, height)):
        raise ValueError('positive dimensions required')
    if not finite(u) or not finite(v) or not 0 <= u <= width-1 or not 0 <= v <= height-1:
        raise ValueError('pixel center outside declared raster')
    return width-1-u, height-1-v


def observation_rejections(obs: dict[str, Any], context: dict[str, Any]) -> list[str]:
    """Illustrative semantic validation, not a motor authorization mechanism."""
    if not isinstance(obs, dict) or not isinstance(context, dict):
        return ['malformed_object']
    required = ('schema_version', 'station_session_id', 'boot_id', 'clock_id', 'clock_epoch',
                'camera_id', 'camera_generation', 'source_frame_sequence', 'frame_id',
                'model_id', 'calibration_id', 'sensor_mode_id', 'transform_chain_id',
                't_observation_ns', 't_capture_received_ns', 't_publish_ns', 'valid_until_ns',
                'timestamp_uncertainty_ns', 'observation_time_quality', 'geometry_valid',
                'selection', 'anchor', 'person_confidence', 'association_confidence')
    missing = [f'missing:{k}' for k in required if k not in obs]
    if missing:
        return missing
    reasons: list[str] = []
    identifiers = ('station_session_id', 'boot_id', 'camera_id', 'frame_id', 'model_id',
                   'calibration_id', 'sensor_mode_id', 'transform_chain_id')
    if any(not isinstance(obs[k], str) or not obs[k].strip() for k in identifiers):
        return ['invalid_identifier']
    if (not isinstance(context.get('cameras'), dict) or
            not isinstance(context.get('allowed_camera_ids'), list) or
            any(not isinstance(x, str) for x in context['allowed_camera_ids'])):
        return ['invalid_camera_context']
    if obs['schema_version'] != VERSION:
        reasons.append('unsupported_schema')
    for field in ('station_session_id', 'boot_id', 'clock_id', 'clock_epoch'):
        if field not in context or obs[field] != context[field]:
            reasons.append(f'mismatch:{field}')
    for field in ('camera_generation', 'clock_epoch'):
        if not isinstance(obs[field], int) or isinstance(obs[field], bool) or obs[field] < 0:
            reasons.append(f'invalid:{field}')
    if obs['clock_id'] != 'CLOCK_MONOTONIC':
        reasons.append('unmapped_clock')
    try:
        times = {k: ns(obs[k]) for k in ('t_observation_ns', 't_capture_received_ns',
                                       't_publish_ns', 'valid_until_ns', 'timestamp_uncertainty_ns')}
        now = ns(context['now_ns'])
        max_age = ns(context['max_observation_age_ns'])
        max_uncertainty = ns(context['max_timestamp_uncertainty_ns'])
        seq = ns(obs['source_frame_sequence'])
    except (KeyError, ValueError, TypeError):
        return reasons + ['invalid_time_or_context']
    t = times['t_observation_ns']
    if not t <= times['t_capture_received_ns'] <= times['t_publish_ns'] <= now:
        reasons.append('noncausal_or_future_time')
    if now - t > max_age:
        reasons.append('stale_observation')
    if times['valid_until_ns'] <= t or now >= times['valid_until_ns']:
        reasons.append('expired_observation')
    if times['valid_until_ns'] - t > max_age:
        reasons.append('ttl_exceeds_policy')
    if times['timestamp_uncertainty_ns'] > max_uncertainty:
        reasons.append('timestamp_uncertainty_excessive')
    if obs['observation_time_quality'] != 'verified':
        reasons.append('unverified_timestamp')
    if obs['geometry_valid'] is not True:
        reasons.append('unqualified_geometry')
    camera = context.get('cameras', {}).get(obs['camera_id'])
    if not isinstance(camera, dict):
        reasons.append('unknown_camera')
    else:
        for field in ('camera_generation', 'model_id', 'calibration_id', 'sensor_mode_id', 'transform_chain_id'):
            if obs[field] != camera.get(field):
                reasons.append(f'mismatch:{field}')
        if camera.get('motion_qualified') is not True:
            reasons.append('camera_not_qualified')
        last = camera.get('last_frame_sequence')
        if last is not None:
            try:
                if seq <= ns(last):
                    reasons.append('duplicate_or_old_frame')
            except ValueError:
                reasons.append('invalid_context_sequence')
    if obs['camera_id'] not in context.get('allowed_camera_ids', []):
        reasons.append('camera_not_authoritative')
    expected_selection = context.get('selection')
    selection = obs['selection']
    if not isinstance(selection, dict) or not isinstance(expected_selection, dict):
        reasons.append('invalid_selection')
    else:
        for field in ('target_session_id', 'selection_generation', 'policy', 'global_track_id'):
            if field not in selection or selection[field] != expected_selection.get(field):
                reasons.append(f'selection_mismatch:{field}')
        if selection.get('policy') not in ('AUTO_SINGLE', 'EXPLICIT_TRACK', 'EXPLICIT_TAG'):
            reasons.append('invalid_selection_policy')
        if any(not isinstance(selection.get(k), str) or not selection[k].strip()
               for k in ('target_session_id', 'global_track_id')):
            reasons.append('invalid_selection_identifier')
        generation = selection.get('selection_generation')
        if not isinstance(generation, int) or isinstance(generation, bool) or generation < 0:
            reasons.append('invalid_selection_generation')
    anchor = obs['anchor']
    if not isinstance(anchor, dict):
        reasons.append('invalid_anchor')
    else:
        if not isinstance(anchor.get('kind'), str) or anchor.get('kind') not in ANCHORS:
            reasons.append('invalid_anchor_kind')
        w, h, u, v = (anchor.get(k) for k in ('raster_width', 'raster_height', 'u_px', 'v_px'))
        valid_dims = all(isinstance(d, int) and not isinstance(d, bool) and d > 0 for d in (w, h))
        if not valid_dims or not finite(u) or not finite(v) or not (0 <= u <= w-1 and 0 <= v <= h-1):
            reasons.append('invalid_anchor_raster')
        score = anchor.get('confidence')
        if not finite(score) or not 0 <= score <= 1:
            reasons.append('invalid_anchor_confidence')
    for field in ('person_confidence', 'association_confidence'):
        score = obs[field]
        if not finite(score) or not 0 <= score <= 1:
            reasons.append(f'invalid:{field}')
    return reasons


@dataclass
class SelectionLatch:
    """Small policy example; deliberately does not implement dwell/association."""
    generation: int = 0
    policy: str = 'AUTO_SINGLE'
    target: str | None = None

    def explicit_select(self, target: str) -> None:
        if not isinstance(target, str) or not target.strip():
            raise ValueError('nonempty target required')
        self.generation += 1
        self.policy = 'EXPLICIT_TRACK'
        self.target = target

    def on_mode_change(self, mode: str) -> None:
        if mode not in {'MANUAL', 'AUTO_TRACK', 'AUTO_ROAM'}:
            raise ValueError('unknown normal mode')
        # Intentional: changing motion mode does NOT clear explicit selection.

    def candidate_allowed(self, candidate: str) -> bool:
        return self.policy == 'AUTO_SINGLE' or candidate == self.target

    def cancel(self) -> None:
        self.generation += 1
        self.policy = 'AUTO_SINGLE'
        self.target = None


def evaluate_stop_evidence(evidence: dict[str, Any], max_feedback_age_ms: float = 100.0) -> dict[str, Any]:
    """Assess supplied evidence only; never infer torque-off from zero request."""
    if not isinstance(evidence, dict) or not finite(max_feedback_age_ms) or max_feedback_age_ms <= 0:
        raise ValueError('invalid evidence or age policy')
    missing: list[str] = []
    for axis in ('yaw', 'pitch'):
        data = evidence.get(axis, {})
        if not isinstance(data, dict):
            data = {}
        age = data.get('feedback_age_ms')
        if not finite(age) or not 0 <= age <= max_feedback_age_ms:
            missing.append(f'{axis}:fresh_feedback')
        if data.get('stationary_observed') is not True:
            missing.append(f'{axis}:stationary_observation')
        if axis == 'yaw':
            if data.get('zero_requested') is not True:
                missing.append('yaw:zero_request')
            if data.get('disable_confirmed') is not None:
                missing.append('yaw:unsupported_disable_claim')
        else:
            if data.get('disable_requested') is not True or data.get('disable_confirmed') is not True:
                missing.append('pitch:disable_confirmation')
    return {'completion_quality': 'limited_complete' if not missing else 'incomplete',
            'missing_evidence': missing, 'yaw_disable_confirmed': None,
            'power_isolated_confirmed': None,
            'note': 'Stationarity and command evidence do not certify de-energization or load support.'}


def quantiles(values: Iterable[float]) -> dict[str, Any]:
    items = list(values)
    if any(not finite(x) or x < 0 for x in items):
        raise ValueError('latencies must be finite and nonnegative')
    items.sort()
    def percentile(p: float) -> float | None:
        return float(items[max(0, math.ceil(p*len(items))-1)]) if items else None
    return {'n': len(items), 'p50_ms': percentile(.50), 'p95_ms': percentile(.95),
            'p99_ms': percentile(.99), 'max_ms': float(items[-1]) if items else None,
            'method': 'nearest_rank'}


def strict_json(text: str) -> Any:
    def reject_constant(x: str) -> Any:
        raise ValueError(f'non-finite JSON constant: {x}')
    def parse_finite_float(text: str) -> float:
        number = float(text)
        if not math.isfinite(number):
            raise ValueError('JSON number overflows finite float')
        return number
    def unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f'duplicate JSON key: {key}')
            result[key] = value
        return result
    return json.loads(text, parse_constant=reject_constant, parse_float=parse_finite_float,
                      object_pairs_hook=unique_object)


def read_ndjson(path: Path) -> list[dict[str, Any]]:
    result = []
    with path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            if len(line.encode('utf-8')) > 1_048_576:
                raise ValueError(f'{path}:{line_number}: record exceeds 1 MiB')
            try:
                item = strict_json(line)
                if not isinstance(item, dict):
                    raise ValueError('record must be an object')
                result.append(item)
            except (ValueError, json.JSONDecodeError) as exc:
                raise ValueError(f'{path}:{line_number}: {exc}') from exc
    return result


def summarize_trace(records: Iterable[dict[str, Any]]) -> dict[str, Any]:
    """Join by session/boot/clock-epoch/camera/camera-generation/frame, never across clock resets.

    Requires proposed event names; it is not an adapter for existing station CSV.
    Incomplete or ambiguous lineages are counted rather than imputed as zero.
    """
    order = ('observation_reference', 'capture_received', 'inference_start', 'inference_end',
             'publish', 'controller_receive')
    pairs = {'capture_to_controller': ('observation_reference', 'controller_receive'),
             'host_to_publish': ('capture_received', 'publish'),
             'inference': ('inference_start', 'inference_end'),
             'publish_to_controller': ('publish', 'controller_receive')}
    groups: dict[tuple[Any, ...], dict[str, dict[str, Any]]] = {}
    duplicate_groups: set[tuple[Any, ...]] = set()
    unknown_events = 0
    for record in records:
        try:
            if not isinstance(record, dict):
                raise ValueError('trace record must be an object')
            if any(not isinstance(record.get(k), str) or not record[k].strip()
                   for k in ('station_session_id', 'boot_id', 'camera_id', 'frame_id', 'event')):
                raise ValueError('trace identifiers and event must be nonempty strings')
            for generation_name in ('clock_epoch', 'camera_generation'):
                epoch = record.get(generation_name)
                if not isinstance(epoch, int) or isinstance(epoch, bool) or epoch < 0:
                    raise ValueError(f'trace {generation_name} must be a nonnegative integer')
            key = tuple(record[k] for k in ('station_session_id', 'boot_id', 'clock_epoch', 'camera_id', 'camera_generation', 'frame_id'))
            event = record['event']
            ns(record['t_ns'])
            if record.get('clock_id') != 'CLOCK_MONOTONIC':
                raise ValueError('trace clock must be CLOCK_MONOTONIC')
            hash(key)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'invalid trace record: {exc}') from exc
        if event not in order:
            unknown_events += 1
            continue
        group = groups.setdefault(key, {})
        if event in group:
            duplicate_groups.add(key)
        group[event] = record
    samples: dict[str, list[float]] = {name: [] for name in pairs}
    missing = {name: 0 for name in pairs}
    unusable_time = {name: 0 for name in pairs}
    invalid_groups = 0
    for key, group in groups.items():
        stamps = [ns(group[e]['t_ns']) for e in order if e in group]
        if key in duplicate_groups or any(a > b for a, b in zip(stamps, stamps[1:])):
            invalid_groups += 1
            continue
        for name, (a, b) in pairs.items():
            if a not in group or b not in group:
                missing[name] += 1
                continue
            if a == 'observation_reference' and group[a].get('time_quality') != 'verified':
                unusable_time[name] += 1
                continue
            samples[name].append((ns(group[b]['t_ns']) - ns(group[a]['t_ns']))/1e6)
    return {'frame_groups': len(groups), 'invalid_frame_groups': invalid_groups,
            'unknown_events': unknown_events,
            'metrics': {name: {**quantiles(items), 'missing_pairs': missing[name],
                               'unusable_time_pairs': unusable_time[name]}
                        for name, items in samples.items()},
            'qualification': 'NOT_EVALUATED',
            'note': 'This report is not hardware qualification and does not impute missing events.'}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    summary = commands.add_parser('summarize')
    summary.add_argument('trace', type=Path)
    observation = commands.add_parser('check-observation')
    observation.add_argument('observation', type=Path)
    observation.add_argument('context', type=Path)
    power = commands.add_parser('decode-power')
    power.add_argument('value')
    args = parser.parse_args(argv)
    try:
        if args.command == 'summarize':
            result = summarize_trace(read_ndjson(args.trace))
        elif args.command == 'decode-power':
            result = decode_throttled(args.value)
        else:
            obs = strict_json(args.observation.read_text(encoding='utf-8'))
            ctx = strict_json(args.context.read_text(encoding='utf-8'))
            reasons = observation_rejections(obs, ctx)
            result = {'semantic_rejections': reasons,
                      'qualification': 'NOT_EVALUATED',
                      'warning': 'A valid fixture is not an authorization to move hardware.'}
            print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
            return 1 if reasons else 0
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, TypeError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
