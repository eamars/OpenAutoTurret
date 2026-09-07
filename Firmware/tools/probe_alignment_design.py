"""Offline reference model for the proposed alignment design, not station firmware.

Standard library only. No camera, network, motor, rangefinder or IMU access.
The camera origin is assumed to coincide with the rotation pivot. See the design
document for the limits of this direction-only model.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def unit(v):
    length = math.sqrt(dot(v, v))
    if not math.isfinite(length) or length <= 0:
        raise ValueError('ray must be finite and nonzero')
    return tuple(x / length for x in v)


def mv(matrix, vector):
    return tuple(dot(row, vector) for row in matrix)


def transpose(matrix):
    return tuple(zip(*matrix))


def mm(a, b):
    return tuple(tuple(dot(row, col) for col in transpose(b)) for row in a)


def rotation(yaw, pitch, camera_to_pitch):
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rz = ((cy, -sy, 0), (sy, cy, 0), (0, 0, 1))
    ry = ((cp, 0, sp), (0, 1, 0), (-sp, 0, cp))
    return mm(mm(rz, ry), camera_to_pitch)


def load_rotation(path):
    rows = []
    for line in path.read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if line and '=' not in line:
            rows.append(tuple(float(x) for x in line.split()))
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError('camera extrinsics must contain a 3x3 rotation')
    if any(not math.isfinite(v) for row in rows for v in row):
        raise ValueError('camera rotation must be finite')
    gram = mm(rows, transpose(rows))
    error = sum(abs(gram[i][j] - (i == j)) for i in range(3) for j in range(3))
    determinant = (rows[0][0] * (rows[1][1]*rows[2][2] - rows[1][2]*rows[2][1])
                   - rows[0][1] * (rows[1][0]*rows[2][2] - rows[1][2]*rows[2][0])
                   + rows[0][2] * (rows[1][0]*rows[2][1] - rows[1][1]*rows[2][0]))
    if error > 1e-6 or abs(determinant - 1) > 1e-6:
        raise ValueError('camera rotation must be a proper orthonormal rotation')
    return tuple(rows)


def beam_point(origin, direction, depth):
    if not all(math.isfinite(v) for v in (*origin, *direction, depth)):
        raise ValueError('geometry must be finite')
    if direction[2] <= 0:
        raise ValueError('laser must point forward')
    along = (depth - origin[2]) / direction[2]
    if depth <= 0 or along <= 0:
        raise ValueError('reference plane must be in front of camera and laser')
    return tuple(o + along * d for o, d in zip(origin, direction))


def load_intrinsics(path):
    values = {}
    for line in path.read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if '=' in line:
            key, value = line.split('=', 1)
            if key in ('fx', 'fy', 'cx', 'cy', 'width', 'height'):
                values[key] = float(value)
    if set(values) != {'fx', 'fy', 'cx', 'cy', 'width', 'height'}:
        raise ValueError('missing camera intrinsics')
    if (not all(math.isfinite(v) for v in values.values()) or
            any(values[k] <= 0 for k in ('fx', 'fy', 'width', 'height'))):
        raise ValueError('invalid camera intrinsics')
    return values


def project(point, intrinsics):
    if not all(math.isfinite(v) for v in point) or point[2] <= 0:
        raise ValueError('cannot project behind camera or nonfinite point')
    return (intrinsics['cx'] + intrinsics['fx'] * point[0] / point[2],
            intrinsics['cy'] + intrinsics['fy'] * point[1] / point[2])


def pixel_ray(pixel, intrinsics):
    return unit(((pixel[0] - intrinsics['cx']) / intrinsics['fx'],
                 (pixel[1] - intrinsics['cy']) / intrinsics['fy'], 1))


def validate_config(config):
    """Strict proposal schema. The station loader does not accept this schema yet."""
    def keys(value, expected, name):
        if not isinstance(value, dict) or set(value) != set(expected.split()):
            raise ValueError(f'{name}: expected exactly {expected}')

    def number(value, name):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')

    keys(config, 'proposal_version tracking alignment', 'root')
    if type(config['proposal_version']) is not int or config['proposal_version'] != 1:
        raise ValueError('unsupported proposal_version')
    keys(config['tracking'], 'aim_point', 'tracking')
    aim = config['tracking']['aim_point']
    keys(aim, 'mode x_fraction y_fraction', 'tracking.aim_point')
    if aim['mode'] not in ('perception_anchor', 'box_fraction'):
        raise ValueError('unknown aim point mode')
    for key in ('x_fraction', 'y_fraction'):
        number(aim[key], key)
        if not 0 <= aim[key] <= 1:
            raise ValueError(f'{key} must be in [0,1]')
    alignment = config['alignment']
    keys(alignment, 'mode camera_from_laser_mm laser_axis_deg assumed_depth_m', 'alignment')
    if alignment['mode'] not in ('off', 'manual_depth'):
        raise ValueError('alignment mode must be off or manual_depth; no sensor is available')
    mount = alignment['camera_from_laser_mm']
    keys(mount, 'right up forward', 'camera_from_laser_mm')
    for key, value in mount.items():
        number(value, key)
    angles = alignment['laser_axis_deg']
    keys(angles, 'right up', 'laser_axis_deg')
    for key, value in angles.items():
        number(value, key)
        if abs(value) >= 45:
            raise ValueError('reference model supports alignment angles strictly inside +/-45 deg')
    number(alignment['assumed_depth_m'], 'assumed_depth_m')
    origin, direction = laser_geometry(config)
    beam_point(origin, direction, alignment['assumed_depth_m'])
    return config


def laser_geometry(config):
    alignment = config['alignment']
    mount = alignment['camera_from_laser_mm']
    # C is the corrected detector frame: x right, y down, z forward.
    origin = (-mount['right']/1000, mount['up']/1000, -mount['forward']/1000)
    angles = alignment['laser_axis_deg']
    direction = unit((math.tan(math.radians(angles['right'])),
                      -math.tan(math.radians(angles['up'])), 1))
    return origin, direction


def aim_point(bbox, anchor, aim):
    if len(anchor) != 2 or not all(math.isfinite(v) and 0 <= v <= 1 for v in anchor):
        raise ValueError('no valid perception anchor')
    if aim['mode'] == 'perception_anchor':
        return tuple(anchor), 'perception_anchor'
    if (len(bbox) != 4 or not all(math.isfinite(v) for v in bbox) or
            not (0 <= bbox[0] < bbox[2] <= 1 and 0 <= bbox[1] < bbox[3] <= 1)):
        return tuple(anchor), 'invalid_box_anchor_fallback'
    return ((bbox[0] + aim['x_fraction'] * (bbox[2]-bbox[0]),
             bbox[1] + aim['y_fraction'] * (bbox[3]-bbox[1])), 'box_fraction')


def configured_sight(config):
    if config['alignment']['mode'] == 'off':
        return (0, 0, 1)
    origin, direction = laser_geometry(config)
    return unit(beam_point(origin, direction, config['alignment']['assumed_depth_m']))


def solve_direction(target_world, sight_camera, camera_to_pitch, seed):
    """Reference adaptation of LosJointSolver's two pitch branches.

    The design generalizes the camera optical axis to an explicit sight ray.
    Limits here are synthetic; no retained hardware calibration is loaded.
    """
    target = unit(target_world)
    body = mv(camera_to_pitch, unit(sight_camera))
    radius = math.hypot(body[0], body[2])
    if radius < 1e-12 or abs(target[2]) > radius + 1e-12:
        raise ValueError('unreachable elevation')
    root = math.acos(max(-1, min(1, target[2] / radius)))
    offset = math.atan2(body[0], body[2])
    choices = []
    for sign in (-1, 1):
        for turn in range(-2, 3):
            pitch = sign * root - offset + turn * math.tau
            if not math.radians(-74) <= pitch <= math.radians(-5):
                continue
            x = body[0] * math.cos(pitch) + body[2] * math.sin(pitch)
            raw_yaw = math.atan2(target[1], target[0]) - math.atan2(body[1], x)
            for yaw_turn in range(-2, 3):
                yaw = raw_yaw + yaw_turn * math.tau
                if -0.39 <= yaw <= 5.58:
                    cost = (yaw - seed[0]) ** 2 + (pitch - seed[1]) ** 2
                    choices.append((cost, yaw, pitch))
    if not choices:
        raise ValueError('outside synthetic joint limits')
    _, yaw, pitch = min(choices)
    return yaw, pitch


def miss_mm(target, world_rotation, origin, direction):
    laser_origin = mv(world_rotation, origin)
    laser_direction = mv(world_rotation, direction)
    delta = tuple(t - o for t, o in zip(target, laser_origin))
    along = dot(delta, laser_direction)
    if along <= 0:
        raise ValueError('target behind laser')
    residual = tuple(x - along * d for x, d in zip(delta, laser_direction))
    return math.sqrt(dot(residual, residual)) * 1000


def probe_geometry(extrinsics, intrinsics, origin, direction):
    """Matched-depth scenes, independently checked as world-space beam/point misses."""
    rows = []
    for depth in (2, 5, 10, 30, 100):
        point = beam_point(origin, direction, depth)
        for yaw_deg in (20, 160, 300):
            for pitch_deg in (-10, -35, -65):
                desired = (math.radians(yaw_deg), math.radians(pitch_deg))
                target = mv(rotation(*desired, extrinsics), point)
                seed = (desired[0] + 0.03, desired[1] - 0.03)
                solved = solve_direction(target, point, extrinsics, seed)
                uncompensated = solve_direction(target, (0, 0, 1), extrinsics, seed)
                solved_rotation = rotation(*solved, extrinsics)
                observed_pixel = project(mv(transpose(solved_rotation), target), intrinsics)
                laser_pixel = project(point, intrinsics)
                rows.append({
                    'depth_m': depth, 'yaw_deg': yaw_deg, 'pitch_deg': pitch_deg,
                    'compensated_miss_mm': miss_mm(target, solved_rotation, origin, direction),
                    'uncompensated_miss_mm': miss_mm(target, rotation(*uncompensated, extrinsics), origin, direction),
                    'crosshair_error_px': math.dist(observed_pixel, laser_pixel),
                })
    if max(row['compensated_miss_mm'] for row in rows) > 1e-5:
        raise ValueError('FAIL: generalized sight ray does not align the beam')
    if max(row['crosshair_error_px'] for row in rows) > 1e-7:
        raise ValueError('FAIL: HUD projection disagrees with solved beam')
    return rows


def distance_sweep(config, intrinsics):
    origin, direction = laser_geometry(config)
    sight = configured_sight(config)
    identity = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    rows = []
    for depth in (2, 5, 10, 30, 100):
        target = tuple(v * depth/sight[2] for v in sight)
        laser_pixel = project(beam_point(origin, direction, depth), intrinsics)
        rows.append({'true_depth_m': depth, 'physical_laser_pixel': laser_pixel,
                     'miss_at_configured_sight_mm': miss_mm(target, identity, origin, direction)})
    return rows


def framing_sweep(config, intrinsics, extrinsics):
    """Synthetic box corners under pure rotation; no detector performance claim."""
    bbox, anchor = (0.4, 0.08, 0.6, 0.92), (0.5, 0.458)
    width, height = intrinsics['width'], intrinsics['height']
    initial = (math.radians(160), math.radians(-35))
    initial_rotation = rotation(*initial, extrinsics)
    corners = [mv(initial_rotation, pixel_ray((x*width, y*height), intrinsics))
               for x in (bbox[0], bbox[2]) for y in (bbox[1], bbox[3])]
    aim = config['tracking']['aim_point']
    rows = []
    for fraction in sorted({0.22, 0.35, 0.45, 0.5, aim['y_fraction']}):
        trial = dict(aim, mode='box_fraction', y_fraction=fraction)
        point, source = aim_point(bbox, anchor, trial)
        target = mv(initial_rotation, pixel_ray((point[0]*width, point[1]*height), intrinsics))
        solved = solve_direction(target, configured_sight(config), extrinsics, initial)
        to_camera = transpose(rotation(*solved, extrinsics))
        pixels = [project(mv(to_camera, corner), intrinsics) for corner in corners]
        bounds = (min(p[0] for p in pixels)/width, min(p[1] for p in pixels)/height,
                  max(p[0] for p in pixels)/width, max(p[1] for p in pixels)/height)
        rows.append({'y_fraction': fraction, 'source': source, 'projected_box_norm': bounds,
                     'fully_in_frame': all(0 <= p[0] <= width and 0 <= p[1] <= height for p in pixels)})
    return rows


def run(config, firmware):
    validate_config(config)
    extrinsics = load_rotation(firmware / 'calibration/camera_extrinsics.yaml')
    intrinsics = load_intrinsics(firmware / 'calibration/camera_intrinsics.yaml')
    origin, direction = laser_geometry(config)
    fixtures = [
        ('configured_mount', origin, direction),
        ('zero_offset', (0, 0, 0), (0, 0, 1)),
        ('reversed_mount', (0.075, -0.075, 0), (0, 0, 1)),
        ('forward_and_angle', (-0.075, 0.075, -0.025), unit((0.01, -0.005, 1))),
    ]
    scenarios = {name: probe_geometry(extrinsics, intrinsics, o, d) for name, o, d in fixtures}
    bbox = (0.4, 0.08, 0.6, 0.92)
    chosen, source = aim_point(bbox, (0.5, 0.458), config['tracking']['aim_point'])
    enabled = config['alignment']['mode'] == 'manual_depth'
    sight = configured_sight(config)
    mark = project(sight, intrinsics)
    if enabled and not (0 <= mark[0] <= intrinsics['width'] and 0 <= mark[1] <= intrinsics['height']):
        raise ValueError('configured laser sight falls outside the camera frame')
    all_rows = [row for rows in scenarios.values() for row in rows]
    return {
        'status': 'reference_geometry_passed; firmware_and_hardware_unverified',
        'config': config, 'intrinsics': intrinsics, 'camera_to_pitch_rotation': extrinsics,
        'assumptions': ['camera origin at rotation pivot', 'ideal pinhole without distortion',
                        'stationary targets and perfect joint positioning', 'no detector or sensor emulation'],
        'configured_example': {
            'aim_point_norm': chosen, 'aim_source': source,
            'sight_ray_camera': sight, 'laser_reticle_px': mark if enabled else None,
            'alignment_status': 'assumed_depth' if enabled else 'disabled',
            'range_source': 'manual' if enabled else 'none', 'range_measured': False,
        },
        'summary': {'scenes': len(all_rows),
                    'max_compensated_miss_mm': max(r['compensated_miss_mm'] for r in all_rows),
                    'max_crosshair_error_px': max(r['crosshair_error_px'] for r in all_rows)},
        'matched_depth_geometry_scenarios': scenarios,
        'distance_sweep_with_fixed_configuration': distance_sweep(config, intrinsics),
        'synthetic_box_framing': framing_sweep(config, intrinsics, extrinsics),
    }


def main():
    firmware = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=firmware / 'tools/fixtures/alignment_design.json')
    parser.add_argument('--depth-m', type=float, help='override assumed camera-axis depth for this run')
    parser.add_argument('--y-fraction', type=float, help='override box-height fraction for this run')
    parser.add_argument('--output', type=Path, help='optional JSON report, preferably under ignored run/')
    args = parser.parse_args()
    try:
        config = validate_config(json.loads(args.config.read_text(encoding='utf-8')))
        if args.depth_m is not None:
            config['alignment']['assumed_depth_m'] = args.depth_m
        if args.y_fraction is not None:
            config['tracking']['aim_point']['y_fraction'] = args.y_fraction
        report = run(config, firmware)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.exit(2, f'alignment probe: {exc}\n')
    print(json.dumps(report['summary'], indent=2, allow_nan=False))
    print('Fixed configuration: true depth m -> beam miss mm')
    for row in report['distance_sweep_with_fixed_configuration']:
        print(f"  {row['true_depth_m']:3} -> {row['miss_at_configured_sight_mm']:.3f}")
    print('Synthetic framing: box-height fraction -> entire box in frame')
    for row in report['synthetic_box_framing']:
        print(f"  {row['y_fraction']:.2f} -> {row['fully_in_frame']}")
    print('Reference model only; no firmware, detector, motor or physical alignment verification.')


if __name__ == '__main__':
    main()
