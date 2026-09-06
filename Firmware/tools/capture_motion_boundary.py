#!/usr/bin/env python3
"""Record timestamped preview packets and controller state on the station.

Defaults to a stationary, read-only capture. --sweep performs explicitly bounded
Manual moves; detections never command motion. Uses the existing camera owner.
"""
import argparse
import json
import math
from pathlib import Path
import time
import urllib.request


def state():
    return json.load(urllib.request.urlopen('http://127.0.0.1:8080/api/state', timeout=1))


def packet(data):
    if data[:4] != b'\xff\xd8\xff\xfe':
        raise ValueError('preview has no timestamped capture comment')
    length = int.from_bytes(data[4:6], 'big')
    payload = data[6:4 + length]
    if not payload.startswith(b'OTA_FRAME\x00'):
        raise ValueError('unknown preview metadata')
    return json.loads(payload[10:])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seconds', type=float, default=10)
    motion = parser.add_mutually_exclusive_group()
    motion.add_argument('--sweep', choices=['yaw', 'pitch', 'both'])
    motion.add_argument('--automatic', action='store_true',
                        help='record the normal bounded auto roam/track service cycle')
    motion.add_argument('--observe', action='store_true',
                        help='record the currently running service without changing modes')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    images = args.output / 'images'
    images.mkdir()
    initial = state()
    assert initial['phase'] == 'hold'
    if not args.observe:
        assert initial['operating_mode'] == 'MANUAL'
    assert initial['soft_limits_valid'] and not initial.get('fault')
    seen = set()
    samples, frames, commands = [], [], []
    stage = 'initial_hold'

    def record():
        s = state()
        s['probe_stage'] = stage
        s['probe_receive_ns'] = time.monotonic_ns()
        samples.append(s)
        assert not s.get('fault'), s.get('fault')
        assert s['phase'] == 'hold'
        if args.observe:
            assert s['operating_mode'] in ('MANUAL', 'AUTO_ROAM', 'AUTO_TRACK')
        elif args.automatic and stage == 'automatic':
            assert s['operating_mode'] in ('AUTO_ROAM', 'AUTO_TRACK'), 'automatic trial interrupted by Manual'
        else:
            assert s['operating_mode'] == 'MANUAL'
        assert s.get('feedback_age_ms', 999) < 150 and not s.get('telemetry_stale')
        for axis in ('yaw', 'pitch'):
            q = s[f'q_{axis}_rad']
            if not args.automatic and not args.observe:
                assert abs(q - initial[f'q_{axis}_rad']) < math.radians(13), 'excursion guard'
            assert s[f'q_soft_min_{axis}_rad'] + .025 < q < s[f'q_soft_max_{axis}_rad'] - .025
        data = Path('/tmp/ota-stack-1000/preview.jpg').read_bytes()
        info = packet(data)
        sensor_ns = info['sensor_timestamp_ns']
        assert 0 <= s['probe_receive_ns'] - sensor_ns < 400_000_000, 'stale camera capture'
        if sensor_ns not in seen:
            seen.add(sensor_ns)
            info['probe_read_ns'] = s['probe_receive_ns']
            info['probe_stage'] = stage
            info['path'] = f'images/{sensor_ns}.jpg'
            (args.output / info['path']).write_bytes(data)
            frames.append(info)
        return s

    def wait(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            tick = time.monotonic()
            record()
            time.sleep(max(0, .025 - (time.monotonic() - tick)))

    def command(name, arg=''):
        before = state().get('cmd_ack_seq')
        request = urllib.request.Request('http://127.0.0.1:8080/api/command',
            data=json.dumps({'command': name, 'arg': arg}).encode(),
            headers={'Content-Type': 'application/json'})
        json.load(urllib.request.urlopen(request, timeout=1))
        commands.append({'command': name, 'arg': arg, 'sent_ns': time.monotonic_ns()})
        for _ in range(40):
            s = state()
            if s.get('cmd_ack_seq') != before and s.get('cmd_ack_command') == name:
                assert s.get('cmd_ack_accepted'), s.get('cmd_ack_reason')
                return
            time.sleep(.025)
        raise RuntimeError('no command acknowledgement')

    try:
        wait(3 if args.sweep or args.automatic else args.seconds)
        if args.automatic:
            command('set_mode', 'AUTO_ROAM')
            stage = 'automatic'
            wait(min(args.seconds, 120))
        if args.sweep:
            axes = ['yaw', 'pitch'] if args.sweep == 'both' else [args.sweep]
            for axis in axes:
                # Two 5 degree steps give a 10 degree traversed field, with settled
                # endpoints and both directions for separating scale from time delay.
                for delta in ('+5', '+5', '-5', '-5'):
                    stage = axis + delta + '_' + str(len(commands))
                    print(stage, flush=True)
                    command('manual_step', axis + delta)
                    wait(6)
            stage = 'final_hold'
            wait(3)
    finally:
        try:
            if args.sweep or args.automatic:
                command('stop_motion')
        finally:
            # Preserve the failure evidence even if the stop acknowledgement
            # itself is unavailable (for example, a disconnected controller).
            (args.output / 'capture.json').write_text(json.dumps({
                'initial': initial, 'commands': commands, 'frames': frames, 'samples': samples}))
            print(json.dumps({'frames': len(frames), 'samples': len(samples),
                'output': str(args.output)}), flush=True)


if __name__ == '__main__':
    main()
