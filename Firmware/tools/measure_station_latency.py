#!/usr/bin/env python3
"""Target-free station measurement via the existing service; no device ownership.

Run on the Pi with its project venv. Default is read-only. --jogs N runs N
cycles of yaw+/yaw-/pitch+/pitch- FINE jogs. Retains Manual/Hold and all limits.
Raw captures belong in ignored run/, never in Git.
"""
import argparse
import json
import math
from pathlib import Path
import threading
import time
import urllib.request


BASE = 'http://127.0.0.1:8080'


def request(path, data=None):
    req = urllib.request.Request(BASE + path,
        data=None if data is None else json.dumps(data).encode(),
        headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=.5) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--jogs', type=int, default=0)
    parser.add_argument('--single', action='store_true', help='first yaw+ probe only')
    args = parser.parse_args()
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    stop = threading.Event()
    errors = []
    latest = {}
    lock = threading.Lock()
    fh = out.open('w')

    def emit(kind, **payload):
        with lock:
            fh.write(json.dumps(dict(kind=kind, observed_ns=time.monotonic_ns(), **payload))+'\n')
            fh.flush()

    def sample():
        last_state = last_frame = last_preview = None
        state_due = 0
        while not stop.is_set():
            start = time.monotonic()
            try:
                if start >= state_due:
                    state = request('/api/state')
                    latest.update(state)
                    if state['ts_ns'] != last_state:
                        emit('state', state=state)
                        last_state = state['ts_ns']
                    state_due = start + .05
                track = json.loads(Path('/tmp/ota-stack-1000/perception/track_set.json').read_text())
                seq = track.get('frame_sequence')
                if seq != last_frame:
                    emit('perception', data={k:v for k,v in track.items() if k not in ('tracks','events')})
                    last_frame = seq
                jpeg = Path('/tmp/ota-stack-1000/preview.jpg').read_bytes()
                marker = jpeg.find(b'OTA_FRAME\x00')
                if marker >= 4:
                    length = int.from_bytes(jpeg[marker-2:marker], 'big')
                    meta = json.loads(jpeg[marker+10:marker+length-2])
                    if meta['frame_sequence'] != last_preview:
                        emit('camera', data={k:v for k,v in meta.items() if k not in ('track_set','detection_set','anchor_mapping')})
                        last_preview = meta['frame_sequence']
            except Exception as exc:
                errors.append(str(exc))
                emit('sample_error', error=str(exc))
            stop.wait(max(0, .02-(time.monotonic()-start)))

    def command(name, arg=''):
        before = time.monotonic_ns()
        response = request('/api/command', dict(command=name, arg=arg))
        emit('command', command=name, arg=arg, sent_ns=before, response=response)
        if response.get('accepted') is False or response.get('ok') is False:
            raise RuntimeError(f'command rejected: {response}')
        return before

    def healthy():
        s = latest.copy()
        if (not s.get('controld_connected') or s.get('fault') or
                s.get('operating_mode') != 'MANUAL' or s.get('phase') != 'hold' or
                s.get('safety_action') != 'ALLOW' or not s.get('soft_limits_valid') or
                s.get('feedback_age_ms', 999) > 50 or
                time.monotonic_ns()-s.get('ts_ns', 0) > 250_000_000):
            emit('health_gate', state=s, current_age_ms=(time.monotonic_ns()-s.get('ts_ns',0))/1e6)
            raise RuntimeError('station health/mode gate failed')
        return s

    def dwell(duration, origin=None):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            s = healthy()
            if errors:
                raise RuntimeError('sampling failed: '+errors[-1])
            if origin and any(abs(s[f'q_{axis}_rad']-origin[f'q_{axis}_rad']) > math.radians(10)
                              for axis in ('yaw','pitch')):
                raise RuntimeError('10 degree excursion guard')
            time.sleep(.04)

    initial = request('/api/state')
    emit('initial', state=initial, args=vars(args))
    worker = threading.Thread(target=sample, daemon=True)
    worker.start()
    moving = False
    try:
        time.sleep(.3)
        if args.jogs or args.single:
            healthy()
            directions = ['yaw+'] if args.single else ['yaw+','yaw-','pitch+','pitch-']*args.jogs
            dwell(3)
            for index, direction in enumerate(directions):
                origin = healthy()
                for axis in ('yaw','pitch'):
                    q = origin[f'q_{axis}_rad']
                    if min(q-origin[f'q_soft_min_{axis}_rad'], origin[f'q_soft_max_{axis}_rad']-q) < math.radians(15):
                        raise RuntimeError('less than 15 degree soft-limit clearance')
                emit('trial_start', index=index, direction=direction, origin=origin)
                print(f'trial {index}: {direction}:FINE', flush=True)
                moving = True
                command('manual_jog_start', direction+':FINE')
                end = time.monotonic()+2.0
                while time.monotonic() < end:
                    dwell(.08, origin)
                    command('manual_jog_keepalive')
                command('manual_jog_stop')
                moving = False
                dwell(4, origin)
                emit('trial_end', index=index, state=healthy())
                dwell(1)
        else:
            stop.wait(args.seconds)
        emit('complete', errors=errors, state=request('/api/state'))
    except Exception as exc:
        emit('aborted', error=str(exc))
        raise
    finally:
        if moving:
            try:
                command('manual_jog_stop')
            except Exception as exc:
                emit('stop_error', error=str(exc), fallback='300 ms jog lease expiry')
        stop.set()
        worker.join(2)
        fh.close()
    print(str(out), flush=True)


if __name__ == '__main__':
    main()
