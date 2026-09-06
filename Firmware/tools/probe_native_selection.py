"""Run native selection through real web/IPC/controller processes and simulated motors.

Synthetic candidates are test fixtures. --camera uses the real perception daemon.
Motor output is always simulated; this tool never opens CAN.
Use a project venv; browse the supplied isolated port while the probe is running.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import time
import uuid

from perception.tests.support import commissioned_config, track_at, track_set_of
from perception.protocol.native_wire import encode_perception_frame
from perception.protocol.wire import SocketPublisher
from perception.selection.service import SelectionService
from perception.selection.target_selection_manager import TargetSelectionManager
from perception.selection.control_context import ControllerContext
from perception.config import SelectionPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s', type=float, default=180)
    parser.add_argument('--port', type=int, default=8081)
    parser.add_argument('--camera', action='store_true', help='use real IMX500 perception')
    parser.add_argument('--auto-cycle', action='store_true', help='exercise autonomous selection with fixtures')
    parser.add_argument('--profile', default='person_detect_available')
    parser.add_argument('--control-file', type=Path,
                        help='optional probe-only JSON: visible_indices and pause_publish')
    args = parser.parse_args()
    evidence = Path('../run/hardening') / ('native-process-' + str(time.time_ns()))
    evidence.mkdir(parents=True)
    stop = False
    def stopped(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGTERM, stopped)
    signal.signal(signal.SIGINT, stopped)
    children, logs = [], []
    with tempfile.TemporaryDirectory(prefix='ota-native-') as directory:
        root = Path(directory)
        env = dict(os.environ, OTA_VISION_SOCKET=str(root/'vision.sock'),
                   OTA_WEB_SOCKET=str(root/'web.sock'), OTA_SELECTION_SOCKET=str(root/'selection.sock'),
                   OTA_WEB_PORT=str(args.port), OTA_WEB_HOST='0.0.0.0', OTA_VIDEO_ENABLE='1' if args.camera else '0',
                   OTA_VISION_FRAME_TAP=str(root/'preview.jpg'))
        config = commissioned_config()
        if args.auto_cycle:
            config.selection.policy = SelectionPolicy.AUTO_SELECT_SINGLE
            config.selection.auto_select_min_detector_score = .7
            config.selection.auto_select_min_identity_confidence = .7
        selector = TargetSelectionManager(config)
        context = ControllerContext(f'http://127.0.0.1:{args.port}/api/state')
        service = SelectionService(env['OTA_SELECTION_SOCKET'])
        publisher = SocketPublisher(env['OTA_VISION_SOCKET'])
        try:
            if not args.camera:
                service.start()
                context.start()
            for name, command in [
                ('controller', ['build/control/controld', 'config/turret.yaml', '--sim']),
                ('web', [sys.executable, '-m', 'web.webd.app'])]:
                log = open(root/(name+'.log'), 'w'); logs.append(log)
                children.append(subprocess.Popen(command, env=env, stdout=log, stderr=log))
            if args.camera:
                log = open(root/'vision.log', 'w'); logs.append(log)
                children.append(subprocess.Popen([
                    sys.executable, '-m', 'perception.visiond',
                    '--config', 'perception/configs/perception_v1.json', '--profile', args.profile,
                    '--publish-socket', env['OTA_VISION_SOCKET'],
                    '--selection-socket', env['OTA_SELECTION_SOCKET'],
                    '--record-dataset', str(evidence/'camera-recording'),
                    '--report', str(evidence/'camera-report.json'),
                    '--publish-dir', str(evidence/'perception'),
                ], env=env, stdout=log, stderr=log))
            print(json.dumps({'port': args.port, 'directory': directory,
                              'evidence': str(evidence), 'camera': args.camera,
                              'simulated_motors': True}), flush=True)
            tracks = [track_at(.35, index=1), track_at(.65, index=2)]
            for index, track in enumerate(tracks):
                track.track_uuid = uuid.UUID(int=17+index).hex
            session = uuid.uuid4().hex
            started = time.monotonic(); sequence = 0
            while not stop and time.monotonic()-started < args.duration_s:
                if any(child.poll() is not None for child in children):
                    for log in root.glob('*.log'):
                        print(log.name, log.read_text()[-5000:], flush=True)
                    raise RuntimeError('probe child exited; inspect '+str(evidence))
                if args.camera:
                    time.sleep(.04)
                    continue
                now = time.monotonic_ns(); sequence += 1
                control = json.loads(args.control_file.read_text()) if args.control_file and args.control_file.exists() else {}
                visible = control.get('visible_indices', [1] if args.auto_cycle else [1, 2])
                for track in tracks:
                    track.last_measurement_ns = now-60_000_000
                frame = track_set_of([t for t in tracks if t.display_index in visible],
                    sequence=sequence, frame_index=sequence,
                    sensor_ns=now-60_000_000, session_uuid=session)
                service.process(selector, frame, now)
                mode = context.operating_mode(session, now) if args.auto_cycle else ''
                observation = selector.update(frame, frame.sensor_timestamp_ns,
                    auto_track_enabled=mode == 'AUTO_TRACK', auto_roam_enabled=mode == 'AUTO_ROAM')
                frame.publish_timestamp_ns = observation.publish_timestamp_ns = time.monotonic_ns()
                if control.get('pause_publish'):
                    publisher.close()
                else:
                    publisher.send(encode_perception_frame(frame, observation))
                time.sleep(.04)
        finally:
            if not args.camera:
                service.close()
                context.close()
            publisher.close()
            for child in children:
                child.send_signal(signal.SIGINT)
            for child in children:
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    child.kill(); child.wait()
            for log in logs:
                log.close()
            for path in root.glob('*.log'):
                shutil.copy2(path, evidence/path.name)


if __name__ == '__main__':
    main()
