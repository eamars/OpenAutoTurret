"""Regression cases from the native selection socket and browser probes."""
import concurrent.futures
import tempfile
import time
import uuid
from pathlib import Path

from perception.protocol.native_wire import HEADER, encode_perception_frame
from perception.selection.protocol import SelectTargetRequest
from perception.selection.service import SelectionService
from perception.selection.target_selection_manager import TargetSelectionManager
from perception.tests.support import commissioned_config, track_at, track_set_of
from web.webd.selection_client import request_selection


def fixture():
    track = track_at(.5, index=1)
    # Production identity factories use .hex; hyphenated fixtures hid an IPC bug.
    track.track_uuid = uuid.UUID(int=17).hex
    frame = track_set_of([track], sequence=12, session_uuid=uuid.UUID(int=2**128-1).hex)
    frame.sensor_timestamp_ns = frame.publish_timestamp_ns = time.monotonic_ns()
    return TargetSelectionManager(commissioned_config()), frame, track


def test_native_packet_preserves_session_and_generation():
    selector, frame, track = fixture()
    selector.select(SelectTargetRequest(track_uuid=track.track_uuid), frame, frame.sensor_timestamp_ns)
    observation = selector.update(frame, frame.sensor_timestamp_ns)
    packet = encode_perception_frame(frame, observation)
    header = HEADER.unpack_from(packet)
    assert len(packet) == 2662
    assert header[:3] == (b'OTP1', 1, 100)
    assert header[3:9] == (2**64-1, 2**64-1, 0, 17, 1, 12)
    assert header[9:11] == (1, 1)


def test_socket_ack_retry_clear_and_expired_request():
    selector, frame, track = fixture()
    with tempfile.TemporaryDirectory() as directory:
        service = SelectionService(str(Path(directory)/'selection.sock'))
        service.start()
        try:
            with concurrent.futures.ThreadPoolExecutor(1) as pool:
                def call(data, process=True):
                    future = pool.submit(request_selection, service.path, data)
                    while not future.done():
                        frame.sensor_timestamp_ns = time.monotonic_ns()
                        if process:
                            service.process(selector, frame, time.monotonic_ns())
                        time.sleep(.002)
                    return future.result()
                request = dict(type='select_target', request_id='a', track_uuid='0:17',
                    session_uuid=f'{2**64-1}:{2**64-1}', track_set_sequence_seen_by_ui=12)
                first = call(request)
                assert first['accepted'] and first['selection_generation'] == 1
                assert call(request) == first
                repeated = call(dict(request, request_id='b'))
                assert repeated['reason'] == 'SELECTION_UNCHANGED'
                assert repeated['selection_generation'] == 1
                assert call(dict(request, request_id='c', session_uuid='0:1'))['reason'] == 'STALE_SESSION'
                cleared = call(dict(request, request_id='d', type='clear_target'))
                assert cleared['reason'] == 'CLEARED' and cleared['selection_generation'] == 2
                assert call(dict(request, request_id='expired'), process=False)['accepted'] is False
                service.process(selector, frame, time.monotonic_ns())
                assert not selector.state.has_selection
        finally:
            service.close()
