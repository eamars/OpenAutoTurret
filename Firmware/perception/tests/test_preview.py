"""Preview output is usable, atomic, and independent of a slow encoder."""
import threading
import time
import json

import numpy as np
from PIL import Image

from perception.pipeline import PreviewTap
from perception.preview import JpegPreviewWorker


def wait_until(predicate):
    deadline = time.monotonic() + 3
    while not predicate() and time.monotonic() < deadline:
        time.sleep(.005)
    assert predicate()


def test_preview_writes_a_complete_jpeg_with_picamera_channel_order(tmp_path):
    tap = PreviewTap(fps=0)
    path = tmp_path / 'preview.jpg'
    worker = JpegPreviewWorker(tap, str(path))
    worker.start()
    try:
        frame = np.zeros((32, 32, 3), dtype=np.uint8)
        frame[..., 2] = 255  # RGB888's byte order is BGR on Picamera2.
        tap.offer(frame, metadata={'sensor_timestamp_ns': 123456789, 'frame_sequence': 7})
        wait_until(lambda: worker.published == 1)
        with Image.open(path) as result:
            result.load()
            red, green, blue = result.getpixel((16, 16))
            assert red > 240 and green < 10 and blue < 10
            comment = result.info['comment']
            assert comment.startswith(b'OTA_FRAME\x00')
            assert json.loads(comment[10:]) == {'sensor_timestamp_ns': 123456789, 'frame_sequence': 7}
        assert not list(tmp_path.glob('*.part'))
    finally:
        worker.stop()
    assert not worker.stats()['running']


def test_slow_encoding_does_not_hold_the_latest_frame_slot(tmp_path):
    tap = PreviewTap(fps=0)
    worker = JpegPreviewWorker(tap, str(tmp_path / 'preview.jpg'))
    worker.start()
    entered, release = threading.Event(), threading.Event()
    encode = worker._encode
    def delayed(frame):
        entered.set()
        release.wait(3)
        return encode(frame)
    worker._encode = delayed
    try:
        tap.offer(np.zeros((8, 8, 3), dtype=np.uint8))
        assert entered.wait(3)
        for value in range(100):
            tap.offer(np.full((8, 8, 3), value, dtype=np.uint8))
        assert tap.overwritten == 99
        assert np.all(tap.take() == 99)
    finally:
        release.set()
        worker.stop()


def test_crowded_diagnostics_cannot_overflow_the_jpeg_comment(tmp_path):
    tap = PreviewTap(fps=0)
    path = tmp_path / 'crowded.jpg'
    worker = JpegPreviewWorker(tap, str(path))
    worker.start()
    try:
        tap.offer(np.zeros((8, 8, 3), dtype=np.uint8), metadata={
            'sensor_timestamp_ns': 123, 'frame_sequence': 9,
            'anchor_mapping': ['x' * 1000] * 100})
        wait_until(lambda: worker.published == 1)
        with Image.open(path) as result:
            result.load()
            info = json.loads(result.info['comment'][10:])
            assert info['sensor_timestamp_ns'] == 123
            assert info['frame_sequence'] == 9
            assert info['detections_omitted'] == 'JPEG comment size limit'
            assert 'anchor_mapping' not in info
        assert worker.failures == 0
    finally:
        worker.stop()


def test_output_failure_is_reported_and_worker_can_recover(tmp_path):
    tap = PreviewTap(fps=0)
    path = tmp_path / 'occupied'
    path.mkdir()
    worker = JpegPreviewWorker(tap, str(path))
    worker.start()
    try:
        tap.offer(np.zeros((8, 8, 3), dtype=np.uint8))
        wait_until(lambda: worker.failures == 1)
        assert worker.last_error
        path.rmdir()
        tap.offer(np.zeros((8, 8, 3), dtype=np.uint8))
        wait_until(lambda: worker.published == 1)
        assert worker.last_error == ''
    finally:
        worker.stop()
