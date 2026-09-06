"""One bounded network-input/ISP comparison from the existing camera owner."""
import sys
import time
from pathlib import Path
from .pipeline import PreviewTap
from .preview import JpegPreviewWorker


class InputTensorProbe:
    def __init__(self, path, camera, imx500):
        self.path = Path(path)
        self.camera, self.imx500 = camera, imx500
        self.taps = [PreviewTap(fps=0), PreviewTap(fps=0)]
        self.workers = [JpegPreviewWorker(self.taps[0], str(self.path), quality=95),
                       JpegPreviewWorker(self.taps[1], str(self.path.with_suffix('.isp.jpg')), quality=90)]
        self.done = False
        self.captured = False
        self.started_ns = 0

    def start(self):
        if 'CnnEnableInputTensor' not in self.camera.camera_controls:
            raise RuntimeError('camera does not expose CnnEnableInputTensor')
        if self.path.exists() or self.path.with_suffix('.isp.jpg').exists():
            raise RuntimeError('input tensor probe output already exists')
        for worker in self.workers:
            worker.start()
        self.camera.set_controls({'CnnEnableInputTensor': True})
        self.started_ns = time.monotonic_ns()
        print('visiond: bounded input tensor probe enabled', file=sys.stderr)

    def offer(self, frame, outcome):
        elapsed = time.monotonic_ns() - self.started_ns
        if self.captured:
            self.done = all(w.published or w.failures for w in self.workers)
        elif elapsed > 2_000_000_000 and frame.metadata.get('CnnInputTensor') is not None:
            pixels = self.imx500.input_tensor_image(frame.metadata['CnnInputTensor'])
            metadata = {'sensor_timestamp_ns': frame.sensor_timestamp_ns,
                'metadata_receive_ns': frame.metadata_receive_ns,
                'frame_sequence': frame.frame_sequence,
                'camera': {k: frame.metadata[k] for k in ('ExposureTime', 'AnalogueGain', 'DigitalGain', 'Lux', 'ScalerCrop') if k in frame.metadata},
                'detection_set': outcome.detection_set.to_dict() if outcome.detection_set else None}
            self.taps[0].offer(pixels, metadata=metadata)
            self.taps[1].offer(frame.image, metadata=metadata)
            self.camera.set_controls({'CnnEnableInputTensor': False})
            self.captured = True
        if elapsed > 10_000_000_000:
            self.done = True

    def close(self):
        try:
            self.camera.set_controls({'CnnEnableInputTensor': False})
        finally:
            for worker in self.workers:
                worker.stop()
        print('visiond: input tensor probe closed: ' + str([w.stats() for w in self.workers]), file=sys.stderr)
