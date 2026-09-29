"""HailoRT adapter for the pinned Hailo-8 YOLOv8n COCO HEF.

This adapter deliberately turns one synchronous Hailo NMS result into the existing
``DetectionSet`` contract. It does not add another tracker, selector, preview stream,
or protocol. The commissioned profile is a 640-wide leg letterboxed to the HEF's 640x640
RGB input, and the leg's height is allowed to differ per camera.

The adapter owns what is *per camera*: geometry, the padding it adds, its counters, its
``camera_id``. The chip is not per camera — see ``hailo_device``, which exists because H5
assumed otherwise and the station came down. Constructing this adapter alone opens its own
device, so a one-camera station behaves exactly as it did before; a second camera is handed
the first one's device and joins it.
"""
from __future__ import annotations

import os
import time
from typing import Any, List

import numpy as np

from ..errors import ModelRejected
from .adapter import ModelAdapter, resolve_artifact
from .hailo_device import HailoDevice


class HailoYoloAdapter(ModelAdapter):
    """Blocking HailoRT InferVStreams adapter; one input frame yields one detection set."""

    name = "hailo"

    def __init__(self, manifest, *, generation: int = 1, device: HailoDevice | None = None) -> None:
        super().__init__(manifest, generation=generation)
        # Passed in for the second camera; None here means this adapter opens (and owns) its own.
        self._owns_device = device is None
        self.device = device or HailoDevice()
        self._member = ""
        self._last_model_inference_ms = 0.0

    @property
    def artifact_sha256(self) -> str:
        return self.device.facts_for(self.camera_id)["artifact_sha256"]

    def open(self) -> None:
        self.manifest.validate()
        gaps = self.manifest.commissioning_gaps(requires_artifact=True)
        if gaps:
            raise ModelRejected("Hailo model manifest is incomplete: " + "; ".join(gaps))
        artifact_path = os.path.realpath(resolve_artifact(self.manifest.path))
        if not os.path.isfile(artifact_path):
            raise ModelRejected(f"Hailo HEF does not exist: {artifact_path}")
        expected_sha = str(self.manifest.sha256).strip().lower()
        if len(expected_sha) != 64 or any(ch not in "0123456789abcdef" for ch in expected_sha):
            raise ModelRejected("Hailo manifest must pin a 64-character SHA-256")

        if (self.manifest.input_width, self.manifest.input_height) != (640, 640):
            raise ModelRejected("this Hailo profile requires the pinned 640x640 HEF input")
        if self.manifest.labels != "coco":
            raise ModelRejected("the pinned YOLOv8n HEF requires the contiguous COCO-80 label map")
        if (self.manifest.bbox_order, self.manifest.bbox_normalized) != ("yxyx", True):
            raise ModelRejected("the Hailo NMS adapter requires normalized [ymin,xmin,ymax,xmax] boxes")

        # A camera id is usually not known yet at open() — the sensor is identified when it starts —
        # so a member is named by profile here and every *counter* is keyed later, by the camera id
        # that arrives with the frame. Refusals still name whoever asked.
        self._member = self.camera_id or f"hailo:{self.manifest.model_id}"
        self.device.open_for(self._member, artifact_path=artifact_path, expected_sha=expected_sha,
                              require_input=(640, 640, 3), profile=str(self.manifest.model_id))
        self.opened = True

    def close(self) -> None:
        self.opened = False
        member, self._member = self._member, ""
        if member:
            self.device.release(member)
        if self._owns_device:
            self.device.close()

    def infer(self, image: Any, metadata: Any = None, *, frame_sequence: int,
              sensor_timestamp_ns: int, publish_timestamp_ns: int,
              camera_id: str = ""):
        if not self.opened:
            raise ModelRejected("HailoYoloAdapter.infer() before open()")
        self.check_camera(camera_id)
        self.note_inference()
        frame = np.asarray(image)
        # The guard compares the frame against the leg this adapter was *configured* for, and the
        # vertical pad is computed from the frame. A literal 640x480 here was left over from the
        # standalone probes and it rejected every frame of the 640x360 ISP leg on the first
        # production boot -- loudly, which is the only reason this was findable in a log.
        if (frame.ndim != 3
                or (frame.shape[1], frame.shape[0]) != (int(self.stream_size[0]),
                                                        int(self.stream_size[1]))
                or frame.shape[2] < 3):
            raise ModelRejected(
                f"inference was configured for a {self.stream_size[0]}x{self.stream_size[1]} leg and "
                f"got a frame of shape {frame.shape}; the capture leg and the adapter disagree")
        # The width is a fact about the artifact: the HEF input is 640 wide and the frame is fed to
        # it without horizontal scaling. Only the vertical axis may vary, and it is centred.
        if frame.shape[1] != 640:
            raise ModelRejected(
                f"the Hailo input is 640 wide and this leg is {frame.shape[1]} wide; ask the sensor "
                "for a 640-wide lores leg")

        # Picamera2 RGB888 arrays are BGR byte order on this platform. The standalone
        # probe uses request.make_image(...).convert('RGB'); reversing these bytes gives
        # the same RGB pixels without retaining a libcamera request or PIL image.
        rgb = np.ascontiguousarray(frame[..., :3][..., ::-1])
        tensor = np.full((1, 640, 640, 3), 114, dtype=np.uint8)
        pad = (640 - int(frame.shape[0])) // 2      # 480 tall -> 80, matching the measured probes
        tensor[0, pad:pad + int(frame.shape[0]), :, :] = rgb

        inference_started = time.monotonic_ns()
        try:
            result = self.device.run(tensor, camera_id=camera_id or self.camera_id)
        except ModelRejected:
            self.failures += 1
            raise
        inference_finished = time.monotonic_ns()
        # Wall time through the shared device: the chip plus whatever this feed waited for the other
        # camera. The split lives in describe() as queue_wait_ms / device_ms, because "the model got
        # slower" and "this camera is starved" want different answers.
        self._last_model_inference_ms = (inference_finished - inference_started) / 1_000_000.0

        parse_started = time.monotonic_ns()
        try:
            batch = result[self.device.output_name]
            if not isinstance(batch, (list, tuple)) or len(batch) != 1:
                raise ValueError("expected one output batch for one input frame")
            class_outputs = batch[0]
            if not isinstance(class_outputs, (list, tuple)) or len(class_outputs) != 80:
                raise ValueError("expected Hailo NMS class output for all 80 COCO classes")
            rows: List[List[float]] = []
            for class_index, class_rows in enumerate(class_outputs):
                boxes = np.asarray(class_rows)
                if boxes.size == 0:
                    continue
                if boxes.ndim != 2 or boxes.shape[1] != 5:
                    raise ValueError(
                        f"NMS class slot {class_index} has shape {boxes.shape}, expected [N,5]")
                if not np.isfinite(boxes).all():
                    raise ValueError(f"NMS class slot {class_index} contains non-finite values")
                # The NMS boxes are normalised to the 640x640 tensor, and the tensor carries the
                # letterbox this method just added. Rows are promised to be normalised to the frame we
                # were fed, so the pad has to be undone here -- it was created here. The x axis is
                # whole-width and needs nothing; the y axis maps tensor row to leg row.
                #
                # Before this the rows left the tensor normalised, which was wrong for every leg: at
                # 480 the picture is 80 px in, so y was 33% too tall and offset by 12.5% of the frame.
                # The probes only ever reported fps and latency, so nothing noticed. On the 640x360
                # production leg the error is big enough to push boxes off the frame entirely, which
                # is how we got 91671 detections and zero tracks.
                scale_y = 640.0 / float(frame.shape[0])
                offset_y = pad / float(frame.shape[0])
                for ymin, xmin, ymax, xmax, score in boxes:
                    lo = float(ymin) * scale_y - offset_y
                    hi = float(ymax) * scale_y - offset_y
                    if hi <= 0.0 or lo >= 1.0:
                        # Entirely inside the letterbox: the network saw padding, not a sighting.
                        # Counted rather than dropped quietly -- a counter that moves is not a failure,
                        # but a silent drop is a lie about what the model reported.
                        self.detections_pad_dropped += 1
                        continue
                    rows.append([float(score), float(class_index),
                                 min(1.0, max(0.0, lo)), float(xmin),
                                 min(1.0, max(0.0, hi)), float(xmax)])
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            self.failures += 1
            raise ModelRejected(f"unexpected Hailo NMS output: {exc}") from exc
        self.last_read_ms = (time.monotonic_ns() - parse_started) / 1_000_000.0

        try:
            detection_set = self._rows_to_set(
                rows, frame_sequence=int(frame_sequence),
                sensor_timestamp_ns=int(sensor_timestamp_ns),
                publish_timestamp_ns=int(publish_timestamp_ns))
        except Exception as exc:  # noqa: BLE001 - invalid model coordinates are fatal to frame
            self.failures += 1
            raise ModelRejected(f"Hailo output violates the detection contract: {exc}") from exc
        self.last_timings_ms["model_inference_ms"] = round(self._last_model_inference_ms, 6)
        self.inferences += 1
        return detection_set

    def describe(self):
        report = super().describe()
        facts = self.device.facts_for(self.camera_id)
        report.update({key: facts[key] for key in
                       ("hailo_device_id", "device_architecture", "artifact_path",
                        "artifact_sha256", "shared_with", "members", "queue_wait_ms", "device_ms",
                        "device_served", "device_failures", "turn_contested")})
        report["model_inference_ms"] = round(self._last_model_inference_ms, 3)
        return report
