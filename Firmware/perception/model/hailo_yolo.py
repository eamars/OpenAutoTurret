"""HailoRT adapter for the pinned Hailo-8 YOLOv8n COCO HEF.

This adapter deliberately turns one synchronous Hailo NMS result into the existing
``DetectionSet`` contract. It does not add another tracker, selector, preview stream,
or protocol. The commissioned profile is the fixed 640x480 IMX477 stream letterboxed
to the HEF's 640x640 RGB input.
"""
from __future__ import annotations

from contextlib import ExitStack
import hashlib
import os
import time
from typing import Any, List

import numpy as np

from ..errors import ModelRejected
from .adapter import ModelAdapter, resolve_artifact


class HailoYoloAdapter(ModelAdapter):
    """Blocking HailoRT InferVStreams adapter; one input frame yields one detection set."""

    name = "hailo"

    def __init__(self, manifest, *, generation: int = 1) -> None:
        super().__init__(manifest, generation=generation)
        self._stack: ExitStack | None = None
        self._infer = None
        self._input_name = ""
        self._output_name = ""
        self._device_id = ""
        self._architecture = ""
        self._artifact_path = ""
        self._last_model_inference_ms = 0.0

    def open(self) -> None:
        self.manifest.validate()
        gaps = self.manifest.commissioning_gaps(requires_artifact=True)
        if gaps:
            raise ModelRejected("Hailo model manifest is incomplete: " + "; ".join(gaps))
        self._artifact_path = os.path.realpath(resolve_artifact(self.manifest.path))
        if not os.path.isfile(self._artifact_path):
            raise ModelRejected(f"Hailo HEF does not exist: {self._artifact_path}")
        expected_sha = str(self.manifest.sha256).strip().lower()
        if len(expected_sha) != 64 or any(ch not in "0123456789abcdef" for ch in expected_sha):
            raise ModelRejected("Hailo manifest must pin a 64-character SHA-256")
        digest = hashlib.sha256()
        with open(self._artifact_path, "rb") as artifact:
            for block in iter(lambda: artifact.read(1024 * 1024), b""):
                digest.update(block)
        self.artifact_sha256 = digest.hexdigest()
        if self.artifact_sha256 != expected_sha:
            raise ModelRejected(
                f"Hailo HEF SHA-256 mismatch for {self._artifact_path}: "
                f"expected {expected_sha}, got {self.artifact_sha256}")

        if (self.manifest.input_width, self.manifest.input_height) != (640, 640):
            raise ModelRejected("this Hailo profile requires the pinned 640x640 HEF input")
        self._check_manifest()

        stack = ExitStack()
        try:
            from hailo_platform import (
                ConfigureParams,
                Device,
                FormatType,
                HEF,
                HailoStreamInterface,
                InferVStreams,
                InputVStreamParams,
                OutputVStreamParams,
                VDevice,
            )

            device_ids = Device.scan()
            if len(device_ids) != 1:
                raise ModelRejected(f"expected one Hailo device, found {len(device_ids)}: {device_ids}")
            with Device(device_ids[0]) as physical:
                board = physical.control.identify()
                self._architecture = str(board.device_architecture)
            if self._architecture != "HAILO8":
                raise ModelRejected(f"pinned HEF targets HAILO8; device reports {self._architecture}")

            hef = HEF(self._artifact_path)
            inputs = hef.get_input_vstream_infos()
            outputs = hef.get_output_vstream_infos()
            if len(inputs) != 1:
                raise ModelRejected(f"expected one input vstream; got {len(inputs)}")
            if tuple(inputs[0].shape) != (640, 640, 3):
                raise ModelRejected(f"HEF input shape is {inputs[0].shape}, expected 640x640x3")
            self._input_name = inputs[0].name
            self._check_outputs(outputs)

            vdevice = stack.enter_context(VDevice(device_ids=device_ids))
            configure = ConfigureParams.create_from_hef(hef, HailoStreamInterface.PCIe)
            groups = vdevice.configure(hef, configure)
            if len(groups) != 1:
                raise ModelRejected(f"expected one Hailo network group, got {len(groups)}")
            network_group = groups[0]
            input_params = InputVStreamParams.make(
                network_group, quantized=True, format_type=FormatType.UINT8)
            output_params = OutputVStreamParams.make(
                network_group, quantized=False, format_type=FormatType.FLOAT32)
            self._infer = stack.enter_context(InferVStreams(
                network_group, input_params, output_params))
            stack.enter_context(network_group.activate(network_group.create_params()))
            self._stack = stack
            self._device_id = str(device_ids[0])
            self.opened = True
        except ModelRejected:
            stack.close()
            raise
        except Exception as exc:  # noqa: BLE001 - runtime dependency failures are model refusal
            stack.close()
            raise ModelRejected(f"HailoRT could not open {self._artifact_path}: {exc}") from exc

    def close(self) -> None:
        self.opened = False
        self._infer = None
        stack, self._stack = self._stack, None
        if stack is not None:
            stack.close()

    def infer(self, image: Any, metadata: Any = None, *, frame_sequence: int,
              sensor_timestamp_ns: int, publish_timestamp_ns: int,
              camera_id: str = ""):
        if not self.opened or self._infer is None:
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
            result = self._infer.infer({self._input_name: tensor})
        except Exception as exc:  # noqa: BLE001 - frame failure is counted by the pipeline
            self.failures += 1
            raise ModelRejected(f"Hailo inference failed: {exc}") from exc
        inference_finished = time.monotonic_ns()
        self._last_model_inference_ms = (inference_finished - inference_started) / 1_000_000.0

        parse_started = time.monotonic_ns()
        rows = self._parse(result, frame, pad)
        self.last_read_ms = (time.monotonic_ns() - parse_started) / 1_000_000.0

        try:
            detection_set = self._rows_to_set(
                rows, frame_sequence=int(frame_sequence),
                sensor_timestamp_ns=int(sensor_timestamp_ns),
                publish_timestamp_ns=int(publish_timestamp_ns), **self._row_layout())
        except Exception as exc:  # noqa: BLE001 - invalid model coordinates are fatal to frame
            self.failures += 1
            raise ModelRejected(f"Hailo output violates the detection contract: {exc}") from exc
        self.last_timings_ms["model_inference_ms"] = round(self._last_model_inference_ms, 6)
        self.inferences += 1
        return detection_set

    def geometry(self):
        """Rows leave ``_parse`` already normalised to the frame (the letterbox pad is undone there,
        where it was made), so the mapping onto the declared stream is a plain scale.

        Station, 2026-10-02: the manifest's true ``preserve_aspect_ratio`` made the standard geometry
        undo the 140 px pad a second time, publishing every vertical coordinate stretched 1.78x
        about the centre -- a person at y 0.25..0.75 came out at 0.056..0.944. It ran the pitch loop
        at 1.78x its gain (the "detections move 1.5-1.75x the scene" finding), clamped anyone in the
        top or bottom 22% of the picture onto its edge, and was tuned around by the 0.22 head fraction.
        """
        geometry = super().geometry()
        geometry.preserve_aspect_ratio = False
        return geometry

    # -- hooks a sibling HEF (hailo_pose.HailoYoloPoseAdapter) overrides -------------------
    def _check_manifest(self) -> None:
        if self.manifest.labels != "coco":
            raise ModelRejected("the pinned YOLOv8n HEF requires the contiguous COCO-80 label map")
        if (self.manifest.bbox_order, self.manifest.bbox_normalized) != ("yxyx", True):
            raise ModelRejected("the Hailo NMS adapter requires normalized [ymin,xmin,ymax,xmax] boxes")

    def _check_outputs(self, outputs) -> None:
        if len(outputs) != 1:
            raise ModelRejected(f"expected one output vstream; got {len(outputs)}")
        self._output_name = outputs[0].name

    def _row_layout(self) -> dict:
        return {}

    def _parse(self, result, frame, pad: int) -> List[List[float]]:
        """One synchronous Hailo NMS result -> rows normalised to the frame that was fed."""
        try:
            batch = result[self._output_name]
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
        return rows

    def describe(self):
        report = super().describe()
        report.update({"hailo_device_id": self._device_id,
                       "device_architecture": self._architecture,
                       "artifact_path": self._artifact_path,
                       "artifact_sha256": getattr(self, "artifact_sha256", ""),
                       "model_inference_ms": round(self._last_model_inference_ms, 3)})
        return report
