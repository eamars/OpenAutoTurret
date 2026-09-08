"""The IMX500 adapter: YOLO-style on-sensor post-processing through ``picamera2``.

Written against the **current** picamera2 API, which is §9's whole complaint about the retired
implementation. The supported shape is:

    imx500 = IMX500(model_path)
    intrinsics = imx500.network_intrinsics
    picam2 = Picamera2(imx500.camera_num)
    ...
    outputs = imx500.get_outputs(metadata)          # metadata from the captured request

Notably absent is ``Picamera2.postprocessing_config``, and notably *present* is
``network_intrinsics`` — which is where the task, the nominal rate, the box order, the box
normalization and the label list come from. The retired code hard-coded assumptions in place of
those readings, which is how a model swap silently produced coordinates in the wrong units.

The adapter therefore does three things in ``open()`` before it will infer anything: construct
the device, read what it declares, and **compare that declaration with the manifest** (§9.3).
Disagreement raises ``ModelRejected`` with every difference listed, because loading a model
whose output convention is guessed is the failure mode §14 exists to prevent — and it produces
results that look plausible while being wrong by a factor of 640.

Nothing here imports ``picamera2`` at module scope: it imports ``libcamera``, which exists only
on the station, and an adapter that cannot be imported off-hardware cannot be tested offline.
The device object arrives through ``imx500_factory``, defaulted to the real import.
"""
from __future__ import annotations

import time
from typing import Any, Callable, List, Optional, Tuple

import numpy as _np
from ..errors import NoInferenceForFrame

from ..config import AnchorConfig
from ..detection.types import DetectionSet, PointNorm, AnchorSource
from ..errors import ModelRejected
from .adapter import ModelAdapter


def _flat(tensor: Any) -> "_np.ndarray":
    return _np.asarray(tensor).reshape(-1)


def _assemble_split_outputs(outputs: Any, spec: Any) -> Optional[List[List[float]]]:
    """Turn the IMX500 ``_pp`` parallel tensors into the row matrix §13's contract expects.

    The sensor's on-sensor post-processor does not emit one ``[score, class, box…]`` row per
    detection; it emits a boxes tensor ``(N, 4)``, a scores vector ``(N,)``, a classes vector
    ``(N,)`` and a scalar count. (Measured on this station, 2026-09-05: ssd 320 and nanodet 416
    emit normalized ``yxyx`` boxes; efficientdet 320 emits ``yxyx`` boxes in 320×320 pixels.)
    Everything downstream — §14's normalization, §16, the tracker — was written for rows, so the
    assembly happens once, here, at the adapter, and the rest of the subsystem stays identical.

    The count tensor is trusted over the shape: the sensor pads its buffer to ``max_rows`` and a
    padded row holds a real-looking score. Reading rows up to ``count`` (capped at ``max_rows``)
    is what keeps the detector's padding out of the tracker, where it would surface as a
    persistent phantom identity (§23).
    """
    if not spec or outputs is None or len(outputs) == 0:
        return None
    try:
        boxes = _flat(outputs[int(spec["boxes"])])
        scores = _flat(outputs[int(spec["scores"])])
        classes = _flat(outputs[int(spec["classes"])])
        count = _flat(outputs[int(spec["count"])])
    except (IndexError, KeyError, TypeError, ValueError):
        return None
    if boxes.size < 4 or scores.size == 0 or classes.size == 0 or count.size == 0:
        return None
    count_value = int(round(float(count[0])))
    if count_value < 0:
        count_value = 0
    cap = int(spec.get("max_rows", 0) or 0) or int(boxes.size // 4)
    count_value = min(count_value, cap, int(boxes.size // 4), scores.size, classes.size)
    rows: List[List[float]] = []
    for index in range(count_value):
        # Box columns are yxyx for the _pp zoo, so a row is [score, class, y0, x0, y1, x1] and
        # §14's parse_row_box is instructed by the manifest's bbox_order.
        rows.append([float(scores[index]), float(classes[index]),
                     float(boxes[4 * index]), float(boxes[4 * index + 1]),
                     float(boxes[4 * index + 2]), float(boxes[4 * index + 3])])
    return rows
from .compatibility_probe import _as_rows
from .manifest import ModelManifest


class Imx500YoloAdapter(ModelAdapter):
    """One IMX500 network, admitted by its manifest, read through ``get_outputs``."""

    name = "imx500"

    def __init__(self, manifest: ModelManifest, *,
                 roi: Optional[Tuple[int, int, int, int]] = None,
                 imx500_factory: Optional[Callable[[str], Any]] = None,
                 anchor_cfg: Optional[AnchorConfig] = None,
                 generation: int = 1) -> None:
        super().__init__(manifest, generation=generation)
        self._roi = roi
        self.factory = imx500_factory
        self.anchor_cfg = anchor_cfg or AnchorConfig()
        self.device: Any = None
        self.intrinsics: Any = None
        self.camera_num: Optional[int] = None
        self.warnings: List[str] = []
        self.probe: Any = None
        self._batch_geometry_verified = False
        self._batch_geometry_rejected = False

    # -- lifecycle ----------------------------------------------------------
    def open(self, *, device: Any = None, camera: Any = None) -> None:
        self.camera = camera
        self.manifest.validate()
        if not self.manifest.path:
            raise ModelRejected(
                f"profile {self.manifest.model_id!r} has no model path configured. "
                f"Install the .rpk (imx500-models) or point the profile at one; a missing "
                f"model must stop startup loudly rather than run a camera with no network.")
        if self.factory is None:
            from .compatibility_probe import default_imx500_factory
            self.factory = default_imx500_factory
        try:
            self.device = device if device is not None else self.factory(self.manifest.path)
        except ModelRejected:
            raise
        except Exception as exc:                                # noqa: BLE001
            raise ModelRejected(
                f"IMX500 could not load {self.manifest.path}: {exc}") from exc

        self.intrinsics = getattr(self.device, "network_intrinsics", None)
        if self.intrinsics is None:
            self.intrinsics = self.manifest.verified_external_intrinsics()

        num = getattr(self.device, "camera_num", None)
        self.camera_num = int(num) if num is not None else None

        # §9.3: fail fast when the model's declaration and the manifest disagree.
        probe = self._probe_without_reopening()
        self.probe = probe
        self.warnings = self._admit(probe)
        if getattr(self.device, 'network_intrinsics', None) is None:
            self.warnings.append('metadata-free model: hash-verified external reference contract ' +
                                 self.manifest.external_contract_source)

        # ``network_intrinsics`` does not always carry the input dimensions, but
        # ``get_input_size()`` always answers. The device wins — §14 normalizes against these
        # numbers and the device is the thing doing the resizing — and the override is
        # recorded, because a manifest that is quietly corrected stops being a check.
        size = getattr(self.device, "get_input_size", lambda: None)()
        if size and len(size) >= 2:
            declared = (self.manifest.input_width, self.manifest.input_height)
            reported = (int(size[0]), int(size[1]))
            if reported != declared:
                self.warnings.append(
                    f"warning: input_size: manifest says {declared}, device reports "
                    f"{reported}; using the device's value for §14's normalization")
                self.manifest = self.manifest.with_runtime(
                    input_width=reported[0], input_height=reported[1])
        self.opened = True

    def _probe_without_reopening(self) -> Any:
        """Read the device we already constructed, rather than paying for a second one.

        ``IMX500()`` uploads firmware to the coprocessor. Probing by constructing a second
        instance would fight the first one for the device, and on a station that has just
        started that race is the difference between a boot and a hang.
        """
        from .compatibility_probe import ProbeResult, _flatten_intrinsics, _read

        probe = ProbeResult(model_path=self.manifest.path, model_id=self.manifest.model_id,
                            probed=True, source="imx500")
        probe.raw_intrinsics = _flatten_intrinsics(self.intrinsics)
        probe.task = _read(self.intrinsics, "task") or ""
        rate = _read(self.intrinsics, "inference_rate") or _read(self.intrinsics, "fps")
        probe.inference_rate_hz = float(rate) if rate else None
        order = _read(self.intrinsics, "bbox_order")
        probe.bbox_order = str(order).strip().lower() if order else None
        probe.bbox_normalized = _read(self.intrinsics, "bbox_normalization")
        probe.preserve_aspect_ratio = _read(self.intrinsics, "preserve_aspect_ratio")
        labels = _read(self.intrinsics, "labels")
        probe.labels = tuple(str(name) for name in labels) if labels else None
        probe.postprocess = _read(self.intrinsics, "postprocess")
        return probe

    def _admit(self, probe: Any) -> List[str]:
        from .compatibility_probe import admit

        try:
            return admit(self.manifest, probe)
        except ModelRejected as exc:
            self.device = None
            self.opened = False
            raise ModelRejected(str(exc)) from exc

    def close(self) -> None:
        self.opened = False
        self.device = None                       # the Picamera2 object's lifetime is the
        self.intrinsics = None                   # pipeline's, not this adapter's

    # -- inference ----------------------------------------------------------
    def infer(self, image: Any, metadata: Optional[Any] = None, *, frame_sequence: int,
              sensor_timestamp_ns: int, publish_timestamp_ns: int) -> DetectionSet:
        if not self.opened or self.device is None:
            raise ModelRejected("Imx500YoloAdapter.infer() before open()")
        if metadata is None:
            self.failures += 1
            raise ModelRejected(
                "get_outputs() needs the captured request's metadata: the IMX500 resizes and "
                "pads internally, so §14's mapping is per-request and cannot be assumed")
        read_started = time.monotonic_ns()
        outputs = self.device.get_outputs(metadata)
        self.last_read_ms = (time.monotonic_ns() - read_started) / 1_000_000.0
        if outputs is None:
            raise NoInferenceForFrame('camera frame has no new inference tensors')
        if not outputs:
            self.failures += 1
            raise ModelRejected("get_outputs() returned no tensors for this request")
        if self.manifest.uses_split_outputs():
            rows = _assemble_split_outputs(outputs, self.manifest.output_tensors)
        else:
            rows = _as_rows(outputs[0])
        if rows is None:
            self.failures += 1
            raise ModelRejected(
                f"unexpected output layout from {self.manifest.model_id}: expected rows of "
                f"[score, class, x1, y1, x2, y2] from on-sensor post-processing "
                f"(postprocess={self.manifest.postprocess})")
        if rows and len(rows[0]) < 6:
            self.failures += 1
            raise ModelRejected(
                f"output rows carry {len(rows[0])} columns; the manifest expects at least "
                f"6 (§9.3 disagreement discovered at runtime)")
        geometry = None
        anchors = {}
        mapping_started = time.monotonic_ns()
        if self.camera is not None:
            rows, anchors = self._map_rows_to_stream(rows, metadata)
            from ..detection.normalize import InferenceGeometry
            width, height = self.stream_size
            geometry = InferenceGeometry(width, height, width, height,
                                         bbox_order='xy', bbox_normalized=True)
        mapping_ms = (time.monotonic_ns()-mapping_started)/1e6
        self.inferences += 1
        result = self._rows_to_set(rows, frame_sequence=int(frame_sequence),
                                 sensor_timestamp_ns=int(sensor_timestamp_ns),
                                 publish_timestamp_ns=int(publish_timestamp_ns),
                                 anchor_cfg=self.anchor_cfg, geometry=geometry)
        if self.camera is not None:
            self.last_timings_ms['coordinate_mapping_ms'] = mapping_ms
        self.last_anchor_mapping = []
        for detection in result.detections:
            mapped = anchors.get(detection.detection_id_in_frame)
            if mapped is None:
                continue
            anchor, valid, original_box = mapped
            self.last_anchor_mapping.append({'detection_id': detection.detection_id_in_frame,
                'model_bbox_xyxy_norm': original_box,
                'post_crop_anchor': detection.measured_anchor.to_dict(),
                'mapped_anchor': anchor.to_dict(), 'anchor_valid': valid})
            detection.anchor_valid = valid
            if valid:
                detection.measured_anchor = anchor
                detection.anchor_source = AnchorSource.BBOX_TORSO
        return result

    def _map_rows_to_stream(self, rows, metadata):
        from ..detection.normalize import parse_row_box
        from ..errors import ValidationError
        width, height = self.stream_size
        iw, ih = self.manifest.input_width, self.manifest.input_height
        score, category, box_index = self.manifest.score_indices()
        mapped = []
        anchors = {}
        fraction = (self.anchor_cfg or AnchorConfig()).torso_fraction
        coordinates = []
        originals = {}
        for index, row in enumerate(rows):
            try:
                x0, y0, x1, y1 = parse_row_box(row[box_index:box_index+4], self.manifest.bbox_order,
                    input_width=iw, input_height=ih, normalized=self.manifest.bbox_normalized)
                if x1 <= x0 or y1 <= y0:
                    raise ValidationError('inverted input box')
                # Same captured ScalerCrop and sensor ROI transform as the official
                # example. The ISP preview is not a generic letterbox of the NN input.
                coordinates.append((y0/ih, x0/iw, y1/ih, x1/iw))
                # Calculate the anatomical point BEFORE the SDK clips the identity
                # box to the ISP crop. A taller detector view means these operations
                # do not commute: the station SDK probe reproduced 99 px of bias.
                # Map a one-input-pixel neighbourhood through the same SDK geometry;
                # its centre keeps the point independent of box clipping. Integer
                # rectangle rounding contributes only about one ISP pixel.
                u, v = .5*(x0+x1), y0 + fraction*(y1-y0)
                coordinates.append(((v-.5)/ih, (u-.5)/iw, (v+.5)/ih, (u+.5)/iw))
                originals[index] = [x0/iw, y0/ih, x1/iw, y1/ih]
            except ValidationError:
                pass  # normalizer still counts the invalid row below

        converted = None
        # This accelerated path is specific to the installed IMX500 SDK. Compare
        # every real coordinate on the first frame, and fall back on disagreement.
        # Rebuild crop/ROI geometry every frame from that request's metadata.
        if (type(self.device).__module__.startswith('picamera2.devices.imx500') and
                not getattr(self,'_batch_geometry_rejected',False)):
            from .sdk_geometry import Imx500FrameMapper
            try:
                converted = Imx500FrameMapper(self.device,self.camera,metadata).convert_many(coordinates)
            except (AttributeError,KeyError,TypeError,ValueError,ImportError):
                self._batch_geometry_rejected = True
            if converted is not None and not getattr(self,'_batch_geometry_verified',False):
                expected = [self.device.convert_inference_coords(c,metadata,self.camera) for c in coordinates]
                if converted != expected:
                    self._batch_geometry_rejected = True
                    converted = expected
                self._batch_geometry_verified = bool(coordinates) and not getattr(self,'_batch_geometry_rejected',False)
        if converted is None:
            converted = [self.device.convert_inference_coords(c,metadata,self.camera) for c in coordinates]
        rectangles = iter(converted)
        for index,row in enumerate(rows):
            coords = [float('nan')]*4
            if index in originals:
                x,y,w,h = next(rectangles)
                ax,ay,aw,ah = next(rectangles)
                coords = [x/width,y/height,(x+w)/width,(y+h)/height]
                anchor = PointNorm((ax+.5*aw)/width,(ay+.5*ah)/height)
                anchors[index] = (anchor,aw > 0 and ah > 0 and anchor.is_valid(),originals[index])
            mapped_row = list(row)
            mapped_row[box_index:box_index+4] = coords
            mapped.append(mapped_row)
        return mapped, anchors

    # -- reporting ----------------------------------------------------------
    def describe(self) -> dict:
        payload = super().describe()
        payload["camera_num"] = self.camera_num
        payload['coordinate_mapping'] = 'sdk_capture_metadata' if self.camera is not None else 'offline_geometry'
        payload['batch_geometry_verified'] = getattr(self,'_batch_geometry_verified',False)
        payload['batch_geometry_rejected'] = getattr(self,'_batch_geometry_rejected',False)
        payload["intrinsics_task"] = getattr(self.intrinsics, "task", None)
        payload["warnings"] = list(self.warnings)
        payload["probe"] = self.probe.to_dict() if self.probe is not None else None
        return payload
