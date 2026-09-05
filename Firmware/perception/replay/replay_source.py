"""§43 Level B — replay: the same detections through the same code, on a machine with no camera.

Level B is the subsystem's most useful test instrument, and it is only honest if three things
hold:

**The recording is not modified on the way in.** Sensor timestamps, publish timestamps,
model generation, stream geometry and every detection field arrive exactly as recorded. A
replay that "helpfully" re-normalised or re-ordered the input is testing its own repair code.

**Replay refuses to pretend the configuration is unchanged.** §43 records the configuration
that produced the detections; a replay under *different* thresholds answers a different
question, which is often the interesting one ("would the new thresholds have merged those two
identities?"). So a mismatch is reported loudly on `config_mismatches`, and
``require_matching_config=True`` turns that report into a refusal. Silent drift between the
recording's thresholds and the replay's is exactly how §46's gates stop meaning anything.

**Missing data is an error, not an empty frame.** A recording with 400 of its 900 lines present
crashes a re-run at frame 401 if you ask for strictness — which is correct, because the
alternative is a "deterministic" comparison of two different lengths of footage.

Level A (recorded pixels through the real model) is deliberately *not* implemented here: on
this station the model runs on the sensor, so Level A needs hardware, and an offline Level-A
"replay" would be a fiction. `ReplaySource` will happily report that a recording is Level-A
only and point at the images instead.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

from ..detection.types import DetectionSet
from ..errors import ValidationError
from ..protocol.jsonio import ScanStats, iter_jsonl, load_mapping_document
from ..protocol.track_set import TrackSet
from .recorder import (CAMERA_FILE, DETECTIONS_FILE, GROUND_TRUTH_FILE, IMAGES_DIR,
                       MANIFEST_FILE, OBSERVATIONS_FILE, RECORDING_SCHEMA_VERSION,
                       RecordingManifest)


@dataclass
class ReplayFrame:
    """One recorded frame, in the order it was written."""

    frame_sequence: int
    sensor_timestamp_ns: int
    publish_timestamp_ns: int
    detection_set: Optional[DetectionSet] = None
    camera: Dict[str, Any] = field(default_factory=dict)
    extra: Dict[str, Any] = field(default_factory=dict)
    image_path: str = ""

    @property
    def has_detections(self) -> bool:
        return self.detection_set is not None


@dataclass
class ReplaySummary:
    """What the source actually contained. Printed at the end of every replay run."""

    directory: str = ""
    frames: int = 0
    frames_with_detections: int = 0
    images: int = 0
    lines_read: int = 0
    malformed_lines: int = 0
    out_of_order_frames: int = 0
    model_id: str = ""
    levels: List[str] = field(default_factory=list)
    config_mismatches: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {"directory": self.directory, "frames": int(self.frames),
                "frames_with_detections": int(self.frames_with_detections),
                "images": int(self.images), "lines_read": int(self.lines_read),
                "malformed_lines": int(self.malformed_lines),
                "out_of_order_frames": int(self.out_of_order_frames),
                "model_id": self.model_id, "levels": list(self.levels),
                "config_mismatches": list(self.config_mismatches),
                "notes": list(self.notes)}


def _dig(payload: Any, path: Sequence[str]) -> Any:
    current = payload
    for key in path:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


class ReplaySource:
    """A recording, opened for reading. Iterable; re-iterable; never mutates its files."""

    def __init__(self, directory: str, *, strict: bool = True,
                 require_matching_config: bool = False,
                 expected_model_id: str = "",
                 expected_stream: Optional[Tuple[int, int]] = None) -> None:
        if not os.path.isdir(directory):
            raise ValidationError(
                f"replay source {directory!r} is not a directory. A recording is a directory "
                f"(manifest + JSONL); pointing --replay at a single .jsonl cannot carry §43's "
                f"configuration or geometry.")
        self.directory = directory
        self.strict = bool(strict)
        self.require_matching_config = bool(require_matching_config)
        self.expected_model_id = expected_model_id
        self.expected_stream = expected_stream
        self.stats = ScanStats()
        manifest_path = os.path.join(directory, MANIFEST_FILE)
        if not os.path.exists(manifest_path):
            raise ValidationError(
                f"{directory} has no {MANIFEST_FILE}: §43's recording must state the model "
                f"and configuration that produced it, or the replay proves nothing")
        self.manifest = RecordingManifest.from_dict(
            load_mapping_document(manifest_path, what="recording manifest"))
        self.summary = ReplaySummary(directory=directory,
                                     model_id=self.manifest.model_id,
                                     levels=list(self.manifest.levels))
        self._camera_by_frame: Optional[Dict[int, Dict[str, Any]]] = None
        self._check_recording()

    # -- checks -------------------------------------------------------------
    def _check_recording(self) -> None:
        if self.manifest.schema_version != RECORDING_SCHEMA_VERSION:
            self.summary.notes.append(
                f"recording schema version {self.manifest.schema_version} != this build's "
                f"{RECORDING_SCHEMA_VERSION}; reading it anyway (field additions are ignored, "
                f"removals are not)")
        if "level_b" not in self.manifest.levels and \
                not os.path.exists(self.path(DETECTIONS_FILE)):
            self.summary.notes.append(
                "no detections.jsonl: this is a Level-A (image) recording. Level A replays on "
                "the station, where the network actually runs — see §43")
        for attribute, expected in (("model_id", self.expected_model_id),):
            if expected and attribute in self.manifest.__dict__ and \
                    getattr(self.manifest, attribute) and \
                    getattr(self.manifest, attribute) != expected:
                self.summary.config_mismatches.append(
                    f"{attribute}: recording says {getattr(self.manifest, attribute)!r}, "
                    f"this run wants {expected!r}")
        stream = self.manifest.stream or {}
        if self.expected_stream and stream:
            recorded = (int(stream.get("width", 0) or 0), int(stream.get("height", 0) or 0))
            if recorded != tuple(self.expected_stream):
                self.summary.config_mismatches.append(
                    f"stream: recording is {recorded[0]}x{recorded[1]}, this run is "
                    f"{self.expected_stream[0]}x{self.expected_stream[1]} — §14's coordinates "
                    f"would be reinterpreted")
        if self.require_matching_config and self.summary.config_mismatches:
            raise ValidationError(
                "replay configuration differs from the recording "
                f"({self.directory}):\n  - " + "\n  - ".join(self.summary.config_mismatches))

    def path(self, name: str) -> str:
        return os.path.join(self.directory, name)

    # -- reading ------------------------------------------------------------
    def frames(self) -> Iterator[ReplayFrame]:
        """Yield frames in written order, joining detections with camera metadata by frame.

        Order is preserved rather than sorted: §52's out-of-order/delayed-metadata case is a
        recording like any other, and a source that silently re-sorted it would make that test
        unable to reach the code it is meant to exercise.
        """
        camera = self._camera_index()
        detections_path = self.path(DETECTIONS_FILE)
        previous_sensor = 0
        if not os.path.exists(detections_path):
            for image in self._image_records():
                yield ReplayFrame(frame_sequence=int(image.get("frame_sequence", 0)),
                                  sensor_timestamp_ns=int(image.get("sensor_timestamp_ns", 0)),
                                  publish_timestamp_ns=0,
                                  image_path=self.path(str(image.get("path", ""))))
            return
        for record in iter_jsonl(detections_path, stats=self.stats,
                                 skip_malformed=not self.strict):
            payload = record.get("set")
            dset: Optional[DetectionSet] = None
            if payload:
                try:
                    dset = DetectionSet.from_dict(payload)
                except (ValidationError, KeyError, TypeError) as exc:
                    if self.strict:
                        raise ValidationError(
                            f"{detections_path}: unreadable DetectionSet at frame "
                            f"{record.get('frame_sequence')}: {exc}") from exc
                    self.stats.malformed += 1
                    continue
            sequence = int(record.get("frame_sequence",
                                      getattr(dset, "frame_sequence", 0)) or 0)
            sensor = int(record.get("sensor_timestamp_ns",
                                    getattr(dset, "sensor_timestamp_ns", 0)) or 0)
            if sensor and previous_sensor and sensor < previous_sensor:
                self.summary.out_of_order_frames += 1
            if sensor:
                previous_sensor = sensor
            joined = camera.get(sequence, {})
            self.summary.frames += 1
            if dset is not None:
                self.summary.frames_with_detections += 1
            yield ReplayFrame(frame_sequence=sequence, sensor_timestamp_ns=sensor,
                              publish_timestamp_ns=int(getattr(dset, "publish_timestamp_ns", 0)),
                              detection_set=dset, camera=dict(joined.get("camera", {})),
                              extra=dict(joined.get("extra", {})))

    def detection_sets(self) -> Iterator[DetectionSet]:
        """The Level-B input stream: recorded sets, unmodified, in recorded order."""
        for frame in self.frames():
            if frame.detection_set is not None:
                yield frame.detection_set

    def _camera_index(self) -> Dict[int, Dict[str, Any]]:
        if self._camera_by_frame is None:
            index: Dict[int, Dict[str, Any]] = {}
            path = self.path(CAMERA_FILE)
            if os.path.exists(path):
                for record in iter_jsonl(path, skip_malformed=not self.strict):
                    kind = record.get("kind")
                    sequence = int(record.get("frame_sequence", 0) or 0)
                    if kind == "camera":
                        index[sequence] = record
                    elif kind == "image":
                        self.summary.images += 1
            self._camera_by_frame = index
        return self._camera_by_frame

    def _image_records(self) -> List[Dict[str, Any]]:
        path = self.path(CAMERA_FILE)
        if not os.path.exists(path):
            return []
        return [record for record in iter_jsonl(path, skip_malformed=not self.strict)
                if record.get("kind") == "image"]

    def observations(self) -> List[Tuple[TrackSet, Optional[Any]]]:
        """What the recording's run published — the reference for a §45 comparison."""
        path = self.path(OBSERVATIONS_FILE)
        if not os.path.exists(path):
            return []
        out: List[Tuple[TrackSet, Optional[Any]]] = []
        from ..protocol.selected_target import SelectedTargetObservation
        for record in iter_jsonl(path, skip_malformed=not self.strict):
            track_payload = record.get("track_set")
            if not track_payload:
                continue
            track_set = TrackSet.from_dict(track_payload)
            selected_payload = record.get("selected")
            out.append((track_set,
                        SelectedTargetObservation.from_dict(selected_payload)
                        if selected_payload else None))
        return out

    def ground_truth(self) -> List[Dict[str, Any]]:
        path = self.path(GROUND_TRUTH_FILE)
        if not os.path.exists(path):
            return []
        return list(iter_jsonl(path, skip_malformed=not self.strict))

    def images_available(self) -> bool:
        return os.path.isdir(self.path(IMAGES_DIR)) and bool(self._image_records())

    def finish(self) -> ReplaySummary:
        """Finalise the summary. Call it *after* iterating: the counters count what was
        read. A summary taken before the walk would report an empty recording, which is
        the kind of result that turns a corrupt file into a passing test.
        """
        self.summary.lines_read = self.stats.lines_read
        self.summary.malformed_lines = self.stats.malformed
        if self.summary.malformed_lines and self.strict:
            raise ValidationError(
                f"{self.directory}: {self.summary.malformed_lines} malformed lines. A replay "
                f"that quietly loses frames is not the run you recorded (§46's gates are "
                f"per-minute and per-frame rates: they move when frames disappear).")
        return self.summary
