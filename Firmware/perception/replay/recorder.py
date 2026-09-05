"""§43 — the recorder: writing a run down well enough to argue about it later.

A recording exists so that a claim about tracking behaviour can be re-checked without the
scene. That goal decides the format:

* **One directory, not one file.** ``manifest.json`` holds the configuration, model identity
  and §9.1 environment record; frames stream into append-only JSONL. A crash mid-run then
  costs frames, not the whole recording — which matters because the interesting recordings are
  the ones that end badly.
* **Frames are one JSON line each, ordered by sensor time.** Diffable, greppable, and readable
  line-by-line without loading a minute of geometry into memory on a Pi.
* **Images are files, referenced by path** — never base64 in the JSON. §43 permits "frame or
  encoded image"; a Level-A recording with inline JPEG becomes unreadable at a few hundred
  megabytes, and the JSONL is the part a human inspects.
* **What the pipeline *decided* is recorded next to what it *saw*.** §45's metrics and §46's
  gates are about published TrackSets and selections. A recording that only contains
  detections cannot answer "why did the turret swing away", and re-deriving the answer depends
  on the code that is under suspicion.

Nothing here captures wall-clock "now" except as a creation stamp: the timestamps that matter
are the sensor stamps carried in the frames (§19), and every one of them is preserved exactly.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from ..config import VisionConfig
from ..detection.types import DetectionSet
from ..errors import ValidationError
from ..events import Event
from ..protocol.jsonio import JsonlWriter, atomic_write_text, dumps
from ..protocol.selected_target import SelectedTargetObservation
from ..protocol.track_set import TrackSet

RECORDING_SCHEMA_VERSION = 1

DETECTIONS_FILE = "detections.jsonl"
CAMERA_FILE = "camera.jsonl"
OBSERVATIONS_FILE = "observations.jsonl"
EVENTS_FILE = "events.jsonl"
MANIFEST_FILE = "manifest.json"
GROUND_TRUTH_FILE = "ground_truth.jsonl"
IMAGES_DIR = "images"


@dataclass
class RecordingManifest:
    """§43's "must preserve" list, as the first thing in the recording."""

    schema_version: int = RECORDING_SCHEMA_VERSION
    created_ns: int = 0
    levels: List[str] = field(default_factory=lambda: ["level_b"])
    model_id: str = ""
    model_sha256: str = ""
    model_generation: int = 1
    model_manifest: Dict[str, Any] = field(default_factory=dict)
    configuration: Dict[str, Any] = field(default_factory=dict)
    configuration_source: str = ""
    environment: Dict[str, Any] = field(default_factory=dict)
    stream: Dict[str, Any] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)
    frames_written: int = 0

    def to_dict(self) -> Dict[str, Any]:
        return {"schema_version": int(self.schema_version),
                "created_ns": int(self.created_ns), "levels": list(self.levels),
                "model_id": self.model_id, "model_sha256": self.model_sha256,
                "model_generation": int(self.model_generation),
                "model_manifest": dict(self.model_manifest),
                "configuration": dict(self.configuration),
                "configuration_source": self.configuration_source,
                "environment": dict(self.environment),
                "stream": dict(self.stream), "notes": list(self.notes),
                "frames_written": int(self.frames_written)}

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "RecordingManifest":
        data = data or {}
        return cls(
            schema_version=int(data.get("schema_version", RECORDING_SCHEMA_VERSION)),
            created_ns=int(data.get("created_ns", 0)),
            levels=list(data.get("levels", ["level_b"])),
            model_id=str(data.get("model_id", "")),
            model_sha256=str(data.get("model_sha256", "")),
            model_generation=int(data.get("model_generation", 1)),
            model_manifest=dict(data.get("model_manifest", {})),
            configuration=dict(data.get("configuration", {})),
            configuration_source=str(data.get("configuration_source", "")),
            environment=dict(data.get("environment", {})),
            stream=dict(data.get("stream", {})),
            notes=[str(note) for note in data.get("notes", [])],
            frames_written=int(data.get("frames_written", 0)))


class Recorder:
    """Appends one recording's files. Open once per run; ``close()`` is not optional."""

    def __init__(self, directory: str, *, config: Optional[VisionConfig] = None,
                 model_manifest: Any = None, environment: Optional[Dict[str, Any]] = None,
                 stream: Optional[Dict[str, Any]] = None,
                 record_detections: bool = True, record_images: bool = False,
                 record_observations: bool = True, record_events: bool = True,
                 flush_every: int = 32, notes: Optional[List[str]] = None,
                 allow_existing: bool = False) -> None:
        if not directory:
            raise ValueError("Recorder needs a target directory")
        self.directory = directory
        os.makedirs(directory, exist_ok=True)
        self.record_detections = bool(record_detections)
        self.record_images = bool(record_images)
        self.record_observations = bool(record_observations)
        self.record_events = bool(record_events)
        self.flush_every = max(1, int(flush_every))
        self.allow_existing = bool(allow_existing)
        levels = ["level_b"] if record_detections else []
        if record_images:
            levels.append("level_a")
        self.manifest = RecordingManifest(
            created_ns=time.time_ns(), levels=levels or ["observations"],
            model_id=getattr(model_manifest, "model_id", ""),
            model_sha256=getattr(model_manifest, "sha256", ""),
            model_manifest=dict(getattr(model_manifest, "to_dict", dict)()),
            configuration=dict(config.to_dict().get("vision", {})) if config else {},
            configuration_source=getattr(config, "source_path", "") if config else "",
            environment=dict(environment or {}), stream=dict(stream or {}),
            notes=list(notes or []))
        self._detections: Optional[JsonlWriter] = None
        self._camera: Optional[JsonlWriter] = None
        self._observations: Optional[JsonlWriter] = None
        self._events: Optional[JsonlWriter] = None
        self.frames = 0
        self.images = 0
        self._closed = False
        if record_images:
            os.makedirs(os.path.join(directory, IMAGES_DIR), exist_ok=True)

    # -- lifecycle ----------------------------------------------------------
    def open(self) -> "Recorder":
        existing = os.path.join(self.directory, MANIFEST_FILE)
        if os.path.exists(existing) and not self.allow_existing:
            # JSONL writers append, and an appended recording is not a recording: §46's gates
            # are per-minute and per-frame rates computed over what turns out to be two runs
            # wearing one directory. Failing here costs a directory name; failing at the gates
            # costs an afternoon of trusting a number.
            raise ValidationError(
                f"{self.directory} already holds a recording (§43 is one directory per run). "
                f"Choose a new directory, or pass allow_existing=True to append deliberately.")
        self._write_manifest()
        if self.record_detections:
            self._detections = JsonlWriter(self.path(DETECTIONS_FILE), self.flush_every)
        if self.record_detections or self.record_images:
            # The camera/index file is written for image-only recordings too: without it the
            # pixels on disk have no sensor stamp and no frame link, which is §43's Level A
            # turned into a folder of JPEGs nobody can replay.
            self._camera = JsonlWriter(self.path(CAMERA_FILE), self.flush_every)
        if self.record_observations:
            self._observations = JsonlWriter(self.path(OBSERVATIONS_FILE), self.flush_every)
        if self.record_events:
            self._events = JsonlWriter(self.path(EVENTS_FILE), self.flush_every)
        return self

    def path(self, name: str) -> str:
        return os.path.join(self.directory, name)

    def _write_manifest(self) -> None:
        # Rewritten on close too: `frames_written` is what tells a reader the recording ended
        # where the writer said it ended, rather than in the middle of a frame.
        atomic_write_text(self.path(MANIFEST_FILE), dumps(self.manifest.to_dict(), indent=2))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for writer in (self._detections, self._camera, self._observations, self._events):
            if writer is not None:
                writer.close()
        self.manifest.frames_written = self.frames
        self._write_manifest()

    def __enter__(self) -> "Recorder":
        return self.open()

    def __exit__(self, *_exc: Any) -> None:
        self.close()

    # -- records ------------------------------------------------------------
    def record_frame(self, dset: Optional[DetectionSet], *,
                     camera: Optional[Dict[str, Any]] = None,
                     image_bytes: Optional[bytes] = None,
                     image_extension: str = "jpg",
                     extra: Optional[Dict[str, Any]] = None) -> None:
        """Record one frame: its DetectionSet, its camera metadata, and optionally its pixels.

        ``camera`` is §43's "camera metadata required for coordinate conversion" — stream size,
        roi, whether the resize preserved aspect ratio, and any lens-shifting metadata. It is
        stored even when it repeats, because §14's mapping is only reproducible per frame: a
        recording whose geometry is reconstructed from the *last* frame's values is a recording
        that silently changes the coordinates it claims to preserve.
        """
        if self._closed:
            raise RuntimeError("Recorder is closed")
        self.frames += 1
        if self._detections is None and self._camera is not None and dset is not None:
            # Image-only recording: the frame still needs its stamp on the record.
            self._camera.write({"kind": "camera",
                                "frame_sequence": int(getattr(dset, "frame_sequence", -1)),
                                "sensor_timestamp_ns": int(
                                    getattr(dset, "sensor_timestamp_ns", 0)),
                                "camera": dict(camera or {}), "extra": dict(extra or {})})
        if self._detections is not None:
            self._detections.write({
                "kind": "detection_set",
                "frame_sequence": int(getattr(dset, "frame_sequence", -1)),
                "sensor_timestamp_ns": int(getattr(dset, "sensor_timestamp_ns", 0)),
                "set": dset.to_dict() if dset is not None else None})
            assert self._camera is not None
            self._camera.write({
                "kind": "camera",
                "frame_sequence": int(getattr(dset, "frame_sequence", -1)),
                "sensor_timestamp_ns": int(getattr(dset, "sensor_timestamp_ns", 0)),
                "camera": dict(camera or {}), "extra": dict(extra or {})})
        if image_bytes is not None and self.record_images:
            name = f"{int(getattr(dset, 'frame_sequence', self.frames)):06d}.{image_extension}"
            with open(os.path.join(self.directory, IMAGES_DIR, name), "wb") as handle:
                handle.write(image_bytes)
            self.images += 1
            # The pixels are a file; the JSONL keeps the link, so the two can be rejoined
            # without assuming the frame order survived.
            if self._camera is not None:
                self._camera.write({"kind": "image",
                                    "frame_sequence": int(getattr(dset, "frame_sequence", -1)),
                                    "sensor_timestamp_ns": int(
                                        getattr(dset, "sensor_timestamp_ns", 0)),
                                    "path": f"{IMAGES_DIR}/{name}",
                                    "bytes": len(image_bytes)})

    def record_observations_frame(self, track_set: TrackSet,
                                  selected: Optional[SelectedTargetObservation]) -> None:
        if self._observations is None:
            return
        self._observations.write({
            "kind": "observations",
            "frame_sequence": int(track_set.frame_sequence),
            "sensor_timestamp_ns": int(track_set.sensor_timestamp_ns),
            "track_set": track_set.to_dict(),
            "selected": selected.to_dict() if selected is not None else None})

    def record_event(self, event: Event) -> None:
        if self._events is None:
            return
        payload = event.to_dict()
        payload.setdefault("kind", "event")
        self._events.write(payload)

    def record_ground_truth(self, record: Dict[str, Any]) -> None:
        """An analyst's annotation (§44). Kept in its own file so it can arrive later."""
        with open(self.path(GROUND_TRUTH_FILE), "a", encoding="utf-8") as handle:
            handle.write(dumps(record) + "\n")

    def flush(self) -> None:
        for writer in (self._detections, self._camera, self._observations, self._events):
            if writer is not None:
                writer.flush()

    def stats(self) -> Dict[str, Any]:
        return {"directory": self.directory, "frames": int(self.frames),
                "images": int(self.images), "levels": list(self.manifest.levels),
                "closed": bool(self._closed)}


def write_ground_truth(directory: str, records: List[Dict[str, Any]]) -> str:
    """Replace a recording's ground-truth file (§44's annotation set)."""
    path = os.path.join(directory, GROUND_TRUTH_FILE)
    os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        for record in records:
            handle.write(dumps(record) + "\n")
    return path
