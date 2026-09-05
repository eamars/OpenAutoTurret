"""The frame pipeline: §40's timings, §39's preview isolation, one owner per stage.

The order is fixed and short — capture → inference → normalize → class filter → §16 dedup →
association → selection → publish — and every stage between normalize and publish consumes the
*previous stage's output*, never a re-derivation of it. That ordering is also what gets recorded:
the recorder stores the normalized set *before* dedup, because the stages downstream of it are
the ones a replay needs to re-run with different thresholds (§43).

Three rules in this file that are easy to get wrong and expensive to fix later:

**Timings are named after what they measure** (§40). ``model_output_parse_ms`` is
``model_output_parse_start/end``; ``end_to_end_sensor_to_publish_ms`` is sensor stamp to publish
stamp; there is deliberately **no** metric called "inference latency" here, because §40 calls
out the retired habit of naming publish→controller time that. A number that lies about its own
boundaries is worse than no number: it gets compared against budgets derived for another
quantity.

**The preview cannot block inference** (§39). ``PreviewTap`` holds one frame; a new frame
overwrites the old one, and the encoder thread takes whatever is there. A queue with depth >1
would be a memory-bound latency guarantee, and a full queue that blocks the capture thread would
turn a browser refresh into a dropped detection — the retirement report's 15 fps → 10 fps
mystery, made structural.

**A stage that raises is counted and reported, not absorbed.** The pipeline catches the failure
at the frame boundary (one bad frame must not end a run), increments the right counter, emits an
event, and keeps the timing sample — so the p99 still shows the stall that accompanied it. The
retired code's habit of returning an empty result on error is what made "no detections" and
"no inference" indistinguishable at 3 a.m.
"""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional, Tuple

from .config import VisionConfig
from .detection.class_filter import filter_permitted


def filter_measurement_size(detection_set: Any, min_height: float, max_area: float = 0.60):
    """§36: drop boxes that cannot be a person-sized measurement, and count them.

    Two degenerate IMX500 outputs break the tracker and block selection of the real person:

    * a 1-2 px sliver at the frame edge (near-zero height) — passes the score/class gates,
      becomes a CONFIRMED track;
    * a box spanning most of the frame (the dark everything misread as one "person") — also
      becomes a CONFIRMED track.

    Both create a second candidate, and auto-selection requires exactly one — so either one makes
    the turret refuse to pick the real person. Height is the discriminator §37.1 argues for (a
    person 0.3% of the frame tall is not aimable); area is the upper bound (a single person never
    fills 60% of the frame in a scene you would track). This is a degenerate-box guard, and it
    counts the drop instead of silently discarding it (§16).
    """
    kept = [d for d in detection_set.detections
            if d.bbox.height >= min_height and d.bbox.area <= max_area]
    out = detection_set.with_detections(kept)
    out.counters.degenerate_rejected = (
        int(detection_set.counters.degenerate_rejected)
        + int(len(detection_set.detections) - len(kept)))
    return out
from .detection.dedup import DetectionDeduplicator
from .detection.types import DetectionSet
from .errors import PerceptionError
from .events import EventLog, EventType
from .measure import RingStats
from .protocol.jsonio import atomic_write_text, dumps
from .protocol.selected_target import SelectedTargetObservation
from .protocol.track_set import TrackSet
from .replay.recorder import Recorder
from .selection.target_selection_manager import TargetSelectionManager
from .tracking.diagnostics import AssociationDiagnostics
from .tracking.track_manager import TrackManager

#: §40's stages, in the order they happen. One RingStats each, named after its own boundary.
#: Upper bound on a credible sensor→publish span. Beyond this the two stamps are almost
#: certainly in different clock domains, and a wild value in a percentile table is worse than no
#: value at all: it looks like a latency finding.
_PLAUSIBLE_END_TO_END_NS = 60_000_000_000

STAGES: Tuple[str, ...] = (
    "capture_to_metadata_ms",
    "model_output_parse_ms",
    "coordinate_normalization_ms",
    "class_filter_ms",
    "dedup_ms",
    "association_ms",
    "selection_update_ms",
    "publish_ms",
    "frame_total_ms",
    "sensor_to_publish_ms",
)


@dataclass
class FrameOutcome:
    """What one frame produced. ``published`` is False only when a stage raised."""

    frame_sequence: int = 0
    sensor_timestamp_ns: int = 0
    detection_set: Optional[DetectionSet] = None
    track_set: Optional[TrackSet] = None
    observation: Optional[SelectedTargetObservation] = None
    timings_ms: Dict[str, float] = field(default_factory=dict)
    published: bool = False
    failure: str = ""
    stage: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {"frame_sequence": int(self.frame_sequence),
                "sensor_timestamp_ns": int(self.sensor_timestamp_ns),
                "detections": (len(self.detection_set.detections)
                               if self.detection_set is not None else 0),
                "tracks": (len(self.track_set.tracks) if self.track_set is not None else 0),
                "target_state": (self.observation.target_state.name
                                 if self.observation is not None else "NO_TARGET"),
                "selection_generation": (int(self.observation.selection_generation)
                                         if self.observation is not None else 0),
                "timings_ms": {key: round(value, 3) for key, value in self.timings_ms.items()},
                "published": bool(self.published), "failure": self.failure, "stage": self.stage}


class PreviewTap:
    """§39: a latest-only buffer between the camera owner and the encoder.

    Depth one, deliberately not configurable at runtime: the whole reason the retired tap cost
    five frames a second was that the preview path was on the critical path. Here the capture
    thread does nothing but publish a reference into a single slot, so the cost is a store.
    """

    def __init__(self, *, enabled: bool = True, fps: float = 10.0,
                 latest_queue_depth: int = 1, clock: Optional[Callable[[], int]] = None) -> None:
        if latest_queue_depth != 1:
            raise ValueError("§39 fixes the preview buffer at depth one; a deeper buffer is "
                             "a latency guarantee that memory has to keep")
        self.enabled = bool(enabled)
        self.fps = float(fps)
        self._minimum_interval_ns = (1_000_000_000.0 / self.fps) if self.fps > 0 else 0.0
        self._lock = threading.Lock()
        self._frame: Any = None
        # -1, not 0: "no frame yet" must not be expressible as a timestamp, or a clock that
        # happens to read 0 (a fake clock in a test, a monotonic clock at boot) would leave the
        # tap permanently un-rate-limited by falsiness rather than by measurement.
        self._enqueued_ns = -1
        self._clock = clock or time.monotonic_ns
        self.offered = 0
        self.enqueued = 0
        self.overwritten = 0
        self.rate_limited = 0
        #: Frames that arrived with no pixels at all (a synthetic or replay run). Counted
        #: separately so ``offered: 0`` cannot be misread as "the tap is dropping frames".
        self.no_pixels = 0
        self.taken = 0

    def offer(self, frame: Any, *, now_ns: Optional[int] = None) -> bool:
        """Put the newest frame in the slot. Never blocks, never raises on a full buffer."""
        if not self.enabled:
            return False
        if frame is None:
            self.no_pixels += 1
            return False
        now = int(now_ns if now_ns is not None else self._clock())
        self.offered += 1
        with self._lock:
            if self._minimum_interval_ns and self._enqueued_ns >= 0:
                if now - self._enqueued_ns < self._minimum_interval_ns:
                    self.rate_limited += 1
                    return False
            if self._frame is not None:
                self.overwritten += 1            # the old frame is discarded, §39's rule
            self._frame = frame
            self._enqueued_ns = now
            self.enqueued += 1
            return True

    def take(self) -> Any:
        with self._lock:
            frame, self._frame = self._frame, None
            if frame is not None:
                self.taken += 1
            return frame

    def stats(self) -> Dict[str, Any]:
        return {"enabled": bool(self.enabled), "fps": float(self.fps), "offered": self.offered,
                "enqueued": self.enqueued, "overwritten": self.overwritten,
                "rate_limited": self.rate_limited, "no_pixels": self.no_pixels,
                "taken": self.taken}


class JsonPublisher:
    """Write the two authoritative documents (§38) so another process can read them.

    ``atomic_write_text`` is a rename, so a reader never sees half a TrackSet: the failure mode
    this replaces was a controller parsing a file that was still being written, and concluding
    from a truncated field that nothing was selected.
    """

    def __init__(self, directory: str, *, track_set_name: str = "track_set.json",
                 selected_name: str = "selected_target.json") -> None:
        if not directory:
            raise ValueError("JsonPublisher needs a directory")
        self.directory = directory
        os.makedirs(directory, exist_ok=True)
        self.track_set_path = os.path.join(directory, track_set_name)
        self.selected_path = os.path.join(directory, selected_name)
        self.published = 0
        self.failures = 0
        self.last_error = ""

    def publish(self, track_set: TrackSet,
                observation: Optional[SelectedTargetObservation]) -> None:
        try:
            atomic_write_text(self.track_set_path, dumps(track_set.to_dict()))
            if observation is not None:
                atomic_write_text(self.selected_path, dumps(observation.to_dict()))
        except OSError as exc:
            self.failures += 1
            self.last_error = str(exc)
            raise
        self.published += 1


@dataclass
class PipelineCounters:
    frames: int = 0
    #: Successful writes of the two §38 documents — deliberately not "frames published". A frame
    #: can complete (``FrameOutcome.published``) while the write fails, and the two numbers mean
    #: different things to the person debugging a controller that stopped seeing targets.
    documents_written: int = 0
    failures: int = 0
    inference_failures: int = 0
    publish_failures: int = 0
    frames_with_detections: int = 0
    frames_with_tracks: int = 0
    empty_frames: int = 0
    clock_domain_rejected: int = 0
    stages_skipped: Dict[str, int] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {"frames": self.frames, "documents_written": self.documents_written,
                "failures": self.failures, "inference_failures": self.inference_failures,
                "publish_failures": self.publish_failures,
                "frames_with_detections": self.frames_with_detections,
                "frames_with_tracks": self.frames_with_tracks,
                "empty_frames": self.empty_frames,
                "clock_domain_rejected": self.clock_domain_rejected,
                "stages_skipped": dict(self.stages_skipped)}


class PerceptionPipeline:
    """One frame in, two documents out, with §40's timings attached."""

    def __init__(self, config: VisionConfig, *, adapter: Any,
                 event_log: Optional[EventLog] = None,
                 diagnostics: Optional[AssociationDiagnostics] = None,
                 recorder: Optional[Recorder] = None,
                 publisher: Optional[JsonPublisher] = None,
                 preview: Optional[PreviewTap] = None,
                 clock: Optional[Callable[[], int]] = None,
                 session_uuid: str = "",
                 publish_selection: bool = True) -> None:
        self.config = config
        self.adapter = adapter
        self.events = event_log if event_log is not None else EventLog()
        self.diagnostics = diagnostics
        self.recorder = recorder
        self.publisher = publisher
        self.preview = preview
        self.clock = clock or time.monotonic_ns
        self.publish_selection = publish_selection
        self.session_uuid = session_uuid
        self.manager = TrackManager(config, session_uuid=session_uuid,
                                    event_log=self.events, diagnostics=diagnostics)
        self.selector = TargetSelectionManager(config, event_log=self.events)
        # §36's degenerate-box floor (fraction of stream height). 0 = off. A sliver at
        # the frame edge is not a target; without this it blocks selection of the real one.
        self.min_measure_height = float(getattr(config, 'min_measure_height_norm', 0.03)
                                      or 0.03)
        self.deduplicator = DetectionDeduplicator(config.dedup, anchor_cfg=config.anchor,
                                                  event_log=self.events)
        self.permitted = tuple(config.active_model.permitted_classes or ())
        self.counters = PipelineCounters()
        self._stats: Dict[str, RingStats] = {
            stage: RingStats(stage, int(config.timing_window)) for stage in STAGES}
        self._last_sensor_ns = 0
        self._last: Dict[str, float] = {}
        self._unmeasured: set = set()
        self._running = False
        self._deselect_reasons: Dict[str, int] = {}

    # -- one frame ----------------------------------------------------------
    def process_frame(self, image: Any, metadata: Any = None, *,
                      frame_sequence: int, sensor_timestamp_ns: int,
                      capture_started_ns: Optional[int] = None,
                      publish: Optional[Callable[[TrackSet, SelectedTargetObservation], None]]
                      = None) -> FrameOutcome:
        """Run the §40 chain once. Never raises for a frame-level failure."""
        outcome = FrameOutcome(frame_sequence=int(frame_sequence),
                               sensor_timestamp_ns=int(sensor_timestamp_ns))
        marks: Dict[str, int] = {}
        self.counters.frames += 1

        def mark(name: str) -> None:
            marks[name] = self.clock()

        try:
            mark("inference_start")
            if capture_started_ns is not None:
                self._record("capture_to_metadata_ms", marks["inference_start"] -
                             int(capture_started_ns))
            try:
                dset = self.adapter.infer(
                    image, metadata, frame_sequence=int(frame_sequence),
                    sensor_timestamp_ns=int(sensor_timestamp_ns),
                    publish_timestamp_ns=self.clock())
            except PerceptionError as exc:
                self.counters.inference_failures += 1
                self.counters.failures += 1
                outcome.failure, outcome.stage = str(exc), "inference"
                self.events.emit(EventType.MODEL_REJECTED_INCOMPATIBLE,
                                 error=str(exc), frame_sequence=int(frame_sequence))
                return outcome
            mark("parse_end")
            # §40 asks for parse and normalization as separate stages, and both happen inside
            # the adapter — so the adapter reports them. Anything it does not report is listed
            # in ``stages_unmeasured`` rather than recorded as zero: a zero percentile reads as
            # "this stage is fast", which is exactly the wrong conclusion to draw from "this
            # stage was never measured".
            reported = dict(getattr(self.adapter, "last_timings_ms", {}) or {})
            for stage in ("model_output_parse_ms", "coordinate_normalization_ms"):
                if stage in reported:
                    self._record(stage, float(reported[stage]) * 1_000_000.0)
                else:
                    self._unmeasured.add(stage)

            outcome.detection_set = dset
            self.counters.frames_with_detections += 1 if dset.detections else 0
            self.counters.empty_frames += 1 if not dset.detections else 0

            mark("filter_start")
            filtered = filter_permitted(dset, self.permitted)
            mark("filter_end")
            self._record("class_filter_ms", marks["filter_end"] - marks["filter_start"])

            # §36: a degenerate box is not a measurement. The IMX500 occasionally emits a 1–2 px
            # sliver at the frame edge as a low-score "person"; it has near-zero height, so it
            # passes the score gate and the class filter, becomes a CONFIRMED track, and — because
            # the auto-select rule is "exactly one candidate" — it blocks selection of the *real*
            # person by making two candidates. Height is the discriminator §37.1 already argues for
            # (a "person" 0.3% of the frame tall is not a target you can aim at), so the floor here
            # rejects boxes below it rather than scoring them.
            if self.min_measure_height > 0.0:
                filtered = filter_measurement_size(filtered, self.min_measure_height)

            mark("dedup_start")
            result = self.deduplicator.run(filtered)
            mark("dedup_end")
            self._record("dedup_ms", marks["dedup_end"] - marks["dedup_start"])
            for reason in result.stages_skipped:
                self.counters.stages_skipped[reason] = \
                    self.counters.stages_skipped.get(reason, 0) + 1

            if self.recorder is not None:
                # Recorded BEFORE §16, so a replay can re-run the dedup decision (§43).
                self.recorder.record_frame(dset, camera={
                    "width": int(dset.stream_width), "height": int(dset.stream_height),
                    "roi": list(dset.roi) if dset.roi else None,
                    "preserve_aspect_ratio": bool(dset.preserve_aspect_ratio)})

            mark("association_start")
            track_set = self.manager.update(result.detections, int(sensor_timestamp_ns),
                                            image=image)
            mark("association_end")
            self._record("association_ms",
                         marks["association_end"] - marks["association_start"])
            self.counters.frames_with_tracks += 1 if track_set.tracks else 0

            mark("selection_start")
            observation = self.selector.update(track_set, int(sensor_timestamp_ns))
            mark("selection_end")
            self._record("selection_update_ms",
                         marks["selection_end"] - marks["selection_start"])

            outcome.track_set, outcome.observation = track_set, observation

            if self.preview is not None:
                self.preview.offer(image, now_ns=self.clock())
                mark("preview_enqueue")

            mark("publish_start")
            if publish is not None:
                publish(track_set, observation)
            elif self.publisher is not None:
                try:
                    self.publisher.publish(track_set, observation)
                    self.counters.documents_written += 1
                except OSError as exc:
                    self.counters.publish_failures += 1
                    self._note_publish_failure(str(exc))
            mark("publish_end")
            self._record("publish_ms", marks["publish_end"] - marks["publish_start"])

            outcome.published = True
            if self.recorder is not None:
                self.recorder.record_observations_frame(track_set, observation)
            self._last_sensor_ns = int(sensor_timestamp_ns)
            self._record("frame_total_ms", self.clock() - marks["inference_start"])
            self._record_end_to_end(int(sensor_timestamp_ns))
            outcome.timings_ms = {stage: value for stage, value in self._last.items()
                                  if value is not None}
            return outcome
        except Exception as exc:                                # noqa: BLE001
            # A frame must not end the run; it must also not disappear quietly.
            self.counters.failures += 1
            outcome.failure = f"{type(exc).__name__}: {exc}"
            outcome.stage = outcome.stage or "unknown"
            self.events.emit(EventType.PIPELINE_FRAME_FAULT, error=outcome.failure,
                             frame_sequence=int(frame_sequence), stage=outcome.stage)
            return outcome

    def _published_at(self) -> int:
        return self.clock()

    def _record_end_to_end(self, sensor_timestamp_ns: int) -> None:
        """§40's one cross-domain number, recorded only when the two clocks can be compared.

        ``sensor_to_publish_ms`` subtracts a sensor stamp from a publish stamp, so the two have
        to live in the same clock domain — on the station both are the monotonic clock. When
        they are not (a replay whose stamps are wall-clock, a camera feeding boot-time), the
        difference is a huge or negative number that would otherwise land in a percentile table
        and be read as a latency problem. §40 refuses to label publish→controller time
        "inference latency"; the same honesty applies here — an incomparable span is reported as
        unmeasured, not as a measurement.
        """
        span_ns = self._published_at() - int(sensor_timestamp_ns)
        if 0 <= span_ns <= _PLAUSIBLE_END_TO_END_NS:
            self._record("sensor_to_publish_ms", span_ns)
            return
        self._unmeasured.add("sensor_to_publish_ms")
        self.counters.clock_domain_rejected += 1

    def _note_publish_failure(self, error: str) -> None:
        self.events.emit(EventType.PUBLISH_FAILED, error=error)

    # -- timing -------------------------------------------------------------
    def _record(self, stage: str, duration_ns: int) -> None:
        if stage not in self._stats:
            return
        milliseconds = max(0.0, int(duration_ns) / 1_000_000.0)
        self._stats[stage].record(milliseconds)
        self._last[stage] = milliseconds

    def timing_report(self) -> Dict[str, Any]:
        """§40's report: p50/p95/p99/max per stage, over a bounded window (§40).

        ``stages_unmeasured`` is part of the report, not an aside: an operator comparing this
        against §40's budget list has to be able to tell "under budget" from "not instrumented".
        """
        report = {stage: stats.to_dict() for stage, stats in self._stats.items() if stats.total}
        if self._unmeasured:
            report["stages_unmeasured"] = sorted(self._unmeasured)
        return report

    def report(self) -> Dict[str, Any]:
        return {"counters": self.counters.to_dict(), "timing": self.timing_report(),
                "preview": self.preview.stats() if self.preview is not None else None,
                "adapter": self.adapter.describe() if hasattr(self.adapter, "describe") else {},
                "selection": self.selector.state_dict(),
                "events": self.events.counts()}

    def start(self) -> "PerceptionPipeline":
        self._running = True
        return self

    def stop(self) -> None:
        self._running = False
        if self.recorder is not None:
            self.recorder.close()

    def __enter__(self) -> "PerceptionPipeline":
        return self.start()

    def __exit__(self, *_exc: Any) -> None:
        self.stop()
