"""Shared fixtures for the §52 tracking and selection tests.

Two things every tracking test needs, stated once: a config whose thresholds are
**commissioned** (the shipped ``configs/perception_v1.json`` deliberately holds
``COMMISSION`` placeholders, so building a manager from it and then wondering why nothing is
ever created would test §50 rather than the tracker), and detections constructed from
*scene* quantities (centre, width, height) rather than box edges, so a test reads as "a
person at 0.3 walking right" instead of arithmetic.

Time is always passed in explicitly. Nothing here reads a wall clock, which is what makes
§52's "same input, same output" replay property testable at all.
"""
from __future__ import annotations

from typing import List, Optional, Sequence

from perception.config import (DedupConfig, ModelConfig, ScoreThresholds,
                               SelectionConfig, TrackingConfig, VisionConfig)
from perception.detection.types import (AnchorSource, BBox, Detection, DetectionCounters,
                                        DetectionSet, PointNorm)

#: First frame's sensor time: one hour, in nanoseconds. Tests subtract whole TTL windows
#: from it (§31's expiry cases), and a stamp at or below zero is not a timestamp — §19's
#: age helper deliberately answers "unknown" for one rather than guessing, which would make
#: an under-sized base clock look like a tracker that refuses to expire identities.
T0_NS = 3_600_000_000_000
FRAME_MS = 60.0                      # ≈16.7 results per second, §5's realistic cadence


def ms(seconds: float) -> int:
    return int(round(seconds * 1_000_000_000))


def at(frame_index: int, *, frame_ms: float = FRAME_MS) -> int:
    """Sensor timestamp of frame *frame_index*, spaced like a real detector."""
    return T0_NS + int(round(frame_index * frame_ms * 1_000_000))


def det(det_id: int, *, cx: float = 0.5, cy: float = 0.5, width: float = 0.1,
        height: float = 0.3, score: float = 0.9, class_id: int = 0,
        class_name: str = "person", keypoints=(),
        anchor: Optional[PointNorm] = None,
        anchor_source: AnchorSource = AnchorSource.BBOX_CENTER_FALLBACK) -> Detection:
    """A detection placed by its centre, so tests read as scenes rather than arithmetic."""
    bbox = BBox(cx - width / 2.0, cy - height / 2.0, cx + width / 2.0, cy + height / 2.0)
    return Detection(detection_id_in_frame=det_id, class_id=class_id,
                     class_name=class_name, detector_score=score, bbox=bbox,
                     measured_anchor=anchor or PointNorm(cx, cy),
                     anchor_source=anchor_source, keypoints=tuple(keypoints))


def dset(detections: Sequence[Detection], *, frame_index: int = 0,
         sensor_ns: Optional[int] = None, model_id: str = "unit-test-model",
         generation: int = 1, publish_ns: Optional[int] = None) -> DetectionSet:
    """A frame's worth of detections with the §13 stamps a real adapter would carry."""
    sensor = at(frame_index) if sensor_ns is None else int(sensor_ns)
    return DetectionSet(
        model_id=model_id, model_generation=generation, frame_sequence=frame_index + 1,
        sensor_timestamp_ns=sensor,
        publish_timestamp_ns=(sensor + ms(0.02)) if publish_ns is None else publish_ns,
        stream_width=1920, stream_height=1080, detections=list(detections),
        counters=DetectionCounters(raw_outputs=len(detections),
                                   post_model_nms=len(detections)))


def commissioned_thresholds(**overrides) -> ScoreThresholds:
    """The four §37.2 numbers a tracker cannot run without, at plausible values.

    ``confirmed_update`` above ``new_track`` above ``low_association``: the bands must not
    overlap, or a mid-score detection would both create and rescue in the same pass.
    """
    values = {"low_association": 0.10, "new_track": 0.30, "confirmed_update": 0.50,
              "selectable": 0.50}
    values.update(overrides)
    return ScoreThresholds(**values)


def commissioned_dedup(**overrides) -> DedupConfig:
    """§16's thresholds, also needed by §25's resolver, which borrows them deliberately."""
    values = {"nms_iou": 0.45, "containment_ratio": 0.85, "center_distance_norm": 0.05}
    values.update(overrides)
    return DedupConfig(**values)


def commissioned_config(*, tracking: Optional[TrackingConfig] = None,
                        selection: Optional[SelectionConfig] = None,
                        dedup: Optional[DedupConfig] = None,
                        thresholds: Optional[ScoreThresholds] = None,
                        max_tracks: int = 16) -> VisionConfig:
    """A production-shaped config with no ``COMMISSION`` placeholders.

    Tests use ``production=False`` validation; only §50's own tests assert that the shipped
    file *fails* production validation.
    """
    model = ModelConfig(profile_name="person_detect", model_id="unit-test-model",
                        adapter="mock", task="object_detection",
                        permitted_classes=("person",),
                        thresholds=thresholds or commissioned_thresholds())
    return VisionConfig(
        profile="person_detect", models={"person_detect": model},
        dedup=dedup or commissioned_dedup(),
        tracking=tracking or TrackingConfig(max_tracks=max_tracks),
        selection=selection or SelectionConfig())


def advance(manager, detections_by_frame, *, start_frame: int = 0, now_step_ms: float = 66.0,
            now_start_ns: int = 2_000_000_000):
    """Feed frames of detections through a manager, returning each published ``TrackSet``.

    ``now_ns`` (host receive time) is deliberately offset from sensor time: several §19
    behaviours distinguish "when the light hit the sensor" from "when the host got the
    result", and a test that sets them equal cannot tell those apart from a bug.
    """
    sets = []
    for offset, detections in enumerate(detections_by_frame):
        index = start_frame + offset
        now_ns = now_start_ns + int(round((start_frame + offset) * now_step_ms * 1e6))
        sets.append(manager.update(dset(list(detections), frame_index=index), now_ns))
    return sets



def track_at(cx: float, *, state=None, sensor_ns: int = T0_NS, index: int = 1,
             velocity_x: float = 0.0, velocity_y: float = 0.0, score: float = 0.9,
             height: float = 0.3, display_index: Optional[int] = None):
    """A track in a chosen state, without running the tracker to get there.

    Half of the §52 matrix is about behaviour that only exists in a state a live tracker
    takes seconds to reach (``LOST_REACQUIRABLE``, or two identities that have already
    collided). Building those tracks directly is not a shortcut around the lifecycle —
    those lifecycle paths are tested in :mod:`test_track_manager` — it is the only way to
    ask the resolver and the selector a question about a state at a specific instant.
    """
    from perception.tracking.track import Track, TrackState
    detection = det(index, cx=cx, score=score, height=height)
    track = Track.create(detection_id=index, class_id=0, class_name="person",
                         bbox=detection.bbox, anchor=detection.measured_anchor,
                         anchor_source=detection.anchor_source,
                         detector_score=score, sensor_timestamp_ns=sensor_ns,
                         receive_timestamp_ns=sensor_ns,
                         display_index=display_index or index)
    track.state = state if state is not None else TrackState.CONFIRMED_VISIBLE
    track.last_measurement_ns = sensor_ns
    track.first_measurement_ns = sensor_ns
    track.last_visible_span_start_ns = sensor_ns
    track.observations = 5
    track.measurement_valid = True
    track.association_quality = 1.0
    # A hand-built track also claims the scores the tracker would have derived, so that
    # selector tests can ask about permissions without replaying a scene to earn them.
    # Anything that tests how those numbers are *computed* drives TrackManager instead.
    track.identity_confidence = 0.75
    track.measurement_quality = 0.6
    track.selectable = True
    track.velocity_x = velocity_x
    track.velocity_y = velocity_y
    return track

def uuids(track_set) -> List[str]:
    return [track.track_uuid for track in track_set.tracks]


def by_label(track_set, label: int):
    return next((track for track in track_set.tracks if track.display_index == label), None)


def track_set_of(tracks: Sequence, *, sequence: int = 1, frame_index: int = 0,
                 sensor_ns: Optional[int] = None, session_uuid: str = "session-test"):
    """A published ``TrackSet`` around tracks built by :func:`track_at`.

    The selector consumes published sets, not the tracker's live objects — the same boundary
    the browser and the recorder consume. Building sets directly is what lets a §52 selection
    case start from "one identity is LOST and one is TENTATIVE" without replaying a whole
    scene to get there.
    """
    from perception.protocol.track_set import TrackSet, TrackSetCounters
    sensor = at(frame_index) if sensor_ns is None else int(sensor_ns)
    return TrackSet(session_uuid=session_uuid, track_set_sequence=int(sequence),
                    frame_sequence=frame_index + 1, sensor_timestamp_ns=sensor,
                    publish_timestamp_ns=sensor + ms(0.02),
                    stream_width=1920, stream_height=1080,
                    model_id="unit-test-model", model_generation=1,
                    tracks=list(tracks), counters=TrackSetCounters(track_capacity=16),
                    events=[])
