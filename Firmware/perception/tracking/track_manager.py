"""``TrackManager`` — identity formation, §17 through §27, in one place.

The pipeline in one paragraph: split detections by score band (§20), gate and optimally
assign high-score detections against live identities, let low-score detections rescue but
never create, advance every identity's state by **elapsed scene time**, merge identities the
resolver has watched collide for a dwell, enforce the capacity ceiling loudly, publish.

Four decisions are worth stating before the code, because each is a place where the obvious
alternative is wrong:

**Age is measured in milliseconds from ``SensorTimestamp``, never in frames (§19).** A
detector at 17 results per second and one at 30 must agree about whether an identity has
been lost. The retired path counted frames, so its thresholds silently changed meaning with
the inference rate — which is how a value tuned on a fast bench becomes an unsafe value on
the station.

**The prediction horizon is the detector interval, not each track's time since its last
measurement.** §22 says to derive the interval from ``SensorTimestamp``, and this honours
that literally: one ``dt`` per detector result, for every track. Extrapolating a track that
has been missing for 900 ms by 900 ms of its own last velocity sounds smarter and is how a
target last seen walking briskly gets reacquired three people down the line; §20 covers the
missing-identity case with a widened *gate* instead, which reaches further without inventing
displacement.

**A low-score measurement keeps an identity alive but is not a measurement (§20, §34).** It
does not refresh the age used by §19's timeouts and never sets ``measurement_valid``. An
identity sustained only by half-seen boxes therefore still expires at ``lost.retain_ms`` —
bounded by design, because "the detector half-saw something" is not a fact the controller
may aim with.

**Capacity overflow is reported, not absorbed (§26).** Twenty people and sixteen slots is a
scene the subsystem must describe honestly: ``detections_dropped_capacity``,
``tracks_evicted``, ``track_capacity``, published every frame. An unbounded or silently
truncated tracker turns a crowded scene into an identity-reliability bug report.

This module never touches CAN, a motor backend or a drive mode (§49). Its output is a
``TrackSet`` and some events.
"""
from __future__ import annotations

from dataclasses import replace
from typing import Dict, List, Optional, Tuple

from ..config import VisionConfig
from ..detection.types import Detection, DetectionSet, PointNorm
from ..events import EventLog, EventType
from ..measure import ms_from_ns
from ..protocol.track_set import TrackSet, TrackSetCounters
from ..selection.alias_map import AliasMap
from .byte_association import (AssociationOutcome, candidate_notes,
                               two_stage_associate)
from .camera_motion import CameraMotionProvider, NoCameraMotion
from .diagnostics import AssociationDiagnostics
from .duplicate_track_resolver import DuplicateTrackResolver, MergeDecision
from .track import Track, TrackState, new_session_uuid

#: Exponential weight applied to a new box over the remembered one. Light on purpose
#: (§36): the box is what the overlay draws and what the IoU term measures, so heavy
#: smoothing would make association disagree with the measurement, while the *anchor* — the
#: thing the controller aims with — stays unsmoothed and honest.
BOX_SMOOTHING = 0.45

#: Same idea for velocity. One frame's displacement is dominated by box jitter, and a
#: velocity estimate that follows it makes the prediction gate ring like a metronome.
VELOCITY_SMOOTHING = 0.5


class TrackManager:
    """Forms and maintains identities, and publishes them as a ``TrackSet``."""

    def __init__(self, config: VisionConfig, *, session_uuid: Optional[str] = None,
                 event_log: Optional[EventLog] = None,
                 diagnostics: Optional[AssociationDiagnostics] = None,
                 alias_map: Optional[AliasMap] = None,
                 camera_motion: Optional[CameraMotionProvider] = None,
                 appearance=None) -> None:
        self.config = config
        self.tracking = config.tracking
        self.thresholds = config.active_model.thresholds
        self.session_uuid = session_uuid or new_session_uuid()
        self.events = event_log
        # Explicit `is None` rather than `or`. AliasMap, EventLog and TrackSet all define
        # __len__, so an empty one is falsy, and `alias_map or AliasMap()` would silently
        # replace a freshly created shared map with a private one — which reads perfectly and
        # then loses every §25.1 alias the resolver wrote, turning a merge into a dangling
        # selection. The bug is invisible until the first merge.
        self.diagnostics = (AssociationDiagnostics(
            capacity=self.tracking.diagnostics_capacity,
            enabled=self.tracking.diagnostics_enabled)
            if diagnostics is None else diagnostics)
        self.aliases = AliasMap() if alias_map is None else alias_map
        self.camera_motion = NoCameraMotion() if camera_motion is None else camera_motion
        self.appearance = appearance
        self.resolver = DuplicateTrackResolver(self.tracking, config.dedup,
                                               diagnostics=self.diagnostics)

        self._tracks: List[Track] = []
        self._next_display_index: Dict[str, int] = {}
        self._selected_uuid: Optional[str] = None
        self._last_sensor_ns: int = 0
        self._sequence: int = 0
        self._model_id: str = config.active_model.model_id
        self._model_generation: int = 0
        self._stream_size: Tuple[int, int] = (0, 0)
        self._created_this_frame: Dict[int, str] = {}
        self.counters = TrackSetCounters(track_capacity=self.tracking.max_tracks)
        self.frames_processed = 0
        self.frames_out_of_order = 0
        self.last_outcome: Optional[AssociationOutcome] = None
        self.last_camera_shift: Optional[PointNorm] = None

    # -- identity surface (§29, §37.1) --------------------------------------
    @property
    def model_id(self) -> str:
        return self._model_id

    @property
    def model_generation(self) -> int:
        return self._model_generation

    def set_selected_uuid(self, track_uuid: Optional[str]) -> None:
        """§26 and §25.1 both need to know whom the operator chose."""
        self._selected_uuid = track_uuid or None
        self.resolver.note_selected(self._selected_uuid)

    def tracks(self) -> List[Track]:
        return [track.copy() for track in self._tracks
                if track.state is not TrackState.RETIRED and not track.alias_of]

    def find(self, track_uuid: str) -> Optional[Track]:
        """Look an identity up, following §25.1's alias if it merged away."""
        if not track_uuid:
            return None
        for track in self._tracks:
            if (track.track_uuid == track_uuid and not track.alias_of
                    and track.state is not TrackState.RETIRED):
                return track.copy()
        resolved = self.aliases.resolve(track_uuid)
        if resolved and resolved != track_uuid:
            for track in self._tracks:
                if track.track_uuid == resolved and not track.alias_of:
                    return track.copy()
        return None

    def exists(self, track_uuid: str) -> bool:
        """True only for an identity that still exists under its own name.

        A merged-away UUID deliberately answers ``False``: §Appendix A must be able to tell
        "gone" from "renamed", and the selector turns that difference into
        ``TRACK_NOT_FOUND`` versus ``TRACK_MERGED_USE_SURVIVOR``.
        """
        return any(track.track_uuid == track_uuid
                   and track.state is not TrackState.RETIRED
                   and not track.alias_of for track in self._tracks)

    def is_selectable(self, track_uuid: str) -> Tuple[bool, str]:
        """§37.1's gate, evaluated here so the browser, the selector and the tracker agree."""
        track = next((t for t in self._tracks if t.track_uuid == track_uuid), None)
        if track is None:
            return False, "unknown identity"
        if track.state is not TrackState.CONFIRMED_VISIBLE:
            return False, f"state is {track.state.label}"
        if track.ambiguous:
            return False, "identity is ambiguous (§32)"
        if track.duplicate_resolving:
            return False, "duplicate resolution in progress (§25)"
        selectable = self.thresholds.selectable
        if selectable is None:
            return False, "selectable threshold is COMMISSION (§50)"
        if track.detector_score < selectable:
            return False, (f"detector_score {track.detector_score:.2f} below "
                           f"selectable {selectable:.2f}")
        if track.identity_confidence < self.config.selection.select_min_identity_confidence:
            return False, (f"identity_confidence {track.identity_confidence:.2f} below "
                           f"{self.config.selection.select_min_identity_confidence:.2f}")
        return True, "selectable"

    def display_index_for(self, class_name: str) -> int:
        """§27: monotonic within a session, never reused, even after retirement.

        The retired path reused the lowest free label, which is the direct cause of the
        handover's ``select_target 2 -> "selected Person #1"``. A label becomes reusable
        only when a new ``session_uuid`` makes every old label meaningless; inside a session
        the numbers only go up.
        """
        key = (class_name or "target").strip().lower()
        nxt = self._next_display_index.get(key, 1)
        self._next_display_index[key] = nxt + 1
        return nxt

    # -- the frame ----------------------------------------------------------
    def update(self, dset: DetectionSet, now_ns: int, *, image=None) -> TrackSet:
        """Advance every identity by one detector result and publish the set."""
        dset.validate()                     # §14: nothing malformed gets this far
        sensor_ns = int(dset.sensor_timestamp_ns)
        dt_s = self._frame_interval(sensor_ns)
        self.frames_processed += 1
        self._model_id = dset.model_id
        self._model_generation = dset.model_generation
        self._stream_size = (dset.stream_width, dset.stream_height)
        self.counters.detections_in += len(dset.detections)
        self._created_this_frame = {}

        shift = self.camera_motion.shift_norm(sensor_ns, now_ns)
        self.last_camera_shift = shift

        detections = list(dset.detections)
        scorer = None
        if self.appearance is not None and getattr(self.appearance, "enabled", False):
            descriptors = self.appearance.describe_detections(image, detections) or {}
            if descriptors:
                from .appearance import AppearanceExtractor
                scorer = AppearanceExtractor.scorer_for(descriptors)

        if self.diagnostics.enabled:
            self.diagnostics.begin_frame(dset.frame_sequence, sensor_ns)

        outcome = two_stage_associate(
            self._tracks, detections, dt_s=dt_s, cfg=self.tracking,
            thresholds=self.thresholds, sensor_timestamp_ns=sensor_ns,
            appearance_scorer=scorer, shift=shift)
        self.last_outcome = outcome

        by_id = {detection.detection_id_in_frame: detection for detection in detections}
        self._apply_matches(outcome, by_id, sensor_ns, now_ns, dset.frame_sequence)
        self._age_unmatched(outcome, sensor_ns, now_ns)
        self._create_tracks(outcome, by_id, sensor_ns, now_ns)
        self._resolve_duplicates(sensor_ns)
        # Ambiguity before scoring: `selectable` consults `ambiguous`, so computing the gate
        # first would publish an ambiguous identity as selectable for one whole frame.
        self._flag_ambiguity(outcome)
        self._score_tracks(sensor_ns)
        if self.diagnostics.enabled:
            self._record_diagnostics(outcome, by_id, sensor_ns)
            self.diagnostics.end_frame()

        self._last_sensor_ns = sensor_ns
        return self.build_track_set(dset.frame_sequence, sensor_ns, now_ns)

    def _frame_interval(self, sensor_ns: int) -> float:
        """Seconds between detector results, from ``SensorTimestamp`` (§19, §22)."""
        if self._last_sensor_ns <= 0:
            return 0.0
        if sensor_ns <= self._last_sensor_ns:
            # A reordered or delayed metadata batch (§52) must not produce a negative dt,
            # and must not silently look like a stationary camera either: it is counted.
            self.frames_out_of_order += 1
            return 0.0
        interval_ms = ms_from_ns(sensor_ns, self._last_sensor_ns)
        # A stalled detector must not teleport a track: a four-second gap is association
        # evidence, not four seconds of extrapolation (§22).
        return min(interval_ms, self.tracking.max_predict_dt_ms) / 1000.0

    # -- pass application ---------------------------------------------------
    def _apply_matches(self, outcome: AssociationOutcome, by_id: Dict[int, Detection],
                       sensor_ns: int, now_ns: int, frame_sequence: int) -> None:
        for track_index, detection_id, quality in outcome.high_matches:
            detection = by_id.get(detection_id)
            if detection is not None:
                self._measure(self._tracks[track_index], detection, quality, sensor_ns,
                              now_ns, low_score=False)

        for track_index, detection_id, quality in outcome.reacquired:
            detection = by_id.get(detection_id)
            if detection is None:
                continue
            track = self._tracks[track_index]
            before = track.state
            # Measured BEFORE the update: after it, the age that explains the reacquisition
            # is gone, and §41's event would report "missed for 0 ms" on every one.
            miss_ms = (ms_from_ns(sensor_ns, track.last_measurement_ns)
                       if track.last_measurement_ns > 0 else 0.0)
            self._measure(track, detection, quality, sensor_ns, now_ns, low_score=False)
            track.just_reacquired = True
            self.counters.tracks_reacquired += 1
            self._emit(EventType.TRACK_REACQUIRED, track_uuid=track.track_uuid,
                       sensor_timestamp_ns=sensor_ns, frame_sequence=frame_sequence,
                       from_state=before.label, miss_ms=round(miss_ms, 1),
                       display_label=track.display_label)

        for track_index, detection_id, quality in outcome.low_matches:
            detection = by_id.get(detection_id)
            if detection is None:
                continue
            # §20's asymmetry, in one call: a rescue holds the identity and does not count
            # as a measurement, so the age clock keeps running and an identity kept alive
            # only by half-seen boxes still expires at ``lost.retain_ms``.
            self._measure(self._tracks[track_index], detection, quality, sensor_ns,
                          now_ns, low_score=True)

    def _measure(self, track: Track, detection: Detection, quality: float,
                 sensor_ns: int, now_ns: int, *, low_score: bool) -> None:
        """Fold one detection into a track's state."""
        before = track.state

        track.velocity_x, track.velocity_y = self._update_velocity(track, detection,
                                                                  sensor_ns)
        track.bbox = self._smooth_box(track.bbox, detection.bbox,
                                      first=not track.measurement_valid)
        track.anchor = detection.measured_anchor          # §36: measured, never smoothed
        track.anchor_source = detection.anchor_source
        track.keypoints = tuple(detection.keypoints)
        track.pose_score = detection.pose_score
        track.detector_score = detection.detector_score
        track.association_quality = quality
        track.measurement_quality = self._measurement_quality(detection)
        track.observations += 1
        track.miss_observations = 0
        track.last_receive_ns = now_ns
        track.just_reacquired = False

        if low_score:
            self.counters.low_score_associations += 1
            track.low_score_only_observations += 1
            track.state = TrackState.OCCLUDED
            track.measurement_valid = False
            if self.diagnostics.enabled:
                self.diagnostics.record_track(
                    track, state_before=before.label, state_after=track.state.label,
                    matched_detection=detection.detection_id_in_frame,
                    miss_age_ms=(ms_from_ns(sensor_ns, track.last_measurement_ns)
                                 if track.last_measurement_ns else 0.0))
            return                                    # the age clock stays where it was

        self._accumulate_visible_ms(track, sensor_ns)
        track.measurement_valid = True
        if track.last_measurement_ns <= 0:
            track.first_measurement_ns = sensor_ns
        track.last_measurement_ns = sensor_ns

        if track.state is TrackState.TENTATIVE and self._confirmed_enough(track, sensor_ns):
            track.state = TrackState.CONFIRMED_VISIBLE
            self.counters.tracks_confirmed += 1
            self._emit(EventType.TRACK_CONFIRMED, track_uuid=track.track_uuid,
                       sensor_timestamp_ns=sensor_ns, observations=track.observations,
                       visible_ms=round(track.visible_ms(sensor_ns), 1),
                       detector_score=round(track.detector_score, 4))
        elif track.state in (TrackState.OCCLUDED, TrackState.LOST_REACQUIRABLE):
            track.state = TrackState.CONFIRMED_VISIBLE

    def _confirmed_enough(self, track: Track, sensor_ns: int) -> bool:
        """§18.1 / §19: three observations *and* 120 ms visible, with no gap over 180 ms."""
        tentative = self.tracking.tentative
        if track.observations < tentative.min_observations:
            return False
        if track.visible_ms(sensor_ns) < tentative.min_visible_ms:
            return False
        return track.miss_observations == 0

    def _accumulate_visible_ms(self, track: Track, sensor_ns: int) -> None:
        """Close the previous visible span when a gap proves it ended, then extend."""
        if track.last_measurement_ns > 0:
            gap_ms = ms_from_ns(sensor_ns, track.last_measurement_ns)
            if gap_ms > self.tracking.tentative.max_gap_ms:
                if track.last_visible_span_start_ns > 0:
                    track.visible_ms_total += ms_from_ns(
                        track.last_measurement_ns, track.last_visible_span_start_ns)
                track.last_visible_span_start_ns = sensor_ns
        elif track.last_visible_span_start_ns <= 0:
            track.last_visible_span_start_ns = sensor_ns

    def _update_velocity(self, track: Track, detection: Detection,
                         sensor_ns: int) -> Tuple[float, float]:
        """EMA of observed displacement per second (§22's constant-velocity model)."""
        if track.last_measurement_ns <= 0 or sensor_ns <= track.last_measurement_ns:
            return track.velocity_x, track.velocity_y
        dt = ms_from_ns(sensor_ns, track.last_measurement_ns) / 1000.0
        if dt <= 0.0:
            return track.velocity_x, track.velocity_y
        observed_x = (detection.measured_anchor.x - track.anchor.x) / dt
        observed_y = (detection.measured_anchor.y - track.anchor.y) / dt
        alpha = VELOCITY_SMOOTHING
        return (alpha * observed_x + (1.0 - alpha) * track.velocity_x,
                alpha * observed_y + (1.0 - alpha) * track.velocity_y)

    @staticmethod
    def _smooth_box(previous, incoming, *, first: bool):
        if first or previous is None or not previous.is_well_formed():
            return incoming
        alpha = BOX_SMOOTHING
        return type(incoming)(
            alpha * incoming.x_min + (1.0 - alpha) * previous.x_min,
            alpha * incoming.y_min + (1.0 - alpha) * previous.y_min,
            alpha * incoming.x_max + (1.0 - alpha) * previous.x_max,
            alpha * incoming.y_max + (1.0 - alpha) * previous.y_max)

    @staticmethod
    def _measurement_quality(detection: Detection) -> float:
        """§37's fourth quality: how suitable this box is as an aim source.

        Height-driven, because the failure it measures is aiming at a person who is twenty
        pixels tall: the same detector confidence on a near target and a far one does not
        imply the same pointing accuracy, and one blended "confidence" hid that difference
        before (§3.5).
        """
        height = detection.bbox.height
        if height <= 0.0:
            return 0.0
        quality = min(1.0, height / 0.5)          # ≥ half the frame tall is as good as it gets
        if detection.has_pose:
            quality = min(1.0, quality + 0.1)     # a skeleton pins the anchor better
        return quality

    # -- ageing -------------------------------------------------------------
    def _age_unmatched(self, outcome: AssociationOutcome, sensor_ns: int,
                       now_ns: int) -> None:
        """Time-based lifecycle transitions for identities nobody measured (§18, §19)."""
        occluded_ms = self.tracking.occluded.max_ms
        retain_ms = self.tracking.lost.retain_ms
        for track_index in outcome.unmatched_tracks:
            track = self._tracks[track_index]
            if track.state is TrackState.RETIRED or track.alias_of:
                continue
            before = track.state
            track.miss_observations += 1
            track.measurement_valid = False
            track.just_reacquired = False
            track.last_receive_ns = now_ns
            reference = track.last_measurement_ns or track.created_ns
            miss_ms = ms_from_ns(sensor_ns, reference)

            if track.state is TrackState.TENTATIVE:
                if miss_ms > self.tracking.tentative.max_gap_ms:
                    # Never confirmed and the gap is too long: §18.1 already says this is
                    # not believable, so it goes rather than lingering as a candidate.
                    self._retire(track, sensor_ns, reason="tentative gap exceeded")
                continue

            if miss_ms > retain_ms:
                self._retire(track, sensor_ns, reason="lost retain_ms exceeded")
                continue
            if miss_ms > occluded_ms:
                if track.state is not TrackState.LOST_REACQUIRABLE:
                    track.state = TrackState.LOST_REACQUIRABLE
                    self.counters.tracks_lost += 1
                    self._emit(EventType.TRACK_LOST, track_uuid=track.track_uuid,
                               sensor_timestamp_ns=sensor_ns, miss_ms=round(miss_ms, 1),
                               retain_ms=retain_ms, from_state=before.label)
            elif track.state is TrackState.CONFIRMED_VISIBLE:
                track.state = TrackState.OCCLUDED
                self.counters.tracks_occluded += 1
                self._emit(EventType.TRACK_OCCLUDED, track_uuid=track.track_uuid,
                           sensor_timestamp_ns=sensor_ns, miss_ms=round(miss_ms, 1))
            if self.diagnostics.enabled:
                self.diagnostics.record_track(
                    track, state_before=before.label, state_after=track.state.label,
                    miss_age_ms=miss_ms, ambiguous=track.ambiguous)

    # -- creation and capacity (§26) ----------------------------------------
    def _create_tracks(self, outcome: AssociationOutcome, by_id: Dict[int, Detection],
                       sensor_ns: int, now_ns: int) -> None:
        new_threshold = self.thresholds.new_track
        if new_threshold is None:
            return          # §50: no commissioned creation threshold, so nothing is created
        candidates = [by_id[detection_id] for detection_id in outcome.new_track_candidates
                      if detection_id in by_id]
        # Score order first: if the ceiling forces a choice, it should be made on evidence
        # rather than on the order the detector emitted its rows.
        candidates.sort(key=lambda d: (-d.detector_score, d.detection_id_in_frame))

        dropped = 0
        refused: List[Tuple[Detection, str]] = []
        for detection in candidates:
            live = self._live_tracks()
            duplicate_of = self.resolver.duplicate_of_live(detection.bbox,
                                                           detection.class_name, live)
            if duplicate_of is not None:
                # The detection duplicates an identity that exists. Minting a second one here
                # is how a merge undoes itself one frame later, so this row is refused and
                # counted — never silently dropped (§26's rule, applied to creation).
                self.counters.detections_refused_duplicate += 1
                refused.append((detection, duplicate_of.track_uuid))
                continue
            if len(live) >= self.tracking.max_tracks and not self._evict_one(live, sensor_ns):
                dropped += 1
                continue
            if any(track.bbox.iou(detection.bbox) > 0.99 for track in self._live_tracks()):
                continue    # a detection cannot create a second identity on itself
            track = Track.create(
                detection_id=detection.detection_id_in_frame,
                class_id=detection.class_id, class_name=detection.class_name,
                bbox=detection.bbox, anchor=detection.measured_anchor,
                anchor_source=detection.anchor_source,
                detector_score=detection.detector_score,
                sensor_timestamp_ns=sensor_ns, receive_timestamp_ns=now_ns,
                display_index=self.display_index_for(detection.class_name),
                model_generation=self._model_generation,
                keypoints=detection.keypoints, pose_score=detection.pose_score)
            self._tracks.append(track)
            self.counters.tracks_created += 1
            self._measure(track, detection, 1.0, sensor_ns, now_ns, low_score=False)
            self._created_this_frame[detection.detection_id_in_frame] = track.track_uuid
            self._emit(EventType.TRACK_CREATED, track_uuid=track.track_uuid,
                       sensor_timestamp_ns=sensor_ns, display_label=track.display_label,
                       detector_score=round(detection.detector_score, 4))

        if refused and self.diagnostics.enabled:
            for detection, existing in refused:
                self.diagnostics.record_detection(
                    detection, suppressed=True,
                    dedup_reason=f"duplicate of live identity {existing[:8]}",
                    assigned_track=existing)

        if dropped:
            self.counters.detections_dropped_capacity += dropped
            self._emit(EventType.TRACK_CAPACITY_DROP, sensor_timestamp_ns=sensor_ns,
                       dropped=dropped, capacity=self.tracking.max_tracks,
                       live=len(self._live_tracks()))

    def _live_tracks(self) -> List[Track]:
        return [track for track in self._tracks
                if track.state is not TrackState.RETIRED and not track.alias_of]

    def _evict_one(self, live: List[Track], sensor_ns: int) -> bool:
        """§26's eviction order. Returns False when everything left is protected."""

        def priority(track: Track) -> Tuple[int, float]:
            selected = bool(self._selected_uuid
                            and track.track_uuid == self._selected_uuid)
            if selected and self.tracking.protect_selected:
                miss_ms = (ms_from_ns(sensor_ns, track.last_measurement_ns)
                           if track.last_measurement_ns else 0.0)
                if miss_ms <= self.tracking.lost.retain_ms:
                    # "Never evict the selected identity while its TTL is still active."
                    # The ninth rank, so the sorted scan below stops instead of evicting it.
                    return (9, 0.0)
            if track.state is TrackState.TENTATIVE:
                return (0, track.detector_score)           # least confident first
            if track.last_measurement_ns <= 0:
                return (1, 0.0)                            # no measurement history
            if track.state is TrackState.LOST_REACQUIRABLE:
                return (2, -(ms_from_ns(sensor_ns, track.last_measurement_ns)))
            return (3, track.identity_confidence)          # least continuous first

        for track in sorted(live, key=priority):
            if priority(track)[0] >= 9:
                return False
            self._retire(track, sensor_ns, reason="capacity eviction (§26)", evicted=True)
            return True
        return False

    def _retire(self, track: Track, sensor_ns: int, *, reason: str,
                evicted: bool = False) -> None:
        if track.state is TrackState.RETIRED:
            return
        track.state = TrackState.RETIRED
        track.measurement_valid = False
        track.appearance = None          # §24: identity and appearance data are discarded
        if evicted:
            self.counters.tracks_evicted += 1
        else:
            self.counters.tracks_retired += 1
        self.resolver.forget(track.track_uuid)
        self._emit(EventType.TRACK_RETIRED, track_uuid=track.track_uuid,
                   sensor_timestamp_ns=sensor_ns, reason=reason,
                   age_ms=round(track.identity_age_ms(sensor_ns), 1),
                   display_label=track.display_label)

    # -- duplicate identities (§25) -----------------------------------------
    def _resolve_duplicates(self, sensor_ns: int) -> None:
        live = self._live_tracks()
        self.resolver.note_selected(self._selected_uuid)
        by_uuid = {track.track_uuid: track for track in live}
        for track in live:
            track.duplicate_resolving = False      # re-derived from the resolver each frame

        for decision in self.resolver.observe(live, sensor_ns):
            self._apply_merge(decision, by_uuid, sensor_ns)

        # A pair that looks duplicate but has not reached the dwell is suppressed from
        # *selection* rather than merged (§25 permits the temporary suppression): the
        # operator cannot pick a second identity for one person, and the resolver does not
        # have to be right yet.
        for pair in self.resolver.pending_pairs():
            loser_uuid = self.resolver.prefer(by_uuid.get(pair.a_uuid),
                                              by_uuid.get(pair.b_uuid))
            loser = by_uuid.get(loser_uuid)
            if loser is not None:
                loser.duplicate_resolving = True

    def _apply_merge(self, decision: MergeDecision, by_uuid: Dict[str, Track],
                     sensor_ns: int) -> None:
        survivor = by_uuid.get(decision.survivor_uuid)
        merged = by_uuid.get(decision.merged_uuid)
        if survivor is None or merged is None or merged is survivor:
            return
        self.aliases.add(merged.track_uuid, survivor.track_uuid,
                         reason=f"{decision.reason}:{decision.survivor_rule}",
                         sensor_timestamp_ns=sensor_ns)
        merged.alias_of = survivor.track_uuid
        selected_was_merged = self._selected_uuid == merged.track_uuid
        self._retire(merged, sensor_ns, reason="merged duplicate (§25)")
        self.counters.tracks_merged += 1
        if selected_was_merged:
            # §25.1: the selection follows the alias. TrackManager only records the fact —
            # TargetSelectionManager owns the selection and resolves it on its next update.
            self._selected_uuid = survivor.track_uuid
            self.resolver.note_selected(survivor.track_uuid)
        self._emit(EventType.TRACK_MERGED, track_uuid=merged.track_uuid,
                   sensor_timestamp_ns=sensor_ns, survivor_uuid=survivor.track_uuid,
                   merged_label=merged.display_label, survivor_label=survivor.display_label,
                   rule=decision.survivor_rule, evidence=decision.evidence.to_dict(),
                   selection_followed=selected_was_merged)

    # -- scores, ambiguity, diagnostics -------------------------------------
    def _flag_ambiguity(self, outcome: AssociationOutcome) -> None:
        """§32: when two LOST identities could both explain one reacquisition, say so.

        Ambiguity is a *published state*, not a coin flip. The margin is §21's
        ``ambiguity_margin`` applied to the association cost: a rival within that band is a
        rival the numbers genuinely cannot separate, and choosing the marginally better one
        anyway is what produces the target-stealing result §46 counts and §53 asks about.
        """
        margin = self.tracking.gates.ambiguity_margin
        for track in self._live_tracks():
            track.ambiguous = False
            track.ambiguity_candidates = ()
        for track_index, detection_id, _quality in outcome.reacquired:
            best = outcome.cells.get((track_index, detection_id))
            if best is None:
                continue
            rivals = []
            for (other_index, other_detection), payload in outcome.cells.items():
                if other_detection != detection_id or other_index == track_index:
                    continue
                if self._tracks[other_index].state is not TrackState.LOST_REACQUIRABLE:
                    continue
                if payload.cost <= best.cost * (1.0 + margin) + 1e-9:
                    rivals.append(other_index)
            if not rivals:
                continue
            winner = self._tracks[track_index]
            winner.ambiguous = True
            winner.ambiguity_candidates = tuple(
                self._tracks[index].track_uuid for index in rivals)
            for index in rivals:
                rival = self._tracks[index]
                rival.ambiguous = True
                rival.ambiguity_candidates = tuple(
                    [winner.track_uuid] + [self._tracks[other].track_uuid
                                           for other in rivals if other != index])

    def _score_tracks(self, sensor_ns: int) -> None:
        """§37's separate fields, computed separately — never blended into one number."""
        min_identity = self.config.selection.select_min_identity_confidence
        selectable_threshold = self.thresholds.selectable
        for track in self._live_tracks():
            track.identity_confidence = self._identity_confidence(track, sensor_ns)
            track.selectable = bool(
                track.state is TrackState.CONFIRMED_VISIBLE
                and not track.ambiguous
                and not track.duplicate_resolving
                and selectable_threshold is not None
                and track.detector_score >= selectable_threshold
                and track.identity_confidence >= min_identity)

    @staticmethod
    def _identity_confidence(track: Track, sensor_ns: int) -> float:
        """Continuity and uniqueness of the identity — not the detector's confidence (§37).

        A documented weighted sum of observable history rather than a learned or guessed
        number: a person observed for two seconds across thirty measurements, in a frame the
        tracker never had to share with a rival, is a *certain* identity; a box that appeared
        one frame ago at 0.99 detector confidence is not. Those two must not be able to
        produce the same number, because §37.1 gates selection on this one.
        """
        observations = min(1.0, track.observations / 12.0)
        visible = min(1.0, track.visible_ms(sensor_ns) / 1500.0)
        association = max(0.0, min(1.0, track.association_quality))
        recency = 1.0
        if track.last_measurement_ns > 0:
            miss_ms = ms_from_ns(sensor_ns, track.last_measurement_ns)
            age_ms = max(track.identity_age_ms(sensor_ns), 1.0)
            recency = max(0.0, 1.0 - miss_ms / age_ms)
        penalty = 0.35 if track.ambiguous else 0.0
        if track.duplicate_resolving:
            penalty += 0.25
        confidence = (0.25 * observations + 0.30 * visible + 0.25 * association
                      + 0.20 * recency - penalty)
        return max(0.0, min(1.0, confidence))

    def _record_diagnostics(self, outcome: AssociationOutcome,
                            by_id: Dict[int, Detection], sensor_ns: int) -> None:
        notes = candidate_notes(self._tracks, outcome)
        matched_by_detection: Dict[int, str] = {}
        for track_index, detection_id, _quality in (list(outcome.high_matches)
                                                    + list(outcome.low_matches)
                                                    + list(outcome.reacquired)):
            matched_by_detection[detection_id] = self._tracks[track_index].track_uuid
        for detection in by_id.values():
            self.diagnostics.record_detection(
                detection,
                candidates=notes.get(detection.detection_id_in_frame, ()),
                assigned_track=matched_by_detection.get(detection.detection_id_in_frame),
                created_track=self._created_this_frame.get(detection.detection_id_in_frame))
        for track_index in outcome.unmatched_tracks:
            track = self._tracks[track_index]
            self.diagnostics.record_track(
                track, state_before=track.state.label, state_after=track.state.label,
                miss_age_ms=(ms_from_ns(sensor_ns, track.last_measurement_ns)
                             if track.last_measurement_ns else 0.0),
                ambiguous=track.ambiguous)

    # -- publication --------------------------------------------------------
    def build_track_set(self, frame_sequence: int, sensor_timestamp_ns: int,
                        now_ns: int) -> TrackSet:
        live = self._live_tracks()
        # Increment on the manager first, then snapshot: the published set must agree with
        # the tracker's own tally, or §43's replay would report an ambiguity the tracker
        # never counted.
        if any(track.ambiguous for track in live):
            self.counters.ambiguous_frames += 1
        counters = replace(self.counters)
        counters.track_capacity = self.tracking.max_tracks
        counters.track_capacity_used = len(live)
        self._sequence += 1
        return TrackSet(
            session_uuid=self.session_uuid,
            track_set_sequence=self._sequence,
            frame_sequence=frame_sequence,
            sensor_timestamp_ns=sensor_timestamp_ns,
            publish_timestamp_ns=now_ns,
            stream_width=self._stream_size[0],
            stream_height=self._stream_size[1],
            model_id=self._model_id,
            model_generation=self._model_generation,
            tracks=[track.copy() for track in live],
            counters=counters,
            events=self.events.to_dicts(limit=8) if self.events is not None else [])

    def stats(self) -> Dict[str, object]:
        return {"session_uuid": self.session_uuid,
                "frames_processed": self.frames_processed,
                "frames_out_of_order": self.frames_out_of_order,
                "counters": self.counters.to_dict(),
                "resolver": self.resolver.stats(),
                "aliases": len(self.aliases),
                "association": self.last_outcome.to_dict() if self.last_outcome else {},
                "camera_shift": (self.last_camera_shift.to_dict()
                                 if self.last_camera_shift is not None else None)}

    # -- small shared plumbing ---------------------------------------------
    def _emit(self, event_type: EventType, **fields) -> None:
        """Emit when an event log exists; never let reporting change behaviour."""
        if self.events is None:
            return
        sensor_timestamp_ns = int(fields.pop("sensor_timestamp_ns", 0))
        track_uuid = fields.pop("track_uuid", "")
        frame_sequence = int(fields.pop("frame_sequence", 0))
        self.events.emit(event_type, track_uuid=track_uuid,
                         sensor_timestamp_ns=sensor_timestamp_ns,
                         frame_sequence=frame_sequence, **fields)
