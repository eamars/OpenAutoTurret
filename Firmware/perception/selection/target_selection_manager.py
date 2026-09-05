"""``TargetSelectionManager`` — ownership of "which person is the target" (§30, §31).

One owner, one generation counter, one answer. The browser does not remember a selection and
paint it; it sends a ``track_uuid``, gets an authoritative ACK, and paints what the ACK said
(§29). The controller does not infer a selection from the track list; it receives one
observation per frame, including an explicit ``NO_TARGET`` (§34).

The rules that make this hard, in the order the code applies them:

**Idempotency before anything else** (§30). Re-selecting the current UUID is accepted and
reported as ``SELECTION_UNCHANGED`` **without** a generation bump — because the generation is
what the controller uses to decide whether to reset acquisition. A double-click, a retried
request and a script that re-runs must all be inert.

**Aliases are followed, and the distinction is kept** (§25.1, Appendix A). If the requested
UUID merged away, the answer is ``TRACK_MERGED_USE_SURVIVOR`` naming the survivor — the
request is *not* silently re-pointed, because the operator clicked an identity that no longer
means what they thought it meant. An *already-held* selection, in contrast, follows the alias
atomically and does not bump the generation: a merge is the tracker tidying itself, not a
change of target.

**Staleness is only claimed when it is true.** ``STALE_UI_TRACK_SET`` requires both that the
identity is gone *and* that the UI's advertised sequence is older than the daemon's. Blaming
stale UI for a track that never existed would teach everyone to ignore the reason code.

**A selection survives loss, bounded by the identity TTL** (§31). ``OCCLUDED`` and ``LOST``
keep the selection — that is the whole point of §18's reacquirable state — and the controller
is told which of the two it is looking at with ``measurement_valid``. When the TTL expires and
the identity is retired, the selection ends exactly once: one ``TARGET_STALE`` event, one
generation bump, an explicit ``NO_TARGET`` publish. Silence is the failure mode §34 exists to
prevent.

**Ambiguity retains and disables** (§32). Two identities could explain the one box: the
selection is not guessed at and not cleared; it is retained with ``target_state=AMBIGUOUS``
and ``measurement_valid=False``, so the controller stops updating its aim instead of jumping
to whichever identity the sort order happened to prefer.

Nothing here touches a motor, a CAN id or a drive mode (§49). The only outbound fact is the
observation, and it carries no lead angle for the same reason (§36).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from ..config import VisionConfig
from ..events import EventLog, EventType
from ..measure import ms_from_ns
from ..protocol.selected_target import SelectedTargetObservation, TargetState
from ..protocol.track_set import TrackSet
from .policy import AutoSelector, evaluate_selectability
from .protocol import (ClearTargetRequest, SelectTargetAck, SelectTargetRequest,
                       SelectionReason)
from ..tracking.track import Track, TrackState
from .alias_map import AliasMap

#: §34's ambiguity degree. Vision-1.0 publishes it binary-by-design: §32's decision is
#: binary ("chose neither"), and a graded value would imply a measured rivalry that only
#: exists once the association cost margin is carried out of §41's diagnostics.
AMBIGUOUS_DEGREE = 1.0


@dataclass
class TargetSelectionState:
    """The authoritative selection. Everything the controller may know, and nothing else."""

    selected_uuid: str = ""
    display_label: str = ""
    generation: int = 0
    selected_since_ns: int = 0
    last_valid_measurement_ns: int = 0
    target_state: TargetState = TargetState.NO_TARGET
    measurement_valid: bool = False
    stale_reported: bool = False

    @property
    def has_selection(self) -> bool:
        return bool(self.selected_uuid)

    def to_dict(self) -> Dict[str, Any]:
        return {"selected_track_uuid": self.selected_uuid,
                "display_label": self.display_label,
                "selection_generation": int(self.generation),
                "selected_since_ns": int(self.selected_since_ns),
                "target_state": self.target_state.name,
                "measurement_valid": bool(self.measurement_valid),
                "stale_reported": bool(self.stale_reported)}


class TargetSelectionManager:
    """Owns the selection, answers commands, and publishes one observation per frame."""

    def __init__(self, config: VisionConfig, *, alias_map: Optional[AliasMap] = None,
                 event_log: Optional[EventLog] = None,
                 on_selected: Optional[Callable[[Optional[str]], None]] = None) -> None:
        self.config = config
        # `is None`, not `or`: an empty AliasMap is falsy, and swapping in a private one
        # here would quietly undo §25.1's atomic selection hand-off. See TrackManager.
        self.aliases = AliasMap() if alias_map is None else alias_map
        self.events = event_log
        self.auto = AutoSelector(config.selection, config.active_model.thresholds)
        #: Wired to ``TrackManager.set_selected_uuid`` by the pipeline: §26 needs to know
        #: whom to protect from capacity eviction. Optional, and failures are counted rather
        #: than propagated — a selection that succeeded must not be reported as failed
        #: because an advisory notification tripped.
        self.on_selected = on_selected
        self.callback_failures = 0

        self.state = TargetSelectionState()
        self.last_sequence = 0
        self.last_sensor_timestamp_ns = 0
        self.frames_processed = 0
        self.rejections: Dict[str, int] = {}
        self._last_observation = SelectedTargetObservation.no_target(0, "")
        self._ambiguous_reported = False

    # -- commands -----------------------------------------------------------
    def select(self, request: SelectTargetRequest, track_set: TrackSet,
               now_ns: int) -> SelectTargetAck:
        """Apply Appendix A's ordering and return the authoritative ACK."""
        track_set.validate()
        self._observe_sequence(track_set)

        if not request.track_uuid:
            return self._reject(request, SelectionReason.TRACK_NOT_FOUND,
                                "no track_uuid in request", track_set)

        # 1. Idempotency, before anything can change state (§30).
        if request.track_uuid == self.state.selected_uuid:
            return self._accept(request, track_set, reason=
                                SelectionReason.SELECTION_UNCHANGED,
                                detail="selection already in force",
                                bump_generation=False)

        # 2. A UUID that merged away is a different truth from one that never existed.
        survivor = self.aliases.resolve(request.track_uuid)
        if survivor and survivor != request.track_uuid:
            if survivor == self.state.selected_uuid:
                return self._accept(request, track_set, reason=
                                    SelectionReason.SELECTION_UNCHANGED,
                                    detail=f"{request.track_uuid[:8]} merged into "
                                           f"{survivor[:8]}, which is already selected",
                                    bump_generation=False)
            return self._reject(request, SelectionReason.TRACK_MERGED_USE_SURVIVOR,
                                f"that identity merged into {survivor[:8]} (§25.1); "
                                f"select the survivor explicitly",
                                track_set)

        # 3. The identity must exist in the set the operator was looking at.
        track = track_set.by_uuid(request.track_uuid)
        if track is None:
            if self.state.has_selection and 0 < request.track_set_sequence_seen_by_ui \
                    < self.last_sequence:
                return self._reject(request, SelectionReason.STALE_UI_TRACK_SET,
                                    f"identity {request.track_uuid[:8]} is not in the "
                                    f"current track set (daemon at "
                                    f"{self.last_sequence}, UI at "
                                    f"{request.track_set_sequence_seen_by_ui})",
                                    track_set)
            return self._reject(request, SelectionReason.TRACK_NOT_FOUND,
                                f"identity {request.track_uuid[:8]} is not in this session",
                                track_set)

        # 4. §37.1's gate, evaluated by the same function the tracker's flag comes from.
        verdict = evaluate_selectability(
            track, thresholds=self.config.active_model.thresholds,
            selection=self.config.selection)
        if not verdict:
            return self._reject(request, verdict.reason, verdict.detail, track_set)

        # 5. Only now: a real change of target.
        return self._accept(request, track_set, reason=SelectionReason.ACCEPTED,
                            detail=f"selected {track.display_label}",
                            bump_generation=True, track=track, now_ns=now_ns)

    def clear(self, request: ClearTargetRequest, track_set: TrackSet,
              now_ns: int) -> SelectTargetAck:
        """Drop the selection because the operator said so (§31)."""
        self._observe_sequence(track_set)
        if not self.state.has_selection:
            # Acknowledged rather than refused: an operator who presses clear twice has not
            # made a mistake, and a red error toast for a no-op trains them to ignore the
            # ACK — which is the one channel §29 needs them to trust.
            echo = SelectTargetRequest(request_id=request.request_id,
                                       source=request.source)
            return SelectTargetAck(
                request_id=echo.request_id, accepted=True,
                reason=SelectionReason.NOTHING_TO_CLEAR, detail="nothing was selected",
                authoritative_track_set_sequence=self.last_sequence,
                selection_generation=self.state.generation,
                selection_unchanged=True)
        previous = self.state.display_label or self.state.selected_uuid[:8]
        self._clear_selection(reason="operator_clear", event=EventType.TARGET_CLEARED,
                              detail=f"cleared {previous}")
        ack = SelectTargetAck.accept(
            SelectTargetRequest(request_id=request.request_id, source=request.source),
            track_uuid="", display_label="", track_set_sequence=self.last_sequence,
            selection_generation=self.state.generation,
            reason=SelectionReason.CLEARED, detail=f"cleared {previous}")
        return ack

    # -- the frame ----------------------------------------------------------
    def update(self, track_set: TrackSet, now_ns: int) -> SelectedTargetObservation:
        """Recompute the observation for one published ``TrackSet``."""
        track_set.validate()
        self._observe_sequence(track_set)
        self.frames_processed += 1
        sensor_ns = track_set.sensor_timestamp_ns

        # The dwell machine is stepped every frame, including the ones where a selection
        # already exists: its state must be "reset while the operator holds the target", not
        # "paused", or it would resume a dwell that started before the operator intervened.
        decision = self.auto.evaluate(track_set.tracks, sensor_ns,
                                      selection_active=self.state.has_selection)
        if not self.state.has_selection and decision.track_uuid:
            self._auto_select(track_set, decision.track_uuid, now_ns, decision)
            return self._publish(track_set, now_ns)
        if not self.state.has_selection:
            return self._publish(track_set, now_ns)

        track = track_set.by_uuid(self.state.selected_uuid)
        if track is None:
            # §25.1: the identity may have merged rather than died. Following the alias here
            # is the atomic case — same person, new name — so it must not bump the
            # generation or the controller resets acquisition for a bookkeeping change.
            survivor = self.aliases.resolve(self.state.selected_uuid)
            if survivor and survivor != self.state.selected_uuid:
                moved = track_set.by_uuid(survivor)
                if moved is not None:
                    self.state.selected_uuid = survivor
                    self.state.display_label = moved.display_label
        track = track_set.by_uuid(self.state.selected_uuid)
        if track is None:
            # Gone, and not by merging: §31's TTL has elapsed and the tracker retired it.
            self._clear_selection(reason="identity_retired", event=EventType.TARGET_STALE,
                                  detail="selected identity exceeded its TTL and retired")
            return self._publish(track_set, now_ns)

        target_state, measurement_valid = _project_state(track)

        if track.ambiguous:
            # §32: keep the selection, publish the ambiguity, and take the measurement away.
            target_state = TargetState.AMBIGUOUS
            measurement_valid = False
            if not self._ambiguous_reported:
                self._ambiguous_reported = True
                self._emit(EventType.TARGET_AMBIGUOUS,
                           track_uuid=track.track_uuid, sensor_ns=sensor_ns,
                           candidates=list(track.ambiguity_candidates),
                           display_label=track.display_label)
        elif self._ambiguous_reported:
            self._ambiguous_reported = False

        if measurement_valid:
            self.state.last_valid_measurement_ns = sensor_ns
            self.state.stale_reported = False
        elif self._check_ttl(track, sensor_ns):
            # The selection ended under us; writing state onto the fresh (empty) selection
            # object would publish a LOST target with no subject.
            return self._publish(track_set, now_ns)

        self.state.target_state = target_state
        self.state.measurement_valid = measurement_valid
        self.state.display_label = track.display_label
        return self._publish(track_set, now_ns)

    def _check_ttl(self, track: Track, sensor_ns: int) -> bool:
        """§31: a selection may coast on LOST, but not past the identity's own TTL.

        Returns whether the selection ended, so the caller does not go on to publish a state
        belonging to an identity that no longer owns the turret.
        """
        retain_ms = self.config.tracking.lost.retain_ms
        reference = track.last_measurement_ns or track.created_ns
        if reference <= 0:
            return False
        miss_ms = ms_from_ns(sensor_ns, reference)
        if miss_ms <= retain_ms:
            return False
        self._clear_selection(
            reason="selection_ttl_expired", event=EventType.TARGET_STALE,
            detail=f"no measurement for {miss_ms:.0f} ms (TTL {retain_ms:.0f} ms)")
        return True

    def _auto_select(self, track_set: TrackSet, track_uuid: str, now_ns: int,
                     decision) -> None:
        track = track_set.by_uuid(track_uuid)
        if track is None:
            return
        self.state.selected_uuid = track.track_uuid
        self.state.display_label = track.display_label
        self.state.generation += 1
        self.state.selected_since_ns = now_ns
        self.state.stale_reported = False
        self._notify_tracker()
        self._emit(EventType.TARGET_SELECTED, track_uuid=track.track_uuid,
                   sensor_ns=track_set.sensor_timestamp_ns,
                   display_label=track.display_label,
                   selection_generation=self.state.generation,
                   auto_selected=True, dwell_ms=round(decision.dwell_ms, 1))

    # -- publication --------------------------------------------------------
    def _publish(self, track_set: TrackSet, now_ns: int) -> SelectedTargetObservation:
        if not self.state.has_selection:
            observation = SelectedTargetObservation.no_target(
                self.state.generation, track_set.session_uuid,
                frame_sequence=track_set.frame_sequence,
                sensor_timestamp_ns=track_set.sensor_timestamp_ns,
                publish_timestamp_ns=now_ns)
        else:
            track = track_set.by_uuid(self.state.selected_uuid)
            if track is None:                    # cleared above; belt and braces
                return self._publish_empty(track_set, now_ns)
            state, valid = _project_state(track)
            observation = SelectedTargetObservation.from_track(
                track, selection_generation=self.state.generation,
                session_uuid=track_set.session_uuid,
                frame_sequence=track_set.frame_sequence,
                sensor_timestamp_ns=track_set.sensor_timestamp_ns,
                publish_timestamp_ns=now_ns,
                target_state=(TargetState.AMBIGUOUS if track.ambiguous else state),
                measurement_valid=valid and not track.ambiguous,
                ambiguity=AMBIGUOUS_DEGREE if track.ambiguous else 0.0)
        observation.validate()
        self._last_observation = observation
        return observation

    def _publish_empty(self, track_set: TrackSet,
                       now_ns: int) -> SelectedTargetObservation:
        observation = SelectedTargetObservation.no_target(
            self.state.generation, track_set.session_uuid,
            frame_sequence=track_set.frame_sequence,
            sensor_timestamp_ns=track_set.sensor_timestamp_ns,
            publish_timestamp_ns=now_ns)
        observation.validate()
        self._last_observation = observation
        return observation

    def observation(self) -> SelectedTargetObservation:
        """The last published observation, for a client that asked rather than subscribed."""
        return self._last_observation

    # -- internals ----------------------------------------------------------
    def _observe_sequence(self, track_set: TrackSet) -> None:
        if track_set.track_set_sequence >= self.last_sequence:
            self.last_sequence = track_set.track_set_sequence
        self.last_sensor_timestamp_ns = track_set.sensor_timestamp_ns

    def _accept(self, request: SelectTargetRequest, track_set: TrackSet, *,
                reason: SelectionReason, detail: str, bump_generation: bool,
                track: Optional[Track] = None, now_ns: int = 0) -> SelectTargetAck:
        if track is not None:
            changed = track.track_uuid != self.state.selected_uuid
            self.state.selected_uuid = track.track_uuid
            self.state.display_label = track.display_label
            self.state.selected_since_ns = now_ns
            self.state.last_valid_measurement_ns = track_set.sensor_timestamp_ns
            self.state.stale_reported = False
            self._ambiguous_reported = False
            if bump_generation and changed:
                self.state.generation += 1
            self._notify_tracker()
            self._emit(EventType.TARGET_SELECTED, track_uuid=track.track_uuid,
                       sensor_ns=track_set.sensor_timestamp_ns,
                       display_label=track.display_label,
                       selection_generation=self.state.generation,
                       source=request.source)
        elif reason is SelectionReason.SELECTION_UNCHANGED:
            self._emit(EventType.TARGET_SELECTION_IDEMPOTENT,
                       track_uuid=self.state.selected_uuid,
                       sensor_ns=track_set.sensor_timestamp_ns,
                       request_id=request.request_id, detail=detail)
        return SelectTargetAck.accept(
            request, track_uuid=self.state.selected_uuid,
            display_label=self.state.display_label,
            track_set_sequence=self.last_sequence,
            selection_generation=self.state.generation,
            reason=reason, detail=detail)

    def _reject(self, request: SelectTargetRequest, reason: SelectionReason, detail: str,
                track_set: TrackSet) -> SelectTargetAck:
        self.rejections[reason.value] = self.rejections.get(reason.value, 0) + 1
        self._emit(EventType.TARGET_SELECTION_REJECTED,
                   track_uuid=self.state.selected_uuid,
                   sensor_ns=track_set.sensor_timestamp_ns,
                   requested_uuid=request.track_uuid, reason=reason.value,
                   detail=detail, request_id=request.request_id)
        return SelectTargetAck.reject(
            request, reason, detail=detail, track_set_sequence=self.last_sequence,
            selection_generation=self.state.generation,
            selected_track_uuid=self.state.selected_uuid,
            selected_display_label=self.state.display_label)

    def _clear_selection(self, *, reason: str, event: EventType, detail: str) -> None:
        if not self.state.has_selection:
            return
        self._emit(event, track_uuid=self.state.selected_uuid,
                   sensor_ns=self.last_sensor_timestamp_ns, reason=reason, detail=detail,
                   display_label=self.state.display_label)
        self.state = TargetSelectionState(generation=self.state.generation + 1)
        self._ambiguous_reported = False
        self._notify_tracker()

    def _notify_tracker(self) -> None:
        if self.on_selected is None:
            return
        try:
            self.on_selected(self.state.selected_uuid or None)
        except Exception:                        # noqa: BLE001 - advisory notification only
            # The selection has already changed; reporting it as failed would be a worse
            # lie than the tracker briefly not knowing whom to protect.
            self.callback_failures += 1

    def _emit(self, event_type: EventType, *, track_uuid: str = "",
              sensor_ns: int = 0, **fields) -> None:
        if self.events is None:
            return
        self.events.emit(event_type, track_uuid=track_uuid, sensor_timestamp_ns=sensor_ns,
                         frame_sequence=self.last_sequence, **fields)

    def state_dict(self) -> Dict[str, Any]:
        return {"selection": self.state.to_dict(),
                "auto": self.auto.to_dict(),
                "rejections": dict(self.rejections),
                "frames_processed": self.frames_processed,
                "callback_failures": self.callback_failures,
                "aliases": self.aliases.to_dict()}


def _project_state(track: Track):
    """``TrackState`` → ``TargetState``, with §34's ``measurement_valid``.

    The two enums are deliberately *not* a single enumeration: §3.6's ``LOST_HOLD`` confusion
    came from someone treating the perception lifecycle and the controller's AutoTrack phase
    as the same variable. A translation function that is obvious and boring is the fix.
    """
    if track.state is TrackState.CONFIRMED_VISIBLE:
        return TargetState.CONFIRMED_VISIBLE, bool(track.measurement_valid)
    if track.state is TrackState.OCCLUDED:
        return TargetState.OCCLUDED, False
    if track.state is TrackState.LOST_REACQUIRABLE:
        return TargetState.LOST, False
    # RETIRED should already have cleared; TENTATIVE cannot be selected. Either way there is
    # no target to publish, and saying so is better than inventing a state.
    return TargetState.NO_TARGET, False
