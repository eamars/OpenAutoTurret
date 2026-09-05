"""§43/§45 — Level-B replay, its metrics, and the determinism comparison.

Three jobs, one module, because they all answer "what did this run do?":

``run_level_b``  — recorded DetectionSets through §43's chain (class filter → §16 dedup →
                   TrackManager → selection). No camera, no model, no wall clock: the recorded
                   sensor timestamps drive §19's lifecycle, so a replay is a *time-accurate*
                   re-run rather than a re-enactment in whatever the host's clock happens to be.

``evaluate``     — §45's tracker and selection metrics, from published TrackSets and
                   observations alone. That restriction is deliberate: an evaluation that
                   reaches into the tracker's private state can pass while the published
                   behaviour is wrong, and the published behaviour is all controld and the HUD
                   ever see. Where §45's true definition needs annotations, the number says it
                   is a proxy in ``notes``, or it comes from ``ground_truth.jsonl`` instead.

``compare_runs`` — the §52 determinism check. It compares *structure*, not identity: UUIDs are
                   random per run by design (§17), so each run's identities are keyed by the
                   frame in which they first appeared, and everything else — labels, states,
                   geometry, selection generations, counters — must match exactly. A comparison
                   that failed on UUIDs would report differences that mean nothing, and the
                   first false alarm is the last run anybody compares.

§46's gates are checked by ``engineering_gates``, phrased as the document phrases them, because
a report that says "1 failure" tells an operator nothing about which behaviour moved.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from ..config import VisionConfig
from ..detection.class_filter import filter_permitted
from ..detection.dedup import DetectionDeduplicator
from ..events import EventLog, EventType
from ..measure import RingStats
from ..protocol.selected_target import SelectedTargetObservation, TargetState
from ..protocol.track_set import Track, TrackSet
from ..selection.protocol import SelectTargetRequest
from ..selection.target_selection_manager import TargetSelectionManager
from ..tracking.track_manager import TrackManager

#: Overlap thresholds for §45's duplicate metric, used only when §16's numbers are still
#: COMMISSION. They are *fallbacks*, not the definition: "two live identities covering one
#: person" has to mean the same thing to the metric and to §25's resolver, or the resolver can
#: be busy merging a pair that §45 reports as no duplication at all — a reading of "clean"
#: that arrives while the operator is watching one person wearing two labels.
_FALLBACK_DUPLICATE = (0.60, 0.85, 0.05)

#: Ground-truth person ↔ identity pairing threshold. A different question from the duplicate
#: metric (which of *my* identities do these recorded boxes belong to), so its own number.
_GT_PAIR_IOU = 0.60

#: Mirrors §25's scale guard: two boxes in the same place with wildly different sizes are a
#: person and something in front of them, not one person measured twice.
_GT_MAX_SCALE_RATIO = 3.0


def _duplicate_thresholds(dedup: Optional[Any]) -> Tuple[float, float, float]:
    """The resolver's own numbers when they exist; the fallbacks when they do not."""
    if dedup is not None and None not in (dedup.nms_iou, dedup.containment_ratio,
                                          dedup.center_distance_norm):
        return (float(dedup.nms_iou), float(dedup.containment_ratio),
                float(dedup.center_distance_norm))
    return _FALLBACK_DUPLICATE


def _scale_ratio(left: Track, right: Track) -> float:
    a, b = left.bbox.area, right.bbox.area
    if a <= 0.0 or b <= 0.0:
        return float("inf")
    return max(a, b) / min(a, b)


def _covers_same_person(left: Track, right: Track,
                        thresholds: Tuple[float, float, float]) -> bool:
    """Would §25 call these two published identities one person? Same numbers, same rule.

    Deliberately not a stricter private threshold: §46's rate gate and §25's merge answer the
    same question, so the two must not disagree about what the question was.
    """
    iou_limit, containment_limit, centre_limit = thresholds
    overlapping = (left.bbox.iou(right.bbox) >= iou_limit
                   or left.bbox.containment(right.bbox) >= containment_limit)
    if not overlapping:
        return False
    if left.bbox.center.distance_to(right.bbox.center) > centre_limit:
        return False
    return _scale_ratio(left, right) <= _GT_MAX_SCALE_RATIO


@dataclass
class ReplayReport:
    """§45's tracker and selection rows. Every field is derived from published output."""

    frames: int = 0
    frames_with_tracks: int = 0
    frames_with_persons: int = 0
    identities_seen: int = 0
    display_labels_seen: int = 0
    max_tracks_in_frame: int = 0
    duplicate_active_identities_max: int = 0
    duplicate_candidate_frames: int = 0
    identity_switches: int = 0
    short_lived_identities: int = 0
    reacquisitions: int = 0
    reacquisition_latency_ms: Optional[Any] = None
    first_selectable_latency_frames: Optional[Any] = None
    wrong_subject_selections: int = 0
    stale_measurements_published: int = 0
    target_stealings: int = 0
    selection_generations: int = 0
    ambiguous_frames: int = 0
    low_score_associations: int = 0
    detections_refused_duplicate: int = 0
    tracks_created: int = 0
    tracks_merged: int = 0
    tracks_evicted: int = 0
    notes: List[str] = field(default_factory=list)

    @property
    def duplicate_candidate_rate(self) -> float:
        """§46: duplicate visible candidates as a share of person-visible frames."""
        if not self.frames_with_persons:
            return 0.0
        return self.duplicate_candidate_frames / float(self.frames_with_persons)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "frames": int(self.frames),
            "frames_with_tracks": int(self.frames_with_tracks),
            "frames_with_persons": int(self.frames_with_persons),
            "identities_seen": int(self.identities_seen),
            "display_labels_seen": int(self.display_labels_seen),
            "max_tracks_in_frame": int(self.max_tracks_in_frame),
            "duplicate_active_identities_max": int(self.duplicate_active_identities_max),
            "duplicate_candidate_frames": int(self.duplicate_candidate_frames),
            "duplicate_candidate_rate": round(self.duplicate_candidate_rate, 5),
            "identity_switches": int(self.identity_switches),
            "short_lived_identities": int(self.short_lived_identities),
            "reacquisitions": int(self.reacquisitions),
            "reacquisition_latency_ms": (self.reacquisition_latency_ms.to_dict()
                                         if self.reacquisition_latency_ms else None),
            "first_selectable_latency_frames": (self.first_selectable_latency_frames.to_dict()
                                                if self.first_selectable_latency_frames
                                                else None),
            "wrong_subject_selections": int(self.wrong_subject_selections),
            "stale_measurements_published": int(self.stale_measurements_published),
            "target_stealings": int(self.target_stealings),
            "selection_generations": int(self.selection_generations),
            "ambiguous_frames": int(self.ambiguous_frames),
            "low_score_associations": int(self.low_score_associations),
            "detections_refused_duplicate": int(self.detections_refused_duplicate),
            "tracks_created": int(self.tracks_created),
            "tracks_merged": int(self.tracks_merged),
            "tracks_evicted": int(self.tracks_evicted),
            "notes": list(self.notes),
        }


@dataclass
class LevelBRun:
    """Everything a replay produced, so a caller never has to re-derive it."""

    report: ReplayReport
    track_sets: List[TrackSet]
    observations: List[SelectedTargetObservation]
    canonical: List[Dict[str, Any]]
    events: List[Dict[str, Any]] = field(default_factory=list)
    source: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {"report": self.report.to_dict(), "frames": len(self.track_sets),
                "source": dict(self.source)}


def run_level_b(frames: Iterable[Any], config: VisionConfig, *,
                session_uuid: str = "replay",
                event_log: Optional[EventLog] = None,
                apply_dedup: bool = True,
                select_request: Optional[SelectTargetRequest] = None,
                select_label: str = "",
                frame_limit: int = 0,
                record: Optional[Any] = None) -> LevelBRun:
    """Feed recorded sets through §43's Level-B chain and publish what it decided.

    ``frames`` may be ``DetectionSet``s or ``ReplayFrame``s; either way the recorded sensor
    timestamp is what drives the tracker, never the host clock.

    ``apply_dedup`` is a switch, not a convenience: a recording stores the normalized set
    *before* §16's dedup — which is the useful thing to store, because it lets a replay ask
    "what would the new thresholds have done?" — so a replay that wants to reproduce live
    behaviour must run that stage again. Turning it off answers a different, also useful,
    question: "what would the tracker do with the model's raw rows".

    ``select_request`` is issued against the first frame in which its UUID exists, and
    ``select_label`` (``"Person #2"``) against the first frame in which that *label* appears.
    Both exist because the operator's click is not reproducible, so a replay has to re-ask the
    question — and the label form is the one that works across runs: §17 mints fresh UUIDs every
    run, so a recorded UUID names an identity that this run never had. Naming the identity the
    way the operator saw it (§27's label) is the only stable reference a replay has.

    ``record`` (a :class:`~perception.replay.recorder.Recorder`) captures the re-run's published
    observations, so a comparison can be re-read later instead of being asserted and lost.
    """
    manager = TrackManager(config, session_uuid=session_uuid, event_log=event_log)
    selector = TargetSelectionManager(config, event_log=event_log)
    deduplicator = (DetectionDeduplicator(config.dedup, anchor_cfg=config.anchor,
                                          event_log=event_log) if apply_dedup else None)
    permitted = tuple(config.active_model.permitted_classes or ())
    stages_skipped: List[str] = []

    track_sets: List[TrackSet] = []
    observations: List[SelectedTargetObservation] = []
    pending = select_request
    wanted_label = select_label.strip().lower()

    for index, item in enumerate(frames):
        if frame_limit and index >= frame_limit:
            break
        dset = getattr(item, "detection_set", item)
        if dset is None:
            continue
        if deduplicator is not None:
            dset = filter_permitted(dset, permitted)
            result = deduplicator.run(dset)
            dset = result.detections
            stages_skipped.extend(result.stages_skipped)
        track_set = manager.update(dset, int(dset.sensor_timestamp_ns))
        if wanted_label and pending is None:
            # Trigger on *selectable*, not on first appearance: a tentative identity already
            # has a display label (§27), and firing a request the moment it appears would ask
            # §37.1 to select something it must refuse — a rejection the operator never made.
            holder = next((track for track in track_set.tracks
                           if track.selectable and track.display_index
                           and f"{track.class_name} #{track.display_index}".lower()
                           == wanted_label), None)
            if holder is not None:
                pending = SelectTargetRequest(
                    request_id=f"replay-label-{holder.display_index}",
                    track_uuid=holder.track_uuid,
                    track_set_sequence_seen_by_ui=int(track_set.track_set_sequence))
                wanted_label = ""
        if pending is not None and track_set.by_uuid(pending.track_uuid) is not None:
            selector.select(pending, track_set, int(dset.sensor_timestamp_ns))
            pending = None
        observation = selector.update(track_set, int(dset.sensor_timestamp_ns))
        track_sets.append(track_set)
        observations.append(observation)
        if record is not None:
            record.record_observations_frame(track_set, observation)

    events = event_log.to_dicts() if event_log is not None else []
    report = evaluate(track_sets, observations, events=events, dedup=config.dedup)
    for reason in dict.fromkeys(stages_skipped):
        report.notes.append(f"dedup stage skipped during replay: {reason}")
    if select_request is not None and pending is not None:
        report.notes.append(
            f"selection request for {select_request.track_uuid[:8]}… was never issued: that "
            f"identity never appeared in the recorded frames")
    if select_label and wanted_label:
        report.notes.append(
            f"selection request for label {select_label!r} was never issued: no recorded "
            f"frame carried that label")
    return LevelBRun(report=report, track_sets=track_sets, observations=observations,
                     canonical=canonical_run(track_sets, observations), events=events)


# --------------------------------------------------------------------------
# Metrics
# --------------------------------------------------------------------------

def _state_name(track: Track) -> str:
    return track.state.name if hasattr(track.state, "name") else str(track.state)


def evaluate(track_sets: Sequence[TrackSet],
             observations: Sequence[SelectedTargetObservation],
             *, events: Optional[Sequence[Dict[str, Any]]] = None,
             dedup: Optional[Any] = None) -> ReplayReport:
    """§45's tracker and selection metrics, from published output only.

    ``dedup`` is the run's §16 threshold block: the duplicate metric reuses those numbers so it
    cannot disagree with the tracker over what "one person, two identities" means.
    """
    report = ReplayReport()
    thresholds = _duplicate_thresholds(dedup)
    if dedup is None or None in (dedup.nms_iou, dedup.containment_ratio,
                                 dedup.center_distance_norm):
        report.notes.append(
            "duplicate metric ran on fallback thresholds "
            f"(iou>={_FALLBACK_DUPLICATE[0]:.2f}, containment>={_FALLBACK_DUPLICATE[1]:.2f}, "
            f"centre<={_FALLBACK_DUPLICATE[2]:.2f}): §16's own numbers are COMMISSION, so the "
            "tracker's resolver was off and this rate is not comparable with a run that used "
            "commissioned thresholds")
    first_seen: Dict[str, int] = {}
    last_seen: Dict[str, int] = {}
    label_owners: Dict[str, set] = {}
    selectable_wait = RingStats("first_selectable_frames", 256)
    selected_uuid = ""

    for index, track_set in enumerate(track_sets):
        observation = observations[index] if index < len(observations) else None
        report.frames += 1
        report.frames_with_tracks += 1 if track_set.tracks else 0
        persons = [track for track in track_set.tracks if track.class_name == "person"]
        report.frames_with_persons += 1 if persons else 0
        report.max_tracks_in_frame = max(report.max_tracks_in_frame, len(track_set.tracks))

        overlapping = 0
        for position, left in enumerate(persons):
            for right in persons[position + 1:]:
                if _covers_same_person(left, right, thresholds):
                    overlapping += 1
        if overlapping:
            report.duplicate_active_identities_max = max(
                report.duplicate_active_identities_max, overlapping)
            report.duplicate_candidate_frames += 1

        present = set()
        for track in track_set.tracks:
            uuid = track.track_uuid
            present.add(uuid)
            first_seen.setdefault(uuid, index)
            last_seen[uuid] = index
            if track.selectable:
                selectable_wait.record(float(index - first_seen[uuid]))
            if track.display_index:
                label_owners.setdefault(
                    f"{track.class_name} #{track.display_index}", set()).add(uuid)

        if observation is None:
            continue
        report.selection_generations = max(report.selection_generations,
                                           int(observation.selection_generation))
        uuid = observation.track_uuid
        if uuid:
            holder = track_set.by_uuid(uuid)
            if holder is None:
                if observation.measurement_valid:
                    # §36/§37: a *measurement* must come from an identity in this set. A
                    # lost-but-held selection is legitimate; a fresh measurement of something
                    # that is not in the set means the selector is describing a different
                    # subject than the tracker published — "wrong-subject selection", §45.
                    report.wrong_subject_selections += 1
            elif observation.measurement_valid and holder.measurement_valid:
                # Both claim a fresh measurement; they must then agree exactly. They disagree
                # only if the selector published geometry it kept from an earlier frame —
                # §34's stale-label race, seen from the side that can actually be measured.
                if _geometry_mismatch(observation, holder):
                    report.stale_measurements_published += 1
            if selected_uuid and uuid != selected_uuid:
                previous = track_set.by_uuid(selected_uuid)
                if previous is not None and getattr(previous, "alias_of", "") != uuid:
                    # The held identity is still published while the selection moved elsewhere,
                    # and the move was not §25's merge handoff. §46: target stealing must be 0.
                    report.target_stealings += 1
            selected_uuid = uuid
        elif selected_uuid and selected_uuid in present:
            # A publish of NO_TARGET while the selected identity is still there is §34's
            # "explicit no target" doing something it should only do when the target is gone.
            report.notes.append(f"frame {index}: NO_TARGET published while "
                                f"{selected_uuid[:8]}… was still present")
            selected_uuid = ""

    for owners in label_owners.values():
        if len(owners) > 1:
            report.identity_switches += len(owners) - 1
    report.identities_seen = len(first_seen)
    report.display_labels_seen = len(label_owners)
    report.short_lived_identities = sum(
        1 for uuid in first_seen if last_seen[uuid] - first_seen[uuid] <= 2)

    if track_sets:
        counters = track_sets[-1].counters.to_dict()
        report.tracks_created = counters.get("tracks_created", 0)
        report.tracks_merged = counters.get("tracks_merged", 0)
        report.tracks_evicted = counters.get("tracks_evicted", 0)
        report.ambiguous_frames = counters.get("ambiguous_frames", 0)
        report.low_score_associations = counters.get("low_score_associations", 0)
        report.detections_refused_duplicate = counters.get("detections_refused_duplicate", 0)
        report.reacquisitions = counters.get("tracks_reacquired", 0)

    latencies = _reacquisition_latency_ms(events or [])
    if latencies.total:
        report.reacquisition_latency_ms = latencies.summary()
    if selectable_wait.total:
        report.first_selectable_latency_frames = selectable_wait.summary()
        report.notes.append(
            "first_selectable_latency is reported in published *frames*, not milliseconds: "
            "the count is exact and its clock is the recording's. Multiply by the recorded "
            "frame interval before comparing with §46's 250 ms gate.")
    report.notes.append(
        "identity_switches counts display labels that were carried by more than one identity "
        "over the run — the published-side proxy for §45's ID switches, since §17's UUIDs are "
        "not in the metric's reach by design.")
    return report


def _geometry_mismatch(observation: SelectedTargetObservation, holder: Track) -> bool:
    if observation.bbox is None:
        return False
    return (abs(observation.bbox.x_min - holder.bbox.x_min) > 1e-9
            or abs(observation.bbox.y_min - holder.bbox.y_min) > 1e-9
            or abs(observation.bbox.x_max - holder.bbox.x_max) > 1e-9
            or abs(observation.bbox.y_max - holder.bbox.y_max) > 1e-9)


def _reacquisition_latency_ms(events: Sequence[Dict[str, Any]]) -> RingStats:
    """§45's reacquisition latency, straight from §42's ``miss_ms`` field on the events."""
    stats = RingStats("reacquisition_miss_ms", 256)
    for event in events:
        # "event" is the key §42's serializer writes. Reading a key that no writer produces
        # would leave this metric permanently empty and permanently passing.
        if event.get("event") != EventType.TRACK_REACQUIRED.value:
            continue
        miss = (event.get("fields") or {}).get("miss_ms")
        if miss is not None:
            stats.record(float(miss))
    return stats


# --------------------------------------------------------------------------
# Determinism: comparing two runs of the same recording
# --------------------------------------------------------------------------

def canonical_run(track_sets: Sequence[TrackSet],
                  observations: Sequence[SelectedTargetObservation]) -> List[Dict[str, Any]]:
    """Published output with the randomness removed, so two runs can be compared.

    Identity keys are assigned by first appearance (§17's UUIDs are random on purpose). Nothing
    else is rounded or relaxed: a re-run that differs in the last published digit *is* a
    different run, and a comparison that hides that is not a determinism check.
    """
    slots: Dict[str, int] = {}
    canonical: List[Dict[str, Any]] = []

    def slot_of(uuid: str) -> int:
        if uuid not in slots:
            slots[uuid] = len(slots)
        return slots[uuid]

    for index, track_set in enumerate(track_sets):
        tracks = []
        for track in track_set.tracks:
            tracks.append({
                "slot": slot_of(track.track_uuid),
                "label_index": int(track.display_index),
                "class": track.class_name,
                "state": _state_name(track),
                "bbox": [round(float(track.bbox.x_min), 9), round(float(track.bbox.y_min), 9),
                         round(float(track.bbox.x_max), 9), round(float(track.bbox.y_max), 9)],
                "anchor": [round(float(track.anchor.x), 9), round(float(track.anchor.y), 9)],
                "selectable": bool(track.selectable),
                "ambiguous": bool(track.ambiguous),
                "measurement_valid": bool(track.measurement_valid),
                "observations": int(track.observations),
                "velocity": [round(float(track.velocity_x), 9),
                             round(float(track.velocity_y), 9)],
            })
        tracks.sort(key=lambda item: item["slot"])
        observation = observations[index] if index < len(observations) else None
        canonical.append({
            "frame_sequence": int(track_set.frame_sequence),
            "sensor_timestamp_ns": int(track_set.sensor_timestamp_ns),
            "tracks": tracks,
            "counters": track_set.counters.to_dict(),
            "selected_slot": (slot_of(observation.track_uuid)
                              if observation is not None and observation.track_uuid else -1),
            "selection_generation": (int(observation.selection_generation)
                                     if observation is not None else 0),
            "target_state": (observation.target_state.name if observation is not None
                             else TargetState.NO_TARGET.name),
            "measurement_valid": (bool(observation.measurement_valid)
                                  if observation is not None else False),
        })
    return canonical


@dataclass
class DeterminismDiff:
    identical: bool = True
    reference_frames: int = 0
    candidate_frames: int = 0
    differences: List[Dict[str, Any]] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {"identical": bool(self.identical),
                "reference_frames": int(self.reference_frames),
                "candidate_frames": int(self.candidate_frames),
                "differences": list(self.differences), "notes": list(self.notes)}


def compare_runs(reference: Sequence[Dict[str, Any]], candidate: Sequence[Dict[str, Any]],
                 *, max_reported: int = 12) -> DeterminismDiff:
    """Frame-by-frame comparison of two ``canonical_run`` outputs."""
    diff = DeterminismDiff(reference_frames=len(reference), candidate_frames=len(candidate))
    if len(reference) != len(candidate):
        diff.identical = False
        diff.notes.append(
            f"length differs: {len(reference)} vs {len(candidate)} frames. Two runs of "
            f"different lengths cannot be compared frame by frame — find out why one stopped "
            f"first.")
    for index in range(min(len(reference), len(candidate))):
        left, right = reference[index], candidate[index]
        if left == right:
            continue
        for path, first, second in _walk(left, right, f"frame[{index}]"):
            diff.identical = False
            if len(diff.differences) < max_reported:
                diff.differences.append({"path": path, "reference": first, "candidate": second})
        if len(diff.differences) >= max_reported:
            diff.notes.append(f"first {max_reported} differences shown")
            break
    return diff


def _walk(left: Any, right: Any, path: str) -> Iterable[Tuple[str, Any, Any]]:
    """Yield ``(path, reference_value, candidate_value)`` for every leaf that differs."""
    if isinstance(left, dict) and isinstance(right, dict):
        for key in sorted(set(left) | set(right)):
            yield from _walk(left.get(key), right.get(key), f"{path}.{key}")
    elif isinstance(left, list) and isinstance(right, list):
        if len(left) != len(right):
            yield f"{path}.length", len(left), len(right)
        for position in range(min(len(left), len(right))):
            yield from _walk(left[position], right[position], f"{path}[{position}]")
    elif left != right:
        yield path, left, right


# --------------------------------------------------------------------------
# §46's gates
# --------------------------------------------------------------------------

DEFAULT_GATES: Dict[str, Any] = {
    "duplicate_candidate_rate_max": 0.01,     # §46: <1% of person-visible frames
    "id_switches_max": 0,
    "target_stealings_max": 0,
    "wrong_subject_selections_max": 0,
    "stale_measurements_published_max": 0,
    "duplicate_active_identities_max": 0,
    "short_lived_identities_max": None,       # scene-dependent; set per scenario
}


def engineering_gates(report: ReplayReport, gates: Optional[Dict[str, Any]] = None) -> List[str]:
    """One line per failed gate: the number, the limit, and which behaviour it protects."""
    limits = dict(DEFAULT_GATES)
    limits.update(gates or {})
    failures: List[str] = []

    def check(name: str, actual: float, limit: Any, meaning: str) -> None:
        if limit is None or actual <= limit:
            return
        failures.append(f"{name}: {actual} exceeds {limit} — {meaning}")

    check("duplicate_candidate_rate", report.duplicate_candidate_rate,
          limits["duplicate_candidate_rate_max"],
          "§46 one-person scene: duplicate visible candidates must stay under 1% of "
          "person-visible frames")
    check("identity_switches", report.identity_switches, limits["id_switches_max"],
          "a display label changing hands is §3.4's recycled-index failure returning")
    check("target_stealings", report.target_stealings, limits["target_stealings_max"],
          "§46 crossing people: target stealing must be 0 in the acceptance set")
    check("wrong_subject_selections", report.wrong_subject_selections,
          limits["wrong_subject_selections_max"],
          "a measurement-bearing observation named an identity that is not in the set")
    check("stale_measurements_published", report.stale_measurements_published,
          limits["stale_measurements_published_max"],
          "the selector published geometry that disagreed with the set it was derived from")
    check("duplicate_active_identities_max", report.duplicate_active_identities_max,
          limits["duplicate_active_identities_max"],
          "§45: two live identities covered one person in the same frame")
    check("short_lived_identities", report.short_lived_identities,
          limits["short_lived_identities_max"],
          "identities that live for a couple of frames are what false positives look like "
          "from the published side")
    return failures


# --------------------------------------------------------------------------
# Ground truth (§44) — the only honest source for these three
# --------------------------------------------------------------------------

def compare_ground_truth(track_sets: Sequence[TrackSet],
                         observations: Sequence[SelectedTargetObservation],
                         truth: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """ID switches, fragmentations and selection mismatch against annotated truth.

    Truth records are ``{"sensor_timestamp_ns": …, "persons": [{"id": "gt-1",
    "bbox": [x0, y0, x1, y1]}], "selected": "gt-1"}`` (§44's annotation set). Frames are joined
    on the exact sensor stamp, because a replay that matched frames by nearest-time could be
    quietly comparing frame 40's output against frame 41's truth.

    Matching is greedy best-IoU with a floor: crude, and stable — a metric whose definition
    moves between runs cannot be an acceptance gate.
    """
    by_time = {int(record.get("sensor_timestamp_ns", 0)): record for record in truth}
    owner_of: Dict[str, str] = {}
    truths_of: Dict[str, set] = {}
    frames = matched = id_switches = selection_mismatch = 0

    for index, track_set in enumerate(track_sets):
        record = by_time.get(int(track_set.sensor_timestamp_ns))
        if record is None:
            continue
        frames += 1
        remaining = list(track_set.tracks)
        pairing: Dict[str, str] = {}
        for person in record.get("persons") or []:
            box = person.get("bbox") or []
            if len(box) != 4:
                continue
            best, best_iou = None, _GT_PAIR_IOU
            for candidate in remaining:
                score = _box_iou(box, candidate.bbox)
                if score > best_iou:
                    best, best_iou = candidate, score
            if best is not None:
                pairing[best.track_uuid] = str(person.get("id", ""))
                remaining.remove(best)
                matched += 1
        for uuid, gt_id in pairing.items():
            previous = owner_of.get(gt_id)
            if previous is not None and previous != uuid:
                id_switches += 1
            owner_of[gt_id] = uuid
            truths_of.setdefault(uuid, set()).add(gt_id)

        wanted = record.get("selected")
        if wanted:
            observation = observations[index] if index < len(observations) else None
            published = pairing.get(observation.track_uuid) if (observation and
                                                                observation.track_uuid) else None
            if published != wanted:
                selection_mismatch += 1

    return {"frames_with_truth": frames, "matched_person_frames": matched,
            "id_switches": id_switches,
            "fragmented_identities": sum(1 for values in truths_of.values() if len(values) > 1),
            "selection_mismatch": selection_mismatch}


def _box_iou(box: Sequence[float], bbox: Any) -> float:
    x_overlap = max(0.0, min(float(box[2]), bbox.x_max) - max(float(box[0]), bbox.x_min))
    y_overlap = max(0.0, min(float(box[3]), bbox.y_max) - max(float(box[1]), bbox.y_min))
    inter = x_overlap * y_overlap
    if inter <= 0.0:
        return 0.0
    union = ((float(box[2]) - float(box[0])) * (float(box[3]) - float(box[1]))
             + (bbox.x_max - bbox.x_min) * (bbox.y_max - bbox.y_min) - inter)
    return inter / union if union > 0.0 else 0.0
