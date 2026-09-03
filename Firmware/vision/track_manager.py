"""TrackManager — multi-object track formation (§8, §10, §22, §58).

v1 ran a *selector* here: it scored every detection and published the single best one,
so the turret could only ever follow what the detector happened to like most. v3
publishes every candidate as a TrackSet (§59) and controld chooses — "controld remains
authoritative for selected target" is a requirement, not a nicety, because the operator
has to be able to point at a specific human being and have that choice survive a
dropout, a mode change, and the detector briefly preferring somebody else.

This lives in visiond because §9's TrackSet carries `state`, `age_frames`,
`visible_frames`, `missing_frames` and `display_index`: the tracks are already formed
by the time they are published, so formation is upstream of the wire. §58's "once per
detector frame" is a camera-rate statement, which is another way of saying the same
thing. controld's side is a view of what arrived (tracks/received_track_set.hpp), not a
second association — two implementations of §10's identity rules is one too many, and
they would disagree first time a target crossed.

Three rules do the real work, and each is here because a plausible alternative fails:

  * Association anchors on the ANCHOR point, with IoU only as a tiebreaker. IoU-first
    looks better on paper and dies on §82's "abrupt bbox-size change": the box halves
    (crouching, occluded legs, one bad frame), overlap with the remembered box
    collapses, the track dies, and a fresh identity is minted one frame later on the
    same person. The operator watches "Person #1" vanish and "Person #3" appear without
    anything moving, and the turret drops a target it was pointing at.
  * Matching is mutual-best, greedy over ascending cost, ties broken by index. Plain
    nearest-neighbour from the detection side lets one detection be claimed by two
    tracks exactly when they cross, which reads as two selector rows pointing at the
    same human being. Greedy is not optimal assignment; it is deterministic, which is
    what §82's crossing test and any §81 replay actually need.
  * State advances in FRAMES, not seconds. §8's thresholds are stated in frames, and a
    timeout in seconds silently changes meaning every time the inference rate changes.
    Silence is handled separately by stale_ms(): a detector that stops publishing sends
    no frames, and nothing may quietly age out tracks in a system whose safety logic
    reads "no target".
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Tuple

from .protocol import K_MAX_TRACKS, BBoxNorm, Track, TrackSet

# Kept identical to §8's suggested values where §8 suggests them; the rest are
# labelled heuristics below. Every one is configurable because §8 says "a configurable
# number", and because the station's camera rate is not a constant (§58).


class TrackState(IntEnum):
    TENTATIVE = 0  # new detection, not yet consistent long enough to select (§8)
    CONFIRMED = 1  # stable candidate, may be selected (§8)
    OCCLUDED = 2   # detection temporarily missing, prediction still valid (§8)
    LOST = 3       # past the allowed missing interval, retained for reacquisition


@dataclass
class TrackManagerConfig:
    confirm_frames: int = 3
    occlude_frames: int = 4
    lost_frames: int = 12
    reacquire_frames: int = 60  # §22: how long a LOST identity is held
    max_jump_norm: float = 0.15  # no association beyond this one-frame anchor jump
    reacquire_gate_norm: float = 0.25  # a LOST identity may be re-found this far out
    iou_bonus: float = 0.35  # weight of IoU as a tiebreaker, never as the gate
    max_tracks: int = K_MAX_TRACKS


@dataclass
class _Slot:
    uuid: Tuple[int, int]
    display_index: int = 1
    class_id: int = 0
    class_name: str = ""
    state: TrackState = TrackState.TENTATIVE
    detector_confidence: float = 0.0
    track_confidence: float = 0.0
    bbox: BBoxNorm = field(default_factory=BBoxNorm)
    anchor_x: float = 0.0
    anchor_y: float = 0.0
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    pred_x: float = 0.0
    pred_y: float = 0.0
    age_frames: int = 0
    visible_frames: int = 0
    missing_frames: int = 0
    last_seen_ns: int = 0
    in_use: bool = True

    def as_track(self) -> Track:
        hi, lo = self.uuid
        return Track(
            track_uuid=(hi, lo),
            display_index=self.display_index,
            class_id=self.class_id,
            class_name=self.class_name,
            state=int(self.state),
            detector_confidence=self.detector_confidence,
            track_confidence=self.track_confidence,
            bbox=self.bbox,
            anchor_x=self.anchor_x,
            anchor_y=self.anchor_y,
            velocity_x_norm_s=self.velocity_x,
            velocity_y_norm_s=self.velocity_y,
            age_frames=self.age_frames,
            visible_frames=self.visible_frames,
            missing_frames=self.missing_frames,
        )


@dataclass
class TrackManagerStats:
    frames: int = 0
    created: int = 0
    retired: int = 0
    matched: int = 0
    lost: int = 0
    reacquired: int = 0


def detections_to_tracks(cap, publish_timestamp_ns: int = 0) -> TrackSet:
    """Adapt a FrameCapture's pixel detections into §9-shaped records.

    The detector speaks pixels (§10.1), the wire speaks normalized (§60), and the
    TrackManager speaks normalized. The conversion happens here, in one place, rather
    than in the manager (which must stay unit-testable without a frame source) or in
    the manager's caller-who-remembers (which is where unit bugs go to live).

    The anchor defaults to the bbox centre, as §10.1 specifies. The resolution is
    carried through because controld turns a normalized anchor back into a pixel to
    build a ray (§17); a TrackSet with width/height unset is unreadable downstream and
    is treated as "no target" rather than guessed at.
    """
    from .protocol import class_name_for  # local: keeps protocol -> manager acyclic

    out = TrackSet(
        frame_sequence=cap.frame_sequence,
        sensor_timestamp_ns=cap.sensor_timestamp_ns,
        publish_timestamp_ns=publish_timestamp_ns,
        width=cap.width,
        height=cap.height,
    )
    for d in cap.detections:
        x_min, y_min, x_max, y_max = d.as_normalized(cap.width, cap.height)
        cx, cy = d.centre_px
        out.tracks.append(
            Track(
                class_id=d.class_id,
                class_name=class_name_for(d.class_id),
                detector_confidence=d.confidence,
                track_confidence=d.confidence,
                bbox=BBoxNorm(x_min, y_min, x_max, y_max),
                anchor_x=cx / max(cap.width, 1),
                anchor_y=cy / max(cap.height, 1),
            )
        )
    return out


def _overlap(a: BBoxNorm, b: BBoxNorm) -> float:
    ix = min(a.x_max, b.x_max) - max(a.x_min, b.x_min)
    iy = min(a.y_max, b.y_max) - max(a.y_min, b.y_min)
    if ix <= 0.0 or iy <= 0.0:
        return 0.0
    inter = ix * iy
    area_a = max(0.0, a.x_max - a.x_min) * max(0.0, a.y_max - a.y_min)
    area_b = max(0.0, b.x_max - b.x_min) * max(0.0, b.y_max - b.y_min)
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


class TrackManager:
    """Turns each detector frame's raw detections into persistent identities."""

    def __init__(self, config: Optional[TrackManagerConfig] = None,
                 session_nonce: int = 0) -> None:
        self.cfg = config or TrackManagerConfig()
        # §10: `track_uuid` is 128-bit and session-unique. The high half is a nonce
        # that distinguishes this process's identities from a previous run's, and the
        # low half is a counter — not a cryptographic UUID, and deliberately not the
        # detector's own per-frame id, which resets whenever the detector restarts and
        # would hand the same number to a different human being.
        self._session_hi = session_nonce & ((1 << 64) - 1)
        self._uuid_counter = 0
        self._slots: List[_Slot] = []
        self.stats = TrackManagerStats()
        self._last_sensor_ns = 0
        self._last_publish_ns = 0
        self._last_receive_ns = 0

    # --- the per-frame step (§58: once per detector frame) ------------------
    def update(self, detections: TrackSet, now_ns: int) -> List[Track]:
        """Advance every track by one frame and return the live tracks.

        Detections arrive as a TrackSet-shaped bag of raw records (the same type is
        used for the outgoing set: it is the §9 schema, not a claim that they are
        tracks yet). Publishing goes through build_track_set(), which is where the
        frame's own stamps get attached.
        """
        self.stats.frames += 1
        self._last_receive_ns = now_ns
        self._last_publish_ns = detections.publish_timestamp_ns
        dt = self._dt_seconds(detections.sensor_timestamp_ns)
        self._last_sensor_ns = detections.sensor_timestamp_ns
        dets = list(detections.tracks)[: self.cfg.max_tracks]

        # 1. Predict each live track to this frame's capture time, so a fast target
        #    is matched where it should be rather than where it was.
        for sl in self._slots:
            sl.pred_x = sl.anchor_x + sl.velocity_x * dt
            sl.pred_y = sl.anchor_y + sl.velocity_y * dt

        # 2. Costs, then greedy mutual-best assignment. Costs are integers so the
        #    sort has no NaN ordering trap and replays are bit-identical (§81).
        cells: List[Tuple[int, int, int]] = []
        for d, det in enumerate(dets):
            for i, sl in enumerate(self._slots):
                gate = (self.cfg.reacquire_gate_norm if sl.state == TrackState.LOST
                        else self.cfg.max_jump_norm)
                dist = math.hypot(det.anchor_x - sl.pred_x,
                                  det.anchor_y - sl.pred_y)
                if dist > gate:
                    continue  # never teleport a track across the frame
                if det.class_id != sl.class_id:
                    continue  # identities do not cross classes
                iou = _overlap(sl.bbox, det.bbox)
                cost = dist - self.cfg.iou_bonus * iou * gate
                cells.append((int(cost * 1000), i, d))
        cells.sort()
        slot_to_det: Dict[int, int] = {}
        det_to_slot: Dict[int, int] = {}
        for _, i, d in cells:
            if i in slot_to_det or d in det_to_slot:
                continue  # already claimed: this is what "mutual-best" means
            slot_to_det[i] = d
            det_to_slot[d] = i

        # 3. Matched: identity preserved (§10 UUID stability).
        for i, d in slot_to_det.items():
            self._adopt(self._slots[i], dets[d], now_ns, dt)

        # 4. Unmatched: the §8 state ladder.
        for i, sl in enumerate(self._slots):
            if i in slot_to_det:
                continue
            sl.missing_frames += 1
            sl.age_frames += 1
            if sl.state == TrackState.LOST:
                if sl.missing_frames >= self.cfg.reacquire_frames + self.cfg.lost_frames:
                    self._retire(sl)  # §22: only now is the label free for reuse
                continue
            if sl.missing_frames >= self.cfg.lost_frames:
                sl.state = TrackState.LOST
                self.stats.lost += 1
            elif sl.missing_frames >= self.cfg.occlude_frames:
                sl.state = TrackState.OCCLUDED
            elif sl.state == TrackState.TENTATIVE and sl.missing_frames >= 2:
                # A tentative track already blinking is not a target being born, it is
                # noise. It was never selectable (§8), so keeping it costs a slot and
                # confuses the overlay with candidates that cannot be picked.
                self._retire(sl)

        # 5. Unmatched detections: new tentative tracks.
        for d, det in enumerate(dets):
            if d not in det_to_slot:
                self._create(det, now_ns, dt)

        self._slots = [s for s in self._slots if s.in_use]
        return self.tracks()

    def build_track_set(self, frame_sequence: int, sensor_timestamp_ns: int,
                        publish_timestamp_ns: int, width: int,
                        height: int) -> TrackSet:
        """The publishable TrackSet (§9).

        The frame metadata is passed in rather than remembered, because only the
        caller knows what it just captured: `frame_sequence` and the resolution belong
        to the frame, and a manager that cached them would publish last frame's
        sequence against this frame's tracks the first time a capture is dropped.
        """
        out = TrackSet(
            frame_sequence=frame_sequence,
            sensor_timestamp_ns=sensor_timestamp_ns,
            publish_timestamp_ns=publish_timestamp_ns,
            width=width,
            height=height,
        )
        out.tracks = self.tracks()[: self.cfg.max_tracks]
        return out

    # --- queries (used by the publisher, the overlay and the tests) ---------
    def tracks(self) -> List[Track]:
        return [s.as_track() for s in self._slots]

    def track_count(self) -> int:
        return len(self._slots)

    def find(self, uuid: Tuple[int, int]) -> Optional[_Slot]:
        for s in self._slots:
            if s.uuid == tuple(uuid):
                return s
        return None

    def exists(self, uuid: Tuple[int, int]) -> bool:
        return self.find(uuid) is not None

    def is_selectable(self, uuid: Tuple[int, int]) -> bool:
        """§8: only CONFIRMED may be selected.

        An occluded track must not become selectable *by selecting it*. The operator
        picking something the detector is not currently confirming would hand
        AUTO_TRACK a target it cannot see.
        """
        s = self.find(uuid)
        return s is not None and s.state == TrackState.CONFIRMED

    def is_trackable(self, uuid: Tuple[int, int]) -> bool:
        """The SELECTED track may ride out a short dropout (§20 coasting needs a
        subject that still exists), which a fresh selection may not."""
        s = self.find(uuid)
        return s is not None and s.state in (TrackState.CONFIRMED, TrackState.OCCLUDED)

    # --- §61 latency ------------------------------------------------------
    def stale_ms(self, now_ns: int) -> float:
        if self._last_receive_ns <= 0 or now_ns < self._last_receive_ns:
            return -1.0
        return (now_ns - self._last_receive_ns) / 1e6

    def sensor_age_ms(self, now_ns: int) -> float:
        if self._last_sensor_ns <= 0 or now_ns < self._last_sensor_ns:
            return -1.0
        return (now_ns - self._last_sensor_ns) / 1e6

    def publish_to_receive_ms(self) -> float:
        if self._last_publish_ns <= 0 or self._last_receive_ns < self._last_publish_ns:
            return -1.0
        return (self._last_receive_ns - self._last_publish_ns) / 1e6

    # --- internals --------------------------------------------------------
    def _dt_seconds(self, sensor_ns: int) -> float:
        if self._last_sensor_ns <= 0 or sensor_ns <= self._last_sensor_ns:
            return 1.0 / 30.0
        dt = (sensor_ns - self._last_sensor_ns) / 1e9
        # A detector that stalls for four seconds must not extrapolate a track across
        # the frame on the next set, and a jittery clock must not produce a negative
        # dt (which would run the predictor backwards).
        return min(dt, 0.25)

    def _create(self, det: Track, now_ns: int, dt: float) -> None:
        if len(self._slots) >= self.cfg.max_tracks:
            # At capacity the tentative with the least evidence gives way — never a
            # confirmed track, which an operator may have selected and the turret may
            # be pointing at. Sacrificing a selected track for a detection that has
            # not proved itself would look like the turret misbehaving, with the
            # explanation somewhere the operator is not looking.
            victims = [s for s in self._slots if s.state == TrackState.TENTATIVE]
            if not victims:
                return  # every slot is confirmed: drop this detection
            victim = max(victims, key=lambda s: (s.visible_frames, -s.uuid[1]))
            self._retire(victim)
        self._uuid_counter += 1
        slot = _Slot(uuid=(self._session_hi, self._uuid_counter))
        slot.class_id = det.class_id
        slot.class_name = det.class_name
        slot.state = TrackState.TENTATIVE
        slot.display_index = self._alloc_display_index(det.class_id)
        self.stats.created += 1
        self._adopt(slot, det, now_ns, dt)
        self._slots.append(slot)

    def _adopt(self, sl: _Slot, det: Track, now_ns: int, dt: float) -> None:
        if sl.visible_frames > 0 and sl.last_seen_ns > 0:
            vx = (det.anchor_x - sl.anchor_x) / max(dt, 1e-3)
            vy = (det.anchor_y - sl.anchor_y) / max(dt, 1e-3)
            # "No more than a max-jump per 50 ms", which is the same physical statement
            # as the association gate in normalized image units per second. A heuristic,
            # labelled as one: the point is that a single wild sample cannot arm the
            # predictor and turn a detection artefact into predicted motion.
            cap = self.cfg.max_jump_norm / 0.05
            sl.velocity_x = max(-cap, min(cap, 0.5 * sl.velocity_x + 0.5 * vx))
            sl.velocity_y = max(-cap, min(cap, 0.5 * sl.velocity_y + 0.5 * vy))
        sl.bbox = det.bbox
        sl.anchor_x = det.anchor_x
        sl.anchor_y = det.anchor_y
        sl.detector_confidence = det.detector_confidence
        # Filtered, not raw: §19 derates motion by confidence, so one lucky frame must
        # not be able to raise the speed ceiling.
        sl.track_confidence = (det.track_confidence if sl.visible_frames == 0 else
                              0.7 * sl.track_confidence + 0.3 * det.track_confidence)
        if det.class_name:
            sl.class_name = det.class_name
        sl.age_frames += 1
        sl.visible_frames += 1
        was_missing = sl.missing_frames > 0
        sl.missing_frames = 0
        sl.last_seen_ns = now_ns
        if sl.state == TrackState.LOST and was_missing:
            self.stats.reacquired += 1  # §21: re-identification keeps the identity
        if sl.state in (TrackState.OCCLUDED, TrackState.LOST):
            sl.state = TrackState.CONFIRMED
        elif (sl.state == TrackState.TENTATIVE
              and sl.visible_frames >= self.cfg.confirm_frames):
            sl.state = TrackState.CONFIRMED  # §8: enough consistent frames to pick
        self.stats.matched += 1

    def _retire(self, sl: _Slot) -> None:
        if not sl.in_use:
            return
        sl.in_use = False
        self.stats.retired += 1
        # The display index becomes reusable exactly here, and not before: §10 says a
        # label may be reused only after a track is fully retired, which is the reason
        # selection commands carry the UUID.

    def _alloc_display_index(self, class_id: int) -> int:
        """Lowest free label per class (§10), so the operator sees Person #1 and #2
        before #3."""
        taken = {s.display_index for s in self._slots
                 if s.in_use and s.class_id == class_id}
        for cand in range(1, K_MAX_TRACKS + 1):
            if cand not in taken:
                return cand
        return K_MAX_TRACKS
