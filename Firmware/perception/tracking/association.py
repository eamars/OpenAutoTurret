"""Association: the hard gates, the cost, and an optimal assignment (§21).

Association is where an identity is either preserved or destroyed, so the ordering here
is deliberate and is the opposite of the usual tutorial implementation: **gate first,
cost second, assign third.**

The gates are physics. A person cannot cross the image at three body-heights a second; a
12-pixel-tall box cannot triple in one frame; a detection of class ``dog`` is not a low
probability ``person``, it is a different class. Such a pair is removed from the matrix
entirely, because a *cost* can be outvoted by the other terms and a gate cannot. The
retired path did this backwards — nearest-anchor distance with an IoU tiebreak — and the
result was that a single wild detection could win on one term and teleport a track.

The cost is §21's weighted sum, and a term is included only when the data for it exists
(an appearance scorer is injected, pose terms appear only when both sides have keypoints).
The weights sum-normalize the cost so ``quality = 1 - cost`` stays comparable across
scenes and across model profiles, which is what §37 asks ``association_quality`` to mean.

The assignment is optimal (Hungarian / shortest augmenting path), not greedy. The
retired code was greedy and its comment argued that greedy is at least *deterministic*.
It was neither: greedy on an ascending-cost list is deterministic but wrong at crossings,
and this solver is both — deterministic for a given input matrix because the tie-break is
the cell order, and minimal because that is the theorem. At §26's capacity of 16 tracks
against a handful of detections the cost is microseconds; "greedy is cheaper" was never
true at this size, only assumed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from ..config import TrackingConfig
from ..detection.types import Detection, PointNorm, is_finite
from ..tracking.track import Track, TrackState

#: Gated cells get this cost instead of being deleted from the matrix, and are discarded
#: again after the solve. A forbidden cell has to carry a *finite* number: the solver
#: subtracts row/column potentials from every cell, and ``inf`` arithmetic turns a
#: solvable matrix into a silent no-match on every row.
FORBIDDEN_COST = 1.0e6

#: §21's "incompatible pose" gate. Structural, not commissioned: skeletons that agree on
#: nothing (OKS below this) are two different bodies, and a person's skeleton cannot
#: reconfigure into that between two 60 ms frames. Only consulted when both sides carry
#: keypoints, so a box-only profile never pays for it.
POSE_MIN_OKS = 0.10

#: A gate distance for a track with no velocity estimate still has to be finite.
_MIN_GATE_DISTANCE = 0.02


def _predicted_anchor(track: Track, dt_s: float,
                      shift: Optional[PointNorm] = None) -> PointNorm:
    """Where this track expects its target, including any camera-motion shift (§23).

    The compensation is applied to the PREDICTION rather than to the detections: the
    detections are the measurements, and editing a measurement to account for the camera
    would put modelled motion into the data the controller is allowed to aim with. Adding
    it to the expectation keeps the measured anchor exactly as the sensor produced it (§36).
    """
    predicted = track.predicted_anchor(dt_s)
    if shift is None:
        return predicted
    return PointNorm(predicted.x + shift.x, predicted.y + shift.y)


@dataclass
class AssociationCost:
    """One matrix cell, with its terms kept separate for §41's trace."""

    cost: float = 0.0
    quality: float = 0.0                     # 1 - normalized cost, §37's association_quality
    distance: float = 0.0                    # predicted-anchor distance, normalized units
    iou: float = 0.0
    terms: Dict[str, float] = field(default_factory=dict)
    weight_sum: float = 0.0                  # for §41: which terms were live


@dataclass
class Assignment:
    """The solver's output, plus everything §41 needs to explain it."""

    matches: List[Tuple[int, int, float]] = field(default_factory=list)  # (track, det, quality)
    unmatched_tracks: List[int] = field(default_factory=list)
    unmatched_detections: List[int] = field(default_factory=list)
    rejected: List[Tuple[int, int, str]] = field(default_factory=list)  # gate reasons
    cells: Dict[Tuple[int, int], AssociationCost] = field(default_factory=dict)

    def quality_for(self, track_index: int) -> float:
        for t, _d, quality in self.matches:
            if t == track_index:
                return quality
        return 0.0


@dataclass
class _Cell:
    track_index: int
    det_index: int
    cost: float
    payload: AssociationCost


# --------------------------------------------------------------------------
# Gates (§21)
# --------------------------------------------------------------------------

def gate_distance(track: Track, dt_s: float, cfg: TrackingConfig, *,
                  reacquiring: bool = False) -> float:
    """How far this track may legitimately move before it stops being this track.

    Derived from a speed, not from a fixed pixel radius, because §19's lifecycle is
    time-based: the same 150 ms gap at 17 inference results per second is one frame and at
    30 is nearly five, and a fixed per-frame jump budget therefore changes meaning every
    time the detector's cadence changes. A LOST identity gets ``reacquire_margin`` more
    room — §20 says reacquisition is allowed to reach further than ordinary tracking —
    but that room is still multiplied by elapsed time, so it widens with the miss instead
    of becoming unlimited.
    """
    dt = max(float(dt_s), 0.0)
    # The gate is a *permission*, and the thing being permitted is the commissioned speed
    # limit §21 names. Deriving it from the track's own observed speed instead — which an
    # earlier revision of this function did — is a trap: a new or stationary track then has a
    # budget of a couple of hundredths of the frame, ordinary detector box-jitter exceeds it,
    # and every frame mints a fresh identity beside the old one. `max_speed_norm_s` would be
    # dead configuration, and the identity churn would be blamed on the detector.
    permitted = float(cfg.gates.max_speed_norm_s) * dt
    observed = track.speed_norm_s * dt          # a genuinely fast target gets room for itself
    base = max(permitted, observed) + _MIN_GATE_DISTANCE
    if reacquiring:
        base *= max(1.0, cfg.gates.reacquire_margin)
    return base


def pose_compatible(a: Sequence, b: Sequence) -> bool:
    """§21's "incompatible pose" gate. True when either side has no keypoints."""
    if not a or not b:
        return True
    pairs = min(len(a), len(b))
    if pairs == 0:
        return True
    xs = [p.x for p in list(a)[:pairs] + list(b)[:pairs]]
    ys = [p.y for p in list(a)[:pairs] + list(b)[:pairs]]
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    diagonal = math.hypot(span_x, span_y)
    if diagonal <= 1e-6:
        return True
    total = 0.0
    counted = 0
    for ka, kb in zip(list(a)[:pairs], list(b)[:pairs]):
        if ka.score <= 0.0 or kb.score <= 0.0:
            continue
        d2 = (ka.x - kb.x) ** 2 + (ka.y - kb.y) ** 2
        sigma2 = (2.0 * POSE_MIN_OKS * diagonal) ** 2
        total += math.exp(-d2 / max(sigma2, 1e-12)) * min(ka.score, kb.score)
        counted += 1
    if counted == 0:
        return True
    return (total / counted) >= POSE_MIN_OKS


def hard_gate(track: Track, detection: Detection, dt_s: float, cfg: TrackingConfig, *,
              sensor_timestamp_ns: int, reacquiring: bool = False,
              shift: Optional[PointNorm] = None) -> Optional[str]:
    """Return the reason this pair may not be associated, or ``None``.

    Each reason is a different physical impossibility, and they stay separate strings
    because §41's answer to "why did this identity stop receiving measurements?" is
    supposed to name one of them.
    """
    if detection.class_id != track.class_id:
        return "class_mismatch"
    if not detection.is_valid():
        return "malformed_detection"
    predicted = _predicted_anchor(track, dt_s, shift)
    distance = predicted.distance_to(detection.measured_anchor)
    if distance > gate_distance(track, dt_s, cfg, reacquiring=reacquiring):
        return "impossible_displacement"
    a_area, b_area = track.bbox.area, detection.bbox.area
    if a_area <= 0.0 or b_area <= 0.0:
        return "degenerate_box"
    ratio = max(a_area, b_area) / min(a_area, b_area)
    if ratio > cfg.gates.max_scale_ratio:
        return "impossible_scale"
    if track.bbox.aspect_change(detection.bbox) > cfg.gates.max_aspect_change:
        return "impossible_aspect"
    if not pose_compatible(track.keypoints, detection.keypoints):
        return "incompatible_pose"
    miss_ms = track.miss_age_ms(sensor_timestamp_ns)
    if miss_ms >= 0.0 and miss_ms > cfg.lost.retain_ms:
        return "stale_beyond_ttl"
    return None


# --------------------------------------------------------------------------
# Cost (§21)
# --------------------------------------------------------------------------

def association_cost(track: Track, detection: Detection, dt_s: float,
                     cfg: TrackingConfig, *, reacquiring: bool = False,
                     appearance_distance: Optional[float] = None,
                     shift: Optional[PointNorm] = None) -> AssociationCost:
    """§21's ``C = w_motion*d + w_iou*(1-IoU) + w_scale*s + w_shape*a + w_app*d_app + w_pose*d_pose``.

    Every term is normalized to roughly [0,1] before weighting, because an unweighted sum
    of a pixel distance, a unitless IoU and a scale ratio is a sum of three different
    units and its minimum is whoever happens to have the largest numbers. The result is
    divided by the sum of the live weights, so ``quality = 1 - cost`` means the same thing
    whether or not appearance and pose are enabled.
    """
    weights = cfg.weights
    predicted = _predicted_anchor(track, dt_s, shift)
    distance = predicted.distance_to(detection.measured_anchor)
    gate = gate_distance(track, dt_s, cfg, reacquiring=reacquiring)
    iou = track.bbox.iou(detection.bbox)

    terms: Dict[str, float] = {}
    weight_sum = 0.0

    motion_term = min(1.0, distance / gate) if gate > 0.0 else 1.0
    if weights.motion:
        terms["motion"] = weights.motion * motion_term
        weight_sum += weights.motion

    if weights.iou:
        terms["iou"] = weights.iou * (1.0 - iou)
        weight_sum += weights.iou

    scale_term = track.bbox.scale_change(detection.bbox)
    if weights.scale:
        terms["scale"] = weights.scale * min(1.0, scale_term)
        weight_sum += weights.scale

    shape_term = track.bbox.aspect_change(detection.bbox)
    if weights.shape:
        terms["shape"] = weights.shape * min(1.0, shape_term)
        weight_sum += weights.shape

    # §24: appearance is injected, never assumed. ``None`` means "no descriptor for one
    # side", which is the common case (a person who just walked in has no history) and
    # must not be scored as a maximum-distance mismatch.
    if weights.appearance and appearance_distance is not None:
        terms["appearance"] = weights.appearance * min(1.0, appearance_distance)
        weight_sum += weights.appearance

    if weights.pose and track.keypoints and detection.keypoints:
        pose_term = min(1.0, _pose_distance(track.keypoints, detection.keypoints))
        terms["pose"] = weights.pose * pose_term
        weight_sum += weights.pose

    total = sum(terms.values())
    normalized = total / weight_sum if weight_sum > 0.0 else min(1.0, total)
    return AssociationCost(
        cost=normalized,
        quality=max(0.0, min(1.0, 1.0 - normalized)),
        distance=distance,
        iou=iou,
        terms=terms,
        weight_sum=weight_sum,
    )


def _pose_distance(a: Sequence, b: Sequence) -> float:
    """Normalized mean keypoint displacement (§21's ``pose_distance``)."""
    pairs = min(len(a), len(b))
    if pairs == 0:
        return 1.0
    total = 0.0
    counted = 0
    for ka, kb in zip(list(a)[:pairs], list(b)[:pairs]):
        if ka.score <= 0.0 or kb.score <= 0.0:
            continue
        total += math.hypot(ka.x - kb.x, ka.y - kb.y)
        counted += 1
    return total / counted if counted else 1.0


# --------------------------------------------------------------------------
# Optimal assignment (§21: "Use Hungarian/min-cost assignment")
# --------------------------------------------------------------------------

def solve_assignment(cost: Sequence[Sequence[float]]) -> List[Tuple[int, int]]:
    """Minimum-cost rectangular assignment (shortest augmenting path, O(n^2*m)).

    Pure Python because scipy is not on the station and a tracker that imports
    ``scipy.optimize`` at 17 Hz on a Pi 5 is a tracker that adds a dependency for a
    matrix no wider than ``max_tracks``. Rows need not be the shorter side: the matrix is
    transposed internally and the result mapped back, so "sixteen tracks, three
    detections" and "three tracks, sixteen detections" both work.

    Raises on ragged or empty input rather than returning a partial answer — a matrix
    with a missing row is a bug in the caller, and an assignment that silently ignored it
    would drop every track in that row.
    """
    if not cost or not cost[0]:
        return []
    width = len(cost[0])
    for row in cost:
        if len(row) != width:
            raise ValueError("cost matrix is ragged")
        for value in row:
            if not is_finite(value):
                raise ValueError("cost matrix contains a non-finite value")
    transposed = len(cost) > width
    matrix = _transpose(cost) if transposed else [list(row) for row in cost]
    # Both dimensions come from the matrix in hand, never from `width`: after a transpose
    # the column count IS the original row count, and carrying the old width through here
    # silently truncates every matrix that needed the transpose — the solver then answers
    # a different question than the one it was asked and still returns "a perfect match".
    rows, cols = len(matrix), len(matrix[0])
    if rows > cols:  # after transposing this cannot happen, but the invariant is cheap
        raise ValueError("assignment matrix has more rows than columns")

    # 1-based potentials, following the classic shortest-augmenting-path formulation.
    u = [0.0] * (rows + 1)
    v = [0.0] * (cols + 1)
    partner = [0] * (cols + 1)   # partner[j] = row matched to column j (0 = free)
    way = [0] * (cols + 1)
    big = float("inf")

    for i in range(1, rows + 1):
        partner[0] = i
        j0 = 0
        minv = [big] * (cols + 1)
        used = [False] * (cols + 1)
        while True:
            used[j0] = True
            i0 = partner[j0]
            delta = big
            j1 = -1
            row = matrix[i0 - 1]
            for j in range(1, cols + 1):
                if used[j]:
                    continue
                current = row[j - 1] - u[i0] - v[j]
                if current < minv[j]:
                    minv[j] = current
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            for j in range(cols + 1):
                if used[j]:
                    u[partner[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if partner[j0] == 0:
                break
        while True:
            j1 = way[j0]
            partner[j0] = partner[j1]
            j0 = j1
            if j0 == 0:
                break

    result: List[Tuple[int, int]] = [(-1, -1)] * rows
    for j in range(1, cols + 1):
        if partner[j] != 0:
            result[partner[j] - 1] = (partner[j] - 1, j - 1)
    pairs = [pair for pair in result if pair[0] >= 0]
    return [(c, r) for r, c in pairs] if transposed else pairs


def _transpose(cost: Sequence[Sequence[float]]) -> List[List[float]]:
    return [list(column) for column in zip(*cost)]


def assign(tracks: Sequence[Track], detections: Sequence[Detection], dt_s: float,
           cfg: TrackingConfig, *, sensor_timestamp_ns: int,
           include_states: Sequence[TrackState],
           reacquire_states: Sequence[TrackState] = (TrackState.LOST_REACQUIRABLE,),
           appearance_scorer: Optional[Callable[[Track, Detection], Optional[float]]] = None,
           shift: Optional[PointNorm] = None) -> Assignment:
    """Gate, cost and optimally assign one detection group to one track group.

    ``include_states`` is what makes §20's passes two calls instead of two trackers:
    pass 1 offers ``CONFIRMED_VISIBLE`` and ``OCCLUDED`` tracks to high-score detections,
    pass 2 offers only the tracks pass 1 could not place to low-score detections. The
    tracks themselves are the same objects; only the candidate set and the score floor
    differ, which is exactly what "a low-score detection may rescue but may not create"
    means operationally.
    """
    out = Assignment()
    track_pool = [i for i, t in enumerate(tracks) if t.state in tuple(include_states)]
    if not track_pool or not detections:
        out.unmatched_tracks = list(track_pool)
        out.unmatched_detections = list(range(len(detections)))
        return out

    matrix: List[List[float]] = []
    payloads: Dict[Tuple[int, int], AssociationCost] = {}
    row_to_track: List[int] = []          # matrix row position -> real track index
    for ti in track_pool:
        row_to_track.append(ti)
        track = tracks[ti]
        reacquiring = track.state in tuple(reacquire_states)
        row: List[float] = []
        for di, detection in enumerate(detections):
            reason = hard_gate(track, detection, dt_s, cfg,
                               sensor_timestamp_ns=sensor_timestamp_ns,
                               reacquiring=reacquiring, shift=shift)
            if reason is not None:
                out.rejected.append((ti, di, reason))
                row.append(FORBIDDEN_COST)
                continue
            appearance_distance = None
            if cfg.weights.appearance and appearance_scorer is not None:
                appearance_distance = appearance_scorer(track, detection)
            payload = association_cost(track, detection, dt_s, cfg,
                                       reacquiring=reacquiring,
                                       appearance_distance=appearance_distance,
                                       shift=shift)
            payloads[(ti, di)] = payload
            row.append(payload.cost)
        matrix.append(row)

    out.cells.update(payloads)

    matched: Dict[int, int] = {}
    for row, di in solve_assignment(matrix):
        ti = row_to_track[row]
        if (ti, di) not in payloads:
            continue  # a gated cell: the solver had no legal alternative and said so
        matched[ti] = di

    matched_dets = set(matched.values())
    for ti, di in sorted(matched.items()):
        out.matches.append((ti, di, payloads[(ti, di)].quality))
    out.unmatched_tracks = [ti for ti in track_pool if ti not in matched]
    out.unmatched_detections = [di for di in range(len(detections))
                                if di not in matched_dets]
    # Determinism is a correctness property here, not a nicety: §52's replay tests and
    # §82's crossing test both claim a specific outcome, which is only reproducible if the
    # match list has one canonical order.
    out.matches.sort()
    return out
