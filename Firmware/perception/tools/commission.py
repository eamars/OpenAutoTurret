"""§50's ``COMMISSION`` values, turned from invented numbers into reviewed ones.

The document is right that a defaulted threshold is a defect with a delay attached (§3.5: one
universal 0.50 sat permanently above a detector that scored 0.31–0.45, and the station looked
like a tracker that refused to engage). It is less explicit about where the replacement number
comes from. Typing 0.35 because it is between 0.30 and 0.40 is still invention — it just looks
tidy.

So this tool measures the one thing that actually decides those values: **what the detector
separates**. A detection that appears again in the next frame with a matching box is a person;
an isolated blip is not. If the score distributions of those two populations are separated, the
gap is the threshold, and the tool reports the gap. If they overlap, **no number in this file is
evidence**, and it says so rather than printing the midpoint of an overlap and letting somebody
commit it.

Two things it will not do:

* propose from a recording it cannot believe (a mock/offline capture has fictional scores, and
  the operator has to assert ``--real-evidence`` before any number is emitted);
* fill a value it has too few samples to characterise (see ``MIN_*``).
"""
from __future__ import annotations

import argparse
import json
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as _np

from ..config import VisionConfig
from ..errors import PerceptionError
from ..replay import ReplaySource

#: Cross-frame IoU at or above which two boxes count as "the same object seen twice". Deliberately
#: lower than §16's NMS IoU: a person moving between frames deforms and shifts, and a match rate
#: measured with a tight threshold would mislabel real people as transients and push the proposed
#: threshold upward, which is the direction that breaks engagement.
MATCH_IOU = 0.30

#: Same-frame IoU at or above which two rows are treated as one object emitted twice.
DUPLICATE_IOU = 0.50

MIN_FRAMES = 30
MIN_DETECTIONS = 50
MIN_SAMPLES = 20

#: Values a mock capture cannot justify, however tidy they look.
_FICTIONAL_MARKERS = ("mock", "test", "offline", "synthetic", "unit")


def _percentile(values: Sequence[float], percent: float) -> Optional[float]:
    if not len(values):
        return None
    return float(_np.percentile(_np.asarray(values, dtype=float), percent))


def _greedy_match(kept: List[Any], candidates: List[Any], floor: float) -> List[Tuple[int, int]]:
    """Best-IoU greedy 1-to-1 matching, the same shape §44's ground-truth matcher uses.

    Not the tracker's Hungarian assignment: this is a measurement of the *detector*, and using
    the tracker's own association here would make the proposed thresholds a restatement of the
    association code rather than an input to it.
    """
    pairs = []
    for i, left in enumerate(kept):
        for j, right in enumerate(candidates):
            score = left.bbox.iou(right.bbox)
            if score >= floor:
                pairs.append((score, i, j))
    pairs.sort(reverse=True)
    used_left, used_right, taken = set(), set(), []
    for _, i, j in pairs:
        if i in used_left or j in used_right:
            continue
        used_left.add(i)
        used_right.add(j)
        taken.append((i, j))
    return taken


def collect_samples(recording: str, *, match_iou: float = MATCH_IOU) -> Dict[str, Any]:
    """Split every detection in a recording into the evidence it is, without a tracker.

    "Persistent" here means *matched into the adjacent frame*, which is not the same claim as
    "the tracker kept it": §22's low-score pass and §23's occlusion window deliberately carry an
    identity across two or three absent frames. That difference is the point. Measuring
    persistence with the tracker's own machinery would make the proposed thresholds a restatement
    of the tracker's tolerances; frame-to-frame persistence asks the detector the question §22's
    thresholds are actually answered by — can this score be relied on to appear again next frame?
    """
    source = ReplaySource(recording, strict=False)
    observed = [frame for frame in source.frames() if frame.has_detections]
    frames = [list(frame.detection_set.detections) for frame in observed]
    stamps = [frame.sensor_timestamp_ns for frame in observed]
    matched: List[float] = []
    transient: List[float] = []
    persistent: List[float] = []
    duplicate_pairs: List[Tuple[float, float, float]] = []
    all_pairs = 0
    detections_seen = 0
    linked = [set() for _ in frames]

    for index, current in enumerate(frames):
        detections_seen += len(current)
        for a in range(len(current)):
            for b in range(a + 1, len(current)):
                left, right = current[a], current[b]
                if left.class_name != right.class_name:
                    continue
                iou = left.bbox.iou(right.bbox)
                all_pairs += 1
                if iou >= DUPLICATE_IOU:
                    duplicate_pairs.append((iou, left.bbox.containment(right.bbox),
                                            left.bbox.center.distance_to(right.bbox.center)))
        for neighbour, into in ((index - 1, -1), (index + 1, +1)):
            if neighbour < 0 or neighbour >= len(frames):
                continue
            for mine, theirs in _greedy_match(frames[index], frames[neighbour], match_iou):
                linked[index].add(mine)
                # Only the forward link is counted once, so a pair contributes two samples: the
                # score that was good enough to keep the identity is evidence at both ends.
                if into > 0:
                    matched.append(float(frames[index][mine].detector_score))
                    matched.append(float(frames[neighbour][theirs].detector_score))

    interior = range(1, len(frames) - 1)          # boundary frames have no neighbour on one side
    for index in interior:
        for position, detection in enumerate(frames[index]):
            score = float(detection.detector_score)
            (persistent if position in linked[index] else transient).append(score)

    return {"frames": len(frames), "first_stamp_ns": int(stamps[0]) if stamps else 0,
            "detections": detections_seen, "matched": matched, "persistent": persistent,
            "transient": transient, "duplicate_pairs": duplicate_pairs,
            "same_frame_pairs": all_pairs, "model_id": source.manifest.model_id}


def propose_thresholds(recording: str, config: Optional[VisionConfig] = None, *,
                       real_evidence: bool = False) -> Dict[str, Any]:
    """Measured candidates for §22/§23's four scores and §16's three dedup numbers."""
    samples = collect_samples(recording)
    out: Dict[str, Any] = {"recording": recording, "model_id": samples["model_id"],
                           "evidence_grade": "asserted-real", "proposals": {},
                           "objections": [], "enforcements": [], "distributions": {}}
    if config is not None:
        # Show the delta, not just the candidate. A proposal that equals what is already
        # configured is different information from one 0.2 away, and the operator is the only
        # one here who can tell which case they are looking at.
        from dataclasses import asdict

        active = config.active_model
        out["current"] = {"thresholds": asdict(active.thresholds),
                          "dedup": {"nms_iou": getattr(config.dedup, "nms_iou", None),
                                    "containment_ratio": getattr(config.dedup, "containment_ratio",
                                                                 None),
                                    "center_distance_norm": getattr(config.dedup,
                                                                    "center_distance_norm", None)}}

    fictional = any(marker in samples["model_id"].lower() for marker in _FICTIONAL_MARKERS)
    if fictional:
        out["evidence_grade"] = "fictional"
        out["objections"].append(
            f"model_id {samples['model_id']!r} looks synthetic. The scores in this recording were "
            f"typed into a fixture; a threshold commissioned from them would be a restatement of "
            f"the fixture, not a measurement of a detector")
    elif not real_evidence:
        out["evidence_grade"] = "unasserted"
        out["objections"].append(
            "no --real-evidence assertion. This tool cannot tell a real capture from a mock one "
            "it has not seen before, so the operator states it; every number below is then "
            "traceable to that statement rather than to a guess")

    too_short = []
    if samples["frames"] < MIN_FRAMES:
        too_short.append(f"{samples['frames']} frames (< {MIN_FRAMES})")
    if samples["detections"] < MIN_DETECTIONS:
        too_short.append(f"{samples['detections']} detections (< {MIN_DETECTIONS})")
    if too_short:
        out["evidence_grade"] = "insufficient"
        out["objections"].append(
            "not enough samples to characterise a distribution: " + ", ".join(too_short) +
            ". A percentile of eight numbers is the eighth number with extra steps")

    out["distributions"] = {
        "persistent_scores": {p: _percentile(samples["persistent"], p) for p in (5, 25, 50, 95)},
        "transient_scores": {p: _percentile(samples["transient"], p) for p in (5, 50, 95)},
        "matched_scores": {p: _percentile(samples["matched"], p) for p in (5, 10, 50)},
        "duplicate_pair_iou": {p: _percentile([item[0] for item in samples["duplicate_pairs"]], p)
                               for p in (5, 50)},
        "samples": {"persistent": len(samples["persistent"]),
                    "transient": len(samples["transient"]),
                    "matched": len(samples["matched"]),
                    "duplicate_pairs": len(samples["duplicate_pairs"]),
                    "same_frame_pairs": samples["same_frame_pairs"]},
    }

    if out["evidence_grade"] != "asserted-real":
        out["proposals"] = {}
        return out

    persistent = samples["persistent"]
    transient = samples["transient"]
    matched = samples["matched"]

    low_persistent, high_transient = _percentile(persistent, 5), _percentile(transient, 95)
    if len(persistent) < MIN_SAMPLES or len(transient) < MIN_SAMPLES:
        out["objections"].append(
            f"only {len(persistent)} persistent and {len(transient)} transient interior detections "
            f"(need {MIN_SAMPLES} each): new_track and selectable stay unresolved")
    elif low_persistent is None or high_transient is None:
        out["objections"].append("percentiles unavailable; nothing proposed")
    elif high_transient >= low_persistent:
        out["objections"].append(
            f"the populations overlap: transient p95 = {high_transient:.3f} is not below "
            f"persistent p05 = {low_persistent:.3f}. **No threshold separates real detections from "
            f"blips in this recording** — any value written here is a preference, and §50 should "
            f"stay open until a capture separates them (usually a lower-confidence detector or a "
            f"scene with one stationary person and one moving one)")
    else:
        gap = low_persistent - high_transient
        value = round(high_transient + gap / 2.0, 3)
        out["proposals"]["new_track"] = {
            "value": value, "basis": f"midpoint of the gap between transient p95 "
                                     f"({high_transient:.3f}) and persistent p05 "
                                     f"({low_persistent:.3f}); gap = {gap:.3f}",
            "samples": {"persistent": len(persistent), "transient": len(transient)}}
        out["proposals"]["selectable"] = {
            "value": round(min(0.99, max(_percentile(persistent, 25) if persistent else value,
                                          value)), 3),
            "basis": "persistent p25: the score a measurement should reach before a human is "
                     "asked to trust the identity with the turret's aim",
            "samples": {"persistent": len(persistent)}}

    if len(matched) >= MIN_SAMPLES:
        low_association = _percentile(matched, 5)
        confirmed_update = _percentile(matched, 10)
        out["proposals"]["confirmed_update"] = {
            "value": round(confirmed_update, 3),
            "basis": f"p10 of scores that successfully held an identity across frames "
                     f"(n = {len(matched)}): below this an update is more likely to be noise than "
                     f"to be the same person",
            "samples": {"matched": len(matched)}}
        out["proposals"]["low_association"] = {
            "value": round(min(low_association, confirmed_update * 0.8), 3),
            "basis": f"p05 of successfully matched scores (n = {len(matched)}), held below "
                     f"confirmed_update so §22's two passes cannot disagree with each other",
            "samples": {"matched": len(matched)}}
    else:
        out["objections"].append(
            f"only {len(matched)} cross-frame matches (need {MIN_SAMPLES}): confirmed_update and "
            f"low_association stay unresolved")

    pairs = samples["duplicate_pairs"]
    if len(pairs) < MIN_SAMPLES:
        out["objections"].append(
            f"the recording contains {len(pairs)} same-frame duplicate candidates out of "
            f"{samples['same_frame_pairs']} same-frame pairs. §16's numbers cannot be commissioned "
            f"from a capture where the detector never double-fired — either the on-sensor "
            f"postprocess already suppresses duplicates (then take its settings and record that "
            f"decision) or the scene never produced two rows for one person")
    else:
        ious = [item[0] for item in pairs]
        containments = [item[1] for item in pairs]
        distances = [item[2] for item in pairs]
        out["proposals"]["nms_iou"] = {
            "value": round(min(0.75, max(0.30, 0.9 * (_percentile(ious, 5) or 0.5))), 3),
            "basis": f"10% below p05 of observed duplicate-pair IoU "
                     f"(p05 {_percentile(ious, 5):.3f}, n = {len(pairs)}): suppress at or above "
                     f"the tightest overlap actually observed, never above it",
            "samples": {"duplicate_pairs": len(pairs)}}
        out["proposals"]["containment_ratio"] = {
            "value": round(min(0.95, max(0.60, 0.9 * (_percentile(containments, 5) or 0.8))), 3),
            "basis": f"10% below p05 of observed duplicate-pair containment "
                     f"({_percentile(containments, 5):.3f})",
            "samples": {"duplicate_pairs": len(pairs)}}
        out["proposals"]["center_distance_norm"] = {
            "value": round(min(0.15, max(0.01, 1.1 * (_percentile(distances, 95) or 0.05))), 4),
            "basis": f"10% above p95 of observed duplicate-pair centre distance "
                     f"({_percentile(distances, 95):.4f}): a generous radius around the true "
                     f"duplicates, because a nested pair that fails the distance test is silently "
                     f"kept (§16.2)",
            "samples": {"duplicate_pairs": len(pairs)}}

    _enforce_ordering(out)
    return out


def _enforce_ordering(out: Dict[str, Any]) -> None:
    """Force ``low_association ≤ new_track ≤ confirmed_update ≤ selectable`` and log the fights.

    Each proposal is measured independently, so they can come out crossed — a p10 of matches
    landing below a gap midpoint is entirely possible. §22's passes are only meaningful in that
    order, so the order wins; but a silently corrected number is a number nobody will trust, so
    every adjustment says what it moved and by how much.
    """
    order = ("low_association", "new_track", "confirmed_update", "selectable")
    present = [name for name in order if name in out["proposals"]]
    for earlier, later in zip(present, present[1:]):
        low, high = out["proposals"][earlier]["value"], out["proposals"][later]["value"]
        if low <= high:
            continue
        out["proposals"][later]["value"] = low
        out["enforcements"].append(
            f"{later} raised from {high} to {low} to stay above {earlier}: the measurements "
            f"crossed, and §22's passes require this ordering. Look at the two bases before "
            f"committing either number")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m perception.tools.commission",
        description="Propose §50 threshold values from a recording, or refuse to.")
    parser.add_argument("--recording", required=True, metavar="DIR")
    parser.add_argument("--config", default="configs/perception_v1.json")
    parser.add_argument("--real-evidence", action="store_true",
                        help="assert that this recording came from a real detector on a real "
                             "scene. Required: a mock capture's scores are fiction")
    parser.add_argument("--write", default="", metavar="PATH",
                        help="write the proposals as a JSON fragment to paste into a profile")
    args = parser.parse_args(argv)

    try:
        from ..model import resolve_artifact

        config = VisionConfig.from_file(resolve_artifact(args.config))
    except PerceptionError:
        config = None

    result = propose_thresholds(args.recording, config, real_evidence=args.real_evidence)
    print(json.dumps(result, indent=2, sort_keys=True))

    if args.write:
        fragment = {key: value["value"] for key, value in result["proposals"].items()}
        if not fragment:
            print("commission: refusing to write a fragment with no proposals — an empty "
                  "thresholds block silently reopens every §50 question", file=sys.stderr)
            return 3
        from ..protocol.jsonio import atomic_write_text

        atomic_write_text(args.write, json.dumps({
            "_warning": "Proposed by perception.tools.commission from "
                        f"{result['recording']}. Evidence grade: {result['evidence_grade']}. "
                        "Review each basis before pasting it into a profile (§50 still requires "
                        "a human).",
            "thresholds": {name: fragment[name] for name in
                           ("low_association", "new_track", "confirmed_update", "selectable")
                           if name in fragment},
            "dedup": {name: fragment[name] for name in
                      ("nms_iou", "containment_ratio", "center_distance_norm")
                      if name in fragment},
        }, indent=2, sort_keys=True) + "\n")
        print(f"commission: fragment written to {args.write}", file=sys.stderr)

    for objection in result["objections"]:
        print(f"commission: {objection}", file=sys.stderr)
    unresolved = [name for name in ("low_association", "new_track", "confirmed_update",
                                    "selectable", "nms_iou", "containment_ratio",
                                    "center_distance_norm") if name not in result["proposals"]]
    if unresolved:
        print(f"commission: still unresolved — {', '.join(unresolved)}", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":                                    # pragma: no cover
    raise SystemExit(main())
