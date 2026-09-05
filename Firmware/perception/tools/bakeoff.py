"""Vision-6's deliverable: "an evidence-based selected model profile", as a command.

A bake-off is easy to fake and hard to run. The faking happens when the comparison script grows
its own tracker call and quietly diverges from the daemon's, or when two profiles are compared
over two *different* captures and the difference in the numbers is attributed to the model
rather than to the person who walked slightly differently. So:

* every metric here comes from ``run_level_b`` — the same Level-B chain the replay command uses,
  over the same recorded frames, with the profile's own thresholds;
* a profile is **excluded**, not ranked, when its recording is not comparable, and the exclusion
  is printed (§45's rows are only comparable if the input was);
* the ranking is lexicographic over §46's gates first and prints which field decided. There is no
  weighted score: a weighted score lets whoever set the weights pick the winner in advance.

This tool does not capture. Capturing needs the sensor — ``--plan`` prints the exact capture
commands, ``--recordings`` compares what they produced.
"""
from __future__ import annotations

import argparse
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ..config import VisionConfig
from ..errors import PerceptionError
from ..replay import ReplaySource, engineering_gates, run_level_b

# Ordered most-important-first. Each entry is (metric, ascending-is-better, why-this-order).
RANKING: Tuple[Tuple[str, bool, str], ...] = (
    ("gate_failures", True, "§46 gates first: a profile that fails acceptance cannot win on speed"),
    ("duplicate_active_identities_max", True,
     "§45: two live identities over one person is the failure this whole subsystem exists to kill"),
    ("short_lived_identities", True, "lifecycle churn: identities that die within §23's window"),
    ("identities_seen", True,
     "identity economy over the same frames. This sits ABOVE latency on purpose: measured "
     "during this file's own tests, loosening §22's bands made a profile selectable one frame "
     "sooner — by promoting a detector blip into a target. Latency rewards whatever reaches "
     "selectability, including a fabrication, so it may only break a tie that identity counts "
     "could not"),
    ("first_selectable_latency_p95_frames", True,
     "how long the operator waits for something selectable. p95, not the mean: §40's convention "
     "is that the tail is the experience"),
)

# A capture shorter than this cannot say anything about §23's lifecycle windows, let alone §46.
MIN_COMPARABLE_FRAMES = 30


def profile_metrics(config: VisionConfig, profile: str, recording: str, *,
                    select_label: str = "",
                    strict_source: bool = True) -> Dict[str, Any]:
    """Run one profile over one recording and return the §45 rows worth comparing.

    Returns a row even for a profile that cannot be ranked, with ``excluded`` naming why — a
    caller that filters the exclusions away would be comparing a survivor set it chose.
    """
    row: Dict[str, Any] = {"profile": profile, "recording": recording, "excluded": "",
                           "objections": [], "metrics": {}}
    if profile not in config.models:
        row["excluded"] = f"profile {profile!r} is not in this configuration"
        return row
    try:
        source = ReplaySource(recording, strict=strict_source)
    except PerceptionError as exc:
        row["excluded"] = f"recording cannot be opened: {exc}"
        return row

    config.profile = profile
    declared = config.models[profile].model_id
    if source.manifest.model_id and declared and source.manifest.model_id != declared:
        # Not fatal: a recording made by model A *is* the right input for comparing profile B's
        # thresholds. But it is not a detector comparison, and a table that let the reader
        # believe otherwise would be selling a Vision-6 result Vision-6 has not produced.
        row["objections"].append(
            f"recorded with {source.manifest.model_id!r}, profile names {declared!r}: this row "
            f"compares tracking/selection settings, not detectors")

    try:
        run = run_level_b(source.detection_sets(), config, select_label=select_label)
    except PerceptionError as exc:
        row["excluded"] = f"level-B run refused: {exc}"
        return row

    report = run.report
    # No limits argument on purpose: these are §46's published limits, the same ones `visiond
    # --gates` applies. A bake-off whose comparison script quietly relaxes the limits is how a
    # losing model gets selected.
    gates = engineering_gates(report)
    metrics = {
        "frames": int(report.frames),
        "frames_with_persons": int(report.frames_with_persons),
        "identities_seen": int(report.identities_seen),
        "duplicate_active_identities_max": int(report.duplicate_active_identities_max),
        "duplicate_candidate_rate": round(report.duplicate_candidate_rate, 4),
        "short_lived_identities": int(report.short_lived_identities),
        "identity_switches": int(report.identity_switches),
        "reacquisitions": int(report.reacquisitions),
        "selection_generations": int(report.selection_generations),
        "wrong_subject_selections": int(report.wrong_subject_selections),
        "target_stealings": int(report.target_stealings),
        "gate_failures": len(gates),
    }
    # §40/§45's latency rows are percentile summaries, not scalars. Flattening them to one
    # number for the ranking and printing the whole table in the row is the honest split: the
    # comparison needs one statistic to order on, the reader needs to see the distribution it
    # came from, and dropping the table would hide a profile whose median is fine and whose p99
    # is two seconds.
    latencies: Dict[str, Any] = {}
    for name in ("first_selectable_latency_frames", "reacquisition_latency_ms"):
        summary = getattr(report, name, None)
        if summary is None:
            latencies[name] = None
            continue
        if hasattr(summary, "to_dict"):
            latencies[name] = summary.to_dict()
            latencies[f"{name}_p95"] = float(summary.p95)
        else:                                          # a scalar, from an older report
            latencies[name] = {"count": 1, "p50": float(summary), "p95": float(summary),
                               "p99": float(summary), "max": float(summary)}
            latencies[f"{name}_p95"] = float(summary)
    row["latencies"] = latencies
    metrics["first_selectable_latency_p95_frames"] = latencies.get(
        "first_selectable_latency_frames_p95")
    metrics["reacquisition_latency_ms_p95"] = latencies.get("reacquisition_latency_ms_p95")
    row["metrics"] = metrics
    row["gates"] = gates
    row["recorded_with"] = source.manifest.model_id
    row["notes"] = list(report.notes) + list(source.summary.notes)

    if metrics["frames"] < MIN_COMPARABLE_FRAMES:
        row["excluded"] = (
            f"only {metrics['frames']} frames. §23's lifecycle windows are hundreds of "
            f"milliseconds and §46 counts over the whole scene: below "
            f"{MIN_COMPARABLE_FRAMES} frames the differences between profiles are the "
            f"recording's, not the models'")
    return row


def compare_recordings(config: VisionConfig, recordings: Dict[str, str], *,
                       select_label: str = "") -> Dict[str, Any]:
    """Compare ``{profile: recording_dir}`` and recommend, or refuse to.

    Two recordings of different lengths are not comparable, so the frame count is checked across
    the whole set: the tool would rather say "unfair comparison" than rank on a difference the
    capture caused.
    """
    rows = [profile_metrics(config, profile, recording, select_label=select_label)
            for profile, recording in sorted(recordings.items())]
    ranked = [row for row in rows if not row["excluded"]]
    result: Dict[str, Any] = {"profiles": rows, "recommendation": None, "objections": []}

    for row in rows:
        if row["excluded"]:
            result["objections"].append(f"{row['profile']}: excluded — {row['excluded']}")
        for objection in row.get("objections", []):
            result["objections"].append(f"{row['profile']}: {objection}")

    counts = {row["metrics"]["frames"] for row in ranked}
    if len(counts) > 1:
        result["objections"].append(
            f"the comparable captures hold different frame counts {sorted(counts)}. Different "
            f"amounts of scene are not a model comparison — re-capture with the same --max-frames")
        ranked = []

    uncommissioned = sorted(
        {row["profile"] for row in ranked
         if any("COMMISSION" in note for note in row.get("notes", []))})
    if uncommissioned:
        result["objections"].append(
            f"{', '.join(uncommissioned)} ran with uncommissioned §16 thresholds, so dedup "
            f"declined and the duplicate rows measure nothing (§50)")
        ranked = []

    if not ranked:
        result["objections"].append("no profile could be ranked; refusing to recommend")
        return result

    ordered = sorted(ranked, key=lambda row: _sort_key(row["metrics"]))
    best, runner_up = ordered[0], ordered[1] if len(ordered) > 1 else None
    decided_by, reason = _deciding_field(best, runner_up)
    result["ranking"] = [row["profile"] for row in ordered]
    result["recommendation"] = {
        "profile": best["profile"],
        "decided_by": decided_by,
        "why": reason,
        "unopposed": runner_up is None,
    }
    return result


def _sort_key(metrics: Dict[str, Any]) -> Tuple[float, ...]:
    def value(name: str, ascending: bool) -> float:
        raw = metrics.get(name)
        # A missing measurement sorts *last*, never first: an unmeasured latency is not a fast
        # one, and treating None as 0.0 would rank silence above everything actually measured.
        if raw is None:
            return float("inf")
        number = float(raw)
        return number if ascending else -number

    return tuple(value(name, ascending) for name, ascending, _ in RANKING)


def _deciding_field(best: Dict[str, Any], runner_up: Optional[Dict[str, Any]]) -> Tuple[str, str]:
    """Name the metric that actually separated the top two, in the order the ranking used."""
    if runner_up is None:
        return "sole-candidate", "only one profile produced a comparable row; nothing was compared"
    for name, _, meaning in RANKING:
        left, right = best["metrics"].get(name), runner_up["metrics"].get(name)
        if left != right:
            return name, (f"{name} = {left} vs {right} for {runner_up['profile']}. {meaning}")
    return "tie", (f"{best['profile']} and {runner_up['profile']} are equal on every ranked "
                   f"metric; the choice must come from something else — §40's latency budget, "
                   f"the licence, or a scenario these frames do not contain")


def plan_captures(config: VisionConfig, profiles: Sequence[str], *, frames: int = 300,
                  out_dir: str = "/tmp/bakeoff", extra: str = "") -> List[str]:
    """The capture recipe, as literal commands.

    Generated rather than written out because the two things that most often invalidate a
    bake-off are a different ``--max-frames`` between captures and a forgotten
    ``--record-dataset``: pinning both here removes the operator's ability to make those two
    mistakes, and the printed commands stay editable for the ones they can still make.
    """
    lines = [f"mkdir -p {out_dir}"]
    for profile in profiles:
        lines.append(
            f"python -m perception.visiond --config configs/perception_v1.json "
            f"--profile {profile} --max-frames {frames} "
            f"--record-dataset {out_dir}/{profile} --publish-dir {out_dir}/{profile}/published "
            f"--report {out_dir}/{profile}.json{(' ' + extra) if extra else ''}")
    lines.append(
        "python -m perception.tools.bakeoff --recordings "
        + ",".join(f"{profile}={out_dir}/{profile}" for profile in profiles))
    return lines


def parse_recordings(spec: str) -> Dict[str, str]:
    """``name=/path,name2=/path2`` → dict. The verbose form is the point: a bake-off where the
    operator has to remember which directory was which has already lost the comparison."""
    recordings: Dict[str, str] = {}
    for item in spec.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise PerceptionError(
                f"--recordings entry {item!r} has no '='. Give profile=path pairs, e.g. "
                f"person_detect_available=/tmp/bakeoff/person_detect_available")
        name, _, path = item.partition("=")
        recordings[name.strip()] = path.strip()
    if not recordings:
        raise PerceptionError("--recordings was empty")
    return recordings


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m perception.tools.bakeoff",
        description="Compare model profiles on recorded frames (Vision-6).")
    parser.add_argument("--config", default="configs/perception_v1.json")
    parser.add_argument("--recordings", default="", metavar="PROFILE=DIR[,…]",
                        help="one recording per profile, compared and ranked")
    parser.add_argument("--plan", action="store_true",
                        help="print the capture commands instead of comparing anything")
    parser.add_argument("--profile", action="append", default=[],
                        help="with --plan: which profiles to capture (repeatable)")
    parser.add_argument("--frames", type=int, default=300,
                        help="with --plan: frames per capture (must be identical)")
    parser.add_argument("--out", default="/tmp/bakeoff", help="with --plan: where to record")
    parser.add_argument("--select-label", default="", metavar="LABEL",
                        help="drive a selection in every profile the same way")
    parser.add_argument("--report", default="", metavar="PATH", help="write the comparison here")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    config = VisionConfig.from_file(_resolve(args.config))

    if args.plan:
        profiles = args.profile or sorted(
            name for name in config.models if name != "offline_replay")
        for line in plan_captures(config, profiles, frames=args.frames, out_dir=args.out):
            print(line)
        return 0
    if not args.recordings:
        print("bakeoff: give --recordings PROFILE=DIR[,…] or --plan", file=sys.stderr)
        return 2

    result = compare_recordings(config, parse_recordings(args.recordings),
                                select_label=args.select_label)
    from ..protocol.jsonio import atomic_write_text, dumps as json_dumps

    payload = json_dumps(result, indent=2)
    if args.report:
        atomic_write_text(args.report, payload + "\n")
    if not args.quiet:
        print(payload)
    recommendation = result.get("recommendation")
    if recommendation:
        print(f"bakeoff: recommends {recommendation['profile']} on "
              f"{recommendation['decided_by']} — {recommendation['why']}", file=sys.stderr)
        if recommendation["unopposed"]:
            return 3
        return 0
    for objection in result["objections"]:
        print(f"bakeoff: {objection}", file=sys.stderr)
    print("bakeoff: no recommendation (§50/§45: no comparable evidence)", file=sys.stderr)
    return 3


def _resolve(path: str) -> str:
    from ..model import resolve_artifact

    return resolve_artifact(path)


if __name__ == "__main__":                                    # pragma: no cover
    raise SystemExit(main())
