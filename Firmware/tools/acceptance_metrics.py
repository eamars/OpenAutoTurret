#!/usr/bin/env python3
"""§55 acceptance-metrics report, extracted from what the daemon already logs.

P13 asks for a metrics report with five families (control timing, CAN, homing,
tracking, limits) at the end of a supervised run. The temptation is to sit and
eyeball a 100 MB log at 2 a.m. and write down numbers that "look right". This
tool exists so those numbers come from the log text instead: it parses the lines
controld actually writes and prints the report, and — this is the important
half — for anything the log does NOT carry it prints `NOT MEASURED` with the
source that would be needed. A metrics report with invented entries is worse
than an incomplete one, because nobody re-tests a number that is already in the
table.

    python3 -m tools.acceptance_metrics --log controld.log --can can-stats.txt
    python3 -m tools.acceptance_metrics --log run1.log run2.log --json
    python3 -m tools.acceptance_metrics --log ... --out docs/acceptance_P13.md

Sources, and what each can and cannot answer:

  controld log      control timing (the 1 Hz `loop:` line, §6.3), homing events
                    (the per-event `motion ... msg=` line, with q and tq),
                    faults, derates, SLOW CYCLE, `vision:` health (needs visiond)
  --telemetry FILE  JSON objects, one per line (tools/station_ipc.py telemetry,
                    or a /api/state capture): track_state history, LOS error,
                    vision age — whatever the operator recorded
  --can FILE        `turret-can stats` output saved to a file: error frames,
                    dropped/invalid counts, per-motor feedback age

Known gap, stated rather than hidden: §55 wants "LOS tracking error (mean/max)"
and "loss→brake time" per axis. controld does not log a LOS error figure today
(§34 notes it is not in telemetry either), so this tool reports NOT MEASURED and
names the change that would fix it. Do not fill that cell by hand.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from collections import Counter
from typing import Dict, List, Optional

# Log patterns, one place, so a format change is a single edit and the report
# says loudly when a pattern stops matching (`unmatched` accounting below).
TS_RE = re.compile(r"^\[([\d-]+ [\d:]+)\.\d+\] \[controld\] \[(\w+)\] (.*)$")
LOOP_RE = re.compile(
    r"loop: target=(\d+) Hz p50=([\d.]+) p95=([\d.]+) p99=([\d.]+) "
    r"worst=([\d.]+) ms \(n=(\d+)\)")
SLOW_RE = re.compile(r"SLOW CYCLE ([\d.]+) ms \(phase=([^,]+), action=([^)]+)\)")
MOTION_RE = re.compile(
    r"motion t=([\d.]+)ms ax=(pitch|yaw) q=([-+\d.]+) v=([-+\d.]+) "
    r"a=([-+\d.]+) j=([-+\d.]+) tq=([-+\d.]+) cmd=([-+\d.]+) msg=(.*)$")
SEC_RE = re.compile(
    r"t=([\d.]+)s phase=(\w+) q_pitch=([-+\d.]+) q_yaw=([-+\d.]+) rad "
    r"temp_pitch=([\d.]+) temp_yaw=([\d.]+)")
VISION_RE = re.compile(
    r"vision: (\d+) frames \((\d+) dropped, seq (\d+), age ([\d.]+) ms\) "
    r"\| tracking=(\w+) state=(\S+) conf=([\d.]+)")
FAULT_RE = re.compile(r"control fault: (.+)$")
SUPERVISOR_RE = re.compile(r"supervisor: (.+?) reason='(.*?)' "
                           r"(?:overrun_us=(\d+) )?misses=(\d+)")
HOMED_RE = re.compile(r"homed \+ at ready pose")
HOMING_START_RE = re.compile(r"homing started")
CAM_RE = re.compile(r"^(error frames|dropped|invalid|feedback age|r\*|"
                    r"tx|rx)\D*([\d.]+)", re.IGNORECASE)

NOT_MEASURED = "NOT MEASURED"
# Provenance for a row that has none. "—" would read as an oversight; this says
# the input was absent, which is a fact about the capture, not the tool.
NOTHING_READ = "nothing to read in this capture"


class Report:
    """One metric row. `source` is what makes the report auditable."""

    def __init__(self, family: str, name: str, value: str, source: str) -> None:
        self.family = family
        self.name = name
        self.value = value
        self.source = source

    def missing(self) -> bool:
        return self.value.startswith(NOT_MEASURED)


def _parse_telemetry(paths: List[str]) -> List[dict]:
    out: List[dict] = []
    for p in paths:
        with open(p, errors="replace") as f:
            for line in f:
                line = line.strip()
                if not line.startswith("{"):
                    continue
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(obj, dict):
                    out.append(obj)
    return out


def _num(series: List[float]) -> str:
    if not series:
        return NOT_MEASURED
    return (f"mean {sum(series)/len(series):.3f} / max {max(series):.3f} "
            f"/ min {min(series):.3f} (n={len(series)})")


def analyse(log_paths: List[str], telemetry_paths: List[str],
            can_paths: List[str], config_path: str = "") -> Dict[str, object]:
    loops: List[tuple] = []
    slows: List[tuple] = []
    motion: List[dict] = []
    seconds: List[dict] = []
    visions: List[dict] = []
    faults: Counter = Counter()
    supervisors: List[dict] = []
    derated: List[str] = []
    homing_start: Optional[float] = None
    homed_at: Optional[float] = None
    t_wall0: Optional[str] = None
    t_wall1: Optional[str] = None
    lines = 0

    for path in log_paths:
        with open(path, errors="replace") as f:
            for raw in f:
                lines += 1
                m = TS_RE.match(raw.rstrip("\n"))
                text = m.group(3) if m else raw.strip()
                if m:
                    t_wall0 = t_wall0 or m.group(1)
                    t_wall1 = m.group(1)
                if (hm := LOOP_RE.search(text)):
                    loops.append(tuple(float(x) for x in hm.groups()))
                    continue
                if (sm := SLOW_RE.search(text)):
                    slows.append((float(sm.group(1)), sm.group(2),
                                  sm.group(3)))
                    continue
                if (mm := MOTION_RE.search(text)):
                    motion.append({"t_ms": float(mm.group(1)),
                                   "ax": mm.group(2),
                                   "q": float(mm.group(3)),
                                   "tq": float(mm.group(7)),
                                   "msg": mm.group(9).strip()})
                    continue
                if (sm := SEC_RE.search(text)):
                    seconds.append({"t_s": float(sm.group(1)),
                                    "phase": sm.group(2),
                                    "q_pitch": float(sm.group(3)),
                                    "q_yaw": float(sm.group(4)),
                                    "temp_pitch": float(sm.group(5)),
                                    "temp_yaw": float(sm.group(6))})
                    continue
                if (vm := VISION_RE.search(text)):
                    visions.append({"frames": int(vm.group(1)),
                                    "dropped": int(vm.group(2)),
                                    "seq": int(vm.group(3)),
                                    "age_ms": float(vm.group(4)),
                                    "tracking": vm.group(5),
                                    "state": vm.group(6),
                                    "conf": float(vm.group(7))})
                    continue
                if (fm := FAULT_RE.search(text)):
                    faults[fm.group(1).strip()] += 1
                    continue
                if (um := SUPERVISOR_RE.search(text)):
                    supervisors.append({"to": um.group(1),
                                        "reason": um.group(2),
                                        "misses": int(um.group(4))})
                    continue
                if (dm := re.search(r"payload check complete: (\S+) "
                                    r"\(derated=(true|false)\)", text)):
                    derated.append(f"{dm.group(1)} (derated={dm.group(2)})")
                    continue
                if "derate" in text.lower():
                    derated.append(text.strip()[:120])
                    continue
                if HOMING_START_RE.search(text) and homing_start is None:
                    homing_start = _epoch_of(m)
                if HOMED_RE.search(text) and homed_at is None:
                    homed_at = _epoch_of(m)

    telem = _parse_telemetry(telemetry_paths)
    can: Dict[str, str] = {}
    for path in can_paths:
        with open(path, errors="replace") as f:
            for line in f:
                cm = CAM_RE.match(line.strip())
                if cm:
                    can[cm.group(1).strip().lower()] = cm.group(2)

    rows: List[Report] = []

    def add(family: str, name: str, value: str, source: str) -> None:
        rows.append(Report(family, name, value, source))

    # -- 1. control timing -------------------------------------------------
    if loops:
        last = loops[-1]
        worst = max(l[4] for l in loops)
        target = int(last[0])
        period = 1000.0 / target
        # "Deadline miss" needs a definition, not a vibe: a cycle longer than the
        # nominal period. The sim loop paces itself to the period, so p50 ~= the
        # period and this count is small by construction; on hardware the number
        # that matters is worst-vs-period and SLOW CYCLE.
        misses = sum(1 for l in loops if l[4] > period * 1.10)
        add("control timing", "target loop rate", f"{target} Hz", "loop: line")
        add("control timing", "latest p50/p95/p99",
            f"{last[1]:.3f} / {last[2]:.3f} / {last[3]:.3f} ms",
            f"last loop: line (statistics ring, n={int(last[5])})")
        add("control timing", "worst cycle ever reported", f"{worst:.3f} ms",
            f"max over {len(loops)} loop: lines")
        add("control timing", "cycles over 110% of period",
            f"{misses} of {len(loops)} reports", "derived: worst > "
            f"{period*1.1:.2f} ms")
        add("control timing", "SLOW CYCLE warnings",
            (f"{len(slows)} (max {max(s[0] for s in slows):.3f} ms, "
             f"phases {sorted({s[1] for s in slows})})" if slows else "0"),
            "SLOW CYCLE lines")
    else:
        add("control timing", "loop statistics", NOT_MEASURED +
            " — no `loop:` line in the log (was controld running with the 1 Hz "
            "stats line? spdlog level must be info)", NOTHING_READ)

    # -- 2. homing ---------------------------------------------------------
    endpoints: Dict[str, List[dict]] = {"pitch": [], "yaw": []}
    for ev in motion:
        if "homed" in ev["msg"]:
            endpoints[ev["ax"]].append(ev)
    if homing_start is not None and homed_at is not None:
        add("homing", "duration (homing started -> at ready)",
            f"{homed_at - homing_start:.2f} s", "log timestamps")
    else:
        add("homing", "duration (homing started -> at ready)", NOT_MEASURED +
            " — need both `homing started` and `homed + at ready pose` in the "
            "captured log (a log rotated mid-homing loses one)", NOTHING_READ)
    for ax in ("pitch", "yaw"):
        evs = endpoints[ax]
        # Two sources, in order of authority. The endpoint events are the marks
        # the homing controller itself accepted, but the daemon logs only
        # `endpoint A homed; starting endpoint B` (measured: 1 event per axis per
        # run), so B's q is not in the text. The swept range of q over the homing
        # moves covers both ends and is what the log can actually answer; it is
        # bounded by event sampling, so it is stated as a range, not a mark.
        home_words = ("approach", "settle", "home", "backoff")
        swept = [e["q"] for e in motion if e["ax"] == ax
                 and any(w in e["msg"].lower() for w in home_words)]
        if len(evs) >= 2:
            travel = abs(evs[0]["q"] - evs[1]["q"])
            add("homing", f"measured travel ({ax}) from endpoint marks",
                f"{travel:.4f} rad = "
                f"{abs(math.degrees(travel)):.2f} deg",
                f"endpoint msg q: {evs[0]['q']:+.5f} -> {evs[1]['q']:+.5f}")
        elif swept:
            travel = max(swept) - min(swept)
            add("homing", f"measured travel ({ax}) swept range",
                f"{travel:.4f} rad = {abs(math.degrees(travel)):.2f} deg",
                f"q range {min(swept):+.5f}..{max(swept):+.5f} over "
                f"{len(swept)} homing motion events (endpoint marks logged "
                f"for {len(evs)} of 2 ends)")
        else:
            add("homing", f"measured travel ({ax})", NOT_MEASURED +
                " — no endpoint events and no homing motion lines for this "
                "axis in the captured log", NOTHING_READ)
        # Homing effort, taken from the events whose own msg= says they are
        # homing moves. Time-windowing instead is not possible from the log
        # alone: motion t= is monotonic ms and the timestamp is wall clock, and
        # pretending they line up would produce a peak from the wrong phase.
        in_home = [e for e in motion if e["ax"] == ax
                   and any(w in e["msg"].lower() for w in home_words)]
        add("homing", f"peak effort (abs) on homing moves ({ax})",
            (f"{max(abs(e['tq']) for e in in_home):.3f} Nm" if in_home
             else NOT_MEASURED + f" — no homing motion lines for {ax}"),
            f"motion lines, msg= approach/settle/home/backoff, tq= field "
            f"({len(in_home)} events)")
    add("homing", "endpoint repeatability", NOT_MEASURED +
        " — needs >= 2 HOMING RUNS in the log (--log a.log --log b.log ...): "
        "one run gives one endpoint, and a spread of one number is not a "
        "spread", NOTHING_READ)

    # -- 3. tracking -------------------------------------------------------
    if visions:
        last = visions[-1]
        add("tracking", "vision frames / dropped",
            f"{last['frames']} / {last['dropped']} (last report)",
            "`vision:` 1 Hz line")
        add("tracking", "measurement age",
            _num([v["age_ms"] for v in visions]) + " ms",
            f"{len(visions)} `vision:` lines")
        states = Counter(v["state"] for v in visions)
        add("tracking", "tracker states observed",
            ", ".join(f"{k}={n}" for k, n in states.most_common()),
            "`vision:` state= field")
        add("tracking", "max dropped between reports",
            str(max((visions[i]["dropped"] - visions[i - 1]["dropped"]
                     for i in range(1, len(visions))), default=0)),
            "derived from the dropped counter")
    else:
        add("tracking", "vision health block", NOT_MEASURED +
            " — no `vision:` line: visiond was not running (or was not "
            "connected) for this log", NOTHING_READ)

    track_states = [t.get("track_state") for t in telem if t.get("track_state")]
    if track_states:
        trans = [(a, b) for a, b in zip(track_states, track_states[1:])
                 if a != b]
        add("tracking", "telemetry track_state transitions",
            f"{len(trans)} ({', '.join(f'{a}->{b}' for a, b in trans[:8])}"
            f"{'...' if len(trans) > 8 else ''})",
            f"{len(track_states)} captured telemetry samples")
    # One row name whether measured or not: a P13 report must have the SAME
    # shape run to run, or "the yaw row disappeared" gets read as "yaw is fine".
    los = [t.get("los_error_px") for t in telem
           if isinstance(t.get("los_error_px"), (int, float))]
    if los:
        add("tracking", "LOS tracking error (mean/max)",
            _num([float(x) for x in los]) + " px",
            "telemetry capture los_error_px")
    else:
        add("tracking", "LOS tracking error (mean/max)", NOT_MEASURED +
            " — controld logs no LOS error figure and it is not in the "
            "telemetry schema (§34 says the same for the CAN block); capturing "
            "telemetry cannot invent it. Add it to Snapshot + web_server.hpp "
            "before P13 if the acceptance run must report it.",
            NOTHING_READ)

    # -- 4. limits / safety ------------------------------------------------
    add("limits", "control faults",
        (", ".join(f"{k} x{v}" for k, v in faults.most_common())
         if faults else "0"),
        "`control fault:` lines")
    add("limits", "supervisor transitions",
        (", ".join(f"{s['to']}({s['reason'][:28]})" for s in supervisors[:8])
         if supervisors else "0"),
        "`supervisor:` lines")
    add("limits", "derate / payload-check outcomes",
        ("; ".join(derated[:6]) if derated else
         NOT_MEASURED + " — no derate or `payload check complete` line "
         "(no payload check ran)"),
        "payload check / derate lines")
    soft = [s for s in supervisors if "soft" in s["reason"].lower()
            or "limit" in s["reason"].lower()]
    add("limits", "soft-limit excursions",
        f"{len(soft)}" + (f" ({', '.join(s['reason'][:30] for s in soft[:3])})"
                         if soft else "") + " — acceptance requires 0",
        "supervisor reasons mentioning soft/limit")

    # -- 5. CAN ------------------------------------------------------------
    if can:
        for k, v in can.items():
            add("CAN", k, v, "turret-can stats file")
    else:
        add("CAN", "feedback age / dropped / error frames", NOT_MEASURED +
            " — save `turret-can stats` output to a file and pass --can FILE "
            "(the daemon log does not carry per-motor CAN counters; §34)", NOTHING_READ)

    ages = [t.get("feedback_age_ms") for t in telem
            if isinstance(t.get("feedback_age_ms"), (int, float))]
    if ages:
        add("CAN", "feedback age (from telemetry capture)",
            _num([float(a) for a in ages]) + " ms", "telemetry capture")

    meta = {"log_lines": lines, "loop_reports": len(loops),
            "motion_events": len(motion), "second_lines": len(seconds),
            "vision_lines": len(visions), "telemetry_samples": len(telem),
            "wall_span": f"{t_wall0} .. {t_wall1}" if t_wall0 else "unknown",
            "temps": (_num([s["temp_pitch"] for s in seconds]) + " / " +
                      _num([s["temp_yaw"] for s in seconds])
                      if seconds else NOT_MEASURED)}
    return {"rows": rows, "meta": meta}


def _epoch_of(ts_match) -> Optional[float]:
    if not ts_match:
        return None
    import datetime
    try:
        return datetime.datetime.strptime(ts_match.group(1),
                                          "%Y-%m-%d %H:%M:%S").timestamp()
    except (ValueError, IndexError):
        return None


def render_md(result: Dict[str, object]) -> str:
    rows: List[Report] = result["rows"]
    meta = result["meta"]
    out = ["# §55 acceptance metrics", "",
           f"- log lines read: **{meta['log_lines']}** "
           f"(loop reports {meta['loop_reports']}, motion events "
           f"{meta['motion_events']}, vision lines {meta['vision_lines']}, "
           f"telemetry samples {meta['telemetry_samples']})",
           f"- wall clock covered: {meta['wall_span']}",
           f"- temperatures pitch / yaw: {meta['temps']}",
           "",
           "Every row names the line it came from. `NOT MEASURED` rows are "
           "gaps in the evidence, not zeros: fill them by capturing the named "
           "source, never by typing a number.", ""]
    families: List[str] = []
    for r in rows:
        if r.family not in families:
            families.append(r.family)
    for fam in families:
        out += [f"## {fam}", "", "| metric | value | source |", "|---|---|---|"]
        for r in rows:
            if r.family != fam:
                continue
            mark = " ⚠" if r.missing() else ""
            # A stray pipe inside a cell silently restructures the table, which
            # in a metrics report means a number lands in the wrong column.
            esc = (lambda t: str(t).replace("|", "\\|"))
            out.append(f"| {esc(r.name)} | {esc(r.value)}{mark} "
                       f"| {esc(r.source)} |")
        out.append("")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", nargs="+", default=[],
                    help="controld log file(s); pass every file of one run")
    ap.add_argument("--telemetry", nargs="*", default=[],
                    help="JSON-lines telemetry capture")
    ap.add_argument("--can", nargs="*", default=[],
                    help="saved `turret-can stats` output")
    ap.add_argument("--json", action="store_true", help="machine-readable")
    ap.add_argument("--out", default="", help="write the report here too")
    args = ap.parse_args(argv)

    if not args.log and not args.telemetry and not args.can:
        ap.error("nothing to read: pass --log/--telemetry/--can")
    for p in list(args.log) + list(args.telemetry) + list(args.can):
        if not os.path.exists(p):
            print(f"error: no such file: {p}", file=sys.stderr)
            return 2

    result = analyse(args.log, args.telemetry, args.can)
    if args.json:
        text = json.dumps({"meta": result["meta"],
                           "rows": [{"family": r.family, "metric": r.name,
                                     "value": r.value, "source": r.source}
                                    for r in result["rows"]],
                           }, indent=2)
    else:
        text = render_md(result)
    print(text)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text + "\n")
        print(f"\nwrote {os.path.abspath(args.out)}", file=sys.stderr)
    missing = sum(1 for r in result["rows"] if r.missing())
    print(f"\n{len(result['rows']) - missing}/{len(result['rows'])} metrics "
          f"measured, {missing} NOT MEASURED", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
