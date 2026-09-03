#!/usr/bin/env python3
"""§110 acceptance, as a file that cannot be ticked by an agent.

The architecture's §110 checklist lives in the document, which is the right place for a
list of things to be true. It is the wrong place to record whether they are: a checkbox in a
markdown file can be set by anyone with a text editor, and the person most likely to tick a
box optimistically is the one who wrote the code and wants it to be finished. This session has
already had to correct itself twice over claims that were "recorded" without evidence, so the
mechanism is now the one carrying that rule:

    the doc is the checklist; this tool's log is the evidence; and a checkbox in the doc is
    deliberately *not* read for status.

What that buys, concretely:

  * An item reaches `passed` only with a named human observer, the station it was observed
    on, and a sentence about what was seen. There is no flag that means "trust me".
  * `--method simulation` cannot produce `passed`. It produces `simulated (not acceptance)`,
    which is what simulation is. This whole project has one failure mode in common with most:
    green tests read like acceptance and nobody re-reads the sentence.
  * Item text is stored with the entry, and the doc's hash with the log. If §110 is edited
    after an item was accepted, the report says so on that item instead of quietly certifying
    a checklist that no longer exists.
  * This tool never issues a command. It reads `/api/state` and nothing else; there is a test
    that says so, because an acceptance runner that can move the turret is not a runner, it
    is an operator — and the operator in this project is a person standing where the arms
    cannot reach them.

Usage:

    v3_acceptance.py report
    v3_acceptance.py record --item 3 --method hardware --by "R. Ams" \\
        --station rpi-turret --evidence "watched it acquire a person at 3 m ..."
    v3_acceptance.py attach --item 28 --by "R. Ams" --url http://192.168.2.83:8080
"""
from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import pathlib
import re
import sys
import urllib.error
import urllib.parse
import urllib.request

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_DOC = (
    REPO_ROOT / "Firmware" / "docs" / "open_auto_turret_v3_three_mode_target_tracking_architecture.md"
)
DEFAULT_LOG = REPO_ROOT / "Firmware" / "docs" / "acceptance" / "v3_acceptance_log.json"

# The ways an item can be evidenced. Only the first two are acceptance; the third exists so
# that the substantial simulation work already done is *visible* in the report rather than
# missing from it — with a label that cannot be misread as acceptance.
ACCEPTING_METHODS = ("hardware", "operator-observed")
NON_ACCEPTING_METHODS = ("simulation", "replay", "unit-test")

# Words that are not a name. The point of `--by` is that a human is answerable for the
# sentence in `--evidence`; an agent signing its own acceptance is exactly the thing this
# file exists to make impossible, and the first draft of it let me do it by omission.
UNNAMED = {"", "agent", "ai", "model", "assistant", "self", "system", "claude", "auto"}


def parse_checklist(doc_text: str):
    """Yield (group, index_in_group, text) for every §110 item, in document order.

    The checklist is parsed from the architecture document rather than copied here, because a
    second list drifts from the first within a month and then the report certifies the copy.
    """
    lines = doc_text.splitlines()
    try:
        start = next(i for i, ln in enumerate(lines) if ln.startswith("# 110."))
    except StopIteration:
        raise SystemExit("v3-acceptance: no `# 110.` section in the document — has it moved?")
    group = None
    seen = {}
    out = []
    for i in range(start + 1, len(lines)):
        ln = lines[i]
        if ln.startswith("# ") and not ln.startswith("## "):
            break  # the next top-level section closes §110
        m = re.match(r"^## (.+)$", ln)
        if m:
            group = m.group(1).strip()
            continue
        m = re.match(r"^- \[[ xX]\] (.+)$", ln)
        if m and group:
            n = seen.get(group, 0) + 1
            seen[group] = n
            out.append((group, n, m.group(1).strip()))
    if not out:
        raise SystemExit("v3-acceptance: §110 parsed with no items — the format changed")
    return out


def item_id(group: str, index: int) -> str:
    return "%s/%d" % (group, index)


def resolve(items, wanted: str):
    """Find an item by 'GROUP/N', by bare N (1-based over the whole checklist), or by a
    unique substring. Ambiguity is an error naming every candidate, never a guess: choosing
    the wrong acceptance item is worse than doing none."""
    listed = {item_id(g, n): (g, n, t) for (g, n, t) in items}
    if wanted in listed:
        return [listed[wanted]]
    if wanted.isdigit():
        n = int(wanted)
        if 1 <= n <= len(items):
            return [items[n - 1]]
        raise SystemExit("v3-acceptance: item %d is outside 1..%d" % (n, len(items)))
    hits = [(g, i, t) for (g, i, t) in items if wanted.lower() in t.lower()]
    if len(hits) == 1:
        return hits
    if not hits:
        raise SystemExit("v3-acceptance: no §110 item matches %r" % wanted)
    raise SystemExit(
        "v3-acceptance: %r matches %d items, name it exactly:\n  %s"
        % (wanted, len(hits), "\n  ".join("%s/%d %s" % h for h in hits))
    )


def load_log(path: pathlib.Path) -> dict:
    if not path.exists():
        return {"entries": {}, "doc_sha256": None}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        raise SystemExit("v3-acceptance: %s is not readable JSON (%s) — refusing to overwrite"
                         % (path, e))


def now_iso() -> str:
    return _dt.datetime.now(_dt.timezone.utc).astimezone().isoformat(timespec="seconds")


def cmd_report(args) -> int:
    doc = pathlib.Path(args.doc).read_text(encoding="utf-8")
    items = parse_checklist(doc)
    doc_hash = hashlib.sha256(doc.encode("utf-8")).hexdigest()
    log = load_log(pathlib.Path(args.log))
    entries = log.get("entries", {})

    counts = {"passed": 0, "simulated": 0, "untried": 0, "stale": 0}
    group = None
    for (g, n, text) in items:
        if g != group:
            group = g
            print("\n%s" % g)
        e = entries.get(item_id(g, n))
        if not e:
            counts["untried"] += 1
            print("  [ ] %2d  %s" % (n, text))
            continue
        stale = e.get("item_text") != text
        if e.get("status") == "passed" and not stale:
            counts["passed"] += 1
            print("  [x] %2d  %s" % (n, text))
        elif e.get("status") == "passed" and stale:
            counts["stale"] += 1
            print("  [?] %2d  %s" % (n, text))
            print("        recorded against different wording — re-check before believing it")
            print("        was: %s" % e.get("item_text"))
        else:
            counts["simulated"] += 1
            print("  [~] %2d  %s" % (n, text))
        tail = "        %s — %s%s" % (
            e.get("method"),
            e.get("by"),
            "" if not e.get("station") else " @ " + str(e.get("station")),
        )
        print(tail)
        if e.get("at"):
            print("        %s" % e["at"])
        if e.get("evidence"):
            print("        %s" % e["evidence"])

    total = len(items)
    print("")
    print("%d items in §110.  %d accepted on hardware by a named person.  %d shown by "
          "simulation only.  %d untried.  %d recorded against wording that has since "
          "changed."
          % (total, counts["passed"], counts["simulated"], counts["untried"], counts["stale"]))
    if log.get("doc_sha256") and log["doc_sha256"] != doc_hash:
        print("NOTE: the checklist has been edited since the last entry was recorded. "
              "Treat every entry above as needing a re-read.")
    print("Nothing here says the controller is accepted. Only the %d above does, and only "
          "those people can." % counts["passed"])
    return 0


def cmd_record(args) -> int:
    doc = pathlib.Path(args.doc).read_text(encoding="utf-8")
    items = parse_checklist(doc)
    (g, n, text) = resolve(items, args.item)[0]

    # The name matters when the entry claims acceptance; a simulation entry names the artifact
    # that showed it (a test binary, a replay transcript), and requiring a human there would
    # only teach people to leave simulation entries out of the record — which is how a report
    # ends up looking emptier than the work is.
    if args.method in ACCEPTING_METHODS:
        if not args.by or args.by.strip().lower() in UNNAMED:
            raise SystemExit(
                "v3-acceptance: --by names the human who observed this. %r is not a person, "
                "and an acceptance signed by whoever ran the tool is the thing this file "
                "exists to prevent." % args.by
            )
    elif not args.by or args.by.strip().lower() in UNNAMED:
        raise SystemExit(
            "v3-acceptance: --by names the artifact behind a non-acceptance entry (for "
            "example `ctest test_control_loop`). %r names nothing." % args.by
        )
    if not args.evidence or len(args.evidence.strip()) < 20:
        raise SystemExit(
            "v3-acceptance: --evidence has to say what was seen, in a sentence. A checkbox "
            "with a name next to it is still a checkbox."
        )

    status = "passed" if args.method in ACCEPTING_METHODS else "simulated"
    if status != "passed":
        print("v3-acceptance: %r cannot be acceptance. Recorded as `simulated`, which is what "
              "it is, and the report will keep saying so." % args.method)
    else:
        if not args.station:
            raise SystemExit(
                "v3-acceptance: acceptance names the station it was observed on. Without it "
                "the entry cannot be repeated, which makes it an opinion."
            )

    path = pathlib.Path(args.log)
    log = load_log(path)
    key = item_id(g, n)
    log.setdefault("entries", {})[key] = {
        "item_text": text,
        "group": g,
        "status": status,
        "method": args.method,
        "by": args.by.strip(),
        "station": args.station,
        "evidence": args.evidence.strip(),
        "at": now_iso(),
    }
    log["doc_sha256"] = hashlib.sha256(doc.encode("utf-8")).hexdigest()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(log, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print("recorded %s/%d as %s (%s)" % (g, n, status, args.method))
    print("  %s" % text)
    if status != "passed":
        print("  not counted as accepted; §110 needs it observed on the station")
    return 0


# The read-only fetch. Note what is *not* here: there is no code path that posts a command.
# §110 items are observed, not provoked — the person at the station decides what the turret
# does, and this tool's job ends at writing down what they saw.
TELEMETRY_PATHS = ("/api/state", "/api/health")


def cmd_attach(args) -> int:
    doc = pathlib.Path(args.doc).read_text(encoding="utf-8")
    items = parse_checklist(doc)
    (g, n, text) = resolve(items, args.item)[0]
    base = args.url.rstrip("/")
    parsed = urllib.parse.urlparse(base)
    if parsed.path not in ("",) + TELEMETRY_PATHS:
        # Not caution theatre: the same base URL serves POST /api/command, and a tool that
        # will fetch an arbitrary URL is a tool that can be pointed at the wrong one by a
        # tired person at 11pm.
        raise SystemExit(
            "v3-acceptance: refusing %r. This tool fetches %s only." % (base, TELEMETRY_PATHS)
        )
    url = base if parsed.path else base + "/api/state"
    try:
        with urllib.request.urlopen(url, timeout=args.timeout) as r:  # noqa: S310 (fixed host)
            body = r.read(400_000).decode("utf-8", "replace")
    except (urllib.error.URLError, OSError) as e:
        raise SystemExit("v3-acceptance: could not read %s: %s" % (url, e))
    try:
        snap = json.loads(body)
    except json.JSONDecodeError:
        raise SystemExit("v3-acceptance: %s did not return JSON" % url)

    keep = ("operating_mode", "mode_phase", "phase", "safety_action", "selected_track_id",
            "intent_type", "intent_source", "q_yaw_rad", "q_pitch_rad", "q_ref_yaw_rad",
            "q_ref_pitch_rad", "v_yaw_rad_s", "v_pitch_rad_s", "aim_point_valid",
            "track_list_age_ms", "control_cycle_us")
    excerpt = {k: snap[k] for k in keep if k in snap}
    args.evidence = "live telemetry from %s: %s" % (url, json.dumps(excerpt, sort_keys=True))
    args.method = "operator-observed"
    args.station = args.station or (parsed.hostname or "named-by-url")
    print("attach: what follows is one snapshot, read while the operator watched. It is "
          "evidence about an instant, not about the behaviour — say in --evidence what the "
          "operator saw.")
    return cmd_record(args)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="v3-acceptance", description=__doc__.splitlines()[0])
    ap.add_argument("--doc", default=str(DEFAULT_DOC))
    ap.add_argument("--log", default=str(DEFAULT_LOG))
    sub = ap.add_subparsers(dest="cmd")

    sub.add_parser("report", help="print §110 with what has been evidenced (default)")

    rec = sub.add_parser("record", help="record evidence for one item")
    rec.add_argument("--item", required=True, help="'GROUP/N', the number in the report, or a phrase")
    rec.add_argument("--method", required=True,
                     choices=list(ACCEPTING_METHODS) + list(NON_ACCEPTING_METHODS))
    rec.add_argument("--by", required=True, help="the human who observed it")
    rec.add_argument("--station", default="", help="which machine this was observed on")
    rec.add_argument("--evidence", required=True, help="what was seen, in a sentence")

    att = sub.add_parser("attach", help="attach a read-only telemetry snapshot as evidence")
    att.add_argument("--item", required=True)
    att.add_argument("--by", required=True)
    att.add_argument("--url", required=True, help="http://station:8080 (or .../api/state)")
    att.add_argument("--station", default="")
    att.add_argument("--timeout", type=float, default=4.0)
    att.add_argument("--evidence", default="")

    args = ap.parse_args(argv)
    if args.cmd in (None, "report"):
        return cmd_report(args)
    if args.cmd == "record":
        return cmd_record(args)
    if args.cmd == "attach":
        return cmd_attach(args)
    return 2


if __name__ == "__main__":
    sys.exit(main())
