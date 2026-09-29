#!/usr/bin/env python3
"""Freeze a campaign design into campaign.lock.json — the planner decides nothing while running.

ADR-002.1 D4 forbids the arrangement that produced the last run: a person picking the next numbers
round by round, with no design and no stop criterion. So the sequence of candidates is computed *once*,
here, offline, from a small spec, and written down. The runner later reads the file and obeys it. If it
wants to deviate, that is a new lock file and a new campaign — which is the point: the document is the
decision, and a decision that can be amended mid-flight by whoever is tired at 1 am is not a design.

What the lock binds, and why each one is here rather than remembered:

  * `design_sha256` over the canonical design (dimensions, levels, order, gates) — so an archived trial
    can be checked against the plan that produced it;
  * `inventory_sha256` plus the binary digest and source revision from the inventory itself — §6 asks for
    the binary SHA to be recorded, and D8 freezes the source for the campaign. Two documents that
    disagree mean the evidence is not about the build that ran;
  * the payload profile, because a tuning result without a payload binding is a result about an
    unladen station, which is not the machine that will be used.

Dimension names are checked against the inventory's `experiment_writable` set rather than typed into a
second list, and a protected name is refused with the clause that protects it: the tuner may not widen
its own current envelope (D7). Numeric ranges are *recorded* here as prose and *enforced* by the
server — the parameter transaction refuses an out-of-range candidate and says so, which beats a second
parser of a sentence like "0 < kp <= 10, enforced by …".

    .venv/bin/python Firmware/tools/adr0021_plan.py --freeze spec.json --out docs/ADR-002.1/manifests/campaign.lock.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys

FIRMWARE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INVENTORY = os.path.join(FIRMWARE, "docs", "ADR-002.1", "manifests", "parameter_inventory.json")
COARSE_CANDIDATES = 16      # ADR-002.1 D4: baseline -> coarse(16) -> conditional refine(8) -> confirm
MAX_REFINE_CANDIDATES = 8
PROTECTED_CLAUSE = ("ADR-002.1 D7: the tuner may not raise the current envelope; a protected field is "
                    "an operator decision made outside the campaign")


def canonical(value) -> str:
    """One spelling of "the same design": sorted keys, no incidental whitespace differences."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def load_inventory(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        raw = handle.read()
    document = json.loads(raw)
    document["_sha256"] = hashlib.sha256(raw.encode("utf-8")).hexdigest()
    return document


def canonical_order(design: dict) -> list:
    """The coarse grid, in an order that does not depend on how the spec was written.

    Dimensions sort by parameter name and levels sort numerically, so two operators writing the same
    design with different key order get the same candidate list and the same hash. A "randomized run
    order" would need a seed recorded anyway, and a frozen order needs none.
    """
    dims = sorted(design["dimensions"], key=lambda d: d["name"])
    names = [d["name"] for d in dims]
    levels = [sorted(d["levels"]) for d in dims]
    candidates = []
    total = 1
    for level in levels:
        total *= len(level)
    for index in range(total):
        rest, values = index, {}
        for position, name in enumerate(names):
            width = len(levels[position])
            values[name] = levels[position][rest % width]
            rest //= width
        candidates.append({"candidate_id": f"c{index:02d}", "params": values})
    return names, candidates


def freeze(spec: dict, inventory: dict) -> dict:
    """Return the lock document, or raise with every reason at once. Refuses to invent a default."""
    problems = []
    for key in ("campaign_id", "objective", "dimensions", "refine", "confirm", "stop", "scorer",
                "fixed", "payload_profile"):
        if key not in spec:
            problems.append(f"the spec has no {key!r}; a frozen design states it rather than "
                            "defaulting it, because a default is a decision nobody wrote down")
    if problems:
        raise ValueError("\n".join(problems))

    by_name = {entry["name"]: entry for entry in inventory["entries"]}
    writable = {name for name, entry in by_name.items() if entry["mutability"] == "experiment_writable"}
    for dimension in spec["dimensions"]:
        name = dimension.get("name", "")
        levels = dimension.get("levels", [])
        if name not in by_name:
            problems.append(f"{name!r} is not in the parameter inventory at all; the inventory is "
                            "generated from the binary, so an unlisted name cannot be tuned")
        elif name not in writable:
            problems.append(f"{name} is {by_name[name]['mutability']}, not experiment_writable. "
                            + PROTECTED_CLAUSE)
        if len(levels) < 2:
            problems.append(f"{name} has {len(levels)} level(s); a dimension that is not varied is "
                            "not a dimension, it is a constant and belongs in 'fixed'")
        if any(not isinstance(level, (int, float)) for level in levels):
            problems.append(f"{name} has a non-numeric level; every searchable field in this firmware "
                            "is numeric (booleans are derived from amplitudes, see the inventory)")

    names, coarse = canonical_order(spec) if not problems else ([], [])
    stated_reason = spec.get("coarse_count_reason")
    if not problems and len(coarse) != COARSE_CANDIDATES and not stated_reason:
        problems.append(
            f"the grid is {len(coarse)} candidates, and D4 freezes coarse at {COARSE_CANDIDATES}. "
            "Either change the levels or state coarse_count_reason — a design quietly resized to fit "
            "the grid is how a campaign stops being the design")
    refine = spec["refine"]
    if not isinstance(refine.get("max_candidates"), int) or refine["max_candidates"] > MAX_REFINE_CANDIDATES:
        problems.append(f"refine.max_candidates must be an integer <= {MAX_REFINE_CANDIDATES} (D4)")
    if not str(refine.get("gate", "")).strip():
        problems.append("refine.gate is empty; the refine stage is *conditional*, and a gate that says "
                        "nothing will always be satisfied")
    if int(spec["confirm"].get("repeats", 0)) < 2:
        problems.append("confirm.repeats must be >= 2: one measurement is not a confirmation")
    if not str(spec["stop"].get("max_trials", "")).strip() or "no_improvement_rounds" not in spec["stop"]:
        problems.append("stop must state max_trials and no_improvement_rounds; the last run's report "
                        "names the absence of a stop criterion as its own finding")
    if not str(spec["payload_profile"]).strip():
        problems.append("payload_profile is empty; a tuning result without a payload binding describes "
                        "an unladen station, which is not the machine that will be used")
    if problems:
        raise ValueError("\n".join(problems))

    design = {
        "campaign_id": spec["campaign_id"],
        "objective": spec["objective"],
        "scorer": spec["scorer"],
        "fixed": spec["fixed"],
        "dimensions": [{"name": dimension["name"],
                        "unit": by_name[dimension["name"]]["unit"],
                        "supported_range": by_name[dimension["name"]]["supported_range"],
                        "readback_source": by_name[dimension["name"]]["readback_source"],
                        "levels": sorted(dimension["levels"])}
                       for dimension in sorted(spec["dimensions"], key=lambda d: d["name"])],
        "coarse": coarse,
        "refine": refine,
        "confirm": spec["confirm"],
        "stop": spec["stop"],
        "payload_profile": spec["payload_profile"],
    }
    if stated_reason:
        design["coarse_count_reason"] = stated_reason
    generated = inventory["generated_from"]
    return {
        "schema": 1,
        "design_sha256": sha256_text(canonical(design)),
        "design": design,
        "bound_to": {
            "inventory_sha256": inventory["_sha256"],
            "binary": generated.get("binary", ""),
            "binary_sha256": generated.get("binary_sha256", ""),
            "source_rev": generated.get("source_rev", ""),
            "config": generated.get("config", ""),
            "hardware_profile": generated.get("hardware_profile", ""),
            "note": "enforcement lives in controld's parameter transaction; this document records the "
                    "ranges as prose so an archived trial can be read without the binary",
        },
        "sequence": ["baseline", "coarse", "refine_if_gate_passes", "confirm", "report"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--freeze", metavar="SPEC.json", help="freeze this spec into a lock document")
    parser.add_argument("--inventory", default=INVENTORY)
    parser.add_argument("--out", default="-", help="write here, or - for stdout")
    parser.add_argument("--check", metavar="LOCK.json", help="re-derive and compare design_sha256")
    args = parser.parse_args()
    inventory = load_inventory(args.inventory)
    if args.check:
        with open(args.check, encoding="utf-8") as handle:
            lock = json.load(handle)
        if lock["bound_to"]["inventory_sha256"] != inventory["_sha256"]:
            raise SystemExit("BLOCKED_inventory_drift: the campaign was frozen against a different "
                             "parameter inventory than the one in the tree")
        if lock["bound_to"]["binary_sha256"] != inventory["generated_from"]["binary_sha256"]:
            raise SystemExit("BLOCKED_binary_drift: the inventory no longer describes the built "
                             "controld; a campaign frozen against the old binary is not about this one")
        if sha256_text(canonical(lock["design"])) != lock["design_sha256"]:
            raise SystemExit("the lock file's design does not hash to its own design_sha256: it was "
                             "edited after freezing, which means a new campaign, not an edit")
        print(f"campaign lock is self-consistent: {lock['design']['campaign_id']}, "
              f"{len(lock['design']['coarse'])} coarse candidates, "
              f"<= {lock['design']['refine']['max_candidates']} refine, "
              f"x{lock['design']['confirm']['repeats']} confirm")
        return 0
    if not args.freeze:
        parser.error("nothing to do: pass --freeze SPEC.json or --check LOCK.json")
    with open(args.freeze, encoding="utf-8") as handle:
        spec = json.load(handle)
    try:
        lock = freeze(spec, inventory)
    except ValueError as reason:
        print(f"campaign design refused:\n{reason}", file=sys.stderr)
        return 1
    text = json.dumps(lock, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    if args.out == "-":
        sys.stdout.write(text)
    else:
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write(text)
        print(f"wrote {os.path.relpath(args.out, FIRMWARE)}: design_sha256={lock['design_sha256'][:16]}…")
    return 0


if __name__ == "__main__":
    sys.exit(main())
