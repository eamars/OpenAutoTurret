#!/usr/bin/env python3
"""Measure the two inference feeds as the station reports them, and judge them fairly.

Why a tool and not a log read: the acceptance for the dual feed is *"each camera ~30 fps, fairness
>= 0.9"*, and both halves are ratios over an interval. A single snapshot of `inferences` says
nothing about either -- it is a total since boot, and a camera that stalled ten minutes ago looks
healthy in it. So this reads the published health document twice and divides, which is the same
thing the phrase "measured rate" means everywhere else in this project.

It reads `/api/state` rather than the file on the Pi so it can be run from anywhere that can see the
station, and so the numbers it judges are the numbers the operator's page is being fed -- if the web
cannot see a feed, this tool cannot either, and that is the same failure the owner would see.

Fairness is `min(fps) / max(fps)`: a ratio, so it says nothing about whether the rig is fast, only
about whether one camera is being fed at the other's expense. Both numbers are needed, which is why
both are printed and neither is averaged away.

Usage:
    .venv/bin/python Firmware/tools/measure_dual_feed.py --url http://192.168.2.103:8080 --window 5
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request


def health(url: str, timeout: float = 5.0) -> dict:
    with urllib.request.urlopen(url.rstrip("/") + "/api/state", timeout=timeout) as response:
        body = json.load(response)
    block = body.get("inference") or {}
    if not block:
        raise SystemExit("no 'inference' block in /api/state: the daemon is not publishing health, "
                         "so there is nothing to measure (this is not a zero-fps reading)")
    return block


def feeds(block: dict) -> dict:
    """"camera id -> counter snapshot", from whichever shape the document has."""
    cameras = block.get("cameras")
    if isinstance(cameras, dict) and cameras:
        return {str(cid): dict(report) for cid, report in cameras.items()}
    # One camera, published flat: still a feed, and saying so keeps a single-camera station able to
    # run this tool instead of the tool being dual-feed-only trivia.
    return {str(block.get("camera_id") or "unbound"): dict(block)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True, help="station base URL, e.g. http://rpi-turret:8080")
    parser.add_argument("--window", type=float, default=5.0,
                        help="seconds between the two readings (default 5; a short window turns "
                             "scheduling jitter into a fairness failure)")
    parser.add_argument("--min-fairness", type=float, default=0.9)
    parser.add_argument("--expect-feeds", type=int, default=1,
                        help="how many feeds must be reporting for this to count as a dual-feed "
                             "measurement; 2 on a station that is meant to be running both")
    args = parser.parse_args()

    first = feeds(health(args.url))
    started = time.monotonic()
    time.sleep(max(1.0, args.window))
    second = feeds(health(args.url))
    elapsed = time.monotonic() - started

    rows = []
    for cid, after in sorted(second.items()):
        before = first.get(cid)
        if before is None:
            rows.append((cid, None, after.get("inferences"), None, "appeared mid-window"))
            continue
        delta = int(after.get("inferences", 0)) - int(before.get("inferences", 0))
        fps = round(delta / elapsed, 2)
        rows.append((cid, delta, after.get("inferences"), fps, after))

    print(f"window: {elapsed:.2f} s   backend: {second.get('adapter') or '(no report)'}")
    for cid, delta, total, fps, report in rows:
        leg = (report or {}).get("stream") if isinstance(report, dict) else None
        fails = (report or {}).get("failures") if isinstance(report, dict) else None
        print(f"  {cid[:14]:14s} leg={'x'.join(str(v) for v in leg) if leg else '?':9s}"
              f" inferences=+{delta if delta is not None else '?':<6} fps={fps if fps is not None else 'n/a':<7}"
              f" total={total} failures={fails}")

    rates = [fps for *_rest, fps, _r in rows if isinstance(fps, (int, float)) and fps > 0]
    reporting = len([r for r in rows if r[3] is not None])
    if reporting < args.expect_feeds:
        print(f"FAIL: {reporting} feed(s) reported, {args.expect_feeds} expected"
              f" -- a feed missing from the document is not a feed at 0 fps")
        return 1
    if len(rates) < 2:
        print("no two rates to compare: fairness is a ratio between feeds, not a property of one")
        return 0 if args.expect_feeds < 2 else 1
    fairness = min(rates) / max(rates)
    print(f"fairness = {fairness:.3f}  (min {min(rates)} / max {max(rates)}, threshold "
          f"{args.min_fairness})")
    if fairness < args.min_fairness:
        print("FAIL: one feed is being served at the other's expense")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
