#!/usr/bin/env python3
"""Can both sensors stream at the same time, and what does that cost?

Everything about the second camera -- the PIP on the page, a narrow-angle detector, a Hailo
that switches its input -- rests on one unmeasured assumption: that this station can hold
**two open sensors at once**. It has never been measured here. The station's own stack opens
exactly one (`visiond: camera 0 stream 1920x1080 …`), and the single-camera Hailo probe opens
one at a time, so neither of them has ever answered it. Pi5 has two CSI connectors and both
drivers are loaded (`imx500`, `imx477` in `lsmod`), which is a reason to expect yes and not
yet an answer.

So this probe asks the question in the cheapest form that still counts as an answer: open one
stream per requested sensor in a single libcamera context, run for a fixed window, and report
per-sensor frame counts, inter-frame intervals, **sequence gaps** (a sensor that silently
drops frames under load is a worse answer than one that refuses to open), system load and SoC
temperature before and after.

Nothing here commands a motor, loads a model, or touches CAN. Like the Hailo probe, it holds
the launcher lock, so the stack cannot start underneath it.

Example on the Pi, with the stack stopped:
  python Firmware/tools/probe_dual_camera.py --seconds 10
  python Firmware/tools/probe_dual_camera.py --selftest      # no hardware needed
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import statistics
import sys
import time
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from probe_hailo_camera import _acquire_station_launcher_lock  # noqa: E402  (one lock, one implementation)

DEFAULT_MODELS = ("imx500", "imx477")
THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"


def _camera_numbers(infos: list[dict[str, Any]], models: tuple[str, ...]) -> dict[str, int]:
    """Map each requested sensor model to its camera number.

    Loud on both directions of wrong: two sensors of the same model, and a requested model
    that is not plugged in. A probe that quietly skipped the second sensor would report a
    confident "yes, one camera streams fine" about a test that never opened two.
    """
    by_model: dict[str, list[int]] = {}
    for info in infos:
        model = str(info.get("Model", info.get("model", ""))).lower()
        num = info.get("Num", info.get("num"))
        if model and num is not None:
            by_model.setdefault(model, []).append(int(num))
    chosen: dict[str, int] = {}
    for model in models:
        found = by_model.get(model.lower(), [])
        if not found:
            raise RuntimeError(
                f"requested sensor {model!r} is not enumerated; present models: "
                f"{sorted(by_model) or 'none'}"
            )
        if len(found) > 1:
            raise RuntimeError(f"sensor model {model!r} enumerated more than once at {found}")
        chosen[model] = found[0]
    return chosen


def _soc_temp_c() -> float | None:
    try:
        return int(Path(THERMAL_PATH).read_text().strip()) / 1000.0
    except (OSError, ValueError):
        return None  # not a Pi, or the node is not readable; the probe still answers


def _percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * pct / 100.0))]


def _run_window(streams: list[dict[str, Any]], seconds: float) -> None:
    """Pull frames from every stream in one loop, recording gaps in each sensor's sequence."""
    deadline = time.monotonic() + seconds
    for stream in streams:
        stream["camera"].start()
    try:
        while time.monotonic() < deadline:
            for stream in streams:
                cam = stream["camera"]
                try:
                    request = cam.capture_request()
                except Exception as exc:  # noqa: BLE001 - named below, never swallowed
                    stream["error"] = f"{type(exc).__name__}: {exc}"
                    continue
                try:
                    metadata = request.get_metadata()
                    now = time.monotonic()
                    seq = metadata.get("SequenceNumber") if metadata else None
                    if stream["frames"]:
                        stream["intervals_ms"].append((now - stream["last_t"]) * 1000.0)
                        if seq is not None and stream["last_seq"] is not None:
                            skipped = int(seq) - int(stream["last_seq"]) - 1
                            if skipped > 0:
                                stream["sequence_gaps"] += skipped
                    stream["frames"] += 1
                    stream["last_t"] = now
                    stream["last_seq"] = seq
                finally:
                    request.dispose()
    finally:
        for stream in streams:
            try:
                stream["camera"].stop()
            except Exception as exc:  # noqa: BLE001
                stream.setdefault("stop_error", f"{type(exc).__name__}: {exc}")


def _selftest() -> int:
    infos = [
        {"Model": "imx500", "Num": 0},
        {"Model": "rpinear-pwme", "Num": 3},
        {"Model": "imx477", "Num": 1},
    ]
    checks = 0
    chosen = _camera_numbers(infos, DEFAULT_MODELS)
    assert chosen == {"imx500": 0, "imx477": 1}, chosen
    checks += 1
    try:
        _camera_numbers(infos, ("imx500", "imx219"))
    except RuntimeError as exc:
        assert "imx219" in str(exc) and "imx477" in str(exc), exc
        checks += 1
    else:
        raise AssertionError("a missing sensor must name itself and list what IS present")
    try:
        _camera_numbers([{"Model": "imx477", "Num": 1}, {"Model": "imx477", "Num": 2}], ("imx477",))
    except RuntimeError as exc:
        assert "more than once" in str(exc), exc
        checks += 1
    else:
        raise AssertionError("two sensors of one model must refuse, not pick silently")
    assert _percentile([1.0, 2.0, 3.0], 95) == 3.0
    assert _percentile([], 50) is None
    checks += 1
    print(f"dual-camera probe selftest: {checks}/{checks} checks passed (no hardware touched)")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--models", default=",".join(DEFAULT_MODELS),
                        help=f"comma-separated sensor models (default: {','.join(DEFAULT_MODELS)})")
    parser.add_argument("--seconds", default=10.0, type=float)
    parser.add_argument("--width", default=1280, type=int)
    parser.add_argument("--height", default=720, type=int)
    parser.add_argument("--fps", default=15.0, type=float)
    parser.add_argument("--selftest", action="store_true", help="run the selection logic with no hardware")
    args = parser.parse_args()

    if args.selftest:
        return _selftest()
    if args.seconds <= 0:
        raise SystemExit("--seconds must be positive")

    try:
        from picamera2 import Picamera2
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(f"picamera2 unavailable, so this probe cannot answer anything: "
                         f"{type(exc).__name__}: {exc}") from exc

    models = tuple(m.strip() for m in args.models.split(",") if m.strip())
    # `Num`, not the position in the list: libcamera's enumeration order and its camera
    # numbers are different things, and picking by position is how a probe ends up opening
    # the sensor it never meant to.
    infos = [{"Model": info.get("Model"), "Num": info.get("Num")}
             for info in Picamera2.global_camera_info()]
    wanted = _camera_numbers(infos, models)

    lock = _acquire_station_launcher_lock()
    opened: list[dict[str, Any]] = []
    try:
        for model, number in wanted.items():
            entry: dict[str, Any] = {
                "model": model, "camera_num": number, "frames": 0, "intervals_ms": [],
                "sequence_gaps": 0, "last_t": 0.0, "last_seq": None, "error": "",
            }
            try:
                cam = Picamera2(number)
                entry["enumerated_model"] = str(cam.camera_properties.get("Model", "?")) \
                    if hasattr(cam, "camera_properties") else "?"
                cfg = cam.create_video_configuration(main={"size": (args.width, args.height)})
                cam.configure(cfg)
                entry["camera"] = cam
                opened.append(entry)
            except Exception as exc:  # noqa: BLE001 - an open failure IS the answer
                entry["error"] = f"open failed: {type(exc).__name__}: {exc}"
                opened.append(entry)

        before = {"load_1m": os.getloadavg()[0], "soc_temp_c": _soc_temp_c()}
        live = [s for s in opened if s.get("camera") is not None]
        if len(live) < 2:
            raise SystemExit("fewer than two sensors opened: "
                             + "; ".join(f"{s['model']}#{s['camera_num']} {s['error'] or 'open'}"
                                        for s in opened))
        _run_window(live, args.seconds)
        after = {"load_1m": os.getloadavg()[0], "soc_temp_c": _soc_temp_c()}
    finally:
        for stream in opened:
            cam = stream.get("camera")
            if cam is not None:
                try:
                    cam.close()
                except Exception as exc:  # noqa: BLE001
                    # Cannot change the answer, but it must not be silence either: a leaked
                    # sensor makes the NEXT opener fail with a message that blames itself.
                    stream.setdefault("close_error", f"{type(exc).__name__}: {exc}")
        os.close(lock)

    report: dict[str, Any] = {
        "requested": {"models": list(models), "size": f"{args.width}x{args.height}",
                      "fps": args.fps, "seconds": args.seconds},
        "both_open": sum(1 for s in opened if s.get("camera") is not None),
        "before": before, "after": after,
        "sensors": [],
    }
    for stream in opened:
        span = (stream["intervals_ms"] and
                sum(stream["intervals_ms"]) / 1000.0 or 0.0)
        report["sensors"].append({
            "model": stream["model"],
            "camera_num": stream["camera_num"],
            "frames": stream["frames"],
            "measured_fps": round(stream["frames"] / span, 3) if span > 0.5 else None,
            "interval_p50_ms": (round(_percentile(stream["intervals_ms"], 50) or 0, 3)
                                or None),
            "interval_p95_ms": (round(_percentile(stream["intervals_ms"], 95) or 0, 3)
                                or None),
            "sequence_gaps": stream["sequence_gaps"],
            "error": stream["error"],
            "stop_error": stream.get("stop_error", ""),
        })
    print(json.dumps(report, indent=2, sort_keys=True))
    silent = [s["model"] for s in report["sensors"] if s["frames"] == 0]
    if silent:
        print(f"ANSWER: no -- {' '.join(silent)} opened and delivered zero frames", file=sys.stderr)
        return 1
    print("ANSWER: yes -- both sensors streamed simultaneously")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
