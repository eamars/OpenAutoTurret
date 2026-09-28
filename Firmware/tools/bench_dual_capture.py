#!/usr/bin/env python3
"""Phase 2 of the dual-camera validation: capture only, no Hailo, no on-sensor NN.

Why a separate instrument instead of extending the feasibility probe: the probe answers a yes/no
("can both sensors be open at once"). This answers *what it costs and whether it is stable*, and
the matrix has more than one shape in it, so the tool takes a spec per camera and reports per
camera.

Three things it is deliberately strict about, because the architect's rules say so:

- **Requested fps is never reported as measured fps.** The rate is pinned with
  `FrameDurationLimits` (min and max together, a paired control), and what is delivered is
  computed from the timestamps the frames actually arrived with.
- **Cameras are named by sensor identity, and the identity is re-read after the open.** Asking
  for `imx500` and getting some other sensor because an enumeration index moved is the failure
  this whole station has been stepping around; the check is `camera_properties` after open, not
  a trust in the number we passed in.
- **One thread per camera.** A round-robin loop measures how fast *the loop* goes, not how fast
  the sensor delivers: blocking on camera A makes camera B's intervals look paced by us. Each
  camera gets its own puller, and the main thread only samples system counters.

Frames are released immediately -- this measures the capture subsystem, not an algorithm.

Example on the Pi, stack stopped:
  python Firmware/tools/bench_dual_capture.py --spec imx500=2028x1520@30,imx477=2028x1080@30
  python Firmware/tools/bench_dual_capture.py --spec imx500=2028x1520@30      # 单路，同一把尺
  python Firmware/tools/bench_dual_capture.py --selftest
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import sys
import threading
import time
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from probe_dual_camera import (  # noqa: E402  (one lock, one camera-number mapping, one percentile)
    _acquire_station_launcher_lock,
    _camera_numbers,
    _percentile,
    _soc_temp_c,
)

SPEC_RE = re.compile(r"^(?P<model>[a-z0-9_]+)=(?P<width>\d+)x(?P<height>\d+)(@(?P<fps>\d+(?:\.\d+)?))?$")


def parse_spec(text: str) -> list[dict[str, Any]]:
    """`imx500=2028x1520@30,imx477=2028x1080` -> specs. Bad entries name themselves."""
    specs = []
    for part in [p.strip() for p in text.split(",") if p.strip()]:
        m = SPEC_RE.match(part.lower())
        if not m:
            raise SystemExit(f"bad --spec entry {part!r}; want model=WxH[@fps], e.g. imx500=2028x1520@30")
        specs.append({"model": m.group("model"), "width": int(m.group("width")),
                      "height": int(m.group("height")), "fps": float(m.group("fps") or 30.0)})
    if not specs:
        raise SystemExit("--spec named nothing")
    return specs


def cpu_busy_percent(before: tuple[int, int], after: tuple[int, int]) -> float | None:
    """Busy fraction of all CPUs from two /proc/stat snapshots (idle+iowait vs total)."""
    idle0, total0 = before
    idle1, total1 = after
    dt = total1 - total0
    if dt <= 0:
        return None
    return round(100.0 * (1.0 - (idle1 - idle0) / dt), 2)


def read_proc_stat() -> tuple[int, int]:
    with open("/proc/stat", "r", encoding="utf-8") as handle:
        fields = [int(x) for x in handle.readline().split()[1:]]
    idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
    return idle, sum(fields)


def mem_available_mb() -> int:
    for line in open("/proc/meminfo", "r", encoding="utf-8"):
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) // 1024
    return -1


def _puller(stream: dict[str, Any]) -> None:
    """One camera's frame puller. Records arrival times and the sensor's own sequence/timestamp."""
    cam = stream["camera"]
    intervals: list[float] = []
    anomalies: list[str] = []
    gaps = 0
    last_mono = 0.0
    last_seq: int | None = None
    last_sensor: int | None = None
    frames = 0
    max_gap_ms = 0.0
    try:
        deadline = stream["deadline"]
        while time.monotonic() < deadline:
            try:
                request = cam.capture_request()
            except Exception as exc:  # noqa: BLE001
                anomalies.append(f"capture: {type(exc).__name__}: {exc}")
                break
            try:
                now = time.monotonic()
                frames += 1
                if last_mono:
                    gap_ms = (now - last_mono) * 1000.0
                    intervals.append(gap_ms)
                    max_gap_ms = max(max_gap_ms, gap_ms)
                metadata = request.get_metadata() or {}
                seq = metadata.get("SequenceNumber")
                sensor_ns = metadata.get("SensorTimestamp", 0)
                if seq is not None and last_seq is not None and int(seq) - int(last_seq) > 1:
                    gaps += int(seq) - int(last_seq) - 1
                if last_sensor and sensor_ns and sensor_ns <= last_sensor:
                    anomalies.append(f"SensorTimestamp went backwards at seq {seq}")
                last_mono, last_seq, last_sensor = now, seq, sensor_ns or last_sensor
            finally:
                request.release()
    finally:
        stream.update(frames=frames, intervals_ms=intervals, sequence_gaps=gaps,
                      max_gap_ms=round(max_gap_ms, 3), anomalies=anomalies[:5],
                      anomaly_count=len(anomalies))


def _selftest() -> int:
    checks = 0
    specs = parse_spec("imx500=2028x1520@30,imx477=2028x1080")
    assert specs[0] == {"model": "imx500", "width": 2028, "height": 1520, "fps": 30.0}, specs[0]
    assert specs[1]["fps"] == 30.0, "缺 @fps 时按 30，但要能被显式覆盖"
    checks += 1
    for bad in ("imx500=2028x1520x30", "imx500=2028", "=2028x1520"):
        try:
            parse_spec(bad)
        except SystemExit as exc:
            assert bad in str(exc), exc
        else:
            raise AssertionError(f"{bad} must be refused by name")
    checks += 1
    assert cpu_busy_percent((100, 200), (110, 300)) == 90.0        # 10 idle of 100 ticks
    assert cpu_busy_percent((100, 200), (100, 200)) is None          # no elapsed time, no answer
    assert _percentile([], 99) is None
    checks += 1
    assert mem_available_mb() > 0, "/proc/meminfo 读不到，测量口径就缺了一角"
    checks += 1
    print(f"dual-capture bench selftest: {checks}/{checks} checks passed（不碰硬件）")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--spec", default="imx500=2028x1520@30,imx477=2028x1080@30")
    parser.add_argument("--seconds", default=20.0, type=float)
    parser.add_argument("--label", default="", help="free text written into the report (which test this is)")
    parser.add_argument("--out", type=Path, help="write the JSON report here as well as stdout")
    parser.add_argument("--no-lock", action="store_true", help="the stack already holds one sensor")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        return _selftest()
    if args.seconds < 3:
        raise SystemExit("--seconds must be at least 3; shorter windows measure startup, not streaming")

    try:
        from picamera2 import Picamera2
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(f"picamera2 missing, so this bench cannot measure anything: "
                         f"{type(exc).__name__}: {exc}")

    specs = parse_spec(args.spec)
    infos = [{"Model": info.get("Model"), "Num": info.get("Num")}
             for info in Picamera2.global_camera_info()]
    wanted = _camera_numbers(infos, tuple(s["model"] for s in specs))

    lock = -1 if args.no_lock else _acquire_station_launcher_lock()
    streams: list[dict[str, Any]] = []
    report: dict[str, Any] = {"label": args.label, "spec": specs, "seconds": args.seconds}
    try:
        for spec in specs:
            number = wanted[spec["model"]]
            cam = Picamera2(number)
            # Identity is re-read after the open, because the number we passed in is exactly the
            # thing the station's own visiond refuses to trust (`identity ... source=by-path`).
            opened_model = str(cam.camera_properties.get("Model", "?")).lower()
            if opened_model != spec["model"]:
                cam.close()
                raise SystemExit(f"asked for {spec['model']!r} as camera {number} but the device "
                                 f"that opened identifies as {opened_model!r}; refusing to measure "
                                 "a sensor under the wrong label")
            # Default queueing: `capture_request` expects requests to be in flight, and an
            # invented `queue=False` here would starve it in a way that looks like a dead sensor.
            cam.configure(cam.create_video_configuration(
                main={"size": (spec["width"], spec["height"])}))
            dur_us = int(round(1e6 / spec["fps"]))
            try:
                cam.set_controls({"FrameDurationLimits": (dur_us, dur_us)})
            except Exception as exc:  # noqa: BLE001
                cam.close()
                raise SystemExit(f"{spec['model']}: could not pin the rate the spec asks for: "
                                 f"{type(exc).__name__}: {exc}")
            streams.append({"model": spec["model"], "camera_num": number, "camera": cam,
                            "requested": spec, "deadline": time.monotonic() + args.seconds + 5.0,
                            "opened_model": opened_model})
        for stream in streams:
            stream["camera"].start()
        now = time.monotonic()
        for stream in streams:
            stream["deadline"] = now + args.seconds
        stat0, mem0, temp0 = read_proc_stat(), mem_available_mb(), _soc_temp_c()
        temps = [t for t in (temp0,) if t is not None]
        threads = [threading.Thread(target=_puller, args=(s,), daemon=True) for s in streams]
        for t in threads:
            t.start()
        while any(t.is_alive() for t in threads):
            time.sleep(1.0)
            t_now = _soc_temp_c()
            if t_now is not None:
                temps.append(t_now)
        for t in threads:
            t.join(timeout=5.0)
        stat1, mem1, temp1 = read_proc_stat(), mem_available_mb(), _soc_temp_c()
        for stream in streams:
            try:
                stream["camera"].stop()
            except Exception as exc:  # noqa: BLE001
                stream["stop_error"] = f"{type(exc).__name__}: {exc}"

        report["system"] = {
            "cpu_busy_percent": cpu_busy_percent(stat0, stat1),
            "mem_available_mb_before": mem0, "mem_available_mb_after": mem1,
            "soc_temp_c_first": temp0, "soc_temp_c_last": temp1,
            "soc_temp_c_max": max(temps) if temps else None,
            "soc_temp_rise_c": (round(temp1 - temp0, 2) if temp0 is not None and temp1 is not None
                                else None),
        }
        rows = []
        for stream in streams:
            intervals = stream.get("intervals_ms", [])
            span = sum(intervals) / 1000.0
            delivered = round((stream.get("frames", 0) - 1) / span, 3) if span > 1.0 else None
            rows.append({
                "model": stream["model"], "verified_opened_model": stream["opened_model"],
                "camera_num": stream["camera_num"],
                "requested_mode": f"{stream['requested']['width']}x{stream['requested']['height']}",
                "requested_fps": stream["requested"]["fps"],
                "frames": stream.get("frames", 0),
                "delivered_fps": delivered,
                "interval_p50_ms": round(_percentile(intervals, 50) or 0, 3) or None,
                "interval_p95_ms": round(_percentile(intervals, 95) or 0, 3) or None,
                "interval_p99_ms": round(_percentile(intervals, 99) or 0, 3) or None,
                "max_gap_ms": stream.get("max_gap_ms"),
                "dropped_by_sequence": stream.get("sequence_gaps", 0),
                "timestamp_anomalies": stream.get("anomalies", []),
                "anomaly_count": stream.get("anomaly_count", 0),
                "stop_error": stream.get("stop_error", ""),
            })
        report["cameras"] = rows
    finally:
        for stream in streams:
            try:
                stream["camera"].close()
            except Exception as exc:  # noqa: BLE001
                stream.setdefault("close_error", f"{type(exc).__name__}: {exc}")
        if lock >= 0:
            os.close(lock)

    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if args.out:
        args.out.write_text(text, encoding="utf-8")
    silent = [row["model"] for row in report["cameras"] if not row["frames"]]
    if silent:
        print(f"ANSWER: no -- {' '.join(silent)} delivered zero frames", file=sys.stderr)
        return 1
    print(f"ANSWER: {len(report['cameras'])} camera(s) streamed; "
          f"delivered " + ", ".join(f"{r['model']}={r['delivered_fps']}" for r in report["cameras"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
