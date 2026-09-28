#!/usr/bin/env python3
"""Camera(s) -> one Hailo detector, with the pixel hot path measured operation by operation.

The baseline (Phase 3/4/5-B, committed) established: dual 1080p caps at ~52 aggregate
inferences/s, dual low-resolution reaches ~60, Hailo is never saturated, and the difference is
host-side preprocessing. This version answers the *next* question: **which operation**, and
**can the pixels stay native while Python only orchestrates**.

Instrumentation is split by operation, because one "preprocess" number hides the answer:

    capture_wait      how long the puller blocked for a frame
    map_into_python   request.make_image / make_array -- mapping the ISP buffer into Python
    resize            PIL bilinear downscale (main-stream path only)
    pad_copy          paste/letterbox + the NumPy handoff buffer
    contiguous        the array handed to HailoRT
    queue_wait        time sitting in the bounded per-camera queue
    infer / postprocess

Two inference sources, everything else identical (same HEF, queue depth, newest-wins policy,
threshold, duration), so cells A and B differ in exactly one thing:

    --inference-stream main    main frame -> PIL -> NumPy -> Hailo                      (cell A)
    --inference-stream lores   the Pi ISP emits a second aspect-preserving small stream
                               (1920x1080 -> 640x360) -> one NumPy copy into the padded
                               640x640 canvas -> Hailo                                  (cell B)

The lores role is *verified at runtime* from ``stream_configuration``: a role the stack silently
did not build would otherwise become the baseline path measured under a "lores" label. Geometry is
preserved -- a 16:9 main stream gets a 16:9 inference stream padded top and bottom, never squashed
into a square.

``e2e_ms`` is one measured interval: ``SensorTimestamp`` to inference completion. **Stage
percentiles are never summed to produce it** -- stages overlap and percentiles are not additive.
The metadata dump records what that timestamp actually is, so if it turns out to start after the
sensor/ISP portion, the unmeasured part is stated separately rather than folded in.

Per-core CPU is reported next to the overall figure: one saturated core can cap the pipeline
while a four-core average still looks idle.

Example, both cameras with an ISP inference stream:
  python Firmware/tools/bench_hailo_pipeline.py --hef <hef> \
      --spec 'imx500=1920x1080@30,imx477=1920x1080@30' --inference-stream lores
"""
from __future__ import annotations

import argparse
import collections
import json
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bench_dual_capture import (  # noqa: E402  (one spec parser, one /proc reader set)
    mem_available_mb,
    parse_spec,
    read_proc_stat,
)
from probe_dual_camera import _camera_numbers, _percentile, _soc_temp_c  # noqa: E402

SCORE_THRESHOLD = 0.5
PAD_VALUE = 114
METADATA_KEYS_OF_INTEREST = ("SensorTimestamp", "Timestamp", "ExposureTime", "SensorFrameDuration",
                             "SensorRollingRow", "SensorSilenceDelay", "Sensitivity")


def lores_size_for(main_wh: tuple[int, int], target_width: int = 640) -> tuple[int, int]:
    """Aspect-preserving inference stream for a main stream, even height, never upscaled.

    1920x1080 -> 640x360 (16:9), 2028x1520 -> 640x480 (4:3). A main stream already small enough is
    used as-is: scaling a 640x480 down to fit would cost time and buy nothing.
    """
    w, h = main_wh
    if w <= target_width and h <= target_width:
        return (w - w % 2, h - h % 2)
    lw = target_width - (target_width % 2)
    lh = int(round(lw * h / w / 2.0)) * 2
    return (lw, max(2, lh))


def pad_into(canvas: Any, arr: Any, out: int) -> None:
    """Copy a small frame into the centred window of a preallocated padded canvas.

    The padding bands are written once at setup, so the per-frame work is exactly one copy of the
    small frame -- that is the whole point of the ISP-stream path.
    """
    h, w = int(arr.shape[0]), int(arr.shape[1])
    if h > out or w > out:
        raise ValueError(f"inference stream {w}x{h} does not fit the {out}x{out} network input")
    top, left = (out - h) // 2, (out - w) // 2
    canvas[0, top:top + h, left:left + w, :] = arr


def letterbox_via_pil(image: Any, target_wh: tuple[int, int], timings: dict[str, float] | None = None) -> Any:
    """The baseline path: PIL bilinear resize + paste onto a padded canvas.

    Kept as one implementation (used by the selftest and by the measured loop) so the geometry the
    selftest pins is the geometry that runs. ``timings`` splits resize out from paste+array, which
    is the whole reason this cell exists.
    """
    import numpy as np
    from PIL import Image

    t0 = time.monotonic()
    src_w, src_h = image.size
    dst_w, dst_h = target_wh
    scale = min(dst_w / src_w, dst_h / src_h)
    if scale < 1.0:
        image = image.resize((max(1, int(src_w * scale)), max(1, int(src_h * scale))),
                             Image.Resampling.BILINEAR)
    t1 = time.monotonic()
    canvas = Image.new("RGB", (dst_w, dst_h), (PAD_VALUE, PAD_VALUE, PAD_VALUE))
    canvas.paste(image, ((dst_w - image.size[0]) // 2, (dst_h - image.size[1]) // 2))
    padded = np.array(canvas, dtype=np.uint8, copy=True)[None, ...]
    if timings is not None:
        timings["resize_ms"] = (t1 - t0) * 1000.0
        timings["pad_ms"] = (time.monotonic() - t1) * 1000.0
    return padded


def per_core_busy_percent(before: dict[str, tuple[int, int]],
                          after: dict[str, tuple[int, int]]) -> dict[str, float]:
    out = {}
    for cpu, (idle0, total0) in before.items():
        idle1, total1 = after.get(cpu, (idle0, total0))
        dt = total1 - total0
        if dt > 0:
            out[cpu] = round(100.0 * (1.0 - (idle1 - idle0) / dt), 2)
    return out


def read_per_core() -> dict[str, tuple[int, int]]:
    out: dict[str, tuple[int, int]] = {}
    with open("/proc/stat", "r", encoding="utf-8") as handle:
        for line in handle:
            if not line.startswith("cpu"):
                break
            fields = [int(x) for x in line.split()[1:]]
            idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
            out[line.split()[0]] = (idle, sum(fields))
    return out


class CameraStats:
    STAGES = ("capture_wait", "map_into_python", "resize", "pad_copy", "letterbox_total",
              "contiguous", "queue_wait", "infer", "postprocess")

    def __init__(self, model: str, number: int, opened_model: str, spec: dict[str, Any],
                 inference_source: str, source_wh: tuple[int, int]) -> None:
        self.model, self.number, self.opened_model, self.spec = model, number, opened_model, spec
        self.inference_source, self.source_wh = inference_source, source_wh
        self.captured = self.submitted = self.inferred = 0
        self.dropped_queue = self.dropped_stale = 0
        self.with_detection = 0
        self.queue_depth_max = 0
        self.python_bytes_copied = 0
        self.stream_roles: dict[str, Any] = {}
        self.metadata_sample: list[dict[str, Any]] = []
        self.samples: dict[str, list[float]] = {stage: [] for stage in self.STAGES}
        self.end_to_end_ms: list[float] = []
        self.errors: list[str] = []

    def add(self, stage: str, ms: float) -> None:
        self.samples[stage].append(ms)

    def _p(self, stage: str, pct: float) -> float:
        return round(_percentile(self.samples[stage], pct) or 0, 3)

    def buckets(self, count: int) -> list[dict[str, Any]]:
        """Latency drift over the window, in time order.

        A flat series is what "the queue is not accumulating" actually means; one aggregate p99
        cannot tell a stable pipeline from one that degraded in minute nine.
        """
        values = self.end_to_end_ms
        if not values:
            return []
        size = max(1, len(values) // count)
        out = []
        for index in range(0, len(values), size):
            chunk = values[index:index + size]
            if len(chunk) < 10:
                continue
            out.append({"n": len(chunk),
                        "p50_ms": round(_percentile(chunk, 50) or 0, 2),
                        "p95_ms": round(_percentile(chunk, 95) or 0, 2),
                        "p99_ms": round(_percentile(chunk, 99) or 0, 2)})
        return out

    def as_dict(self) -> dict[str, Any]:
        return {
            "camera_id": self.model, "camera_num": self.number,
            "verified_opened_model": self.opened_model,
            "requested_mode": f"{self.spec['width']}x{self.spec['height']}",
            "requested_fps": self.spec["fps"],
            "inference_source": self.inference_source,
            "inference_stream_size": f"{self.source_wh[0]}x{self.source_wh[1]}",
            "stream_roles": self.stream_roles,
            "metadata_sample": self.metadata_sample,
            "captured": self.captured, "submitted": self.submitted, "inferred": self.inferred,
            "dropped_queue_full": self.dropped_queue, "dropped_superseded": self.dropped_stale,
            "frames_with_detection_at_threshold": self.with_detection,
            "queue_depth_max": self.queue_depth_max,
            "python_bytes_copied_per_frame": (round(self.python_bytes_copied / max(1, self.captured))
                                              if self.captured else 0),
            "stage_p50_ms": {stage: self._p(stage, 50) for stage in self.STAGES},
            "stage_p95_ms": {stage: self._p(stage, 95) for stage in self.STAGES},
            "end_to_end_p50_ms": round(_percentile(self.end_to_end_ms, 50) or 0, 3),
            "end_to_end_p95_ms": round(_percentile(self.end_to_end_ms, 95) or 0, 3),
            "end_to_end_p99_ms": round(_percentile(self.end_to_end_ms, 99) or 0, 3),
            "end_to_end_buckets": self.buckets(6),
            "errors": self.errors[:4],
        }


def _selftest() -> int:
    import numpy as np

    checks = 0
    assert lores_size_for((1920, 1080)) == (640, 360), "16:9 必须保比例，不能压成方"
    assert lores_size_for((2028, 1520)) == (640, 480), "4:3 就配 4:3 的小流"
    assert lores_size_for((640, 480)) == (640, 480), "已经够小的主图不该再缩"
    checks += 1
    out = 640
    canvas = np.full((1, out, out, 3), PAD_VALUE, dtype=np.uint8)
    pad_into(canvas, np.zeros((360, 640, 3), dtype=np.uint8), out)
    assert canvas.shape == (1, 640, 640, 3) and canvas.flags.c_contiguous
    assert tuple(canvas[0, 0, 320]) == (PAD_VALUE,) * 3, "顶边是 padding"
    assert tuple(canvas[0, 140, 320]) == (0, 0, 0), "画面应当居中贴进去"
    assert tuple(canvas[0, 639, 320]) == (PAD_VALUE,) * 3
    checks += 1
    try:
        pad_into(canvas, np.zeros((700, 700, 3), dtype=np.uint8), out)
    except ValueError as exc:
        assert "700x700" in str(exc), exc
    else:
        raise AssertionError("放不下的推理流必须报错，不能悄悄裁掉")
    checks += 1
    assert per_core_busy_percent({"cpu0": (100, 200)}, {"cpu0": (110, 300)}) == {"cpu0": 90.0}
    assert read_per_core().get("cpu0"), "/proc/stat 没有 per-core，单核瓶颈就量不到"
    checks += 1
    from PIL import Image

    timings: dict[str, float] = {}
    padded = letterbox_via_pil(Image.new("RGB", (1920, 1080), (10, 20, 30)), (640, 640), timings)
    arr = np.array(padded)
    assert arr.shape == (1, 640, 640, 3) and arr.flags.c_contiguous, "Hailo 要 NHWC 连续内存"
    assert tuple(arr[0, 0, 320]) == (114, 114, 114) and tuple(arr[0, 320, 320]) == (10, 20, 30)
    assert timings.get("resize_ms", -1) >= 0 and timings.get("pad_ms", -1) >= 0, "计时没接上"
    checks += 1
    print(f"hailo pipeline bench selftest: {checks}/{checks} checks passed（不碰硬件、不碰相机）")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hef", type=Path)
    parser.add_argument("--spec", default="imx500=1920x1080@30,imx477=1920x1080@30")
    parser.add_argument("--seconds", default=30.0, type=float)
    parser.add_argument("--queue-depth", default=2, type=int)
    parser.add_argument("--inference-stream", choices=("main", "lores"), default="main",
                        help="main = 基线（整帧进 Python 再缩放）；lores = 让 ISP 出第二条小流")
    parser.add_argument("--lores-width", default=640, type=int)
    parser.add_argument("--label", default="")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--no-lock", action="store_true")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        return _selftest()
    if not args.hef or not args.hef.exists():
        raise SystemExit(f"--hef is required and must exist (got {args.hef})")
    if args.queue_depth < 1:
        raise SystemExit("--queue-depth must be >= 1")

    try:
        import numpy as np
        from hailo_platform import (ConfigureParams, Device, FormatType, HEF,
                                    HailoStreamInterface, InferVStreams, InputVStreamParams,
                                    OutputVStreamParams, VDevice)
        from picamera2 import Picamera2
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(f"station runtime missing: {type(exc).__name__}: {exc}")
    from probe_hailo_camera import DEFAULT_MANIFEST, _artifact_digest, _detection_counts

    _artifact_digest(args.hef.expanduser().resolve(),
                     json.loads(DEFAULT_MANIFEST.read_text(encoding="utf-8")))
    specs = parse_spec(args.spec)
    infos = [{"Model": i.get("Model"), "Num": i.get("Num")} for i in Picamera2.global_camera_info()]
    wanted = _camera_numbers(infos, tuple(s["model"] for s in specs))

    from probe_dual_camera import _acquire_station_launcher_lock
    lock = -1 if args.no_lock else _acquire_station_launcher_lock()

    hef = HEF(str(args.hef))
    input_infos, output_infos = hef.get_input_vstream_infos(), hef.get_output_vstream_infos()
    if len(input_infos) != 1 or len(output_infos) != 1:
        raise SystemExit(f"expected one input and one output vstream, got "
                         f"{len(input_infos)}/{len(output_infos)}")
    in_h, in_w = int(input_infos[0].shape[0]), int(input_infos[0].shape[1])
    out_name = output_infos[0].name

    streams: list[dict[str, Any]] = []
    stats: dict[str, CameraStats] = {}
    main_samples: list[dict[str, Any]] = []
    rc = 1
    try:
        for spec in specs:
            number = wanted[spec["model"]]
            cam = Picamera2(number)
            opened = str(cam.camera_properties.get("Model", "?")).lower()
            if opened != spec["model"]:
                cam.close()
                raise SystemExit(f"asked for {spec['model']!r}, device identifies as {opened!r}")
            main_wh = (spec["width"], spec["height"])
            lores_wh = lores_size_for(main_wh, args.lores_width)
            if args.inference_stream == "lores":
                config = cam.create_video_configuration(
                    main={"size": main_wh, "format": "RGB888"},
                    lores={"size": lores_wh, "format": "RGB888"})
            else:
                lores_wh = main_wh
                config = cam.create_video_configuration(
                    main={"size": main_wh, "format": "RGB888"})
            cam.configure(config)
            # Verify what the stack accepted and what it reports back, rather than assuming the
            # role exists: `stream_configuration()` on this Picamera2 reports only the main stream
            # (with stride/framesize, which is what the copy accounting below needs), so role
            # presence comes from the accepted configuration and the byte size from the API report.
            configured = cam.stream_configuration
            reported: dict[str, Any] = {}
            if callable(configured):
                try:
                    flat = configured()
                    if isinstance(flat, dict):
                        reported["main"] = {"size": [int(x) for x in flat.get("size", [])],
                                            "format": str(flat.get("format")),
                                            "stride": flat.get("stride"),
                                            "framesize": flat.get("framesize"),
                                            "verified_via": "stream_configuration()"}
                except Exception as exc:  # noqa: BLE001
                    cam.close()
                    raise SystemExit(f"{spec['model']}: stream_configuration() failed: "
                                     f"{type(exc).__name__}: {exc}")
                for role in ("lores", "raw"):
                    if role not in config:
                        continue
                    try:
                        per_role = configured(role)
                    except TypeError:
                        reported[role] = {"verified_via": "accepted configuration only "
                                                          "(this Picamera2 reports main alone)"}
                        continue
                    except Exception as exc:  # noqa: BLE001
                        cam.close()
                        raise SystemExit(f"{spec['model']}: stream_configuration({role!r}) failed: "
                                         f"{type(exc).__name__}: {exc}")
                    if isinstance(per_role, dict):
                        reported[role] = {"size": [int(x) for x in per_role.get("size", [])],
                                          "format": str(per_role.get("format")),
                                          "stride": per_role.get("stride"),
                                          "framesize": per_role.get("framesize"),
                                          "verified_via": f"stream_configuration({role!r})"}
            roles = {role: {"size": [int(x) for x in cfg["size"]], "format": str(cfg["format"])}
                     for role, cfg in config.items()
                     if role in ("main", "lores", "raw") and isinstance(cfg, dict) and cfg}
            if args.inference_stream == "lores" and "lores" not in roles:
                cam.close()
                raise SystemExit(f"{spec['model']}: the stack accepted a configuration with no "
                                 f"lores role ({roles}); refusing to measure the baseline path "
                                 "under a 'lores' label")
            actual_lores = reported.get("lores", {}).get("size", [])
            if args.inference_stream == "lores" and actual_lores and tuple(actual_lores) != lores_wh:
                cam.close()
                raise SystemExit(f"{spec['model']}: asked for lores {lores_wh} but the stack built "
                                 f"{tuple(actual_lores)}; not measuring that under this label")
            if args.inference_stream == "main":
                # The bytes the baseline path really maps: main framesize, from the API itself.
                main_framesize = reported.get("main", {}).get("framesize")
                if main_framesize:
                    mapped_bytes = int(main_framesize)
                else:
                    mapped_bytes = spec["width"] * spec["height"] * 3
            else:
                mapped_bytes = int(reported.get("lores", {}).get("framesize")
                                   or lores_wh[0] * lores_wh[1] * 3)
            dur_us = int(round(1e6 / spec["fps"]))
            cam.set_controls({"FrameDurationLimits": (dur_us, dur_us)})
            canvas = np.full((1, in_h, in_w, 3), PAD_VALUE, dtype=np.uint8)
            st = CameraStats(spec["model"], number, opened, spec,
                             "isp_lores" if args.inference_stream == "lores" else "python_main",
                             lores_wh)
            st.stream_roles = {"requested": roles, "reported": reported}
            streams.append({"spec": spec, "cam": cam, "stats": st, "canvas": canvas,
                            "role": "lores" if args.inference_stream == "lores" else "main",
                            "mapped_bytes": mapped_bytes,
                            "queue": collections.deque(maxlen=args.queue_depth)})
            stats[spec["model"]] = st
        for s in streams:
            s["cam"].start()

        stop = threading.Event()
        frame_ready = threading.Event()

        def capture_loop(entry: dict[str, Any]) -> None:
            cam, st, dq, canvas, role = (entry["cam"], entry["stats"], entry["queue"],
                                         entry["canvas"], entry["role"])
            try:
                while not stop.is_set():
                    t0 = time.monotonic()
                    try:
                        request = cam.capture_request()
                    except Exception as exc:  # noqa: BLE001
                        st.errors.append(f"capture: {type(exc).__name__}: {exc}")
                        return
                    st.add("capture_wait", (time.monotonic() - t0) * 1000.0)
                    t0 = time.monotonic()
                    try:
                        metadata = request.get_metadata() or {}
                        capture_ns = int(metadata.get("SensorTimestamp", 0) or 0)
                        if role == "lores":
                            arr = request.make_array("lores")
                            arr_bytes = int(arr.nbytes)
                        else:
                            image = request.make_image("main")
                            # The main buffer's real size comes from the API's own framesize, not
                            # from width*height*3, so the copy accounting cannot drift from reality.
                            arr_bytes = int(entry["mapped_bytes"])
                    finally:
                        request.release()
                    st.add("map_into_python", (time.monotonic() - t0) * 1000.0)
                    if len(st.metadata_sample) < 3:
                        st.metadata_sample.append(
                            {key: metadata[key] for key in METADATA_KEYS_OF_INTEREST
                             if key in metadata}
                            | {"_all_keys": sorted(metadata.keys())})
                    t0 = time.monotonic()
                    if role == "lores":
                        pad_into(canvas, arr, in_w)
                        frame = canvas
                        st.add("pad_copy", (time.monotonic() - t0) * 1000.0)
                    else:
                        timings = {}
                        frame = letterbox_via_pil(image, (in_w, in_h), timings)
                        # Two separate stages, not one nested number: `resize` is the PIL
                        # downscale, `pad_copy` is canvas + paste + the NumPy handoff. A previous
                        # version recorded the whole letterbox as pad_copy, which made the two
                        # stages look additive when one was inside the other.
                        st.add("resize", timings.get("resize_ms", 0.0))
                        st.add("pad_copy", timings.get("pad_ms", 0.0))
                        st.add("letterbox_total", (time.monotonic() - t0) * 1000.0)
                    st.python_bytes_copied += arr_bytes + int(frame.nbytes)
                    t0 = time.monotonic()
                    frame = np.ascontiguousarray(frame)
                    st.add("contiguous", (time.monotonic() - t0) * 1000.0)
                    st.captured += 1
                    if len(dq) == dq.maxlen:                      # bounded: the oldest loses
                        dq.popleft()
                        st.dropped_queue += 1
                    dq.append((frame, capture_ns, time.monotonic()))
                    st.submitted += 1
                    st.queue_depth_max = max(st.queue_depth_max, len(dq))
                    frame_ready.set()
            except Exception as exc:  # noqa: BLE001
                st.errors.append(f"capture loop died: {type(exc).__name__}: {exc}")

        def feeder(infer: Any) -> None:
            turn = 0
            while not stop.is_set():
                if not frame_ready.wait(0.02):
                    continue
                frame_ready.clear()
                picked = None
                for offset in range(len(streams)):
                    entry = streams[(turn + offset) % len(streams)]
                    dq = entry["queue"]
                    while len(dq) > 1:                            # newest wins, older ones paid for
                        dq.popleft()
                        entry["stats"].dropped_stale += 1
                    if dq:
                        picked = (entry, dq.popleft())
                        turn = (turn + offset + 1) % len(streams)
                        break
                if picked is None:
                    continue
                entry, (frame, capture_ns, queued_at) = picked
                st = entry["stats"]
                t0 = time.monotonic()
                try:
                    result = infer.infer({input_infos[0].name: frame})
                except Exception as exc:  # noqa: BLE001
                    st.errors.append(f"infer: {type(exc).__name__}: {exc}")
                    stop.set()
                    return
                t1 = time.monotonic()
                payload = result.get(out_name)
                counts: dict[str, int] = {}
                if payload is not None:
                    batch = payload[0] if isinstance(payload, (list, tuple)) and payload else payload
                    try:
                        counts = _detection_counts(batch, score_threshold=SCORE_THRESHOLD)
                    except Exception as exc:  # noqa: BLE001
                        st.errors.append(f"postprocess: {type(exc).__name__}: {exc}")
                t2 = time.monotonic()
                st.add("queue_wait", (t0 - queued_at) * 1000.0)
                st.add("infer", (t1 - t0) * 1000.0)
                st.add("postprocess", (t2 - t1) * 1000.0)
                e2e_ms = ((t2 - capture_ns / 1e9) * 1000.0 if capture_ns
                          else (t2 - queued_at) * 1000.0)
                if e2e_ms < 0:
                    st.errors.append("end-to-end came out negative: camera clock and "
                                     "CLOCK_MONOTONIC disagree")
                    e2e_ms = (t2 - queued_at) * 1000.0
                st.end_to_end_ms.append(e2e_ms)
                st.inferred += 1
                if counts:
                    st.with_detection += 1
                if len(main_samples) < 8:
                    main_samples.append({"camera_id": st.model, "capture_timestamp_ns": capture_ns,
                                         "detections": counts})

        with VDevice(device_ids=Device.scan()) as device:
            group = device.configure(hef, ConfigureParams.create_from_hef(
                hef, HailoStreamInterface.PCIe))[0]
            with InferVStreams(group,
                               InputVStreamParams.make(group, quantized=True,
                                                       format_type=FormatType.UINT8),
                               OutputVStreamParams.make(group, quantized=False,
                                                        format_type=FormatType.FLOAT32)) as infer:
                with group.activate(group.create_params()):
                    stat0, core0 = read_proc_stat(), read_per_core()
                    mem0, th0 = mem_available_mb(), _soc_temp_c()
                    threads = [threading.Thread(target=capture_loop, args=(s,), daemon=True)
                               for s in streams]
                    threads.append(threading.Thread(target=feeder, args=(infer,), daemon=True))
                    for t in threads:
                        t.start()
                    deadline = time.monotonic() + args.seconds
                    while time.monotonic() < deadline:
                        time.sleep(1.0)
                    if any(t.is_alive() for t in threads) and time.monotonic() > deadline + 15.0:
                        for st in stats.values():
                            st.errors.append("watchdog: threads overran the deadline by >15 s; "
                                             "releasing the camera anyway")
                    stop.set()
                    frame_ready.set()
                    for t in threads:
                        t.join(timeout=8.0)
                    stat1, core1 = read_proc_stat(), read_per_core()
                    mem1, th1 = mem_available_mb(), _soc_temp_c()

        busy_s = sum(sum(st.samples["infer"]) for st in stats.values()) / 1000.0
        dt_total = max(stat1[1] - stat0[1], 1)
        cores = per_core_busy_percent(core0, core1)
        report = {
            "label": args.label, "seconds": args.seconds, "queue_depth": args.queue_depth,
            "inference_stream": args.inference_stream, "model_input": [in_h, in_w, 3],
            "spec": specs,
            "aggregate": {
                "inferences_total": sum(st.inferred for st in stats.values()),
                "aggregate_inference_fps": round(sum(st.inferred for st in stats.values())
                                                 / args.seconds, 2),
                "submitted_fps": round(sum(st.submitted for st in stats.values()) / args.seconds, 2),
                "captured_fps": round(sum(st.captured for st in stats.values()) / args.seconds, 2),
                "hailo_busy_fraction": round(busy_s / args.seconds, 3),
                "cpu_busy_percent": round(100.0 * (1.0 - (stat1[0] - stat0[0]) / dt_total), 2),
                "per_core_busy_percent": cores,
                "max_single_core_busy_percent": max(cores.values()) if cores else None,
                "mem_available_mb_before": mem0, "mem_available_mb_after": mem1,
                "soc_temp_c_first": th0, "soc_temp_c_last": th1,
            },
            "cameras": [st.as_dict() for st in stats.values()],
            "results_sample": main_samples,
        }
        text = json.dumps(report, indent=2, sort_keys=True)
        print(text)
        if args.out:
            args.out.write_text(text, encoding="utf-8")
        rc = 0 if all(st.inferred > 0 for st in stats.values()) else 1
        peak_core = max(cores.values()) if cores else None
        for st in stats.values():
            if not st.inferred:
                print(f"ANSWER: no -- {st.model} produced no inference: {st.errors[:2]}",
                      file=sys.stderr)
            else:
                print(f"ANSWER: {st.model} 交付 {st.inferred/args.seconds:.2f} 次推理/秒，"
                      f"端到端 p50 {(_percentile(st.end_to_end_ms,50) or 0):.1f} "
                      f"p95 {(_percentile(st.end_to_end_ms,95) or 0):.1f} "
                      f"p99 {(_percentile(st.end_to_end_ms,99) or 0):.1f} ms，"
                      f"单核峰值 {peak_core}%", file=sys.stderr)
    finally:
        for entry in streams:
            try:
                entry["cam"].close()
            except Exception as exc:  # noqa: BLE001
                entry["stats"].errors.append(f"close: {type(exc).__name__}: {exc}")
        if lock >= 0:
            os.close(lock)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
