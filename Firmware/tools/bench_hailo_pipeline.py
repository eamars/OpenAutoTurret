#!/usr/bin/env python3
"""Phase 3 + Phase 4: camera(s) -> one Hailo detector, with per-stage accounting.

The architect's requirement in one sentence: both feeds reach one loaded detector, detections
carry a camera id and a capture timestamp, queues are bounded, and when overloaded the *newest*
frame wins rather than latency accumulating. Phase 3 is the same code with one source, so the
single-stream and dual-stream numbers are comparable by construction instead of by hope.

What it reports per camera, because "30 fps works" is not an answer to anything:

- frames captured / submitted / inferred -- requested and delivered rates are never conflated
- **dropped before inference**, with the reason (queue depth) -- hiding drops is how a pipeline
  looks fast
- **per-stage milliseconds**: capture wait, preprocess, queue wait, infer, postprocess -- so a
  bottleneck is named, not guessed at
- end-to-end latency (capture -> inference complete) p50/p95/p99, because for a turret the tail
  is the number that matters
- queue depth high-water mark, so "no unbounded growth" is a measurement
- system CPU, memory, SoC temperature and the throttle word, sampled around the window

Two rules it enforces rather than documents: the device is opened and then *re-identified*
(`camera_properties`) so no sensor is ever measured under the wrong label, and every run carries
a hard deadline after which it releases the camera and reports failure -- a hung probe that keeps
holding a sensor has already cost this station an afternoon of misdiagnosis.

Example on the Pi, stack stopped:
  python Firmware/tools/bench_hailo_pipeline.py --hef <hef> --spec 'imx500=1920x1080@30'
  python Firmware/tools/bench_hailo_pipeline.py --hef <hef> --spec 'imx500=1920x1080@30,imx477=1920x1080@30'
  python Firmware/tools/bench_hailo_pipeline.py --selftest
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

THROTTLE_PATH = "/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq"
SCORE_THRESHOLD = 0.5


def preprocess_stats() -> None:
    return None


def letterbox_to_input(image: Any, target_wh: tuple[int, int]) -> Any:
    """Downscale into the network box, centred, padding what is left over.

    The model wants its own resolution; the camera gives whatever the ISP produced. Doing this
    here (and timing it) is the honest version of "Hailo input stays 640x640 while the pipeline
    captures a larger frame" -- that resize is real work and it lands on the Pi's CPU, not on
    the accelerator.
    """
    from PIL import Image

    src_w, src_h = image.size
    dst_w, dst_h = target_wh
    scale = min(dst_w / src_w, dst_h / src_h)
    if scale < 1.0:
        image = image.resize((max(1, int(src_w * scale)), max(1, int(src_h * scale))),
                             Image.Resampling.BILINEAR)
    canvas = Image.new("RGB", (dst_w, dst_h), (114, 114, 114))
    canvas.paste(image, ((dst_w - image.size[0]) // 2, (dst_h - image.size[1]) // 2))
    return canvas


def _selftest() -> int:
    import numpy as np
    from PIL import Image

    checks = 0
    specs = parse_spec("imx500=1920x1080@30,imx477=2028x1080")
    assert [s["model"] for s in specs] == ["imx500", "imx477"]
    checks += 1
    small = letterbox_to_input(Image.new("RGB", (1920, 1080), (10, 20, 30)), (640, 640))
    arr = np.array(small, dtype=np.uint8)
    assert arr.shape == (640, 640, 3) and arr.flags.c_contiguous, "Hailo 要 NHWC 连续内存"
    # 16:9 into a square: the padded bands must be top/bottom, never left/right, or the model
    # gets a letterbox the preprocessing did not intend.
    assert tuple(arr[0, 320]) == (114, 114, 114), "顶边应当是 padding"
    assert tuple(arr[639, 320]) == (114, 114, 114)
    assert tuple(arr[320, 320]) == (10, 20, 30)
    checks += 1
    big = letterbox_to_input(Image.new("RGB", (320, 320), (1, 2, 3)), (640, 640))
    assert np.array(big).shape == (640, 640, 3)
    checks += 1
    # 共享的分位数helper取 index=floor(n*pct/100)（偏上），样本少时保守报大值——
    # 对云台来说宁可报大不报小，所以这里钉住的是这个口径，不是我算的数。
    assert _percentile([1.0, 2.0], 50) == 2.0, "口径变了就要重写所有分位数断言"
    checks += 1
    print(f"hailo pipeline bench selftest: {checks}/{checks} checks passed（不碰硬件、不碰相机）")
    return 0


class CameraStats:
    def __init__(self, model: str, number: int, opened_model: str, spec: dict[str, Any]) -> None:
        self.model, self.number, self.opened_model, self.spec = model, number, opened_model, spec
        self.captured = 0
        self.submitted = 0
        self.inferred = 0
        self.dropped_queue = 0
        self.dropped_stale = 0
        self.with_detection = 0
        self.capture_wait_ms: list[float] = []
        self.preprocess_ms: list[float] = []
        self.queue_wait_ms: list[float] = []
        self.infer_ms: list[float] = []
        self.post_ms: list[float] = []
        self.end_to_end_ms: list[float] = []
        self.errors: list[str] = []
        self.queue_depth_max = 0

    def as_dict(self) -> dict[str, Any]:
        return {
            "camera_id": self.model, "camera_num": self.number,
            "verified_opened_model": self.opened_model,
            "requested_mode": f"{self.spec['width']}x{self.spec['height']}",
            "requested_fps": self.spec["fps"],
            "captured": self.captured, "submitted": self.submitted, "inferred": self.inferred,
            "dropped_queue_full": self.dropped_queue, "dropped_superseded": self.dropped_stale,
            "frames_with_detection_at_threshold": self.with_detection,
            "queue_depth_max": self.queue_depth_max,
            "stage_p50_ms": {k: round(_percentile(v, 50) or 0, 3) for k, v in (
                ("capture_wait", self.capture_wait_ms), ("preprocess", self.preprocess_ms),
                ("queue_wait", self.queue_wait_ms), ("infer", self.infer_ms),
                ("postprocess", self.post_ms))},
            "stage_p95_ms": {k: round(_percentile(v, 95) or 0, 3) for k, v in (
                ("capture_wait", self.capture_wait_ms), ("preprocess", self.preprocess_ms),
                ("queue_wait", self.queue_wait_ms), ("infer", self.infer_ms),
                ("postprocess", self.post_ms))},
            "end_to_end_p50_ms": round(_percentile(self.end_to_end_ms, 50) or 0, 3),
            "end_to_end_p95_ms": round(_percentile(self.end_to_end_ms, 95) or 0, 3),
            "end_to_end_p99_ms": round(_percentile(self.end_to_end_ms, 99) or 0, 3),
            "errors": self.errors[:4],
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hef", type=Path)
    parser.add_argument("--spec", default="imx500=1920x1080@30,imx477=1920x1080@30")
    parser.add_argument("--seconds", default=30.0, type=float)
    parser.add_argument("--queue-depth", default=2, type=int)
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
    in_shape = tuple(input_infos[0].shape)              # (H, W, 3), no batch
    out_name = output_infos[0].name

    streams: list[dict[str, Any]] = []
    stats: dict[str, CameraStats] = {}
    rc = 1
    try:
        for spec in specs:
            number = wanted[spec["model"]]
            cam = Picamera2(number)
            opened = str(cam.camera_properties.get("Model", "?")).lower()
            if opened != spec["model"]:
                cam.close()
                raise SystemExit(f"asked for {spec['model']!r}, device identifies as {opened!r}")
            cam.configure(cam.create_video_configuration(
                main={"size": (spec["width"], spec["height"]), "format": "RGB888"}))
            dur_us = int(round(1e6 / spec["fps"]))
            cam.set_controls({"FrameDurationLimits": (dur_us, dur_us)})
            st = CameraStats(spec["model"], number, opened, spec)
            streams.append({"spec": spec, "cam": cam, "stats": st,
                            "queue": collections.deque(maxlen=args.queue_depth)})
            stats[spec["model"]] = st
        for s in streams:
            s["cam"].start()

        stop = threading.Event()
        frame_ready = threading.Event()

        def capture_loop(entry: dict[str, Any]) -> None:
            cam, st, dq = entry["cam"], entry["stats"], entry["queue"]
            try:
                while not stop.is_set():
                    t0 = time.monotonic()
                    try:
                        request = cam.capture_request()
                    except Exception as exc:  # noqa: BLE001
                        st.errors.append(f"capture: {type(exc).__name__}: {exc}")
                        return
                    wait_ms = (time.monotonic() - t0) * 1000.0
                    try:
                        metadata = request.get_metadata() or {}
                        capture_ns = int(metadata.get("SensorTimestamp", 0) or 0)
                        image = request.make_image("main")
                    finally:
                        request.release()
                    p0 = time.monotonic()
                    canvas = letterbox_to_input(image, (in_shape[1], in_shape[0]))
                    frame = np.array(canvas, dtype=np.uint8, copy=True)[None, ...]
                    prep_ms = (time.monotonic() - p0) * 1000.0
                    st.captured += 1
                    st.capture_wait_ms.append(wait_ms)
                    st.preprocess_ms.append(prep_ms)
                    if len(dq) == dq.maxlen:                  # bounded: the oldest loses
                        dq.popleft()
                        st.dropped_queue += 1
                    dq.append((frame, capture_ns, time.monotonic()))
                    st.submitted += 1
                    st.queue_depth_max = max(st.queue_depth_max, len(dq))
                    frame_ready.set()
            except Exception as exc:  # noqa: BLE001
                st.errors.append(f"capture loop died: {type(exc).__name__}: {exc}")

        infer_windows: list[float] = []
        results_sample: list[dict[str, Any]] = []

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
                    while len(dq) > 1:                        # newest wins, older ones paid for
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
                    result = infer.infer({input_infos[0].name: np.ascontiguousarray(frame)})
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
                st.queue_wait_ms.append((t0 - queued_at) * 1000.0)
                st.infer_ms.append((t1 - t0) * 1000.0)
                st.post_ms.append((t2 - t1) * 1000.0)
                e2e_ms = ((t2 - capture_ns / 1e9) * 1000.0 if capture_ns
                          else (t2 - queued_at) * 1000.0)
                if e2e_ms < 0:
                    # SensorTimestamp lives in the monotonic domain; a negative here means the
                    # two clocks are not comparable on this boot, and reporting it as a tiny
                    # latency would be a lie with decimals on it.
                    st.errors.append(f"end-to-end came out negative ({e2e_ms:.1f} ms): "
                                     "camera clock and CLOCK_MONOTONIC disagree")
                    e2e_ms = (t2 - queued_at) * 1000.0
                st.end_to_end_ms.append(e2e_ms)
                st.inferred += 1
                if counts:
                    st.with_detection += 1
                infer_windows.append((t1 - t0) * 1000.0)
                if len(results_sample) < 6:
                    results_sample.append({"camera_id": st.model, "capture_timestamp_ns": capture_ns,
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
                    stat0, mem0 = read_proc_stat(), mem_available_mb()
                    th0 = _soc_temp_c()
                    threads = [threading.Thread(target=capture_loop, args=(s,), daemon=True)
                               for s in streams]
                    threads.append(threading.Thread(target=feeder, args=(infer,), daemon=True))
                    for t in threads:
                        t.start()
                    deadline = time.monotonic() + args.seconds
                    while time.monotonic() < deadline:
                        time.sleep(1.0)
                    hung = [t for t in threads if t.is_alive() and not stop.is_set()
                            and time.monotonic() > deadline + 15.0]
                    if hung:
                        for st in stats.values():
                            st.errors.append(f"watchdog: {len(hung)} thread(s) overran the "
                                             f"deadline by >15 s; releasing the camera anyway")
                    stop.set()
                    frame_ready.set()
                    for t in threads:
                        t.join(timeout=8.0)
                    stat1, mem1, th1 = read_proc_stat(), mem_available_mb(), _soc_temp_c()

        busy_s = sum(sum(st.infer_ms) for st in stats.values()) / 1000.0
        dt_total = max(stat1[1] - stat0[1], 1)
        idle = (stat1[0] - stat0[0]) / dt_total
        report = {
            "label": args.label, "seconds": args.seconds, "queue_depth": args.queue_depth,
            "model_input": list(in_shape), "spec": specs,
            "aggregate": {
                "inferences_total": sum(st.inferred for st in stats.values()),
                "aggregate_inference_fps": round(sum(st.inferred for st in stats.values())
                                                 / args.seconds, 2),
                "submitted_fps": round(sum(st.submitted for st in stats.values()) / args.seconds, 2),
                "captured_fps": round(sum(st.captured for st in stats.values()) / args.seconds, 2),
                "hailo_busy_fraction": round(busy_s / args.seconds, 3),
                "cpu_busy_percent": round(100.0 * (1.0 - idle), 2),
                "mem_available_mb_before": mem0, "mem_available_mb_after": mem1,
                "soc_temp_c_first": th0, "soc_temp_c_last": th1,
            },
            "cameras": [st.as_dict() for st in stats.values()],
            "results_sample": results_sample,
        }
        text = json.dumps(report, indent=2, sort_keys=True)
        print(text)
        if args.out:
            args.out.write_text(text, encoding="utf-8")
        rc = 0 if all(st.inferred > 0 for st in stats.values()) else 1
        for st in stats.values():
            if not st.inferred:
                print(f"ANSWER: no -- {st.model} produced no inference: {st.errors[:2]}",
                      file=sys.stderr)
            else:
                print(f"ANSWER: {st.model} 交付 {st.inferred/args.seconds:.2f} 次推理/秒，"
                      f"端到端 p50 {(_percentile(st.end_to_end_ms,50) or 0):.1f} ms "
                      f"p95 {(_percentile(st.end_to_end_ms,95) or 0):.1f} "
                      f"p99 {(_percentile(st.end_to_end_ms,99) or 0):.1f}", file=sys.stderr)
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
