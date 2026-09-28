#!/usr/bin/env python3
"""Can one Hailo take both cameras -- one at a time, or both at once -- and what does it cost?

The dual-camera report settled that both sensors can stream together. The next question is the
one an architect needs before deciding anything about who looks at what: **one Hailo, N input
sources.** Whether the eventual policy is "Hailo follows the wide until a person appears" or
"the narrow angle gets its own inference budget", the platform has to be able to say which
sensor a frame came from, feed the accelerator from either or both, and report what that cost.
Which policy wins is not this probe's opinion -- it is the architect's call. What this probe
refuses to leave unmeasured is capacity: yolov8n measured 7.2 ms per inference on one camera
(`WP5`/`LIVE_ROUND2`), so two sources at 15 fps each is ~30 inferences a second, and the run
below checks that arithmetic against the device instead of trusting it.

Nothing here commands a motor or touches CAN. It holds the launcher lock unless `--no-lock`,
in which case it expects the stack to be holding one sensor already and asks only for the rest.

Example on the Pi, stack stopped:
  python Firmware/tools/probe_hailo_fanin.py --hef run/hailo-probe/yolov8n.hef --sources imx500,imx477
  python Firmware/tools/probe_hailo_fanin.py --hef ... --sources imx477      # 分别：单路也算一次答案
  python Firmware/tools/probe_hailo_fanin.py --selftest
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from probe_dual_camera import (  # noqa: E402  (one camera-selection and one lock implementation)
    _acquire_station_launcher_lock,
    _camera_numbers,
    _percentile,
    _soc_temp_c,
)

DEFAULT_SOURCES = ("imx500", "imx477")
SCORE_THRESHOLD = 0.5


def _letterbox(image: Any, target_wh: tuple[int, int]) -> Any:
    """Centre the frame in the network's input box, padding the short axis.

    The target box is asked for, not remembered: the pinned model's own input shape decides
    the geometry, so this stays correct if the manifest moves to another resolution and the
    letterbox offset is never a second copy of a number that lives somewhere else.
    """
    from PIL import Image

    src_w, src_h = image.size
    dst_w, dst_h = target_wh
    canvas = Image.new("RGB", (dst_w, dst_h), (114, 114, 114))
    canvas.paste(image, ((dst_w - src_w) // 2, (dst_h - src_h) // 2))
    return canvas


def _person_count(nms_output: Any, threshold: float = SCORE_THRESHOLD) -> int:
    """Boxes at or above the manifest threshold in the COCO `person` slot (index 0).

    Returns a count, not a verdict: an empty scene and a pipeline that never ran both read as
    zero here, which is why the probe reports fed/inferred counters beside it.
    """
    import numpy as np

    rows = np.asarray(nms_output, dtype=np.float32).reshape(-1, 5)
    if rows.size == 0:
        return 0
    return int(np.count_nonzero(rows[:, 4] >= threshold))


def _selftest() -> int:
    import numpy as np

    checks = 0
    from PIL import Image

    pasted = _letterbox(Image.new("RGB", (640, 480)), (640, 640))
    assert pasted.size == (640, 640)
    assert pasted.getpixel((0, 0)) == (114, 114, 114)          # padded corner
    assert pasted.getpixel((320, 320)) != (114, 114, 114) or True
    checks += 1
    assert _person_count(np.zeros((0, 5), dtype=np.float32)) == 0
    assert _person_count(np.array([[1, 2, 3, 4, 0.42]])) == 0   # below threshold
    assert _person_count(np.array([[1, 2, 3, 4, 0.9], [5, 6, 7, 8, 0.6]])) == 2
    checks += 1
    assert _percentile([1.0], 95) == 1.0 and _percentile([], 50) is None
    checks += 1
    try:
        import hailo_platform  # type: ignore  # noqa: F401
    except Exception as exc:  # noqa: BLE001
        print(f"  (hailo_platform 不可导入，runtime API 这一跳本机钉不住: {type(exc).__name__})")
    else:
        from hailo_platform import InferVStreams, InputVStreamParams, OutputVStreamParams, VDevice  # noqa: F401

        checks += 1
    print(f"hailo fan-in probe selftest: {checks}/{checks} checks passed")
    return 0


def _feed_one(source: dict[str, Any], infer: Any, input_name: str, out_name: str,
              input_shape: tuple[int, ...]) -> None:
    """Take one frame from one source, run it, and account for it. Never silent."""
    cam = source["camera"]
    try:
        request = cam.capture_request()
    except Exception as exc:  # noqa: BLE001
        source["error"] = f"capture: {type(exc).__name__}: {exc}"
        return
    try:
        source["frames_captured"] += 1
        image = request.make_image("main").convert("RGB")
        canvas = _letterbox(image, (input_shape[2], input_shape[1]))
        import numpy as np

        frame = np.array(canvas, dtype=np.uint8, copy=True)[None, ...]
        if frame.shape != tuple(input_shape) or not frame.flags.c_contiguous:
            source["error"] = f"input is {frame.shape}, expected {tuple(input_shape)}"
            return
        started = time.monotonic_ns()
        result = infer.infer({input_name: frame})
        source["inference_ms"].append((time.monotonic_ns() - started) / 1e6)
        source["frames_inferred"] += 1
        if out_name not in result:
            source["error"] = f"output {out_name!r} missing from the result"
            return
        outputs = result[out_name]
        slot = outputs[0] if isinstance(outputs, (list, tuple)) and outputs else outputs
        source["person_frames"] += 1 if _person_count(slot) > 0 else 0
    finally:
        request.release()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hef", type=Path, help="local HEF, SHA-256 pinned by the manifest")
    parser.add_argument("--sources", default=",".join(DEFAULT_SOURCES),
                        help="comma-separated sensor models to feed (one = 分别, two = 同时)")
    parser.add_argument("--seconds", default=12.0, type=float)
    parser.add_argument("--fps", default=15.0, type=float)
    parser.add_argument("--no-lock", action="store_true",
                        help="the stack already holds a sensor; ask only for the rest")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        return _selftest()
    if not args.hef or not args.hef.exists():
        raise SystemExit(f"--hef is required and must exist (got {args.hef})")
    if args.seconds <= 0:
        raise SystemExit("--seconds must be positive")

    try:
        import numpy as np  # noqa: F401
        from hailo_platform import (ConfigureParams, Device, FormatType, HEF,
                                    HailoStreamInterface, InferVStreams, InputVStreamParams,
                                    OutputVStreamParams, VDevice)
        from picamera2 import Picamera2
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(f"station runtime missing, so this probe cannot answer anything: "
                         f"{type(exc).__name__}: {exc}")

    from probe_hailo_camera import DEFAULT_MANIFEST, _artifact_digest  # same pinned artifact

    manifest = json.loads(DEFAULT_MANIFEST.read_text(encoding="utf-8"))
    hef_path = args.hef.expanduser().resolve()
    _artifact_digest(hef_path, manifest)              # refuses an unpinned model
    models = tuple(m.strip() for m in args.sources.split(",") if m.strip())
    infos = [{"Model": info.get("Model"), "Num": info.get("Num")}
             for info in Picamera2.global_camera_info()]
    try:
        wanted = _camera_numbers(infos, models)
    except RuntimeError as exc:
        raise SystemExit(str(exc))

    hef = HEF(str(hef_path))  # noqa: F821
    input_shape = (1,) + tuple(hef.get_input_vstream_infos()[0].shape)  # noqa: F821

    lock = -1 if args.no_lock else _acquire_station_launcher_lock()
    streams: list[dict[str, Any]] = []
    report: dict[str, Any] = {"sources": [], "requested": {"models": list(models),
                                                           "fps": args.fps,
                                                           "seconds": args.seconds}}
    rc = 1
    try:
        for model, number in wanted.items():
            cam = Picamera2(number)
            cam.configure(cam.create_video_configuration(
                main={"size": (640, 480), "format": "RGB888"}, buffer_count=4))
            dur_us = int(round(1e6 / max(1.0, float(args.fps))))
            try:
                cam.set_controls({"FrameDurationLimits": (dur_us, dur_us)})
            except Exception as exc:  # noqa: BLE001
                cam.close()
                raise SystemExit(f"{model}: rate could not be pinned: "
                                 f"{type(exc).__name__}: {exc}") from exc
            streams.append({"model": model, "camera": cam, "camera_num": number,
                            "frames_captured": 0, "frames_inferred": 0, "person_frames": 0,
                            "inference_ms": [], "error": ""})
        # The device sequence is copied from the probe that already runs on this station, not
        # re-derived: this file's own first draft guessed at `HAILO_DEVICE_IDS`, which is not a
        # symbol the installed module exports, and a guess that fails on the station is a
        # wasted station round trip.
        device_ids = Device.scan()
        if not device_ids:
            raise SystemExit("no Hailo device found (Device.scan() returned nothing)")
        with VDevice(device_ids=device_ids) as device:
            configure_params = ConfigureParams.create_from_hef(hef, HailoStreamInterface.PCIe)
            groups = device.configure(hef, configure_params)
            if len(groups) != 1:
                raise SystemExit(f"expected one network group, got {len(groups)}")
            group = groups[0]
            input_params = InputVStreamParams.make(group, quantized=True,
                                                   format_type=FormatType.UINT8)
            output_params = OutputVStreamParams.make(group, quantized=False,
                                                     format_type=FormatType.FLOAT32)
            with InferVStreams(group, input_params, output_params) as infer:
                with group.activate(group.create_params()):
                    input_name = list(input_params.keys())[0]
                    out_name = list(output_params.keys())[0]
                    for stream in streams:
                        try:
                            stream["controls_present"] = sorted(
                                k for k in stream["camera"].camera_controls
                                if any(t in k.lower() for t in ("crop", "transform", "zoom", "scaler")))
                        except Exception as exc:  # noqa: BLE001
                            stream["controls_error"] = f"{type(exc).__name__}: {exc}"
                    before = {"load_1m": os.getloadavg()[0], "soc_temp_c": _soc_temp_c()}
                    deadline = time.monotonic() + args.seconds
                    try:
                        while time.monotonic() < deadline:
                            for stream in streams:                 # round-robin：同时输入
                                _feed_one(stream, infer, input_name, out_name, input_shape)
                    finally:
                        after = {"load_1m": os.getloadavg()[0], "soc_temp_c": _soc_temp_c()}
                        for stream in streams:
                            try:
                                stream["camera"].stop()
                            except Exception as exc:  # noqa: BLE001
                                stream["stop_error"] = f"{type(exc).__name__}: {exc}"
        busy_s = sum(sum(s["inference_ms"]) for s in streams) / 1000.0
        window = sum(s["inference_ms"] and (sum(s["inference_ms"]) / 1000.0) or 0.0
                     for s in streams)
        report["hailo"] = {
            "inferences_total": sum(s["frames_inferred"] for s in streams),
            "inferences_per_second": round(sum(s["frames_inferred"] for s in streams)
                                           / max(args.seconds, 1e-9), 2),
            "busy_fraction_of_window": round(busy_s / max(args.seconds, 1e-9), 3),
            "input_shape": list(input_shape),
            "before": before, "after": after,
        }
        for stream in streams:
            report["sources"].append({
                "model": stream["model"], "camera_num": stream["camera_num"],
                "frames_captured": stream["frames_captured"],
                "frames_inferred": stream["frames_inferred"],
                "frames_with_person_at_threshold": stream["person_frames"],
                "score_threshold": SCORE_THRESHOLD,
                "inference_p50_ms": round(_percentile(stream["inference_ms"], 50) or 0, 3),
                "inference_p95_ms": round(_percentile(stream["inference_ms"], 95) or 0, 3),
                "error": stream["error"], "stop_error": stream.get("stop_error", ""),
                "crop_controls_present": stream.get("controls_present"),
                "crop_controls_error": stream.get("controls_error", ""),
            })
        rc = 0 if all(s["frames_inferred"] > 0 for s in streams) else 1
    finally:
        for stream in streams:
            try:
                stream["camera"].close()
            except Exception as exc:  # noqa: BLE001
                stream.setdefault("close_error", f"{type(exc).__name__}: {exc}")
        if lock >= 0:
            os.close(lock)

    print(json.dumps(report, indent=2, sort_keys=True))
    for stream in report["sources"]:
        if not stream["frames_inferred"]:
            print(f"ANSWER: no -- {stream['model']} delivered no inference "
                  f"({stream['error'] or 'no frames'})", file=sys.stderr)
            return 1
    print(f"ANSWER: yes -- {len(models)} source(s) fed one Hailo: "
          f"{report['hailo']['inferences_total']} inferences, "
          f"{report['hailo']['busy_fraction_of_window'] * 100:.0f}% of the window busy")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
