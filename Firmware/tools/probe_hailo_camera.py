#!/usr/bin/env python3
"""Bounded, no-motion Hailo-8 + Picamera2 inference probe.

This opens exactly one explicitly selected camera and the pinned local HEF. It
does not save frames, start station services, access CAN, or command motors.
Run under the station project virtualenv. It takes the same launcher lock as
``run_application.sh`` for the configured ``OTA_RUN_DIR`` so a station stack
cannot start while this process owns a camera. The motion lock is deliberately
not used: perception-only launcher runs own a camera without taking that
motor-ownership lock.

Example on the Pi after stopping the stack through its launcher:
  python Firmware/tools/probe_hailo_camera.py --hef /path/to/yolov8n.hef
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = ROOT / "config" / "hailo_yolov8n_manifest.json"


def _positive_frames(value: str) -> int:
    try:
        frames = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer from 1 to 300") from exc
    if not 1 <= frames <= 300:
        raise argparse.ArgumentTypeError("must be from 1 to 300")
    return frames


def _percentile(values: list[float], percentile: float) -> float:
    # statistics.quantiles needs multiple samples and interpolates differently;
    # NumPy is already required by pyHailoRT, so use its conventional percentile.
    import numpy as np

    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile))


def _acquire_station_launcher_lock() -> int:
    """Acquire run_application.sh's stack lock before touching the camera."""
    uid = os.getuid()
    run_dir = Path(os.environ.get("OTA_RUN_DIR", f"/tmp/ota-stack-{uid}"))
    run_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    lock_path = run_dir / "launcher.lock"
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        os.close(fd)
        raise RuntimeError(
            f"station launcher lock is held ({lock_path}); stop/check the stack "
            "through Firmware/scripts/run_application.sh before probing"
        ) from exc
    return fd


def _artifact_digest(hef_path: Path, manifest: dict[str, Any]) -> str:
    expected = manifest["artifact"]["sha256"].lower()
    digest = hashlib.sha256()
    with hef_path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    actual = digest.hexdigest()
    if actual != expected:
        raise ValueError(
            f"HEF SHA-256 mismatch: expected {expected}, got {actual}; "
            "refusing an unpinned model"
        )
    return actual


def _camera_number(Picamera2: Any, selected_model: str) -> tuple[int, dict[str, Any]]:
    """Match one camera from enumeration without opening any camera first."""
    matches = []
    for info in Picamera2.global_camera_info():
        model = str(info.get("Model", info.get("model", ""))).lower()
        if selected_model.lower() in model:
            matches.append(info)
    if len(matches) != 1:
        visible = [
            {"Num": item.get("Num"), "Model": item.get("Model", item.get("model"))}
            for item in Picamera2.global_camera_info()
        ]
        raise RuntimeError(
            f"expected exactly one {selected_model} camera, found {len(matches)}; "
            f"enumerated cameras: {visible}"
        )
    info = matches[0]
    number = info.get("Num", info.get("num"))
    if number is None:
        raise RuntimeError(f"camera enumeration lacks a numeric index: {info}")
    return int(number), info


def _sensor_timestamp_ns(metadata: Any) -> int:
    if not isinstance(metadata, dict):
        return 0
    value = metadata.get("SensorTimestamp")
    if isinstance(value, (tuple, list)):
        value = value[0] if value else None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return 0
    stamp = int(value)
    return stamp if stamp > 0 else 0


def _detection_counts(output: Any, score_threshold: float) -> dict[str, int]:
    """Validate Hailo's class-grouped NMS payload and count thresholded boxes."""
    import numpy as np

    if not isinstance(output, (list, tuple)) or len(output) != 80:
        raise RuntimeError("expected Hailo NMS output as a list of 80 class arrays")
    counts: dict[str, int] = {}
    for class_index, class_rows in enumerate(output):
        rows = np.asarray(class_rows)
        if rows.size == 0:
            continue
        if rows.ndim != 2 or rows.shape[1] != 5:
            raise RuntimeError(
                f"NMS class slot {class_index} has shape {rows.shape}, expected [N,5]"
            )
        if not np.isfinite(rows).all():
            raise RuntimeError(f"NMS class slot {class_index} contains non-finite values")
        scores = rows[:, 4]
        if np.any((scores < 0.0) | (scores > 1.0)):
            raise RuntimeError(f"NMS class slot {class_index} has scores outside [0,1]")
        count = int(np.count_nonzero(scores >= score_threshold))
        if count:
            counts[str(class_index)] = count
    return counts


def _run(args: argparse.Namespace) -> dict[str, Any]:
    manifest = json.loads(DEFAULT_MANIFEST.read_text(encoding="utf-8"))
    hef_path = args.hef.expanduser().resolve(strict=True)
    if not hef_path.is_file():
        raise ValueError(f"HEF is not a regular file: {hef_path}")
    digest = _artifact_digest(hef_path, manifest)

    # Reject another station process before importing/opening the camera.
    lock_fd = _acquire_station_launcher_lock()
    camera = None
    camera_started = False
    try:
        import numpy as np
        from PIL import Image
        from picamera2 import Picamera2
        from hailo_platform import (
            ConfigureParams,
            FormatType,
            HEF,
            HailoStreamInterface,
            InferVStreams,
            InputVStreamParams,
            OutputVStreamParams,
            Device,
            VDevice,
        )

        hef = HEF(str(hef_path))
        input_infos = hef.get_input_vstream_infos()
        output_infos = hef.get_output_vstream_infos()
        if len(input_infos) != 1 or len(output_infos) != 1:
            raise RuntimeError(
                f"expected one input and one output vstream; got "
                f"{len(input_infos)} input(s), {len(output_infos)} output(s)"
            )
        input_info, output_info = input_infos[0], output_infos[0]
        if tuple(input_info.shape) != (640, 640, 3):
            raise RuntimeError(f"HEF input shape is {input_info.shape}, expected 640x640x3")

        # Identify the physical card read-only and bind this run to that device.
        # Do not report the manifest's architecture as if it were a live reading.
        device_ids = Device.scan()
        if len(device_ids) != 1:
            raise RuntimeError(f"expected exactly one Hailo device, found {len(device_ids)}: {device_ids}")
        with Device(device_ids[0]) as physical_device:
            board_info = physical_device.control.identify()
            actual_architecture = str(board_info.device_architecture)
        if actual_architecture != manifest["artifact"]["architecture"]:
            raise RuntimeError(
                f"HEF manifest targets {manifest['artifact']['architecture']}, "
                f"but device identifies as {actual_architecture}"
            )

        camera_number, camera_info = _camera_number(Picamera2, args.camera_model)
        configure_params = ConfigureParams.create_from_hef(
            hef, HailoStreamInterface.PCIe
        )
        with VDevice(device_ids=device_ids) as device:
            network_groups = device.configure(hef, configure_params)
            if len(network_groups) != 1:
                raise RuntimeError(f"expected one configured network group, got {len(network_groups)}")
            network_group = network_groups[0]
            input_params = InputVStreamParams.make(
                network_group,
                quantized=True,
                format_type=FormatType.UINT8,
            )
            output_params = OutputVStreamParams.make(
                network_group,
                quantized=False,
                format_type=FormatType.FLOAT32,
            )

            camera = Picamera2(camera_number)
            configuration = camera.create_video_configuration(
                main={"size": (640, 480), "format": "RGB888"},
                controls={"FrameRate": 15.0},
                buffer_count=6,
            )
            camera.configure(configuration)
            camera.start()
            camera_started = True

            inference_ms: list[float] = []
            sensor_to_result_ms: list[float] = []
            previous_sensor_ns = 0
            frame_class_counts: list[dict[str, int]] = []
            all_started_ns = time.monotonic_ns()
            with InferVStreams(network_group, input_params, output_params) as infer:
                with network_group.activate(network_group.create_params()):
                    for _ in range(args.frames):
                        request = camera.capture_request()
                        try:
                            metadata = request.get_metadata()
                            sensor_ns = _sensor_timestamp_ns(metadata)
                            if sensor_ns == 0:
                                raise RuntimeError("camera frame has no valid SensorTimestamp")
                            if previous_sensor_ns and sensor_ns <= previous_sensor_ns:
                                raise RuntimeError("camera SensorTimestamp did not increase")
                            previous_sensor_ns = sensor_ns

                            image = request.make_image("main").convert("RGB")
                            if image.size != (640, 480):
                                raise RuntimeError(f"camera returned {image.size}, expected (640,480)")
                            canvas = Image.new("RGB", (640, 640), (114, 114, 114))
                            canvas.paste(image, (0, 80))
                            frame = np.array(canvas, dtype=np.uint8, copy=True)[None, ...]
                            if frame.shape != (1, 640, 640, 3) or not frame.flags.c_contiguous:
                                raise RuntimeError("preprocessed input is not contiguous NHWC 640x640x3")

                            started_ns = time.monotonic_ns()
                            result = infer.infer({input_info.name: frame})
                            completed_ns = time.monotonic_ns()
                            inference_ms.append((completed_ns - started_ns) / 1e6)
                            end_to_end_ms = (completed_ns - sensor_ns) / 1e6
                            if end_to_end_ms < 0:
                                raise RuntimeError(
                                    "SensorTimestamp and monotonic clock appear to use different domains"
                                )
                            sensor_to_result_ms.append(end_to_end_ms)
                            if output_info.name not in result:
                                raise RuntimeError(f"inference output missing {output_info.name!r}")
                            batch_output = result[output_info.name]
                            if not isinstance(batch_output, (list, tuple)) or len(batch_output) != 1:
                                raise RuntimeError("expected one NMS output for batch size 1")
                            frame_class_counts.append(
                                _detection_counts(batch_output[0], score_threshold=0.5)
                            )
                        finally:
                            request.release()
            ended_ns = time.monotonic_ns()
            if camera_started:
                camera.stop()
                camera_started = False
            camera.close()
            camera = None

        total_counts: dict[str, int] = {}
        for counts in frame_class_counts:
            for class_index, count in counts.items():
                total_counts[class_index] = total_counts.get(class_index, 0) + count

        def stats(samples: list[float]) -> dict[str, float]:
            return {
                "p50_ms": round(_percentile(samples, 50), 3),
                "p95_ms": round(_percentile(samples, 95), 3),
            }

        elapsed_s = (ended_ns - all_started_ns) / 1e9
        return {
            "status": "PASS",
            "scope": "one selected real camera + pinned HEF; no motion/CAN; frames not saved",
            "model_id": manifest["model_id"],
            "hef": hef_path.name,
            "sha256": digest,
            "device_architecture": actual_architecture,
            "hailo_device_id": device_ids[0],
            "hailort_api_qualified_version": manifest["runtime"]["hailort_version"],
            "camera": {
                "selected_model": args.camera_model,
                "camera_num": camera_number,
                "enumerated_model": camera_info.get("Model", camera_info.get("model")),
                "stream": "640x480 RGB888 at requested 15 fps",
            },
            "frames_inferred": len(inference_ms),
            "elapsed_s": round(elapsed_s, 3),
            "probe_rate_fps": round(len(inference_ms) / elapsed_s, 3) if elapsed_s > 0 else 0.0,
            "inference_latency": stats(inference_ms),
            "sensor_to_result_latency": stats(sensor_to_result_ms),
            "score_threshold": 0.5,
            "detections_per_class_slot_at_or_above_threshold": total_counts,
            "semantic_accuracy_tested": False,
        }
    finally:
        if camera is not None:
            if camera_started:
                try:
                    camera.stop()
                except Exception:
                    pass
            try:
                camera.close()
            except Exception:
                pass
        os.close(lock_fd)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hef", required=True, type=Path, help="local HEF, SHA-256 pinned by manifest")
    parser.add_argument("--frames", default=30, type=_positive_frames,
                        help="bounded capture/inference count (1..300, default 30)")
    parser.add_argument("--camera-model", default="imx477", choices=("imx477", "imx500"),
                        help="select exactly one camera by enumerated sensor model")
    args = parser.parse_args(argv)
    try:
        report = _run(args)
    except Exception as exc:  # noqa: BLE001 - probe reports dependency/device failures as concise JSON
        print(json.dumps({"status": "FAIL", "error": str(exc)}, indent=2), file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
