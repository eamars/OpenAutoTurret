"""Record the tracking camera for ADR-003 stage 2 (timing calibration), beside a commissiond session.

Opens the camera exactly as visiond's Hailo profile does (perception.camera.open_picamera2_sensor,
the profile's sensor, size, rate and orientation) and keeps, for every frame:
  OUT.camera.bin    the lores leg as 8-bit grey (width*height bytes per frame, frames in order)
  OUT.camera.jsonl  sequence, SensorTimestamp, receive time, ExposureTime, FrameDuration, gains
SensorTimestamp is the stamp production uses; nothing here corrects it. Runs until SIGTERM/SIGINT
or `seconds`. It drives no motor and opens no CAN socket.

    camera_record.py OUT_STEM SECONDS [EXPOSURE_US ANALOGUE_GAIN]

With an exposure the auto-exposure is switched off for this recording only (the timing
calibration's second session, which separates the exposure term); production keeps its AE.
"""
import json
import signal
import sys
import time
from pathlib import Path

FIRMWARE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(FIRMWARE))

from perception.camera import _sensor_timestamp_ns, open_picamera2_sensor  # noqa: E402
from perception.config import VisionConfig  # noqa: E402

PROFILE = "hailo_yolov8n"  # the launcher's default perception profile (run_application.sh)
KEYS = ("ExposureTime", "FrameDuration", "AnalogueGain", "DigitalGain", "SensorTemperature")


def main():
    stem, seconds = Path(sys.argv[1]), float(sys.argv[2])
    fixed = (int(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else None
    model = VisionConfig.from_file(str(FIRMWARE / "perception/configs/perception_v1.json")).model_for(PROFILE)
    lores = (int(model.camera_lores_width), int(model.camera_lores_height))
    camera, info = open_picamera2_sensor(model.camera_model, stream_size=(int(model.camera_width), int(model.camera_height)),
                                         frame_rate_hz=float(model.camera_frame_rate_hz), orientation=model.camera_orientation,
                                         lores_size=lores)
    stop = False

    def done(_signum, _frame):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGTERM, done)
    signal.signal(signal.SIGINT, done)
    if fixed:
        camera.set_controls({"AeEnable": False, "ExposureTime": fixed[0], "AnalogueGain": fixed[1]})
    camera.start()
    end = time.monotonic() + seconds
    frames = 0
    with open(stem.with_suffix(".camera.bin"), "xb") as images, open(stem.with_suffix(".camera.jsonl"), "x") as rows:
        rows.write(json.dumps({"kind": "header", "camera": info, "lores": lores, "bytes_per_frame": lores[0] * lores[1],
                               "clock": "SensorTimestamp as delivered (CLOCK_BOOTTIME; equals CLOCK_MONOTONIC on this station)"}) + "\n")
        while not stop and time.monotonic() < end:
            request = camera.capture_request()
            try:
                receive_ns = time.monotonic_ns()
                grey = request.make_array("lores").mean(axis=2).astype("uint8")
                metadata = request.get_metadata() or {}
            finally:
                request.release()
            images.write(grey.tobytes())
            frames += 1
            rows.write(json.dumps({"kind": "frame", "sequence": frames, "sensor_ns": _sensor_timestamp_ns(metadata),
                                   "receive_ns": receive_ns, **{k: metadata.get(k) for k in KEYS}}) + "\n")
        rows.write(json.dumps({"kind": "footer", "frames": frames, "stopped": "signal" if stop else "duration"}) + "\n")
    camera.stop()
    camera.close()


if __name__ == "__main__":
    main()
