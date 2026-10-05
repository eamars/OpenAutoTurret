"""One line per published measurement: what the aim anchor was and where it came from.

controld's tracking trace (``tracking-trace.jsonl``) records each measurement as the estimator saw
it -- pixel, optical time, the joint angles at that time, the world LOS. It cannot say *why* the
pixel moved: the anchor's source (face keypoints, shoulders, a box fraction) does not cross the
native wire. This file does, keyed by ``sensor_ns`` so the two join exactly. Station, 2026-10-05:
a 2.2 Hz yaw limit cycle in which the world estimate of a stationary person moved with the turret.

Writing happens on a listener thread with a bounded rotating file, so slow storage never delays
the native publish (the control path).
"""
from __future__ import annotations

import json
import logging
import logging.handlers
import queue
from typing import Any, Mapping, Optional

MAX_BYTES = 16 * 1024 * 1024   # two files: about 2 x 16 MB, ~2 h at 30 Hz


class ObservationTrace:
    def __init__(self, path: str, max_bytes: int = MAX_BYTES) -> None:
        self._queue: "queue.Queue[logging.LogRecord]" = queue.Queue(maxsize=4096)
        handler = logging.handlers.RotatingFileHandler(path, maxBytes=max_bytes, backupCount=1)
        handler.setFormatter(logging.Formatter("%(message)s"))
        self._listener = logging.handlers.QueueListener(self._queue, handler)
        self._logger = logging.Logger("ota.perception.observation_trace")
        self._logger.addHandler(_DroppingQueueHandler(self._queue))
        self._listener.start()

    def record(self, observation: Any, metadata: Optional[Mapping[str, Any]] = None) -> None:
        if observation is None:
            return
        anchor, box = observation.measured_anchor, observation.bbox
        row = {
            "sensor_ns": int(observation.sensor_timestamp_ns),
            "seq": int(observation.frame_sequence),
            "track": observation.track_uuid,
            "state": observation.target_state.name,
            "valid": bool(observation.measurement_valid),
            "anchor": [round(float(anchor.x), 6), round(float(anchor.y), 6)],
            "source": observation.anchor_source.name,
            "bbox": [round(float(v), 5) for v in (box.x_min, box.y_min, box.x_max, box.y_max)],
            "score": round(float(observation.detector_score), 4),
        }
        if metadata:
            for key, name in (("ExposureTime", "exposure_us"), ("AnalogueGain", "gain")):
                if key in metadata:
                    row[name] = metadata[key]
        self._logger.info(json.dumps(row, separators=(",", ":")))

    def close(self) -> None:
        self._listener.stop()


class _DroppingQueueHandler(logging.handlers.QueueHandler):
    """A full queue drops the line rather than block the frame loop."""

    def enqueue(self, record: logging.LogRecord) -> None:
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            pass
