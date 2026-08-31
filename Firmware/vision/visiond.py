"""visiond — the vision daemon (architecture §5.1).

Pipeline: FrameSource (capture + SensorTimestamp + detections) ->
TargetSelector (one selected target) -> IpcPublisher (latest-value IPC).

SAFETY: this process NEVER opens CAN and NEVER drives the motor. It only
publishes timestamped target measurements. In the default ``--synthetic`` mode
it runs the whole pipeline with a synthetic camera so the system can be
exercised (and tested) without the IMX500 and without any motor involvement.
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from typing import Optional

from .frame_source import FrameSource, SyntheticFrameSource
from .ipc import IpcPublisher
from .protocol import TargetMeasurement
from .target_selector import TargetSelector, TargetSelectorConfig


class VisionDaemon:
    """Runs the vision pipeline: capture -> select -> publish."""

    def __init__(
        self,
        frame_source: FrameSource,
        target_selector: TargetSelector,
        ipc_publisher: IpcPublisher,
    ) -> None:
        self._fs = frame_source
        self._ts = target_selector
        self._ipc = ipc_publisher
        self._stop = False

    def run(self, num_frames: int = 0) -> int:
        """Run the pipeline. Returns the number of frames published.

        ``num_frames == 0`` runs until stopped (e.g. SIGINT/SIGTERM).
        """
        self._fs.start()
        self._ipc.start()
        published = 0
        try:
            while not self._stop:
                cap = self._fs.capture()
                m = self._ts.update(cap)
                self._ipc.publish(m)
                published += 1
                if 0 < num_frames <= published:
                    break
        finally:
            self._ipc.stop()
            self._fs.stop()
        return published

    def request_stop(self) -> None:
        self._stop = True


def _make_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="OpenAutoTurret vision daemon")
    p.add_argument(
        "--synthetic",
        action="store_true",
        help="run with a synthetic camera (no IMX500, no motor) [SAFE default]",
    )
    p.add_argument("--real", action="store_true", help="use the real Picamera2/IMX500 camera")
    p.add_argument("--socket", default="/tmp/ota_vision.sock", help="controld IPC socket path")
    p.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = run until stopped)")
    p.add_argument("--framerate", type=float, default=30.0, help="synthetic frame rate (Hz)")
    p.add_argument("--image-config", default="", help="path to the picamera2 config JSON (real mode)")
    p.add_argument("--detector-rpk", default="", help="path to the IMX500 detector RPK JSON (real mode)")
    return p


def main(argv: Optional[list] = None) -> int:
    args = _make_parser().parse_args(argv)

    if args.real:
        try:
            from .frame_source import Picamera2FrameSource
        except Exception as e:
            print(f"error: real camera unavailable: {e}", file=sys.stderr)
            return 1
        fs: FrameSource = Picamera2FrameSource(args.image_config, args.detector_rpk)
    else:
        # SAFE default: synthetic camera (no hardware, no motor driver).
        fs = SyntheticFrameSource(framerate_hz=args.framerate)

    ts = TargetSelector(TargetSelectorConfig())
    ipc = IpcPublisher(args.socket)

    daemon = VisionDaemon(fs, ts, ipc)

    def _handle(signum, frame):  # noqa: ARG001
        daemon.request_stop()

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    published = daemon.run(num_frames=args.frames)
    print(f"visiond: published {published} measurements on {args.socket}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
