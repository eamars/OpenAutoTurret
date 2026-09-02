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
    p.add_argument(
        "--detector",
        default="auto",
        choices=["auto", "rpk", "simple", "none"],
        help=(
            "detection backend for --real. rpk = the IMX500 AI stack (production "
            "intent, §10.1; degrades to no detections where the platform has no "
            "AI API). simple = the classical bridge detector for P8 bring-up "
            "(reports the largest moving blob; NOT a production detector). "
            "none = stream frames only (measurements stay valid=false). "
            "auto = rpk when --detector-rpk is given, else none."
        ),
    )
    p.add_argument("--orientation", default="none", help="install orientation applied to the detector boxes (none|rotate_180|flip_horizontal|flip_vertical); keeps control geometry correct when the camera is mounted upside-down")
    return p


def main(argv: Optional[list] = None) -> int:
    args = _make_parser().parse_args(argv)

    detector_choice = args.detector
    if detector_choice == "auto":
        detector_choice = "rpk" if (args.real and args.detector_rpk) else "none"

    fs: FrameSource
    bridge = None
    if args.real:
        try:
            from .frame_source import Picamera2FrameSource
        except Exception as e:
            print(f"error: real camera unavailable: {e}", file=sys.stderr)
            return 1
        fs = Picamera2FrameSource(
            args.image_config,
            args.detector_rpk if detector_choice == "rpk" else "",
            orientation=args.orientation,
            framerate_hz=args.framerate,
        )
        if detector_choice == "rpk" and not args.detector_rpk:
            print("warning: --detector rpk but no --detector-rpk given: the "
                  "camera will stream WITHOUT detections (measurements stay "
                  "valid=false)", file=sys.stderr)
        if detector_choice == "simple":
            from .frame_source import DetectedFrameSource
            from .simple_detector import MotionBlobDetector

            bridge = DetectedFrameSource(fs, MotionBlobDetector())
            fs = bridge
            print(
                "WARNING: --detector simple is the P8 BRING-UP bridge detector "
                "(largest moving blob, see vision/simple_detector.py). It is "
                "NOT the §10.1 detector: it has no notion of a target CLASS. "
                "Swap in the RPK detector for any production use.",
                file=sys.stderr,
            )
    else:
        # SAFE default: synthetic camera (no hardware, no motor driver). Its
        # detections are SCRIPTED, so no detector backend applies — say so
        # rather than quietly pretending the choice took effect.
        if args.detector != "auto" and args.detector != "none":
            print("note: --detector is ignored in --synthetic mode (the "
                  "synthetic source publishes scripted detections)",
                  file=sys.stderr)
        fs = SyntheticFrameSource(framerate_hz=args.framerate)

    ts = TargetSelector(TargetSelectorConfig())
    ipc = IpcPublisher(args.socket)

    daemon = VisionDaemon(fs, ts, ipc)

    def _handle(signum, frame):  # noqa: ARG001
        daemon.request_stop()

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    published = daemon.run(num_frames=args.frames)
    extra = ""
    if bridge is not None:
        extra = f", bridge blobs {bridge.blobs}/{bridge.frames} frames"
    print(f"visiond: published {published} measurements on {args.socket}{extra}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
