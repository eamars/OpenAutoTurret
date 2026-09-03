"""visiond — the vision daemon (architecture §5.1).

v3 pipeline (§59): FrameSource (capture + SensorTimestamp + detections) ->
TrackManager (multi-object tracks, §8/§9/§10) -> IpcPublisher (one TrackSet per frame).

v1 stopped at TargetSelector, which scored every detection and published the single
best one — so "which target" was decided in visiond. §59 moves that decision to
controld ("controld remains authoritative for selected target"), because the operator
has to be able to choose a specific human being and have that choice survive a dropout,
a mode change, and the detector momentarily preferring somebody else. TargetSelector is
no longer wired in; it stays in the tree, unused, until V3-3 (TargetSelectionManager)
lands and its heuristics can be judged against something real rather than deleted from
memory.

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

from common import image_corrections as ic
from .frame_source import FrameSource, SyntheticFrameSource
from .ipc import IpcPublisher
from .track_manager import TrackManager, detections_to_tracks


class VisionDaemon:
    """Runs the vision pipeline: capture -> select -> publish."""

    def __init__(
        self,
        frame_source: FrameSource,
        track_manager: TrackManager,
        ipc_publisher: IpcPublisher,
        connect_timeout_s: float = 0.0,
    ) -> None:
        self._fs = frame_source
        self._tm = track_manager
        self._ipc = ipc_publisher
        self._connect_timeout_s = float(connect_timeout_s)
        self._stop = False

    def run(self, num_frames: int = 0) -> int:
        """Run the pipeline. Returns the number of frames published.

        ``num_frames == 0`` runs until stopped (e.g. SIGINT/SIGTERM).
        """
        self._fs.start()
        # controld binds the socket, so waiting for it is normal at service
        # start; Ctrl-C / SIGTERM must still be able to abort the wait.
        self._ipc.start(timeout_s=self._connect_timeout_s,
                        should_stop=lambda: self._stop)
        published = 0
        try:
            while not self._stop:
                cap = self._fs.capture()
                # §58: association runs once per detector frame, here, at camera rate —
                # never at the 200 Hz control rate. publish_timestamp_ns is stamped
                # after the work and before the send, so §61's publish-to-receive
                # interval measures the transport, not the inference that produced it.
                now_ns = time.monotonic_ns()
                self._tm.update(detections_to_tracks(cap, now_ns), now_ns)
                self._ipc.publish(self._tm.build_track_set(
                    cap.frame_sequence, cap.sensor_timestamp_ns, now_ns,
                    cap.width, cap.height))
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
    p.add_argument(
        "--connect-timeout-s",
        type=float,
        default=0.0,
        help=(
            "seconds to wait for controld's socket at start (0 = wait forever, "
            "the service default: controld BINDS the socket, so a systemd start "
            "can legitimately race it). Scripts that want a fast failure should "
            "set e.g. 5."
        ),
    )
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
    p.add_argument(
        "--orientation",
        default="none",
        choices=list(ic.ORIENTATIONS),
        help=(
            "install orientation applied to the frame AND the detector boxes "
            "(keeps control geometry correct when the camera is mounted "
            "upside-down; this station's IMX500 needs rotate_180). Choices are "
            "enforced here so a typo in a systemd unit fails with a usage "
            "message in the journal instead of a traceback."
        ),
    )
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

    tm = TrackManager()
    ipc = IpcPublisher(args.socket)

    daemon = VisionDaemon(fs, tm, ipc, connect_timeout_s=args.connect_timeout_s)

    def _handle(signum, frame):  # noqa: ARG001
        daemon.request_stop()

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    published = 0
    try:
        published = daemon.run(num_frames=args.frames)
    except OSError as e:
        # Almost always: controld is not up (or the socket paths disagree). One
        # clear line beats a traceback in the journal; the unit's Restart= handles
        # the retry.
        print(f"error: cannot publish to {args.socket}: {e}", file=sys.stderr)
        print("hint: controld binds this socket (§6.1). Is it running, and does "
              "its vision.socket_path / OTA_VISION_SOCKET match --socket?",
              file=sys.stderr)
        return 1
    extra = ""
    if bridge is not None:
        extra = f", bridge blobs {bridge.blobs}/{bridge.frames} frames"
    print(f"visiond: published {published} TrackSets (§59) on {args.socket}{extra}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
