"""`fps` in the video state is a request. `fps_published` is a measurement. They must stay apart.

Measured on this station on 2026-09-06, because the panel said one thing and the machine another:

* publish rate was **exactly 200 frames in 20.0 s** on two independent samples — 9.99 fps idle and
  10.00 fps with a client attached draining ~1 MiB/s — so neither the network nor a slow reader was
  involved, and `frames in 20.0s` twice is what a frame-locked sensor looks like, not a timer;
* at **1280x720** it published **121 frames in 12 s ≈ 10.1 fps**, so the ceiling was not the
  requested resolution either;
* yet `/api/video/state` reported `fps: 15.00`, and `OTA_VIDEO_FPS=15` only sets a *publish cap*
  (`_min_publish_s`), while `create_video_configuration(...)` is handed a size and **no frame rate at
  all**. The 15 was never asked of the camera, and never measured either.

A §20 field named `fps` that reports an unrequested, unmeasured wish is the same species of defect as
the archived `safety_action` that said ALLOW during 98 brakes: the artifact is well-formed and wrong.
So the source now reports both numbers, and this test pins the measurement semantics — including the
part that makes it a measurement rather than a label, namely that it goes to **zero** when frames
stop arriving.

Note on honesty about the investigation itself: an earlier probe of mine read "0.00 fps at 720p",
which was an artifact of sampling `frames_published` across a restart (the counter resets on start).
The reproduction gave 121 frames in 12 s. Both numbers are recorded in PROGRESS; only the second one
is true.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from webd.video import VideoSource  # noqa: E402


def _source():
    # Real signature, read by introspection rather than guessed: (enabled, orientation,
    # white_balance). `enabled=False` keeps the constructor from ever reaching for the camera, which
    # on this station is owned by webd - a unit test must not be able to take the lens.
    return VideoSource(enabled=False)


def test_measured_rate_counts_the_trailing_window():
    src = _source()
    t0 = 1000.0
    src._recent = [t0 - 4.0, t0 - 3.0, t0 - 2.0, t0 - 0.5]
    assert abs(src.measured_fps(t0, window_s=5.0) - 4 / 5.0) < 1e-9
    assert len(src._recent) == 4, "everything is inside a 5 s window"


def test_stale_timestamps_are_dropped_not_average_into_a_lie():
    src = _source()
    t0 = 1000.0
    src._recent = [t0 - 30.0, t0 - 20.0]
    assert src.measured_fps(t0, window_s=5.0) == 0.0
    assert src._recent == [], "the window must be trimmed, or a dead stream advertises old frames"


def test_no_frames_means_zero_and_not_the_requested_rate():
    src = _source()
    src._recent = []
    assert src.measured_fps(1000.0) == 0.0


def test_requested_and_measured_are_separate_fields():
    # The bug was one field standing in for two meanings; re-merging them must fail loudly.
    from dataclasses import fields
    from webd.video import VideoState
    reported = {f.name for f in fields(VideoState)}
    assert {"fps", "fps_published", "frames_published"} <= reported
    assert VideoState().fps == 0.0 and VideoState().fps_published == 0.0, (
        "an idle source must claim neither a requested nor a delivered rate")
