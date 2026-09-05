"""Record and replay (§43): the way to make a tracking claim re-checkable without the scene.

``Recorder`` writes a run to a directory — manifest (§43's must-preserve list), per-frame
DetectionSets and camera metadata, published observations, critical events, optional images.
``ReplaySource`` reads one back, unmodified, and reports what it found rather than what was
hoped for: malformed lines, out-of-order frames, and any drift between the configuration that
made the recording and the one replaying it.

``evaluator.run_level_b`` is §43's Level B — recorded detections through class filter, §16
dedup, ``TrackManager`` and selection, driven by the *recorded* sensor clock. Its metrics come
from published output only, and ``compare_runs`` decides whether two runs of the same recording
were the same run.

Level A (recorded images through the real network) needs the IMX500, so this package records
what Level A would need and says so plainly rather than pretending to run a network it cannot
reach.
"""
from __future__ import annotations

from .evaluator import (DEFAULT_GATES, DeterminismDiff, LevelBRun, ReplayReport,
                        canonical_run, compare_ground_truth, compare_runs, engineering_gates,
                        evaluate, run_level_b)
from .recorder import (DETECTIONS_FILE, GROUND_TRUTH_FILE, IMAGES_DIR, MANIFEST_FILE,
                       OBSERVATIONS_FILE, RECORDING_SCHEMA_VERSION, Recorder, RecordingManifest,
                       write_ground_truth)
from .replay_source import ReplayFrame, ReplaySource, ReplaySummary

__all__ = ["DETECTIONS_FILE", "DEFAULT_GATES", "DeterminismDiff", "GROUND_TRUTH_FILE",
           "IMAGES_DIR", "LevelBRun", "MANIFEST_FILE", "OBSERVATIONS_FILE",
           "RECORDING_SCHEMA_VERSION", "Recorder", "RecordingManifest", "ReplayFrame",
           "ReplayReport", "ReplaySource", "ReplaySummary", "canonical_run",
           "compare_ground_truth", "compare_runs", "engineering_gates", "evaluate",
           "run_level_b", "write_ground_truth"]
