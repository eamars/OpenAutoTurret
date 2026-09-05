"""§51 Vision-6 and §50 — the two tools that turn measurements into decisions, tested.

These tests exist because both tools' whole purpose is *refusal*. A bake-off that always ranks a
winner and a commission tool that always proposes a number are both machines for laundering a
guess into a config file, and they would pass any test that only checks the happy path. So most
of what is asserted here is an objection.

The recordings are still synthetic — but for these tools that is the *right* input: the question
under test is whether the tool reads the separation in a recording correctly, and a fixture whose
gap, overlap or duplicate pairs are known exactly is the only way to know what "correctly" is.
"""
from __future__ import annotations

import json
import os
import tempfile
import unittest
from dataclasses import replace

from perception.config import ScoreThresholds, VisionConfig
from perception.errors import PerceptionError
from perception.model import ModelManifest
from perception.replay import Recorder
from perception.tests.support import at, commissioned_config, det, dset
from perception.tools import bakeoff, commission

RECORDED_MODEL = "imx500-ssd-mobilenetv2-fpnlite320-pp-coco"
MANIFEST = ModelManifest(model_id=RECORDED_MODEL, input_width=320, input_height=320,
                         inference_rate_hz=26.0)


def person(frame: int, *, cx: float = 0.30, score: float = 0.85):
    return det(0, cx=cx + 0.002 * frame, cy=0.5, width=0.06, height=0.25, score=score)


def blip(frame: int, *, score: float = 0.20):
    """One detection per frame, at an alternating position: never matches its neighbour.

    The alternation is the whole point. A stationary blip matches the next frame's blip and
    becomes *persistent* evidence, and the test would silently measure the wrong thing.
    """
    return det(1, cx=0.80 if frame % 2 else 0.05, cy=0.15, width=0.04, height=0.08, score=score)


def duplicate_partner(frame: int, *, score: float = 0.60):
    """A second row over the same person: §16's numbers are only commissionable from these."""
    return det(2, cx=0.302 + 0.002 * frame, cy=0.505, width=0.062, height=0.252, score=score)


def frames_with(count: int, *, blips: bool = True, partner: bool = False,
                person_score: float = 0.85, blip_score: float = 0.20):
    frames = []
    for index in range(count):
        detections = [person(index, score=person_score)]
        if partner:
            detections.append(duplicate_partner(index))
        if blips:
            detections.append(blip(index, score=blip_score))
        frames.append(dset(detections, frame_index=index, sensor_ns=at(index)))
    return frames


def record(directory: str, frames, *, model_manifest=MANIFEST, config=None) -> str:
    os.makedirs(directory, exist_ok=True)
    with Recorder(directory, config=config or commissioned_config(),
                  model_manifest=model_manifest,
                  stream={"width": 1920, "height": 1080}, flush_every=1) as recorder:
        for frame in frames:
            recorder.record_frame(frame, camera={"width": 1920, "height": 1080,
                                                 "preserve_aspect_ratio": True})
    return directory


def bakeoff_config() -> VisionConfig:
    """Two profiles over one recording, differing only in §22/§23's score bands.

    ``loose`` will let a 0.12 blip create an identity; ``strict`` will not. Everything else is
    identical, so any difference in the table is the thresholds and nothing else.
    """
    config = commissioned_config()
    strict = replace(config.active_model, model_id=RECORDED_MODEL)
    loose = replace(strict, thresholds=ScoreThresholds(
        low_association=0.05, new_track=0.05, confirmed_update=0.10, selectable=0.20))
    config.models = {"strict": strict, "loose": loose}
    config.profile = "strict"
    return config


class Case(unittest.TestCase):
    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.root = self._temporary.name
        self.addCleanup(self._temporary.cleanup)
        self._counter = 0

    def record(self, frames=None, **kwargs) -> str:
        self._counter += 1
        return record(os.path.join(self.root, f"rec-{self._counter}"),
                      frames if frames is not None else frames_with(60), **kwargs)


class TestCapturePlan(Case):
    def test_every_capture_is_pinned_to_the_same_frame_count(self):
        lines = bakeoff.plan_captures(bakeoff_config(), ["strict", "loose"], frames=240,
                                      out_dir="/tmp/bo")
        captures = [line for line in lines if "--max-frames" in line]
        self.assertEqual(len(captures), 2)
        self.assertEqual({line.split("--max-frames")[1].split()[0] for line in captures},
                         {"240"},
                         "two captures of different lengths are not a model comparison")
        self.assertTrue(all("--record-dataset /tmp/bo/" in line for line in captures))

    def test_the_plan_ends_with_the_command_that_compares_them(self):
        lines = bakeoff.plan_captures(bakeoff_config(), ["strict", "loose"], out_dir="/tmp/bo")
        self.assertIn("perception.tools.bakeoff --recordings", lines[-1])
        self.assertIn("strict=/tmp/bo/strict", lines[-1])
        self.assertIn("loose=/tmp/bo/loose", lines[-1])

    def test_a_recording_spec_without_a_name_is_refused(self):
        with self.assertRaises(PerceptionError) as caught:
            bakeoff.parse_recordings("/tmp/bo/strict")
        self.assertIn("profile=path", str(caught.exception))
        with self.assertRaises(PerceptionError):
            bakeoff.parse_recordings("   ")


class TestBakeoffComparison(Case):
    def test_a_short_capture_is_excluded_not_ranked(self):
        directory = self.record(frames_with(12))
        row = bakeoff.profile_metrics(bakeoff_config(), "strict", directory)
        self.assertTrue(row["excluded"])
        self.assertIn("frames", row["excluded"])
        self.assertIn(str(bakeoff.MIN_COMPARABLE_FRAMES), row["excluded"])

    def test_the_looser_thresholds_lose_and_the_tool_says_which_number_decided(self):
        directory = self.record(frames_with(60))
        result = bakeoff.compare_recordings(bakeoff_config(),
                                            {"strict": directory, "loose": directory})
        self.assertIsNotNone(result["recommendation"], result["objections"])
        self.assertEqual(result["recommendation"]["profile"], "strict",
                         "a threshold that lets a blip become an identity must not win")
        self.assertEqual(result["ranking"][0], "strict")
        self.assertEqual(result["recommendation"]["decided_by"], "identities_seen",
                         "identity economy must be what decides here; if latency ever is, the "
                         "profile that reached selectability by promoting a blip has won")
        self.assertIn("vs", result["recommendation"]["why"])
        by_profile = {row["profile"]: row for row in result["profiles"]}
        self.assertGreater(by_profile["loose"]["metrics"]["identities_seen"],
                           by_profile["strict"]["metrics"]["identities_seen"])

    def test_an_earlier_selection_of_a_spurious_identity_does_not_win(self):
        # The ordering bug this test pins: loose thresholds reached "selectable" a frame sooner
        # by promoting a detector blip. Latency rewards whatever gets there, so it cannot be
        # allowed to outrank the number of identities a profile manufactured.
        directory = self.record(frames_with(60))
        rows = {profile: bakeoff.profile_metrics(bakeoff_config(), profile, directory)
                for profile in ("strict", "loose")}
        self.assertLess(rows["loose"]["metrics"]["first_selectable_latency_p95_frames"],
                        rows["strict"]["metrics"]["first_selectable_latency_p95_frames"],
                        "the premise: the loose profile does look faster")
        result = bakeoff.compare_recordings(bakeoff_config(),
                                            {"strict": directory, "loose": directory})
        self.assertEqual(result["recommendation"]["profile"], "strict")

    def test_a_sole_candidate_is_reported_as_no_comparison(self):
        directory = self.record(frames_with(60))
        result = bakeoff.compare_recordings(bakeoff_config(), {"strict": directory})
        self.assertIsNotNone(result["recommendation"])
        self.assertTrue(result["recommendation"]["unopposed"])
        self.assertEqual(result["recommendation"]["decided_by"], "sole-candidate")

    def test_unequal_capture_lengths_refuse_the_comparison(self):
        long_dir = self.record(frames_with(60))
        short_dir = self.record(frames_with(45))
        result = bakeoff.compare_recordings(bakeoff_config(),
                                            {"strict": long_dir, "loose": short_dir})
        self.assertIsNone(result["recommendation"])
        self.assertTrue(any("frame counts" in objection for objection in result["objections"]),
                        result["objections"])

    def test_a_recording_made_by_another_model_is_flagged_as_not_a_detector_comparison(self):
        directory = self.record(frames_with(60),
                                model_manifest=ModelManifest(model_id="some-other-network",
                                                             input_width=320, input_height=320,
                                                             inference_rate_hz=26.0))
        result = bakeoff.compare_recordings(bakeoff_config(),
                                            {"strict": directory, "loose": directory})
        self.assertTrue(any("not detectors" in objection for objection in result["objections"]),
                        result["objections"])

    def test_a_profile_that_is_not_in_the_config_is_excluded(self):
        directory = self.record(frames_with(60))
        row = bakeoff.profile_metrics(bakeoff_config(), "ghost", directory)
        self.assertIn("not in this configuration", row["excluded"])

    def test_an_unmeasured_latency_never_ranks_first(self):
        unmeasured = bakeoff._sort_key({"first_selectable_latency_p95_frames": None,
                                        "duplicate_active_identities_max": 0,
                                        "gate_failures": 0, "short_lived_identities": 0,
                                        "identities_seen": 0})
        measured = bakeoff._sort_key({"first_selectable_latency_p95_frames": 40.0,
                                      "duplicate_active_identities_max": 9,
                                      "gate_failures": 3, "short_lived_identities": 9,
                                      "identities_seen": 9})
        latency_index = [name for name, _, _ in bakeoff.RANKING].index(
            "first_selectable_latency_p95_frames")
        self.assertGreater(unmeasured[latency_index], measured[latency_index],
                           "a profile whose latency was never seen must not outrank one "
                           "measured as slow — silence is not speed")

    def test_the_cli_exit_code_distinguishes_a_recommendation_from_silence(self):
        # 0 = a comparison was made, 3 = nothing was. A CI job that reads "no recommendation" as
        # success is worse than one that crashes, so the codes have to stay distinct.
        directory = self.record(frames_with(60))
        config_path = os.path.join(self.root, "bakeoff-config.json")
        with open(config_path, "w", encoding="utf-8") as handle:
            json.dump(bakeoff_config().to_dict(), handle)
        common = ["--config", config_path, "--quiet"]
        self.assertEqual(bakeoff.main(["--recordings", f"strict={directory}"] + common), 3,
                         "a sole candidate is not a comparison")
        self.assertEqual(bakeoff.main(["--recordings",
                                       f"strict={directory},loose={directory}"] + common), 0)


class TestCommission(Case):
    def result(self, directory, config=None, **kwargs):
        return commission.propose_thresholds(directory, config, **kwargs)

    def test_a_mock_recording_is_refused_even_with_the_assertion(self):
        directory = self.record(frames_with(60),
                                model_manifest=ModelManifest(model_id="offline-mock",
                                                             input_width=320, input_height=320,
                                                             inference_rate_hz=16.0))
        result = self.result(directory, real_evidence=True)
        self.assertEqual(result["evidence_grade"], "fictional")
        self.assertEqual(result["proposals"], {})
        self.assertTrue(any("fixture" in objection for objection in result["objections"]))

    def test_the_operator_has_to_assert_the_evidence_is_real(self):
        directory = self.record(frames_with(60))
        result = self.result(directory)
        self.assertEqual(result["evidence_grade"], "unasserted")
        self.assertEqual(result["proposals"], {})
        self.assertIn("--real-evidence", result["objections"][0])

    def test_too_few_samples_refuse_a_number(self):
        directory = self.record(frames_with(8))
        result = self.result(directory, real_evidence=True)
        self.assertEqual(result["evidence_grade"], "insufficient")
        self.assertIn("not enough samples", result["objections"][0])

    def test_a_real_gap_in_the_scores_becomes_a_threshold_with_its_basis(self):
        directory = self.record(frames_with(60))
        result = self.result(directory, real_evidence=True)
        self.assertEqual(result["evidence_grade"], "asserted-real")
        for name in ("new_track", "selectable", "confirmed_update", "low_association"):
            self.assertIn(name, result["proposals"], result["objections"])
            self.assertIn("basis", result["proposals"][name])
            self.assertTrue(result["proposals"][name]["samples"],
                            "a proposal that does not say how many samples stand behind it is "
                            "the number without the evidence")
        values = [result["proposals"][name]["value"]
                  for name in ("low_association", "new_track", "confirmed_update", "selectable")]
        self.assertEqual(values, sorted(values), f"§22's bands crossed: {values}")
        proposed = result["proposals"]["new_track"]["value"]
        self.assertGreater(proposed, 0.20, "must sit above the transient population")
        self.assertLess(proposed, 0.85, "must sit below the real detections")
        self.assertIn("transient p95", result["proposals"]["new_track"]["basis"])

    def test_overlapping_populations_refuse_a_threshold_rather_than_average_them(self):
        directory = self.record(frames_with(60, person_score=0.50, blip_score=0.50))
        result = self.result(directory, real_evidence=True)
        self.assertNotIn("new_track", result["proposals"])
        self.assertTrue(any("overlap" in objection for objection in result["objections"]),
                        result["objections"])
        self.assertTrue(any("No threshold separates" in objection
                            for objection in result["objections"]),
                        "the refusal has to say it in words a commit message can quote")

    def test_dedup_numbers_need_observed_double_fires(self):
        directory = self.record(frames_with(60))
        result = self.result(directory, real_evidence=True)
        self.assertNotIn("nms_iou", result["proposals"])
        self.assertTrue(any("double-fired" in objection for objection in result["objections"]))

    def test_observed_duplicate_pairs_do_yield_dedup_numbers(self):
        directory = self.record(frames_with(60, partner=True))
        result = self.result(directory, real_evidence=True)
        for name in ("nms_iou", "containment_ratio", "center_distance_norm"):
            self.assertIn(name, result["proposals"], result["objections"])
        self.assertLessEqual(result["proposals"]["nms_iou"]["value"], 0.75)
        self.assertGreaterEqual(result["proposals"]["nms_iou"]["value"], 0.30)
        self.assertIn("p05", result["proposals"]["containment_ratio"]["basis"])

    def test_currently_configured_values_are_shown_for_comparison(self):
        directory = self.record(frames_with(60))
        result = self.result(directory, bakeoff_config(), real_evidence=True)
        self.assertIn("current", result)
        self.assertIn("new_track", result["current"]["thresholds"])

    def test_the_main_command_refuses_to_write_an_empty_fragment(self):
        directory = self.record(frames_with(60))
        target = os.path.join(self.root, "fragment.json")
        config_path = os.path.join(self.root, "config.json")
        with open(config_path, "w", encoding="utf-8") as handle:
            json.dump(commissioned_config().to_dict(), handle)
        code = commission.main(["--recording", directory, "--config", config_path,
                                "--write", target])
        self.assertEqual(code, 3)
        self.assertFalse(os.path.exists(target),
                         "an empty thresholds fragment silently reopens every §50 question")

    def test_the_main_command_writes_a_reviewable_fragment(self):
        directory = self.record(frames_with(60, partner=True))
        target = os.path.join(self.root, "fragment.json")
        config_path = os.path.join(self.root, "config.json")
        with open(config_path, "w", encoding="utf-8") as handle:
            json.dump(commissioned_config().to_dict(), handle)
        code = commission.main(["--recording", directory, "--config", config_path,
                                "--real-evidence", "--write", target])
        self.assertIn(code, (0, 3))                 # 3 = something still unresolved
        with open(target, encoding="utf-8") as handle:
            fragment = json.load(handle)
        self.assertIn("_warning", fragment)
        self.assertIn("new_track", fragment["thresholds"])
        self.assertIn("nms_iou", fragment["dedup"])
        self.assertIn("asserted-real", fragment["_warning"])


if __name__ == "__main__":                                          # pragma: no cover
    unittest.main()
