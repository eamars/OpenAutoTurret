from pathlib import Path
import json
import os
import tempfile
import unittest

from Firmware.commissioning.model_family import FamilyNative
from Firmware.tools import adr0022_load_structure_verification as verification


class LoadStructureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native=FamilyNative(os.environ["OTA_AXIS_CORE_LIBRARY"])
        cls.local=Path("run/adr0022-stage2/architect-review-response-02/cycle-03/load-structure/tests-local")
        cls.local.mkdir(parents=True,exist_ok=True)

    def test_native_affine_estimation_rejects_wrong_constant_without_holdout_selection(self):
        with tempfile.TemporaryDirectory(dir=self.local) as folder:
            output=Path(folder)
            result=verification.verify_world(self.native,"affine",53,output)
            self.assertTrue(result["passed"])
            self.assertEqual(result["selected_load"],"affine")
            wrong=next(row for row in result["candidates"] if row["load"]=="constant")
            self.assertTrue(wrong["optimizer"]["success"])
            self.assertFalse(all(row["passed"] for row in wrong["selection"]))
            freeze=json.loads((output/"frozen-selection-before-holdout.json").read_text())
            self.assertEqual(freeze["selected_model"],next(row["model"] for row in result["candidates"] if row["load"]=="affine"))
            self.assertEqual(freeze["holdout_observations"],"NOT_GENERATED_OR_READ")
            self.assertFalse(result["rejected_alternatives_see_holdout"])

    def test_native_constant_truth_does_not_add_unneeded_affine_structure(self):
        with tempfile.TemporaryDirectory(dir=self.local) as folder:
            result=verification.verify_world(self.native,"constant",53,Path(folder))
            self.assertTrue(result["passed"])
            self.assertEqual(result["selected_load"],"constant")
            self.assertTrue(all(row["optimizer"]["success"] for row in result["candidates"]))
            self.assertTrue(all(all(report["passed"] for report in row["selection"]) for row in result["candidates"]))

    def test_gauge_equivalent_load_and_directional_friction_are_not_uniquely_identified(self):
        report=verification.gauge_report(self.native)
        self.assertTrue(report["passed"])
        self.assertFalse(report["estimating_separate_offset_and_directional_friction_supported"])
        self.assertLess(report["maximum_all_native_channel_difference"],1e-10)

    def candidate(self,load,complexity,*,passed=True,score=1.):
        return {"load":load,"complexity":complexity,"optimizer":{"success":True},
            "training":[{"passed":passed}],"selection":[{"passed":passed,"errors":{"q_rms_rad":score,"gyro_rms_rad_s":score}}]}

    def test_better_score_cannot_override_failed_absolute_gate(self):
        passing=self.candidate("affine",6)
        failing=self.candidate("constant",4,passed=False,score=0.)
        self.assertIs(verification.select_candidate([failing,passing]),passing)
        self.assertIsNone(verification.select_candidate([failing]))

    def test_complexity_precedes_score_among_absolute_passing_candidates(self):
        constant=self.candidate("constant",4,score=1.)
        affine=self.candidate("affine",6,score=.01)
        self.assertIs(verification.select_candidate([affine,constant]),constant)

    def test_unqualified_input_outside_support_is_rejected_without_clipping(self):
        with self.assertRaisesRegex(ValueError,"exceeds supplied load interpolation support"):
            verification.generate("constant","train_left",3251)


if __name__=="__main__":unittest.main()
