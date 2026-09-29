import unittest
from dataclasses import replace
import numpy as np
from Firmware.commissioning.contracts import Rejected,Reason,digest
from Firmware.commissioning.applicability import *
from Firmware.commissioning.probe_control import fixture_control


class ApplicabilityTests(unittest.TestCase):
    def setUp(self):
        self.snapshot,_,_,_=fixture_control()
        self.domain=OperatingDomain(self.snapshot.identity.hardware,self.snapshot.identity.measurement,
                                    {"payload":["base","up"]},{"temperature_C":[10,50],"mass_kg":[.1,2]},("mass_kg",))
        self.point={"payload":"base","temperature_C":25.,"mass_kg":None}
        self.bounds={"current_A":[-2,2],"temperature_C":[0,70],"bus_V":[20,26],"position_rad":[-1,1],
                     "approved_duration_s":10.,"approved_unknown_temperature_window_s":3.}
    def test_unknown_mass_can_be_covered_by_response_not_zero(self):
        self.assertTrue(self.domain.check(self.snapshot.identity.hardware,self.snapshot.identity.measurement,
                                         self.point,independent_prediction_passed=True))
        with self.assertRaises(Rejected):self.domain.check(self.snapshot.identity.hardware,
            self.snapshot.identity.measurement,self.point,independent_prediction_passed=False)
    def test_uncovered_temperature_cannot_extrapolate(self):
        with self.assertRaises(Rejected):self.domain.check(self.snapshot.identity.hardware,
            self.snapshot.identity.measurement,{**self.point,"temperature_C":80.},independent_prediction_passed=True)
    def test_unknown_temperature_cannot_hide_bus_danger(self):
        result=protection_decision({"feedback_valid":True,"current_A":.2,"temperature_C":None,
                                   "bus_V":30.,"position_rad":0.},self.bounds,elapsed_s=1.)
        self.assertEqual(result["reason"],"HARD_ABORT")
    def test_temperature_unknown_only_permits_approved_finite_evidence(self):
        data={"feedback_valid":True,"current_A":.2,"temperature_C":None,"bus_V":24.,"position_rad":0.}
        result=protection_decision(data,self.bounds,elapsed_s=1.)
        self.assertEqual(result["response"],"CONTINUE_APPROVED_FINITE_WINDOW")
        self.assertFalse(result["continuous_qualified"])
        self.assertEqual(protection_decision(data,self.bounds,elapsed_s=4.)["response"],"END_QUALIFICATION_KEEP_VERIFIED_SUPPORT")
    def test_thirty_minutes_is_not_automatic_thermal_equilibrium(self):
        t=np.arange(0,1801,10.)
        self.assertFalse(thermal_equilibrium(t,25+t*.01,temperature_max_C=70,required_margin_C=5)["passed"])
        self.assertTrue(thermal_equilibrium(t,25+np.exp(-t/60),temperature_max_C=70,required_margin_C=5)["passed"])
    def test_production_revision_invalidates_only_affected_evidence(self):
        result=invalidated_assets(["software.production.reference_manager"])
        self.assertIn("3b",result["invalidate"]);self.assertNotIn("3a",result["invalidate"])
        self.assertIn("immutable_raw_history",result["preserve"])
    def test_session_tare_does_not_erase_model_history(self):
        result=invalidated_assets(["measurement.session.tare"])
        self.assertEqual(result["invalidate"],["session_mapping","session_readiness"])
    def test_old_profile_without_current_prediction_cannot_be_rollback(self):
        with self.assertRaises(Rejected):applicable_rollback([self.snapshot],self.domain,self.point,
            hardware=self.snapshot.identity.hardware,measurement=self.snapshot.identity.measurement,prediction_checks={})


if __name__=="__main__":unittest.main()
