from __future__ import annotations

from dataclasses import replace
import os
import unittest

import numpy as np

from Firmware.commissioning.contracts import Reason, Rejected
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.threshold_identification import (
    ThresholdPolicy,RestSupport,PlateauTrial,censored_threshold_intervals,fit_threshold_family,threshold_outcome_support)
from Firmware.commissioning.model_family import FamilyRun
from Firmware.commissioning.synthetic_family_oracle import independent_rollout
from Firmware.tools import adr0022_threshold_verification as fixture


class ThresholdIdentificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native=FamilyNative(os.environ["OTA_AXIS_CORE_LIBRARY"])
        cls.training_run,cls.trials,cls.truth,_=fixture.generate("development",43)
        cls.rest_support=fixture.synthetic_rest_support(cls.training_run,cls.trials,cls.truth)

    def intervals(self,trials=None,model=None):
        chosen_trials=trials or self.trials
        ids={(x.run_id,x.trial_id) for x in chosen_trials}
        supports=[x for x in self.rest_support if (x.run_id,x.trial_id) in ids]
        return censored_threshold_intervals(model or fixture.model(),[self.training_run],chosen_trials,
            bounds=fixture.MOVING_BOUNDS,threshold_bounds=fixture.THRESHOLD_BOUNDS,rest_support=supports)

    def test_real_native_fit_retains_nonunique_threshold_interval(self):
        initial=replace(fixture.model(),a=.085,viscous=.075,coulomb_negative=.09,coulomb_positive=.13,
            static_negative=.21,static_positive=.23)
        fit=fit_threshold_family(self.native,initial,[self.training_run],self.trials,
            bounds=fixture.MOVING_BOUNDS,threshold_bounds=fixture.THRESHOLD_BOUNDS,rest_support=self.rest_support)
        record=fit["threshold_identification"]
        self.assertTrue(fit["optimizer"]["success"])
        self.assertLess(record["training_outer_cost_span"],1e-8)
        self.assertEqual(record["outer_candidates_evaluated"],9)
        self.assertFalse(record["representative_is_exact_static_estimate"])
        for direction in ("negative","positive"):
            interval=record["interval_evidence"]["intervals"][direction]
            self.assertLessEqual(interval["lower_A"],getattr(fixture.model(),"static_"+direction))
            self.assertGreater(interval["upper_A"],getattr(fixture.model(),"static_"+direction))
        for coordinate in fixture.MOVING_BOUNDS:
            actual=getattr(fixture.model(),coordinate)
            self.assertLess(abs(getattr(fit["model"],coordinate)-actual)/actual,.05)

    def test_short_no_start_is_dynamic_censor_not_threshold_lower_bound(self):
        trial=replace(self.trials[1],end_time=self.trials[1].command_time+.09)
        record=self.intervals([trial])
        self.assertEqual(record["trials"][0]["outcome"],"DYNAMICALLY_OR_OBSERVATION_CENSORED")
        self.assertEqual(record["intervals"]["negative"]["lower_A"],fixture.THRESHOLD_BOUNDS["negative"][0])

    def test_single_direction_does_not_claim_both_thresholds(self):
        with self.assertRaises(Rejected) as error:
            fit_threshold_family(self.native,fixture.model(),[self.training_run],self.trials[:1],
                bounds=fixture.MOVING_BOUNDS,threshold_bounds=fixture.THRESHOLD_BOUNDS)
        self.assertEqual(error.exception.reason,Reason.INSUFFICIENT_EXCITATION)

    def test_future_run_trial_cannot_enter_training(self):
        with self.assertRaises(Rejected) as error:
            self.intervals([replace(self.trials[0],run_id="selection-acquisition")])
        self.assertEqual(error.exception.reason,Reason.DATA_INVALID)

    def test_duplicate_trial_rejected(self):
        with self.assertRaises(Rejected):self.intervals([self.trials[0],self.trials[0]])

    def test_repeated_successful_tx_of_same_current_is_a_plateau(self):
        trial=self.trials[0]
        index=np.searchsorted(self.training_run.tx_t,trial.command_time,side="right")-1
        repeat_time=trial.command_time+.05
        updated=replace(self.training_run,
            tx_t=np.insert(self.training_run.tx_t,index+1,repeat_time),
            tx_A=np.insert(self.training_run.tx_A,index+1,self.training_run.tx_A[index]))
        record=censored_threshold_intervals(fixture.model(),[updated],[trial],
            bounds=fixture.MOVING_BOUNDS,threshold_bounds=fixture.THRESHOLD_BOUNDS,
            rest_support=[x for x in self.rest_support if x.trial_id==trial.trial_id])
        self.assertEqual(record["trials"][0]["outcome"],"NO_START_CENSORED")

    def test_unsupported_load_gauge_is_explicit(self):
        with self.assertRaises(Rejected) as error:
            self.intervals(model=replace(fixture.model(),load_offset=.02))
        self.assertEqual(error.exception.reason,Reason.INSUFFICIENT_EXCITATION)

    def test_interval_interior_does_not_become_a_supported_start(self):
        interval={"lower_A":.135,"upper_A":.145,"upper_inclusive":False}
        self.assertEqual(threshold_outcome_support(interval,.135),"SUPPORTED_STATIC_HOLD")
        self.assertEqual(threshold_outcome_support(interval,.14),"AMBIGUOUS_INSIDE_THRESHOLD_INTERVAL")
        self.assertEqual(threshold_outcome_support(interval,.145),"SUPPORTED_STATIC_RELEASE")
        self.assertEqual(threshold_outcome_support(interval,.145,input_uncertainty_A=.001),
                         "AMBIGUOUS_INSIDE_THRESHOLD_INTERVAL")

    def test_wrong_sign_and_bad_policy_rejected(self):
        with self.assertRaises(Rejected):self.intervals([replace(self.trials[0],direction=-1)])
        with self.assertRaises(Rejected):ThresholdPolicy(grid_fractions=(0.,)).validate()

    def test_noisy_rest_observations_do_not_certify_physical_rest(self):
        record=self.intervals()
        self.assertFalse(record["interval_confidence_calibrated"])
        self.assertIn("quiet sensor bands alone never establish sticking",record["rest_support"])
        self.assertFalse(record["physical_rest_qualification_from_synthetic"])

    def test_quiet_creep_does_not_create_false_static_threshold_bound(self):
        t=np.arange(201)*.005;tx_t=np.array([-.1,.3,1.]);tx_A=np.array([.14072,.16,0.])
        initial=np.array([0.,.012,.14072,.012,.14072])
        truth=independent_rollout(fixture.model(),t,tx_t,tx_A,initial).trace
        rng=np.random.default_rng(43);quantum=2*np.pi/8192
        q=np.round((truth[:,0]+rng.normal(0,.00015,len(t)))/quantum)*quantum
        v_new=np.arange(len(t))%4==0;v=np.full(len(t),np.nan)
        v[v_new]=truth[v_new,3]+rng.normal(0,.005,int(v_new.sum()))
        current=truth[:,4]+rng.normal(0,.002,len(t))
        run=FamilyRun("quiet-creep","independent-creep",t,q,v,current,np.ones(len(t),bool),v_new,
            np.ones(len(t),bool),tx_t,tx_A,initial,.00015,.005,.002,provenance="SYNTHETIC",encoder_quantum=quantum)
        trial=PlateauTrial(run.run_id,"substatic-creep",1,.3,.9,.1)
        record=censored_threshold_intervals(fixture.model(),[run],[trial],bounds=fixture.MOVING_BOUNDS,
            threshold_bounds={"negative":(.12,.21),"positive":(.15,.24)})
        self.assertTrue(record["trials"][0]["observation_window_quiet"])
        self.assertEqual(record["trials"][0]["outcome"],"AMBIGUOUS_REST_NOT_ESTABLISHED")
        self.assertEqual(record["intervals"]["positive"]["upper_A"],.24)
        self.assertEqual(record["intervals"]["positive"]["start_constraints"],0)

    def test_synthetic_rest_support_cannot_be_used_for_measured_run(self):
        measured=replace(self.training_run,provenance="MEASURED")
        with self.assertRaises(Rejected):
            censored_threshold_intervals(fixture.model(),[measured],self.trials,bounds=fixture.MOVING_BOUNDS,
                threshold_bounds=fixture.THRESHOLD_BOUNDS,rest_support=self.rest_support)

    def test_quiet_window_without_rest_support_remains_ambiguous(self):
        record=censored_threshold_intervals(fixture.model(),[self.training_run],self.trials,bounds=fixture.MOVING_BOUNDS,
                threshold_bounds=fixture.THRESHOLD_BOUNDS)
        self.assertTrue(all(x["outcome"]=="AMBIGUOUS_REST_NOT_ESTABLISHED" for x in record["trials"]))
        self.assertTrue(all(x["start_constraints"]==x["no_start_constraints"]==0 for x in record["intervals"].values()))


if __name__ == "__main__":unittest.main()
