from __future__ import annotations
import copy
import json
from pathlib import Path
import unittest
import numpy as np
from jsonschema import Draft202012Validator, ValidationError
from reference.model_math import (IdentificationError, integral_rows, fit_initializer,
                                  ideal_pi, linear_discrete_matrix, spectral_radius)
from reference.contracts import (canonical_hash, reuse_action, can_start, bundle_reasons,
                                 completion_reasons, DIGEST_KEYS)
from reference.demo import synthetic_data, fixture_bundle

ROOT=Path(__file__).resolve().parents[1]

class MathematicsTests(unittest.TestCase):
    def test_integral_initializer_recovers_known_truth(self):
        data, truth=synthetic_data()
        X,y=integral_rows(*data)
        r=fit_initializer(X,y)
        for key,value in truth.items(): self.assertAlmostEqual(getattr(r,key),value,places=5)
        self.assertGreater(r.rows,100)
    def test_ideal_pi_characteristic_coefficients(self):
        a,b,wn=.08,.045,5.
        p=ideal_pi(a,b,wn)
        self.assertAlmostEqual((b+p['kp_A_s_per_rad'])/a,2*wn)
        self.assertAlmostEqual(p['ki_A_per_rad']/a,wn*wn)
    def test_negative_kp_is_not_clamped(self):
        with self.assertRaises(ValueError):ideal_pi(.01,1.,1.)
    def test_stable_local_discrete_example(self):
        p=ideal_pi(.08,.045,5)
        self.assertLess(spectral_radius(linear_discrete_matrix(.08,.045,
                     p['kp_A_s_per_rad'],p['ki_A_per_rad'],.005)),1)
    def test_delay_can_invalidate_high_bandwidth(self):
        p=ideal_pi(.08,.045,50)
        no=linear_discrete_matrix(.08,.045,p['kp_A_s_per_rad'],p['ki_A_per_rad'],.005)
        late=linear_discrete_matrix(.08,.045,p['kp_A_s_per_rad'],p['ki_A_per_rad'],.005,transport_delay_s=.04)
        self.assertLess(spectral_radius(no),1)
        self.assertGreater(spectral_radius(late),1)
    def test_fractional_delay_rounds_up(self):
        A=linear_discrete_matrix(.08,.045,.5,1,.005,transport_delay_s=.006)
        self.assertEqual(A.shape,(5,5))
    def test_zero_drag_plant_supported(self):
        self.assertTrue(np.isfinite(linear_discrete_matrix(.08,0,.5,1,.005)).all())
    def test_rank_deficiency_rejected(self):
        with self.assertRaises(IdentificationError):fit_initializer(np.ones((20,4)),np.ones(20))
    def test_unexcited_direction_rejected(self):
        X=np.random.default_rng(1).normal(size=(20,4));X[:,3]=0
        with self.assertRaises(IdentificationError):fit_initializer(X,np.ones(20))
    def test_nonfinite_design_rejected(self):
        X=np.random.default_rng(1).normal(size=(20,4));X[0,0]=np.nan
        with self.assertRaises(IdentificationError):fit_initializer(X,np.ones(20))
    def test_bad_shapes_rejected(self):
        with self.assertRaises(IdentificationError):fit_initializer(np.zeros((20,5)),np.ones(20))
    def test_too_few_equations_rejected(self):
        with self.assertRaises(IdentificationError):fit_initializer(np.eye(4),np.ones(4))
    def test_duplicate_timestamps_rejected(self):
        data,_=synthetic_data(); data=list(data);data[0]=data[0].copy();data[0][4]=data[0][3]
        with self.assertRaises(IdentificationError):integral_rows(*data)
    def test_descending_timestamps_rejected(self):
        data,_=synthetic_data();data=list(data);data[0]=-data[0]
        with self.assertRaises(IdentificationError):integral_rows(*data)
    def test_length_mismatch_rejected(self):
        data,_=synthetic_data();data=list(data);data[1]=data[1][:-1]
        with self.assertRaises(IdentificationError):integral_rows(*data)
    def test_no_motion_windows_rejected(self):
        data,_=synthetic_data();data=list(data);data[-1]=np.zeros_like(data[-1])
        with self.assertRaises(IdentificationError):integral_rows(*data)
    def test_direction_change_not_a_single_window(self):
        t=np.arange(0,2,.01);q=t**2;v=2*t;i=np.ones_like(t)
        d=np.ones_like(t);d[50]=-1
        X,y=integral_rows(t,q,v,i,d,window_samples=50)
        self.assertEqual(len(y),1)
    def test_bool_window_not_number(self):
        data,_=synthetic_data()
        with self.assertRaises(IdentificationError):integral_rows(*data,window_samples=True)

class ReuseTests(unittest.TestCase):
    def test_identical_supported_state_reuses_assets(self):
        self.assertEqual(reuse_action(same_hardware=True,same_measurement=True,
           covered_operating_point=True,prediction_check_passed=True),'REUSE_EXACT')
    def test_payload_change_updates_not_rebuilds(self):
        self.assertEqual(reuse_action(same_hardware=True,same_measurement=True,
           covered_operating_point=False,prediction_check_passed=False),'UPDATE_PARAMETERS')
    def test_unannounced_friction_change_updates(self):
        self.assertEqual(reuse_action(same_hardware=True,same_measurement=True,
           covered_operating_point=True,prediction_check_passed=False),'UPDATE_PARAMETERS')
    def test_changed_hardware_invalidates_affected_assets(self):
        self.assertEqual(reuse_action(same_hardware=False,same_measurement=True,
           covered_operating_point=True,prediction_check_passed=True),'INVALIDATE_AFFECTED_ASSETS')
    def test_changed_measurement_invalidates_not_reuses(self):
        self.assertEqual(reuse_action(same_hardware=True,same_measurement=False,
           covered_operating_point=True,prediction_check_passed=True),'INVALIDATE_AFFECTED_ASSETS')
    def test_reuse_truthy_string_rejected(self):
        with self.assertRaises(ValueError):reuse_action(same_hardware='false',same_measurement=True,
           covered_operating_point=True,prediction_check_passed=True)

class TransactionTests(unittest.TestCase):
    def setUp(self):
        self.h=canonical_hash({'kp':2})
        self.receipt={'applied':True,'readback_verified':True,'parameters_hash':self.h,'revision':2}
    def test_exact_receipt_starts(self):self.assertTrue(can_start(self.h,2,self.receipt,acquisition_ready=True))
    def test_rejected_apply_does_not_start(self):
        self.receipt['applied']=False
        self.assertFalse(can_start(self.h,2,self.receipt,acquisition_ready=True))
    def test_readback_failure_does_not_start(self):
        self.receipt['readback_verified']=False
        self.assertFalse(can_start(self.h,2,self.receipt,acquisition_ready=True))
    def test_actual_kp1_requested_kp2_does_not_start(self):
        self.receipt['parameters_hash']=canonical_hash({'kp':1})
        self.assertFalse(can_start(self.h,2,self.receipt,acquisition_ready=True))
    def test_revision_mismatch_does_not_start(self):self.assertFalse(can_start(self.h,3,self.receipt,acquisition_ready=True))
    def test_missing_capture_does_not_start(self):self.assertFalse(can_start(self.h,2,self.receipt,acquisition_ready=False))
    def test_bool_revision_does_not_start(self):
        self.receipt['revision']=True
        self.assertFalse(can_start(self.h,1,self.receipt,acquisition_ready=True))
    def test_hash_order_independent(self):self.assertEqual(canonical_hash({'a':1,'b':2}),canonical_hash({'b':2,'a':1}))
    def test_nan_cannot_be_hashed_as_valid_parameter(self):
        with self.assertRaises(ValueError):canonical_hash({'kp':float('nan')})

class EvidenceTests(unittest.TestCase):
    def test_matching_synthetic_metadata_fixture(self):
        p,e=fixture_bundle();self.assertEqual(bundle_reasons(p,e),[])
    def test_bench_only_not_complete(self):
        p,e=fixture_bundle();self.assertIn('3b_REQUIRES_EXACTLY_ONE_CERTIFICATE',bundle_reasons(p,e[:1]))
    def test_production_only_not_complete(self):
        p,e=fixture_bundle();self.assertIn('3a_REQUIRES_EXACTLY_ONE_CERTIFICATE',bundle_reasons(p,e[1:]))
    def test_changed_snapshot_invalidates_old_certificate(self):
        p,e=fixture_bundle();p['model_hash']=canonical_hash('changed')
        self.assertTrue(any('model_hash' in x for x in bundle_reasons(p,e)))
    def test_same_label_different_payload_is_not_same_condition(self):
        p,e=fixture_bundle();e[1]['coverage'][0]['operating_point_hash']=canonical_hash('different payload')
        self.assertIn('CROSS_ENVIRONMENT_OPERATING_POINT_MISMATCH',bundle_reasons(p,e))
    def test_missing_direction_case_rejected(self):
        p,e=fixture_bundle();e[0]['coverage'].pop()
        self.assertIn('3a_MISSING_CASES',bundle_reasons(p,e))
    def test_only_yaw_rejected(self):
        p,e=fixture_bundle();p['axes']=['yaw']
        self.assertIn('PROFILE_REQUIRES_BOTH_AXES',bundle_reasons(p,e))
    def test_bad_profile_digest_rejected(self):
        p,e=fixture_bundle();p['model_hash']=None
        self.assertIn('PROFILE_INVALID_model_hash',bundle_reasons(p,e))
    def test_case_not_run_rejected(self):
        p,e=fixture_bundle();e[0]['coverage'][0]['status']='NOT_RUN'
        self.assertIn('3a_CASE_NOT_PASS',bundle_reasons(p,e))
    def test_duplicate_coverage_rejected(self):
        p,e=fixture_bundle();e[1]['coverage'].append(copy.deepcopy(e[1]['coverage'][0]))
        self.assertIn('3b_DUPLICATE_COVERAGE',bundle_reasons(p,e))
    def test_insufficient_repetitions_rejected(self):
        p,e=fixture_bundle();e[1]['coverage'][0]['repetitions']=1
        self.assertIn('3b_INSUFFICIENT_REPETITIONS',bundle_reasons(p,e))
    def test_changed_parameters_rejected(self):
        p,e=fixture_bundle();e[1]['parameters_hash']=canonical_hash('changed')
        self.assertIn('3b_MISMATCH_parameters_hash',bundle_reasons(p,e))
    def test_actual_readback_rejected(self):
        p,e=fixture_bundle();e[1]['actual_parameters_hash']=canonical_hash('wrong')
        self.assertIn('3b_READBACK_MISMATCH',bundle_reasons(p,e))
    def test_duplicate_certificates_rejected(self):
        p,e=fixture_bundle();e.append(copy.deepcopy(e[1]))
        self.assertIn('3b_REQUIRES_EXACTLY_ONE_CERTIFICATE',bundle_reasons(p,e))

# Each mutation is a separate unittest, rather than an inflated loop count.
for name,key,value,reason in [
 ('production_shadow','shadow',True,'3b_NOT_PHYSICAL_EXECUTION'),
 ('production_replay','execution','replay','3b_NOT_PHYSICAL_EXECUTION'),
 ('production_synthetic','execution','synthetic','3b_NOT_PHYSICAL_EXECUTION'),
 ('production_bypass','route','bypass','3b_WRONG_EXECUTION_ROUTE'),
 ('production_running_bench','program','commissiond','3b_WRONG_EXECUTION_ROUTE'),
 ('not_pass','status','BLOCKED','3b_NOT_PASS'),
 ('wrong_candidate','candidate_id','OTHER','3b_CANDIDATE_MISMATCH'),
 ('no_trace','trace_hashes',[],'3b_MISSING_RAW_TRACE'),
 ('no_binary','binary_hash',None,'3b_MISSING_BINARY'),
 ('unresolved_abort','unresolved_abort',True,'3b_UNRESOLVED_ABORT'),
 ('no_production_load','production_load_verified',False,'3b_PRODUCTION_INCOMPLETE'),
 ('no_lifecycle','lifecycle_verified',False,'3b_PRODUCTION_INCOMPLETE'),
 ('parity_failed','parity_passed',False,'3b_PRODUCTION_INCOMPLETE')]:
    def test(self,key=key,value=value,reason=reason):
        p,e=fixture_bundle();e[1][key]=value
        self.assertIn(reason,bundle_reasons(p,e))
    setattr(EvidenceTests,'test_'+name,test)

for key in ['hardware_signature','measurement_signature','core_build_hash','observer_hash','metrics_hash','test_spec_hash','envelope_hash']:
    def test(self,key=key):
        p,e=fixture_bundle();e[1][key]=canonical_hash('new:'+key)
        self.assertIn('3b_MISMATCH_'+key,bundle_reasons(p,e))
    setattr(EvidenceTests,'test_binding_'+key,test)

class CompletionTests(unittest.TestCase):
    def setUp(self):
        self.req=json.loads((ROOT/'contracts/requirements.json').read_text())['requirements']
        self.results=[{'id':r['id'],'status':'PASS','execution':r['evidence_kind'],
                       'evidence_ids':['SYNTHETIC_UNIT_TEST_METADATA']} for r in self.req]
    def test_all_mandatory_metadata_required(self):self.assertEqual(completion_reasons(self.req,self.results,True),[])
    def test_missing_requirement_not_done(self):
        self.assertTrue(completion_reasons(self.req,self.results[:-1],True))
    def test_no_evidence_not_done(self):
        self.results[0]['evidence_ids']=[]
        self.assertIn('UNMET_S1-01',completion_reasons(self.req,self.results,True))
    def test_mock_not_physical_requirement(self):
        row=next(r for r in self.results if r['id']=='S3B-01');row['execution']='synthetic'
        self.assertIn('NOT_PHYSICAL_S3B-01',completion_reasons(self.req,self.results,True))
    def test_mvp_label_does_not_override_missing_work(self):
        self.results[-1]['status']='MVP_DONE'
        self.assertTrue(completion_reasons(self.req,self.results,True))
    def test_dual_validation_false_not_done(self):
        self.assertIn('PROFILE_DUAL_VALIDATION_INCOMPLETE',completion_reasons(self.req,self.results,False))

class SchemaTests(unittest.TestCase):
    def test_all_schemas_well_formed(self):
        for p in (ROOT/'contracts').glob('*.schema.json'):Draft202012Validator.check_schema(json.loads(p.read_text()))
    def test_valid_profile(self):
        p,_=fixture_bundle();Draft202012Validator(json.loads((ROOT/'contracts/profile.schema.json').read_text())).validate(p)
    def test_valid_certificate_metadata(self):
        _,e=fixture_bundle();v=Draft202012Validator(json.loads((ROOT/'contracts/evidence.schema.json').read_text()))
        for cert in e:v.validate(cert)
    def test_schema_rejects_unknown_field(self):
        p,_=fixture_bundle();p['skip_production']=True
        with self.assertRaises(ValidationError):Draft202012Validator(json.loads((ROOT/'contracts/profile.schema.json').read_text())).validate(p)
    def test_schema_rejects_missing_hash(self):
        p,_=fixture_bundle();del p['parameters_hash']
        with self.assertRaises(ValidationError):Draft202012Validator(json.loads((ROOT/'contracts/profile.schema.json').read_text())).validate(p)


class EvidenceSequenceTests(unittest.TestCase):
    def test_3b_bound_to_exact_bench_certificate(self):
        p,e=fixture_bundle();e[1]['bench_certificate_hash']=canonical_hash('other bench')
        self.assertIn('PRODUCTION_NOT_BOUND_TO_THIS_BENCH_CERTIFICATE',bundle_reasons(p,e))
    def test_production_cannot_precede_bench(self):
        p,e=fixture_bundle();e[1]['started_at']='2026-09-29T09:00:00Z'
        self.assertIn('PRODUCTION_PRECEDES_BENCH_COMPLETION',bundle_reasons(p,e))
    def test_naive_time_rejected(self):
        p,e=fixture_bundle();e[1]['started_at']='2026-09-29T11:00:00'
        self.assertIn('3b_INVALID_TIME_INTERVAL',bundle_reasons(p,e))
    def test_negative_duration_rejected(self):
        p,e=fixture_bundle();e[0]['finished_at']='2026-09-29T09:00:00Z'
        self.assertIn('3a_INVALID_TIME_INTERVAL',bundle_reasons(p,e))
    def test_invalid_bundle_type_rejected(self):
        self.assertEqual(bundle_reasons({},[None]),['INVALID_BUNDLE_TYPE'])
    def test_malformed_coverage_rejected_without_crash(self):
        p,e=fixture_bundle();e[0]['coverage'][0]['axis']=['yaw']
        self.assertIn('3a_INVALID_COVERAGE_ROW',bundle_reasons(p,e))

if __name__=='__main__':unittest.main()
