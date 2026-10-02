from __future__ import annotations
import copy
import importlib.util
import math
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('adr0021',ROOT/'tools/adr0021.py')
assert spec and spec.loader
m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)

class Base(unittest.TestCase):
    def setUp(self):
        self.policy=m.load_json(ROOT/'manifests/policy.json')
        self.space=m.load_json(ROOT/'manifests/search_space.example.json')
        self.params={'kp':1.0,'ki':.6}
        self.h=m.digest(self.params)
    def result(self,name='smooth',direction=1):
        return m.score(m.synthetic_trace(name,self.h,direction),3.0*direction,self.h,self.policy)
    def good_receipt(self):
        return {'accepted':True,'readback_verified':True,'revision':1,
                'binary_hash':'binary','plan_hash':'plan','effective_values':self.params,
                'effective_hash':self.h}
    def gate(self): return m.TrialGate(self.params,1,'binary','plan')

class PlannerTests(Base):
    def test_plan_is_reproducible_and_bounded(self):
        a=m.make_plan(self.space,self.policy); b=m.make_plan(self.space,self.policy)
        self.assertEqual(a,b); self.assertEqual(len(a['candidates']),16)
        m.verify_plan(a)
        for c in a['candidates']:
            p=c['parameters']; self.assertTrue(1<=p['kp']<=8 and 0<=p['ki']<=.6)
            self.assertTrue(0<=p['run_pos']<=p['break_pos']<=.2)
            self.assertTrue(0<=p['run_neg']<=p['break_neg']<=.2)
    def test_plan_tampering_is_detected(self):
        p=m.make_plan(self.space,self.policy); p['candidates'][0]['parameters']['kp']=99
        with self.assertRaises(m.ContractError): m.verify_plan(p)
    def test_reference_tool_rejects_hardware_authorization(self):
        self.space['physical_execution_authorized']=True
        with self.assertRaises(m.ContractError): m.make_plan(self.space,self.policy)
    def test_change_seed_or_budget_rejected(self):
        for key,value in [('seed',100),('count',15),('count',17)]:
            with self.subTest(key=key,value=value):
                s=copy.deepcopy(self.space);s[key]=value
                with self.assertRaises(m.ContractError):m.make_plan(s,self.policy)
    def test_invalid_domain_rejected(self):
        for low,high in [(0,8),(9,8),(float('nan'),8),(True,8)]:
            s=copy.deepcopy(self.space);s['dimensions'][0].update(low=low,high=high)
            with self.assertRaises(m.ContractError):m.make_plan(s,self.policy)
    def test_refinement_single_bounded_batch(self):
        centers=m.make_plan(self.space,self.policy)['candidates'][:2]
        a=m.refine_plan(self.space,centers,self.policy)
        self.assertEqual(len(a),8)
        self.assertEqual(a,m.refine_plan(self.space,centers,self.policy))
        for c in a:
            self.assertTrue(all(0<=x<=1 for x in c['normalized']))
    def test_wrong_refinement_centers_rejected(self):
        with self.assertRaises(m.ContractError):m.refine_plan(self.space,[],self.policy)

class GateTests(Base):
    def test_rejected_apply_never_runs(self):
        g=self.gate()
        with self.assertRaises(m.ContractError):g.accept_receipt({'accepted':False})
        with self.assertRaises(m.ContractError):g.start(True)
        self.assertEqual(g.run_count,0);self.assertEqual(g.state,'ABORT_CAMPAIGN')
    def test_kp2_requested_kp1_effective_never_runs(self):
        g=m.TrialGate({'kp':2.0,'ki':.6},1,'binary','plan')
        with self.assertRaises(m.ContractError):g.accept_receipt(self.good_receipt())
        self.assertEqual(g.run_count,0)
    def test_tx_success_without_readback_never_runs(self):
        g=self.gate();r=self.good_receipt();r['readback_verified']=False
        with self.assertRaises(m.ContractError):g.accept_receipt(r)
        self.assertEqual(g.run_count,0)
    def test_wrong_revision_binary_or_plan_rejected(self):
        for key,value in [('revision',2),('binary_hash','other'),('plan_hash','other')]:
            with self.subTest(key=key):
                g=self.gate();r=self.good_receipt();r[key]=value
                with self.assertRaises(m.ContractError):g.accept_receipt(r)
    def test_capture_must_be_ready(self):
        g=self.gate();g.accept_receipt(self.good_receipt())
        with self.assertRaises(m.ContractError):g.start(False)
        self.assertEqual(g.run_count,0)
    def test_good_trial_quality_failure_is_not_hard_abort(self):
        g=self.gate();g.accept_receipt(self.good_receipt());g.start(True)
        g.observe_identity(self.h,1,'binary');g.finish('FAIL_QUALITY')
        self.assertEqual(g.state,'FINISHED');self.assertEqual(g.run_count,1)
    def test_identity_change_aborts_running_trial(self):
        g=self.gate();g.accept_receipt(self.good_receipt());g.start(True)
        with self.assertRaises(m.ContractError):g.observe_identity(self.h,2,'binary')
        self.assertEqual(g.state,'ABORT_CAMPAIGN')
    def test_duplicate_start_rejected(self):
        g=self.gate();g.accept_receipt(self.good_receipt());g.start(True)
        with self.assertRaises(m.ContractError):g.start(True)
        self.assertEqual(g.run_count,1)
    def test_readonly_unknown_nan_and_out_of_range_rejected(self):
        reg={'kp':{'mutability':'experiment_writable','low':1.,'high':8.},
             'current_cap':{'mutability':'protected_read_only','low':0.,'high':.8}}
        self.assertEqual(m.apply_registry(reg,{'kp':2.}),{'kp':2.})
        for values in [{'current_cap':.8},{'missing':1},{'kp':float('nan')},{'kp':True},{'kp':99}]:
            with self.assertRaises(m.ContractError):m.apply_registry(reg,values)

class MetricTests(Base):
    def test_smooth_both_directions_pass(self):
        for d in [1,-1]:self.assertEqual(self.result(direction=d)['status'],'PASS_SCOPE')
    def test_stalled_zero_jitter_fails_tracking(self):
        r=self.result('stalled');self.assertEqual(r['status'],'FAIL_QUALITY')
        self.assertEqual(r['metrics']['jitter_position_p95_p5_deg'],0)
        self.assertIn('tracking_ratio',r['reasons']);self.assertIn('sustained_motion',r['reasons'])
    def test_creep_low_jitter_fails_tracking(self):
        r=self.result('creep');self.assertEqual(r['status'],'FAIL_QUALITY')
        self.assertIn('tracking_ratio',r['reasons'])
    def test_oscillation_fails_position_jitter(self):
        r=self.result('oscillating');self.assertEqual(r['status'],'FAIL_QUALITY')
        self.assertIn('position_jitter',r['reasons'])
    def test_single_spike_not_hidden_by_percentiles(self):
        r=self.result('spike');self.assertEqual(r['status'],'FAIL_QUALITY')
        self.assertIn('position_spike',r['reasons'])
    def test_stop_drift_fails(self):
        r=self.result('drifting');self.assertIn('stop_drift',r['reasons'])
    def test_missing_interval_is_invalid_not_zero_error(self):
        rows=[r for r in m.synthetic_trace('smooth',self.h) if not 1.4<r['t_s']<1.7]
        self.assertEqual(m.score(rows,3,self.h,self.policy)['status'],'INVALID_DATA')
    def test_duplicate_or_reversed_timestamp_is_invalid(self):
        for t in [0.,-.1]:
            rows=m.synthetic_trace('smooth',self.h);rows[1]['t_s']=t
            self.assertEqual(m.score(rows,3,self.h,self.policy)['status'],'INVALID_DATA')
    def test_changed_parameter_hash_invalidates_trace(self):
        rows=m.synthetic_trace('smooth',self.h);rows[10]['parameters_hash']='different'
        self.assertEqual(m.score(rows,3,self.h,self.policy)['status'],'INVALID_DATA')
    def test_vref_cannot_replace_backend_command(self):
        rows=m.synthetic_trace('smooth',self.h)
        rows[5]['reference_origin']='vref'
        self.assertEqual(m.score(rows,3,self.h,self.policy)['status'],'INVALID_DATA')
    def test_wrong_reference_and_zero_reference_rejected(self):
        rows=m.synthetic_trace('smooth',self.h)
        for ref in [0,5]:self.assertEqual(m.score(rows,ref,self.h,self.policy)['status'],'INVALID_DATA')
    def test_nonfinite_sample_rejected(self):
        for x in [float('nan'),float('inf'),True]:
            rows=m.synthetic_trace('smooth',self.h);rows[5]['q_deg']=x
            self.assertEqual(m.score(rows,3,self.h,self.policy)['status'],'INVALID_DATA')
    def test_short_hold_not_accepted(self):
        rows=[r for r in m.synthetic_trace('smooth',self.h) if r['t_s']<4.5]
        self.assertEqual(m.score(rows,3,self.h,self.policy)['status'],'INVALID_DATA')
    def test_missing_steady_not_accepted(self):
        rows=[r for r in m.synthetic_trace('smooth',self.h) if r['phase']=='hold']
        self.assertEqual(m.score(rows,3,self.h,self.policy)['status'],'INVALID_DATA')
    def test_longer_hold_uses_exact_frozen_window(self):
        rows=m.synthetic_trace('smooth',self.h)
        for r in rows:
            if r['phase']=='hold' and r['t_s']>5.08:r['q_deg']+=1
        r=m.score(rows,3,self.h,self.policy)
        self.assertEqual(r['status'],'PASS_SCOPE')
        self.assertEqual(r['hold_window_s'],2.)
    def test_reference_plateau_variation_invalid(self):
        rows=m.synthetic_trace('smooth',self.h);rows[30]['be_cmd_dps']=1.
        self.assertEqual(m.score(rows,3,self.h,self.policy)['status'],'INVALID_DATA')

class SelectionTests(Base):
    def group(self,name='smooth'):
        return m.aggregate(name,{'positive':self.result(name,1),'negative':self.result(name,-1)},['positive','negative'])
    def test_missing_direction_invalidates_group(self):
        g=m.aggregate('one',{'positive':self.result()},['positive','negative'])
        self.assertEqual(g['status'],'INVALID_DATA')
    def test_worst_direction_controls_feasibility(self):
        g=m.aggregate('mixed',{'positive':self.result(),'negative':self.result('creep',-1)},['positive','negative'])
        self.assertEqual(g['status'],'FAIL_QUALITY')
    def test_existing_feasible_skips_refinement(self):
        rs=[self.group('smooth'),self.group('creep')]
        decision=m.next_stage(rs,'coarse',['smooth','creep'])
        self.assertEqual(decision,{'next':'CONFIRM','candidate_ids':['smooth']})
    def test_no_feasible_refines_only_once(self):
        rs=[self.group('creep'),self.group('stalled')]
        self.assertEqual(m.next_stage(rs,'coarse',['creep','stalled'])['next'],'REFINE_ONCE')
        self.assertEqual(m.next_stage(rs,'refine',['creep','stalled'])['next'],'NO_FEASIBLE_CANDIDATE')
    def test_incomplete_batch_cannot_advance(self):
        rs=[self.group('smooth')]
        self.assertEqual(m.next_stage(rs,'coarse',['smooth','creep'])['next'],'ABORT_CAMPAIGN')
    def test_duplicate_or_extra_candidate_cannot_advance(self):
        a=self.group('smooth')
        self.assertEqual(m.next_stage([a,a],'coarse',['smooth'])['next'],'ABORT_CAMPAIGN')
    def test_invalid_or_hard_abort_not_silently_ranked(self):
        for status in ['HARD_ABORT','INVALID_DATA','BLOCKED']:
            rs=[{'candidate_id':'bad','status':status},self.group('smooth')]
            self.assertEqual(m.next_stage(rs,'coarse',['bad','smooth'])['next'],'ABORT_CAMPAIGN')
    def test_retry_is_exactly_once_and_hard_abort_never_retries(self):
        self.assertEqual(m.retry_decision('INVALID_DATA',0),'RETRY_IDENTICAL_ONCE')
        self.assertEqual(m.retry_decision('INVALID_DATA',1),'ABORT_CAMPAIGN')
        self.assertEqual(m.retry_decision('HARD_ABORT',0),'ABORT_CAMPAIGN')
    def test_ranking_ignores_current_and_stable_tiebreak(self):
        a={'candidate_id':'a','status':'PASS_SCOPE','max_normalized_quality':.2,'mean_normalized_quality':.1,'current':.8}
        b={**a,'candidate_id':'b','current':.1}
        self.assertEqual([r['candidate_id'] for r in m.rank_results([b,a],True)],['a','b'])

if __name__=='__main__':unittest.main(verbosity=2)
