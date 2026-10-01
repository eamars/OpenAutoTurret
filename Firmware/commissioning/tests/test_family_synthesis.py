"""Selected-family synthesis contracts after actual lift/core/oracle probes."""
from dataclasses import asdict, replace
import ctypes as ct
import unittest

from Firmware.commissioning.contracts import Reason, Rejected
from Firmware.commissioning.family_synthesis import FamilySynthesisStatus
from Firmware.commissioning.synthesis import selected_family_solve
from Firmware.tools.adr0022_family_sampled_probe import family_fixture
from Firmware.tools.adr0022_family_synthesis_probe import dynamic_fixture, DYNAMIC_POLICY


class FamilySynthesisTests(unittest.TestCase):
    def setUp(self):
        fixtures=[family_fixture(d,filtered=True,stribeck=True) for d in (-1,1)]
        self.model,self.params,self.schedule,self.nominal,self.policy=(fixtures[0][i] for i in (0,3,4,5,6))
        self.points=tuple(f[1] for f in fixtures)
        self.supports=tuple(f[2] for f in fixtures)

    def solve(self,grid=(.5,1.,2.,4.),**changed):
        arguments=dict(model=self.model,points=self.points,supports=self.supports,
            observer=self.params.observer,nominal_gains=self.nominal,schedule=self.schedule,
            wn_grid=grid,ff_policy=self.policy)
        return selected_family_solve(**{**arguments,**changed})

    def test_fastest_fully_passing_candidate_and_failed_grid_points_are_retained(self):
        result=self.solve()
        self.assertEqual(result.status,FamilySynthesisStatus.SELECTED)
        self.assertEqual(result.selected_wn_rad_s,4.)
        self.assertEqual([c['passed'] for c in result.candidates],[False,True,True,True])
        self.assertEqual([len(c['points']) for c in result.candidates],[2]*4)
        self.assertLess(result.minimum_incremental_damping_A_s_rad,0.)
        for point in result.candidates[-1]['points']:
            self.assertGreaterEqual(point['diagnostics']['phase_margin_deg']-.02,50.)
            self.assertGreaterEqual(point['diagnostics']['gain_margin_db']-.02,6.)
            self.assertTrue(point['diagnostics']['pole_diagnostics']['local_stable'])
        self.assertEqual(result.document()['physical_qualification'],'NOT_RUN')

    def test_actuator_command_gain_changes_units_without_changing_effective_loop(self):
        base=self.solve((4.,))
        mapped=self.solve((4.,),model=replace(self.model,actuator_gain=2.))
        self.assertEqual(mapped.status,FamilySynthesisStatus.SELECTED)
        self.assertAlmostEqual(mapped.gains.kp*2,base.gains.kp,places=13)
        self.assertAlmostEqual(mapped.gains.ki*2,base.gains.ki,places=13)
        for a,b in zip(base.candidates[0]['points'],mapped.candidates[0]['points']):
            for name in ('phase_margin_deg','gain_margin_db'):
                self.assertAlmostEqual(a['diagnostics'][name],b['diagnostics'][name],places=7)

    def test_evaluated_grid_failure_returns_no_gains_and_explicit_search_domain(self):
        result=self.solve((.5,))
        self.assertEqual(result.status,FamilySynthesisStatus.NO_FEASIBLE)
        self.assertEqual(result.reason,Reason.ENVELOPE_LIMITED)
        self.assertIsNone(result.gains)
        self.assertIsNone(result.selected_wn_rad_s)
        self.assertEqual(len(result.candidates),1)
        self.assertTrue(all(not p['passed'] and p['diagnostics']['phase_margin_deg'] is not None
            for p in result.candidates[0]['points']))

    def test_unsupported_measurement_ack_and_limit_context_is_distinct_from_grid_failure(self):
        for schedule,reason in ((replace(self.schedule,encoder_quantum_rad=.001),Reason.MEASUREMENT_LIMITED),
            (replace(self.schedule,immediate_successful_ack=False),Reason.INTEGRATION_MISMATCH),
            (replace(self.schedule,limits_inactive=False),Reason.INTEGRATION_MISMATCH)):
            with self.subTest(reason=reason,schedule=schedule):
                result=self.solve(schedule=schedule)
                self.assertEqual(result.status,FamilySynthesisStatus.UNSUPPORTED)
                self.assertEqual(result.reason,reason)
                self.assertIsNone(result.gains)
                self.assertEqual(result.candidates,())

    def test_unsupported_family_and_operating_slice_do_not_emit_a_candidate(self):
        for model in (replace(self.model,actuator='first_order',actuator_tau=.007),
            replace(self.model,transport_delay=.001),
            replace(self.model,actuator_tau=.007)):
            result=self.solve(model=model)
            self.assertEqual(result.status,FamilySynthesisStatus.UNSUPPORTED)
            self.assertIsNone(result.gains)
        result=self.solve(points=(replace(self.points[0],v_rad_s=.05),self.points[1]),
            supports=(self.supports[1],self.supports[1]))
        self.assertEqual(result.status,FamilySynthesisStatus.UNSUPPORTED)
        self.assertEqual(result.reason,Reason.OPERATING_POINT_CHANGED)

    def test_dynamic_reference_curve_failure_preserves_all_actual_family_cases(self):
        fixtures=dynamic_fixture()
        model,_,_,params,schedule,nominal,policy=fixtures[0]
        result=self.solve(model=model,points=tuple(f[1] for f in fixtures),
            supports=tuple(f[2] for f in fixtures),observer=params.observer,
            nominal_gains=nominal,schedule=schedule,ff_policy=policy)
        self.assertEqual(policy,DYNAMIC_POLICY)
        self.assertEqual(result.status,FamilySynthesisStatus.NO_FEASIBLE)
        self.assertIsNone(result.gains)
        self.assertEqual(len(result.candidates),4)
        self.assertEqual([len(c['points']) for c in result.candidates],[2]*4)
        self.assertTrue(all(p['diagnostics']['pole_diagnostics']['local_stable']
            for c in result.candidates for p in c['points']))
        self.assertTrue(all(c['points'][0]['diagnostics']['phase_margin_deg']<50.
            for c in result.candidates))
        self.assertEqual(model.actuator,'first_order')
        self.assertGreater(model.transport_delay,0.)
        self.assertEqual(result.document()['nonlinear_qualification'],'NOT_RUN')

    def test_dynamic_policy_and_nonsmooth_prerequisites_remain_explicit(self):
        fixtures=dynamic_fixture()
        model,_,_,params,schedule,nominal,_=fixtures[0]
        arguments=dict(model=model,points=tuple(f[1] for f in fixtures),
            supports=tuple(f[2] for f in fixtures),observer=params.observer,
            nominal_gains=nominal,schedule=schedule)
        for policy in ('SHARED_POSTERIOR_SLIDE_ALGEBRAIC','FROZEN_COMMAND_OFFSET'):
            result=self.solve(ff_policy=policy,**arguments)
            self.assertEqual(result.status,FamilySynthesisStatus.UNSUPPORTED)
            self.assertEqual(result.reason,Reason.INTEGRATION_MISMATCH)
            self.assertEqual(result.candidates,())
        result=self.solve(ff_policy=DYNAMIC_POLICY,
            **{**arguments,'schedule':replace(schedule,encoder_quantum_rad=.001)})
        self.assertEqual(result.status,FamilySynthesisStatus.UNSUPPORTED)
        self.assertEqual(result.reason,Reason.MEASUREMENT_LIMITED)
        self.assertIsNone(result.gains)

    def test_multiple_declared_signed_sliding_points_are_all_evaluated(self):
        points=(self.points[0],self.points[1],replace(self.points[0],v_rad_s=-.14),
            replace(self.points[1],v_rad_s=.14))
        supports=self.supports+self.supports
        result=self.solve((4.,),points=points,supports=supports)
        self.assertEqual(len(result.candidates[0]['points']),4)
        self.assertEqual([p['v_rad_s'] for p in result.candidates[0]['points']],
            [-.1,.1,-.14,.14])

    def test_declared_damping_curve_has_bounded_budget_and_fastest_lowest_tie(self):
        result=self.solve((1.,2.),damping_ratio_grid=(1.,1.5))
        passing=[c for c in result.candidates if c['passed']]
        fastest=max(c['wn_rad_s'] for c in passing)
        damping=min(c['damping_ratio'] for c in passing if c['wn_rad_s']==fastest)
        self.assertEqual((result.selected_wn_rad_s,result.selected_damping_ratio),(fastest,damping))
        self.assertEqual(len(result.candidates),4)
        for grid in ((),(False,1.),(0.,1.),(1.,1.),(2.,1.),(float('nan'),)):
            with self.subTest(grid=grid),self.assertRaises(Rejected) as caught:
                self.solve(damping_ratio_grid=grid)
            self.assertEqual(caught.exception.reason,Reason.DATA_INVALID)
        with self.assertRaises(Rejected) as caught:
            self.solve(tuple(range(1,33)),damping_ratio_grid=tuple(range(1,9)))
        self.assertEqual(caught.exception.reason,Reason.DATA_INVALID)

    def test_actual_dynamic_third_curve_selects_only_fully_passing_candidate(self):
        fixtures=dynamic_fixture()
        model,_,_,params,schedule,nominal,policy=fixtures[0]
        before=ct.string_at(ct.addressof(params),ct.sizeof(params))
        result=self.solve(model=model,points=tuple(f[1] for f in fixtures),
            supports=tuple(f[2] for f in fixtures),observer=params.observer,
            nominal_gains=nominal,schedule=schedule,ff_policy=policy,damping_ratio_grid=(2.5,3.,4.))
        self.assertEqual(result.status,FamilySynthesisStatus.SELECTED)
        self.assertEqual((result.selected_wn_rad_s,result.selected_damping_ratio),(.5,4.))
        self.assertEqual(len(result.candidates),12)
        self.assertEqual(sum(c['passed'] for c in result.candidates),1)
        selected=next(c for c in result.candidates if c['passed'])
        self.assertTrue(all(p['diagnostics']['passed'] for p in selected['points']))
        self.assertEqual(result.gains.kaw,nominal.kaw)
        self.assertEqual(ct.string_at(ct.addressof(params),ct.sizeof(params)),before)
        self.assertEqual(result.document()['nonlinear_qualification'],'NOT_RUN')

    def test_declared_grid_malformed_inputs_remain_typed_data_errors(self):
        for grid in ((),(False,1.),(0.,1.),(1.,1.),(2.,1.),(1.,float('inf'))):
            with self.subTest(grid=grid),self.assertRaises(Rejected) as caught:
                self.solve(grid)
            self.assertEqual(caught.exception.reason,Reason.DATA_INVALID)

    def test_explicit_revised_margin_selects_faster_candidate_without_relabelling_original(self):
        fixtures=dynamic_fixture()
        model,_,_,params,schedule,nominal,policy=fixtures[0]
        arguments=dict(model=model,points=tuple(f[1] for f in fixtures),
            supports=tuple(f[2] for f in fixtures),observer=params.observer,
            nominal_gains=nominal,schedule=schedule,ff_policy=policy,
            damping_ratio_grid=(1.,1.5,2.,2.5,3.,4.))
        result=self.solve(**arguments,phase_required_deg=45.,gain_required_db=6.)
        self.assertEqual((result.selected_wn_rad_s,result.selected_damping_ratio),(2.,1.5))
        self.assertEqual((result.phase_required_deg,result.gain_required_db),(45.,6.))
        self.assertEqual(len(result.candidates),24)
        selected=next(c for c in result.candidates if c['wn_rad_s']==2. and c['damping_ratio']==1.5)
        self.assertTrue(selected['passed'])
        self.assertTrue(all(p['diagnostics']['pole_diagnostics']['local_stable'] for p in selected['points']))
        self.assertLess(selected['points'][0]['diagnostics']['phase_margin_deg'],50.)
        original=self.solve((2.,),**{**arguments,'damping_ratio_grid':(1.5,)})
        self.assertEqual(original.status,FamilySynthesisStatus.NO_FEASIBLE)
        self.assertEqual((original.document()['phase_required_deg'],original.document()['gain_required_db']),(50.,6.))
        self.assertEqual(result.gains.kaw,nominal.kaw)

    def test_margin_requirements_are_finite_positive_and_retained_on_unsupported_result(self):
        for name in ('phase_required_deg','gain_required_db'):
            for value in (False,0.,-1.,float('nan'),float('inf'),'45'):
                with self.subTest(name=name,value=value),self.assertRaises(Rejected) as caught:
                    self.solve(**{name:value})
                self.assertEqual(caught.exception.reason,Reason.DATA_INVALID)
        result=self.solve(schedule=replace(self.schedule,encoder_quantum_rad=.001),
            phase_required_deg=45.,gain_required_db=8.)
        self.assertEqual(result.status,FamilySynthesisStatus.UNSUPPORTED)
        self.assertEqual((result.document()['phase_required_deg'],result.document()['gain_required_db']),(45.,8.))

    def test_coulomb_uses_actual_damping_and_dynamic_shared_policy_without_relaxing_prerequisites(self):
        model=replace(self.model,friction='coulomb',actuator_gain=1.7)
        result=self.solve(model=model)
        self.assertEqual(result.status,FamilySynthesisStatus.SELECTED)
        self.assertEqual(result.minimum_incremental_damping_A_s_rad,model.viscous)
        self.assertAlmostEqual(result.gains.kp,(2*model.a*result.selected_wn_rad_s-model.viscous)/model.actuator_gain)
        self.assertAlmostEqual(result.gains.ki,model.a*result.selected_wn_rad_s**2/model.actuator_gain)
        self.assertEqual((result.phase_required_deg,result.gain_required_db),(50.,6.))
        fixtures=dynamic_fixture()
        dynamic,_,_,params,schedule,nominal,policy=fixtures[0]
        arguments=dict(model=replace(dynamic,friction='coulomb'),points=tuple(f[1] for f in fixtures),
            supports=tuple(f[2] for f in fixtures),observer=params.observer,
            nominal_gains=nominal,schedule=schedule,ff_policy=policy)
        selected=self.solve(**arguments)
        self.assertEqual(selected.status,FamilySynthesisStatus.SELECTED)
        self.assertEqual(selected.minimum_incremental_damping_A_s_rad,dynamic.viscous)
        self.assertEqual(len(selected.candidates),4)
        for changed,reason in (({'schedule':replace(schedule,encoder_quantum_rad=.001)},Reason.MEASUREMENT_LIMITED),
            ({'ff_policy':'SHARED_POSTERIOR_SLIDE_ALGEBRAIC'},Reason.INTEGRATION_MISMATCH),
            ({'points':(replace(fixtures[0][1],v_rad_s=0.),fixtures[1][1])},Reason.OPERATING_POINT_CHANGED)):
            unsupported=self.solve(**{**arguments,**changed})
            self.assertEqual(unsupported.status,FamilySynthesisStatus.UNSUPPORTED)
            self.assertEqual(unsupported.reason,reason)
            self.assertIsNone(unsupported.gains)

    def test_nominal_parameters_and_observer_are_not_modified(self):
        before=ct.string_at(ct.addressof(self.params),ct.sizeof(self.params))
        nominal=asdict(self.nominal)
        result=self.solve((4.,))
        self.assertEqual(ct.string_at(ct.addressof(self.params),ct.sizeof(self.params)),before)
        self.assertEqual(asdict(self.nominal),nominal)
        self.assertEqual(result.gains.kaw,self.nominal.kaw)
        self.assertNotEqual(result.gains,self.nominal)


if __name__=='__main__': unittest.main()
