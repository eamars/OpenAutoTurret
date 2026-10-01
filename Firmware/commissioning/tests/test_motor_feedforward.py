"""Offline FF contracts through the actual shared native controller."""
from dataclasses import replace
import math
import unittest

from Firmware.commissioning.model_family import FamilyModel
from Firmware.commissioning.applicability import (ConfigurationFact, ConfigurationFacts,
    ConfigurationSupport, FactStatus)
from Firmware.commissioning.motor_feedforward import (CausalState, Failure, FeedforwardRejected,
    BoundedStartPolicy, FeedforwardSupport, MotorFeedforward, ReferencePacket, parameters_for_family)
from Firmware.commissioning.native import CReference, Controller
from Firmware.commissioning.tests import test_controller_faults as fault_fixture


class MotorFeedforwardTests(unittest.TestCase):
    def setUp(self):
        self.case = fault_fixture.ControllerFaultTests()
        self.case.setUp()
        self.model = FamilyModel(a=.1, viscous=.06, coulomb_negative=.12, coulomb_positive=.12,
            static_negative=.16, static_positive=.16, q_min=-1e6, q_max=1e6,
            actuator_gain=2., actuator_bias=.03, transport_delay=0., gyro_bias=0., gyro_tau=0., gyro_delay=0.,
            current_gain=1., current_bias=0., current_tau=0., current_delay=0., load_offset=.02)
        self.configuration = ConfigurationFacts("synthetic-fixed-yaw", "SYNTHETIC", {
            "payload.distribution": ConfigurationFact((2., .1, .03), FactStatus.SYNTHETIC, "declared fixture"),
            "motor.settings": ConfigurationFact("synthetic-current-map-2", FactStatus.SYNTHETIC, "declared fixture"),
            "sensor.calibration": ConfigurationFact("synthetic-sensors", FactStatus.SYNTHETIC, "declared fixture")})
        binding = ConfigurationSupport("synthetic-ff-model-1", self.configuration,
            tuple(self.configuration.facts), qualification="SYNTHETIC_OFFLINE")
        self.support = FeedforwardSupport(configuration_id="synthetic-fixed-yaw", frame="output-shaft-rad",
            q_min_rad=-1., q_max_rad=1., velocity_max_rad_s=1., acceleration_max_rad_s2=3.,
            fixed_posture_rad=0., winding_min_rad=-2., winding_max_rad=2., temperature_min=10., temperature_max=30.,
            actuator_gain_min=.5, actuator_gain_max=3., command_cap_A=.9, command_slew_A_s=1.,
            max_reference_source_age_s=.02, max_state_age_s=.02, max_ack_delay_s=.02,
            rest_speed_rad_s=.001, state_encoder_consistency_rad=.01, state_gyro_consistency_rad_s=.03,
            qualification="SYNTHETIC_OFFLINE", configuration_support=binding)
        self.state = CausalState(q_rad=0., v_rad_s=0., posture_rad=0., winding_rad=0., temperature=20., time_s=0.,
            frame=self.support.frame, configuration_id=self.support.configuration_id, generation=1,
            friction_state="STICKING", static_balance_A=.01, fresh=True, valid=True, provenance="SYNTHETIC",
            configuration_facts=self.configuration)
        self.reference = ReferencePacket(q_ref_rad=.5, v_ref_rad_s=.8, a_ref_rad_s2=2., time_s=.005,
            source_time_s=0., expires_at_s=.02, frame=self.support.frame, configuration_id=self.support.configuration_id,
            trajectory_id="single-shaped-synthetic-trajectory", generation=1, fresh=True, valid=True)

    def open_adapter(self, *, model=None, support=None, previous_current=0.):
        model, support = model or self.model, support or self.support
        core = Controller(self.case.native, parameters_for_family(self.case.params, model,
            actuator_policy=support.actuator_policy, actuation_memory_max_s=support.actuation_memory_max_s,
            start_policy=support.start_policy))
        self.addCleanup(core.close)
        adapter = MotorFeedforward(core, model, support)
        adapter.reset(self.state, now=0., previous_current_A=previous_current, accepted_time_s=-.1)
        return core, adapter

    def step(self, adapter, reference=None, state=None):
        state = state or replace(self.state, time_s=.005)
        observation = self.case.observation(.005, 1)
        observation.position, observation.gyro_rate = state.q_rad, state.v_rad_s
        return adapter.step(observation, reference or self.reference, state)

    def assert_native_inhibited(self, core):
        out = core.step(self.case.observation(.010, 2), CReference(0., .8, 0., 0.))
        self.assertEqual((out.status, out.sequence, out.requested, out.limited), (5, 0, 0., 0.))

    def test_algebraic_map_replaces_one_ff_and_preserves_outer_pi_and_limits(self):
        _, adapter = self.open_adapter()
        out, demand = self.step(adapter)
        self.assertAlmostEqual(demand.effective_current_A, .388)
        self.assertAlmostEqual(demand.command_current_A, .179)
        self.assertEqual(out.feedforward, demand.command_current_A)
        self.assertAlmostEqual(out.requested, demand.command_current_A+1.)  # one clamped outer correction
        self.assertAlmostEqual(out.limited, .005)
        self.assertTrue(adapter.acknowledge(out, successful=True, accepted_time_s=.0052))

    def test_affine_load_uses_causal_position(self):
        model = replace(self.model, load="affine", load_slope=.04, q_origin=.1)
        _, adapter = self.open_adapter(model=model)
        adapter.reset(replace(self.state, q_rad=.18), now=0., previous_current_A=0., accepted_time_s=-.1)
        out, demand = self.step(adapter, state=replace(self.state, q_rad=.2, time_s=.005))
        self.assertEqual(demand.load_A, .02+.04*(out.position-.1))
        self.assertEqual(out.position, adapter.last_posterior.position)
        self.assertNotAlmostEqual(demand.load_A, .024)
        self.assertNotAlmostEqual(demand.load_A, .02+.04*(self.reference.q_ref_rad-.1))

    def test_rest_static_balance_is_explicit_and_can_retain_nonzero_current(self):
        _, adapter = self.open_adapter(previous_current=.02)
        reference = replace(self.reference, q_ref_rad=0., v_ref_rad_s=0., a_ref_rad_s2=0.)
        out, demand = self.step(adapter, reference, replace(self.state, time_s=.005, static_balance_A=.05))
        self.assertEqual(demand.phase, "REST")
        self.assertAlmostEqual(demand.command_current_A, .02)
        self.assertAlmostEqual(out.limited, .02)

    def test_reverse_and_stop_keep_causal_sliding_friction_direction(self):
        for velocity, phase in ((-.1, "REVERSE"), (0., "STOP")):
            with self.subTest(phase=phase):
                _, adapter = self.open_adapter()
                state = replace(self.state, time_s=.005, v_rad_s=.2, friction_state="SLIDING", static_balance_A=None)
                _, demand = self.step(adapter, replace(self.reference, q_ref_rad=0., v_ref_rad_s=velocity, a_ref_rad_s2=-1.), state)
                self.assertEqual((demand.phase, demand.direction), (phase, 1))
                self.assertEqual(demand.friction_A, .12)

    def test_stribeck_uses_supported_friction_without_static_kick_in_slide(self):
        model = replace(self.model, friction="stribeck", stribeck_negative=.1, stribeck_positive=.1)
        _, adapter = self.open_adapter(model=model)
        state = replace(self.state, time_s=.005, v_rad_s=.2, friction_state="SLIDING", static_balance_A=None)
        _, demand = self.step(adapter, replace(self.reference, v_ref_rad_s=.05), state)
        self.assertEqual(demand.phase, "SLIDE")
        self.assertAlmostEqual(demand.friction_A, .12+.04*math.exp(-.25))

    def test_invalid_reference_variants_latch_and_valid_packet_cannot_rearm(self):
        cases = [dict(valid=False), dict(fresh=False), dict(time_s=.004), dict(source_time_s=.006),
                 dict(source_time_s=-1.), dict(expires_at_s=.004), dict(trajectory_id=""),
                 dict(frame="different-frame"), dict(configuration_id="changed"), dict(generation=2),
                 dict(q_ref_rad=1.01), dict(v_ref_rad_s=1.01), dict(a_ref_rad_s2=3.01), dict(v_ref_rad_s=float("nan"))]
        for changes in cases:
            with self.subTest(changes=changes):
                core, adapter = self.open_adapter()
                with self.assertRaises(FeedforwardRejected):
                    self.step(adapter, replace(self.reference, **changes))
                self.assertFalse(adapter.armed)
                self.assert_native_inhibited(core)
                with self.assertRaises(FeedforwardRejected):
                    self.step(adapter)

    def test_invalid_causal_state_and_configuration_latch(self):
        cases = [dict(valid=False), dict(fresh=False), dict(time_s=.006), dict(time_s=-1.),
                 dict(q_rad=1.01), dict(winding_rad=2.01), dict(temperature=31.), dict(posture_rad=.01),
                 dict(provenance="MEASURED"), dict(configuration_id="new-payload"),
                 dict(friction_state="SLIDING", static_balance_A=None), dict(static_balance_A=.17), dict(generation=None),
                 dict(v_rad_s=.02)]
        for changes in cases:
            with self.subTest(changes=changes):
                core, adapter = self.open_adapter()
                with self.assertRaises(FeedforwardRejected):
                    self.step(adapter, state=replace(self.state, time_s=.005, **changes) if "time_s" not in changes else
                              replace(self.state, **changes))
                self.assert_native_inhibited(core)

    def test_dynamic_and_delayed_actuator_maps_are_typed_rejections(self):
        for model in (replace(self.model, actuator="first_order", actuator_tau=.01),
                      replace(self.model, transport_delay=.008)):
            with self.assertRaises(FeedforwardRejected) as rejected:
                parameters_for_family(self.case.params, model)
            self.assertEqual(rejected.exception.reason, Failure.UNSUPPORTED_ACTUATOR)

    def test_physical_support_and_numeric_guard_remain_separate(self):
        # The numerical integration guard supplies no physical support authority.
        model = replace(self.model, q_min=-.01, q_max=.01)
        _, adapter = self.open_adapter(model=model)
        out, _ = self.step(adapter)
        self.assertEqual(out.status, 0)
        core, adapter = self.open_adapter()
        with self.assertRaises(FeedforwardRejected) as rejected:
            self.step(adapter, state=replace(self.state, time_s=.005, q_rad=100.))
        self.assertEqual(rejected.exception.reason, Failure.OUTSIDE_SUPPORT)
        self.assert_native_inhibited(core)

    def test_unqualified_physical_model_and_mismatched_core_are_refused(self):
        core = Controller(self.case.native, parameters_for_family(self.case.params, self.model))
        self.addCleanup(core.close)
        with self.assertRaises(FeedforwardRejected):
            MotorFeedforward(core, self.model, replace(self.support, qualification="PHYSICAL_UNQUALIFIED"))
        self.assert_native_inhibited(core)
        wrong = Controller(self.case.native, self.case.params)
        self.addCleanup(wrong.close)
        with self.assertRaises(FeedforwardRejected) as rejected:
            MotorFeedforward(wrong, self.model, self.support)
        self.assertEqual(rejected.exception.reason, Failure.CORE_MISMATCH)

    def test_model_invalidation_latches_before_next_command(self):
        core, adapter = self.open_adapter()
        adapter.model = replace(self.model, actuator_gain=0.)
        with self.assertRaises(FeedforwardRejected):
            self.step(adapter)
        self.assert_native_inhibited(core)

    def test_mutable_python_template_cannot_substitute_for_active_native_readback(self):
        core = Controller(self.case.native, self.case.params)
        self.addCleanup(core.close)
        compatible = parameters_for_family(self.case.params, self.model)
        core.params = compatible  # native configure has already copied the incompatible original
        with self.assertRaises(FeedforwardRejected) as rejected:
            MotorFeedforward(core, self.model, self.support)
        self.assertEqual(rejected.exception.reason, Failure.CORE_MISMATCH)
        self.assert_native_inhibited(core)

    def test_constant_load_rejects_nonzero_inactive_slope(self):
        with self.assertRaises(FeedforwardRejected) as rejected:
            parameters_for_family(self.case.params, replace(self.model, load_slope=.04))
        self.assertEqual(rejected.exception.reason, Failure.UNSUPPORTED_MODEL)

    def test_missing_packets_latch_instead_of_allowing_next_valid_packet(self):
        for missing in ("reference", "state", "observation"):
            with self.subTest(missing=missing):
                core, adapter = self.open_adapter()
                inputs = dict(observation=self.case.observation(.005, 1), reference=self.reference,
                              state=replace(self.state, time_s=.005))
                inputs[missing] = None
                with self.assertRaises(FeedforwardRejected):
                    adapter.step(**inputs)
                self.assert_native_inhibited(core)
                with self.assertRaises(FeedforwardRejected):
                    self.step(adapter)

    def test_missing_ack_output_latches(self):
        core, adapter = self.open_adapter()
        self.step(adapter)
        with self.assertRaises(FeedforwardRejected) as rejected:
            adapter.acknowledge(None, successful=True, accepted_time_s=.006)
        self.assertEqual(rejected.exception.reason, Failure.TX_FAILURE)
        self.assert_native_inhibited(core)

    def test_contradictory_measured_motion_or_position_inhibits_before_friction_reversal(self):
        for position, velocity in ((0., .2), (.2, 0.)):
            with self.subTest(position=position, velocity=velocity):
                core, adapter = self.open_adapter()
                observation = self.case.observation(.005, 1)
                observation.position, observation.gyro_rate = position, velocity
                with self.assertRaises(FeedforwardRejected) as rejected:
                    adapter.step(observation, replace(self.reference, v_ref_rad_s=-.1), replace(self.state, time_s=.005))
                self.assertEqual(rejected.exception.reason, Failure.INVALID_STATE)
                self.assert_native_inhibited(core)

    def test_malformed_ack_current_or_result_latches_before_ctypes_conversion(self):
        for successful, applied in ((True, "bad"), ("bad", None), (True, float("nan")), (True, 1.)):
            with self.subTest(successful=successful, applied=applied):
                core, adapter = self.open_adapter()
                out, _ = self.step(adapter)
                with self.assertRaises(FeedforwardRejected) as rejected:
                    adapter.acknowledge(out, successful=successful, accepted_time_s=.006, applied_current_A=applied)
                self.assertEqual(rejected.exception.reason, Failure.TX_FAILURE)
                self.assert_native_inhibited(core)
                self.assertFalse(adapter.acknowledge(out, successful=True, accepted_time_s=.007))

    def test_failed_tx_and_expired_ack_inhibit(self):
        for successful, accepted_time in ((False, .006), (True, .026)):
            with self.subTest(successful=successful):
                core, adapter = self.open_adapter()
                out, _ = self.step(adapter)
                with self.assertRaises(FeedforwardRejected) as rejected:
                    adapter.acknowledge(out, successful=successful, accepted_time_s=accepted_time)
                self.assertEqual(rejected.exception.reason, Failure.TX_FAILURE)
                self.assert_native_inhibited(core)

    def test_explicit_reset_rejects_old_ack_without_losing_new_pending_output(self):
        _, adapter = self.open_adapter()
        old, _ = self.step(adapter)
        state = replace(self.state, time_s=.005, generation=2)
        adapter.reset(state, now=.005, previous_current_A=0., accepted_time_s=.005)
        new, _ = adapter.step(self.case.observation(.010, 1, 2),
            replace(self.reference, time_s=.010, generation=2), replace(state, time_s=.010))
        self.assertGreater(new.sequence, old.sequence)
        self.assertFalse(adapter.acknowledge(old, successful=True, accepted_time_s=.011))
        self.assertTrue(adapter.acknowledge(new, successful=True, accepted_time_s=.012))

    def test_invalid_external_ff_directly_latches_native_inhibit(self):
        core, _ = self.open_adapter()
        out = core.step_feedforward(self.case.observation(.005, 1), CReference(), float("nan"))
        self.assertEqual((out.status, out.sequence, out.requested, out.limited), (5, 0, 0., 0.))
        self.assert_native_inhibited(core)

    def test_new_observation_changes_ff_with_unchanged_advisory_state(self):
        model = replace(self.model, load="affine", load_slope=.4)
        _, adapter = self.open_adapter(model=model)
        observation = self.case.observation(.005, 1)
        observation.position = .008
        out, demand = adapter.step(observation, self.reference, replace(self.state, time_s=.005))
        self.assertEqual(demand.load_A, model.load_offset+model.load_slope*out.position)
        self.assertNotEqual(demand.load_A, model.load_offset)
        self.assertEqual(out.velocity, adapter.last_posterior.velocity)
        self.assertEqual(adapter.last_posterior.encoder_time, .005)
        self.assertTrue(adapter.acknowledge(out, successful=True, accepted_time_s=.0052, applied_current_A=.001))
        _, _ = adapter.step(self.case.observation(.01, 2), replace(self.reference, time_s=.01),
                            replace(self.state, time_s=.01))
        self.assertEqual((adapter.last_posterior.accepted_current, adapter.last_posterior.accepted_time,
                          adapter.last_posterior.accepted_actual_time), (.001, .0052, 1))

    def test_opt_in_posterior_policy_crosses_threshold_without_rest_claim(self):
        _, adapter = self.open_adapter()
        context = replace(self.state, friction_state="POSTERIOR_POLICY")
        adapter.reset(context, now=0., previous_current_A=0., accepted_time_s=-.1)
        observation = self.case.observation(.005, 1)
        observation.gyro_rate = .005
        out, demand = adapter.step(observation, self.reference, replace(context, time_s=.005))
        self.assertEqual(demand.phase, "SLIDE")
        self.assertTrue(adapter.acknowledge(out, successful=True, accepted_time_s=.0052))
        observation = self.case.observation(.01, 2)
        observation.gyro_rate = .0005
        _, demand = adapter.step(observation, replace(self.reference, time_s=.01, v_ref_rad_s=-.1),
                                 replace(context, time_s=.01))
        self.assertEqual((demand.phase, demand.direction), ("REVERSE_UNRESOLVED", 1))

    def test_source_freshness_is_checked_on_updated_native_posterior(self):
        core, adapter = self.open_adapter(support=replace(self.support, max_state_age_s=.003))
        observation = self.case.observation(.005, 1)
        observation.encoder_time = observation.gyro_time = 0.
        with self.assertRaises(FeedforwardRejected) as rejected:
            adapter.step(observation, self.reference, replace(self.state, time_s=.005))
        self.assertEqual(rejected.exception.reason, Failure.INVALID_STATE)
        self.assert_native_inhibited(core)

    def test_native_callback_exception_nonfinite_and_missing_callback_inhibit(self):
        def fail(_posterior, _reference):
            raise ValueError("intentional invalid FF")
        for callback in (fail, lambda *_: float("nan"), None):
            with self.subTest(callback=callback):
                core, _ = self.open_adapter()
                with self.assertRaises(ValueError):
                    core.step_posterior_feedforward(self.case.observation(.005, 1), CReference(), callback)
                self.assert_native_inhibited(core)

    def test_callback_cannot_reenter_reset_or_preserve_output_after_inhibit(self):
        for action in ("inhibit", "step", "reset", "close"):
            with self.subTest(action=action):
                core, _ = self.open_adapter()
                def callback(_posterior, _reference):
                    if action == "inhibit":
                        core.inhibit()
                    elif action == "step":
                        self.assertEqual(core.step(self.case.observation(.01, 2), CReference()).status, 5)
                    elif action == "close":
                        core.close()
                    else:
                        with self.assertRaises(ValueError):
                            core.reset(.005, 0., 0., 0.)
                        raise ValueError("reset attempted from callback")
                    return .1
                if action in ("reset", "close"):
                    with self.assertRaises(ValueError):
                        core.step_posterior_feedforward(self.case.observation(.005, 1), CReference(), callback)
                else:
                    out = core.step_posterior_feedforward(self.case.observation(.005, 1), CReference(), callback)
                    self.assertEqual((out.status, out.sequence, out.limited), (5, 0, 0.))
                self.assert_native_inhibited(core)

    def test_c_abi_destroy_inside_callback_defers_until_abort_output_returns(self):
        core, _ = self.open_adapter()
        def callback(_posterior, _reference):
            self.case.native.lib.ota_controller_destroy(core.handle)
            core.handle = None  # caller relinquishes the handle it requested to destroy
            return .1
        out = core.step_posterior_feedforward(self.case.observation(.005, 1), CReference(), callback)
        self.assertEqual((out.status, out.sequence, out.limited), (5, 0, 0.))

    def test_next_observation_cannot_precede_actual_accepted_command(self):
        core, adapter = self.open_adapter()
        out, _ = self.step(adapter)
        self.assertTrue(adapter.acknowledge(out, successful=True, accepted_time_s=.02))
        with self.assertRaises(FeedforwardRejected) as rejected:
            adapter.step(self.case.observation(.01, 2), replace(self.reference, time_s=.01),
                         replace(self.state, time_s=.01))
        self.assertEqual(rejected.exception.reason, Failure.CORE_FAULT)
        self.assert_native_inhibited(core)

    def test_callback_and_constant_ff_share_identical_pi_limiter_ack_behavior(self):
        with Controller(self.case.native, self.case.params) as direct, Controller(self.case.native, self.case.params) as shared:
            for core in (direct, shared):
                core.reset(0., 0., 0., 0., accepted_time=-.1)
            for k in range(1, 21):
                observation = self.case.observation(k*.005, k)
                observation.position, observation.gyro_rate = .1*k*.005, .1
                reference = CReference(.1*k*.005, .1, 0., 0.)
                old = direct.step_feedforward(observation, reference, .126)
                new = shared.step_posterior_feedforward(observation, reference, lambda *_: .126)
                self.assertEqual(bytes(old), bytes(new))
                self.assertTrue(direct.ack(old, applied=old.limited*.9, accepted_time=k*.005+.0002))
                self.assertTrue(shared.ack(new, applied=new.limited*.9, accepted_time=k*.005+.0002))

    def start_candidate(self, **changes):
        values = dict(configuration_id=self.support.configuration_id, source="declared synthetic start interval",
            q_min_rad=-1., q_max_rad=1., static_negative_interval_A=(.15, .17),
            static_positive_interval_A=(.15, .17), negative_excess_A=.02, positive_excess_A=.02,
            max_attempt_s=.2, max_command_dose_A2s=.15)
        return BoundedStartPolicy(**{**values, **changes})

    def test_static_threshold_point_is_not_a_qualified_start_total(self):
        legacy = parameters_for_family(self.case.params, self.model)
        self.assertEqual(legacy.start_policy_qualification, "STATIC_POINT_DIAGNOSTIC_UNQUALIFIED")
        policy = self.start_candidate()
        candidate = parameters_for_family(self.case.params, self.model, start_policy=policy)
        self.assertEqual(candidate.start_policy_qualification, "SYNTHETIC_START_CANDIDATE")
        self.assertAlmostEqual(candidate.start_total[15], (.02+.17+.02-.03)/2.)
        self.assertGreater(candidate.start_total[15], legacy.start_total[15])
        self.assertEqual((candidate.kp, candidate.ki, candidate.kpos, candidate.kaw,
                          candidate.current_cap, candidate.slew),
                         (legacy.kp, legacy.ki, legacy.kpos, legacy.kaw, legacy.current_cap, legacy.slew))
        self.assertLessEqual(candidate.start_timeout_s, legacy.start_timeout_s)

    def test_bounded_start_candidate_requires_interval_count_dose_and_existing_cap(self):
        cases = [dict(static_positive_interval_A=(.15, .16)), dict(static_negative_interval_A=[.15, .17]),
                 dict(positive_excess_A=0.), dict(positive_excess_A=2.), dict(max_attempt_s=.201),
                 dict(max_attempts=True), dict(max_attempts=2), dict(max_command_dose_A2s=.1),
                 dict(qualification="PHYSICAL_QUALIFIED")]
        for values in cases:
            with self.subTest(values=values), self.assertRaises(FeedforwardRejected):
                parameters_for_family(self.case.params, self.model, start_policy=self.start_candidate(**values))

    def test_start_candidate_context_and_native_timeout_cannot_be_bypassed(self):
        policy = self.start_candidate(configuration_id="other-context")
        core = Controller(self.case.native, parameters_for_family(self.case.params, self.model, start_policy=policy))
        self.addCleanup(core.close)
        with self.assertRaises(FeedforwardRejected):
            MotorFeedforward(core, self.model, replace(self.support, start_policy=policy))
        self.assert_native_inhibited(core)

    def test_start_attempt_counter_prevents_repeated_native_restart(self):
        policy = self.start_candidate()
        core, adapter = self.open_adapter(support=replace(self.support, start_policy=policy))
        context = replace(self.state, friction_state="POSTERIOR_POLICY")
        adapter.reset(context, now=0., previous_current_A=0., accepted_time_s=-.1)
        for k in range(1, 15):
            now = k*.005
            observation = self.case.observation(now, k)
            observation.position, observation.gyro_rate = .2*now, .2
            reference = replace(self.reference, time_s=now, source_time_s=now, expires_at_s=now+.02)
            out, _ = adapter.step(observation, reference,
                                 replace(context, time_s=now, q_rad=.2*now, v_rad_s=.2))
            self.assertTrue(adapter.acknowledge(out, successful=True, accepted_time_s=now))
        self.assertEqual((out.motion, adapter.start_attempt_count), (2, 1))
        rejected = False
        for k in range(15, 51):
            now = k*.005
            observation = self.case.observation(now, k)
            observation.position, observation.gyro_rate = .014, 0.
            reference = replace(self.reference, time_s=now, source_time_s=now, expires_at_s=now+.02,
                q_ref_rad=.014, v_ref_rad_s=0. if k < 21 else .8, a_ref_rad_s2=0.)
            try:
                out, _ = adapter.step(observation, reference, replace(context, time_s=now, q_rad=.014))
                self.assertTrue(adapter.acknowledge(out, successful=True, accepted_time_s=now))
            except FeedforwardRejected as exc:
                self.assertEqual(exc.reason, Failure.OUTSIDE_SUPPORT)
                self.assertIn("attempt count", str(exc))
                rejected = True
                break
        self.assertTrue(rejected)
        self.assertEqual(adapter.start_attempt_count, 1)
        self.assert_native_inhibited(core)

    def departure(self, **changes):
        values = dict(q_ref_rad=0., v_ref_rad_s=0., a_ref_rad_s2=0., trajectory_phase="DEPARTURE",
                      planned_direction=1, departure_offset_s=.005, departure_position_rad=0.)
        return replace(self.reference, **{**values, **changes})

    def test_departure_intent_uses_existing_native_start_before_velocity_threshold(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                _, adapter = self.open_adapter(support=replace(self.support, start_policy=self.start_candidate()))
                out, _ = self.step(adapter, self.departure(planned_direction=direction))
                self.assertEqual((out.motion, adapter.start_attempt_count), (1, 1))
                self.assertEqual(out.shaped_reference_velocity, 0.)
                self.assertGreater(direction*out.requested, 0.)

    def test_stationary_phase_retains_legitimate_native_position_correction(self):
        _, adapter = self.open_adapter()
        out, _ = self.step(adapter, replace(self.reference, trajectory_phase="HOLD",
                                            q_ref_rad=.01, v_ref_rad_s=0., a_ref_rad_s2=0.))
        self.assertEqual(out.motion, 1)
        self.assertGreater(out.shaped_reference_velocity, 0.)

    def test_invalid_departure_contract_latches_before_token(self):
        cases = [dict(trajectory_phase="departure"), dict(planned_direction=True), dict(planned_direction=0),
                 dict(departure_offset_s=None), dict(departure_offset_s=-.001),
                 dict(departure_offset_s=.006), dict(departure_offset_s=0., q_ref_rad=.001),
                 dict(departure_position_rad=2.), dict(q_ref_rad=.01), dict(v_ref_rad_s=.1),
                 dict(a_ref_rad_s2=-.1), dict(q_ref_rad=-.001)]
        for values in cases:
            with self.subTest(values=values):
                core, adapter = self.open_adapter(support=replace(self.support, start_policy=self.start_candidate()))
                with self.assertRaises(FeedforwardRejected):
                    self.step(adapter, self.departure(**values))
                self.assert_native_inhibited(core)
        core, adapter = self.open_adapter()
        with self.assertRaises(FeedforwardRejected):
            self.step(adapter, self.departure())
        self.assert_native_inhibited(core)

    def test_departure_anchor_cannot_roll_or_replay_after_phase_progression(self):
        for roll in (True, False):
            with self.subTest(roll=roll):
                core, adapter = self.open_adapter(support=replace(self.support, start_policy=self.start_candidate()))
                reference = self.departure()
                out, _ = self.step(adapter, reference)
                self.assertTrue(adapter.acknowledge(out, successful=True, accepted_time_s=.005))
                if not roll:
                    hold = replace(self.reference, time_s=.010, trajectory_phase="HOLD",
                                   q_ref_rad=0., v_ref_rad_s=0., a_ref_rad_s2=0.)
                    out, _ = adapter.step(self.case.observation(.010, 2), hold, replace(self.state, time_s=.010))
                    self.assertTrue(adapter.acknowledge(out, successful=True, accepted_time_s=.010))
                now = .010 if roll else .015
                bad = replace(reference, time_s=now, departure_offset_s=.010 if roll else .005)
                with self.assertRaises(FeedforwardRejected):
                    adapter.step(self.case.observation(now, 3), bad, replace(self.state, time_s=now))
                self.assert_native_inhibited(core)

    def test_native_departure_rejects_conflicting_start_direction_without_restart(self):
        core, _ = self.open_adapter()
        calls = []
        callback = lambda state, ref: calls.append(state.motion) or 0.
        first = core.step_posterior_feedforward(self.case.observation(.005, 1), CReference(0., 0., 0., 0.),
                                               callback, planned_start_intent=1)
        self.assertEqual(first.motion, 1)
        self.assertTrue(core.ack(first, accepted_time=.005))
        fault = core.step_posterior_feedforward(self.case.observation(.010, 2), CReference(0., 0., 0., 0.),
                                               callback, planned_start_intent=-1)
        self.assertEqual((fault.status, fault.sequence, fault.requested, fault.limited), (5, 0, 0., 0.))
        self.assertEqual(calls, [1])

    def test_native_departure_rejects_malformed_hint_and_opposed_reference(self):
        for hint, reference in ((True, CReference(0., 0., 0., 0.)),
                                (2, CReference(0., 0., 0., 0.)),
                                (1, CReference(0., -.1, 0., 0.)),
                                (-1, CReference(0., 0., .1, 0.))):
            with self.subTest(hint=hint, reference=reference):
                core, _ = self.open_adapter()
                calls = []
                if type(hint) is not int or abs(hint) > 1:
                    with self.assertRaises(ValueError):
                        core.step_posterior_feedforward(self.case.observation(.005, 1), reference,
                                                       lambda state, ref: calls.append(state) or 0., planned_start_intent=hint)
                else:
                    fault = core.step_posterior_feedforward(self.case.observation(.005, 1), reference,
                        lambda state, ref: calls.append(state) or 0., planned_start_intent=hint)
                    self.assertEqual((fault.status, fault.sequence), (5, 0))
                self.assertEqual(calls, [])
                self.assert_native_inhibited(core)

    def test_explicit_braking_uses_stop_without_start_and_hold_retains_position_correction(self):
        core, adapter = self.open_adapter()
        reference = replace(self.reference, q_ref_rad=0., v_ref_rad_s=.1, a_ref_rad_s2=-.2,
                            trajectory_phase="BRAKING")
        out, _ = self.step(adapter, reference)
        self.assertEqual((out.motion, out.start_increment), (3, 0.))
        self.assertTrue(adapter.acknowledge(out, successful=True, accepted_time_s=.005))
        hold = replace(reference, time_s=.010, q_ref_rad=.01, v_ref_rad_s=0., a_ref_rad_s2=0.,
                       trajectory_phase="HOLD")
        corrected, _ = adapter.step(self.case.observation(.010, 2), hold, replace(self.state, time_s=.010))
        self.assertEqual(corrected.motion, 1)
        self.assertGreater(corrected.shaped_reference_velocity, 0.)

    def test_explicit_braking_cannot_accelerate_or_supply_departure_direction(self):
        for phase, hint, reference in ((True, 0, CReference(0., 0., 0., 0.)),
                                       (3, 0, CReference(0., 0., 0., 0.)),
                                       (1, 0, CReference(0., 0., 0., 0.)),
                                       (2, 1, CReference(0., .1, -.1, 0.)),
                                       (2, 0, CReference(0., .1, .1, 0.)),
                                       (2, 0, CReference(.01, 0., 0., 0.)),
                                       (2, 0, CReference(0., .1, 0., 0.)),
                                       (2, 0, CReference(0., 0., -.1, 0.))):
            with self.subTest(phase=phase, hint=hint):
                core, _ = self.open_adapter()
                calls = []
                if phase == 2 and hint == 0:
                    out = core.step_posterior_feedforward(self.case.observation(.005, 1), reference,
                        lambda state, ref: calls.append(state) or 0., reference_phase=phase, planned_start_intent=hint)
                    self.assertEqual((out.status, out.sequence), (5, 0))
                else:
                    with self.assertRaises(ValueError):
                        core.step_posterior_feedforward(self.case.observation(.005, 1), reference,
                            lambda state, ref: calls.append(state) or 0., reference_phase=phase, planned_start_intent=hint)
                self.assertEqual(calls, [])
                self.assert_native_inhibited(core)

    def test_explicit_phase_callback_inhibit_cannot_issue_token(self):
        core, _ = self.open_adapter()
        def callback(state, reference):
            self.assertEqual(state.motion, 3)
            core.inhibit()
            return .1
        out = core.step_posterior_feedforward(self.case.observation(.005, 1), CReference(0., .1, -.1, 0.),
                                            callback, reference_phase=2)
        self.assertEqual((out.status, out.sequence, out.requested, out.limited), (5, 0, 0., 0.))

    def test_explicit_legacy_phase_preserves_entire_native_output(self):
        import ctypes as ct
        from Firmware.commissioning.native import COutput, POSTERIOR_FEEDFORWARD_CALLBACK
        first, _ = self.open_adapter()
        second, _ = self.open_adapter()
        callback = POSTERIOR_FEEDFORWARD_CALLBACK(lambda context, state, ref, command: (command.__setitem__(0, .1), 1)[1])
        for k in range(1, 15):
            observation = self.case.observation(k*.005, k)
            reference = CReference(0., .1, 0., 0.)
            legacy = first.step_posterior_feedforward(observation, reference, lambda state, ref: .1)
            explicit = COutput()
            self.assertTrue(self.case.native.controller_step_posterior_ff_phase(second.handle,
                ct.byref(observation), ct.byref(reference), 0, 0, callback, None, ct.byref(explicit)))
            self.assertEqual(bytes(legacy), bytes(explicit))
            self.assertTrue(first.ack(legacy, accepted_time=k*.005))
            self.assertTrue(second.ack(explicit, accepted_time=k*.005))


if __name__ == "__main__":
    unittest.main()
