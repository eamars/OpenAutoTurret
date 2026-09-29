from __future__ import annotations
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import control_math as cm
import trace_metrics as tm

class ControlMathTests(unittest.TestCase):
    def test_blocked_axis_5_deg_example(self):
        self.assertAlmostEqual(cm.blocked_axis_time(5, 1, .6, .4, .8), 5.972770601744311, places=8)
    def test_reverse_direction_same_magnitude(self):
        self.assertEqual(cm.blocked_axis_time(-5, 1, .6, .4, .8), cm.blocked_axis_time(5, 1, .6, .4, .8))
    def test_current_cap_makes_threshold_unreachable(self):
        self.assertIsNone(cm.blocked_axis_time(5, 1, .6, .9, .8))
    def test_zero_ki_no_accumulation(self):
        self.assertIsNone(cm.blocked_axis_time(5, 1, 0, .4, .8))
    def test_proportional_already_sufficient(self):
        self.assertEqual(cm.blocked_axis_time(30, 1, 0, .4, .8), 0)
    def test_no_motion_demand_does_not_accumulate(self):
        self.assertIsNone(cm.blocked_axis_time(0, 1, .6, .4, .8))
    def test_nan_rejected(self):
        with self.assertRaises(ValueError): cm.blocked_axis_time(math.nan, 1, .6, .4, .8)
    def test_negative_gain_rejected(self):
        with self.assertRaises(ValueError): cm.blocked_axis_time(5, -1, .6, .4, .8)
    def test_encoder_quantum(self):
        self.assertAlmostEqual(cm.encoder_speed_quantum_deg_s(.005), 8.7890625)
    def test_invalid_encoder_period(self):
        with self.assertRaises(ValueError): cm.encoder_speed_quantum_deg_s(0)
    def test_rx_slope(self):
        self.assertAlmostEqual(cm.rx_window_velocity([(0,0.),(10_000_000,.02),(20_000_000,.04)]), 2)
    def test_identical_rx_duplicate_ignored(self):
        self.assertAlmostEqual(cm.rx_window_velocity([(0,0.),(0,0.),(20_000_000,.04)]),2)
    def test_conflicting_rx_duplicate_rejected(self):
        with self.assertRaises(ValueError): cm.rx_window_velocity([(0,0.),(0,.1)])
    def test_reversed_rx_rejected(self):
        with self.assertRaises(ValueError): cm.rx_window_velocity([(2,0.),(1,.1)])
    def test_one_rx_not_zero_velocity(self):
        self.assertIsNone(cm.rx_window_velocity([(100,2.)]))
    def test_large_integer_clock_precision(self):
        origin=10**18
        self.assertAlmostEqual(cm.rx_window_velocity([(origin,1.),(origin+1000,1.001)]),1000, places=6)
    def test_aw_sees_final_output(self):
        self.assertAlmostEqual(cm.current_aw_step(.5, .6, .1, .005, 20, 1.5, .8, .8), .4303)
    def test_aw_integral_bound(self):
        self.assertEqual(cm.current_aw_step(.79, 20, 100, .005, 0, 0, 0, .8), .8)
    def test_aw_invalid_dt(self):
        with self.assertRaises(ValueError): cm.current_aw_step(0,1,1,0,1,1,1,1)


def sample(t=1_000_000_000, **changes):
    row={'t_ns':t,'rx_ns':t-1_000_000,'axis':'yaw','trial_id':'T1','phase':'steady',
         'drive_mode':'current','q_ref_rad':0.,'v_ref_rad_s':.1,'q_rad':0.,'v_est_rad_s':.1,
         'clamped':0,'guard_action':'RUN','u_applied_a':.4,'iq_a':None,
         'temperature_c':None,'bus_voltage_v':24.}
    row.update(changes)
    return row

class TraceTests(unittest.TestCase):
    def parse(self, rows, fields=None):
        fields=fields or list(sample().keys())
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'trace.csv'
            with path.open('w',newline='',encoding='utf-8') as f:
                writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore')
                writer.writeheader(); writer.writerows(rows)
            return tm.load_rows(path)
    def test_percentile_empty(self): self.assertIsNone(tm.percentile([],95))
    def test_percentile_interpolated(self): self.assertEqual(tm.percentile([1,3],50),2)
    def test_percentile_nonfinite_rejected(self):
        with self.assertRaises(ValueError): tm.percentile([math.nan],50)
    def test_missing_header_field(self):
        with self.assertRaises(ValueError): self.parse([sample()], [k for k in sample() if k!='axis'])
    def test_nan_trace_rejected(self):
        with self.assertRaises(ValueError): self.parse([sample(q_rad='NaN')])
    def test_infinite_optional_not_unknown(self):
        with self.assertRaises(ValueError): self.parse([sample(iq_a='inf')])
    def test_missing_iq_is_unknown(self):
        rows=self.parse([sample(),sample(1_005_000_000)])
        metric=tm.summarize_group(rows,.001)
        self.assertIsNone(metric['measured_current_a']['rms'])
        self.assertEqual(metric['measured_current_a']['coverage_fraction'],0)
    def test_control_time_must_increase_per_axis(self):
        with self.assertRaises(ValueError): self.parse([sample(),sample()])
    def test_rx_time_can_repeat(self):
        rows=self.parse([sample(),sample(1_005_000_000,rx_ns=999_000_000)])
        self.assertIsNone(tm.summarize_group(rows,.001)['observed_distinct_feedback_hz_NOT_total_CAN_hz'])
    def test_rx_time_cannot_reverse(self):
        with self.assertRaises(ValueError): self.parse([sample(),sample(1_005_000_000,rx_ns=998_000_000)])
    def test_future_rx_is_preserved_and_flagged(self):
        rows=self.parse([sample(rx_ns=1_001_000_000)])
        m=tm.summarize_group(rows,.001)
        self.assertEqual(m['rx_later_than_cycle_start_rows'],1)
        self.assertEqual(m['feedback_age_ms']['p50'],-1)
    def test_pitch_speed_current_command_unknown(self):
        rows=self.parse([sample(axis='pitch',drive_mode='speed',u_applied_a=None)])
        self.assertIsNone(rows[0]['u_applied_a'])
    def test_pitch_speed_cannot_fake_current_command(self):
        with self.assertRaises(ValueError): self.parse([sample(axis='pitch',drive_mode='speed',u_applied_a=0)])
    def test_voltage_count_not_ampere(self):
        with self.assertRaises(ValueError): self.parse([sample(drive_mode='voltage')])
    def test_zoh_weighted_rms_not_sample_average(self):
        rows=[sample(1_000_000_000),sample(2_000_000_000),sample(5_000_000_000)]
        self.assertAlmostEqual(tm.weighted_rms(rows,[1,3,0])['rms'],math.sqrt(7))
    def test_missing_weighted_coverage(self):
        rows=[sample(1_000_000_000),sample(2_000_000_000),sample(5_000_000_000)]
        r=tm.weighted_rms(rows,[1,None,0])
        self.assertEqual(r['rms'],1); self.assertEqual(r['coverage_fraction'],.25)
    def test_single_sample_no_duration_rms(self):
        self.assertIsNone(tm.weighted_rms([sample()],[1])['rms'])
    def test_guard_counts_episodes_not_ticks(self):
        rows=[sample(1_000_000_000),sample(1_005_000_000,guard_action='LIMITED'),sample(1_010_000_000,guard_action='LIMITED')]
        self.assertEqual(tm.summarize_group(rows,.001)['guard_label_episode_counts'],{'RUN':1,'LIMITED':1})
    def test_nonconsecutive_same_phase_not_bridged(self):
        rows=[sample(1_000_000_000),sample(1_005_000_000,phase='brake'),sample(1_010_000_000)]
        self.assertEqual(len(tm.analyze(rows,.001)['groups']),3)
    def test_two_axes_same_control_timestamp(self):
        rows=self.parse([sample(),sample(axis='pitch',drive_mode='speed',u_applied_a=None)])
        self.assertEqual(len(tm.analyze(rows,.001)['groups']),2)
    def test_no_implicit_qualification(self):
        r=tm.analyze([sample()],.001)
        self.assertFalse(r['hardware_qualified_by_this_tool'])
        json.dumps(r,allow_nan=False)
    def test_temperature_slope_not_equilibrium_flag(self):
        rows=[sample(1_000_000_000,temperature_c=30.),sample(61_000_000_000,temperature_c=32.)]
        self.assertAlmostEqual(tm.temperature_slope(rows),2)
    def test_no_motion_latency_is_null(self):
        rows=[sample(),sample(1_005_000_000)]
        self.assertIsNone(tm.summarize_group(rows,.001)['first_motion_threshold_crossing_ms_NOT_confirmed_onset'])
    def test_positive_motion_latency(self):
        rows=[sample(),sample(1_005_000_000,q_rad=.002)]
        self.assertEqual(tm.summarize_group(rows,.001)['first_motion_threshold_crossing_ms_NOT_confirmed_onset'],5)
    def test_invalid_threshold_rejected(self):
        with self.assertRaises(ValueError): tm.summarize_group([sample()],0)
    def test_empty_csv_rejected(self):
        with self.assertRaises(ValueError): self.parse([])

if __name__=='__main__': unittest.main()
