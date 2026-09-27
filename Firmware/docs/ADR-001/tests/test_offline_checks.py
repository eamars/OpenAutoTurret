"""Only synthetic/offline contract examples; not repository or hardware tests."""
import contextlib
import copy
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import offline_checks as c


def example(name):
    return c.strict_json((ROOT / 'examples' / name).read_text(encoding='utf-8'))


class TimestampTests(unittest.TestCase):
    def test_zero(self): self.assertEqual(c.ns('0'), 0)
    def test_int64_limit(self): self.assertEqual(c.ns(str(2**63-1)), 2**63-1)
    def test_int64_overflow(self):
        with self.assertRaises(ValueError): c.ns(str(2**63))
    def test_integer_not_string(self):
        with self.assertRaises(ValueError): c.ns(1)
    def test_float_not_string(self):
        with self.assertRaises(ValueError): c.ns(1.0)
    def test_boolean(self):
        with self.assertRaises(ValueError): c.ns(True)
    def test_noncanonical(self):
        for value in ('01','-1','+1','1.0','1e9',' 1','', '١'):
            with self.subTest(value=value), self.assertRaises(ValueError): c.ns(value)
    def test_finite_rejects_nan(self): self.assertFalse(c.finite(float('nan')))
    def test_finite_rejects_bool(self): self.assertFalse(c.finite(True))
    def test_finite_huge_int_no_crash(self): self.assertFalse(c.finite(10**1000))


class PowerTests(unittest.TestCase):
    def test_current_and_history(self):
        r=c.decode_throttled('throttled=0x50005')
        self.assertEqual(r['state'],'current_fault')
        self.assertTrue(r['current']['undervoltage']);self.assertTrue(r['current']['throttled'])
        self.assertTrue(r['history']['undervoltage']);self.assertTrue(r['history']['throttled'])
        self.assertFalse(r['current']['frequency_capped'])
    def test_history_not_current(self): self.assertEqual(c.decode_throttled('0x50000')['state'],'history_only')
    def test_clear(self): self.assertEqual(c.decode_throttled('0x0')['state'],'clear')
    def test_unknown_not_ignored(self): self.assertEqual(c.decode_throttled('0x10')['state'],'unknown_bits')
    def test_integer_input(self): self.assertEqual(c.decode_throttled(5)['state'],'current_fault')
    def test_malformed(self):
        for x in (-1, 2**32, True, '50005', '0x5 garbage'):
            with self.subTest(x=x), self.assertRaises(ValueError): c.decode_throttled(x)


class GeometryTests(unittest.TestCase):
    def test_letterbox_full_source(self):
        self.assertEqual(c.invert_center_letterbox_yxyx((.125,0,.875,1)),(0.,0.,640.,480.))
    def test_letterbox_interior(self):
        self.assertEqual(c.invert_center_letterbox_yxyx((.25,.25,.75,.75)),(160.,80.,480.,400.))
    def test_letterbox_padding_only(self): self.assertIsNone(c.invert_center_letterbox_yxyx((0,0,.1,1)))
    def test_letterbox_clips(self): self.assertEqual(c.invert_center_letterbox_yxyx((-.2,-.1,1.2,1.1)),(0.,0.,640.,480.))
    def test_letterbox_scaled_source(self):
        self.assertEqual(c.invert_center_letterbox_yxyx((.125,0,.875,1),source_width=1280,source_height=960),(0.,0.,1280.,960.))
    def test_reversed_box(self): self.assertIsNone(c.invert_center_letterbox_yxyx((.8,.2,.2,.8)))
    def test_nan_box(self):
        with self.assertRaises(ValueError): c.invert_center_letterbox_yxyx((0,0,math.nan,1))
    def test_wrong_box_size(self):
        with self.assertRaises(ValueError): c.invert_center_letterbox_yxyx((0,1,2))
    def test_bad_dimensions(self):
        with self.assertRaises(ValueError): c.invert_center_letterbox_yxyx((0,0,1,1),source_width=0)
    def test_rotate_corner(self): self.assertEqual(c.rotate_180_pixel(0,0,640,480),(639,479))
    def test_rotate_center(self): self.assertEqual(c.rotate_180_pixel(319.5,239.5,640,480),(319.5,239.5))
    def test_rotate_involution(self):
        p=c.rotate_180_pixel(102,76,640,480);self.assertEqual(c.rotate_180_pixel(*p,640,480),(102,76))
    def test_rotate_edge_outside(self):
        with self.assertRaises(ValueError): c.rotate_180_pixel(640,0,640,480)


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.obs=example('selected_observation.json');self.ctx=example('validation_context.json')
    def reject(self,reason): self.assertIn(reason,c.observation_rejections(self.obs,self.ctx))
    def test_valid_synthetic(self): self.assertEqual(c.observation_rejections(self.obs,self.ctx),[])
    def test_missing_field(self):
        del self.obs['frame_id'];self.reject('missing:frame_id')
    def test_wrong_schema(self): self.obs['schema_version']='unknown';self.reject('unsupported_schema')
    def test_station_reset(self): self.obs['station_session_id']='new';self.reject('mismatch:station_session_id')
    def test_boot_reset(self): self.obs['boot_id']='new';self.reject('mismatch:boot_id')
    def test_clock_epoch_reset(self): self.obs['clock_epoch']=1;self.reject('mismatch:clock_epoch')
    def test_unmapped_clock(self): self.obs['clock_id']='CLOCK_BOOTTIME';self.reject('unmapped_clock')
    def test_camera_restart(self): self.obs['camera_generation']=2;self.reject('mismatch:camera_generation')
    def test_frame_duplicate(self): self.obs['source_frame_sequence']='42';self.reject('duplicate_or_old_frame')
    def test_frame_older(self): self.obs['source_frame_sequence']='41';self.reject('duplicate_or_old_frame')
    def test_frame_sequence_number_rejected(self): self.obs['source_frame_sequence']=43;self.reject('invalid_time_or_context')
    def test_stale(self): self.ctx['now_ns']='1200000000';self.reject('stale_observation')
    def test_expiry_boundary(self): self.ctx['now_ns']='1150000000';self.reject('expired_observation')
    def test_future_publish(self): self.obs['t_publish_ns']='1060000000';self.reject('noncausal_or_future_time')
    def test_capture_before_observation(self): self.obs['t_capture_received_ns']='999000000';self.reject('noncausal_or_future_time')
    def test_overlong_ttl(self): self.obs['valid_until_ns']='1300000000';self.reject('ttl_exceeds_policy')
    def test_unknown_timestamp(self): self.obs['observation_time_quality']='unknown';self.reject('unverified_timestamp')
    def test_excess_timestamp_uncertainty(self): self.obs['timestamp_uncertainty_ns']='6000000';self.reject('timestamp_uncertainty_excessive')
    def test_shadow_geometry(self): self.obs['geometry_valid']=False;self.reject('unqualified_geometry')
    def test_new_calibration(self): self.obs['calibration_id']='new';self.reject('mismatch:calibration_id')
    def test_wrong_model(self): self.obs['model_id']='new';self.reject('mismatch:model_id')
    def test_wrong_sensor_mode(self): self.obs['sensor_mode_id']='new';self.reject('mismatch:sensor_mode_id')
    def test_wrong_transform(self): self.obs['transform_chain_id']='new';self.reject('mismatch:transform_chain_id')
    def test_unqualified_camera(self): self.ctx['cameras']['SYNTHETIC-wide']['motion_qualified']=False;self.reject('camera_not_qualified')
    def test_non_authoritative_camera(self): self.ctx['allowed_camera_ids']=[];self.reject('camera_not_authoritative')
    def test_unknown_camera(self): self.obs['camera_id']='other';self.reject('unknown_camera')
    def test_stale_selection(self): self.obs['selection']['selection_generation']=2;self.reject('selection_mismatch:selection_generation')
    def test_wrong_selected_person(self): self.obs['selection']['global_track_id']='SYNTHETIC-person-B';self.reject('selection_mismatch:global_track_id')
    def test_bool_generation(self): self.obs['selection']['selection_generation']=True;self.reject('invalid_selection_generation')
    def test_bad_anchor_kind(self): self.obs['anchor']['kind']='face_identity';self.reject('invalid_anchor_kind')
    def test_bad_anchor_kind_type(self): self.obs['anchor']['kind']=[];self.reject('invalid_anchor_kind')
    def test_pixel_outside_raster(self): self.obs['anchor']['u_px']=640;self.reject('invalid_anchor_raster')
    def test_anchor_nan(self): self.obs['anchor']['v_px']=float('nan');self.reject('invalid_anchor_raster')
    def test_confidence_not_probability(self): self.obs['person_confidence']=1.1;self.reject('invalid:person_confidence')
    def test_bad_camera_context(self): self.ctx['cameras']=[];self.reject('invalid_camera_context')
    def test_malformed_camera_id(self): self.obs['camera_id']=[];self.reject('invalid_identifier')
    def test_input_not_object(self): self.assertEqual(c.observation_rejections([],{}),['malformed_object'])


class SelectionTests(unittest.TestCase):
    def test_auto_candidate(self): self.assertTrue(c.SelectionLatch().candidate_allowed('A'))
    def test_explicit_rejects_other(self):
        s=c.SelectionLatch();s.explicit_select('A');self.assertFalse(s.candidate_allowed('B'));self.assertTrue(s.candidate_allowed('A'))
    def test_roam_does_not_clear(self):
        s=c.SelectionLatch();s.explicit_select('A');s.on_mode_change('AUTO_ROAM');self.assertFalse(s.candidate_allowed('B'));self.assertEqual(s.generation,1)
    def test_manual_does_not_clear(self):
        s=c.SelectionLatch();s.explicit_select('A');s.on_mode_change('MANUAL');self.assertFalse(s.candidate_allowed('B'))
    def test_cancel_is_new_generation(self):
        s=c.SelectionLatch();s.explicit_select('A');s.cancel();self.assertEqual(s.generation,2);self.assertTrue(s.candidate_allowed('B'))
    def test_reselect_same_new_generation(self):
        s=c.SelectionLatch();s.explicit_select('A');s.explicit_select('A');self.assertEqual(s.generation,2)
    def test_empty_target(self):
        with self.assertRaises(ValueError): c.SelectionLatch().explicit_select(' ')
    def test_unknown_mode(self):
        with self.assertRaises(ValueError): c.SelectionLatch().on_mode_change('UNBOUNDED')


class StopTests(unittest.TestCase):
    def setUp(self): self.e=example('stop_evidence.json')
    def test_limited_never_isolated(self):
        r=c.evaluate_stop_evidence(self.e);self.assertEqual(r['completion_quality'],'limited_complete');self.assertIsNone(r['yaw_disable_confirmed']);self.assertIsNone(r['power_isolated_confirmed'])
    def test_yaw_disable_claim_rejected(self):
        self.e['yaw']['disable_confirmed']=True;self.assertIn('yaw:unsupported_disable_claim',c.evaluate_stop_evidence(self.e)['missing_evidence'])
    def test_stale_pitch(self):
        self.e['pitch']['feedback_age_ms']=101;self.assertEqual(c.evaluate_stop_evidence(self.e)['completion_quality'],'incomplete')
    def test_missing_yaw(self):
        del self.e['yaw'];self.assertEqual(c.evaluate_stop_evidence(self.e)['completion_quality'],'incomplete')
    def test_pitch_disable_unconfirmed(self):
        self.e['pitch']['disable_confirmed']=None;self.assertIn('pitch:disable_confirmation',c.evaluate_stop_evidence(self.e)['missing_evidence'])
    def test_negative_age(self):
        self.e['yaw']['feedback_age_ms']=-1;self.assertIn('yaw:fresh_feedback',c.evaluate_stop_evidence(self.e)['missing_evidence'])
    def test_no_stationary_observation(self):
        self.e['pitch']['stationary_observed']=False;self.assertIn('pitch:stationary_observation',c.evaluate_stop_evidence(self.e)['missing_evidence'])
    def test_zero_not_requested(self):
        self.e['yaw']['zero_requested']=False;self.assertIn('yaw:zero_request',c.evaluate_stop_evidence(self.e)['missing_evidence'])


class TraceTests(unittest.TestCase):
    def setUp(self): self.records=c.read_ndjson(ROOT/'examples/synthetic_trace.ndjson')
    def test_expected_quantiles(self):
        r=c.summarize_trace(self.records);self.assertEqual(r['frame_groups'],4)
        m=r['metrics']['capture_to_controller'];self.assertEqual(m['n'],2);self.assertEqual(m['p50_ms'],28);self.assertEqual(m['p95_ms'],34)
    def test_unknown_not_zero(self):
        m=c.summarize_trace(self.records)['metrics']['capture_to_controller'];self.assertEqual(m['unusable_time_pairs'],1);self.assertEqual(m['missing_pairs'],1)
    def test_report_not_qualification(self): self.assertEqual(c.summarize_trace(self.records)['qualification'],'NOT_EVALUATED')
    def test_camera_ids_separate_same_frame(self): self.assertEqual(c.summarize_trace(self.records)['frame_groups'],4)
    def test_epoch_ids_not_joined(self):
        other=copy.deepcopy(self.records)
        for e in other:e['clock_epoch']=1
        self.assertEqual(c.summarize_trace(self.records+other)['frame_groups'],8)
    def test_camera_generations_not_joined(self):
        other=copy.deepcopy(self.records)
        for e in other:e['camera_generation']=2
        self.assertEqual(c.summarize_trace(self.records+other)['frame_groups'],8)
    def test_invalid_camera_generation(self):
        self.records[0]['camera_generation']=True
        with self.assertRaises(ValueError): c.summarize_trace(self.records)
    def test_duplicate_event_excludes_group(self):
        r=c.summarize_trace(self.records+[self.records[0]]);self.assertEqual(r['invalid_frame_groups'],1);self.assertEqual(r['metrics']['capture_to_controller']['n'],1)
    def test_negative_duration_excludes_group(self):
        self.records[3]['t_ns']='2005000000';self.assertEqual(c.summarize_trace(self.records)['invalid_frame_groups'],1)
    def test_unmapped_clock_rejected(self):
        self.records[0]['clock_id']='CLOCK_BOOTTIME'
        with self.assertRaises(ValueError): c.summarize_trace(self.records)
    def test_timestamp_number_rejected(self):
        self.records[0]['t_ns']=2000000000
        with self.assertRaises(ValueError): c.summarize_trace(self.records)
    def test_empty_samples_are_null(self): self.assertIsNone(c.quantiles([])['p95_ms'])
    def test_nearest_rank(self):
        r=c.quantiles(range(1,101));self.assertEqual(r['p50_ms'],50);self.assertEqual(r['p95_ms'],95);self.assertEqual(r['p99_ms'],99)
    def test_negative_latency_rejected(self):
        with self.assertRaises(ValueError): c.quantiles([-1])
    def test_nan_latency_rejected(self):
        with self.assertRaises(ValueError): c.quantiles([float('nan')])
    def test_unknown_event_counted(self):
        r=copy.deepcopy(self.records[0]);r['event']='new_stage';self.assertEqual(c.summarize_trace(self.records+[r])['unknown_events'],1)
    def test_invalid_epoch(self):
        self.records[0]['clock_epoch']=True
        with self.assertRaises(ValueError): c.summarize_trace(self.records)
    def test_nonobject(self):
        with self.assertRaises(ValueError): c.summarize_trace([[]])


class ParsingAndCliTests(unittest.TestCase):
    def test_nan_literal_rejected(self):
        with self.assertRaises(ValueError): c.strict_json('{"x":NaN}')
    def test_float_overflow_rejected(self):
        with self.assertRaises(ValueError): c.strict_json('{"x":1e999}')
    def test_duplicate_key_rejected(self):
        with self.assertRaises(ValueError): c.strict_json('{"x":1,"x":2}')
    def test_ndjson_nonobject_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'bad.ndjson';p.write_text('[]\n')
            with self.assertRaises(ValueError): c.read_ndjson(p)
    def test_cli_valid_fixture(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            code=c.main(['check-observation',str(ROOT/'examples/selected_observation.json'),str(ROOT/'examples/validation_context.json')])
        self.assertEqual(code,0);self.assertEqual(json.loads(out.getvalue())['qualification'],'NOT_EVALUATED')
    def test_cli_summary(self):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            code=c.main(['summarize',str(ROOT/'examples/synthetic_trace.ndjson')])
        self.assertEqual(code,0);self.assertEqual(json.loads(out.getvalue())['frame_groups'],4)
    def test_cli_missing_file(self):
        with contextlib.redirect_stderr(io.StringIO()): code=c.main(['summarize','/NONEXISTENT-SYNTHETIC-FILE.ndjson'])
        self.assertEqual(code,2)


if __name__=='__main__': unittest.main()
