"""Native-rate/provenance boundaries for canonical whole-run yaw captures."""
from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from Firmware.commissioning.contracts import Rejected
from Firmware.commissioning.yaw_events import (
    ENCODER_QUANTUM_RAD, check_physical_run_split, imu_report_clock, load_yaw_journal,
)


CALIBRATION = {"yaw_column": [0., 0., 1.], "baseline_sensor_bias": [0., 0., .1],
               "sensor_internal_filter": "unknown", "added_gyro_filter_tau_s": 0.}


def journal_rows():
    begin = 1_000_000_000
    rows = [{"kind": "header", "provenance": "MEASURED", "manifest_yaml": json.dumps(
            {"session_label": "physical-A", "candidate_label": "configuration-A"})},
            {"kind": "session_begin", "time_ns": begin},
            {"kind": "yaw_current_tx", "phase": "baseline", "requested_A": .2,
             "limited_A": .1, "successful_tx_A": .099, "success": True,
             "begin_ns": begin + 40_000_000, "kernel_accepted_ns": begin + 50_000_000}]
    for number, (offset, count) in enumerate(((100, 8190), (200, 8191), (300, 1), (400, 0)), 1):
        stamp = begin + offset * 1_000_000
        rows.extend([{"kind": "can_rx", "axis": "yaw", "generation": 7, "sequence": number,
                      "kernel_monotonic_ns": stamp, "clock_uncertainty_ns": 100,
                      "bytes": [count >> 8, count & 255, 0, 0, 0, 1, 25, 0]},
                     {"kind": "yaw_feedback", "encoder_raw": count, "q_relative_rad": 23.,
                      "current_raw": 1, "current_A": .012, "kernel_monotonic_ns": stamp,
                      "dequeue_ns": stamp + 3000}])
    for sequence, offset in enumerate((150, 250, 350), 1):
        stamp = begin + offset * 1_000_000
        rows.append({"kind": "imu_raw", "dequeue_ns": stamp + 9000, "raw_json": json.dumps(
                     {"kind": "sample", "sensor": "gyro", "generation": 3, "sequence": sequence,
                      "sample_ns": stamp, "rx_ns": stamp + 5000, "sh2_us": offset * 1000,
                      "status": 0, "values": [0., 0., .1 + sequence]})})
    rows.extend([{"kind": "yaw_current_tx", "phase": "control", "requested_A": .9,
                  "limited_A": .3, "successful_tx_A": .3, "success": False,
                  "begin_ns": begin + 220_000_000, "kernel_accepted_ns": begin + 221_000_000},
                 {"kind": "yaw_current_tx", "phase": "stop", "requested_A": 0., "limited_A": 0.,
                  "successful_tx_A": 0., "success": True,
                  "begin_ns": begin + 390_000_000, "kernel_accepted_ns": begin + 395_000_000},
                 {"kind": "footer", "status": "FAILED_START", "censored": True}])
    return rows


class YawEventsTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "capture.jsonl"

    def load(self, rows=None, **kwargs):
        rows = journal_rows() if rows is None else rows
        self.path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
        return load_yaw_journal(self.path, gyro_calibration=CALIBRATION, encoder_datum_count=0,
                                calibration_revision="calibration-fixed", **kwargs)

    def test_channels_raw_parents_and_censored_failure_survive(self):
        journal = self.load(archive_member="raw_archive/physical-A/capture.jsonl")
        originals = [e for e in journal.events if e["channel"] == "raw"]
        self.assertEqual(len(originals), len(journal_rows()))
        self.assertEqual(originals[-1]["raw"]["status"], "FAILED_START")
        self.assertTrue(originals[-1]["raw"]["censored"])
        self.assertEqual(len(journal.streams["requested_current"]), 3)
        self.assertEqual(len(journal.streams["limited_current"]), 3)
        self.assertEqual(len(journal.streams["successful_tx"]), 2)
        self.assertEqual(journal.streams["requested_current"][0]["value_A"], .2)
        self.assertEqual(journal.streams["limited_current"][0]["value_A"], .1)
        self.assertEqual(journal.streams["successful_tx"][0]["value_A"], .099)
        self.assertEqual(journal.streams["decoded_current"][0]["value_A"], .012)
        encoder = journal.streams["encoder"][0]
        self.assertEqual(encoder["original_line"], 5)
        self.assertEqual(encoder["archive_member"], "raw_archive/physical-A/capture.jsonl")
        self.assertEqual(encoder["generation"], 7)
        self.assertEqual(encoder["parents"][0]["original_line"], 5)
        self.assertEqual(encoder["parents"][-1]["original_line"], 4)

    def test_coordinates_and_native_clock_identities_are_separate(self):
        journal = self.load()
        arrays = journal.fitter_arrays()
        np.testing.assert_allclose(arrays["encoder_q_rad"], np.array([-2, -1, 1, 0]) * ENCODER_QUANTUM_RAD)
        np.testing.assert_allclose(arrays["encoder_session_relative_rad"], np.array([0, 1, 3, 2]) * ENCODER_QUANTUM_RAD)
        self.assertIsNone(journal.registration["world_frame_angle"])
        self.assertFalse(journal.registration["cross_session_registration_qualified"])
        self.assertEqual(journal.streams["encoder"][0]["logged_position_rad"], 23.)
        gyro = journal.streams["gyro"][0]
        self.assertEqual(gyro["timestamp"]["time_ns"], 1_150_000_000)
        self.assertEqual(gyro["timestamp"]["receive_ns"], 1_150_005_000)
        self.assertEqual(gyro["timestamp"]["dequeue_ns"], 1_150_009_000)
        self.assertEqual(gyro["timestamp"]["native_sh2_us"], 150_000)
        self.assertEqual(gyro["timestamp"]["mapping_status"], "HOST_POLL_REPORT_TIME_EPOCH_LIFT_UNVERIFIED")
        self.assertFalse(gyro["timestamp"]["epoch_lift_verified"])
        self.assertEqual(gyro["status"], 0)
        self.assertTrue(gyro["validity"]["valid"])
        np.testing.assert_allclose(arrays["gyro_rad_s"], [1., 2., 3.])
        self.assertEqual(arrays["calibration_support"]["gyro_internal_filter"], "unknown")
        self.assertFalse(arrays["calibration_support"]["physical_qualification"])

    def test_union_counts_native_samples_once_and_keeps_input_prehistory(self):
        journal = self.load()
        union = journal.union_observations(delay_bound_s=.08)
        self.assertEqual(union["q_new"].sum(), 3)
        self.assertEqual(union["v_new"].sum(), 3)
        self.assertEqual(union["current_new"].sum(), 3)
        self.assertTrue(np.isnan(union["q_obs"][~union["q_new"]]).all())
        self.assertTrue(np.isnan(union["v_obs"][~union["v_new"]]).all())
        self.assertLess(union["tx_t"][0], -.08)
        self.assertEqual(union["initial_support"]["state_resets_after_initialization"], 0)
        self.assertTrue(union["initial_support"]["earlier_raw_events_retained"])
        self.assertEqual(union["initial_support"]["observations"]["encoder"]["time_s_from_session_begin"], .1)
        self.assertEqual(union["initial"][0], -2 * ENCODER_QUANTUM_RAD)
        self.assertEqual(union["initial"][1], 1.)

    def test_run_relabelling_does_not_defeat_split_guard(self):
        journal = self.load()
        with self.assertRaises(Rejected):
            check_physical_run_split([journal], [], [replace(journal, physical_run_id="renamed-window")])
        elsewhere = replace(journal, source_journal=str(self.path.with_name("other.jsonl")))
        with self.assertRaises(Rejected):
            check_physical_run_split([journal], [], [elsewhere])
        with self.assertRaises(Rejected):
            check_physical_run_split([journal], [], [replace(elsewhere, physical_run_id="renamed-copy")])
        distinct = replace(elsewhere, physical_run_id="physical-B", origin_ns=journal.origin_ns + 1,
                           manifest={**journal.manifest, "session_label": "physical-B"})
        report = check_physical_run_split([journal], [], [distinct])
        self.assertEqual(report["split_unit"], "complete_physical_journal")
        self.assertFalse(report["prospective_validation"])

    def test_generation_reset_is_retained_and_rejects_one_state_rollout(self):
        rows = journal_rows()
        for row in rows:
            if row.get("kind") == "can_rx" and row["sequence"] >= 3:
                row["generation"] = 8
        journal = self.load(rows)
        self.assertEqual(len(journal.streams["encoder"]), 4)
        self.assertEqual(journal.streams["encoder"][2]["session_relative_rad"], 0.)
        with self.assertRaises(Rejected):
            journal.union_observations()

    def test_duplicate_native_sample_time_rejects_without_deleting_event(self):
        rows = journal_rows()
        gyro = next(row for row in rows if row.get("kind") == "imu_raw")
        rows.append(dict(gyro))
        journal = self.load(rows)
        self.assertEqual(len(journal.streams["gyro"]), 4)
        with self.assertRaises(Rejected):
            journal.fitter_arrays()

    def test_raw_decoder_mismatch_is_retained_as_invalid(self):
        rows = journal_rows()
        feedback = next(row for row in rows if row.get("kind") == "yaw_feedback")
        feedback["speed_rpm"] = 1
        journal = self.load(rows, clock_provenance={"gyro": {
            "status": "BOUNDED_FOR_SESSION", "offset_bound_ns": 1000, "drift_bound_ppm": 10}})
        encoder = journal.streams["encoder"][0]
        self.assertFalse(encoder["validity"]["valid"])
        self.assertIn("raw_can_decoder_mismatch", encoder["validity"]["flags"])
        self.assertFalse(journal.streams["decoded_current"][0]["validity"]["valid"])
        self.assertEqual(journal.streams["gyro"][0]["timestamp"]["mapping_declaration"]["offset_bound_ns"], 1000)
        self.assertEqual(len(journal.streams["encoder"]), 4)
        with self.assertRaises(Rejected):
            journal.union_observations()

    def test_epoch_lift_verification_checks_each_record_and_keeps_physical_unknowns(self):
        # A host epoch beyond the SDK's 32-bit microsecond range, with a report
        # time straddling that range. The callback adds 5 ms and sub-us precision.
        sample_us = 3 * 2**32 - 2000
        raw = {"sh2_us": sample_us % 2**32, "sample_ns": sample_us * 1000,
               "rx_ns": (sample_us + 5000) * 1000 + 739}
        clock = imu_report_clock(raw)
        self.assertTrue(clock["epoch_lift_verified"])
        self.assertEqual(clock["epoch_lift_residual_ns"], 0)
        self.assertEqual(clock["mapping_status"], "HOST_POLL_REPORT_TIME_EPOCH_LIFT_VERIFIED")
        self.assertFalse(clock["independent_device_clock"])
        self.assertEqual(clock["physical_sample_timing"], "UNKNOWN")
        self.assertIsNone(clock["physical_offset_bound_ns"])
        self.assertIsNone(clock["physical_drift_bound_ppm"])
        self.assertIsNone(clock["sensor_filter_latency_bound_ns"])
        different = imu_report_clock({**raw, "sample_ns": raw["sample_ns"] + 1000})
        self.assertFalse(different["epoch_lift_verified"])
        self.assertEqual(different["epoch_lift_residual_ns"], 1000)
        self.assertFalse(imu_report_clock({"sample_ns": raw["sample_ns"]})["epoch_lift_verified"])


if __name__ == "__main__":
    unittest.main()
