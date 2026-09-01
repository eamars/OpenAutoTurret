"""Round-trip tests for the webd <-> controld JSON wire protocol."""
from __future__ import annotations

import unittest

from ..protocol import (
    CommandMessage,
    ResponseMessage,
    Telemetry,
    command_to_json,
    parse_message,
    telemetry_from_json,
    telemetry_to_json,
)


class ProtocolTest(unittest.TestCase):
    def test_command_round_trip(self) -> None:
        raw = command_to_json("start_tracking")
        mtype, msg = parse_message(raw)
        self.assertEqual(mtype, "command")
        self.assertIsInstance(msg, CommandMessage)
        self.assertEqual(msg.command, "start_tracking")
        self.assertEqual(msg.arg, "")

    def test_command_with_arg(self) -> None:
        raw = command_to_json("select_target", "7")
        mtype, msg = parse_message(raw)
        self.assertEqual(mtype, "command")
        self.assertEqual(msg.command, "select_target")
        self.assertEqual(msg.arg, "7")

    def test_command_dataclass_to_json(self) -> None:
        raw = CommandMessage("run_test_motion", "0.5").to_json()
        mtype, msg = parse_message(raw)
        self.assertEqual(mtype, "command")
        self.assertEqual(msg.arg, "0.5")

    def test_response_ok(self) -> None:
        raw = '{"type":"response","command":"hold","ok":true}'
        mtype, msg = parse_message(raw)
        self.assertEqual(mtype, "response")
        self.assertIsInstance(msg, ResponseMessage)
        self.assertTrue(msg.ok)
        self.assertEqual(msg.command, "hold")
        self.assertEqual(msg.error, "")

    def test_response_reject(self) -> None:
        raw = (
            '{"type":"response","command":"start_tracking",'
            '"ok":false,"error":"not homed"}'
        )
        mtype, msg = parse_message(raw)
        self.assertFalse(msg.ok)
        self.assertEqual(msg.error, "not homed")

    def test_telemetry_round_trip(self) -> None:
        t = Telemetry(
            ts_ns=123,
            track_state="tracking",
            tracking_active=True,
            target_confidence=0.87,
            q_yaw_rad=0.1,
            v_yaw_rad_s=0.05,
            q_ref_yaw_rad=0.12,
            q_pitch_rad=-0.2,
            v_pitch_rad_s=-0.01,
            q_ref_pitch_rad=-0.2,
            effort_yaw=1.5,
            effort_pitch=-0.7,
            target_az_world_rad=0.3,
            target_el_world_rad=0.1,
            base_roll_rad=0.0,
            base_pitch_rad=0.1,
            base_yaw_rad=0.0,
            installation_calibrated=True,
            installation_source="visual_calibration",
            safety_action="ALLOW",
            feedback_age_ms=2,
            control_cycle_us=2500,
        )
        mtype, obj = parse_message(telemetry_to_json(t))
        self.assertEqual(mtype, "telemetry")
        self.assertIsInstance(obj, dict)
        self.assertEqual(obj["track_state"], "tracking")
        self.assertEqual(obj["installation_source"], "visual_calibration")
        self.assertEqual(obj["safety_action"], "ALLOW")
        back = telemetry_from_json(obj)
        self.assertEqual(back.track_state, "tracking")
        self.assertEqual(back.installation_source, "visual_calibration")
        self.assertEqual(back.safety_action, "ALLOW")
        self.assertAlmostEqual(back.q_yaw_rad, 0.1)
        self.assertAlmostEqual(back.effort_pitch, -0.7)
        self.assertTrue(back.tracking_active)
        self.assertEqual(back.ts_ns, 123)

    def test_telemetry_missing_fields_default(self) -> None:
        # A minimal telemetry message (older controld) should not break.
        obj = telemetry_from_json({"type": "telemetry", "track_state": "search"})
        self.assertEqual(obj.track_state, "search")
        self.assertEqual(obj.q_yaw_rad, 0.0)
        self.assertEqual(obj.safety_action, "ALLOW")

    def test_unknown_type_raises(self) -> None:
        with self.assertRaises(ValueError):
            parse_message('{"type":"bogus"}')

    def test_telemetry_payload_fields_parse(self) -> None:
        # Phase 9 payload fields (§28.5, §31.3, §42.1) come over the wire.
        obj = telemetry_from_json(
            {
                "type": "telemetry",
                "track_state": "ready_hold",
                "payload_profile_name": "sniper_rifle",
                "payload_profile_status": "mismatch",
                "payload_derated": True,
                "payload_check_active": False,
            }
        )
        self.assertEqual(obj.payload_profile_name, "sniper_rifle")
        self.assertEqual(obj.payload_profile_status, "mismatch")
        self.assertTrue(obj.payload_derated)
        self.assertFalse(obj.payload_check_active)

    def test_telemetry_payload_fields_default(self) -> None:
        # An older controld without the payload fields still parses.
        obj = telemetry_from_json({"type": "telemetry", "track_state": "search"})
        self.assertEqual(obj.payload_profile_name, "")
        self.assertEqual(obj.payload_profile_status, "no_profile")
        self.assertFalse(obj.payload_derated)
        self.assertFalse(obj.payload_check_active)

    def test_telemetry_payload_round_trip(self) -> None:
        t = Telemetry(
            ts_ns=7,
            track_state="ready_hold",
            payload_profile_name="smg",
            payload_profile_status="ok",
            payload_derated=False,
            payload_check_active=True,
        )
        mtype, obj = parse_message(telemetry_to_json(t))
        self.assertEqual(mtype, "telemetry")
        back = telemetry_from_json(obj)
        self.assertEqual(back.payload_profile_name, "smg")
        self.assertEqual(back.payload_profile_status, "ok")
        self.assertFalse(back.payload_derated)
        self.assertTrue(back.payload_check_active)


if __name__ == "__main__":
    unittest.main()
