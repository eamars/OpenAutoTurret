"""MENU > SETTINGS and the stats overlay (owner, 2026-10-03).

The overlay replaced both the DIAG drawer and the /dashboard page, after an audit of every field on
them; it is off by default, per browser, like a video player's "stats for nerds". The settings send
controld's set_speed with the exact value the row shows. Both are built by pure functions, so these
tests run them under node with no browser.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
import unittest

from ..hud import HUD_CSS, HUD_GEOMETRY_JS, HUD_HTML, HUD_JS

STATION = {
    "speed_patrol_wide_deg_s": 15.0, "speed_patrol_wide_default_deg_s": 15.0,
    "speed_patrol_detail_deg_s": 3.0, "speed_patrol_detail_default_deg_s": 3.0,
    "speed_track_deg_s": 12.0, "speed_track_default_deg_s": 20.0,
    "speed_roam_max_deg_s": 20.0, "speed_track_max_deg_s": 20.0,
}


@unittest.skipUnless(shutil.which("node"), "node not installed; the builders cannot be executed")
class Builders(unittest.TestCase):
    maxDiff = None

    def setUp(self) -> None:
        self._geo = tempfile.NamedTemporaryFile("w", suffix=".js", delete=False)
        self._geo.write(HUD_GEOMETRY_JS + "\nmodule.exports = { hudSpeedRows, hudStatsSections, hudStateLabel };\n")
        self._geo.close()

    def tearDown(self) -> None:
        os.unlink(self._geo.name)

    def _run(self, expression: str, t: dict):
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write("const T = require(%r);\nconst t = %s;\nconsole.log(JSON.stringify(%s));\n"
                     % (self._geo.name, json.dumps(t), expression))
            main = fh.name
        try:
            out = subprocess.run(["node", main], capture_output=True, text=True, check=True).stdout
        finally:
            os.unlink(main)
        return json.loads(out)

    def _stats(self, t: dict) -> dict:
        return {s["title"]: dict((k, v) for k, v in s["rows"]) for s in
                self._run("T.hudStatsSections(t)", t)}

    def test_each_arrow_is_the_exact_command_it_sends(self) -> None:
        rows = {r["key"]: r for r in self._run("T.hudSpeedRows(t)", STATION)}
        self.assertEqual(rows["patrol_wide"]["down"], "patrol_wide=14")
        self.assertEqual(rows["patrol_wide"]["up"], "patrol_wide=16")
        self.assertEqual(rows["patrol_detail"]["down"], "patrol_detail=2.5")
        self.assertIsNone(rows["patrol_wide"]["reset"], "at its configured value: nothing to reset")
        self.assertEqual(rows["track"]["reset"], "track=default")

    def test_the_arrows_stop_at_the_mode_maximum_and_the_floor(self) -> None:
        t = dict(STATION, speed_patrol_wide_deg_s=20.0, speed_patrol_detail_deg_s=0.5)
        rows = {r["key"]: r for r in self._run("T.hudSpeedRows(t)", t)}
        self.assertIsNone(rows["patrol_wide"]["up"], "no + past the roam maximum")
        self.assertIsNone(rows["patrol_detail"]["down"], "no - below controld's 0.5 deg/s floor")

    def test_unknown_speeds_offer_nothing(self) -> None:
        rows = self._run("T.hudSpeedRows(t)", {})
        self.assertTrue(all(r["value"] is None and r["up"] is None and r["down"] is None
                            and r["reset"] is None for r in rows))

    def test_shutdown_says_what_starts_it(self) -> None:
        # Owner ruling 2026-10-03: Homed or Shutdown, and a boot is Shutdown (controld's "idle").
        label = self._run("T.hudStateLabel(t)", {"supervisory": "idle", "mode": "MANUAL"})
        self.assertEqual(label["line1"], "SHUTDOWN")
        self.assertIn("HOME", label["line2"])

    def test_the_park_pose_names_its_next_steps(self) -> None:
        label = self._run("T.hudStateLabel(t)", {"supervisory": "parked", "mode": "MANUAL"})
        self.assertEqual(label["line1"], "PARKED")
        self.assertIn("AUTO", label["line2"])
        self.assertIn("MANUAL", label["line2"])

    def test_both_buses_and_bus_off_are_shown(self) -> None:
        stats = self._stats({"can_buses": [
            {"device": "can0", "up": True, "state": 0, "rx_frames": 10, "rx_error_frames": 0,
             "tx_frames": 9, "tx_failed": 0, "last_rx_age_ms": 1},
            {"device": "can1", "up": True, "state": 3, "rx_frames": 5, "rx_error_frames": 2,
             "tx_frames": 4, "tx_failed": 1, "last_rx_age_ms": 900}]})
        self.assertIn("ERROR-ACTIVE", stats["CAN"]["CAN0"])
        self.assertIn("BUS-OFF", stats["CAN"]["CAN1"])

    def test_no_can_link_is_said_as_absence_not_zeros(self) -> None:
        # A simulated backend publishes can_available=false; a table of zeros reads as a healthy bus.
        self.assertEqual(self._stats({"can_available": False})["CAN"], {"CAN": "no CAN link reported"})

    def test_motor_temperatures_show_against_the_trip(self) -> None:
        axes = self._stats({"temp_yaw_c": 41.4, "temp_pitch_c": 37.0, "motor_overtemp_c": 75})["AXES"]
        self.assertEqual(axes["MOTOR TEMP YAW / PITCH"], "41°C / 37°C  (trip 75°C)")
        raw = self._stats({"temp_yaw_c": None, "temp_raw_yaw": 30, "temp_pitch_c": 28.6})["AXES"]
        self.assertEqual(raw["MOTOR TEMP YAW / PITCH"], "≈30°C (raw) / 29°C",
                         "the GM6020's byte is shown as raw, not as a calibrated temperature")
        unknown = self._stats({"temp_yaw_c": None})["AXES"]["MOTOR TEMP YAW / PITCH"]
        self.assertTrue(unknown.startswith("-- / --"), "no reading is not a cold motor")

    def test_events_are_listed_newest_first_with_their_age(self) -> None:
        stats = self._stats({"ts_ns": 10_000_000_000, "events": [
            {"t_ns": 4_000_000_000, "event": "MODE_CHANGED", "detail": "AUTO_ROAM"},
            {"t_ns": 9_000_000_000, "event": "TARGET_LOST", "detail": ""}]})
        self.assertEqual(list(stats["EVENTS"].items())[0], ("-1 S", "TARGET_LOST"))
        self.assertIn(("-6 S", "MODE_CHANGED  AUTO_ROAM"), stats["EVENTS"].items())


class PageWiring(unittest.TestCase):

    def test_the_overlay_is_off_until_the_settings_turn_it_on(self) -> None:
        self.assertIn('<div id="stats" hidden', HUD_HTML)
        self.assertIn('hudPref("ota.hud.stats", false)', HUD_JS)
        self.assertIn("STATS FOR NERDS", HUD_JS)

    def test_the_health_chips_live_in_the_folded_status_bar(self) -> None:
        strip = HUD_HTML[HUD_HTML.index('<div id="strip"'):]
        strip = strip[:strip.index("</div>")]
        self.assertIn('class="folded"', strip)
        self.assertIn('id="health"', strip, "the chips moved from the top right into the bottom bar")
        self.assertEqual(HUD_HTML.count('id="health"'), 1)
        # Folded hides only healthy chips: an alert is never folded away.
        self.assertIn("#strip.folded #health .chip.ok { display: none; }", HUD_CSS)

    def test_a_hailo_adapter_by_any_name_is_healthy(self) -> None:
        # The station's adapter is "hailo_pose"; testing for exactly "hailo" kept the NN chip amber.
        self.assertIn("/^hailo/.test(", HUD_JS)
        self.assertNotIn('.toLowerCase() === "hailo"', HUD_JS)

    def test_an_unhealthy_imu_keeps_its_words(self) -> None:
        # "NO SAMPLES" was cut to "NO"; only the healthy FRESH state is shortened to one word.
        self.assertIn('st === "ok" ? lbl.split(" ")[0] : lbl', HUD_JS)

    def test_park_is_not_a_place_to_be_stuck(self) -> None:
        # Owner, 2026-10-03: from the park pose, Auto and Manual are one press each.
        self.assertIn('(t.phase === "hold" || t.phase === "parked")', HUD_JS)
        self.assertIn('if (lastTelemetry && lastTelemetry.phase === "parked") sendCommand("set_mode", "MANUAL");',
                      HUD_JS)

    def test_settings_rows_send_set_speed(self) -> None:
        self.assertIn('data-cmd="set_speed"', HUD_JS)
        self.assertIn("Speeds hold until the station restarts.", HUD_JS)


if __name__ == "__main__":
    unittest.main()
