"""§21 state wording and §22 safety presentation.

The page used to echo the daemon's phase string at the operator and put every safety state in one chip
that changed colour. §21 asks for named state wording per mode/phase, and §22 asks for a ladder - green
and compact, amber naming the limit, amber more prominent, red with a reason prominent enough to
interrupt - which one chip cannot express.

Both builders are pure, so the wording and the ladder are checked under node. What is NOT checked here is
real-browser paint: no assertion in this file says what the station looks like, only what the code that
draws it decides. Visual fidelity is §24 and is signed by a person, not by a test.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
import unittest

from ..hud import HUD_CSS, HUD_GEOMETRY_JS, HUD_HTML, HUD_JS


class _NodeBuilders(unittest.TestCase):
    """Loads the HUD's pure builders into node so their decisions can be executed, not read."""

    maxDiff = None

    def setUp(self) -> None:
        self._mod = tempfile.NamedTemporaryFile("w", suffix=".js", delete=False)
        self._mod.write(HUD_GEOMETRY_JS +
                        "\nmodule.exports = { hudStateLabel, hudSafetyPresentation, hudSafetyEdge };\n")
        self._mod.close()

    def tearDown(self) -> None:
        os.unlink(self._mod.name)

    def _node(self, script: str):
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write("const T = require(%r);\n%s" % (self._mod.name, script))
            main = fh.name
        try:
            r = subprocess.run(["node", main], capture_output=True, text=True, timeout=30)
        finally:
            os.unlink(main)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout.strip()
        try:
            return json.loads(out)
        except json.JSONDecodeError:
            return out

    def _label(self, **kw):
        return self._node("console.log(JSON.stringify(T.hudStateLabel(%s)));" % json.dumps(kw))

    def test_tracking_is_worded_as_tracking_not_as_the_internal_phase(self) -> None:
        # §21.1. The daemon publishes mode_phase "TRACK"; the HUD's word for the operator is TRACKING.
        got = self._label(mode="AUTO_TRACK", phase="TRACK")
        self.assertEqual((got["line1"], got["line2"]), ("AUTO TRACK", "TRACKING"))
        self.assertTrue(got["named"])

    def test_coasting_and_lost_hold_read_the_way_the_revision_says(self) -> None:
        coast = self._label(mode="AUTO_TRACK", phase="COAST")
        self.assertEqual((coast["line1"], coast["line2"]), ("AUTO TRACK", "COASTING"))
        lost = self._label(mode="AUTO_TRACK", phase="LOST_HOLD")
        # §21.3: "TARGET LOST / HOLDING". The loss goes on the strong line: it is the fact the operator
        # has to act on, and burying it under a mode name is how an HUD hides the interesting part.
        self.assertEqual((lost["line1"], lost["line2"]), ("TARGET LOST", "HOLDING"))

    def test_roam_and_manual_states(self) -> None:
        self.assertEqual(self._label(mode="AUTO_ROAM", phase="SWEEP")["line2"], "SWEEP")
        self.assertEqual(self._label(mode="MANUAL", phase="HOLD")["line2"], "HOLD")
        jog = self._label(mode="MANUAL", phase="HOLD", jogging=True)
        self.assertEqual(jog["line2"], "JOG")

    def test_jog_comes_from_the_lease_not_from_motion_being_nonzero(self) -> None:
        # manual_lease_active is published; inferring JOG from a non-zero rate would also light up during
        # homing, a roam, or the settling after a hold.
        self.assertIn("manual_lease_active", HUD_JS, "the page must read the published lease")
        self.assertIn("o.jogging ?", HUD_GEOMETRY_JS,
                      "the builder keys JOG on the lease argument, not on motion being non-zero")

    def test_a_state_the_revision_does_not_name_keeps_the_daemons_own_word(self) -> None:
        # WAIT_TARGET is published but §21 does not enumerate it. Inventing friendly wording for an
        # unspecified state would read as if the HUD knew it. The raw case is marked so it can be styled
        # as the daemon's word rather than HUD wording.
        got = self._label(mode="AUTO_TRACK", phase="SOMETHING_NEW")
        self.assertEqual(got["line2"], "SOMETHING_NEW")
        self.assertFalse(got["named"])
        self.assertIn('class="m2\' + (st.named ? "" : " raw")', HUD_JS)

    def test_no_sweep_direction_is_claimed(self) -> None:
        # §21.4 mentions SWEEP LEFT|RIGHT. controld publishes no direction, so the HUD says SWEEP. An
        # arrow inferred from the sign of a rate would be a claim about the station it cannot support.
        got = self._label(mode="AUTO_ROAM", phase="SWEEP")
        self.assertNotIn("LEFT", got["line1"] + got["line2"])
        self.assertNotIn("RIGHT", got["line1"] + got["line2"])


@unittest.skipUnless(shutil.which("node"), "node not installed; the builders cannot be executed")
class StateWording(_NodeBuilders):
    """§21: the top-left block says the thing the revision names for this state."""

    def _label(self, **kw):
        return self._node("console.log(JSON.stringify(T.hudStateLabel(%s)));" % json.dumps(kw))


@unittest.skipUnless(shutil.which("node"), "node not installed; the builders cannot be executed")
class SafetyLadder(_NodeBuilders):
    """§22: four presentations, four different amounts of visual weight."""

    def _pres(self, t):
        return self._node("console.log(JSON.stringify(T.hudSafetyPresentation(%s)));" % json.dumps(t))

    def test_normal_is_green_compact_and_says_allow(self) -> None:
        got = self._pres({"safety_action": "ALLOW"})
        self.assertEqual(got["label"], "SAFETY ALLOW")
        self.assertEqual((got["tone"], got["tier"]), ("green", "normal"))

    def test_derate_is_amber_and_names_the_relevant_edge(self) -> None:
        # §22 asks for the travel-tape edge or FOR boundary. The reference is close to pitch MAX here.
        import math
        t = {"safety_action": "DERATE",
             "q_ref_yaw_rad": 0.0, "q_soft_min_yaw_rad": math.radians(-100),
             "q_soft_max_yaw_rad": math.radians(100),
             "q_ref_pitch_rad": math.radians(28), "q_soft_min_pitch_rad": math.radians(-12),
             "q_soft_max_pitch_rad": math.radians(30)}
        got = self._pres(t)
        self.assertEqual(got["label"], "DERATE")
        self.assertEqual((got["tone"], got["tier"]), ("amber", "caution"))
        self.assertEqual(got["reason"], "PITCH MAX",
                         "the operator needs to know which edge, not merely that something is derated")

    def test_a_breached_limit_is_named_ahead_of_one_still_ahead(self) -> None:
        import math
        t = {"safety_action": "DERATE",
             "q_ref_yaw_rad": math.radians(101), "q_soft_min_yaw_rad": math.radians(-100),
             "q_soft_max_yaw_rad": math.radians(100),
             "q_ref_pitch_rad": math.radians(29), "q_soft_min_pitch_rad": math.radians(-12),
             "q_soft_max_pitch_rad": math.radians(30)}
        self.assertEqual(self._pres(t)["reason"], "YAW MAX")

    def test_derate_admits_when_the_limits_are_not_published(self) -> None:
        got = self._pres({"safety_action": "DERATE"})
        self.assertIsNotNone(got["reason"])
        self.assertIn("not published", got["reason"],
                      "naming no edge must not be silently shown as no edge")

    def test_brake_is_amber_and_more_prominent_than_derate(self) -> None:
        self.assertEqual(self._pres({"safety_action": "BRAKE"})["label"], "BRAKING")
        self.assertEqual(self._pres({"safety_action": "BRAKE"})["tier"], "prominent")

    def test_fault_is_red_carries_a_reason_and_interrupts(self) -> None:
        got = self._pres({"safety_action": "FAULT_STOP", "fault": "motor amplifier over-temperature"})
        self.assertEqual(got["label"], "FAULT")
        self.assertEqual((got["tone"], got["tier"]), ("red", "interrupt"))
        self.assertIn("over-temperature", got["reason"],
                      "a red box that says only FAULT sends the operator to a log at the worst moment")

    def test_a_fault_string_alone_raises_the_banner(self) -> None:
        # controld can carry a fault reason without the safety action being FAULT_STOP. Either is enough.
        got = self._pres({"safety_action": "ALLOW", "fault": "encoder CRC storm on yaw"})
        self.assertEqual((got["tone"], got["tier"]), ("red", "interrupt"))

    def test_states_section_22_does_not_name_keep_the_daemons_word(self) -> None:
        # §22 words four presentations; the enum has six (Allow, Derate, Brake, Hold, FaultStop,
        # Disable). Folding HOLD into ALLOW, or silence, would understate what the station said.
        hold = self._pres({"safety_action": "HOLD"})
        self.assertEqual(hold["label"], "SAFETY HOLD")
        self.assertNotEqual(hold["tone"], "green")
        dis = self._pres({"safety_action": "DISABLE"})
        self.assertEqual(dis["label"], "DISABLED")

    def test_an_unseen_state_is_never_green(self) -> None:
        got = self._pres({"safety_action": "SOMETHING_NEW"})
        self.assertNotEqual(got["tone"], "green", "defaulting an unknown safety state to green would be "
                                                 "the one default that can kill")
        self.assertEqual(got["tier"], "caution")


class SafetyPresentationIsOnThePage(unittest.TestCase):
    maxDiff = None

    def test_the_banner_element_exists_and_is_not_a_chip(self) -> None:
        self.assertIn('id="safety"', HUD_HTML)
        self.assertIn('#safety {', HUD_CSS)
        self.assertIn('aria-live="assertive"', HUD_HTML,
                      "§22 asks a fault to interrupt; a screen reader should be told too")

    def test_only_the_fault_tier_is_red(self) -> None:
        # §14 reserves red for stop and fault. The CSS must not hand it to the amber tiers.
        block = HUD_CSS[HUD_CSS.index("/* --- §22"):]
        self.assertIn("#safety.red .s1 { color:#ff5d5d;", block)
        self.assertIn("#safety.amber .s1 { color:#f2b329;", block)
        amber_lines = [ln for ln in block.splitlines() if ".amber" in ln and "#ff5d5d" in ln]
        self.assertEqual(amber_lines, [])

    def test_the_ladder_is_actually_applied_in_the_render_path(self) -> None:
        at = HUD_JS.index("const sf = hudSafetyPresentation(t);")
        body = HUD_JS[at:at + 900]
        self.assertIn('chip("SAFETY", "ok", "ALLOW")', body, "normal stays green and compact")
        self.assertIn("banner.hidden = false", body)
        self.assertIn('banner.className = sf.tone + " " + sf.tier', body,
                      "tier is what changes the weight; a fixed size would flatten §22's ladder")

    def test_the_mode_block_uses_the_state_builder(self) -> None:
        at = HUD_JS.index("hudStateLabel({")
        self.assertIn("t.mode_phase", HUD_JS[at:at + 200])
        self.assertIn("manual_lease_active", HUD_JS[at:at + 200])
        self.assertIn('<div class="m1">\' + st.line1', HUD_JS)


if __name__ == "__main__":
    unittest.main()
