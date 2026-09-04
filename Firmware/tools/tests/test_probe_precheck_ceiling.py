"""Round 62: the pre-flight "achievable" verdict is computed against a ceiling that is not in force.

Round 49 moved the C6 *analysis* onto controld's published `effective_speed_ceiling_deg_s`. The pre-check was not
moved: line 89 calls `envelope_min_time_s(deg)` and lets it default to `TRACK_V_MAX` (30 deg/s), which is why the
tool printed `25.0 deg needs >= 1.43 s at 30 deg/s ... -> achievable` for a dart the station's 10 deg/s ceiling
forbids. These tests pin the arithmetic - not the wiring, which needs a run - so the defect cannot be argued away
later: the same dart that is legal at 30 is not legal at the ceiling in force.
"""
import unittest

from tools.probe_track_loop import envelope_min_time_s


class PrecheckCeiling(unittest.TestCase):
    DART_DEG = 25.0
    RAMP_S = 1.6                      # the acceptance dart: 25 deg in 1.60 s

    def test_the_acceptance_dart_is_legal_only_at_the_configured_speed(self) -> None:
        # What the pre-check actually uses today (30 deg/s): passes, and says "achievable".
        self.assertLessEqual(envelope_min_time_s(self.DART_DEG), self.RAMP_S)

    def test_the_same_dart_is_not_legal_at_the_ceiling_in_force(self) -> None:
        # controld publishes 10 deg/s on this station (hold_speed_effective, round 40; on the panel since 41).
        self.assertGreater(envelope_min_time_s(self.DART_DEG, v_max=10.0), self.RAMP_S)

    def test_the_pre_check_ignores_the_published_ceiling_today(self) -> None:
        import pathlib
        src = pathlib.Path("tools/probe_track_loop.py").read_text()
        self.assertIn("t_min = envelope_min_time_s(deg)", src,
                      "still defaulting to the configured speed rather than the ceiling in force")

    def test_a_dart_sized_to_the_binding_ceiling_is_legal_at_both(self) -> None:
        # Gives the operator the arithmetic for the other option: size the dart to 10 deg/s.
        sized = 12.0                  # ~7.5 deg/s average over 1.6 s, comfortably inside 10
        self.assertLessEqual(envelope_min_time_s(sized, v_max=10.0), self.RAMP_S)


if __name__ == "__main__":
    unittest.main()
