#include "web/command_validation.hpp"

#include <gtest/gtest.h>

using namespace ota;
using namespace ota::web;

namespace {

SystemCommandState homed_state() {
  SystemCommandState s;
  s.homed = true;
  s.at_ready = true;   // homed AND holding the ready pose (§38.1)
  s.limits_valid = true;
  s.q_min_rad[kPitchIx] = -1.5;
  s.q_max_rad[kPitchIx] = 1.5;
  s.q_min_rad[kYawIx] = -2.9;
  s.q_max_rad[kYawIx] = 2.9;
  return s;
}

}  // namespace

TEST(CommandValidation, HoldAlwaysAllowed) {
  SystemCommandState s;  // not homed, faulted, anything.
  s.fault = true;
  EXPECT_TRUE(validate_command(s, "hold").ok);
  EXPECT_TRUE(validate_command(s, "request_park").ok);
  EXPECT_TRUE(validate_command(s, "request_shutdown").ok);
}

TEST(CommandValidation, FaultLocksMotion) {
  auto s = homed_state();
  s.fault = true;
  EXPECT_FALSE(validate_command(s, "start_tracking").ok);
  EXPECT_FALSE(validate_command(s, "start_homing").ok);
  EXPECT_FALSE(validate_command(s, "enable_search").ok);
  EXPECT_FALSE(validate_command(s, "run_test_motion", "1.0").ok);
}

TEST(CommandValidation, NotHomedLocksTracking) {
  SystemCommandState s;  // homed=false
  EXPECT_FALSE(validate_command(s, "start_tracking").ok);
  EXPECT_FALSE(validate_command(s, "enable_search").ok);
  EXPECT_FALSE(validate_command(s, "select_target", "1").ok);
  EXPECT_EQ(validate_command(s, "start_tracking").error,
            "not homed (position validity unknown)");
}

TEST(CommandValidation, TrackingLifecycle) {
  auto s = homed_state();
  EXPECT_TRUE(validate_command(s, "start_tracking").ok);
  s.tracking_active = true;
  s.tracking_enabled = true;
  EXPECT_FALSE(validate_command(s, "start_tracking").ok);  // already active
  EXPECT_TRUE(validate_command(s, "stop_tracking").ok);
  s.tracking_enabled = false;
  EXPECT_FALSE(validate_command(s, "stop_tracking").ok);   // not enabled
}

TEST(CommandValidation, SearchLifecycle) {
  auto s = homed_state();
  EXPECT_TRUE(validate_command(s, "enable_search").ok);
  s.search_enabled = true;
  EXPECT_FALSE(validate_command(s, "enable_search").ok);
  EXPECT_TRUE(validate_command(s, "disable_search").ok);
  s.search_enabled = false;
  EXPECT_FALSE(validate_command(s, "disable_search").ok);
}

TEST(CommandValidation, SelectTargetRequiresTrackingAndValidId) {
  auto s = homed_state();
  EXPECT_FALSE(validate_command(s, "select_target", "1").ok);  // tracking off
  s.tracking_enabled = true;
  EXPECT_TRUE(validate_command(s, "select_target", "1").ok);
  EXPECT_FALSE(validate_command(s, "select_target", "99").ok);  // out of range
  EXPECT_FALSE(validate_command(s, "select_target", "abc").ok);  // not a number
  EXPECT_FALSE(validate_command(s, "select_target", "").ok);
}

TEST(CommandValidation, HomingOnlyWhenNotHomed) {
  auto s = homed_state();  // already homed
  EXPECT_FALSE(validate_command(s, "start_homing").ok);
  s.homed = false;
  EXPECT_TRUE(validate_command(s, "start_homing").ok);  // can start when not homed
}

TEST(CommandValidation, CalibrationBlockedWhileMoving) {
  auto s = homed_state();
  EXPECT_TRUE(validate_command(s, "start_installation_calibration").ok);
  s.tracking_active = true;
  EXPECT_FALSE(validate_command(s, "start_installation_calibration").ok);
}

TEST(CommandValidation, RestrictedTestMotion) {
  auto s = homed_state();
  EXPECT_TRUE(validate_command(s, "run_test_motion", "1.0").ok);  // in envelope
  EXPECT_FALSE(validate_command(s, "run_test_motion", "5.0").ok);  // beyond yaw max
  EXPECT_FALSE(validate_command(s, "run_test_motion", "-5.0").ok); // beyond yaw min
  EXPECT_FALSE(validate_command(s, "run_test_motion", "abc").ok);  // not a number
  s.limits_valid = false;
  EXPECT_FALSE(validate_command(s, "run_test_motion", "1.0").ok);  // no limits
}

TEST(CommandValidation, UnknownCommandRejected) {
  auto s = homed_state();
  auto r = validate_command(s, "launch_missiles");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("unknown command"), std::string::npos);
}

TEST(CommandValidation, PayloadVerificationRequiresReadyPose) {
  // Homing passes through Hold between stages, so "not moving" is not enough:
  // a check requested while the station is still travelling to the ready pose
  // would step from a travel stop, where the safe central region is empty.
  // Rejecting synchronously gives the operator a reason instead of an async
  // abort buried in the log (§42.2).
  SystemCommandState s = homed_state();
  s.at_ready = false;
  const auto r = validate_command(s, "start_payload_verification");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("ready pose"), std::string::npos) << r.error;

  // select_payload_profile only changes caps (no motion), so it stays allowed:
  // the operator can prepare a profile during the ready move.
  const auto sel = validate_command(s, "select_payload_profile", "conservative");
  EXPECT_TRUE(sel.ok) << sel.error;
}

// --- v3 §51/§52: the mode commands ---------------------------------------

TEST(CommandValidation, StopMotionIsAcceptedInEveryState) {
  // The state below says: not homed, faulted, no valid limits. Everything the
  // other gates refuse, this one must still accept. §27 is not a request to move,
  // it is a request to stop, and a stop that is conditional on the machine being
  // healthy is not a stop.
  SystemCommandState s;
  s.fault = true;
  s.homed = false;
  s.at_ready = false;
  s.limits_valid = false;
  EXPECT_TRUE(validate_command(s, "stop_motion").ok);
}

TEST(CommandValidation, SetModeChecksShapeNotState) {
  auto s = homed_state();
  EXPECT_TRUE(validate_command(s, "set_mode", "MANUAL").ok);
  EXPECT_TRUE(validate_command(s, "set_mode", "AUTO_TRACK").ok);
  EXPECT_TRUE(validate_command(s, "set_mode", "auto_roam").ok);
  // The interesting half: the gate deliberately does NOT re-implement controld's
  // state checks (homing, feedback freshness, safety ladder, roam envelope).
  // Duplicating them here is what produced v1's enable_search/disable_search
  // contract mismatch — a UI that refused a request the loop would have honoured,
  // and vice versa. controld owns that answer and gives a reason (§52).
  SystemCommandState bad;  // nothing homed, nothing valid
  EXPECT_TRUE(validate_command(bad, "set_mode", "AUTO_TRACK").ok);
}

TEST(CommandValidation, SetModeRejectsNamesThatDoNotExist) {
  auto s = homed_state();
  EXPECT_FALSE(validate_command(s, "set_mode", "").ok);
  EXPECT_FALSE(validate_command(s, "set_mode", "SEARCH").ok)
      << "v1's SEARCH is an AUTO_ROAM phase now; accepting the old spelling would "
         "leave a stale UI half-working instead of failing it with a reason";
  EXPECT_FALSE(validate_command(s, "set_mode", "HOLD").ok)
      << "HOLD is a phase of MANUAL, not a mode (§2)";
  auto r = validate_command(s, "set_mode", "AUTOPILOT");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("MANUAL"), std::string::npos)
      << "the rejection must name the accepted spellings, not just say no";
}
