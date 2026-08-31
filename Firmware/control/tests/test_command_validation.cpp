#include "web/command_validation.hpp"

#include <gtest/gtest.h>

using namespace ota;
using namespace ota::web;

namespace {

SystemCommandState homed_state() {
  SystemCommandState s;
  s.homed = true;
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
