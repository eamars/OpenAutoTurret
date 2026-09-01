// End-to-end tests of the daemon-side payload verification (§27, §31.3,
// §31.4, §42.1): baseline comparison on the SimMotorBackend, automatic
// derating on mismatch, telemetry publication, and manual re-verification.
// No CAN, no motor, no camera — the sim plant is the only axis hardware.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "calibration/homing_plan.hpp"
#include "control/control_loop.hpp"
#include "payload/payload_profile.hpp"
#include "payload/payload_profiler.hpp"
#include "sim/sim_motor_backend.hpp"

namespace {

using ota::AxisId;
using ota::AxisLimits;
using ota::ControlLoop;
using ota::TimeNs;
using ota::HomingAction;
using ota::HomingActionType;
using ota::HomingParams;
using ota::HomingPlan;
using ota::HomingPlanConfig;
using ota::TravelBand;
using ota::payload::AxisPayloadProfile;
using ota::payload::AxisProfileMetrics;
using ota::payload::PayloadProfile;
using ota::payload::PayloadProfileStore;
using ota::payload::PayloadProfiler;
using ota::payload::ProfilerConfig;
using ota::payload::PayloadStatus;
using ota::sim::SimMotorBackend;

constexpr double kDeg = M_PI / 180.0;
constexpr TimeNs kDtNs = 5'000'000;  // 200 Hz

ControlLoop::Config make_cfg(bool auto_verify) {
  ControlLoop::Config c;
  c.control_hz = 200;
  c.hold_speed_rad_s = 30.0 * kDeg;
  c.emergency_speed_rad_s = 10.0 * kDeg;
  c.soft_margin_rad = 2.0 * kDeg;
  c.payload_auto_verify = auto_verify;
  return c;
}

HomingPlan make_plan() {
  HomingPlanConfig hcfg;
  HomingParams hp;
  hp.coarse_speed_rad_s = 20.0 * kDeg;
  hp.fine_speed_rad_s = 2.0 * kDeg;
  hp.settle_time_s = 0.3;
  hcfg.homing = hp;
  hcfg.travel_bands[0] = TravelBand{0.0, 120.0};
  hcfg.travel_bands[1] = TravelBand{0.0, 120.0};
  std::vector<HomingAction> actions;
  actions.push_back(HomingAction{.type = HomingActionType::HomeFullRange,
                                 .axis = AxisId::Pitch});
  actions.push_back(HomingAction{.type = HomingActionType::HomeFullRange,
                                 .axis = AxisId::Yaw});
  return HomingPlan(std::move(actions), hcfg);
}

// Commissioning path: profile a twin sim of the same plant and store the
// baseline (what `turret-payload profile --sim` does offline, §44).
PayloadProfile commission_baseline() {
  SimMotorBackend sim(0.005);
  std::string err;
  sim.enter_position_mode(AxisId::Pitch, 0.5, err);
  sim.enter_position_mode(AxisId::Yaw, 0.5, err);
  TimeNs t = 0;
  const ProfilerConfig pcfg;
  PayloadProfiler prof(sim, pcfg, [&t]() { return t += 5'000'000; },
                       [](void) {});
  PayloadProfile p;
  p.name = "test_payload";
  p.created_ns = 1;
  for (int i = 0; i < ota::kAxisCount; ++i) {
    const AxisId a = static_cast<AxisId>(i);
    AxisProfileMetrics m =
        prof.profile_axis(a, 0.0, AxisLimits{}, err);
    if (!m.valid) throw std::runtime_error(err);
    AxisPayloadProfile ax;
    PayloadProfiler::derive_limits(m, pcfg, ax);
    p.axis(a) = ax;
  }
  return p;
}

// Minimal rig: sim plant + control loop (no camera).
class PayloadRig {
 public:
  explicit PayloadRig(bool auto_verify)
      : backend_(std::make_unique<SimMotorBackend>(0.005)),
        loop_(make_cfg(auto_verify), std::move(backend_)) {
    sim_.set_stops(AxisId::Pitch, -1.0, 1.0);
    sim_.set_stops(AxisId::Yaw, -1.0, 1.0);
    sim_.set_position(AxisId::Pitch, 10.0 * kDeg);
    sim_.set_position(AxisId::Yaw, 0.0);
  }
  SimMotorBackend& sim() { return sim_; }  // sim_ stays valid; backend_ moved into loop_
  ControlLoop& loop() { return loop_; }
  void step(TimeNs& t) {
    t += kDtNs;
    loop_.step(t, kDtNs);
  }
  bool run_to_ready(TimeNs& t, int max_steps = 20000) {
    std::string err;
    if (!loop_.start_homing(make_plan(), err)) return false;
    for (int i = 0; i < max_steps && !(loop_.homed() && loop_.at_ready());
         ++i)
      step(t);
    return loop_.homed() && loop_.at_ready();
  }
  // Step until an in-progress payload check completes (phase returns to
  // Hold). Returns false if the cap is hit while the check is still active.
  bool wait_check_done(TimeNs& t, int max_steps = 8000) {
    bool active = false;
    for (int i = 0; i < max_steps; ++i) {
      step(t);
      if (loop_.payload_check_active()) active = true;
      else if (active) return true;
    }
    return active && !loop_.payload_check_active();
  }

 private:
  std::unique_ptr<SimMotorBackend> backend_;
  SimMotorBackend& sim_ = *backend_;
  ControlLoop loop_;
};

}  // namespace

TEST(PayloadDaemon, AutoVerifyMatchesBaselineAndStaysFullSpeed) {
  PayloadRig rig(/*auto_verify=*/true);
  TimeNs t = 0;
  rig.run_to_ready(t);
  ASSERT_TRUE(rig.loop().homed() && rig.loop().at_ready());

  const auto prof = commission_baseline();
  rig.loop().set_payload_profile(prof);
  EXPECT_EQ(rig.loop().payload_status(), PayloadStatus::Ok);
  EXPECT_FALSE(rig.loop().payload_derated());
  // The profiled v_max (20 deg/s) already caps the 30 deg/s hold speed.
  EXPECT_NEAR(rig.loop().hold_speed_effective(), 20.0 * kDeg, 1e-9);

  // §27: on the first post-homing hold at rest, the check runs itself.
  EXPECT_TRUE(rig.wait_check_done(t)) << rig.loop().payload_detail();
  EXPECT_EQ(rig.loop().payload_status(), PayloadStatus::Ok)
      << rig.loop().payload_detail();
  EXPECT_FALSE(rig.loop().payload_derated());
  EXPECT_NEAR(rig.loop().hold_speed_effective(), 20.0 * kDeg, 1e-9);
  EXPECT_TRUE(rig.loop().homed());
  EXPECT_EQ(rig.loop().phase(), ota::Phase::Hold);
}

TEST(PayloadDaemon, MismatchDeratesAndReverifyClearsIt) {
  PayloadRig rig(/*auto_verify=*/false);
  TimeNs t = 0;
  rig.run_to_ready(t);
  ASSERT_TRUE(rig.loop().homed() && rig.loop().at_ready());

  const auto prof = commission_baseline();
  rig.loop().set_payload_profile(prof);

  // Emulate a heavier payload on yaw: 3x the position time constant.
  rig.sim().set_response_tau(AxisId::Yaw, 0.15);
  const auto res = rig.loop().submit_command("start_payload_verification", "");
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_TRUE(rig.wait_check_done(t)) << rig.loop().payload_detail();

  EXPECT_EQ(rig.loop().payload_status(), PayloadStatus::Mismatch)
      << rig.loop().payload_detail();
  EXPECT_TRUE(rig.loop().payload_derated());
  // §31.4: hold speed = min(30, 20) deg/s * 0.5 derate factor = 10 deg/s.
  EXPECT_NEAR(rig.loop().hold_speed_effective(), 10.0 * kDeg, 1e-9);

  // §42.1: the web snapshot carries the payload status.
  const auto snap = rig.loop().telemetry().snapshot();
  EXPECT_EQ(snap.payload_profile_name, "test_payload");
  EXPECT_EQ(snap.payload_profile_status, "mismatch");
  EXPECT_TRUE(snap.payload_derated);
  EXPECT_FALSE(snap.payload_check_active);

  // Payload back to the commissioned mass: manual re-verification passes
  // again and clears the derate (§28.5 repeatable commissioning).
  rig.sim().set_response_tau(AxisId::Yaw, 0.05);
  const auto res2 = rig.loop().submit_command("start_payload_verification", "");
  ASSERT_TRUE(res2.ok) << res2.error;
  EXPECT_TRUE(rig.wait_check_done(t)) << rig.loop().payload_detail();
  EXPECT_EQ(rig.loop().payload_status(), PayloadStatus::Ok)
      << rig.loop().payload_detail();
  EXPECT_FALSE(rig.loop().payload_derated());
  EXPECT_NEAR(rig.loop().hold_speed_effective(), 20.0 * kDeg, 1e-9);
}

TEST(PayloadDaemon, NoProfileMeansNoProfileAndNoAutoCheck) {
  PayloadRig rig(/*auto_verify=*/true);
  TimeNs t = 0;
  rig.run_to_ready(t);
  ASSERT_TRUE(rig.loop().homed() && rig.loop().at_ready());

  // No profile loaded: status stays NoProfile and the §27 stage is skipped
  // (verification is meaningless without a baseline; profiling is §44's job).
  for (int i = 0; i < 1000; ++i) rig.step(t);
  EXPECT_EQ(rig.loop().payload_status(), PayloadStatus::NoProfile);
  EXPECT_FALSE(rig.loop().payload_check_active());
  EXPECT_FALSE(rig.loop().payload_derated());
  EXPECT_EQ(rig.loop().phase(), ota::Phase::Hold);

  // A manual check without a profile still runs the motion battery and
  // reports NoProfile (measured, but nothing to compare against).
  const auto res = rig.loop().submit_command("start_payload_verification", "");
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_TRUE(rig.wait_check_done(t)) << rig.loop().payload_detail();
  EXPECT_EQ(rig.loop().payload_status(), PayloadStatus::NoProfile)
      << rig.loop().payload_detail();
  EXPECT_FALSE(rig.loop().payload_derated());
}

TEST(PayloadDaemon, ProfileRoundTripsThroughTheStore) {
  const std::string dir =
      std::string("/tmp/ota_daemon_profiles_") + std::to_string(::getpid());
  const auto prof = commission_baseline();
  PayloadProfileStore store(dir);
  std::string err;
  ASSERT_TRUE(store.save(prof, err)) << err;
  PayloadProfile out;
  ASSERT_TRUE(store.load("test_payload", out, err)) << err;
  EXPECT_DOUBLE_EQ(out.pitch.v_max_rad_s, prof.pitch.v_max_rad_s);
  EXPECT_DOUBLE_EQ(out.yaw.baseline.step_pos.rise_time_s,
                   prof.yaw.baseline.step_pos.rise_time_s);
  EXPECT_TRUE(out.pitch.baseline.valid);
  EXPECT_TRUE(out.yaw.baseline.valid);
  ::system(("rm -rf " + dir).c_str());
}
