// End-to-end tests of the daemon-side payload verification (§27, §31.3,
// §31.4, §42.1): baseline comparison on the SimMotorBackend, automatic
// derating on mismatch, telemetry publication, and manual re-verification.
// No CAN, no motor, no camera — the sim plant is the only axis hardware.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

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

// A sim backend that records the drive-configuration calls the control loop
// issues at payload-check start (P6). The check must raise BOTH axes' current
// limit and their inner speed-loop gains so a 2 deg position-mode step can be
// driven at the commanded rate against the pitch's gravity load. The sim plant
// has no drive-internal velocity loop (its mass dependence is a fixed tau), so
// the gain writes are a no-op in sim — we only want to observe that the loop
// issues them with the configured values, for both axes.
class RecordingGainsBackend : public SimMotorBackend {
 public:
  explicit RecordingGainsBackend(double dt_s) : SimMotorBackend(dt_s) {}
  void set_current_limit(AxisId axis, double limit_cur_a) override {
    SimMotorBackend::set_current_limit(axis, limit_cur_a);
    cur_calls_.push_back({axis, limit_cur_a});
  }
  void set_speed_loop_gains(AxisId axis, double spd_kp,
                            double spd_ki) override {
    SimMotorBackend::set_speed_loop_gains(axis, spd_kp, spd_ki);
    gain_calls_.push_back({axis, spd_kp, spd_ki});
  }
  struct CurCall {
    AxisId axis;
    double amps;
  };
  struct GainCall {
    AxisId axis;
    double kp;
    double ki;
  };
  std::vector<CurCall> cur_calls_;
  std::vector<GainCall> gain_calls_;
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

// P6 blocker regression (ControlLoop level): the ready pose is the midpoint
// of the measured travel, and on this rig it sits far from the old static
// default region (0 +/- 20 deg) — begin() then aborted the whole check with
// "start pose outside the safe central region" (status Error). The check
// region is now per-axis and centered on the current pose, so verification
// must complete (NoProfile: measured, nothing to compare against) from an
// off-center ready pose. Here yaw stops 0.2..1.0 rad -> ready midpoint
// 0.6 rad (34.4 deg), outside the legacy region.
TEST(PayloadDaemon, CheckRunsFromOffCenterReadyPose) {
  PayloadRig rig(/*auto_verify=*/false);
  // 1.2 rad = 68.8 deg of travel (the plan's 120 deg band validates the
  // span as [60, 180] deg), with the ready midpoint 1.0 rad (57.3 deg)
  // far outside the legacy default region (center 0, span 20 deg).
  rig.sim().set_stops(AxisId::Yaw, 0.4, 1.6);
  rig.sim().set_position(AxisId::Yaw, 1.2);
  TimeNs t = 0;
  ASSERT_TRUE(rig.run_to_ready(t));
  // Sanity: the yaw ready pose is really outside the legacy default region.
  EXPECT_GT(rig.sim().position(AxisId::Yaw), 20.0 * kDeg);

  const auto res = rig.loop().submit_command("start_payload_verification", "");
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_TRUE(rig.wait_check_done(t)) << rig.loop().payload_detail();
  EXPECT_EQ(rig.loop().payload_status(), PayloadStatus::NoProfile)
      << rig.loop().payload_detail();
  EXPECT_FALSE(rig.loop().payload_derated());
  EXPECT_EQ(rig.loop().phase(), ota::Phase::Hold);
}

// Regression for P6 live Run A: the pitch check timed out (its settle band
// sat below the drive's steady-state position error) and the handler then
// silently switched to the YAW check — so the abort only logged 16 s after
// check start, with the SECOND axis's reason, and a healthy axis ran its
// full motion battery after its partner had failed. A failed axis must abort
// the whole check immediately, on the first axis's own reason, ~8 s (one
// move timeout) after check start.
TEST(PayloadDaemon, FailedFirstAxisAbortsTheWholeCheck) {
  PayloadRig rig(/*auto_verify=*/false);
  TimeNs t = 0;
  rig.run_to_ready(t);
  ASSERT_TRUE(rig.loop().homed() && rig.loop().at_ready());

  // Make the pitch response 200x slower (tau 10 s vs the 0.05 s default):
  // the 2 deg step is then a slow first-order ramp, ~55 % complete at the
  // 8 s per-move timeout, so the PosStep segment times out.
  rig.sim().set_response_tau(AxisId::Pitch, 10.0);
  const TimeNs t0 = t;
  const auto res = rig.loop().submit_command("start_payload_verification", "");
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_TRUE(rig.wait_check_done(t)) << rig.loop().payload_detail();

  EXPECT_EQ(rig.loop().payload_status(), PayloadStatus::Error)
      << rig.loop().payload_detail();
  // The abort reason is the PITCH's own segment — the yaw check must never
  // have started (old bug: the logged reason came from the yaw's timeout).
  EXPECT_NE(rig.loop().payload_detail().find("pos_step"), std::string::npos)
      << rig.loop().payload_detail();
  EXPECT_EQ(rig.loop().phase(), ota::Phase::Hold);
  // Aborted at the first move's 8 s timeout, not 8 s + the yaw's battery
  // (the old handler logged the abort ~16 s after check start).
  EXPECT_LE((t - t0) / 1e9, 9.0);
}

// P6: at payload-check start the control loop must raise BOTH axes' inner
// speed-loop gains (and current limit) so the 2 deg position-mode step can be
// driven at the commanded rate against the pitch's gravity load. The sim plant
// ignores the gain writes (no drive-internal velocity loop), so this asserts
// the loop ISSUES them — with the configured values, for both axes.
TEST(PayloadDaemon, CheckStartRaisesSpeedLoopGainsAndCurrentOnBothAxes) {
  auto backend = std::make_unique<RecordingGainsBackend>(0.005);
  RecordingGainsBackend* rec = backend.get();
  backend->set_stops(AxisId::Pitch, -1.0, 1.0);
  backend->set_stops(AxisId::Yaw, -1.0, 1.0);
  backend->set_position(AxisId::Pitch, 10.0 * kDeg);
  backend->set_position(AxisId::Yaw, 0.0);

  ControlLoop::Config cfg = make_cfg(/*auto_verify=*/false);
  // make_cfg leaves the gain knobs at their ControlLoop::Config defaults;
  // pin them explicitly so the test is robust to a default change.
  cfg.payload_check_spd_kp = 5.0;
  cfg.payload_check_spd_ki = 0.02;
  cfg.payload_check_current_a = 5.0;
  ControlLoop loop(cfg, std::move(backend));

  TimeNs t = 0;
  std::string err;
  ASSERT_TRUE(loop.start_homing(make_plan(), err)) << err;
  for (int i = 0; i < 20000 && !(loop.homed() && loop.at_ready()); ++i) {
    t += kDtNs;
    loop.step(t, kDtNs);
  }
  ASSERT_TRUE(loop.homed() && loop.at_ready());

  const auto res = loop.submit_command("start_payload_verification", "");
  ASSERT_TRUE(res.ok) << res.error;
  // The command only raises a flag; the check starts on the next control step
  // (so the gain writes land on the control thread). Run it to completion.
  bool active = false;
  for (int i = 0; i < 8000; ++i) {
    t += kDtNs;
    loop.step(t, kDtNs);
    if (loop.payload_check_active()) active = true;
    else if (active) break;
  }
  EXPECT_TRUE(active) << "the check never started";

  // Both axes get the speed-loop gains, with the configured values.
  ASSERT_EQ(rec->gain_calls_.size(), 2u);
  for (const auto& g : rec->gain_calls_) {
    EXPECT_DOUBLE_EQ(g.kp, 5.0);
    EXPECT_DOUBLE_EQ(g.ki, 0.02);
  }
  bool pitch_gained = false, yaw_gained = false;
  for (const auto& g : rec->gain_calls_) {
    if (g.axis == AxisId::Pitch) pitch_gained = true;
    if (g.axis == AxisId::Yaw) yaw_gained = true;
  }
  EXPECT_TRUE(pitch_gained) << "pitch gains not applied";
  EXPECT_TRUE(yaw_gained) << "yaw gains not applied";

  // Both axes get the check current limit (5 A) too. Homing also raises the
  // current limit (adaptive, §22), so there are more than two calls — assert
  // the check's 5 A value landed on both axes rather than the total count.
  bool pitch_cur = false, yaw_cur = false;
  for (const auto& c : rec->cur_calls_) {
    if (std::fabs(c.amps - 5.0) < 1e-9 && c.axis == AxisId::Pitch)
      pitch_cur = true;
    if (std::fabs(c.amps - 5.0) < 1e-9 && c.axis == AxisId::Yaw)
      yaw_cur = true;
  }
  EXPECT_TRUE(pitch_cur) << "pitch check current (5 A) not applied";
  EXPECT_TRUE(yaw_cur) << "yaw check current (5 A) not applied";
}
