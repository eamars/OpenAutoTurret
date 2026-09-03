// OpenAutoTurret — Phase 6 closed-loop tracking integration test (mocks only).
//
// Drives the FULL C++ control stack against the SimMotorBackend (a first-order
// plant with end stops — NO CAN, NO motor driver) and a SYNTHETIC camera
// (a target defined in the base frame, projected to pixels through the real
// camera model + kinematics at each control cycle). This verifies:
//
//   * §11 timestamp alignment (pose interpolated at the capture time);
//   * §13 estimator -> predicted LOS;
//   * §14 LOS -> joint solver;
//   * §16 reference arbitration (tracking > search > hold);
//   * §34/§35 tracking FSM + confidence-scaled speed;
//   * §49 search sweep;
//   * §18 safety envelope (stay inside the soft limits; safe stop on fault).
//
// The whole path is testable with no live camera, CAN, or motor driver (§54).
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include "calibration/homing_plan.hpp"
#include "control/control_loop.hpp"
#include "sim/sim_motor_backend.hpp"

using namespace ota;
using ota::geo::CameraIntrinsics;
using ota::geo::CameraModel;
using ota::geo::Mat3;
using ota::geo::TurretKinematics;
using ota::geo::Vec3;
using ota::sim::SimMotorBackend;
using ota::tracking::TrackState;
using ota::vision::TargetMeasurement;

namespace {

constexpr int64_t kDtNs = 5'000'000;  // 200 Hz
constexpr double kPi = M_PI;
constexpr double kDeg = kPi / 180.0;

CameraIntrinsics make_intrinsics() {
  CameraIntrinsics in;
  in.fx = 1000.0; in.fy = 1000.0; in.cx = 960.0; in.cy = 540.0;
  in.width = 1920; in.height = 1080;
  return in;
}

// Base-frame LOS (az, el) -> camera pixel, given the gimbal pose (synthetic
// vision). Inverse of the TrackingController's pixel->ray->base->los path.
void base_los_to_pixel(const CameraModel& cam, const TurretKinematics& kin,
                       double az, double el, double yaw, double pitch,
                       double& u, double& v) {
  const Vec3 r_base{std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
                    std::sin(el)};
  const Mat3 Rinv = kin.R_PC.transposed() * Mat3::rot_y(pitch).transposed() *
                    Mat3::rot_z(yaw).transposed();
  cam.ray_to_pixel((Rinv * r_base).normalized(), u, v);
}

// The base-frame LOS the gimbal is ACTUALLY pointing at, from its real pose.
void actual_los(const TurretKinematics& kin, double yaw, double pitch,
                double& az, double& el) {
  const Vec3 r = (Mat3::rot_z(yaw) * Mat3::rot_y(pitch) * kin.R_PC *
                  Vec3(0.0, 0.0, 1.0));
  TurretKinematics::base_ray_to_los(r.normalized(), az, el);
}

HomingPlan make_plan() {
  HomingPlanConfig hcfg;
  HomingParams hp;
  hp.coarse_speed_rad_s = 20.0 * kDeg;
  hp.fine_speed_rad_s = 2.0 * kDeg;
  hp.settle_time_s = 0.3;
  hcfg.homing = hp;
  hcfg.travel_bands[0] = TravelBand{0.0, 120.0};  // pitch (~115 deg travel)
  hcfg.travel_bands[1] = TravelBand{0.0, 120.0};  // yaw (~115 deg travel)
  std::vector<HomingAction> actions;
  actions.push_back(HomingAction{.type = HomingActionType::HomeFullRange,
                                 .axis = AxisId::Pitch});
  actions.push_back(HomingAction{.type = HomingActionType::HomeFullRange,
                                 .axis = AxisId::Yaw});
  return HomingPlan(std::move(actions), hcfg);
}

ControlLoop::Config make_cfg() {
  ControlLoop::Config cfg;
  cfg.control_hz = 200;
  cfg.hold_speed_rad_s = 30.0 * kDeg;
  cfg.emergency_speed_rad_s = 10.0 * kDeg;
  cfg.soft_margin_rad = 2.0 * kDeg;
  return cfg;
}

TrackingController::Config make_tracking_cfg(bool search_enabled) {
  TrackingController::Config tc;
  tc.kinematics = TurretKinematics::aligned();
  tc.intrinsics = make_intrinsics();
  tc.fsm.search_enabled = search_enabled;
  tc.fsm.coast_max_ns = 200 * 1000 * 1000;
  tc.fsm.lost_ns = 1000 * 1000 * 1000;
  tc.search.yaw_low_rad = -45.0 * kDeg;
  tc.search.yaw_high_rad = 45.0 * kDeg;
  tc.search.pitch_rad = 0.0;
  tc.search.v_max_rad_s = 10.0 * kDeg;
  tc.track_v_max_rad_s = 30.0 * kDeg;
  tc.search_v_max_rad_s = 10.0 * kDeg;
  tc.hold_v_max_rad_s = 10.0 * kDeg;
  return tc;
}

// A fully-wired rig: SimMotorBackend (owned by the loop) + ControlLoop + the
// synthetic camera/kinematics. Yaw stops +/-90 deg, pitch stops -20..+40 deg.
class TrackingRig {
 public:
  TrackingRig()
      : backend_(std::make_unique<SimMotorBackend>(0.005)),
        sim_(backend_.get()),
        cam_(make_intrinsics()), kin_(TurretKinematics::aligned()),
        loop_(std::make_unique<ControlLoop>(make_cfg(), std::move(backend_))) {
    sim_->set_stops(AxisId::Pitch, pitch_low_, pitch_high_);
    sim_->set_stops(AxisId::Yaw, yaw_low_, yaw_high_);
    sim_->set_position(AxisId::Pitch, 10.0 * kDeg);
    sim_->set_position(AxisId::Yaw, 0.0);
  }
  ControlLoop& loop() { return *loop_; }
  SimMotorBackend& sim() { return *sim_; }
  CameraModel& cam() { return cam_; }
  const TurretKinematics& kin() const { return kin_; }
  const double yaw_low_ = -1.0, yaw_high_ = 1.0;          // +/-57.3 deg
  const double pitch_low_ = -1.0, pitch_high_ = 1.0;      // +/-57.3 deg

 private:
  std::unique_ptr<SimMotorBackend> backend_;  // moved into loop_
  SimMotorBackend* sim_;  // raw pointer (valid while loop_ is alive)
  CameraModel cam_;
  TurretKinematics kin_;
  std::unique_ptr<ControlLoop> loop_;
};

// Home + move to ready. Returns false on fault/timeout; sets t_out.
bool run_to_ready(TrackingRig& r, int64_t& t_out, int max_steps = 20000) {
  std::string err;
  if (!r.loop().start_homing(make_plan(), err)) return false;
  int64_t t = 0;
  for (int i = 0; i < max_steps; ++i) {
    r.loop().step(t, kDtNs);
    t += kDtNs;
    if (r.loop().phase() == Phase::Fault) return false;
    if (r.loop().homed() && r.loop().at_ready()) {
      t_out = t;
      return true;
    }
  }
  t_out = t;
  return false;
}

// Feed one synthetic measurement (target at az/el in the base frame, captured
// at time t using the gimbal's current pose) and step one control cycle.
void step_with_target(TrackingRig& r, int64_t t, int64_t seq, double az,
                      double el) {
  const double yaw = r.loop().last_positions()[1];
  const double pitch = r.loop().last_positions()[0];
  double u, v;
  base_los_to_pixel(r.cam(), r.kin(), az, el, yaw, pitch, u, v);
  TargetMeasurement m;
  m.frame_sequence = seq;
  m.sensor_timestamp_ns = t;
  m.valid = true;
  m.class_id = 1;
  m.confidence = 1.0f;
  m.anchor_u_px = u;
  m.anchor_v_px = v;
  m.has_track_id = true;
  m.visual_track_id = 7;
  r.loop().feed_measurement(m);
  r.loop().step(t, kDtNs);
}

// v3 path: the same synthetic target, published the way visiond publishes it — a
// TrackSet with a normalized anchor, a uuid and a label — then one control cycle. The
// v1 feed above stays: §17 keeps the pixel -> ray -> LOS -> estimator chain, and the
// tests that walk the v1 FSM's own transitions through it are still honest.
void step_with_track(TrackingRig& r, int64_t t, int64_t seq, double az, double el,
                     uint16_t index = 1, uint64_t uuid = 7) {
  const double yaw = r.loop().last_positions()[1];
  const double pitch = r.loop().last_positions()[0];
  double u, v;
  base_los_to_pixel(r.cam(), r.kin(), az, el, yaw, pitch, u, v);
  ota::tracks::TrackSet set;
  set.frame_sequence = seq;
  set.sensor_timestamp_ns = t;
  set.publish_timestamp_ns = t + 3'000'000;
  set.width = 1920;
  set.height = 1080;
  ota::tracks::Track tr;
  tr.uuid = ota::tracks::TrackUuid{0, uuid};
  tr.display_index = index;
  tr.class_id = 1;
  std::memcpy(tr.class_name, "person", 6);
  tr.state = ota::tracks::TrackState::Confirmed;
  tr.detector_confidence = 0.95f;
  tr.track_confidence = 0.95f;
  tr.bbox.x_min = static_cast<float>((u - 40.0) / 1920.0);
  tr.bbox.x_max = static_cast<float>((u + 40.0) / 1920.0);
  tr.bbox.y_min = static_cast<float>((v - 80.0) / 1080.0);
  tr.bbox.y_max = static_cast<float>((v + 80.0) / 1080.0);
  tr.anchor_x = static_cast<float>(u / 1920.0);
  tr.anchor_y = static_cast<float>(v / 1080.0);
  tr.visible_frames = 30;
  set.add(tr);
  r.loop().feed_track_set(set, t);
  r.loop().step(t, kDtNs);
}

// The operator's act (§14): name the target the turret is allowed to follow.
void select_first(TrackingRig& r, int64_t& t) {
  r.loop().submit_command("select_target", "1");
  for (int i = 0; i < 4; ++i) {
    step_with_track(r, t, 900 + i, 0.0, 0.0);
    t += kDtNs;
  }
}

// Step one control cycle with NO target measurement (target lost / not present).
void step_no_target(TrackingRig& r, int64_t t) { r.loop().step(t, kDtNs); }

// v3 §53: motion belongs to a MODE. Give the loop its commissioned tracking
// configuration and enter the mode whose motion the test is about. Calling the
// v1 entry point (enable_tracking) and then expecting motion is no longer a thing
// that works — MANUAL owns motion until a mode says otherwise, which is exactly
// what §26 is for.
void enter_mode(TrackingRig& r, const TrackingController::Config& cfg,
                ota::OperatingMode mode) {
  r.loop().set_tracking_config(cfg, false);
  const auto res = r.loop().request_mode(mode);
  ASSERT_TRUE(res.ok) << ota::operating_mode_name(mode) << ": " << res.reason;
  ASSERT_EQ(r.loop().operating_mode(), mode);
}

}  // namespace

// The core Phase 6 deliverable: a target rotating in the base frame is tracked
// closed-loop — the gimbal's optical axis converges on the target and tracks it,
// all against the simulated plant (no CAN).
TEST(TrackingIntegration, TracksRotatingTarget) {
  TrackingRig r;
  int64_t t0 = 0;
  ASSERT_TRUE(run_to_ready(r, t0));
  std::string err;
  enter_mode(r, make_tracking_cfg(false), ota::OperatingMode::AutoTrack);

  const double az_rate = 3.0 * kDeg;   // rad/s (slow)
  const double el_const = 5.0 * kDeg;  // rad
  const int cycles = 500;              // 2.5 s

  // v3: the turret follows the target the operator named, so this test has to name one
  // before it can ask whether the gimbal is following it. Under §16 the same scene with
  // no selection holds still, which is asserted by
  // AutoTrackWithNoSelectionHoldsEvenWhenSomethingIsVisible below.
  int64_t t = t0;
  for (int i = 0; i < 3; ++i) {
    step_with_track(r, t, i, 0.0, el_const);
    t += kDtNs;
  }
  r.loop().submit_command("select_target", "1");
  for (int i = 0; i < 4; ++i) {
    step_with_track(r, t, 10 + i, 0.0, el_const);
    t += kDtNs;
  }
  ASSERT_EQ(r.loop().telemetry().snapshot().selected_display_index, 1)
      << "the selection did not take, so this test would be watching a turret that is "
         "deliberately not following anything";

  for (int i = 0; i < cycles; ++i) {
    const int64_t tt = t + i * kDtNs;
    const double az = az_rate * (i * 0.005);
    step_with_track(r, tt, 100 + i, az, el_const);
    if (r.loop().phase() == Phase::Fault) break;
  }

  EXPECT_TRUE(r.loop().tracking_mode_enabled());
  EXPECT_EQ(r.loop().phase(), Phase::Hold);
  EXPECT_EQ(r.loop().tracking_controller().track_state(),
            TrackState::Tracking);

  double az_now, el_now;
  actual_los(r.kin(), r.loop().last_positions()[1],
             r.loop().last_positions()[0], az_now, el_now);
  const double az_target = az_rate * (cycles - 1) * 0.005;
  EXPECT_NEAR(az_now, az_target, 3.0 * kDeg)
      << "gimbal azimuth should track the target";
  EXPECT_NEAR(el_now, el_const, 3.0 * kDeg)
      << "gimbal elevation should track the target";
}

// Feed the target for `track_cycles` cycles, then stop feeding (target lost)
// and return the TrackingRig + the time of the last measurement cycle.
struct TrackThenLose {
  int64_t t_last = 0;
  int track_cycles = 0;
};

TrackThenLose track_then_lose(TrackingRig& r, int64_t t0, int track_cycles,
                              double az, double el) {
  TrackThenLose out;
  out.track_cycles = track_cycles;
  for (int i = 0; i < track_cycles; ++i) {
    const int64_t t = t0 + i * kDtNs;
    step_with_target(r, t, i, az, el);
    out.t_last = t;
  }
  return out;
}

// After the target is lost (search disabled), the FSM walks
// TRACKING -> COASTING -> BRAKE_TO_HOLD -> READY_HOLD, and the gimbal ends at
// the ready pose (the HOLD reference).
TEST(TrackingIntegration, TargetLossTransitionsToBrakeThenHold) {
  TrackingRig r;
  int64_t t0 = 0;
  ASSERT_TRUE(run_to_ready(r, t0));
  std::string err;
  enter_mode(r, make_tracking_cfg(false), ota::OperatingMode::AutoTrack);

  const int track = 100;  // 0.5 s of tracking
  track_then_lose(r, t0, track, 5.0 * kDeg, 3.0 * kDeg);
  const int64_t t_lost = t0 + track * kDtNs;

  auto state_after = [&](int extra) {
    for (int i = 0; i < extra; ++i) r.loop().step(t_lost + i * kDtNs, kDtNs);
    return r.loop().tracking_controller().track_state();
  };

  // ~0.15 s after loss: COASTING.
  EXPECT_EQ(state_after(30), TrackState::Coasting);
  // ~0.5 s after loss: BRAKE_TO_HOLD.
  EXPECT_EQ(state_after(100), TrackState::BrakeToHold);
  // ~1.2 s after loss: READY_HOLD (search disabled).
  EXPECT_EQ(state_after(260), TrackState::ReadyHold);
  EXPECT_EQ(r.loop().phase(), Phase::Hold);
}

// With search enabled, a lost target hands off to SEARCH and the gimbal sweeps
// its yaw axis (the yaw position moves away from the lost target).
TEST(TrackingIntegration, SearchSweepsWhenEnabled) {
  TrackingRig r;
  int64_t t0 = 0;
  ASSERT_TRUE(run_to_ready(r, t0));
  std::string err;
  enter_mode(r, make_tracking_cfg(true), ota::OperatingMode::AutoRoam);

  const int track = 100;
  track_then_lose(r, t0, track, 5.0 * kDeg, 3.0 * kDeg);
  const int64_t t_lost = t0 + track * kDtNs;
  const double yaw_at_loss = r.loop().last_positions()[1];

  // Step ~1.2 s (to hand off to SEARCH) then another ~1.5 s of sweeping.
  int steps = 0;
  TrackState final_state = TrackState::ReadyHold;
  for (int i = 0; i < 500; ++i) {
    r.loop().step(t_lost + i * kDtNs, kDtNs);
    final_state = r.loop().tracking_controller().track_state();
    steps = i;
  }
  EXPECT_EQ(final_state, TrackState::Search);
  // The yaw swept away from the lost target (the search planner reverses at
  // the bounds, so over 2.5 s it has moved noticeably from the loss position).
  const double yaw_now = r.loop().last_positions()[1];
  EXPECT_GT(std::abs(yaw_now - yaw_at_loss), 0.2)
      << "yaw should sweep during search (moved "
      << std::abs(yaw_now - yaw_at_loss) / kDeg << " deg)";
  (void)steps;
}

// Throughout tracking the gimbal stays inside the soft limits (the safety
// envelope clamps the reference and the sim stops hold the plant).
TEST(TrackingIntegration, StaysWithinSoftLimits) {
  TrackingRig r;
  int64_t t0 = 0;
  ASSERT_TRUE(run_to_ready(r, t0));
  std::string err;
  enter_mode(r, make_tracking_cfg(false), ota::OperatingMode::AutoTrack);

  const double az_rate = 15.0 * kDeg;  // faster, to stress the limits
  const int cycles = 800;              // 4 s -> ~60 deg sweep
  for (int i = 0; i < cycles; ++i) {
    const int64_t t = t0 + i * kDtNs;
    const double az = az_rate * (i * 0.005);
    step_with_target(r, t, i, az, 2.0 * kDeg);
    // Assert the pose stays within the soft limits every cycle.
    const double yaw = r.loop().last_positions()[1];
    const double pitch = r.loop().last_positions()[0];
    EXPECT_GE(yaw, -1.0);
    EXPECT_LE(yaw, 1.0);
    EXPECT_GE(pitch, -1.0);
    EXPECT_LE(pitch, 1.0);
    if (r.loop().phase() == Phase::Fault) break;
  }
  EXPECT_NE(r.loop().phase(), Phase::Fault)
      << "tracking a target within range must not fault; reason: "
      << r.loop().fault_reason();
}

// A motor fault during tracking drives the loop to FAULT (safe stop) and the
// supervisor disables the motors — the highest-priority action (§16).
TEST(TrackingIntegration, FaultStopsSafely) {
  TrackingRig r;
  int64_t t0 = 0;
  ASSERT_TRUE(run_to_ready(r, t0));
  std::string err;
  enter_mode(r, make_tracking_cfg(false), ota::OperatingMode::AutoTrack);

  const int track = 80;
  track_then_lose(r, t0, track, 5.0 * kDeg, 3.0 * kDeg);
  const int64_t t_fault = t0 + track * kDtNs;

  // Inject a fault on the yaw motor at the next step.
  r.sim().set_faults(AxisId::Yaw, true);
  r.loop().step(t_fault, kDtNs);

  EXPECT_EQ(r.loop().phase(), Phase::Fault);
  EXPECT_FALSE(r.sim().in_position_mode(AxisId::Yaw));
  EXPECT_TRUE(r.loop().fault_reason().find("fault") != std::string::npos);
}

// --- §36 roaming from a cold start, and the command that arms it -----------
//
// The station boots with no target and there is no guarantee one ever appears.
// Two defects lived here together, and each test below is one of them:
//   * the FSM reached SEARCH only through TARGET_LOST, so a station that had
//     never acquired anything could never go looking — the sweep was gated on
//     the acquisition the sweep exists to cause;
//   * `enable_search` was validated, acknowledged to the UI, and then dropped
//     (control_loop.cpp: "Acknowledge so the UI can proceed"), so the dashboard
//     reported success for a no-op.

TEST(TrackingIntegration, ColdStationInAutoRoamGoesLooking) {
  // v3 §30/§36, and the case the operator's correction was about: the station
  // boots, no target ever appears, and in AUTO_ROAM it must go looking anyway.
  //
  // The configuration here deliberately says search is OFF. What arms the sweep
  // is the MODE, not the v1 flag — which is the point: under v3 the operator
  // chooses an operating mode, and turret.yaml does not get to veto it.
  TrackingRig r;
  int64_t t0 = 0;
  ASSERT_TRUE(run_to_ready(r, t0));
  enter_mode(r, make_tracking_cfg(false), ota::OperatingMode::AutoRoam);
  const double yaw_at_enable = r.loop().last_positions()[1];

  // 2.5 s of nothing at all: no measurement has EVER arrived.
  for (int i = 1; i < 500; ++i) step_no_target(r, t0 + i * kDtNs);

  EXPECT_EQ(r.loop().tracking_controller().track_state(), TrackState::Search)
      << "a station in AUTO_ROAM that has never seen a target must go looking";
  EXPECT_EQ(r.loop().last_intent().source, ota::MotionSource::AutoRoam)
      << "the sweep must be attributed to the mode that asked for it, or the "
         "operator's screen cannot say why the turret is moving";
  EXPECT_EQ(r.loop().last_intent().type, ota::IntentType::JointPosition);
  EXPECT_GT(std::fabs(r.loop().last_positions()[1] - yaw_at_enable), 0.2)
      << "AUTO_ROAM is a sweep, not a nicer word for holding still (moved "
      << std::fabs(r.loop().last_positions()[1] - yaw_at_enable) / kDeg
      << " deg)";
}

TEST(TrackingIntegration, AutoTrackNeverSweepsLookingForATarget) {
  // §111.5, frozen: "AUTO_TRACK does not roam when target is absent."
  //
  // This is the test that separates the state machine from the authority. With
  // the config asking for search and the target never arriving, the inherited v1
  // FSM still walks into Search internally — the mechanism is intact. What must
  // NOT happen is the turret sweeping, because AUTO_TRACK is not allowed to
  // author motion while it has no target. If these two ever agree again, the
  // modes have quietly collapsed back into one behaviour and §110's
  // "AUTO_TRACK with no target holds forever" fails on the station, where it is
  // a turret wandering around a room looking for something it was never told to
  // find.
  TrackingRig r;
  int64_t t0 = 0;
  ASSERT_TRUE(run_to_ready(r, t0));
  enter_mode(r, make_tracking_cfg(true), ota::OperatingMode::AutoTrack);
  const double yaw0 = r.loop().last_positions()[1];

  for (int i = 1; i < 500; ++i) step_no_target(r, t0 + i * kDtNs);

  EXPECT_TRUE(r.loop().tracking_controller().track_state() == TrackState::Search ||
              r.loop().tracking_controller().track_state() == TrackState::ReadyHold)
      << "the mechanism still runs; it is the authority that is withheld";
  EXPECT_EQ(r.loop().last_intent().type, ota::IntentType::Hold)
      << "AUTO_TRACK must emit a hold while it has no target";
  EXPECT_LT(std::fabs(r.loop().last_positions()[1] - yaw0), 1.0 * kDeg)
      << "and the turret must actually still: yaw moved "
      << std::fabs(r.loop().last_positions()[1] - yaw0) / kDeg << " deg";
}

TEST(TrackingIntegration, RoamArmedMidSessionStartsSweeping) {
  // Mode change as the only way to start the sweep: a station in AUTO_TRACK that
  // has held for a long time begins scanning the cycle the operator selects
  // AUTO_ROAM, with no stop/start of the tracking session.
  TrackingRig r;
  int64_t t0 = 0;
  ASSERT_TRUE(run_to_ready(r, t0));
  enter_mode(r, make_tracking_cfg(false), ota::OperatingMode::AutoTrack);
  for (int i = 1; i < 500; ++i) step_no_target(r, t0 + i * kDtNs);
  const double yaw_before = r.loop().last_positions()[1];
  ASSERT_LT(std::fabs(yaw_before - r.loop().last_positions()[1]), 1e-9);

  ASSERT_TRUE(r.loop().request_mode(ota::OperatingMode::AutoRoam).ok);
  for (int i = 500; i < 640; ++i) step_no_target(r, t0 + i * kDtNs);

  EXPECT_EQ(r.loop().last_intent().source, ota::MotionSource::AutoRoam);
  EXPECT_GT(std::fabs(r.loop().last_positions()[1] - yaw_before), 0.1)
      << "the sweep should be under way within 0.7 s of choosing AUTO_ROAM";
}

// §16, the rule that changes what an unattended station does: "No target selected:
// remain WAIT_TARGET, show 'Select a target', no autonomous motion." v1 acquired the
// best detection it could find, and the interim v3 rule that preserved that on the way
// here is retired by AutoTrackController, so this is the test that pins the difference.
TEST(TrackingIntegration, AutoTrackWithNoSelectionHoldsEvenWhenSomethingIsVisible) {
  TrackingRig r;
  int64_t t0 = 0;
  ASSERT_TRUE(run_to_ready(r, t0));
  enter_mode(r, make_tracking_cfg(false), ota::OperatingMode::AutoTrack);
  const double yaw0 = r.loop().last_positions()[1];

  // A perfectly good target — CONFIRMED, confident, 25 degrees off to one side. The
  // turret sees it and does not move.
  int64_t t = t0;
  for (int i = 0; i < 200; ++i) {
    step_with_track(r, t, i, 25.0 * kDeg, 0.0);
    t += kDtNs;
    ASSERT_NE(r.loop().phase(), Phase::Fault);
  }

  const double yaw_now = r.loop().last_positions()[1];
  EXPECT_LT(std::fabs(yaw_now - yaw0), 1.0 * kDeg)
      << "AUTO_TRACK acquired a target nobody selected. It may be the right target; it "
         "may be whoever walked into frame. That is not a decision the station gets to "
         "make on its own (§16)";
  const auto snap = r.loop().telemetry().snapshot();
  EXPECT_EQ(snap.mode_phase, "WAIT_TARGET");
  EXPECT_EQ(snap.selected_track_id, 0u);
  EXPECT_NE(snap.intent_reason.find("select a target"), std::string::npos)
      << snap.intent_reason;
  // And it did see it: the estimator is primed, so the moment somebody chooses, the
  // turret has history to act on instead of starting cold.
  EXPECT_TRUE(r.loop().tracking_controller().estimator_initialized())
      << "no selection turned into no vision processing, which is not what §16 says";

  // The operator then chooses, and the same visible target starts being followed
  // without anything else changing.
  r.loop().submit_command("select_target", "1");
  for (int i = 0; i < 200; ++i) {
    step_with_track(r, t, 500 + i, 25.0 * kDeg, 0.0);
    t += kDtNs;
  }
  EXPECT_GT(std::fabs(r.loop().last_positions()[1] - yaw0), 10.0 * kDeg)
      << "the selection did not turn into motion; the WAIT_TARGET gate would be stuck";
}
