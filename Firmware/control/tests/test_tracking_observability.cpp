// OpenAutoTurret — tracking / vision / profile OBSERVABILITY tests.
//
// These cover what the station operator and the web UI actually see, and which
// a live probe cannot pin down on its own:
//   * `at_ready()` must still mean "holding the safe ready pose" when tracking
//     is enabled (the operator's P0 hold criterion + the §27 auto-check gate);
//   * the §6.3 snapshot must carry the vision-link health (a silent visiond
//     must be visible, not merely absent);
//   * a runtime payload-profile selection (§42.2) is NOT commissioned until it
//     is verified (§31.3) — the status must say so.
// Driven against SimMotorBackend: NO CAN, NO motor driver, NO camera.
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "calibration/homing_plan.hpp"
#include "common/time.hpp"
#include "control/control_loop.hpp"
#include "control/tracking_controller.hpp"
#include "payload/payload_profile.hpp"
#include "sim/sim_motor_backend.hpp"
#include "vision/vision_ingest.hpp"

#include <gtest/gtest.h>

using namespace ota;

namespace {

constexpr int64_t kDtNs = 5'000'000;  // 200 Hz
constexpr int kMaxSteps = 60000;

HomingPlan make_plan() {
  HomingPlanConfig hcfg;
  HomingParams hp;
  hp.coarse_speed_rad_s = 20.0 * kDeg2Rad;
  hp.fine_speed_rad_s = 2.0 * kDeg2Rad;
  hp.settle_time_s = 0.3;
  hcfg.homing = hp;
  hcfg.travel_bands[0] = TravelBand{0.0, 115.0};
  hcfg.travel_bands[1] = TravelBand{0.0, 115.0};
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
  cfg.hold_speed_rad_s = 30.0 * kDeg2Rad;
  cfg.emergency_speed_rad_s = 10.0 * kDeg2Rad;
  cfg.soft_margin_rad = 2.0 * kDeg2Rad;
  cfg.park.park_logical_deg = {30.0, 60.0};
  cfg.park.speed_deg_s = 50.0;
  cfg.park.dwell_ms = 300;
  cfg.park.pos_tol_deg = 0.5;
  cfg.park.vel_tol_deg_s = 1.0;
  cfg.park.min_soft_margin_deg = 2.0;
  return cfg;
}

// Boot + home + reach the ready pose. `auto_tracking` mirrors what the daemon
// does when tracking.enabled is true in the config (§38.1 gate).
bool home_to_ready(ControlLoop& loop, int64_t& t, bool auto_tracking) {
  TrackingController::Config tc;  // sane defaults
  loop.set_tracking_config(tc, auto_tracking);
  std::string err;
  if (!loop.start_homing(make_plan(), err)) return false;
  t = 0;
  for (int i = 0; i < kMaxSteps; ++i) {
    loop.step(t, kDtNs);
    t += kDtNs;
    if (loop.phase() == Phase::Fault) return false;
    if (loop.homed() && loop.at_ready()) return true;
  }
  return false;
}

std::string temp_dir() {
  char tmpl[] = "/tmp/ota_observability_XXXXXX";
  const char* d = mkdtemp(tmpl);
  EXPECT_NE(d, nullptr);
  return d ? d : "/tmp/ota_observability_fallback";
}

payload::PayloadProfile make_profile(const std::string& name) {
  payload::PayloadProfile p;
  p.name = name;
  p.created_ns = 123;
  p.config_revision = "test";
  p.hardware = "sim";
  p.notes = "observability test";
  for (int i = 0; i < 2; ++i) {
    auto& ax = p.axis(static_cast<AxisId>(i));
    ax.v_max_rad_s = 0.5;
    ax.a_max_rad_s2 = 1.5;
    ax.j_max_rad_s3 = 4.5;
    ax.baseline.step_pos.valid = true;
    ax.baseline.step_pos.rise_time_s = 0.11;
    ax.baseline.step_pos.settling_time_s = 0.20;
    ax.baseline.step_pos.peak_effort_nm = 0.7;
    ax.baseline.step_pos.tracking_rms_rad = 0.001;
    ax.baseline.step_neg = ax.baseline.step_pos;
    ax.baseline.triangle_rms_rad = 0.0005;
    ax.baseline.hold_effort_nm = 0.2;
    ax.baseline.max_verified_speed_rad_s = 0.35;
    ax.baseline.valid = true;
  }
  return p;
}

}  // namespace

// `at_ready()` with tracking ON: the reference arbitration is holding the ready
// pose, so the station IS at the ready pose. Regression: the tracking branch of
// Phase::Hold used to clear it unconditionally, which silenced the operator's
// "at ready pose" signal and gated off the §27 auto payload check.
TEST(TrackingObservability, AtReadyReportedWhileTrackingIsEnabled) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  ControlLoop loop(make_cfg(), std::move(backend));
  int64_t t = 0;
  ASSERT_TRUE(home_to_ready(loop, t, /*auto_tracking=*/true))
      << "never reached the ready pose with tracking enabled";
  EXPECT_TRUE(loop.at_ready());
  EXPECT_EQ(loop.phase(), Phase::Hold);
  loop.step(t, kDtNs);
  EXPECT_TRUE(loop.at_ready()) << "holding the ready pose must stay 'at ready'";
  EXPECT_EQ(loop.telemetry().snapshot().phase, "hold");
  // Tracking is armed but idle: no target, so nothing may move.
  EXPECT_EQ(loop.telemetry().snapshot().track_state,
            tracking::TrackState::ReadyHold);
  EXPECT_FALSE(loop.telemetry().snapshot().tracking_active);
}

// With tracking OFF the flag behaves exactly as before (no behaviour change for
// the station's current `tracking.enabled: false` config).
TEST(TrackingObservability, AtReadyWithoutTrackingUnchanged) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  ControlLoop loop(make_cfg(), std::move(backend));
  int64_t t = 0;
  ASSERT_TRUE(home_to_ready(loop, t, /*auto_tracking=*/false));
  EXPECT_TRUE(loop.at_ready());
  EXPECT_FALSE(loop.telemetry().snapshot().tracking_active);
}

// The §6.3 vision block: a publisher that is connected but silent, and one that
// is gone, must be distinguishable from the snapshot alone.
TEST(TrackingObservability, SnapshotCarriesVisionHealth) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  ControlLoop loop(make_cfg(), std::move(backend));
  int64_t t = 0;
  ASSERT_TRUE(home_to_ready(loop, t, /*auto_tracking=*/false));

  vision::VisionLink link;
  loop.set_vision_link(&link);
  loop.step(t, kDtNs);
  t += kDtNs;
  auto snap = loop.telemetry().snapshot();
  EXPECT_FALSE(snap.vision_connected);
  EXPECT_EQ(snap.vision_frames, 0u);
  EXPECT_EQ(snap.vision_measurement_age_ms, -1)
      << "no measurement EVER must be -1, not a huge age";

  link.note_client_added();
  // The arrival stamp is in the loop's clock domain (in the daemon BOTH are
  // CLOCK_MONOTONIC; here the loop is driven by the test clock `t`).
  link.note_frame(41, static_cast<TimeNs>(t));
  link.note_frame(42, static_cast<TimeNs>(t));
  link.note_dropped();
  loop.step(t, kDtNs);
  t += kDtNs;
  snap = loop.telemetry().snapshot();
  EXPECT_TRUE(snap.vision_connected);
  EXPECT_EQ(snap.vision_frames, 2u);
  EXPECT_EQ(snap.vision_dropped, 1u);
  EXPECT_EQ(snap.vision_last_frame_sequence, 42u);
  EXPECT_GE(snap.vision_measurement_age_ms, 0);
  EXPECT_LE(snap.vision_measurement_age_ms, 50);

  // A stamp AHEAD of the loop clock (a clock-domain mistake, never a real
  // measurement) must clamp to 0 rather than publish a negative age.
  link.note_frame(43, static_cast<TimeNs>(t) + 10'000'000'000);
  loop.step(t, kDtNs);
  t += kDtNs;
  EXPECT_EQ(loop.telemetry().snapshot().vision_measurement_age_ms, 0);
  link.note_frame(44, static_cast<TimeNs>(t));

  // Publisher dies: connected goes false, the CUMULATIVE counters stay (an
  // operator must still see that frames were flowing before it dropped).
  link.note_client_removed();
  loop.step(t, kDtNs);
  snap = loop.telemetry().snapshot();
  EXPECT_FALSE(snap.vision_connected);
  EXPECT_EQ(snap.vision_frames, 4u) << "counters are cumulative since boot";

  // Detaching the link must not leave the snapshot showing a dead link.
  loop.set_vision_link(nullptr);
  loop.step(t, kDtNs);
  EXPECT_FALSE(loop.telemetry().snapshot().vision_connected);
}

// §42.2 + §31.3: selecting a profile at runtime is a LIMIT change, not a
// commissioning. The station must say "no_profile" until a verification passes.
TEST(TrackingObservability, RuntimeProfileSelectionRequiresVerification) {
  const std::string dir = temp_dir();
  payload::PayloadProfileStore store(dir);
  std::string err;
  ASSERT_TRUE(store.save(make_profile("alpha"), err)) << err;
  ASSERT_TRUE(store.save(make_profile("beta"), err)) << err;

  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  ControlLoop loop(make_cfg(), std::move(backend));
  loop.set_payload_profile_dir(dir);
  loop.set_payload_profile(make_profile("alpha"), /*commissioned=*/true);
  int64_t t = 0;
  ASSERT_TRUE(home_to_ready(loop, t, /*auto_tracking=*/false));
  loop.step(t, kDtNs);
  t += kDtNs;
  EXPECT_EQ(loop.telemetry().snapshot().payload_profile_name, "alpha");
  EXPECT_EQ(loop.telemetry().snapshot().payload_profile_status, "ok");

  auto res = loop.submit_command("select_payload_profile", "beta");
  EXPECT_TRUE(res.ok) << res.error;
  for (int i = 0; i < 10; ++i) {
    loop.step(t, kDtNs);
    t += kDtNs;
  }
  auto snap = loop.telemetry().snapshot();
  EXPECT_EQ(snap.payload_profile_name, "beta");
  EXPECT_EQ(snap.payload_profile_status, "no_profile")
      << "a runtime-selected profile is not commissioned (§31.3)";
  EXPECT_FALSE(snap.payload_check_active);

  // An unknown profile must be refused and change nothing.
  auto bad = loop.submit_command("select_payload_profile", "ghost");
  EXPECT_FALSE(bad.ok) << "unknown profile accepted";
  EXPECT_NE(bad.error.find("ghost"), std::string::npos)
      << bad.error << " — the reason must name the profile";
  for (int i = 0; i < 5; ++i) {
    loop.step(t, kDtNs);
    t += kDtNs;
  }
  EXPECT_EQ(loop.telemetry().snapshot().payload_profile_name, "beta");

  // Path traversal / injection attempts in the name are rejected at the web
  // layer (§42.2 command validation).
  EXPECT_FALSE(loop.submit_command("select_payload_profile", "../etc/passwd").ok);
  EXPECT_FALSE(loop.submit_command("select_payload_profile", "").ok);
  ::unlink((dir + "/alpha.yaml").c_str());
  ::unlink((dir + "/beta.yaml").c_str());
  ::rmdir(dir.c_str());
}

// The measurement hand-off is thread-safe by contract (§46): feed_measurement
// is called from the ingest thread while the control thread steps. Here the
// two are the SAME thread, which is exactly what must never deadlock or fault.
TEST(TrackingObservability, MeasurementHandoffWhileHoldingIsHarmless) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  ControlLoop loop(make_cfg(), std::move(backend));
  int64_t t = 0;
  ASSERT_TRUE(home_to_ready(loop, t, /*auto_tracking=*/false));

  vision::TargetMeasurement m;
  m.valid = false;  // no target: must not start tracking, must not fault
  m.frame_sequence = 1;
  m.sensor_timestamp_ns = static_cast<uint64_t>(t);
  for (int i = 0; i < 20; ++i) {
    loop.feed_measurement(m);
    loop.step(t, kDtNs);
    t += kDtNs;
    m.frame_sequence = static_cast<uint64_t>(i + 2);
    m.sensor_timestamp_ns = static_cast<uint64_t>(t);
  }
  EXPECT_EQ(loop.phase(), Phase::Hold);
  EXPECT_TRUE(loop.at_ready());
  EXPECT_NE(loop.phase(), Phase::Fault);
}
