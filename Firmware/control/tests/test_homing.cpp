// Unit tests for the sensorless precision homing state machine (architecture
// §22/§23/§26). The HomingController is driven against a simulated axis that
// responds to the DesiredState (target + speed limit) and stalls when it pushes
// into its end-stop — no CAN, no trajectory generator (§54 simulated plant).
#include <gtest/gtest.h>

#include <cmath>

#include "calibration/homing_controller.hpp"

namespace {

using ota::DesiredState;
using ota::HomingController;
using ota::HomingFeedback;
using ota::HomingParams;
using ota::kDeg2Rad;

constexpr double kDtS = 0.005;  // 200 Hz
constexpr int64_t kDtNs = 5'000'000;

// A simulated axis with a physical end-stop. It follows the DesiredState: it
// drives toward the target at the speed limit (first-order velocity response)
// and stalls — velocity to zero, position clamped, effort rising — when it
// pushes into the stop. Moving away from the stop releases it.
struct SimAxis {
  double stop_at = 1.0;        // end-stop position (rad)
  double drive_effort = 1.0;   // N·m while driving
  double contact_effort = 5.0; // N·m while pushing into the stop
  double noise = 0.05;         // rad/s velocity noise amplitude

  // p3f: a static-friction zone short of the stop (the measured 75-79 deg
  // pitch breakaway stall, which the contact detector misreads as a stop).
  // On the (stall_after_approaches+1)th and later speed-mode approaches, the
  // first entry into the band [zone_near, zone_far] from the stop stalls the
  // axis (v=0, plateau torque). Position-mode moves (the backoffs) break it.
  // sticky_stalls_left bounds how many entries stall (0 = none, >0 = that
  // many, so a retry can succeed once it is spent).
  bool has_zone = false;
  int stall_after_approaches = 2;  // stall the 3rd+ speed approach (2nd fine)
  int sticky_stalls_left = 0;      // 0 = zone disabled even if has_zone
  double zone_near_rad = 0.3 * kDeg2Rad;
  double zone_far_rad = 1.0 * kDeg2Rad;

  double q = 0.0;
  double v = 0.0;
  double torque = 0.0;
  bool at_stop = false;
  int stop_dir = 0;  // travel direction when the stop was reached

  bool was_moving_toward_stop_ = false;
  int speed_approach_count_ = 0;
  bool zone_stalled_ = false;

  void reset(double start_q, double stop) {
    q = start_q;
    v = 0.0;
    torque = 0.0;
    at_stop = false;
    stop_dir = 0;
    stop_at = stop;
    was_moving_toward_stop_ = false;
    speed_approach_count_ = 0;
    zone_stalled_ = false;
  }

  void step(const DesiredState& ds, int64_t t_ns) {
    const double to_stop = stop_at - q;
    // Speed mode: the commanded velocity is ds.velocity_rad_s (signed). The
    // drive's own velocity loop holds it — the plant moves at that constant
    // speed (first-order, ~50 ms). This mirrors the real CyberGear's speed-mode
    // behavior (the smooth source of motion, as opposed to a host-regenerated
    // moving position target, which stick-slips).
    // Position mode (the backoff moves, p3c fix): the drive's own position
    // loop drives toward ds.target_rad at up to ds.speed_rad_s, with full
    // torque from the first cycle — it releases from the stop and breaks any
    // static friction immediately (the velocity-mode P-term cannot; that is
    // what the position-mode backoff exists for).
    double v_cmd = 0.0;
    if (ds.position_move && !ds.hold) {
      const double err = ds.target_rad - q;
      v_cmd = std::copysign(
          std::min(std::fabs(err) / kDtS, ds.speed_rad_s), err);
    } else {
      v_cmd = ds.hold ? 0.0 : ds.velocity_rad_s;
    }

    // p3f static-friction zone (see the field comment). Counted on each
    // speed-mode approach start (rest/hold -> driving toward the stop); the
    // coarse approach is #1, the first fine #2, the second fine #3.
    const bool in_speed_mode = !ds.position_move;
    const bool moving_toward_stop =
        in_speed_mode && ((v_cmd > 0 && stop_at > q) || (v_cmd < 0 && stop_at < q));
    if (moving_toward_stop && !was_moving_toward_stop_) ++speed_approach_count_;
    was_moving_toward_stop_ = moving_toward_stop;
    const double dist_to_stop = std::fabs(stop_at - q);
    const bool in_zone_band = has_zone &&
                              dist_to_stop >= zone_near_rad &&
                              dist_to_stop <= zone_far_rad;
    if (in_speed_mode) {
      if (zone_stalled_) {
        if (!in_zone_band) {
          zone_stalled_ = false;  // a backoff pulled the axis out of the zone
        } else {
          // Still stuck: hold position, push with the stop-plateau torque
          // (clears the detector's effort gates -> a latched false contact).
          v = 0.0;
          torque = contact_effort * (v_cmd >= 0 ? 1 : -1);
          at_stop = false;
          return;
        }
      } else if (moving_toward_stop && in_zone_band &&
                 speed_approach_count_ > stall_after_approaches &&
                 sticky_stalls_left > 0) {
        zone_stalled_ = true;
        --sticky_stalls_left;
        v = 0.0;
        torque = contact_effort * (v_cmd > 0 ? 1 : -1);
        at_stop = false;
        return;
      }
    } else {
      zone_stalled_ = false;  // position mode (backoff) breaks the friction
    }

    // First-order velocity response (~50 ms time constant).
    const double alpha = kDtS / (0.05 + kDtS);
    v += alpha * (v_cmd - v);

    if (at_stop) {
      // At the stop it can only leave by travelling away from it (-stop_dir);
      // pushing into it (or holding) keeps it clamped at the stop.
      const bool into_stop = (stop_dir > 0 && v > 0) || (stop_dir < 0 && v < 0);
      if (into_stop || std::fabs(v) < 1e-6) {
        q = stop_at;
        v = 0.0;
      } else {
        q += v * kDtS;
        if (std::fabs(q - stop_at) > 1e-6) at_stop = false;
      }
    } else {
      // Not at the stop: move, and clamp if this step would push into it.
      const bool toward_stop = (v > 0 && to_stop > 0) || (v < 0 && to_stop < 0);
      if (toward_stop && std::fabs(v * kDtS) >= std::fabs(to_stop)) {
        q = stop_at;
        at_stop = true;
        v = 0.0;
        stop_dir = (v_cmd > 0) ? 1 : -1;
      } else {
        q += v * kDtS;
      }
    }

    // Effort is reported in the direction of travel (the contact detector
    // signs it by the approach direction). High when pushing into the stop,
    // moderate while driving, ~0 while holding.
    const bool pushing = at_stop &&
                         ((stop_dir > 0 && v_cmd > 0) || (stop_dir < 0 && v_cmd < 0));
    const int dir_cmd = (v_cmd >= 0) ? 1 : -1;
    const int dir_v = (v >= 0) ? 1 : -1;
    if (pushing) {
      torque = contact_effort * dir_cmd;
    } else if (std::fabs(v) > 1e-3) {
      torque = drive_effort * dir_v;
    } else {
      torque = 0.0;
    }
  }

  HomingFeedback feedback(int64_t t_ns) const {
    HomingFeedback fb;
    fb.t_ns = t_ns;
    fb.pos_rad = q;
    fb.vel_rad_s = v + std::sin(t_ns / 1e5) * noise;
    fb.torque_nm = torque + std::sin(t_ns / 3e4) * 0.1;
    fb.motor_fault = false;
    return fb;
  }
};

// Run the homing FSM to a terminal state (or max_steps). Returns the step count.
int run_homing(HomingController& hc, SimAxis& axis, int max_steps) {
  int64_t t = 0;
  for (int i = 0; i < max_steps; ++i) {
    HomingFeedback fb = axis.feedback(t);
    DesiredState ds = hc.step(fb);
    axis.step(ds, t);
    t += kDtNs;
    if (hc.terminal()) return i + 1;
  }
  return max_steps;
}

HomingParams test_params() {
  HomingParams p;
  p.coarse_speed_rad_s = 10.0 * kDeg2Rad;
  p.fine_speed_rad_s = 1.0 * kDeg2Rad;
  p.backoff_speed_rad_s = 10.0 * kDeg2Rad;
  p.backoff_rad = 5.0 * kDeg2Rad;
  p.small_backoff_rad = 2.0 * kDeg2Rad;
  p.repeatability_rad = 0.5 * kDeg2Rad;
  p.settle_time_s = 0.5;
  p.approach_timeout_s = 30.0;
  p.max_travel_rad = 150.0 * kDeg2Rad;
  p.arrival_tol_rad = 0.01;
  return p;
}

}  // namespace

TEST(Homing, CoarseDirectionReachesEndpointAndPasses) {
  SimAxis axis;
  axis.stop_at = 1.0;
  axis.reset(0.5, 1.0);  // start 0.5 rad before the stop, approach in +
  HomingController hc(ota::AxisId::Pitch, +1, test_params());

  int steps = run_homing(hc, axis, 6000);
  const ota::HomingResult& r = hc.result();
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Complete)
      << "failed: " << r.fail_reason;
  EXPECT_TRUE(r.valid);
  EXPECT_EQ(r.fine_samples, 2);
  // The validated fine contact should be at (or a hair into) the stop.
  EXPECT_NEAR(r.fine_contact_rad, 1.0, 0.01);
  // Repeatability within the limit.
  EXPECT_LE(r.repeatability_rad, test_params().repeatability_rad);
  SUCCEED();
}

TEST(Homing, NegativeDirectionReachesEndpointAndPasses) {
  SimAxis axis;
  axis.stop_at = -1.0;
  axis.reset(-0.5, -1.0);  // start 0.5 rad before the stop, approach in -
  HomingController hc(ota::AxisId::Yaw, -1, test_params());

  int steps = run_homing(hc, axis, 6000);
  const ota::HomingResult& r = hc.result();
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Complete)
      << "failed: " << r.fail_reason;
  EXPECT_TRUE(r.valid);
  EXPECT_NEAR(r.fine_contact_rad, -1.0, 0.01);
  SUCCEED();
}

TEST(Homing, NoContactFailsOnTimeout) {
  // The stop is far beyond the travel limit, so the approach never finds
  // contact and must fail (travel limit / timeout), leaving the axis unhomed.
  SimAxis axis;
  axis.stop_at = 100.0;  // never reached
  axis.reset(0.0, 100.0);
  HomingParams p = test_params();
  p.approach_timeout_s = 3.0;  // short timeout so the test is fast
  p.max_travel_rad = 2.0;      // 2 rad travel limit, well short of the stop
  HomingController hc(ota::AxisId::Pitch, +1, p);

  int steps = run_homing(hc, axis, 2000);
  const ota::HomingResult& r = hc.result();
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Failed)
      << "expected failure, got complete";
  EXPECT_FALSE(r.valid);
  EXPECT_FALSE(r.fail_reason.empty());
  SUCCEED();
}

TEST(Homing, MotorFaultFailsImmediately) {
  SimAxis axis;
  axis.stop_at = 1.0;
  axis.reset(0.5, 1.0);
  HomingController hc(ota::AxisId::Pitch, +1, test_params());

  // Drive a few steps, then assert a motor fault on the next feedback.
  int64_t t = 0;
  for (int i = 0; i < 50; ++i) {
    HomingFeedback fb = axis.feedback(t);
    if (i == 49) fb.motor_fault = true;  // fault on the last step
    DesiredState ds = hc.step(fb);
    axis.step(ds, t);
    t += kDtNs;
  }
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Failed);
  EXPECT_FALSE(hc.result().valid);
  SUCCEED();
}

TEST(Homing, HardAbortOnLargeEffortFails) {
  // The axis reaches a stop that produces a very large effort (above the hard
  // abort level of 9 N·m). The contact detector's hard abort must fire
  // immediately (no dwell), failing the homing instead of recording a contact.
  SimAxis axis;
  axis.stop_at = 0.2;         // reached quickly
  axis.contact_effort = 20.0; // well above effort_hard_abort_nm (9)
  axis.reset(0.0, 0.2);
  HomingController hc(ota::AxisId::Pitch, +1, test_params());

  run_homing(hc, axis, 2000);
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Failed) << hc.result().fail_reason;
  EXPECT_FALSE(hc.result().valid);
  EXPECT_NE(hc.result().fail_reason.find("hard abort"), std::string::npos);
  SUCCEED();
}

TEST(Homing, RunsAtFixedHomingCurrentNoRaise) {
  // Speed-mode homing runs at a FIXED homing current for the whole approach —
  // there is no adaptive current raise (the position-mode push-through hack
  // that made friction notches *hold* instead of slipping). A latched contact
  // at the homing current is the true mechanical end-stop, accepted directly.
  // The SimAxis models a hard mechanical stop (it stalls at the stop
  // regardless of current), so the homing must latch it once and complete,
  // with zero raises and the current held at the initial value.
  SimAxis axis;
  axis.stop_at = 1.0;
  // Above the contact threshold (3.0, strict >) and below the hard-abort (9).
  axis.contact_effort = 4.0;
  axis.reset(0.0, 1.0);
  HomingParams p = test_params();
  p.limit_cur_initial_a = 3.0;  // a low homing current (POC pitch value)
  p.limit_cur_step_a = 0.0;     // no adaptive raise (speed mode)
  p.limit_cur_max_a = 0.0;
  HomingController hc(ota::AxisId::Pitch, +1, p);

  run_homing(hc, axis, 6000);
  const ota::HomingResult& r = hc.result();
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Complete)
      << "failed: " << r.fail_reason;
  EXPECT_TRUE(r.valid);
  EXPECT_EQ(r.current_raises, 0) << "speed-mode homing never raises the current";
  EXPECT_DOUBLE_EQ(r.final_limit_cur_a, 3.0);
  EXPECT_NEAR(r.fine_contact_rad, 1.0, 0.01);
  SUCCEED();
}

TEST(Homing, RotationCapFailsWhenNoStopFound) {
  // A full-rotation axis with NO end-stop in this direction: the homing must
  // not run forever. It caps the cumulative rotation at max_rotation_rad
  // (360 deg) and fails. Use a fast coarse speed so the cap is reached well
  // inside the test window (180 deg/s -> 360 deg in ~2 s).
  SimAxis axis;
  axis.stop_at = 100.0;  // effectively no stop
  axis.reset(0.0, 100.0);
  ota::HomingParams p = test_params();
  p.coarse_speed_rad_s = 180.0 * kDeg2Rad;  // reach 360 deg in ~2 s
  HomingController hc(ota::AxisId::Yaw, +1, p);

  run_homing(hc, axis, 2000);
  const ota::HomingResult& r = hc.result();
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Failed) << r.fail_reason;
  EXPECT_FALSE(r.valid);
  EXPECT_NE(r.fail_reason.find("rotation cap"), std::string::npos)
      << "fail_reason was: " << r.fail_reason;
  SUCCEED();
}

TEST(Homing, TorqueSafetyAbortsBelowHardAbort) {
  // The end-stop push exceeds the (lowered) torque-safety threshold but stays
  // below the contact-detector hard-abort (9 N·m). The torque safety must fire
  // first, protecting the mechanical stop. Set torque_safety to 5 N·m and the
  // contact effort to 7 N·m (5 < 7 < 9).
  SimAxis axis;
  axis.stop_at = 0.2;
  axis.contact_effort = 7.0;  // above torque safety (5), below hard abort (9)
  axis.reset(0.0, 0.2);
  ota::HomingParams p = test_params();
  p.torque_safety_nm = 5.0;
  HomingController hc(ota::AxisId::Pitch, +1, p);

  run_homing(hc, axis, 2000);
  const ota::HomingResult& r = hc.result();
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Failed) << r.fail_reason;
  EXPECT_FALSE(r.valid);
  EXPECT_NE(r.fail_reason.find("torque safety"), std::string::npos)
      << "fail_reason was: " << r.fail_reason;
  SUCCEED();
}

TEST(Homing, RepeatabilityRetrySucceedsWhenStallBreaksOnRetry) {
  // p3f: the second fine approach stalls in a static-friction zone short of
  // the stop and the detector latches a false contact, so the (correct)
  // repeatability check rejects a non-repeatable q2. The breakaway is
  // stochastic: on the retry the zone gives way and the approach reaches the
  // stop, so the (backoff + second approach) pass succeeds and the homing
  // completes with exactly one retry recorded.
  SimAxis axis;
  axis.stop_at = 1.0;
  axis.has_zone = true;
  axis.stall_after_approaches = 2;  // stall the 3rd speed approach (2nd fine)
  axis.sticky_stalls_left = 1;      // the stall breaks on the retry
  axis.reset(0.5, 1.0);
  HomingController hc(ota::AxisId::Pitch, +1, test_params());

  run_homing(hc, axis, 6000);
  const ota::HomingResult& r = hc.result();
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Complete)
      << "failed: " << r.fail_reason;
  EXPECT_TRUE(r.valid);
  EXPECT_EQ(r.repeatability_retries, 1)
      << "the false contact should be retried exactly once";
  EXPECT_NEAR(r.fine_contact_rad, 1.0, 0.01);
  EXPECT_LE(r.repeatability_rad, test_params().repeatability_rad);
  SUCCEED();
}

TEST(Homing, RepeatabilityRetryExhaustsThenFails) {
  // p3f: a persistently non-repeatable second approach (the friction stall
  // never breaks) must fault AFTER the configured retries — the
  // repeatability check stays the safety authority, never waived.
  SimAxis axis;
  axis.stop_at = 1.0;
  axis.has_zone = true;
  axis.stall_after_approaches = 2;
  axis.sticky_stalls_left = 100;  // the stall never breaks
  axis.reset(0.5, 1.0);
  HomingParams p = test_params();
  p.repeatability_retries = 2;
  HomingController hc(ota::AxisId::Pitch, +1, p);

  run_homing(hc, axis, 6000);
  const ota::HomingResult& r = hc.result();
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Failed) << r.fail_reason;
  EXPECT_FALSE(r.valid);
  EXPECT_EQ(r.repeatability_retries, 2)
      << "must run all retries before faulting";
  EXPECT_NE(r.fail_reason.find("repeatability"), std::string::npos)
      << "fail_reason was: " << r.fail_reason;
  SUCCEED();
}
