// Unit tests for the sensorless contact detector (architecture §21).
#include <gtest/gtest.h>

#include <cmath>

#include "calibration/contact_detector.hpp"

namespace {

using ota::ContactDetector;
using ota::ContactDetectorParams;

constexpr int64_t kDtNs = 5'000'000;  // 200 Hz sample interval

// A simple synthetic axis: moves at a fixed velocity until it reaches a stop,
// then stalls (velocity ~ noise, position fixed, effort rises). State is held
// in the member `q`; step() advances it and reports v/torque.
struct SimAxis {
  double stop_at = 1.0;        // position (rad) where the stop is
  double v_cmd = 0.5;          // commanded approach velocity (rad/s)
  double effort_drive = 1.0;   // N·m while driving
  double effort_contact = 4.0; // N·m while pushing into the stop
  double noise = 0.05;         // rad/s velocity noise amplitude
  double q = 0.0;
  bool at_stop = false;

  void step(int64_t t_ns, double& v, double& torque) {
    if (!at_stop) {
      q += v_cmd * (kDtNs / 1e9);
      if (q >= stop_at) {
        q = stop_at;
        at_stop = true;
      }
      v = v_cmd + std::sin(t_ns / 1e5) * noise;  // driving + small noise
      torque = effort_drive;
    } else {
      q = stop_at;
      v = std::sin(t_ns / 3e4) * noise;  // noise around 0
      torque = effort_contact;
    }
  }
};

}  // namespace

TEST(ContactDetector, NoContactWhileMoving) {
  ContactDetector det(ContactDetectorParams{});
  det.set_approach_direction(+1);
  SimAxis sim;
  sim.stop_at = 100.0;  // far away — never reached in the window
  int64_t t = 0;
  bool saw_contact = false;
  for (int i = 0; i < 400; ++i) {
    double v, torque;
    sim.step(t, v, torque);
    auto r = det.update(t, sim.q, v, torque, false);
    if (r.contact) saw_contact = true;
    t += kDtNs;
  }
  EXPECT_FALSE(saw_contact);
}

TEST(ContactDetector, DeclaresContactAtStopAfterDwell) {
  ContactDetector det(ContactDetectorParams{});
  det.set_approach_direction(+1);
  SimAxis sim;
  sim.stop_at = 0.5;  // stop reached after ~1 s
  int64_t t = 0;
  int64_t contact_t = -1;
  int64_t stop_t = -1;
  for (int i = 0; i < 800; ++i) {
    double v, torque;
    sim.step(t, v, torque);
    if (sim.at_stop && stop_t < 0) stop_t = t;
    auto r = det.update(t, sim.q, v, torque, false);
    if (r.contact && contact_t < 0) contact_t = t;
    if (contact_t >= 0) break;
    t += kDtNs;
  }
  EXPECT_GE(contact_t, 0) << "contact was never declared";
  // Contact must come after the stop AND after the dwell.
  const int64_t dwell_ns = 200 * 1'000'000;
  EXPECT_GE(contact_t - stop_t, dwell_ns - 2 * kDtNs);
}

TEST(ContactDetector, HardAbortOnLargeEffort) {
  ContactDetectorParams p;
  p.contact_dwell_ms = 500;  // long dwell, so a hard abort must be immediate
  ContactDetector det(p);
  det.set_approach_direction(+1);
  int64_t t = 0;
  bool abort = false;
  for (int i = 0; i < 100; ++i) {
    double q = 0.1;
    double v = 0.3;
    double torque = 10.0;  // well above effort_hard_abort_nm (9)
    auto r = det.update(t, q, v, torque, false);
    if (r.hard_abort) {
      abort = true;
      break;
    }
    t += kDtNs;
  }
  EXPECT_TRUE(abort);
}

TEST(ContactDetector, HardAbortOnFault) {
  ContactDetector det(ContactDetectorParams{});
  det.set_approach_direction(+1);
  int64_t t = 0;
  double q = 0.1, v = 0.1, torque = 0.5;
  auto r = det.update(t, q, v, torque, true);  // motor_fault = true
  EXPECT_TRUE(r.hard_abort);
}

TEST(ContactDetector, NoContactWithoutEffort) {
  // Stalled (low v, no progress) but low effort (a soft coast-to-stop) must NOT
  // be declared as contact — the effort condition must also hold.
  ContactDetector det(ContactDetectorParams{});
  det.set_approach_direction(+1);
  int64_t t = 0;
  bool saw_contact = false;
  const double q = 0.5;
  for (int i = 0; i < 600; ++i) {
    double v = std::sin(t / 3e4) * 0.05;  // noise around 0
    double torque = 0.3;                  // below effort_contact_threshold (3)
    auto r = det.update(t, q, v, torque, false);
    if (r.contact) saw_contact = true;
    t += kDtNs;
  }
  EXPECT_FALSE(saw_contact);
}

// --- Jitter (stick-slip) detection tests (C1, drive_current_friction_tuning.md)
// ---

namespace {
// Drive the detector along a scripted velocity/torque profile, keeping q
// consistent with v (q += v*dt) so the position-derived velocity is correct.
struct JitterSim {
  double q = 0.0;
  int64_t t = 0;
  double v_cmd = 0.5;
  const double dt_s = kDtNs / 1e9;

  // Ramp from rest to v_cmd over `n` samples (the start-up breakaway).
  void startup(ota::ContactDetector& det, int n) {
    for (int i = 0; i < n; ++i) {
      const double v = v_cmd * (i + 1) / n;
      q += v * dt_s;
      det.update(t, q, v, 1.0, false);
      t += kDtNs;
    }
  }
  // Drive at v_cmd for n samples.
  void drive(ota::ContactDetector& det, int n) {
    for (int i = 0; i < n; ++i) {
      q += v_cmd * dt_s;
      det.update(t, q, v_cmd, 1.0, false);
      t += kDtNs;
    }
  }
  // Stall (v ~ 0, high effort) for n samples; q held.
  void stall(ota::ContactDetector& det, int n, double torque) {
    for (int i = 0; i < n; ++i) {
      det.update(t, q, 0.0, torque, false);
      t += kDtNs;
    }
  }
  // Recover: ramp v from 0 back to v_cmd over n samples.
  void recover(ota::ContactDetector& det, int n) {
    for (int i = 0; i < n; ++i) {
      const double v = v_cmd * (i + 1) / n;
      q += v * dt_s;
      det.update(t, q, v, 1.0, false);
      t += kDtNs;
    }
  }
};
}  // namespace

// A clean approach from rest to a stop must NOT count the start-up breakaway
// (rest -> motion) as a recovery, and must declare contact at the stop.
TEST(ContactDetector, CleanApproachStartupNotARecovery) {
  ota::ContactDetector det(ota::ContactDetectorParams{});
  det.set_approach_direction(+1);
  JitterSim s;
  s.startup(det, 8);   // rest -> v_cmd (NOT a recovery)
  s.drive(det, 200);   // approach
  bool saw_contact = false;
  int64_t contact_t = -1;
  int64_t stop_t = -1;
  s.drive(det, 1);     // one more approach sample
  // Now hold at the stop (v~0, high effort, persistent) -> a real stop.
  for (int i = 0; i < 400; ++i) {
    auto r = det.update(s.t, s.q, 0.0, 4.0, false);
    if (r.contact && contact_t < 0) {
      contact_t = s.t;
      saw_contact = true;
    }
    s.t += kDtNs;
  }
  EXPECT_TRUE(saw_contact) << "contact was never declared at the real stop";
  auto r = det.update(s.t, s.q, 0.0, 4.0, false);
  EXPECT_EQ(r.total_stall_recoveries, 0)
      << "the start-up breakaway must not be counted as a recovery";
  EXPECT_FALSE(r.recovered_in_window);
}

// Stick-slip: repeated stall->recovery cycles mid-approach must each be counted
// as a recovery, must NOT latch a (false) contact, and the real stop (a
// permanent stall with no recovery) must still be detected.
TEST(ContactDetector, StickSlipRecoveriesCountedNoFalseContact) {
  ota::ContactDetector det(ota::ContactDetectorParams{});
  det.set_approach_direction(+1);
  JitterSim s;
  s.startup(det, 8);
  s.drive(det, 100);
  bool contact_during_stickslip = false;
  // Three stick-slip cycles: drive, stall (100 ms, high effort), recover.
  for (int c = 0; c < 3; ++c) {
    s.drive(det, 40);
    for (int i = 0; i < 20; ++i) {  // 100 ms stall
      auto r = det.update(s.t, s.q, 0.0, 5.0, false);
      if (r.contact) contact_during_stickslip = true;
      s.t += kDtNs;
    }
    s.recover(det, 8);
    s.drive(det, 40);
  }
  EXPECT_FALSE(contact_during_stickslip)
      << "a breakaway stall must not latch a contact";
  // Real stop: hold at the stop (v~0, high effort) -> contact.
  bool saw_contact = false;
  for (int i = 0; i < 400; ++i) {
    auto r = det.update(s.t, s.q, 0.0, 4.0, false);
    if (r.contact) saw_contact = true;
    s.t += kDtNs;
  }
  EXPECT_TRUE(saw_contact) << "the real stop must still be detected";
  auto r = det.update(s.t, s.q, 0.0, 4.0, false);
  EXPECT_EQ(r.total_stall_recoveries, 3)
      << "each stick-slip cycle is one recovery; the start-up is not counted";
  EXPECT_TRUE(r.max_accel_since_reset > 0.0)
      << "the slip acceleration peaks must be recorded";
}
