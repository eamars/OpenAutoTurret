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
