#pragma once

namespace ota::axis {
// Host position loop around a drive's own speed loop (pitch CyberGear, RunMode 2):
// speed = v_ref + kp*(q_ref-q) + integral, clamped to speed_limit. The integral
// does not wind further into the clamp. Shared by commissiond and the simulator.
struct PositionLoopParameters {
  double kp;              // 1/s
  double ki;              // 1/s^2
  double integral_clamp;  // rad/s
  double speed_limit;     // rad/s
};

class PositionLoop {
 public:
  bool configure(const PositionLoopParameters& p);
  void reset() { integral_=0.; }
  // dt: seconds since the previous step (0 on the first).
  double step(double dt,double q_ref,double v_ref,double q);
  double integral() const { return integral_; }
  void set_gains(double kp,double ki) { p_.kp=kp; p_.ki=ki; }
  const PositionLoopParameters& parameters() const { return p_; }
 private:
  PositionLoopParameters p_{};
  double integral_=0.;
};
}
