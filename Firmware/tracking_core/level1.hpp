#pragma once
#include <array>
#include <limits>
#include <cstdint>

namespace ota::track {
// ADR-003 sec. 5: Level 1, the only owner of the joint reference q/v/a handed to the
// ADR-002.2 servos. Per joint axis (0 yaw, 1 pitch):
//
//   a_request = lambda^2 (q_t - q_r) + 2 lambda (v_t - v_r)
//
// the target-motion feedforward (v_t, from the estimator through the joint Jacobian) plus
// reference-error feedback. It is then constrained in this order: the lead limit (the
// reference may not run more than lead_limit ahead of the measured axis), the travel
// boundary (the target is clamped into it and the reference brakes in time), the speed
// ceiling with the a^2/2j release reserve, the acceleration bound and the jerk bound. The
// result is one constant-jerk segment per control tick:
//   q(tau) = q + v tau + a tau^2/2 + j tau^3/6, v(tau) = v + a tau + j tau^2/2, a(tau) = a + j tau
// The servos evaluate that polynomial at their own rate between ticks; the next tick
// integrates the same polynomial to its actual time, so q/v/a are one integral, never three
// separately shaped signals. There is no target-acceleration term (none is estimated).
struct Level1Axis {
  double lambda=0;            // rad/s
  double v_max=0, a_max=0, j_max=0;
  double lead_limit=0;        // rad: largest reference-to-axis lead before advance stops
  double q_min=0, q_max=0;    // travel (q_min==q_max: unbounded, e.g. continuous yaw)
  // Directional speed caps set each tick by the host (>=0; infinite = none): the speed toward
  // each end from which a stop under the host's own safety model still fits.
  double v_pos_cap=std::numeric_limits<double>::infinity(), v_neg_cap=std::numeric_limits<double>::infinity();
};
struct Level1Parameters {
  std::array<Level1Axis,2> axis{};
  double period_s=0.005;      // nominal control period (jerk projection horizon)
  double valid_s=0.02;        // a sample older than this is stale for the servo
};
bool valid(const Level1Parameters& p);

enum Level1Flag : uint32_t {
  kVelocityLimited=1u<<0, kAccelerationLimited=1u<<1, kJerkLimited=1u<<2, kBoundary=1u<<3,
  kLeadLimited=1u<<4, kGoalInvalid=1u<<5, kVelocityInvalid=1u<<6,
};

struct JointGoal {
  bool valid=false;                       // false: hold (brake to rest from the current reference)
  std::array<double,2> q{}, v{};          // unconstrained joint target and its rate
  std::array<bool,2> velocity_valid{};    // false: v treated as 0
};

struct ReferenceSample {
  int64_t t_ns=0;
  std::array<double,2> q{}, v{}, a{}, j{};
  std::array<uint32_t,2> flags{};
  std::array<double,2> a_request{};       // the unconstrained law, for telemetry
  std::array<double,2> a_max{}, v_max{};  // the bounds the segment is evaluated under
  bool valid=false;
  double valid_s=0;
  // Evaluate axis i at time t (t >= t_ns); false when stale.
  bool at(int i,int64_t t,double& q_out,double& v_out,double& a_out) const;
};

class Level1Generator {
 public:
  bool configure(const Level1Parameters& p);
  bool configured() const { return configured_; }
  // Take over from the actual axis state (source change, re-engagement). Never per cycle.
  void reset(int64_t t_ns,const std::array<double,2>& q,const std::array<double,2>& v,const std::array<double,2>& a);
  bool initialized() const { return initialized_; }
  ReferenceSample step(int64_t t_ns,const JointGoal& goal,const std::array<double,2>& q_measured);
  const ReferenceSample& last() const { return last_; }
  const Level1Parameters& parameters() const { return p_; }
  void set_lambda(int axis,double lambda) { p_.axis[axis].lambda=lambda; }
  // The host's envelope this tick (speed, acceleration, jerk and travel already reduced for
  // derating and boundaries): Level 1 never asks for more than the existing limits allow.
  bool set_limits(int axis,double v_max,double a_max,double j_max,double q_min,double q_max);
  // Speed allowed toward q_min (negative_speed) and toward q_max (positive_speed), >=0.
  bool set_speed_bounds(int axis,double negative_speed,double positive_speed);
 private:
  Level1Parameters p_{};
  bool configured_=false, initialized_=false;
  std::array<bool,2> lead_hold_{};
  ReferenceSample last_{};
};

// Exact integration of one axis' constant-jerk segment over tau, splitting at the
// acceleration bound and the speed ceiling (the segment's events), so that a late tick
// can neither exceed a_max nor v_max.
void integrate(double& q,double& v,double& a,double j,double tau,double a_max,double v_max);
}
