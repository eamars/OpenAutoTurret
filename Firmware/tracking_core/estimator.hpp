#pragma once
#include <array>
#include <cstdint>

namespace ota::track {
// ADR-003 sec. 3-4: the selected subject's line of sight (LOS) in the fixed base frame,
// one constant-velocity Kalman filter per axis (0 azimuth, 1 elevation), x = [theta, omega].
//
// * Process noise is continuous white angular acceleration with spectral density q_a
//   (rad^2/s^3): Q(dt) = q_a [[dt^3/3, dt^2/2], [dt^2/2, dt]], so irregular frame intervals
//   propagate consistently. (This is not the old angular_accel_sigma, which had other units.)
// * One predict/update per accepted observation, Joseph form. Azimuth innovations are
//   wrapped; the state keeps a continuous branch.
// * Manoeuvres: when the normalised innovation squared (NIS, both axes) exceeds the fixed
//   99% gate, the process noise is scaled once by min(scale_max, NIS/gate), recomputed from
//   the pre-update state, and decays back to 1 with time constant scale_tau_s.
// * Robust update: the whitened innovation norm r gives w = min(1, sqrt(gate)/r) and the
//   update uses R/w (covariance included), so a real stop or reversal is followed instead of
//   rejected forever. Only invalid numbers, time or geometry are rejected outright.
// * |omega| beyond the validated rate domain (a guard against nonsense, set well above what the
//   turret can follow): the rate is held at the domain edge (rate_limited counts it).
// * Queries never advance the committed state. The query applies the velocity-confidence
//   weight w_noise and the coast fade g(age), and returns one motion hypothesis:
//   theta_goal = theta + v_ff * integral_0^age g, omega_goal = v_ff * g(age).
// There is no acceleration state: acceleration_valid is always false (ADR-003 D06).
struct EstimatorParameters {
  std::array<double,2> process_density{};    // q_a, rad^2/s^3
  std::array<double,2> measurement_floor{};  // smallest R, rad^2
  double initial_rate_sigma=0;               // rad/s at (re)initialisation
  std::array<double,2> scale_max{};          // max(1, A^2*T_m/q_a)
  double scale_tau_s=0;                      // two median frame intervals
  double rate_domain=0;                      // rad/s, validated LOS rate
  double fresh_s=0, horizon_s=0;             // coast: g=1 to fresh_s, cubic fade to horizon_s, invalid after
  double position_sigma_limit=0;             // rad: goal invalid when the propagated sigma exceeds it (0: off)
};
bool valid(const EstimatorParameters& p);

constexpr double kNisGate=9.21034;  // 2-D chi-square 99%: a fixed statistical design value

struct LosObservation {
  int64_t t_ns=0;            // optical observation time on the control clock
  double az=0, el=0;         // base-frame LOS, rad
  double var_az=0, var_el=0; // rad^2 (the observation's own variance; floored by measurement_floor)
};

struct AxisState { double theta=0, omega=0, pp=0, pv=0, vv=0; };

struct EstimatorDiagnostics {
  double nis=0, weight=1;
  std::array<double,2> scale{1.,1.}, innovation{};
  uint64_t accepted=0, rejected=0, downweighted=0, rate_limited=0, gap_resets=0;
  bool last_accepted=false;
};

struct LosGoal {
  bool position_valid=false, velocity_valid=false;
  bool acceleration_valid=false;               // always false: no acceleration state (D06)
  std::array<double,2> theta{}, omega{};       // the goal LOS and its rate, FF weight and fade applied
  std::array<double,2> rate{}, rate_sigma{};   // the raw estimate, for telemetry
  std::array<double,2> ff_weight{};            // w_noise per axis
  double fade=0, age_s=0;                      // g(age) and age = t_eval - t_o
  int64_t state_ns=0;
};

// Coast fade g(age) and its integral from 0 (both analytic, finite for any age).
double coast_fade(double age,double fresh,double horizon);
double coast_integral(double age,double fresh,double horizon);
// Velocity-confidence weight: r=|omega|/sigma, s=clamp((r-1)/2,0,1), w=3s^2-2s^3.
double velocity_weight(double omega,double sigma);

class TargetEstimator {
 public:
  bool configure(const EstimatorParameters& p);
  void reset();
  bool initialized() const { return initialized_; }
  bool update(const LosObservation& z);
  // use_motion=false is the FB-only comparison (ADR-003 sec. 7A): no velocity FF and no
  // extrapolation of the position by the estimated motion.
  LosGoal query(int64_t t_eval_ns,bool use_motion=true) const;
  const std::array<AxisState,2>& state() const { return x_; }
  int64_t state_ns() const { return t_ns_; }
  const EstimatorDiagnostics& diagnostics() const { return d_; }
  const EstimatorParameters& parameters() const { return p_; }
 private:
  void initialise(const LosObservation& z,const double r[2]);
  EstimatorParameters p_{};
  bool configured_=false, initialized_=false;
  std::array<AxisState,2> x_{};
  int64_t t_ns_=0;
  EstimatorDiagnostics d_{};
};
}
