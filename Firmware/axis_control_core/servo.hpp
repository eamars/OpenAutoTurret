#pragma once
#include <cstdint>
#include <deque>
#include <utility>

namespace ota::axis {
// Position servo for one current-commanded axis. SI output-shaft units; currents
// are motor-command amperes. One reference sample {q, v, a} per step: the caller
// owns trajectory shaping. Feedforward uses the reference only (never the noisy
// measured velocity), so friction compensation cannot chatter at rest.
struct ServoParameters {
  // Encoder/gyro velocity observer (constant-velocity Kalman filter).
  double encoder_variance, gyro_variance, process_variance;
  double max_encoder_age_s, max_gyro_age_s;
  int use_gyro;
  // Plant feedforward: inertia * a_ref + friction(v_ref) + load.
  double inertia;                                // A*s^2/rad
  double coulomb_positive, coulomb_negative;     // A, magnitudes
  double stribeck_positive, stribeck_negative;   // A, extra low-speed magnitude
  double stribeck_speed;                         // rad/s
  double viscous;                                // A*s/rad
  double creep_drop, creep_speed;                // A, rad/s: below ~creep_speed the level falls by up to creep_drop
  double friction_band;                          // rad/s: v_ref below this scales friction FF linearly
  double load;                                   // A, constant (cable/gravity at the operating pose)
  double friction_correction_rate;               // 1/s: friction FF follows v_ref + rate*error (0: v_ref only)
  double friction_correction_deadband;           // rad: error inside it adds no correction (no hunting at rest)
  double dither_amplitude, dither_period_s;      // A, s: square-wave dither keeps stiction from locking (0: off)
  // Learned friction map: kFrictionBins equal angle bins per direction, added to
  // the Coulomb terms. While tracking steadily, the integral is moved into the
  // bin under the axis at rate friction_learning_rate (1/s); 0 disables learning.
  static constexpr int kFrictionBins=24;
  double friction_map_positive[kFrictionBins], friction_map_negative[kFrictionBins];
  double friction_learning_rate, friction_learning_speed, friction_map_limit;  // 1/s, rad/s, A
  // Encoder current crosstalk: the GM6020 angle reading moves with phase current,
  // q_measured = q + g(angle)*i(t-delay). g is a periodic table over absolute
  // angle (rad/A), measured on the station by a probe-tone scan; zeros disable it.
  static constexpr int kCrosstalkBins=120;
  double crosstalk_delay_s;
  double crosstalk_map[kCrosstalkBins];
  // PID on position error.
  double kq, kv, ki;                             // A/rad, A*s/rad, A/(rad*s)
  double integral_cap, error_clamp;              // A, rad
  double hold_band, hold_speed;                  // rad, rad/s: hold when |error|<band and |v_ref|<speed (0 band: never)
  double hold_relax_tau_s;                       // s: inside the hold band the integral decays with this time constant (0: freeze)
  // Stall recovery: pushing hard (|requested|>stall_current) against an error
  // beyond stall_error with no motion (|v|<stall_speed) for stall_time_s triggers
  // a rock_s pulse of rock_current in the opposite direction, then the integral
  // restarts from zero so the loop re-applies force fast. 0 stall_time_s disables.
  // With stall_reference_speed > 0 it acts only while |v_ref| is below it: a stall
  // at a target, not a moving reference whose own reversal will free the bearing.
  double stall_error, stall_speed, stall_current, stall_time_s, rock_current, rock_s;
  double stall_reference_speed;                  // rad/s (0: any reference)
  // Output limits and supervision.
  double current_cap, slew;                      // A, A/s: peak authority and its rate
  double rms_limit, rms_tau_s;                   // A, s: thermal budget; above it the cap falls to rms_limit
  double dt_min, dt_max;                         // s
  double following_error_limit;                  // rad: beyond it the servo reports a fault
};

enum class ServoStatus : int { Ok=0, DataInvalid=1, FollowingError=2, NotReady=3 };

struct ServoOutput {
  double requested, limited, position, velocity, error, velocity_error;
  double feedforward, friction, proportional, derivative, integral, rms, cap;
  int status, saturated, rocking;
  long stall_events;
};

class Servo {
 public:
  bool configure(const ServoParameters& p);
  bool reset(double time, double position, double velocity, double previous_current);
  // Each measurement carries its own sample time; call for every frame received.
  bool observe_encoder(double sample_time, double position);
  bool observe_gyro(double sample_time, double rate);
  ServoOutput step(double now, double q_ref, double v_ref, double a_ref);
  // The current actually transmitted for the last step (or failure).
  void acknowledge(bool success, double applied);
  const ServoParameters& parameters() const { return p_; }
  bool ready() const { return ready_; }
  // Commissioning: change feedback gains between steps (integral kept).
  bool set_gains(double kq,double kv,double ki);
  long stale_samples() const { return stale_samples_; }
  double friction(double v_ref) const;
  double friction_at(double v_ref, double q) const;
  double crosstalk_gain(double q) const;
  const ServoParameters& learned() const { return p_; }
 private:
  void predict(double to);
  bool update(double value, double hq, double hv, double variance);
  ServoParameters p_{};
  bool configured_=false, ready_=false;
  double t_=0, q_=0, v_=0, p00_=0, p01_=0, p11_=0;
  double encoder_time_=0, gyro_time_=0, step_time_=0;
  // last_applied_: the total current on the wire (crosstalk history, thermal RMS);
  // last_output_: the servo's own output, which the slew limit follows (an added
  // identification excitation must not drag it).
  double integral_=0, last_applied_=0, last_output_=0, mean_square_=0;
  int last_saturated_=0;
  long stale_samples_=0, stall_events_=0;
  double stall_since_=-1, stall_position_=0, rock_until_=-1, rock_direction_=0;
  std::deque<std::pair<double,double>> applied_history_;  // (time, applied current)
};
bool valid(const ServoParameters& p);
}

extern "C" {
void* ota_servo_create(const ota::axis::ServoParameters*);
void ota_servo_destroy(void*);
int ota_servo_reset(void*, double time, double position, double velocity, double previous_current);
int ota_servo_observe_encoder(void*, double sample_time, double position);
int ota_servo_observe_gyro(void*, double sample_time, double rate);
int ota_servo_step(void*, double now, double q_ref, double v_ref, double a_ref, ota::axis::ServoOutput*);
int ota_servo_acknowledge(void*, int success, double applied);
}
