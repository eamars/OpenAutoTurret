#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <limits>

namespace ota::axis {
// ABI uses SI output-shaft units. All measured/tunable numbers are supplied.
struct Model {
  int n = 5, periodic = 0;
  std::array<double, 8> q{};
  std::array<double, 3> z{};
  // a[3], b[3], h[direction=negative,positive][z=3][q=n], delay[s]
  std::array<double, 55> theta{};
};
bool valid(const Model& m);
bool coefficients(const Model& m, double q, double z, int direction,
                  double& a, double& b, double& h);

enum class Status : int {Ok=0, DataInvalid=1, MeasurementLimited=2, EnvelopeLimited=3,
                         OperatingPointChanged=4, HardAbort=5};
enum class Motion : int {Rest=0, Start=1, Move=2, Stop=3, Reverse=4};
struct ObserverConfig {
  double encoder_variance, gyro_variance, process_variance;
  double max_encoder_age_s, max_gyro_age_s, initial_position_variance, initial_velocity_variance;
  int encoder_only_verified;
};
struct Parameters {
  Model model;
  ObserverConfig observer;
  double kp, ki, kpos, kaw;
  double current_cap, slew, integral_cap, velocity_cap;
  double dt_min, dt_max, intent_threshold, rest_speed, sustained_s, start_timeout_s;
  std::array<double,48> start_total; // total sustained-motion currents, not additive boosts
  std::array<int,48> start_censored;
  double acceleration_cap, acceleration_noise_sigma, acceleration_sample_period_s;
  double acceleration_current_window_enabled; // 0: guidance/telemetry, 1: provisional model-current window
};
struct Observation {
  double now, encoder_time, gyro_time, position, gyro_rate;
  std::uint64_t encoder_seq, gyro_seq, generation;
  int encoder_valid, gyro_valid;
};
struct Reference {double position, velocity, acceleration, posture;};
// Created at the one FF insertion point, after this cycle's observe().
// The callback may inspect this snapshot; it does not own another observer.
struct PosteriorState {
  double now, dt, position, velocity, encoder_time, gyro_time, accepted_current, accepted_time;
  std::uint64_t encoder_seq, gyro_seq, generation;
  int encoder_only, motion, accepted_actual_time;
};
using FeedforwardCallback = int (*)(void*, const PosteriorState*, const Reference*, double*);
enum class ReferencePhase { Legacy=0, Departure=1, Braking=2 };
struct Output {
  double requested, limited, position, velocity, integral, feedforward, start_increment;
  std::uint64_t sequence;
  int status, motion, encoder_only;
  double requested_reference_velocity, shaped_reference_velocity;
  double requested_reference_acceleration, shaped_reference_acceleration;
  double measured_acceleration, acceleration_sample_time, acceleration_interval_s;
  double acceleration_noise_sigma, acceleration_feedback_horizon_s, delayed_applied_current;
  double acceleration_current_min, acceleration_current_max, acceleration_limited_request;
  int acceleration_fresh, acceleration_valid, acceleration_limit_reason, current_history_actual_time;
};
bool valid(const Parameters& p);
class Controller {
 public:
  bool configure(const Parameters& p);
  bool reset(double time, double position, double velocity, double previous_current,
             std::uint64_t generation, double previous_current_time=std::numeric_limits<double>::quiet_NaN());
  Output step(const Observation& observation, const Reference& reference);
  // Additive offline interface: one supplied host-command FF term replaces the
  // legacy model FF, with the same observer, PI, transitions, limits and ACKs.
  Output step_with_feedforward(const Observation&, const Reference&, double command_feedforward);
  // Offline additive interface. A rejected/nonfinite callback latches inhibit
  // before any command token; the existing PI/limiter/ACK path stays sole owner.
  Output step_with_posterior_feedforward(const Observation&, const Reference&, FeedforwardCallback, void*,
                                         int planned_start_intent=0);
  Output step_with_posterior_phase(const Observation&, const Reference&, FeedforwardCallback, void*,
                                   ReferencePhase, int planned_start_intent);
  void inhibit();
  bool acknowledge(std::uint64_t sequence, bool successful, double applied);
  bool acknowledge_at(std::uint64_t sequence, bool successful, double applied, double accepted_time);
  bool switch_parameters(const Parameters& p, const Reference& reference);
  // C ABI destruction during a callback waits until the outer call returns.
  bool defer_destruction();
  bool destruction_ready() const {return destruction_requested_ && !evaluating_feedforward_;}
  const Parameters& parameters() const { return p_; }
 private:
  Output step_impl(const Observation&, const Reference&, const double* command_feedforward,
                   FeedforwardCallback callback=nullptr, void* callback_context=nullptr,
                   int planned_start_intent=0, ReferencePhase phase=ReferencePhase::Legacy);
  bool observe(const Observation& o);
  bool acknowledge_current(std::uint64_t sequence, bool successful, double applied,
                           double accepted_time, bool actual_time);
  bool interval_current(double begin, double end, double& mean, bool& actual_time) const;
  bool configured_=false, ready_=false, pending_=false, initialize_integral_=false;
  bool hard_abort_=false; // only an explicit successful reset clears a startup fault
  bool evaluating_feedforward_=false; // callback cannot re-enter or reset its command owner
  bool destruction_requested_=false;
  bool transient_output_=false;
  Parameters p_{};
  double time_=0,q_=0,v_=0,p00_=0,p01_=0,p11_=0;
  double encoder_time_=0,gyro_time_=0,integral_=0,error_previous_=0,last_applied_=0;
  double dt_=0,error_=0,requested_=0,limited_=0,base_requested_=0,start_time_=0,sustained_since_=-1;
  double quiet_initial_offset_=0;
  double encoder_position_=0,gyro_travel_=0,last_gyro_rate_=0;
  double motion_check_time_=0,motion_check_position_=0,motion_check_gyro_=0;
  std::array<double,2> running_integral_{};
  // Command tokens never restart with sensor generation: a delayed old ACK
  // must not acknowledge a new command after reset.
  std::uint64_t generation_=0,encoder_seq_=0,gyro_seq_=0,sequence_=0;
  Motion motion_=Motion::Rest;
  int direction_=1;
  struct CurrentEvent {double time,current; bool actual_time;};
  std::deque<CurrentEvent> current_history_;
  double shaped_velocity_=0,measured_acceleration_=0,acceleration_time_=0,acceleration_interval_=0;
  double delayed_applied_current_=0;
  bool acceleration_fresh_=false,acceleration_valid_=false,current_history_actual_time_=false;
};
}

extern "C" {
// A deterministic local model evaluator; no transport, configuration or file IO.
int ota_model_rollout(const ota::axis::Model* model, int count, const double* time,
                      const double* successful_tx, const double* posture,
                      const int* direction, double q0, double v0, double* qv);
// State begins at time[0]; separate accepted-event history supplies u(t-delay).
// Status 3 means no actual command covers time[0]-delay.
int ota_model_rollout_with_history(const ota::axis::Model* model, int count,
                      const double* time, const double* posture, const int* direction,
                      int tx_count, const double* tx_time, const double* successful_tx,
                      double q0, double v0, double* qv);
int ota_core_abi();
void* ota_controller_create(const ota::axis::Parameters* parameters);
void ota_controller_destroy(void* handle);
int ota_controller_reset(void* handle, double time, double position, double velocity,
                         double previous_current, std::uint64_t generation);
int ota_controller_reset_at(void* handle, double time, double position, double velocity,
                            double previous_current, std::uint64_t generation,
                            double previous_current_accepted_time);
int ota_controller_step(void* handle, const ota::axis::Observation*,
                        const ota::axis::Reference*, ota::axis::Output*);
int ota_controller_step_ff(void* handle, const ota::axis::Observation*,
                          const ota::axis::Reference*, double command_feedforward, ota::axis::Output*);
int ota_controller_step_posterior_ff(void* handle, const ota::axis::Observation*,
                          const ota::axis::Reference*, ota::axis::FeedforwardCallback,
                          void* callback_context, ota::axis::Output*);
// Synthetic opt-in departure intent; zero preserves the legacy reference intent.
// The coherent reference adapter validates its phase/anchor before this call.
int ota_controller_step_posterior_ff_intent(void* handle, const ota::axis::Observation*,
                          const ota::axis::Reference*, int planned_start_intent,
                          ota::axis::FeedforwardCallback, void* callback_context, ota::axis::Output*);
int ota_controller_step_posterior_ff_phase(void* handle, const ota::axis::Observation*,
                          const ota::axis::Reference*, int reference_phase, int planned_start_intent,
                          ota::axis::FeedforwardCallback, void* callback_context, ota::axis::Output*);
int ota_controller_inhibit(void* handle);
int ota_controller_parameters(void* handle, ota::axis::Parameters*);
int ota_controller_ack(void* handle, std::uint64_t sequence, int successful, double applied);
int ota_controller_ack_at(void* handle, std::uint64_t sequence, int successful, double applied,
                          double accepted_time);
int ota_controller_switch(void* handle, const ota::axis::Parameters*, const ota::axis::Reference*);
// Offline simulation settings are explicit injected mathematical values.
struct OtaSimulation {
  double dt, encoder_quantum, encoder_noise, gyro_noise, measurement_delay, gyro_filter_tau;
  int encoder_period, gyro_period;
  std::uint64_t seed;
};
// trace columns: true q/v, observed q/v, requested/limited, integral, ff,
// start increment, motion, status, successful applied current.
int ota_closed_rollout(const ota::axis::Parameters* controller,
    const ota::axis::Parameters* plant, const OtaSimulation*, int count,
    const ota::axis::Reference* references, double q0, double v0, double* trace);
}
