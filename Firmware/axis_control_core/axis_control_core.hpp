#pragma once
#include <array>
#include <cstdint>

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
};
struct Observation {
  double now, encoder_time, gyro_time, position, gyro_rate;
  std::uint64_t encoder_seq, gyro_seq, generation;
  int encoder_valid, gyro_valid;
};
struct Reference {double position, velocity, acceleration, posture;};
struct Output {
  double requested, limited, position, velocity, integral, feedforward, start_increment;
  std::uint64_t sequence;
  int status, motion, encoder_only;
};
bool valid(const Parameters& p);
class Controller {
 public:
  bool configure(const Parameters& p);
  bool reset(double time, double position, double velocity, double previous_current,
             std::uint64_t generation);
  Output step(const Observation& observation, const Reference& reference);
  bool acknowledge(std::uint64_t sequence, bool successful, double applied);
  bool switch_parameters(const Parameters& p, const Reference& reference);
 private:
  bool observe(const Observation& o);
  bool configured_=false, ready_=false, pending_=false, initialize_integral_=false;
  Parameters p_{};
  double time_=0,q_=0,v_=0,p00_=0,p01_=0,p11_=0;
  double encoder_time_=0,gyro_time_=0,integral_=0,error_previous_=0,last_applied_=0;
  double dt_=0,error_=0,requested_=0,limited_=0,start_time_=0,sustained_since_=-1;
  std::uint64_t generation_=0,encoder_seq_=0,gyro_seq_=0,sequence_=0;
  Motion motion_=Motion::Rest;
  int direction_=1;
};
}

extern "C" {
// A deterministic local model evaluator; no transport, configuration or file IO.
int ota_model_rollout(const ota::axis::Model* model, int count, const double* time,
                      const double* successful_tx, const double* posture,
                      const int* direction, double q0, double v0, double* qv);
int ota_core_abi();
void* ota_controller_create(const ota::axis::Parameters* parameters);
void ota_controller_destroy(void* handle);
int ota_controller_reset(void* handle, double time, double position, double velocity,
                         double previous_current, std::uint64_t generation);
int ota_controller_step(void* handle, const ota::axis::Observation*,
                        const ota::axis::Reference*, ota::axis::Output*);
int ota_controller_ack(void* handle, std::uint64_t sequence, int successful, double applied);
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
