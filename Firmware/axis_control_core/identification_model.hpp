#pragma once

// Offline fixed-pitch yaw identification. This API does not produce controller gains.
// SI shaft coordinates and A-equivalent mechanical parameters; no physical defaults.
struct OtaIdentificationModel {
  int actuator; // 0 algebraic, 1 first-order regulated effective current
  int friction; // 0 hybrid Coulomb, 1 hybrid Stribeck
  int load; // 0 constant, 1 signed local affine spatial load
  double a, viscous;
  double coulomb_negative, coulomb_positive, static_negative, static_positive;
  double stribeck_negative, stribeck_positive, stribeck_power;
  double load_offset, load_slope, q_origin;
  double actuator_gain, actuator_bias, actuator_tau, transport_delay;
  double gyro_bias, gyro_tau, gyro_delay;
  double current_gain, current_bias, current_tau, current_delay;
  double q_min, q_max, max_step;
};

extern "C" {
// State is initialized once at time[0]. It is never replaced by future observations.
// initial: q, v, effective current, unbiased gyro filter, unscaled current filter.
// trace: q, v, effective current, observed gyro, observed current, stick (0/1).
// Status 1 invalid contract, 2 predicted state outside declared q domain,
// 3 successful-TX prehistory does not cover time[0]-transport_delay (and
// current_delay when unfiltered algebraic current is observed).
int ota_identification_rollout(const OtaIdentificationModel*, int count, const double* time,
    int tx_count, const double* accepted_time, const double* accepted_A,
    const double* initial, double* trace);
}
