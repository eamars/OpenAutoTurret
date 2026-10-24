#pragma once
#include <deque>
#include <random>
#include <utility>
#include <yaml-cpp/yaml.h>

namespace ota::axis {
// Plant models identified by tools/servo_commission (config/servo/*.json "plant").
// They exist to design and check controllers before the station runs them; what
// they leave out is listed in docs/operations/servo-commissioning.md.

// GM6020 yaw in CAN current mode: rigid inertia, LuGre friction (an elastic
// pre-sliding contact whose steady sliding level falls with speed, plus an angle
// map), a delayed first-order current response, and an 8192-count encoder whose
// reading moves with the commanded current.
struct YawPlantParameters {
  static constexpr int kFrictionBins=24, kCrosstalkBins=120;
  double inertia;                                  // A*s^2/rad
  double coulomb_positive, coulomb_negative;       // A
  double stribeck_positive, stribeck_negative;     // A, extra at zero speed (static-coulomb), decays exp(-|v|/stribeck_speed)
  double stribeck_speed, viscous;                  // rad/s, A*s/rad
  double creep_drop, creep_speed;                  // A, rad/s: below ~creep_speed the level falls by up to creep_drop (creep)
  double presliding_stiffness, presliding_damping; // A/rad, A*s/rad: the LuGre contact (sigma0, sigma1)
  double friction_map_positive[kFrictionBins], friction_map_negative[kFrictionBins];  // A, added by angle
  double load;                                     // A, constant
  double actuation_delay_s, current_tau_s;         // command -> current
  double encoder_delay_s;                          // sample -> host receipt
  double crosstalk_delay_s;                        // reading moves with the command sent this long before receipt
  double crosstalk_map[kCrosstalkBins];            // rad/A over absolute angle
  double counts_per_rev;
  // What the station adds beyond the rigid body (measured 2026-10-24 on three inertia sessions and
  // the survey plateaus, docs/ADR-005/STAGE2.md): the reading carries a noise of this rms, flat to
  // the corner and falling as 1/f above it (0.28 mrad, 40 Hz: the same at every speed and
  // inertia, so it is the encoder's, not motion); and the sliding friction level wanders with this
  // rms and corner (a torque roughness that moves the axis below ~8 Hz). Zero disables either.
  double encoder_noise_rad, encoder_noise_corner_hz;
  double friction_noise_A, friction_noise_corner_hz;
};
YawPlantParameters yaw_plant_from_yaml(const YAML::Node& node);

class YawPlant {
 public:
  explicit YawPlant(const YawPlantParameters& p,double position);
  // A command sent at `time` acts after actuation_delay_s.
  void command(double time,double current);
  void advance(double to);
  double position() const { return q_; }
  double velocity() const { return v_; }
  double current() const { return i_; }
  // Host reading received at `receipt`: the angle sampled encoder_delay_s
  // earlier (call advance(receipt-encoder_delay_s) first), plus the encoder's noise and
  // crosstalk, quantized. Readings are drawn in receipt order (the noise is a process).
  double reading(double sampled_position,double receipt);
  double friction(double v,double q) const;
 private:
  double commanded(double time) const;
  YawPlantParameters p_;
  double t_=0, q_=0, v_=0, i_=0, z_=0;
  std::deque<std::pair<double,double>> commands_;
  // Noise processes (first-order, exact discretisation), from a fixed seed: runs stay repeatable.
  std::mt19937_64 rng_{20261024};
  double friction_noise_=0, encoder_noise_=0, last_reading_time_=0;
  bool read_before_=false;
};

// CyberGear pitch in its own speed mode (RunMode 2): the speed follows SpdRef
// through a delay and a first-order lag with an acceleration limit; each command
// is answered by a type-2 frame carrying the quantized position.
struct PitchPlantParameters {
  double speed_gain;                   // speed / SpdRef at low frequency (<1: friction and gravity load)
  double speed_delay_s, speed_tau_s;   // SpdRef -> speed
  double acceleration_limit;           // rad/s^2 (0: none)
  double reply_delay_s;                // command -> type-2 position receipt
  double position_quantum_rad;
};
PitchPlantParameters pitch_plant_from_yaml(const YAML::Node& node);

class PitchPlant {
 public:
  explicit PitchPlant(const PitchPlantParameters& p,double position);
  void command(double time,double speed);
  void advance(double to);
  double position() const { return q_; }
  double velocity() const { return v_; }
  double reading() const;
 private:
  PitchPlantParameters p_;
  double t_=0, q_=0, v_=0;
  std::deque<std::pair<double,double>> commands_;
};
}
