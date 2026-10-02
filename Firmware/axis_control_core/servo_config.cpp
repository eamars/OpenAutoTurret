#include "servo_config.hpp"
#include <sstream>
#include <stdexcept>
#include <vector>

namespace ota::axis {
namespace {
template <std::size_t N>
void table(const YAML::Node& node,const char* key,double (&target)[N]) {
  const auto values=node[key].as<std::vector<double>>();
  if (values.size()!=N) throw std::runtime_error(std::string("DATA_INVALID: servo table size: ")+key);
  std::copy(values.begin(),values.end(),target);
}
void array_json(std::ostream& out,const double* values,int n) {
  out<<'[';
  for (int k=0;k<n;++k) out<<(k?",":"")<<values[k];
  out<<']';
}
}
#define OTA_SERVO_SCALARS(X) \
  X(encoder_variance) X(gyro_variance) X(process_variance) X(max_encoder_age_s) X(max_gyro_age_s) \
  X(inertia) X(coulomb_positive) X(coulomb_negative) X(stribeck_positive) X(stribeck_negative) X(stribeck_speed) \
  X(viscous) X(friction_band) X(load) X(friction_correction_rate) X(friction_correction_deadband) \
  X(dither_amplitude) X(dither_period_s) X(friction_learning_rate) X(friction_learning_speed) X(friction_map_limit) \
  X(kq) X(kv) X(ki) X(integral_cap) X(error_clamp) X(hold_band) X(hold_speed) X(hold_relax_tau_s) \
  X(current_cap) X(slew) X(rms_limit) X(rms_tau_s) X(dt_min) X(dt_max) X(following_error_limit)

ServoParameters servo_from_yaml(const YAML::Node& node) {
  ServoParameters p{};
#define OTA_LOAD(name) p.name=node[#name].as<double>();
  OTA_SERVO_SCALARS(OTA_LOAD)
#undef OTA_LOAD
  p.use_gyro=node["use_gyro"].as<int>();
  p.creep_drop=node["creep_drop"].as<double>(0.); p.creep_speed=node["creep_speed"].as<double>(0.01);
  table(node,"friction_map_positive",p.friction_map_positive);
  table(node,"friction_map_negative",p.friction_map_negative);
  p.crosstalk_delay_s=node["crosstalk_delay_s"]?node["crosstalk_delay_s"].as<double>():0.;
  if (node["crosstalk_map"]) table(node,"crosstalk_map",p.crosstalk_map);
  if (const auto stall=node["stall_recovery"]) {
    p.stall_error=stall["error_rad"].as<double>(); p.stall_speed=stall["speed_rad_s"].as<double>();
    p.stall_current=stall["current_A"].as<double>(); p.stall_time_s=stall["time_s"].as<double>();
    p.rock_current=stall["rock_current_A"].as<double>(); p.rock_s=stall["rock_s"].as<double>();
    p.stall_reference_speed=stall["reference_speed_rad_s"].as<double>(0.);
  }
  if (!valid(p)) throw std::runtime_error("DATA_INVALID: servo parameters invalid");
  return p;
}

std::string servo_to_json(const ServoParameters& p) {
  std::ostringstream out; out.precision(9); out<<'{';
#define OTA_JSON(name) out<<"\"" #name "\":"<<p.name<<',';
  OTA_SERVO_SCALARS(OTA_JSON)
#undef OTA_JSON
  out<<"\"use_gyro\":"<<p.use_gyro<<",\"creep_drop\":"<<p.creep_drop<<",\"creep_speed\":"<<p.creep_speed<<",\"friction_map_positive\":";
  array_json(out,p.friction_map_positive,ServoParameters::kFrictionBins);
  out<<",\"friction_map_negative\":"; array_json(out,p.friction_map_negative,ServoParameters::kFrictionBins);
  out<<",\"crosstalk_delay_s\":"<<p.crosstalk_delay_s<<",\"crosstalk_map\":";
  array_json(out,p.crosstalk_map,ServoParameters::kCrosstalkBins);
  out<<",\"stall_recovery\":{\"error_rad\":"<<p.stall_error<<",\"speed_rad_s\":"<<p.stall_speed
     <<",\"current_A\":"<<p.stall_current<<",\"time_s\":"<<p.stall_time_s<<",\"rock_current_A\":"<<p.rock_current
     <<",\"rock_s\":"<<p.rock_s<<",\"reference_speed_rad_s\":"<<p.stall_reference_speed<<"}}";
  return out.str();
}
#undef OTA_SERVO_SCALARS
}
