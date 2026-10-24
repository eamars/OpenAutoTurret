#include "control/servo_assets.hpp"

#include <cmath>

#include <yaml-cpp/yaml.h>

#include "servo_config.hpp"

namespace ota {

bool parse_servo_assets(const std::string& yaw_path, const std::string& pitch_path, ServoAssets& out,
                        std::string& err) {
  ServoAssets a;
  a.yaw_path = yaw_path;
  a.pitch_path = pitch_path;
  try {
    const auto yaw_asset = YAML::LoadFile(yaw_path);
    a.yaw = axis::servo_from_yaml(yaw_asset["servo_parameters"]);
    if (axis::Servo probe; !probe.configure(a.yaw)) {
      err = "yaw servo asset rejected: " + yaw_path;
      return false;
    }
    const auto identified = yaw_asset["identified"]["crosstalk"]["table"];
    if (identified && identified.IsSequence() && identified.size() == a.yaw_crosstalk.size()) {
      bool finite = true;
      for (std::size_t k = 0; k < a.yaw_crosstalk.size(); ++k) {
        a.yaw_crosstalk[k] = identified[k].as<double>();
        finite = finite && std::isfinite(a.yaw_crosstalk[k]);
      }
      a.yaw_crosstalk_valid = finite;
    }
    const auto trial = YAML::LoadFile(pitch_path)["servo_trial"];
    // The asset's speed clamp was a commissioning choice; the backend applies the owner's cap.
    a.pitch = {trial["kp_per_s"].as<double>(), trial["ki_per_s2"].as<double>(),
               trial["integral_clamp_rad_s"].as<double>(), 1.0};
    a.pitch_following_error_rad = trial["following_error_rad"].as<double>();
    if (axis::PositionLoop probe; !probe.configure(a.pitch) || !(a.pitch_following_error_rad > 0)) {
      err = "pitch servo asset rejected: " + pitch_path;
      return false;
    }
  } catch (const std::exception& error) {
    err = std::string("servo asset: ") + error.what();
    return false;
  }
  out = a;
  return true;
}

}  // namespace ota
