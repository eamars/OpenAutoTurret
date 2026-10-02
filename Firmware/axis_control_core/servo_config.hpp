#pragma once
#include "servo.hpp"
#include <string>
#include <yaml-cpp/yaml.h>

namespace ota::axis {
// The servo asset format (config/servo/yaw_servo.json "servo_parameters"): one
// loader for commissiond, the simulator and the tooling. Throws on missing,
// malformed or invalid values.
ServoParameters servo_from_yaml(const YAML::Node& node);
// Round-trips through servo_from_yaml. 9 significant digits keeps the whole
// record (maps included) inside one 4 KiB journal line.
std::string servo_to_json(const ServoParameters& p);
}
