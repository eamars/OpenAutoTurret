#pragma once
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace ota::axis {
// Closed-loop simulation of one commissioning session against a plant model,
// using the same control classes and timing structure as commissiond:
//  yaw   -- Servo stepped on every encoder receipt (servo_event_control), the
//           reference relative to the start, excitation added after the servo,
//           gain schedule, 20 ms encoder speed trip, following-error stop;
//  pitch -- PositionLoop at command_period_s on the latest type-2 position,
//           excitation added to SpdRef, following-error and window stops.
// Deterministic: no random numbers; the encoder quantizes.
struct SimulationResult {
  std::vector<std::string> columns;
  std::vector<std::vector<double>> rows;
  std::string status;   // COMPLETE, or the stop reason
  std::string learned;  // yaw: the servo's parameters at the end (learned friction maps), as servo_to_json
};
SimulationResult simulate(const YAML::Node& request);
}
