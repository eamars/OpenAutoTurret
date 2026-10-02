#pragma once
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace ota::track {
// ADR-003 stage 1: the whole chain in simulation, with the production classes.
//
//   target truth (base-frame LOS, piecewise constant acceleration, jumps, identity changes)
//   -> camera (frame period and jitter, exposure, timestamp at exposure start, a deliberately
//      wrong timestamp offset if asked, pixel noise, latency and its jitter, drops, extra delays)
//   -> latest-only delivery to the control tick -> Tracker (estimator, Level 1)
//   -> yaw: axis::Servo stepped on every 1 kHz encoder receipt against YawPlant (friction,
//      crosstalk, delay); pitch: axis::PositionLoop at 1 kHz against PitchPlant; or an ideal
//      actuator (the axis is exactly at the reference: Level 1 in isolation)
//   -> the true framing error: the true target projected through the true camera pose.
// Deterministic: the noise comes from a seeded generator.
//
// Injections (ADR-003 sec. 6, the "controlled test entry"): drops, extra delay, velocity
// unavailable (target motion off in the tracker for a window), actuator saturation (yaw
// current or pitch speed clamped for a window), identity change (a truth segment field).
struct TrackingSimulation {
  std::vector<std::string> tick_columns, frame_columns;
  std::vector<std::vector<double>> ticks, frames;
  std::string status;       // COMPLETE or the reason it stopped
  std::string parameters;   // the tracker parameters actually applied (readback)
};
TrackingSimulation simulate_tracking(const YAML::Node& request);
}
