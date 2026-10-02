#pragma once
#include "tracker.hpp"
#include <string>
#include <yaml-cpp/yaml.h>

namespace ota::track {
// The tracking parameter asset (config/tracking/tracking.json, schema ota.tracking/1): every
// Level-1 and estimator number, in SI units, read with no defaults. A missing or invalid value
// fails the load: a parameter change can never be silently ignored (ADR-003 decision 11).
TrackerParameters tracker_from_yaml(const YAML::Node& node);
// The applied parameters, read back as JSON in the same schema (round-trips through the loader).
std::string tracker_to_json(const TrackerParameters& p);
}
