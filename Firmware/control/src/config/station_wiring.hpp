#pragma once
// One place where turret.yaml becomes a station.
//
// These mappings used to live in main.cpp, which meant the daemon was the only thing that
// could build a station from a configuration file. A replay tool that needed the same
// station had two bad options: duplicate the mapping and watch it drift, or replay against
// built-in defaults and quietly compare two different machines. The second is the kind of
// mistake that produces a transcript everyone trusts about a station that never existed, so
// the mapping moved out here and both the daemon and the tools call it.
//
// Degrees in the file, radians in the control layer. Every conversion in this project
// happens in a function like these, never in the middle of a component.
#include <string>

#include "calibration/homing_plan.hpp"
#include "config/turret_config.hpp"
#include "control/control_loop.hpp"

namespace ota {
namespace wire {

// §58 `homing_plan:` + the contact-detection parameters. An empty plan section means
// "full-range home both axes, pitch then yaw". `err` is only set for an action the file
// spells badly; a caller that ignores it will home something it did not ask for.
HomingPlan make_homing_plan(const config::TurretConfig& cfg, std::string& err);

// The control loop's own configuration, including the §72 values an operator may name.
// Values the file did not name stay zero/false, which the loop reads as "derive it" — see
// the comment in V3Config about why an absent key must not look like a chosen one.
ControlLoop::Config make_control_cfg(const config::TurretConfig& cfg);

}  // namespace wire
}  // namespace ota
