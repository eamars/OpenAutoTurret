#pragma once
// §81: replay a recorded session through the real controller.
//
// The point of this file is that "deterministic" is not a property of the input, it is a
// property of what happens after the input. Feeding a recorded TrackSet stream into the
// *vision* side proves the tracker agrees with itself; feeding it into ControlLoop with a
// scripted operator proves the mode machine, the selection, the loss timing and the
// reacquisition decision — the parts where the interesting mistakes live. So the replay
// drives the same class that drives the motors.
//
// Two things it deliberately does not do:
//   - It does not own the loop or the backend. The caller hands both over already homed.
//     A replay that re-derives homing is a second station, and a second station can drift
//     from the one that moves metal.
//   - It does not simulate the clock faster than the loop expects. Time advances in whole
//     control periods, so a timing bug in the controller still shows up here. Replaying a
//     20-second occlusion takes 20 seconds' worth of cycles; that is the cost of the
//     result meaning something.
#include <cstdint>
#include <string>
#include <vector>

#include "control/control_loop.hpp"
#include "tracks/track_set.hpp"

namespace ota {

// One detector frame from the recording. `t_ms` is the wall-clock time of the frame in the
// session being replayed, not a cycle count: a recording whose frames arrive irregularly
// is a normal thing on a real station, and a replay that silently regularises them hides
// exactly the staleness paths that matter (§112).
struct ReplayFrame {
  double t_ms = 0.0;
  std::vector<tracks::Track> tracks;
};

// One operator action: SET_MODE, SELECT_TARGET, CLEAR_TARGET, STOP_MOTION, a step...
// Carried as controld's own command name and argument rather than a richer type, so the
// replay cannot accept an operator action the station would refuse. It goes through
// ControlLoop::command_ack the same way a click does.
struct ReplayEvent {
  double t_ms = 0.0;
  std::string command;
  std::string argument;
};

struct ReplayScript {
  std::vector<ReplayFrame> frames;
  std::vector<ReplayEvent> events;
};

// The text format, because a station tool has to run on the station with nothing
// installed: one record per line, fields space-separated, `#` to the end of a line is a
// comment.
//
//   E <t_ms> <command> [argument]
//   F <t_ms> <n> (<uuid_lo> <class_id> <class> <conf> <anchor_x> <anchor_y>
//       <w> <h> <state>){n}
//       state: CONF | TENT | OCCL
//
// Not JSON. Not because JSON is wrong, but because this file is written by a human at a
// bench under time pressure, and a missing comma in a format with no line identity is a
// bad afternoon. A bad line names its line number.
bool parse_replay_script(const std::string& text, ReplayScript& out, std::string& err);

// One line per observable change, in time order. Lines are the transcript, not a report:
// the test greps them, and a person reads them.
//
//   t=1200ms mode=AUTO_TRACK phase=TRACKING sel=1 ev=TARGET_ACQUIRED
//
// Nothing is summarised away: if the phase did not change, no line is written, but every
// published event produces one, because an event that arrives at the wrong moment is the
// thing a recording is replayed to look at.
struct ReplayResult {
  std::vector<std::string> lines;
  std::string error;
  bool ok = false;
};

ReplayResult replay_session(ControlLoop& loop, const ReplayScript& script,
                            int64_t start_ns);

}  // namespace ota
