// A condition is a transient until it persists (owner ruling 2026-10-02, STATION_OPERATIONS.md
// "Fault, hold, degrade"): an episode starts when the condition is present, ends only after the
// condition has been clearly absent for `clear_ns` (hysteresis: "present" and "clearly absent" are
// separate tests), and becomes a failure only once it has lasted `persist_ns`.
#pragma once

#include "common/types.hpp"

namespace ota::control {

class EpisodeLatch {
 public:
  enum class Event { None, Started, Persisted, Cleared };

  Event update(TimeNs now, bool present, bool clearly_absent, TimeNs persist_ns, TimeNs clear_ns) {
    if (present) {
      quiet_ = false;
      if (!active_) { active_ = true; since_ = now; return Event::Started; }
    } else if (active_ && clearly_absent) {
      if (!quiet_) { quiet_ = true; quiet_since_ = now; }
      if (now - quiet_since_ >= clear_ns) {
        duration_ns_ = quiet_since_ - since_;
        active_ = quiet_ = persisted_ = false;
        return Event::Cleared;
      }
    } else {
      quiet_ = false;   // back between "clearly absent" and "present": the quiet period restarts
    }
    if (active_ && !persisted_ && now - since_ >= persist_ns) {
      persisted_ = true;
      return Event::Persisted;
    }
    return Event::None;
  }
  bool active() const { return active_; }
  bool persisted() const { return persisted_; }
  // The length of the episode that last cleared (start to the beginning of its quiet period).
  TimeNs last_duration_ns() const { return duration_ns_; }

 private:
  TimeNs since_ = 0, quiet_since_ = 0, duration_ns_ = 0;
  bool active_ = false, quiet_ = false, persisted_ = false;
};

}  // namespace ota::control
