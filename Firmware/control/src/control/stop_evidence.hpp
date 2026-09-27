#pragma once
// Per-axis stop evidence (ADR-001 contracts §8, WP2).
//
// Why this exists when `MIXED STOPPED: …` already logs the truth: prose in a log is
// written for whoever is reading it at that moment, and a stop is exactly the record
// that gets read later, by someone who wasn't there, asking "which axis do we believe,
// and on what grounds". The launcher cannot carry a sentence; an artifact can.
//
// The contract's two hardest lines, encoded in the types rather than in a comment:
//   1. Fields must distinguish requested / observed / confirmed / unsupported. So an
//      axis is NOT a bool. `pitch && yaw` -- the forbidden "two booleans ANDed and
//      painted green as safely powered down" -- is not expressible here.
//   2. `stationary_observed` proves only that no motion was detected during a stated
//      window; it does not prove the motor was de-torqued. Which is why the GM6020's
//      evidence stays `Unsupported` instead of being flattered into `false`:
//      the feedback frame has no enable bit, and a thing we cannot see is not a thing
//      we observed to be absent.
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace ota {

// Ordered from "nothing at all" upward, so a comparison reads as a claim of strength.
enum class Evidence : uint8_t {
  Absent,       // nothing happened and nothing was seen: belongs in missing_evidence
  Requested,    // we asked the hardware to do it; no readback consulted
  Observed,     // inferred from feedback over a stated window
  Confirmed,    // the device itself said so
  Unsupported,  // this device cannot report it at all -- absence of the capability
};

inline const char* evidence_name(Evidence e) {
  switch (e) {
    case Evidence::Absent: return "absent";
    case Evidence::Requested: return "requested";
    case Evidence::Observed: return "observed";
    case Evidence::Confirmed: return "confirmed";
    case Evidence::Unsupported: return "unsupported";
  }
  return "absent";
}

struct AxisStopEvidence {
  const char* axis = "";
  Evidence zero_requested = Evidence::Absent;
  Evidence disable_requested = Evidence::Absent;
  Evidence disable_confirmed = Evidence::Unsupported;  // GM6020: no such bit to read
  Evidence stationary_observed = Evidence::Absent;
  long long stationary_window_ns = 0;  // meaningful only when stationary_observed
  double feedback_age_ms = -1.0;       // -1 = unknown. 0 would be a reading, and a lie.
  long long last_neutral_request_ns = 0;
};

struct StopEvidence {
  std::string stop_id;   // "stop-<monotonic ns of the request>": same clock family as traces
  std::string reason;    // who/what asked, in one machine-readable token
  long long requested_at_ns = 0;
  std::string stage;     // how far the sequence got when this record was written
  AxisStopEvidence axes[2];
  Evidence power_isolated_confirmed = Evidence::Unsupported;  // nobody can observe this
  std::vector<std::string> missing_evidence;                  // computed, never hand-set
  std::string completion_quality;                             // computed, see below

  // "verified" only when every claim that THIS hardware could support reached at least
  // `Observed`, and nothing observable is `Absent`. `Unsupported` deliberately neither
  // helps nor hurts: it is a fact about the drive, not a gap in this stop.
  // "unverified" means we asked and cannot say anything about the outcome; "partial" is
  // the honest middle, and it names what is missing rather than hiding behind a green.
  static std::string judge(const AxisStopEvidence* axes,
                           std::vector<std::string>& missing) {
    bool any_observed = false;
    for (int i = 0; i < 2; ++i) {
      const AxisStopEvidence& a = axes[i];
      if (a.zero_requested == Evidence::Absent && a.disable_requested == Evidence::Absent)
        missing.push_back(std::string(a.axis) + ":no_request");
      if (a.stationary_observed == Evidence::Absent)
        missing.push_back(std::string(a.axis) + ":no_stationary_observation");
      if (a.feedback_age_ms < 0.0)
        missing.push_back(std::string(a.axis) + ":feedback_age_unknown");
      if (a.stationary_observed == Evidence::Observed ||
          a.stationary_observed == Evidence::Confirmed ||
          a.disable_confirmed == Evidence::Confirmed)
        any_observed = true;
    }
    if (missing.empty() && any_observed) return "verified";
    return any_observed ? "partial" : "unverified";
  }

  void finalise() {
    missing_evidence.clear();
    completion_quality = judge(axes, missing_evidence);
  }

  std::string to_json_line() const {
    char buf[160];
    std::string s = "{\"schema_version\":\"ota.n1.draft.stop-evidence/1\",\"stop_id\":\"" +
                    stop_id + "\",\"reason\":\"" + reason + "\",\"stage\":\"" + stage +
                    "\",\"requested_at_ns\":\"" + std::to_string(requested_at_ns) +
                    "\",\"axes\":[";
    for (int i = 0; i < 2; ++i) {
      const AxisStopEvidence& a = axes[i];
      if (i) s += ",";
      s += "{\"axis\":\"";
      s += a.axis;
      s += "\",\"zero_requested\":\"";
      s += evidence_name(a.zero_requested);
      s += "\",\"disable_requested\":\"";
      s += evidence_name(a.disable_requested);
      s += "\",\"disable_confirmed\":\"";
      s += evidence_name(a.disable_confirmed);
      s += "\",\"stationary_observed\":\"";
      s += evidence_name(a.stationary_observed);
      s += "\"";
      std::snprintf(buf, sizeof buf, ",\"stationary_window_ms\":%.1f",
                    static_cast<double>(a.stationary_window_ns) / 1e6);
      s += buf;
      if (a.feedback_age_ms < 0.0)
        s += ",\"feedback_age_ms\":null";
      else {
        std::snprintf(buf, sizeof buf, ",\"feedback_age_ms\":%.3f", a.feedback_age_ms);
        s += buf;
      }
      s += ",\"last_neutral_request_ns\":\"" + std::to_string(a.last_neutral_request_ns) + "\"}";
    }
    s += "],\"power_isolated_confirmed\":\"";
    s += evidence_name(power_isolated_confirmed);
    s += "\",\"completion_quality\":\"" + completion_quality + "\",\"missing_evidence\":[";
    for (std::size_t i = 0; i < missing_evidence.size(); ++i) {
      if (i) s += ",";
      s += "\"";
      s += missing_evidence[i];
      s += "\"";
    }
    s += "]}\n";
    return s;
  }
};

}  // namespace ota
