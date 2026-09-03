#include "control/session_replay.hpp"

#include <cmath>
#include <algorithm>
#include <cstdio>
#include <sstream>

#include "control/control_loop.hpp"

namespace ota {
namespace {

constexpr int64_t kMsToNs = 1000000;

tracks::TrackState parse_state(const std::string& s, bool& ok) {
  ok = true;
  if (s == "CONF") return tracks::TrackState::Confirmed;
  if (s == "TENT") return tracks::TrackState::Tentative;
  if (s == "OCCL") return tracks::TrackState::Occluded;
  if (s == "LOST") return tracks::TrackState::Lost;
  ok = false;
  return tracks::TrackState::Tentative;
}

}  // namespace

bool parse_replay_script(const std::string& text, ReplayScript& out, std::string& err) {
  std::istringstream in(text);
  std::string line;
  int lineno = 0;
  double last_t = -1.0;
  while (std::getline(in, line)) {
    ++lineno;
    const auto hash = line.find('#');
    if (hash != std::string::npos) line.erase(hash);
    std::istringstream f(line);
    std::string kind;
    if (!(f >> kind)) continue;
    if (kind != "E" && kind != "F") {
      err = "line " + std::to_string(lineno) + ": records begin with E or F, got '" + kind +
            "'";
      return false;
    }
    double t = 0.0;
    if (!(f >> t) || !std::isfinite(t) || t < 0.0) {
      err = "line " + std::to_string(lineno) + ": expected a non-negative t_ms";
      return false;
    }
    // Refused rather than sorted, and this one is not pedantry. The file's order *is* the
    // operator's action order: a recording that asked for SELECT_TARGET before SET_MODE
    // produced a different session from the one being replayed, and silently reordering
    // the two replays a session nobody performed.
    if (t < last_t) {
      char msg[160];
      std::snprintf(msg, sizeof msg,
                    "line %d: t_ms %.3f goes backwards from %.3f — the order of operator "
                    "actions is part of the session being replayed",
                    lineno, t, last_t);
      err = msg;
      return false;
    }
    last_t = t;

    if (kind == "E") {
      ReplayEvent ev;
      ev.t_ms = t;
      if (!(f >> ev.command)) {
        err = "line " + std::to_string(lineno) + ": E needs controld's command name";
        return false;
      }
      f >> ev.argument;  // optional: stop_motion has none
      out.events.push_back(ev);
      continue;
    }

    ReplayFrame fr;
    fr.t_ms = t;
    int n = 0;
    if (!(f >> n) || n < 0 || n > tracks::kMaxTracks) {
      err = "line " + std::to_string(lineno) + ": F needs a track count in [0," +
            std::to_string(tracks::kMaxTracks) + "]";
      return false;
    }
    for (int i = 0; i < n; ++i) {
      tracks::Track tr;
      unsigned long long uuid = 0;
      int class_id = 0;
      std::string cls, state;
      double conf = 0.0, ax = 0.0, ay = 0.0, w = 0.0, h = 0.0;
      if (!(f >> uuid >> class_id >> cls >> conf >> ax >> ay >> w >> h >> state)) {
        err = "line " + std::to_string(lineno) + ": track " + std::to_string(i) +
              " of this frame is incomplete (uuid class_id class conf ax ay w h state)";
        return false;
      }
      // The id is in the format beside the name, and not redundantly: §14's selectability
      // is decided by class *id*, which belongs to the station's configuration, while the
      // name is what the detector wrote. A recording carrying only a name would force a
      // guess, and the guessed number decides whether the operator may select this target.
      if (class_id < 0 || class_id > 65535) {
        err = "line " + std::to_string(lineno) + ": class_id out of range";
        return false;
      }
      tr.class_id = static_cast<uint16_t>(class_id);
      bool sok = false;
      tr.state = parse_state(state, sok);
      if (!sok) {
        err = "line " + std::to_string(lineno) + ": state '" + state +
              "' is not CONF, TENT, OCCL or LOST";
        return false;
      }
      if (cls.size() >= sizeof(tr.class_name)) {
        err = "line " + std::to_string(lineno) + ": class name too long";
        return false;
      }
      tr.uuid.lo = uuid;
      std::snprintf(tr.class_name, sizeof tr.class_name, "%s", cls.c_str());
      tr.detector_confidence = static_cast<float>(conf);
      tr.track_confidence = static_cast<float>(conf);
      tr.anchor_x = static_cast<float>(ax);
      tr.anchor_y = static_cast<float>(ay);
      // The format speaks width/height because that is what a person reads off a
      // detector; the wire carries a box, and the anchor is the centre the tracker follows
      // (§9). Deriving one from the other here keeps that translation in the one place a
      // recording ever enters the controller.
      tr.bbox.x_min = static_cast<float>(ax - w / 2.0);
      tr.bbox.x_max = static_cast<float>(ax + w / 2.0);
      tr.bbox.y_min = static_cast<float>(ay - h / 2.0);
      tr.bbox.y_max = static_cast<float>(ay + h / 2.0);
      fr.tracks.push_back(tr);
    }
    out.frames.push_back(fr);
  }
  err.clear();
  return true;
}

ReplayResult replay_session(ControlLoop& loop, const ReplayScript& script,
                            int64_t start_ns) {
  ReplayResult res;
  const int64_t dt_ns = 5000000;  // the period the loop expects, not one of our invention
  size_t next_frame = 0;
  size_t next_event = 0;
  int64_t t = start_ns;
  const int64_t stop_ns = [&script]() {
    int64_t s = 0;
    for (const auto& f : script.frames) s = std::max(s, static_cast<int64_t>(f.t_ms * kMsToNs));
    for (const auto& e : script.events)
      s = std::max(s, static_cast<int64_t>(e.t_ms * kMsToNs));
    return s;
  }();

  std::string last_phase, last_mode;
  int last_sel = -1;
  uint64_t last_event_ns = 0;
  bool have_event_watermark = false;
  bool saw_ready = false;

  while (t - start_ns <= stop_ns) {
    while (next_frame < script.frames.size() &&
           script.frames[next_frame].t_ms * kMsToNs <= t - start_ns) {
      const ReplayFrame& f = script.frames[next_frame++];
      tracks::TrackSet set;
      set.frame_sequence = next_frame;
      // The frame carries the time it was recorded at, not the time the replay happens to
      // be at. That is the difference between replaying a session and replaying a
      // generator: the staleness of an estimate (§50) has to come out of the recording.
      set.sensor_timestamp_ns = start_ns + static_cast<int64_t>(f.t_ms * kMsToNs);
      set.publish_timestamp_ns = set.sensor_timestamp_ns;
      set.width = 1280;
      set.height = 720;
      set.count = static_cast<uint16_t>(f.tracks.size());
      for (size_t i = 0; i < f.tracks.size(); ++i) {
        set.tracks[i] = f.tracks[i];
        if (set.tracks[i].display_index == 0) set.tracks[i].display_index = uint16_t(i + 1);
      }
      loop.feed_track_set(set, set.sensor_timestamp_ns);
    }

    while (next_event < script.events.size() &&
           script.events[next_event].t_ms * kMsToNs <= t - start_ns) {
      const ReplayEvent& e = script.events[next_event++];
      // Submitted, then a cycle to apply it, exactly as the web thread does. The ack the
      // loop writes back is part of the transcript: a replay that reports the mode the
      // station reached while quietly refusing the command is a replay of a fantasy.
      loop.submit_command(e.command.c_str(), e.argument.c_str());
      loop.step(t, dt_ns);
      t += dt_ns;
      const auto& ack = loop.last_command_ack();
      char line[256];
      std::snprintf(line, sizeof line, "t=%.0fms cmd=%s %s%s%s", double(t - start_ns) / 1e6,
                    e.command.c_str(), ack.accepted ? "accepted" : "REFUSED",
                    ack.accepted ? "" : " reason=", ack.accepted ? "" : ack.reason.c_str());
      res.lines.emplace_back(line);
    }

    loop.step(t, dt_ns);
    t += dt_ns;

    const telemetry::TelemetrySnapshot snap = loop.telemetry().snapshot();
    if (!saw_ready && snap.phase == "hold") saw_ready = true;

    if (!have_event_watermark) {
      last_event_ns = 0;
      for (int i = 0; i < snap.event_tail_count; ++i)
        last_event_ns = std::max<uint64_t>(last_event_ns, snap.event_tail[i].t_ns);
      have_event_watermark = true;
    }

    if (snap.operating_mode != last_mode || snap.mode_phase != last_phase ||
        int(snap.selected_display_index) != last_sel) {
      char line[256];
      std::snprintf(line, sizeof line, "t=%.0fms mode=%s phase=%s sel=%u",
                    double(t - start_ns) / 1e6, snap.operating_mode.c_str(),
                    snap.mode_phase.c_str(), unsigned(snap.selected_display_index));
      res.lines.emplace_back(line);
      last_mode = snap.operating_mode;
      last_phase = snap.mode_phase;
      last_sel = int(snap.selected_display_index);
    }

    // The tail is the last eight events, newest last. Anything newer than the watermark is
    // new to the transcript; the watermark starts at whatever the ring already held, so
    // the boot-time decisions the station made before the session began are not reported
    // as if the recording caused them.
    for (int i = 0; i < snap.event_tail_count; ++i) {
      const auto& ev = snap.event_tail[i];
      if (uint64_t(ev.t_ns) <= last_event_ns) continue;
      last_event_ns = uint64_t(ev.t_ns);
      char line[256];
      std::snprintf(line, sizeof line, "t=%.0fms event=%s %s", double(ev.t_ns - start_ns) / 1e6,
                    ev.name, ev.detail[0] ? ev.detail : "");
      res.lines.emplace_back(line);
    }
  }

  if (!saw_ready) {
    res.error =
        "the loop never reached a holding phase: the replay needs a homed station "
        "(start_homing run to Ready) before it can mean anything";
    return res;
  }
  res.ok = true;
  return res;
}

}  // namespace ota
