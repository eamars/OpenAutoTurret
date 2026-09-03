#pragma once
// v3 §38–§41 — MANUAL mode: a dead-man jog, three speed profiles, and fixed steps.
//
// The lease is the whole design. A web button is not a hand on a trigger: the tab can
// be backgrounded, the wifi can drop, the browser can be closed, and none of those
// events are guaranteed to arrive as anything at all. So MANUAL motion is granted for a
// short, renewable deadline rather than "until told to stop", and its absence — not a
// message — is what stops the turret (§38: keepalive expires -> controlled stop ->
// MANUAL/HOLD). A browser that stops talking leaves the turret still.
//
// Three things this deliberately does not do:
//   * It never accepts a raw current, CAN frame, or motor value (§39's last line). The
//     browser sends a direction and a named profile; the numbers come from station
//     configuration that has been validated.
//   * It does not clamp a step to the limits itself. §41 sends "current q + delta"
//     through v1 safety and the v1 trajectory generator, and the envelope is the only
//     thing that knows the limits. Clamping here would produce a turret that quietly
//     goes somewhere slightly different from where it was told, and the operator would
//     never hear about it.
//   * It does not survive a mode change. Leaving MANUAL cancels the lease; coming back
//     does not resume a jog that was interrupted by an AUTO_ROAM sweep (§93's stale
//     intent, in the one form an operator can actually cause by clicking).
//
// Control-thread only. The web thread queues commands; the lease clock is advanced once
// per control cycle, so an expired lease is noticed within 5 ms rather than whenever the
// next HTTP request happens to land.
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "common/time.hpp"
#include "control/motion_intent.hpp"

namespace ota {

enum class ManualProfile : uint8_t { Fine = 0, Normal = 1, Fast = 2 };

inline const char* manual_profile_name(ManualProfile p) {
  switch (p) {
    case ManualProfile::Fine: return "FINE";
    case ManualProfile::Normal: return "NORMAL";
    case ManualProfile::Fast: return "FAST";
  }
  return "NORMAL";
}

// §39: each profile maps to validated v/a/j limits. Expressed as scales on the
// station's configured manual limits rather than absolute numbers, so the safe envelope
// stays the single source of what this turret can do and a profile change cannot
// silently out-authority a commissioning result. FAST is 1.0 of the configured limit
// because the configured limit *is* the validated ceiling; FINE is what a first move on
// an unfamiliar rig should look like.
struct ManualProfileLimits {
  double velocity_scale = 1.0;
  double acceleration_scale = 1.0;
  double jerk_scale = 1.0;
};

struct ManualConfig {
  // §38's suggested 250-350 ms, and §72's `jog_lease_timeout_ms: 300`. The browser
  // refreshes at jog_keepalive_ms (100), so three keepalives' worth of network loss is
  // tolerated before the lease lapses — enough for a couple of lost packets, short
  // enough that nobody has time to worry about what the turret is doing.
  int64_t lease_ms = 300;
  int64_t keepalive_ms = 100;
  // A step is a finite move. If it has not arrived within this long — blocked by the
  // envelope, or by a drive that stopped answering — the intent is dropped rather than
  // held forever, and the operator is told. Without a deadline a refused step leaves a
  // position target pushing against a limit indefinitely.
  int64_t step_timeout_ms = 4000;
  double step_done_tol_rad = 0.0035;  // ~0.2 deg
  ManualProfileLimits fine{0.15, 0.25, 0.40};
  ManualProfileLimits normal{0.45, 0.60, 0.80};
  ManualProfileLimits fast{1.00, 1.00, 1.00};
  // How far ahead of the turret the jog's position reference sits. Not a speed: the
  // rate is set by manual v_max times the profile scale, and the envelope caps it again.
  // The horizon only decides how far the moving target is in front, which is what lets
  // the trajectory generator build an acceleration ramp instead of crawling toward a
  // point it reaches within a cycle. Measured on this station, CyberGear position mode
  // follows a moving reference; its speed mode does not move a loaded axis at the
  // commanded rate on default gains — which is why the intent converter refuses
  // JointVelocity outright (§25) and ManualController integrates instead.
  int64_t jog_horizon_ms = 150;
  // §39: diagonal combined jog is allowed, because the v1 reference interface takes both
  // axes in one request and the envelope limits them together. Two independent
  // single-axis requests would not be the same thing: each would be limited as if the
  // other axis were still.
  bool allow_diagonal = true;
};

// Which way. Bit flags so the four buttons compose, and so "nothing" is a value rather
// than the absence of one.
struct JogDirection {
  int8_t yaw = 0;     // -1, 0, +1
  int8_t pitch = 0;   // -1, 0, +1

  bool any() const { return yaw != 0 || pitch != 0; }
};

struct ManualOutput {
  MotionIntent intent;
  bool lease_active = false;
  int64_t lease_remaining_ms = 0;
  ManualProfile profile = ManualProfile::Normal;
  bool step_in_progress = false;
  const char* reason = "manual hold";
  // A step that the envelope refused is reported once, here, rather than being
  // swallowed: the operator asked for a degree and got nothing, and "no reason given"
  // is the answer that costs an evening of debugging.
  bool step_rejected = false;
  const char* step_reject_reason = "";
};

class ManualController {
 public:
  explicit ManualController(ManualConfig cfg = ManualConfig()) : cfg_(cfg) {}

  void set_config(const ManualConfig& cfg) { cfg_ = cfg; }
  const ManualConfig& config() const { return cfg_; }
  bool lease_active() const { return lease_until_ns_ > 0; }
  ManualProfile profile() const { return profile_; }

  ManualProfileLimits limits(ManualProfile p) const {
    switch (p) {
      case ManualProfile::Fine: return cfg_.fine;
      case ManualProfile::Normal: return cfg_.normal;
      case ManualProfile::Fast: return cfg_.fast;
    }
    return cfg_.normal;
  }

  // Parse "yaw+|pitch-:fine". Returns false with a reason the operator can act on; the
  // direction grammar is shared with the dashboard, and a mismatch there would otherwise
  // show up as a jog that does nothing.
  static bool parse_jog_arg(const char* arg, JogDirection& dir, ManualProfile& profile,
                            char* why, size_t n) {
    dir = {};
    if (arg == nullptr || arg[0] == '\0') {
      std::snprintf(why, n, "jog needs a direction (yaw+ / yaw- / pitch+ / pitch-)");
      return false;
    }
    std::string text(arg);
    const size_t colon = text.find(':');
    if (colon != std::string::npos) {
      std::string name = text.substr(colon + 1);
      for (auto& c : name) c = static_cast<char>(std::tolower(c));
      if (name == "fine") profile = ManualProfile::Fine;
      else if (name == "normal") profile = ManualProfile::Normal;
      else if (name == "fast") profile = ManualProfile::Fast;
      else {
        std::snprintf(why, n, "unknown speed profile (fine/normal/fast)");
        return false;
      }
      text = text.substr(0, colon);
    }
    bool yaw_seen = false, pitch_seen = false;
    size_t pos = 0;
    while (pos < text.size()) {
      size_t bar = text.find('|', pos);
      std::string part = text.substr(pos, bar == std::string::npos
                                             ? std::string::npos : bar - pos);
      // Trim spaces the browser may have added around the separator.
      while (!part.empty() && part.front() == ' ') part.erase(part.begin());
      while (!part.empty() && part.back() == ' ') part.pop_back();
      if (part == "yaw+" || part == "yaw-") {
        if (yaw_seen) { std::snprintf(why, n, "yaw given twice"); return false; }
        yaw_seen = true;
        dir.yaw = part.back() == '+' ? 1 : -1;
      } else if (part == "pitch+" || part == "pitch-") {
        if (pitch_seen) { std::snprintf(why, n, "pitch given twice"); return false; }
        pitch_seen = true;
        dir.pitch = part.back() == '+' ? 1 : -1;
      } else if (!part.empty()) {
        std::snprintf(why, n, "unknown jog direction '%s'", part.c_str());
        return false;
      }
      if (bar == std::string::npos) break;
      pos = bar + 1;
    }
    if (!dir.any()) {
      std::snprintf(why, n, "jog needs a direction (yaw+ / yaw- / pitch+ / pitch-)");
      return false;
    }
    return true;
  }

  // §38. Starting a jog replaces any step in flight: an operator who presses a jog
  // button has changed their mind about where the turret should be going, and the
  // fastest path to a confused machine is to obey both requests.
  bool jog_start(JogDirection dir, ManualProfile profile, TimeNs now_ns) {
    if (!dir.any()) return false;
    if (!cfg_.allow_diagonal && dir.yaw != 0 && dir.pitch != 0) {
      dir.yaw = 0;  // §39 leaves diagonal optional; refusing the second axis is honest,
                    // refusing the whole jog with no explanation is not.
    }
    dir_ = dir;
    profile_ = profile;
    lease_until_ns_ = now_ns + cfg_.lease_ms * 1000000;
    step_active_ = false;
    return true;
  }

  bool jog_keepalive(TimeNs now_ns) {
    if (lease_until_ns_ == 0) return false;  // nothing to renew: the jog already lapsed
    lease_until_ns_ = now_ns + cfg_.lease_ms * 1000000;
    return true;
  }

  void jog_stop(TimeNs now_ns) {
    (void)now_ns;
    lease_until_ns_ = 0;
    dir_ = {};
  }

  // §41: a relative move, formed from the *current logical q* and handed to v1 safety
  // and v1 trajectory untouched. `q_now` is the logical (commanded) position rather than
  // measured feedback, which is what "relative" has to mean on a system where feedback
  // lags and may be absent: a delta applied to a stale measurement would move the turret
  // a different amount than the one that was asked for.
  bool step_move(int axis, double delta_rad, double q_now_rad, TimeNs now_ns) {
    if (axis != 0 && axis != 1) return false;
    if (!(delta_rad != 0.0)) return false;
    // A jog lease and a step are two different intentions; the newer one wins.
    lease_until_ns_ = 0;
    dir_ = {};
    step_active_ = true;
    step_started_ns_ = now_ns;
    step_axis_ = axis;
    step_target_rad_ = q_now_rad + delta_rad;
    step_rejected_ = false;
    return true;
  }

  void cancel(TimeNs now_ns) {
    (void)now_ns;
    lease_until_ns_ = 0;
    dir_ = {};
    step_active_ = false;
  }

  // One control cycle. `q_now` is the logical position of each axis, used only to
  // decide when a step has arrived.
  // `v_max_rad_s` is the station's configured manual limit per axis, already derated by
  // whatever the loop has decided (payload profile, §33.2). ManualController multiplies
  // it by the profile scale; it does not invent a speed of its own.
  ManualOutput update(const double q_now[2], const double v_max_rad_s[2],
                      TimeNs now_ns, TimeNs period_ns) {
    ManualOutput out;
    if (step_rejected_) {
      // Reported once, on the cycle after the envelope refused it, and then cleared. The
      // command acknowledgement has already gone out saying "issued to v1 safety" —
      // which was true at the time — so this is where the outcome lands, and a step that
      // vanished against a limit leaves a trace instead of silence.
      out.step_rejected = true;
      out.step_reject_reason = step_reject_reason_;
      out.reason = step_reject_reason_;
      step_rejected_ = false;
    }
    out.intent.source = MotionSource::Manual;
    out.intent.timestamp_ns = now_ns;
    out.profile = profile_;
    out.step_in_progress = step_active_;

    // The lease is checked before anything else, on the *cycle* clock, so a browser that
    // vanished is noticed in one cycle rather than on the next request that never comes.
    if (lease_until_ns_ != 0 && now_ns >= lease_until_ns_) {
      lease_until_ns_ = 0;
      dir_ = {};
      // Returned, not merely noted. The first version fell through, and the ordinary
      // hold path below overwrote the reason with "manual hold" — so the machine did the
      // right thing (stopped) and the operator was left with a status line that said
      // nothing about the jog they were holding, which is the reporting failure that
      // costs an evening: "it stopped" and "it stopped because you let go" looked
      // identical, and only one of them is a fault.
      out.reason = "jog lease expired";
      out.lease_active = false;
      out.lease_remaining_ms = 0;
      out.intent.set_reason(out.reason);
      return out;
    }

    if (lease_until_ns_ != 0 && dir_.any()) {
      const ManualProfileLimits lim = limits(profile_);
      out.lease_active = true;
      out.lease_remaining_ms = (lease_until_ns_ - now_ns) / 1000000;
      // Integrated into a position reference at the control rate, which is what the
      // converter's note in reference_manager.hpp asks for. The target is re-derived
      // from `q_now` every cycle rather than accumulated, and that is the safety
      // property, not a shortcut: if the turret is stopped by the envelope, the
      // reference stays a fixed short distance ahead instead of running away, so the
      // motion that resumes when the way opens is small. An accumulated target would
      // bank up a debt during the block and spend it the instant the limit moved —
      // a lurch at exactly the moment the operator was told to be careful.
      const double horizon = static_cast<double>(cfg_.jog_horizon_ms) / 1000.0;
      out.intent.type = IntentType::JointPosition;
      out.intent.has_joint_target = true;
      out.intent.v_yaw_rad_s = static_cast<double>(dir_.yaw) * v_max_rad_s[1] *
                              lim.velocity_scale;
      out.intent.v_pitch_rad_s = static_cast<double>(dir_.pitch) * v_max_rad_s[0] *
                                lim.velocity_scale;
      out.intent.q_yaw_rad = q_now[1] + out.intent.v_yaw_rad_s * horizon;
      out.intent.q_pitch_rad = q_now[0] + out.intent.v_pitch_rad_s * horizon;
      out.intent.velocity_scale = lim.velocity_scale;
      out.intent.acceleration_scale = lim.acceleration_scale;
      out.intent.jerk_scale = lim.jerk_scale;
      // A lease is a deadline that renews; the intent carries one a cycle longer than the
      // browser's refresh, so an intent is never honoured after the lease it came from.
      out.intent.valid_until_ns = lease_until_ns_ + period_ns;
      // Set on the intent as well as the output: the intent is what reaches telemetry,
      // and a jog whose published reason was empty looked like a bug in the intent
      // builder when it was only a missing line here.
      out.intent.set_reason("jogging");
      out.reason = "jogging";
      return out;
    }

    if (step_active_) {
      const double err = step_target_rad_ - q_now[step_axis_];
      const bool arrived = std::fabs(err) <= cfg_.step_done_tol_rad;
      const bool timed_out =
          (now_ns - step_started_ns_) / 1000000 >= cfg_.step_timeout_ms;
      if (arrived || timed_out) {
        step_active_ = false;
        // Cleared on the output too, not just internally. The field is set from the
        // member at the top of update(), so leaving it means telemetry keeps reporting
        // an in-progress step after the turret has stopped moving — a status line that
        // says the machine is still doing something it finished.
        out.step_in_progress = false;
        out.reason = arrived ? "step complete" : "step timed out";
        out.intent.set_reason(out.reason);
        return out;  // Hold, which is what an arrived step is.
      }
      out.step_in_progress = true;
      out.intent.type = IntentType::JointPosition;
      out.intent.has_joint_target = true;
      out.intent.q_pitch_rad = q_now[0];
      out.intent.q_yaw_rad = q_now[1];
      out.intent.v_pitch_rad_s = 0.0;
      out.intent.v_yaw_rad_s = 0.0;
      if (step_axis_ == 0) out.intent.q_pitch_rad = step_target_rad_;
      else out.intent.q_yaw_rad = step_target_rad_;
      // The step inherits the last profile, so a 5 deg move at FINE is slow in the same
      // way a jog at FINE is: the profile means something to the operator and should
      // keep meaning the same thing whichever button produced the motion.
      const ManualProfileLimits lim = limits(profile_);
      out.intent.velocity_scale = lim.velocity_scale;
      out.intent.acceleration_scale = lim.acceleration_scale;
      out.intent.jerk_scale = lim.jerk_scale;
      out.intent.valid_until_ns = now_ns + 2 * period_ns;
      out.reason = "stepping";
      return out;
    }

    out.reason = "manual hold";
    out.intent.set_reason(out.reason);
    return out;  // type stays Hold; a hold has no deadline, per motion_intent.hpp.
  }

  // §70's report-back: the loop tells us when the envelope refused the step we asked
  // for, so it can be said out loud instead of looking like an unresponsive button.
  void notify_step_refused() {
    if (!step_active_) return;
    step_active_ = false;
    step_rejected_ = true;
    step_reject_reason_ = "target outside the safe envelope (22)";
  }

  bool last_step_rejected() const { return step_rejected_; }
  const char* last_step_reject_reason() const { return step_reject_reason_; }
  void clear_step_rejection() { step_rejected_ = false; }

 private:
  ManualConfig cfg_;
  JogDirection dir_{};
  ManualProfile profile_ = ManualProfile::Normal;
  TimeNs lease_until_ns_ = 0;
  bool step_active_ = false;
  TimeNs step_started_ns_ = 0;
  int step_axis_ = 1;
  double step_target_rad_ = 0.0;
  bool step_rejected_ = false;
  const char* step_reject_reason_ = "";
};

}  // namespace ota
