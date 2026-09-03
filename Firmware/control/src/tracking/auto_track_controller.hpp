#pragma once
// v3 §15–§20 — what AUTO_TRACK is doing, and what it is therefore willing to ask for.
//
// This is not a second tracking controller. The prediction, the timestamp alignment and
// the joint solve stay exactly where they are (§17, §111.18); what lives here is the
// *policy* around them — whether following the prediction is allowed at all this cycle,
// and with how much authority. The v1 FSM answers "is the estimator tracking?" which is
// a question about a filter. §15 asks a different question: does the operator have a
// selection, can we see it, how long has it been gone, and should we still be moving?
//
// The one behaviour change that matters to anyone standing at the station: with no
// selection, AUTO_TRACK does nothing. v1 acquired the best detection it could find. §16
// is explicit — "No target selected: remain WAIT_TARGET, show 'Select a target', no
// autonomous motion" — and the interim rule that preserved v1's behaviour on the way
// here is retired by this file, not left as a fallback. A turret that moves because it
// found *somebody* is a different machine from one that moves because the operator
// chose somebody, and the difference is only visible when the wrong person walks by.
#include <algorithm>
#include <cstdint>

#include "common/time.hpp"

namespace ota {

// §15's diagram, as an enum. Names match the document because they are what appears on
// the dashboard (§50) and in the operator's manual.
enum class AutoTrackState : uint8_t {
  WaitTarget = 0,       // no selection, or a selection that cannot be seen
  Acquire = 1,          // seen; building the evidence to follow it
  Tracking = 2,         // following the predicted LOS
  Coasting = 3,         // §20.1: prediction continues, authority shrinks
  LostHold = 4,         // §20.2: prediction stopped, controlled brake to hold
  Reacquire = 5,        // §20.3: back, without an operator click
  TargetUnreachable = 6  // §22: the LOS is outside what the envelope allows
};

inline const char* auto_track_state_name(AutoTrackState s) {
  switch (s) {
    case AutoTrackState::WaitTarget: return "WAIT_TARGET";
    case AutoTrackState::Acquire: return "ACQUIRE";
    case AutoTrackState::Tracking: return "TRACKING";
    case AutoTrackState::Coasting: return "COASTING";
    case AutoTrackState::LostHold: return "LOST_HOLD";
    case AutoTrackState::Reacquire: return "REACQUIRE";
    case AutoTrackState::TargetUnreachable: return "TARGET_UNREACHABLE";
  }
  return "WAIT_TARGET";
}

// §19's four confidence states. These are not the estimator's probability: they are the
// bands that decide how much authority the turret grants itself.
enum class ConfidenceBand : uint8_t { Invalid = 0, Low = 1, Medium = 2, High = 3 };

inline const char* confidence_band_name(ConfidenceBand b) {
  switch (b) {
    case ConfidenceBand::Invalid: return "INVALID";
    case ConfidenceBand::Low: return "LOW";
    case ConfidenceBand::Medium: return "MEDIUM";
    case ConfidenceBand::High: return "HIGH";
  }
  return "INVALID";
}

// Namespace scope, not nested: a nested Config used as a default argument inside its
// own class does not compile under this g++ (see TrackManagerConfig for the same
// finding).
struct AutoTrackConfig {
  // §20's parameters. The document gives conceptual ranges and says the actual values
  // are commissioning/configuration; these are the middle of its examples, and §72's
  // config file overrides them.
  int64_t coast_ms = 300;              // §20: 250-400
  int64_t lost_hold_ms = 2000;         // §20: held after coast, before giving up
  int64_t reacquire_window_ms = 3000;  // §20: 2000-5000
  // §16: a couple of cycles of valid evidence before the turret commits to following.
  // One frame of a newly visible track is a hand on the lens, not a target.
  int32_t acquire_cycles = 2;
  // §19's bands on selected_target_confidence.
  float medium_min = 0.50f;
  float high_min = 0.75f;
  // §19's authority ceilings: "modest derating" and "do not accelerate aggressively".
  float medium_scale = 0.60f;
  float low_scale = 0.30f;
  // How old an estimate may be before it stops counting as evidence. One and a half
  // frames at the design rate (30 Hz), so a normally-behaving publisher is never
  // mistaken for a lost target, and a publisher that has actually stalled is.
  int64_t fresh_ms = 50;
  // §20.1: authority at the start of a coast, and the floor it decays to.
  float coast_scale_start = 0.60f;
  float coast_scale_floor = 0.20f;
};

// What the control cycle knows about the selected target. Assembled by ControlLoop from
// the TrackSet it just observed (§8/§9) and from the tracker's own view (§17).
struct AutoTrackInput {
  bool has_selection = false;          // §12: the operator chose somebody
  bool target_visible = false;         // selection visibility == VISIBLE
  bool target_occluded = false;        // §8 OCCLUDED: still a subject, not evidence
  bool ambiguous = false;              // §21: refused to guess between two candidates
  // Age of the newest measurement applied to the estimator, in milliseconds, from the
  // control clock; -1 for never. An age rather than a "fresh this cycle" flag, and the
  // reason is the frame rate: vision arrives at 30 Hz and the loop runs at 200 Hz, so a
  // per-cycle "did a measurement arrive?" is false five cycles out of six even when
  // everything is healthy. Gating COASTING on that would make a working station coast
  // and resume continuously — and the first draft of this class did exactly that, until
  // a test tried to sit in COASTING and found itself back in TRACKING.
  int64_t measurement_age_ms = -1;
  bool estimator_ready = false;        // §17: prediction exists to follow
  bool los_feasible = true;            // §22: the envelope could satisfy the request
  float detector_confidence = 0.0f;    // §19's five inputs, as §9 publishes them
  float track_confidence = 0.0f;
  int32_t visible_frames = 0;
  int32_t missing_frames = 0;
  float reacquisition_score = 0.0f;    // §21, and §78's diagnostic
  bool just_reacquired = false;
};

struct AutoTrackOutput {
  AutoTrackState state = AutoTrackState::WaitTarget;
  // The two things the caller needs: is the predicted LOS something to follow now, and
  // with how much of the configured velocity authority. Everything else is telemetry.
  bool follow_los = false;
  double velocity_scale = 0.0;
  ConfidenceBand band = ConfidenceBand::Invalid;
  float selected_confidence = 0.0f;    // §19's value, published not implied
  const char* reason = "no selection";  // §50: the operator's question, answered
};

class AutoTrackController {
 public:
  explicit AutoTrackController(AutoTrackConfig cfg = AutoTrackConfig())
      : cfg_(cfg) {}

  void set_config(AutoTrackConfig cfg) { cfg_ = cfg; }
  const AutoTrackConfig& config() const { return cfg_; }
  AutoTrackState state() const { return state_; }

  // §19's selected_target_confidence, from the five factors the document names. The
  // weighting is deliberately crude and stated in the open: continuity is what makes a
  // track trustworthy beyond a single frame's detector score, and a reacquired target
  // carries its reacquisition score as a permanent discount until it has been watched
  // long enough to earn the continuity term back. There is no association-quality term
  // separate from track_confidence, because §9 does not publish one; adding a weight for
  // a signal the wire does not carry would make this look more capable than it is.
  static float selected_confidence(const AutoTrackInput& in, float base) {
    const float continuity =
        in.visible_frames >= 10 ? 1.0f
                                : 0.35f + 0.065f * static_cast<float>(in.visible_frames);
    float c = base * continuity;
    if (in.just_reacquired) c *= in.reacquisition_score;
    if (in.ambiguous) c *= 0.5f;  // §21: still contested, so half as trustworthy
    if (c < 0.0f) c = 0.0f;
    if (c > 1.0f) c = 1.0f;
    return c;
  }

  ConfidenceBand band_for(float c) const {
    if (c >= cfg_.high_min) return ConfidenceBand::High;
    if (c >= cfg_.medium_min) return ConfidenceBand::Medium;
    // Anything that has lost a measurement outright is INVALID rather than LOW: §19's
    // INVALID means "coast/hold", which is the right response to having no evidence,
    // whereas LOW still permits careful motion toward a weakly-held target.
    if (c > 0.0f) return ConfidenceBand::Low;
    return ConfidenceBand::Invalid;
  }

  AutoTrackOutput update(const AutoTrackInput& in, TimeNs now_ns) {
    AutoTrackOutput out;
    const bool fresh = in.measurement_age_ms >= 0 &&
                       in.measurement_age_ms <= cfg_.fresh_ms;
    const bool this_cycle = in.measurement_age_ms == 0;
    out.state = state_;
    out.selected_confidence = selected_confidence(in, in.track_confidence);
    out.band = band_for(out.selected_confidence);
    out.reason = "hold";

    switch (state_) {
      case AutoTrackState::WaitTarget:
        // §16 verbatim: no selection -> WAIT_TARGET, "Select a target", no autonomous
        // motion. The reason string is what the dashboard shows, so it is written for
        // the person standing in front of it.
        out.reason = in.has_selection ? "selected target not visible"
                                      : "select a target";
        if (in.has_selection && in.target_visible) enter(AutoTrackState::Acquire, now_ns);
        break;

      case AutoTrackState::Acquire:
        if (!in.has_selection || (!in.target_visible && !in.target_occluded)) {
          // Never acquired, so there is nothing to coast toward. Back to waiting.
          enter(AutoTrackState::WaitTarget, now_ns);
          out.reason = "target gone before acquisition";
        } else if (!this_cycle || !in.estimator_ready) {
          out.reason = "acquiring: no measurement yet";
        } else if (out.band == ConfidenceBand::Invalid) {
          out.reason = "acquiring: confidence invalid";
        } else if (++cycles_in_state_ >= cfg_.acquire_cycles) {
          enter(AutoTrackState::Tracking, now_ns);
          out.reason = "acquired";
        } else {
          out.reason = "acquiring";
        }
        break;

      case AutoTrackState::Tracking:
        if (!in.los_feasible) {
          // §22. Reported by the intent converter (§67's target_unreachable), not
          // guessed here: the envelope is the only thing that knows the limits.
          enter(AutoTrackState::TargetUnreachable, now_ns);
          out.reason = "target outside the safe envelope";
        } else if (!fresh || out.band == ConfidenceBand::Invalid) {
          enter(AutoTrackState::Coasting, now_ns);
          out.reason = fresh ? "confidence invalid: coasting"
                             : "measurement missing: coasting";
        } else {
          out.reason = "tracking";
        }
        break;

      case AutoTrackState::Coasting: {
        // §20.1: prediction continues while authority shrinks with uncertainty. §20.2
        // then stops the prediction outright — the difference between the two is the
        // whole point, and it is a difference the operator should be able to hear in
        // the motor sound.
        if (fresh && out.band != ConfidenceBand::Invalid) {
          // Evidence, not the label. A track can stay CONFIRMED on the wire for a frame
          // and a half after its measurement stops arriving; calling that a returned
          // target is how the coast timer never gets to run.
          enter(AutoTrackState::Tracking, now_ns);
          out.reason = "target returned";
        } else if ((now_ns - entered_state_ns_) / 1000000 >= cfg_.coast_ms) {
          enter(AutoTrackState::LostHold, now_ns);
          out.reason = "coast timeout: holding";
        } else {
          out.reason = "coasting";
        }
        break;
      }

      case AutoTrackState::LostHold:
        // §20.2: stop predicting motion into the future, brake to a hold, and keep
        // vision running. Nothing here touches the selection: losing sight of somebody
        // does not unchoose them (§12).
        out.reason = "lost: holding";
        if (in.ambiguous) {
          out.reason = "ambiguous reacquisition: reselect (21)";
        } else if (in.has_selection && in.target_visible && fresh &&
                   out.band != ConfidenceBand::Invalid) {
          // `fresh` is load-bearing here. A track can still be published CONFIRMED —
          // and so still be "visible" — while its measurement is not reaching the
          // estimator (a malformed frame, tracking off, a publisher mid-stall);
          // granting REACQUIRE, and therefore motion, on that combination would restart
          // the turret on evidence that does not exist (§20.3 requires it to be
          // *confidently* reacquired).
          enter(AutoTrackState::Reacquire, now_ns);  // §20.3: no operator click needed
          out.reason = "reacquiring";
        } else if (lost_ms(now_ns) >= cfg_.lost_hold_ms && !in.has_selection) {
          enter(AutoTrackState::WaitTarget, now_ns);
          out.reason = "select a target";
        }
        break;

      case AutoTrackState::Reacquire:
        if (!fresh) {
          // The evidence went away again mid-handover. Back to the hold, immediately —
          // not after another timeout, because there is nothing to wait for.
          enter(AutoTrackState::LostHold, now_ns);
          out.reason = "reacquisition lost again: holding";
          break;
        }
        // One state of its own rather than jumping straight to TRACKING, so the
        // operator sees the handover and the §50 telemetry records that motion resumed
        // on its own rather than on a click.
        if (!this_cycle || !in.estimator_ready ||
            out.band == ConfidenceBand::Invalid) {
          out.reason = "reacquiring: no measurement yet";
          if (lost_ms(now_ns) >= cfg_.lost_hold_ms) {
            enter(AutoTrackState::LostHold, now_ns);
            out.reason = "reacquisition failed: holding";
          }
        } else if (++cycles_in_state_ >= cfg_.acquire_cycles) {
          enter(AutoTrackState::Tracking, now_ns);
          out.reason = "reacquired";
        } else {
          out.reason = "reacquiring";
        }
        break;

      case AutoTrackState::TargetUnreachable:
        out.reason = "target outside the safe envelope (22)";
        if (in.los_feasible && fresh) {
          enter(AutoTrackState::Tracking, now_ns);
          out.reason = "target back inside the envelope";
        } else if (!in.has_selection || (!in.target_visible && !in.target_occluded)) {
          enter(AutoTrackState::WaitTarget, now_ns);
          out.reason = "selected target not visible";
        }
        break;
    }

    out.state = state_;
    if (state_ == AutoTrackState::Coasting) {
      // §20.1, computed here rather than in the case above so that the cycle which
      // *entered* coasting coasts as well. Setting the authority only on later cycles
      // put a full stop into the middle of the first missed frame — the opposite of
      // what a coast is for, and audible as a hiccup.
      const int64_t coasted_ms = now_ns > entered_state_ns_
                                     ? (now_ns - entered_state_ns_) / 1000000
                                     : 0;
      const double frac = cfg_.coast_ms > 0
                              ? double(coasted_ms) / double(cfg_.coast_ms)
                              : 1.0;
      out.velocity_scale =
          cfg_.coast_scale_start -
          (cfg_.coast_scale_start - cfg_.coast_scale_floor) * std::min(1.0, frac);
      out.follow_los = in.estimator_ready;
    }
    if (state_ == AutoTrackState::Tracking || state_ == AutoTrackState::Reacquire) {
      out.follow_los = in.estimator_ready;
      out.velocity_scale = band_scale(out.band);
    } else if (state_ == AutoTrackState::Acquire) {
      // ACQUIRE points the turret at the target without the authority to swing at it:
      // §16's "immediately enter ACQUIRE -> TRACKING" is about starting, not about
      // arriving at speed. A target that turns out to be a misassociation should cost a
      // nudge, not a sweep across the room.
      out.follow_los = in.estimator_ready;
      out.velocity_scale = std::min(band_scale(out.band), double(cfg_.low_scale));
    }
    return out;
  }

  // Called when the mode leaves AUTO_TRACK, and when a selection changes: the state
  // machine has no business carrying an acquisition history from one target to another.
  void reset(AutoTrackState to = AutoTrackState::WaitTarget) {
    state_ = to;
    entered_state_ns_ = 0;
    cycles_in_state_ = 0;
  }

 private:
  int64_t lost_ms(TimeNs now_ns) const {
    return entered_state_ns_ > 0 ? (now_ns - entered_state_ns_) / 1000000 : 0;
  }

  void enter(AutoTrackState s, TimeNs now_ns) {
    if (s == state_) return;
    state_ = s;
    entered_state_ns_ = now_ns;
    cycles_in_state_ = 0;
  }

  double band_scale(ConfidenceBand b) const {
    switch (b) {
      case ConfidenceBand::High: return 1.0;
      case ConfidenceBand::Medium: return cfg_.medium_scale;
      case ConfidenceBand::Low: return cfg_.low_scale;
      case ConfidenceBand::Invalid: return 0.0;
    }
    return 0.0;
  }

  AutoTrackConfig cfg_;
  AutoTrackState state_ = AutoTrackState::WaitTarget;
  TimeNs entered_state_ns_ = 0;
  int32_t cycles_in_state_ = 0;
};

}  // namespace ota
