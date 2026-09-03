// OpenAutoTurret v3 — ModeManager (§43, §44).
//
// Owns which of MANUAL / AUTO_TRACK / AUTO_ROAM is authoritative, and — the part
// that actually matters — *refuses* transitions that are not allowed, with a
// reason a human can read (§52: "Do not silently ignore commands").
//
// What it deliberately does NOT do:
//
//   * It produces no motion. Mode change is a change of *authority*; the motion
//     still comes from the mode's controller through MotionIntent into the v1
//     reference / safety / trajectory path. §71 is explicit that a transition
//     must not reset trajectory state, current acceleration or braking state,
//     so nothing in here touches them either.
//   * It does not clear target selection. §12 and §44 both say selection is
//     persistent and mode-independent; TargetSelectionManager owns it, and the
//     test that a mode switch leaves it alone lives there.
//   * It does not decide whether the turret may move. Safety and the supervisory
//     states (BOOT / UNHOMED / HOMING / CALIBRATING / PARKING / FAULT) sit above
//     operating modes (§2) and can override any of them; this class only refuses
//     requests it can see to be unsafe, and reports the override it was told about.
//
// Stale-intent invalidation (§43) is the other thing that lives here in spirit:
// every transition bumps `epoch()`. A controller stamps the epoch it observed
// when it forms an intent, and the loop discards an intent whose epoch is no
// longer current. That makes "no stale mode intent survives a transition" (§93)
// a property of the data rather than of every consumer remembering to reset a
// flag — v1's mode-ish behaviour had no equivalent, which is how a superseded
// search reference could outlive the command that started it.
//
// Pure: no CAN, no camera, no time source (the caller supplies facts, not reads).
#pragma once

#include <cstdint>
#include <ostream>

#include "control/motion_intent.hpp"

namespace ota {

// The three primary operator modes (§2). HOLD / JOG / TRACKING / COASTING /
// LOST_HOLD / SWEEP ... are phases *inside* these, not peers.
enum class OperatingMode : uint8_t {
  Manual,
  AutoTrack,
  AutoRoam,
};

inline const char* operating_mode_name(OperatingMode m) {
  switch (m) {
    case OperatingMode::Manual:    return "MANUAL";
    case OperatingMode::AutoTrack: return "AUTO_TRACK";
    case OperatingMode::AutoRoam:  return "AUTO_ROAM";
  }
  return "?";
}

// Parse a mode name from a web command. Case-insensitive on the documented
// spellings. Returns false (rather than guessing) for anything else — a mode
// command that silently fell back to MANUAL would be a safety surprise.
inline bool operating_mode_from_name(const char* name, OperatingMode& out) {
  auto eq = [](const char* a, const char* b) {
    int i = 0;
    for (;; ++i) {
      char ca = a[i], cb = b[i];
      if (ca >= 'a' && ca <= 'z') ca -= 32;
      if (cb >= 'a' && cb <= 'z') cb -= 32;
      if (ca != cb) return false;
      if (ca == '\0') return true;
    }
  };
  if (name == nullptr) return false;
  if (eq(name, "MANUAL") || eq(name, "MAN")) { out = OperatingMode::Manual; return true; }
  if (eq(name, "AUTO_TRACK") || eq(name, "AUTOTRACK") || eq(name, "TRACK")) {
    out = OperatingMode::AutoTrack; return true;
  }
  if (eq(name, "AUTO_ROAM") || eq(name, "AUTOROAM") || eq(name, "ROAM")) {
    out = OperatingMode::AutoRoam; return true;
  }
  return false;
}

// The states that sit above operating modes (§2). Mirrors the loop's phases the
// operator can see; the loop is the only writer.
enum class SupervisoryState : uint8_t {
  Ready,      // homed and able to obey a mode
  Boot,
  Unhomed,
  Homing,
  Calibrating,
  Parking,
  Fault,
};

inline const char* supervisory_state_name(SupervisoryState s) {
  switch (s) {
    case SupervisoryState::Ready:       return "READY";
    case SupervisoryState::Boot:        return "BOOT";
    case SupervisoryState::Unhomed:     return "UNHOMED";
    case SupervisoryState::Homing:      return "HOMING";
    case SupervisoryState::Calibrating: return "CALIBRATING";
    case SupervisoryState::Parking:     return "PARKING";
    case SupervisoryState::Fault:       return "FAULT";
  }
  return "?";
}

// The facts a transition request is judged on. The loop fills these from its own
// authoritative state; ModeManager never reaches for them (§84 needs the gate to
// be a pure function of its inputs so every transition is unit-testable).
struct ModeRequestContext {
  // Feedback is fresh and the axes are homed: we know where the turret is.
  bool position_valid = false;
  // No blocking safety fault (§38 decision is ALLOW).
  bool safety_healthy = false;
  // The configured roam envelope is non-empty and inside the safe envelope.
  // Only consulted for AUTO_ROAM, and only ever to refuse: an invalid envelope
  // must stop the mode from starting, not quietly widen it (§32, §69).
  bool roam_envelope_valid = false;
  SupervisoryState supervisory = SupervisoryState::Ready;
};

struct ModeResult {
  bool ok = false;
  bool changed = false;        // false for an accepted no-op (already in mode)
  const char* reason = "";     // human-readable either way; never empty
};

class ModeManager {
 public:
  OperatingMode mode() const { return mode_; }
  SupervisoryState supervisory() const { return supervisory_; }
  uint64_t epoch() const { return epoch_; }
  // True while a supervisory state outranks the operating mode (§26). The mode
  // is still reported for the UI, but no mode controller may own motion.
  bool mode_is_overridden() const { return supervisory_ != SupervisoryState::Ready; }

  // Request a mode change. Never partial: either the mode changes, or nothing
  // does and the reason says why (§52).
  ModeResult request(OperatingMode target, const ModeRequestContext& ctx) {
    ModeResult r;
    if (target == mode_) {
      r.ok = true;
      r.changed = false;
      r.reason = "already in mode";
      return r;
    }
    // §2/§26: while something outranks the modes, a mode request cannot take
    // effect. Saying "not now, HOMING" is the honest answer; queueing the
    // request so the turret changes mode under someone halfway through a homing
    // run is not.
    if (ctx.supervisory != SupervisoryState::Ready) {
      r.reason = refusal_for_supervisory(ctx.supervisory);
      return r;
    }
    // Entering MANUAL is never gated (except by a supervisory state, which owns
    // the motors outright and says so below). §44 lists preconditions only for
    // the transitions INTO autonomy, and the asymmetry is the point: MANUAL is
    // where the operator takes the stick back, so refusing it because a fault
    // appeared would trap them inside the autonomous mode that may have caused
    // it. Note that allowing the *mode* is not allowing *motion* — a jog still
    // has to survive its own validation (lease, position validity, envelope),
    // and STOP MOTION below is unconditional in any case.
    if (target != OperatingMode::Manual) {
      if (!ctx.safety_healthy) {
        r.reason = "rejected: safety fault active";
        return r;
      }
      if (!ctx.position_valid) {
        r.reason = "rejected: turret not homed or position invalid";
        return r;
      }
    }
    if (target == OperatingMode::AutoRoam && !ctx.roam_envelope_valid) {
      r.reason = "rejected: AUTO_ROAM envelope invalid";
      return r;
    }

    // §44 permits all six pairwise transitions once the destination's
    // preconditions hold. AUTO_TRACK does not require a visible or even a
    // selected target: it holds in WAIT_TARGET until one exists (§16), which is
    // why no target facts appear in this gate at all.
    const OperatingMode previous = mode_;
    mode_ = target;
    ++epoch_;
    changed_from_ = previous;
    r.ok = true;
    r.changed = true;
    r.reason = "accepted";
    return r;
  }

  // §27 STOP MOTION: cancel the active intent and land in MANUAL/HOLD. The mode
  // change is what prevents an autonomous mode from re-issuing motion on the
  // next cycle, which is the whole point of it not merely being a brake.
  // It is *not* motor disable, not power removal, not shutdown — those are other
  // paths with their own states, and conflating them here would be a mistake a
  // frightened operator makes at exactly the wrong moment.
  ModeResult stop_motion(const ModeRequestContext& ctx) {
    ModeResult r;
    const bool was_manual = (mode_ == OperatingMode::Manual);
    mode_ = OperatingMode::Manual;
    ++epoch_;  // always bump: any in-flight intent is now stale, manual or not
    changed_from_ = mode_;
    r.ok = true;
    r.changed = !was_manual;
    r.reason = "STOP MOTION: manual hold";
    (void)ctx;  // accepted unconditionally — see below.
    return r;
  }

  // Loop reports supervisory changes (§84: "return from homing to safe
  // MANUAL/HOLD"). Landing in MANUAL after a supervisory sequence is deliberate:
  // resuming AUTO_ROAM because "that is what we were doing before homing" would
  // start sweeping at the end of a power cycle with nobody asking for it. FAULT
  // additionally drops the mode immediately, since a fault can be the reason the
  // last mode was unsafe.
  void notify_supervisory(SupervisoryState state) {
    if (state == supervisory_) return;
    supervisory_ = state;
    if (state == SupervisoryState::Fault || state == SupervisoryState::Ready) {
      if (mode_ != OperatingMode::Manual) {
        changed_from_ = mode_;
        mode_ = OperatingMode::Manual;
        ++epoch_;
      }
    }
  }

 private:
  static const char* refusal_for_supervisory(SupervisoryState s) {
    switch (s) {
      case SupervisoryState::Homing:      return "rejected: homing in progress";
      case SupervisoryState::Parking:     return "rejected: parking in progress";
      case SupervisoryState::Calibrating: return "rejected: calibration in progress";
      case SupervisoryState::Fault:       return "rejected: fault state";
      case SupervisoryState::Unhomed:     return "rejected: turret not homed";
      default:                            return "rejected: supervisory state active";
    }
  }

  OperatingMode mode_ = OperatingMode::Manual;  // §73/Appendix E: boot ends in MANUAL/HOLD
  SupervisoryState supervisory_ = SupervisoryState::Ready;
  OperatingMode changed_from_ = OperatingMode::Manual;
  uint64_t epoch_ = 1;
};

// Streamable by name, not as an integer. The enum is operator-facing vocabulary
// — it lands in logs, gtest failures and the dashboard — and a fault log that
// says "mode 2" instead of "AUTO_ROAM" is a log somebody has to go and decode
// while the turret is moving.
inline std::ostream& operator<<(std::ostream& os, OperatingMode m) {
  return os << operating_mode_name(m);
}
inline std::ostream& operator<<(std::ostream& os, SupervisoryState s) {
  return os << supervisory_state_name(s);
}

}  // namespace ota
