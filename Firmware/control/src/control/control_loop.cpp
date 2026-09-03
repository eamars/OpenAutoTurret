// OpenAutoTurret — ControlLoop implementation (§46 per-cycle engine).
#include "control/control_loop.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "vision/vision_ingest.hpp"

namespace ota {

namespace {
// At-rest threshold for the fault-stop "keep stopping, then hold" logic.
constexpr double kAtRestVelRadS = 0.05;
// Position tolerance for "at the ready pose".
constexpr double kReadyPosTolRad = 0.01;
}  // namespace

ControlLoop::ControlLoop(Config cfg, std::unique_ptr<MotorBackend> backend)
    : cfg_(std::move(cfg)), backend_(std::move(backend)) {
  SupervisorParams sp;
  sp.feedback_max_age_ms = cfg_.feedback_max_age_ms;
  sp.deadline_max_us = cfg_.deadline_max_us;
  sp.deadline_miss_threshold = cfg_.deadline_miss_threshold;
  sp.motor_overtemp_c = cfg_.motor_overtemp_c;
  sp.a_brake_rad_s2 = cfg_.a_brake_rad_s2;
  sp.j_brake_rad_s3 = cfg_.j_brake_rad_s3;
  sp.stop_margin_rad = cfg_.stop_margin_rad;
  supervisor_ = SafetySupervisor(sp);

  SafetyEnvelopeParams ep;
  ep.a_brake_rad_s2 = cfg_.a_brake_rad_s2;
  ep.j_brake_rad_s3 = cfg_.j_brake_rad_s3;
  ep.margin_rad = cfg_.stop_margin_rad;
  ep.v_max_rad_s = cfg_.hold_speed_rad_s;
  env_ = SafetyEnvelope(ep);

  deadline_ns_ = 1000000000LL / std::max(1, cfg_.control_hz);
}

bool ControlLoop::enter_position_mode_all(double limit_spd, std::string& err) {
  for (int i = 0; i < kAxisCount; ++i) {
    const AxisId a = static_cast<AxisId>(i);
    std::string e;
    if (!backend_->enter_position_mode(a, limit_spd, e)) {
      err = std::string(axis_name(a)) + ": " + e;
      return false;
    }
  }
  return true;
}

bool ControlLoop::enter_speed_mode_all(
    const double limit_cur_a[kAxisCount], std::string& err) {
  for (int i = 0; i < kAxisCount; ++i) {
    const AxisId a = static_cast<AxisId>(i);
    std::string e;
    if (!backend_->enter_speed_mode(a, limit_cur_a[i], e)) {
      err = std::string(axis_name(a)) + ": " + e;
      return false;
    }
  }
  return true;
}

bool ControlLoop::start_homing(HomingPlan plan, std::string& err) {
  if (phase_ == Phase::Fault) {
    err = "in fault; reset required";
    return false;
  }
  homing_.reset(new HomingPlan(std::move(plan)));
  // Speed mode (velocity) for homing: the drive's own velocity loop holds the
  // constant approach speed (SpdRef) — the smooth source of motion. This is
  // the root-cause fix for the position-mode stick-slip (P0o): a host-regenerated
  // moving LocRef at 200 Hz executes as stick-slip on this CyberGear, while a
  // constant SpdRef is smooth. The per-axis homing current (pitch 3 A / yaw
  // 1 A) is carried by the HomingController and applied on its first cycle
  // (set_current_limit); here we enter speed mode with a hold default so the
  // non-active axis has a sane current until the active axis is switched.
  double limit_cur[kAxisCount];
  for (int i = 0; i < kAxisCount; ++i) limit_cur[i] = 5.0;
  std::string e;
  if (!enter_speed_mode_all(limit_cur, e)) {
    err = e;
    return false;
  }
  homed_ = false;
  at_ready_ = false;
  homing_log_cycle_ = 0;
  phase_ = Phase::Homing;
  return true;
}

bool ControlLoop::start_hold(std::string& err) {
  if (!homed_) {
    err = "not homed";
    return false;
  }
  std::string e;
  if (!enter_position_mode_all(cfg_.hold_speed_rad_s, e)) {
    err = e;
    return false;
  }
  phase_ = Phase::Hold;
  return true;
}

bool ControlLoop::start_parking(std::string& err) {
  if (!homed_) {
    err = "cannot park: not homed (position validity unknown, §38.1)";
    return false;
  }
  park_.reset(new ParkController(cfg_.park, limits_, models_));
  if (park_->failed()) {
    err = park_->fail_reason();
    fault(err);
    return false;
  }
  // The park MOVES run in speed mode (SpdRef, per-axis current limit from
  // config — pitch 3 A / yaw 1 A, under the 10 A cap): the drive's velocity
  // loop is the strong, smooth motion source (P0o). The old position-mode
  // park move crawled ~0.07 deg/s against gravity + friction and never
  // landed at the real station (rehome4: full 40 s park timeout, yaw
  // de-energized 1.4 deg short of the 180 deg target; rehome1: 3.96 deg
  // short). Position mode is entered once, at the ParkController's Verify
  // state, for the §33.2 target-hold (executor, Phase::Parking).
  std::string e;
  if (!enter_speed_mode_all(cfg_.park.limit_cur_a.data(), e)) {
    err = e;
    return false;
  }
  park_pos_mode_entered_ = false;
  phase_ = Phase::Parking;
  return true;
}

void ControlLoop::deenergize_all() {
  for (int i = 0; i < kAxisCount; ++i) backend_->deenergize(static_cast<AxisId>(i));
}

bool ControlLoop::enable_tracking(const TrackingController::Config& cfg_in,
                                  std::string& err) {
  if (tracking_) {
    err = "tracking already enabled";
    return false;
  }
  if (!homed_) {
    err = "cannot enable tracking: not homed (position validity unknown, §38.1)";
    return false;
  }
  // §36/§49: the SearchPlanner requires its yaw bounds to be STRICTLY inside
  // the tracking soft limits (it has no envelope of its own — the envelope
  // brakes, it does not veto). Center the configured span on the safe ready
  // pose and take only what fits inside the soft limits with the soft margin
  // AND the envelope's stop margin on top. A narrow-travel station therefore
  // gets a narrow sweep, never a sweep that reaches for a stop.
  TrackingController::Config cfg = cfg_in;
  const AxisLimits& yl = limits_[ix(AxisId::Yaw)];
  const double inset = cfg_.soft_margin_rad + cfg_.stop_margin_rad;
  const double ready_yaw = ready_raw_[ix(AxisId::Yaw)];
  double lo = std::max(ready_yaw - cfg_.search_span_rad, yl.q_soft_min_rad + inset);
  double hi = std::min(ready_yaw + cfg_.search_span_rad, yl.q_soft_max_rad - inset);
  if (!(hi > lo)) {  // degenerate: no room to sweep — hold the ready yaw
    spdlog::warn("search sweep clamped to the ready pose (no yaw room inside "
                 "the soft limits)");
    lo = hi = ready_yaw;
  } else if (hi - lo < 2.0 * cfg_.search_span_rad - 1e-9) {
    spdlog::info("search sweep clamped to [{:.1f}, {:.1f}] deg (logical) to stay "
                 "inside the soft limits",
                 models_[ix(AxisId::Yaw)].raw_to_logical_deg(lo),
                 models_[ix(AxisId::Yaw)].raw_to_logical_deg(hi));
  }
  // §36 runtime opt-in. Until now the ONLY way to enable the sweep was a config
  // edit plus a daemon restart, while `enable_search` existed, was validated and
  // answered ok:true having changed nothing — an operator who pressed it got a
  // confirmation for an action that never happened, which is worse than no
  // button. The operator's word outranks turret.yaml in both directions: armed
  // when the config says off, and disarmed when config (or
  // target_lost_behavior=search) says on. The line below prints the EFFECTIVE
  // value, so the log records which one won.
  if (search_override_.has_value()) cfg.fsm.search_enabled = *search_override_;
  // The intent converter needs the same kinematics the tracker solves against —
  // two different R_P_C estimates in one loop would make the tracking reference
  // and the mode reference disagree about where the camera is pointing.
  ref_mgr_.emplace(geo::LosJointSolver(cfg.kinematics));
  cfg.search.yaw_low_rad = lo;
  cfg.search.yaw_high_rad = hi;
  cfg.search.pitch_rad = ready_raw_[ix(AxisId::Pitch)];  // the safe elevation
  cfg.search.v_max_rad_s = cfg.search_v_max_rad_s;
  tracking_.reset(new TrackingController(cfg));
  spdlog::info(
      "tracking ENABLED (v_max track={:.1f} deg/s search={:.1f} deg/s, "
      "search_enabled={}, fresh={} ms, intrinsics fx={:.1f} fy={:.1f} "
      "cx={:.1f} cy={:.1f} {}x{})",
      cfg.track_v_max_rad_s / kDeg2Rad, cfg.search_v_max_rad_s / kDeg2Rad,
      cfg.fsm.search_enabled ? "yes" : "no", cfg.fresh_threshold_ns / 1000000,
      cfg.intrinsics.fx, cfg.intrinsics.fy, cfg.intrinsics.cx,
      cfg.intrinsics.cy, cfg.intrinsics.width, cfg.intrinsics.height);
  return true;
}

void ControlLoop::feed_measurement(const vision::TargetMeasurement& m) {
  // Called from the vision-ingest thread (§6.1). The critical section is a
  // 58-byte copy — the control thread holds this mutex once per cycle (1b).
  // `tracking_` is deliberately NOT touched here: it is control-thread state.
  // A measurement delivered while tracking is off is discarded by the
  // consumer, so the "no-op unless tracking enabled" contract still holds.
  std::lock_guard<std::mutex> lk(measurement_mutex_);
  pending_measurement_ = m;
  has_pending_measurement_ = true;
}

bool ControlLoop::finalize_homing() {
  for (int i = 0; i < kAxisCount; ++i) {
    const AxisId a = static_cast<AxisId>(i);
    if (!homing_->axis_homed(a)) return false;
    models_[i] = homing_->model(a);
    limits_[i].set_from_endpoints(homing_->raw_low(a), homing_->raw_high(a),
                                  cfg_.soft_margin_rad);
    // Safe ready pose = midpoint of the measured travel (never at a stop).
    const double ll = models_[i].raw_to_logical_deg(homing_->raw_low(a));
    const double lh = models_[i].raw_to_logical_deg(homing_->raw_high(a));
    ready_raw_[i] = models_[i].logical_to_raw_rad(0.5 * (ll + lh));
  }
  homed_ = true;
  at_ready_ = false;
  return true;
}

HomingFeedback ControlLoop::to_feedback(const AxisSnapshot& s, double vel_rad_s) {
  HomingFeedback fb;
  fb.t_ns = s.rx_ns;
  fb.pos_rad = s.q_rad;
  fb.vel_rad_s = vel_rad_s;
  fb.torque_nm = s.torque_nm;
  fb.motor_fault = (s.faults != 0);
  return fb;
}

Phase ControlLoop::step(TimeNs now_ns, TimeNs period_ns) {
  now_ns_ = now_ns;
  // Commands are executed inside the cycle and need a timestamp to record a selection
  // against (§13's selection_timestamp). It is this cycle's clock, not the instant the
  // operator clicked: a selection made between two cycles takes effect at the next one,
  // and saying so with the clock it actually happened on is what makes the log
  // reconcile with the turret's behaviour later.
  // 1. Non-blocking snapshots.
  AxisSnapshot sp[kAxisCount];
  for (int i = 0; i < kAxisCount; ++i) {
    sp[i] = backend_->snapshot(static_cast<AxisId>(i), now_ns);
    last_q_[i] = sp[i].q_rad;  // for telemetry
    last_temp_[i] = sp[i].temp_c;  // for telemetry (drive NTC, degC)
    // Position-derived velocity (see header): refresh only when fresh
    // feedback arrives so the 200 Hz loop does not average in zeros.
    if (sp[i].has_feedback && sp[i].rx_ns > v_est_t_prev_[i]) {
      if (v_est_t_prev_[i] != 0) {
        const double dt = (sp[i].rx_ns - v_est_t_prev_[i]) / 1e9;
        if (dt >= 1e-3) {
          const double v_inst = (sp[i].q_rad - v_est_q_prev_[i]) / dt;
          const double alpha = dt / (kVestTauS + dt);
          const double v_prev = v_est_[i];
          v_est_[i] += alpha * (v_inst - v_est_[i]);
          // Acceleration and jerk for telemetry capture (C1, A.1): filtered
          // derivatives of the position-derived velocity. The drive's own
          // velocity is a +/-0.05 rad/s noise band at rest and is never used.
          const double a_inst = (v_est_[i] - v_prev) / dt;
          const double alpha_a = dt / (kATauS + dt);
          const double a_prev = a_est_[i];
          a_est_[i] += alpha_a * (a_inst - a_est_[i]);
          const double j_inst = (a_est_[i] - a_prev) / dt;
          const double alpha_j = dt / (kJTauS + dt);
          jerk_est_[i] += alpha_j * (j_inst - jerk_est_[i]);
        }
      }
      v_est_t_prev_[i] = sp[i].rx_ns;
      v_est_q_prev_[i] = sp[i].q_rad;
    }
  }

  // 1a. Phase 8: execute any developer commands submitted via the web UI
  //     (§42.2). They were validated against the previous cycle's state on the
  //     web thread; execution happens HERE on the control thread so all state
  //     mutation stays single-threaded.
  process_commands();

  // 1b. Phase 6: feed the tracking controller the current pose (for §11
  //     timestamp interpolation) and consume any pending visiond measurement.
  //
  //     Auto-enable (§38.1): tracking is HARD-disabled until the homing gates
  //     pass — this branch is unreachable before homed_ is set by
  //     finalize_homing(), and enable_tracking() re-checks it.
  if (tracking_auto_enable_ && !tracking_ && homed_) {
    std::string terr;
    if (!enable_tracking(tracking_cfg_, terr))
      spdlog::warn("auto-enable tracking failed: {}", terr);
  }
  if (tracking_) {
    tracking_->update_snapshots(now_ns, sp[ix(AxisId::Pitch)].q_rad,
                                sp[ix(AxisId::Yaw)].q_rad);
  }
  {
    vision::TargetMeasurement m;
    bool have = false;
    tracks::TrackSet set;
    bool have_set = false;
    {
      std::lock_guard<std::mutex> lk(measurement_mutex_);
      if (has_pending_set_) {
        set = pending_set_;
        has_pending_set_ = false;
        have_set = true;
      } else if (has_pending_measurement_) {
        m = pending_measurement_;
        has_pending_measurement_ = false;
        have = true;
      }
    }
    // A TrackSet wins when both arrived, which is the steady state for a v3 publisher:
    // the legacy measurement is the same frame's selected target as visiond chose it,
    // and visiond no longer chooses (§59). During an upgrade window this order is also
    // what makes controld-first safe — a v1 publisher never sets has_pending_set_, so
    // nothing changes for it.
    if (have_set) {
      // Observed whether or not a tracking session is running. The first version gated
      // this on tracking_, and every selection attempt then answered "no vision data
      // has reached controld yet" while TrackSets were arriving by the hundred: §12/§14
      // let the operator choose a target before anything is following anything, so the
      // observer cannot be downstream of the tracker.
      const tracks::Track* followed = apply_track_set(set, now_ns);
      // §78: which candidate is actually being followed, published rather than
      // inferred. Zero while tracking is off is not a contradiction with the selection
      // fields beside it — those say what the operator chose, this says what the
      // turret is acting on, and the two legitimately differ in MANUAL.
      // The identity the machine is *acting on*, which is the operator's selection — not
      // whatever the estimator happened to be primed with. They differ, legitimately:
      // with no selection the estimator is still fed (vision keeps being processed, so
      // a later selection can be acted on immediately), and reporting its pick here
      // would tell the operator the turret was following somebody it was not.
      const tracks::Track* acting = selection_.selected_track();
      selected_track_id_ =
          (tracking_ && acting != nullptr) ? acting->uuid.lo : 0;
    } else if (have && tracking_) {
      tracking_->set_measurement(m);
      selected_track_id_ = m.has_track_id ? m.visual_track_id : 0;
    } else if (!tracking_) {
      selected_track_id_ = 0;
    }
  }

  // 2. §39.3 deadline watchdog. A cycle is a deadline miss only when its
  //    overrun past the control period exceeds the configured grace
  //    (deadline_max_us, default 2 ms) — the same threshold the supervisor
  //    uses for its single-overrun Derate. A loop that runs a few tens of µs
  //    over its period (ordinary host scheduling jitter; sleep-for wakeup
  //    latency) is tolerated: such a cycle is not a miss and clears the
  //    streak, so only (near-)consecutive true misses can escalate to Hold.
  //    A persistently overloaded loop (every cycle over the grace) still
  //    accumulates misses and Holds.
  //    (History: counting every over-period cycle as a miss made a 198 Hz
  //    loop Hold all axes within 5 cycles — the P0 no-motion root cause.
  //    The first fix reset only on strictly on-time cycles, but on a host
  //    where the period is always a hair over the deadline that branch never
  //    executes, so five sporadic >2 ms spikes latched a permanent Hold —
  //    the P0f root cause. Reset on any sub-grace cycle instead.)
  int64_t overrun_us = 0;
  if (period_ns > deadline_ns_) {
    overrun_us = (period_ns - deadline_ns_) / 1000;
  }
  if (overrun_us > static_cast<int64_t>(cfg_.deadline_max_us)) {
    ++deadline_miss_count_;
  } else {
    deadline_miss_count_ = 0;
  }

  // 2a2. v3 §2: the supervisory states outrank every operating mode. Derived from
  //      the phase each cycle rather than hooked at each phase assignment, so a
  //      phase added later cannot forget to say so and leave a mode thinking it is
  //      in charge during a homing run.
  // Leaving Ready means the station owes the post-homing duty again (reach the ready
  // pose, §27 verification at rest, then READY_HOLD), so "hold" reverts to its v1
  // meaning until a mode moves it somewhere else.
  if (mode_mgr_.supervisory() != SupervisoryState::Ready) mode_has_moved_ = false;
  mode_mgr_.notify_supervisory(phase_ == Phase::Homing    ? SupervisoryState::Homing
                               : phase_ == Phase::Parking ? SupervisoryState::Parking
                               : phase_ == Phase::Fault   ? SupervisoryState::Fault
                               : (phase_ == Phase::Idle || phase_ == Phase::Parked)
                                   ? SupervisoryState::Unhomed
                                   : SupervisoryState::Ready);

  // 2b. v3 §53: exactly one mode owns motion. The mode's controller *proposes*
  //     (the v1 TrackingController still produces the numbers: its FSM,
  //     estimator and search planner are what V3-4 / V3-6 replace); the mode
  //     decides whether that proposal is honoured; the ReferenceManager converts
  //     the surviving intent into a joint reference. Everything after this point —
  //     envelope, trajectory, adapter — is unchanged v1 (§111.18).
  tracking_ref_ = ReferenceRequest{};
  mode_proposal_ = ReferenceRequest{};
  last_intent_ = MotionIntent{};
  if (phase_ == Phase::Hold) {
    // §25/§53: the intent path belongs to the modes, not to the tracking session. It
    // used to be gated on `tracking_`, which was harmless while AUTO_TRACK was the only
    // mode that could move anything — and it is why MANUAL motion did nothing at all the
    // first time it was wired up: MANUAL releases the tracking session, so the block that
    // builds the intent never ran, and the turret sat in MANUAL acknowledging jog and
    // step commands at 200 Hz with an intent source of "none". Nothing in the log. The
    // converter is created here for exactly that reason: a jog needs the reference
    // layers, and it does not need a camera.
    // The kinematics come from the tracking configuration, which is the only place
    // they are configured. MANUAL motion needs the *converter*, not the solver: a jog is
    // a joint reference and never touches the camera, so an unset calibration cannot make
    // a jog unsafe here — it would only make a LosDirection intent wrong, and MANUAL
    // does not form one.
    if (!ref_mgr_) ref_mgr_.emplace(geo::LosJointSolver(tracking_cfg_.kinematics));
    if (tracking_) {
      mode_proposal_ = tracking_->compute_reference(
          now_ns, ready_raw_[ix(AxisId::Pitch)], ready_raw_[ix(AxisId::Yaw)]);
    // §28.5/§31.3: the payload profile v_max (and any mismatch derate) caps
    // station motion. Applied to the proposal here and again to the intent's
    // ceiling in intent_limits(); the double application is deliberate — the cap
    // is applied *before* the confidence derate so a degrading target cannot lift
    // the effective ceiling back up, which is the safer ordering and slightly
    // stricter than v1's.
      mode_proposal_.v_max_rad_s =
          std::min(mode_proposal_.v_max_rad_s, hold_speed_effective());
    }
    // §15-§20. AUTO_TRACK's state advances every control cycle (its coast is wall
    // clock), from facts that only change when a frame arrives. Every other mode resets
    // it, so coming back to AUTO_TRACK never resumes an acquisition history that
    // belongs to a different moment — §93's stale-intent class of bug, seen from the
    // target's side.
    if (mode_mgr_.mode() == OperatingMode::Manual) {
      // §38: the lease is checked on the cycle clock. A browser that stopped talking is
      // noticed within one cycle (5 ms), not on the next request that never arrives.
      //
      // "Current logical q" is the reference being followed, not the measured feedback.
      // A jog integrated against lagging feedback would drift by the lag every cycle;
      // against the reference it advances by exactly what was asked, and the difference
      // between the two is what an operator would feel as a turret that keeps creeping.
      // The *measured* joint positions, not the reference being followed. Section 2b
      // clears tracking_ref_ at the top of every cycle, so reading it here returned
      // 0.0 — and a jog integrates its moving waypoint from "where the turret is", so a
      // zero base pinned the waypoint one horizon step from joint zero. Measured: a jog
      // commanded for two seconds moved the turret 2.0 degrees and stopped, with the
      // intent looking perfectly healthy the whole time, and a test that only asked
      // "did it move at all" passed.
      double q_logical[2] = {sp[ix(AxisId::Pitch)].q_rad, sp[ix(AxisId::Yaw)].q_rad};
      const double vmax = hold_speed_effective();
      const double v_max[2] = {vmax, vmax};
      const bool was_leased = manual_.lease_active();
      const bool was_expired = std::string(manual_out_.reason) == "jog lease expired";
      manual_out_ = manual_.update(q_logical, v_max, now_ns, period_ns);
      if (was_leased && !manual_.lease_active() && !was_expired) {
        // §79's MANUAL_JOG_EXPIRED, as a log line until the event bus lands. This is the
        // one MANUAL event that is not the operator's deliberate act, so it is the one
        // worth recording: it is what a dropped wifi, a backgrounded tab, or a laptop
        // going to sleep looks like from the turret's side, and "the jog stopped by
        // itself" is the report that needs a timestamp.
        spdlog::warn("MANUAL jog lease expired after {} ms — controlled stop to "
                     "MANUAL/HOLD (38)",
                     manual_.config().lease_ms);
      }
      if (manual_out_.step_rejected) {
        spdlog::warn("manual step refused: {}", manual_out_.step_reject_reason);
      }
    } else if (manual_.lease_active() || manual_out_.step_in_progress) {
      // §41/§93: leaving MANUAL cancels the lease and any step in flight. Coming back
      // must not resume a jog that was interrupted by an AUTO_TRACK acquisition.
      manual_.cancel(now_ns);
      manual_out_ = ManualOutput{};
    }
    if (mode_mgr_.mode() == OperatingMode::AutoRoam) {
      // §34: the sweep needs no camera and no tracking session. Vision keeps running
      // alongside it and the selection stays visible, but pursuing what it sees means
      // switching to AUTO_TRACK — which is also what makes this the mode that still
      // works when no target ever appears.
      const double qy = sp[ix(AxisId::Yaw)].q_rad;
      const double qp = sp[ix(AxisId::Pitch)].q_rad;
      roam_.set_config(roam_config());
      if (!roam_.active()) roam_.enter(qy, qp);  // §36: seeded from where it is
      roam_out_ = roam_.update(qy, qp, now_ns, period_ns);
    } else if (roam_.active()) {
      // §35: a safety override or an operator STOP ends the sweep, and §36's continuity
      // means the next AUTO_ROAM re-enters from wherever the turret ended up.
      roam_.exit();
      roam_out_ = RoamOutput{};
    }
    if (mode_mgr_.mode() == OperatingMode::AutoTrack) {
      at_input_.measurement_age_ms =
          last_measurement_ns_ > 0 ? (now_ns - last_measurement_ns_) / 1000000 : -1;
      at_input_.estimator_ready = tracking_ && tracking_->estimator_initialized();
      // §22 via §67: the converter reports what it could not satisfy and this decides
      // what to do about it. One cycle of lag — the refused request was built from the
      // previous cycle's view. Removing the lag would mean the intent builder consulting
      // the envelope mid-construction, which is the coupling the layer split exists to
      // prevent.
      at_input_.los_feasible = !tracking_ref_.target_unreachable;
      if (tracking_ref_.target_unreachable && manual_out_.step_in_progress)
        manual_.notify_step_refused();  // §41: say so, do not push against a limit
      at_out_ = autotrack_.update(at_input_, now_ns);
    } else if (autotrack_.state() != AutoTrackState::WaitTarget || at_out_.follow_los) {
      autotrack_.reset();
      at_out_ = AutoTrackOutput{};
    }
    last_intent_ = build_mode_intent(now_ns);
    // §36/§44 across every handover, not just the one the section is titled after.
    // A mode's velocity_scale is authority, and authority changes discontinuously at a
    // switch: AUTO_ROAM runs at the roam ceiling and AUTO_TRACK at the track ceiling, so
    // clicking from one to the other stepped the turret 3x on the spot (measured:
    // 0.175 -> 0.524 rad/s, which is within every commissioning limit and still a jolt).
    // Ramping the authority over 300 ms is §44's own "brake/hold first", applied
    // uniformly, and it leaves the envelope the last word exactly as before.
    if (mode_ramp_cycles_ > 0) {
      const double frac =
          1.0 - static_cast<double>(mode_ramp_cycles_ - 1) /
                    static_cast<double>(kModeRampCycles);
      last_intent_.velocity_scale *= frac;
      last_intent_.acceleration_scale *= frac;
      --mode_ramp_cycles_;
    }
    if (last_intent_.type != IntentType::Hold)
      mode_has_moved_ = true;  // from here on, "hold" means *here*, not "back to ready"
    tracking_ref_ = ref_mgr_
                      ? ref_mgr_->resolve(last_intent_, intent_limits(now_ns))
                      : ReferenceRequest{};
    if (!ref_mgr_) {
      // Cannot happen on any path that reaches here (it is built in
      // enable_tracking), but the alternative is dereferencing an optional on the
      // control thread and guessing. Hold, and say so once per boot-ish.
      static bool warned_no_converter = false;
      if (!warned_no_converter) {
        warned_no_converter = true;
        spdlog::error("no intent converter: holding instead of following the "
                      "mode reference");
      }
    }
    if (tracking_) {  // MANUAL has no tracking session, and this block used to be
                      // gated on having one — the dereference below is what a segfault
                      // looks like when the gate is opened and its guards are not.
      tracking_->record_pose(sp[ix(AxisId::Yaw)].q_rad);
      tracking_->record_reference(tracking_ref_);
    }
  }

  // 2c. Phase 9: payload verification (§27, §31.3, §42.2). A web-commanded
  //     check (validated in process_commands) starts here, on the control
  //     thread, with the real cycle timestamp. The §27 OPTIONAL_PAYLOAD_
  //     RESPONSE_CHECK stage runs once per boot: post-homing, on first hold,
  //     at rest, before READY_HOLD — only when a profile is loaded (verifying
  //     against a baseline is the point; profiling is the tool's job, §44).
  if (payload_check_requested_) {
    payload_check_requested_ = false;
    start_payload_check(now_ns, true, sp);
  } else if (cfg_.payload_auto_verify && !payload_auto_done_ &&
             phase_ == Phase::Hold && !tracking_ && at_ready_ &&
             payload_profile_.has_value()) {
    bool at_rest = true;
    for (int i = 0; i < kAxisCount; ++i)
      if (std::fabs(sp[i].v_rad_s) > kAtRestVelRadS) at_rest = false;
    if (at_rest) {
      payload_auto_done_ = true;
      start_payload_check(now_ns, false, sp);
    }
  }

  // 3. Build the supervisor input.
  SupervisorInput in;
  for (int i = 0; i < kAxisCount; ++i) {
    in.axes[i].q_raw_rad = sp[i].q_rad;
    // Use the position-derived velocity (v_est_), NOT the drive's
    // self-reported v_rad_s: the latter is a +/-0.05 rad/s noise band at
    // rest (P0j) that trips the supervisor's Layer-3 stop_feasible ("stop
    // infeasible before soft boundary") when a freshly-homed axis sits at a
    // stop (just outside the inset soft limit, so distance_to_soft is
    // negative). v_est_ is the trustworthy motion derivative — the same one
    // the Fault phase uses for its at-rest gate.
    in.axes[i].v_rad_s = v_est_[i];
    in.axes[i].has_feedback = sp[i].has_feedback;
    in.axes[i].feedback_age_ms =
        sp[i].has_feedback ? (now_ns - sp[i].rx_ns) / 1000000 : INT64_MAX;
    in.axes[i].temp_c = sp[i].temp_c;
    in.axes[i].motor_faults = sp[i].faults;
    in.axes[i].limits = limits_[i];
  }
  in.homing_valid = homed_;
  // §38.1: the supervisor applies the stricter tracking checks only when the
  // reference is actually a tracking reference (position validity is known
  // because tracking requires a valid homing).
  in.tracking_enabled = tracking_ref_.is_tracking_reference;
  in.cycle_overrun_us = overrun_us;
  in.deadline_miss_count = deadline_miss_count_;

  // 4. Safety decision (authoritative).
  last_decision_ = supervisor_.evaluate(in);
  // Low-rate observation of the safety ladder: log an action *transition*
  // (rate-limited to at most once per 100 ms). High-frequency per-command
  // logging in this loop (the P0 [DBG] change-detector) drove the cycle over
  // its 200 Hz deadline and tripped the watchdog — P0h. A transition log is
  // cheap and still reveals chatter (an oscillating action logs ~10x/s).
  {
    static int last_action = -1;
    static TimeNs last_log_ns = 0;
    const int action = static_cast<int>(last_decision_.action);
    if (action != last_action && now_ns - last_log_ns >= 100'000'000) {
      spdlog::info("supervisor: {} reason='{}' overrun_us={} misses={}",
                   safety_action_name(last_decision_.action),
                   last_decision_.reason, in.cycle_overrun_us,
                   in.deadline_miss_count);
      last_action = action;
      last_log_ns = now_ns;
    }
  }

  // 5. Phase reference (per-axis q_ref, limit_spd). Default: hold in place.
  double q_ref[kAxisCount], lim[kAxisCount];
  for (int i = 0; i < kAxisCount; ++i) {
    q_ref[i] = sp[i].q_rad;
    lim[i] = 0.0;
  }
  switch (phase_) {
    case Phase::Homing: {
      const AxisId a = homing_->active_axis();
      // Position-derived velocity (v_est_), not the drive's noisy self-
      // reported v: the MoveTo arrival test and the contact detector's
      // motion/stall logic need a trustworthy velocity (P0j).
      DesiredState ds = homing_->step(to_feedback(sp[ix(a)], v_est_[ix(a)]));
      // One-shot mode entries (see HomingController):
      //  rearm_speed_mode: de-energize/re-energize with the verified
      //    speed-mode recipe before a fine re-approach (after a position-mode
      //    backoff) or endpoint B's approach, resetting the drive's
      //    velocity-loop integral (winds up to ~1.3-1.8 N.m while the
      //    contact dwell pushes the stop — rehome3 root cause, wire-
      //    measured; it exceeds the P-term of ANY SpdRef, Kp ~0.38 N.m/(rad/s)).
      //  enter_pos_mode: the position-mode entry recipe before a backoff
      //    move (de-energize, RunMode=1, re-energize, LimitSpd, pin LocRef to
      //    the current position). The backoff then runs on the drive's own
      //    position loop (p3c fix, 2026-09-02 — see HomingParams backoff
      //    comment): full current-limit torque from the first cycle, no
      //    integral to wind up.
      // Both are blocking (~150-250 ms): one deadline miss (a single Derate;
      // Hold needs a 5-streak) and a brief feedback gap on the non-active
      // axis (benign — the supervisor does not act during homing). The axis
      // bounces/slides slightly during the de-energize; the backoff starts
      // from wherever it lands and the fine re-approach re-measures the
      // end-stop.
      bool rearmed = true;
      if (ds.rearm_speed_mode) {
        std::string e;
        rearmed = backend_->enter_speed_mode(a, ds.limit_cur_a, e);
        if (!rearmed) {
          fault("homing re-arm speed mode failed: " + e);
        }
      } else if (ds.enter_pos_mode) {
        std::string e;
        if (!backend_->enter_position_mode(a, ds.speed_rad_s, e)) {
          fault("homing backoff position mode failed: " + e);
        }
      }
      if (rearmed) {
        if (ds.position_move) {
          // Position mode (backoff): pin LocRef = target, LimitSpd = speed
          // (both write-on-change — a fixed target costs one CAN write per
          // backoff) and let the drive's position loop drive the move.
          backend_->command(a, ds.target_rad, ds.speed_rad_s);
          // PUBLISH the reference in q_ref[]/lim[] too: step 6 (below) runs
          // after this handler and re-commands every position-mode axis from
          // q_ref[]/lim[], applying the safety action on the way. Without
          // this, an Allow cycle re-commands the DEFAULT hold (current
          // position @ 0) and stomps the backoff target every single cycle —
          // the drive's position loop sees the target flip between
          // "backoff @ 10 deg/s" and "hold in place @ 0" at 100 Hz, can never
          // break static friction, and the backoff times out (p3d/p3e root
          // cause, wire-verified: A/C 1:1 alternation for the full 15 s).
          // Safety authority is preserved: a Brake/Hold/Derate cycle still
          // overrides this reference in step 6.
          q_ref[ix(a)] = ds.target_rad;
          lim[ix(a)] = ds.speed_rad_s;
        } else {
          // Speed mode (velocity): command the constant speed (SpdRef) the
          // drive's velocity loop should hold. The homing controller produces
          // a signed velocity (approach +v·dir, fine +v_f·dir, or 0 to hold).
          // This is the root-cause fix for the position-mode stick-slip
          // (P0o): the drive's own velocity loop is the smooth source of
          // motion, not a host-regenerated moving LocRef.
          backend_->command_velocity(a, ds.velocity_rad_s);
        }
        // Hold the non-active axis in place (SpdRef=0). It is in speed mode
        // too (entered in start_homing); holding at its (low) homing current
        // is fine — the axes are orthogonal, so a little drift doesn't affect
        // the homing.
        const AxisId other = (a == AxisId::Pitch) ? AxisId::Yaw : AxisId::Pitch;
        backend_->command_velocity(other, 0.0);
        // The homing carries the drive current limit on the cycle it changes
        // it (the initial per-axis value; also on a re-arm cycle — the
        // backend de-duplicates). 0.0 = leave it unchanged. (Redundant with
        // start_homing's enter_speed_mode, but keeps the controller
        // authoritative.)
        if (ds.limit_cur_a > 0.0)
          backend_->set_current_limit(a, ds.limit_cur_a);
      }
      // High-rate motion log (100 Hz) for the active axis: position, the
      // position-derived velocity/acceleration/jerk, torque, and the commanded
      // velocity. This is the evidence stream for detecting jitter / stuck-slip
      // (the "observe the acceleration" directive): a clean stop shows a->0 with
      // v~0 and flat q; a stick-slip shows a spikes as the drive slips free and
      // re-catches; a stuck (not-driven) axis shows cmd!=0 but v,a~0. Logged
      // every 2nd cycle (100 Hz) to stay under the 200 Hz deadline (a per-cycle
      // log tripped the watchdog — P0h).
      if ((homing_log_cycle_++ & 1) == 0) {
        spdlog::info(
            "motion t={:.2f}ms ax={} q={:+.5f} v={:+.4f} a={:+.2f} j={:+.1f} "
            "tq={:+.3f} cmd={:+.4f} msg={}",
            static_cast<double>(now_ns) / 1e6, axis_name(a), sp[ix(a)].q_rad,
            v_est_[ix(a)], a_est_[ix(a)], jerk_est_[ix(a)],
            sp[ix(a)].torque_nm, ds.velocity_rad_s, ds.message);
      }
      if (homing_->failed()) {
        fault("homing failed: " + homing_->fail_reason());
      } else if (homing_->complete()) {
        if (finalize_homing()) {
          std::string e;
          if (enter_position_mode_all(cfg_.hold_speed_rad_s, e))
            phase_ = Phase::Hold;  // one-time re-pin, then ready-hold
          else
            fault(e);
        } else {
          fault("homing plan incomplete");
        }
      }
      break;
    }
    case Phase::Hold: {
      if (tracking_ref_.source != ReferenceSource::Hold) {
        // v3: a mode asked for motion, so the envelope constrains that reference
        // (§15: clamp the position to the soft limits, and cap the speed at the
        // largest value from which a full stop still fits before the boundary).
        // Nothing below this line runs for a moving mode.
        at_ready_ = false;  // "at the ready pose" and "a mode is pointing
                            // elsewhere" are exclusive by definition
        for (int i = 0; i < kAxisCount; ++i) {
          const double r = (i == ix(AxisId::Yaw)) ? tracking_ref_.q_yaw_rad
                                                  : tracking_ref_.q_pitch_rad;
          q_ref[i] = env_.constrain_reference(r, limits_[i]);
          lim[i] = std::min(tracking_ref_.v_max_rad_s,
                            env_.max_speed_at(q_ref[i], limits_[i]));
        }
        break;
      }
      // Quiet hold. Kept in its v1 shape on purpose, including the part a
      // refactor tends to "clean up": once both axes are on the ready pose the
      // command is *stay exactly here with no speed authority* (lim = 0), not
      // *drive to the ready pose at hold speed*. On paper the same answer; on
      // this station it is the difference between a parked arm and one whose
      // position loop fights static friction forever. Not something to change as
      // a side effect of wiring modes in.
      // Phase 8: restricted one-shot test motion (§42.2) on yaw, validated by
      // the web UI and re-clamped by the envelope below. Takes priority over
      // the ready pose for that cycle.
      const bool test_motion = has_test_motion_;
      has_test_motion_ = false;
      // Whose pose a "hold" holds is a decision, not a constant — and it took a failing
      // test to make that visible. v1's hold meant "return to the safe ready pose", which
      // is right for the end of homing and wrong for §52's STOP MOTION: measured in
      // simulation, a stop taken 14 degrees into a sweep drove the turret 14 degrees the
      // other way to get back to ready. During the three operating modes at Ready the hold
      // pose is the place it stopped, latched on the stopping cycle; everywhere else it is
      // the ready pose, exactly as v1 had it.
      //
      // Everything below is unchanged on purpose, including the arrived case's lim = 0:
      // on this station that is the difference between a parked arm and one whose position
      // loop fights static friction forever (§33.2's lesson, from metal).
      const double hold_pose[2] = {
          mode_hold_in_place_ ? mode_hold_pitch_rad_ : ready_raw_[0],
          mode_hold_in_place_ ? mode_hold_yaw_rad_ : ready_raw_[1]};
      at_ready_ = true;
      for (int i = 0; i < kAxisCount; ++i) {
        if (test_motion && i == (int)ix(AxisId::Yaw)) {
          const double target =
              env_.constrain_reference(test_motion_target_rad_, limits_[i]);
          q_ref[i] = target;
          lim[i] = std::min(hold_speed_effective(),
                            env_.max_speed_at(target, limits_[i]));
          at_ready_ = false;
          continue;
        }
        if (std::fabs(sp[i].q_rad - hold_pose[i]) > kReadyPosTolRad) {
          q_ref[i] = hold_pose[i];
          lim[i] = hold_speed_effective();
          at_ready_ = false;
        } else {
          q_ref[i] = sp[i].q_rad;
          lim[i] = 0.0;
        }
      }
      break;
    }
    case Phase::PayloadCheck: {
      // Phase 9: one axis at a time (pitch, then yaw); the other axis holds.
      // The check stepper is a pure reference computer; the loop commands
      // (so the supervisor above keeps authority, §38).
      //
      // Inactive-axis hold: EVERY axis is held at its FIXED check-start
      // position (payload_hold_target_) with a NONZERO speed limit, so the
      // CyberGear position loop actively holds it. At LimitSpd=0 the
      // position loop is pinned and the inactive axis free-drifts under
      // gravity/friction stick-slip (P6 live Run A: yaw +3.6 deg, incl. a
      // 2.7 deg/s burst, while the pitch axis was checked). A fixed target
      // (not a re-pin to the live position) is what makes it a real hold —
      // the proven park Verify pattern (§33.2).
      for (int i = 0; i < kAxisCount; ++i) {
        q_ref[i] = payload_hold_target_[i];
        lim[i] = std::min(cfg_.hold_speed_rad_s,
                          env_.max_speed_at(payload_hold_target_[i], limits_[i]));
      }
      auto& check = payload_checks_[payload_axis_ix_];
      if (!check.active() && !check.failed()) {
        if (!check.begin(now_ns, sp[payload_axis_ix_].q_rad,
                         sp[payload_axis_ix_].has_feedback)) {
          abort_payload_check(now_ns, check.fail_reason());
          break;
        }
      }
      if (check.active()) {
        // Settle gate on POSITION-derived velocity (v_est_), not the drive's
        // self-reported v: at rest that is a ±0.05 rad/s noise band (P0j)
        // sitting exactly on the at_rest_vel_rad_s gate, so the 0.3 s dwell
        // could never confirm. Same substitution the Fault phase uses.
        AxisSnapshot gate = sp[payload_axis_ix_];
        gate.v_rad_s = v_est_[payload_axis_ix_];
        const auto out = check.step(now_ns, gate);
        q_ref[payload_axis_ix_] = out.q_ref_rad;
        lim[payload_axis_ix_] = out.limit_spd_rad_s;
        if (!check.active()) {
          // A FAILED axis aborts the whole check — do NOT silently advance
          // to the other axis (P6 live Run A: the swallowed pitch failure let
          // the yaw check run its own 8 s, so the abort logged 16 s after
          // check start with the SECOND axis's reason).
          if (check.failed()) {
            abort_payload_check(now_ns, check.fail_reason());
          } else if (payload_axis_ix_ == 0) {
            payload_axis_ix_ = 1;  // pitch done OK; begin the yaw check next cycle
          } else {
            finish_payload_check(now_ns);
          }
        }
      } else if (check.failed()) {
        abort_payload_check(now_ns, check.fail_reason());
      }
      break;
    }
    case Phase::Parking: {
      ParkOutput po = park_->step(
          to_feedback(sp[ix(AxisId::Pitch)], v_est_[ix(AxisId::Pitch)]),
          to_feedback(sp[ix(AxisId::Yaw)], v_est_[ix(AxisId::Yaw)]));
      if (po.speed_mode) {
        // Speed-mode park move (MoveYaw/MovePitch): SpdRef-driven, mirroring
        // the Homing phase (P0o). MoveTo emits the signed velocity for the
        // active axis (capped by the stop-distance profile); the inactive
        // axis holds at SpdRef=0. The generic command path below sees
        // in_speed_mode and only issues a controlled stop (SpdRef=0) on a
        // safety action, which is issued after this and therefore wins.
        backend_->command_velocity(AxisId::Pitch, po.pitch.velocity_rad_s);
        backend_->command_velocity(AxisId::Yaw, po.yaw.velocity_rad_s);
      } else {
        // §33.2 target-hold (Verify/Dwell/Disable): position mode holding AT
        // THE PARK TARGET (the drive's position loop pulls the axis back to
        // the target — no re-pin to the current position; see
        // ParkController::step for the real-station evidence). The hold
        // carries a NON-ZERO speed limit (verify_speed_deg_s): the CyberGear
        // position loop is pinned at LimitSpd=0, so a 0-limit hold can never
        // pull an axis back into the §33.2 window (p3: 40 s stall at the
        // overshoot point). One-time blocking mode entry = a single Derate.
        if (!park_pos_mode_entered_) {
          std::string e;
          if (enter_position_mode_all(cfg_.park.verify_speed_deg_s * kDeg2Rad,
                                      e)) {
            park_pos_mode_entered_ = true;
          } else {
            fault(e);
          }
        }
        q_ref[ix(AxisId::Pitch)] = po.pitch.target_rad;
        lim[ix(AxisId::Pitch)] = cfg_.park.verify_speed_deg_s * kDeg2Rad;
        q_ref[ix(AxisId::Yaw)] = po.yaw.target_rad;
        lim[ix(AxisId::Yaw)] = cfg_.park.verify_speed_deg_s * kDeg2Rad;
      }
      if (po.disable_pitch) backend_->deenergize(AxisId::Pitch);
      if (po.disable_yaw) backend_->deenergize(AxisId::Yaw);
      if (po.failed) fault("park failed: " + po.message);
      if (po.complete) phase_ = Phase::Parked;
      break;
    }
    case Phase::Fault: {
      // Controlled stop: keep braking while moving, then hold in place.
      // Use the position-derived velocity (v_est_), NOT the drive's
      // self-reported v_rad_s: the latter is a ±0.05 rad/s noise band at rest
      // (P0j) that chatters the kAtRestVelRadS gate and ping-ponged the
      // emergency-stop reference at 1 Hz while the axis was actually stopped.
      for (int i = 0; i < kAxisCount; ++i) {
        if (std::fabs(v_est_[i]) > kAtRestVelRadS) {
          q_ref[i] = env_.emergency_stop_target(sp[i].q_rad, v_est_[i],
                                                limits_[i]);
          lim[i] = cfg_.emergency_speed_rad_s;
        } else {
          q_ref[i] = sp[i].q_rad;
          lim[i] = 0.0;
        }
      }
      break;
    }
    case Phase::Idle:
    case Phase::Parked:
    default:
      break;  // hold (no motion)
  }

  // 6. Apply the safety action (overrides the phase reference), then command.
  bool any_disable = false;
  for (int i = 0; i < kAxisCount; ++i) {
    const AxisId a = static_cast<AxisId>(i);
    double qr = q_ref[i], ls = lim[i];
    bool do_command = true;
    switch (last_decision_.action) {
      case SafetyAction::Allow:
        break;
      case SafetyAction::Derate:
        ls *= cfg_.derate_factor;
        break;
      case SafetyAction::Hold:
        qr = sp[i].q_rad;
        ls = 0.0;
        break;
      case SafetyAction::Brake:
      case SafetyAction::FaultStop:
        // Position-derived velocity (v_est_), as in the Fault phase: the
        // drive's self-reported v is a noise band at rest (P0j) that would
        // ping-pong the stop reference each cycle.
        qr = env_.emergency_stop_target(sp[i].q_rad, v_est_[i], limits_[i]);
        ls = cfg_.emergency_speed_rad_s;
        break;
      case SafetyAction::Disable:
        backend_->deenergize(a);
        any_disable = true;
        do_command = false;
        break;
    }
    if (do_command) {
      if (sp[i].in_speed_mode) {
        // Speed mode (homing): the motion reference is the SpdRef command from
        // the Homing phase, NOT the position-mode (qr, ls). command() is a
        // position-mode reference — issuing it here would switch the axis out
        // of speed mode and reintroduce the stick-slip. On a safety action
        // (anything but Allow) command a controlled stop (SpdRef=0); the
        // velocity loop decelerates smoothly to rest. On Allow issue NOTHING
        // that changes the reference — but do keep the feedback alive: the
        // drive has no periodic telemetry, so without a ping the feedback age
        // crosses feedback_max_age_ms after ~5 quiet cycles and the supervisor
        // flaps BRAKE/ALLOW forever, each BRAKE stomping the other axis's
        // reference (p0p hold phase; p3e fault phase, wire-verified B/C 1:1).
        if (last_decision_.action != SafetyAction::Allow)
          backend_->command_velocity(a, 0.0);
        else
          backend_->keepalive(a);
      } else {
        backend_->command(a, qr, ls);
      }
    }
  }

  // 7. Fault transitions (a fault is sticky; it needs a manual reset).
  if (any_disable) {
    fault(last_decision_.reason);
  } else if (last_decision_.action == SafetyAction::FaultStop &&
             phase_ != Phase::Fault) {
    fault(last_decision_.reason);
  }

  // 8. Telemetry (§43, §6.3): ALWAYS fill the §6.3 snapshot (webd/logd read it
  //    at 10-20 Hz from a non-RT thread) plus the high-rate logs. In-memory
  //    only (§46: no I/O in the control loop). Tracking fields are populated
  //    only while tracking mode is enabled.
  {
    telemetry::ControlLogRecord rec;
    rec.timestamp_ns = now_ns;
    rec.q_actual[ix(AxisId::Pitch)] = sp[ix(AxisId::Pitch)].q_rad;
    rec.q_actual[ix(AxisId::Yaw)] = sp[ix(AxisId::Yaw)].q_rad;
    rec.v_actual[ix(AxisId::Pitch)] = sp[ix(AxisId::Pitch)].v_rad_s;
    rec.v_actual[ix(AxisId::Yaw)] = sp[ix(AxisId::Yaw)].v_rad_s;
    // Position-derived acceleration / jerk (C1, A.1): the trustworthy motion
    // derivatives for smoothness observation and jitter-threshold tuning.
    rec.a_actual[ix(AxisId::Pitch)] = a_est_[ix(AxisId::Pitch)];
    rec.a_actual[ix(AxisId::Yaw)] = a_est_[ix(AxisId::Yaw)];
    rec.jerk_actual[ix(AxisId::Pitch)] = jerk_est_[ix(AxisId::Pitch)];
    rec.jerk_actual[ix(AxisId::Yaw)] = jerk_est_[ix(AxisId::Yaw)];
    rec.q_ref[ix(AxisId::Pitch)] = q_ref[ix(AxisId::Pitch)];
    rec.q_ref[ix(AxisId::Yaw)] = q_ref[ix(AxisId::Yaw)];
    rec.safety_action = last_decision_.action;
    rec.feedback_age_ms = sp[ix(AxisId::Yaw)].has_feedback
                              ? (now_ns - sp[ix(AxisId::Yaw)].rx_ns) / 1000000
                              : INT64_MAX;
    rec.cycle_duration_us = period_ns / 1000;
    rec.track_state = tracking_ ? tracking_->track_state()
                                : tracking::TrackState::ReadyHold;
    for (int i = 0; i < kAxisCount; ++i)
      rec.soft_limit_distance[i] = limits_[i].valid ? limits_[i].distance_to_soft(sp[i].q_rad) : 0.0;
    telemetry_.push_control(rec);
    telemetry_.push_blackbox(rec);

    telemetry::TelemetrySnapshot snap;
    snap.timestamp_ns = now_ns;
    snap.phase = phase_name(phase_);
    snap.fault_reason = fault_reason_;
    snap.at_ready = at_ready_;
    snap.q_yaw_rad = sp[ix(AxisId::Yaw)].q_rad;
    snap.v_yaw_rad_s = sp[ix(AxisId::Yaw)].v_rad_s;
    snap.effort_yaw = sp[ix(AxisId::Yaw)].torque_nm;
    snap.q_pitch_rad = sp[ix(AxisId::Pitch)].q_rad;
    snap.v_pitch_rad_s = sp[ix(AxisId::Pitch)].v_rad_s;
    snap.effort_pitch = sp[ix(AxisId::Pitch)].torque_nm;
    snap.q_ref_yaw_rad = q_ref[ix(AxisId::Yaw)];
    snap.q_ref_pitch_rad = q_ref[ix(AxisId::Pitch)];
    snap.track_state = tracking_ ? tracking_->track_state()
                                 : tracking::TrackState::ReadyHold;
    snap.target_confidence = tracking_ ? tracking_->confidence() : 0.0;
    snap.tracking_active = tracking_ && tracking_ref_.is_tracking_reference;
    // Which candidate is being followed is loop state, not link state. Publishing it
    // from inside the `if (vision_link_)` block below would report "following nothing"
    // for every unit test and for controld started without a vision socket, while the
    // turret was in fact tracking something — a field that lies only in some
    // configurations is worse than one that is missing.
    // §11/§78: publish the candidates, not just a count. The distinction matters at
    // 11 pm over a network link: "2 tracks" tells the operator the detector is alive,
    // and nothing at all about which one to name. The list comes from the set the
    // selection manager holds, so it is by construction the same list the selection was
    // validated against — a UI built on a different copy could offer a label controld
    // would refuse, which is the dead-button problem again.
    snap.track_count = 0;
    snap.track_list_age_ms =
        last_set_receive_ns_ > 0 ? static_cast<int64_t>((now_ns - last_set_receive_ns_) /
                                                        1000000)
                                : -1;
    {
      const tracks::TrackSet& shown = selection_.last_set();
      const tracks::Track* chosen = selection_.selected_track();
      for (int i = 0; i < shown.count && snap.track_count < telemetry::TelemetrySnapshot::kMaxTrackList;
           ++i) {
        const tracks::Track& t = shown.tracks[i];
        telemetry::TrackListing& out = snap.tracks[snap.track_count++];
        out = telemetry::TrackListing{};
        out.uuid_lo = t.uuid.lo;
        out.display_index = t.display_index;
        // Same shape as the selection's own descriptor (§10): a label is a label in
        // both places or the operator is being shown two vocabularies.
        char cap[16];
        std::snprintf(cap, sizeof cap, "%s", t.class_name);
        if (cap[0] >= 'a' && cap[0] <= 'z') cap[0] = static_cast<char>(cap[0] - 32);
        std::snprintf(out.label, sizeof out.label, "%s #%u", cap,
                      static_cast<unsigned>(t.display_index));
        std::snprintf(out.class_name, sizeof out.class_name, "%s", t.class_name);
        // Telemetry's state vocabulary is uppercase throughout (READY, VISIBLE,
        // SWEEP); the tracks header names these lowercase for the vision wire. Both are
        // right where they live, and a page that compares one against the other needs
        // the conversion said out loud rather than a mix of cases to debug.
        std::snprintf(out.state, sizeof out.state, "%s",
                      tracks::track_state_name(t.state));
        for (char* c = out.state; *c; ++c)
          if (*c >= 'a' && *c <= 'z') *c = static_cast<char>(*c - 32);
        out.confidence = t.detector_confidence;
        out.anchor_x = t.anchor_x;
        out.anchor_y = t.anchor_y;
        out.selectable = t.state == tracks::TrackState::Confirmed;
        out.selected = chosen != nullptr && chosen->uuid.lo == t.uuid.lo;
      }
    }
    snap.selected_track_id = selected_track_id_;
    {
      const auto& sel = selection_.selection();
      snap.selected_display_index = sel.has_selection ? sel.selected_display_index : 0;
      snap.selected_descriptor =
          sel.has_selection ? std::string(sel.selected_descriptor) : std::string();
      snap.selection_visibility =
          tracks::visibility_name(sel.has_selection ? sel.visibility_state
                                                   : tracks::Visibility::None);
      snap.selection_ambiguous = sel.has_selection && sel.ambiguous_reacquisition;
      snap.reacquisition_score = sel.reacquisition_score;
      snap.ambiguity_margin = sel.ambiguity_margin;
    }
    snap.safety_action = last_decision_.action;
    snap.feedback_age_ms = rec.feedback_age_ms;
    snap.control_cycle_us = period_ns / 1000;
    // CAN link health (§55/§54.4): read the transport's own counters. Same
    // cost class as the vision stats read below (a mutex + a POD copy), and it
    // is observation only — no decision in the loop consults these numbers.
    {
      const CanHealth ch = backend_->can_health();
      snap.can_available = ch.available;
      snap.can_kind = ch.kind;
      snap.can_device = ch.device;
      snap.can_up = ch.up;
      snap.can_state = static_cast<int8_t>(ch.state);
      snap.can_rx_frames = ch.rx_frames;
      snap.can_rx_error_frames = ch.rx_error_frames;
      snap.can_tx_frames = ch.tx_frames;
      snap.can_tx_failed = ch.tx_failed;
      // A negative age would mean the two stamps live in different clock
      // domains (a test clock against a monotonic one); publish -1 = "unknown"
      // rather than nonsense, the same rule as the vision age below.
      snap.can_last_rx_age_ms =
          (ch.last_rx_ns > 0 && now_ns >= ch.last_rx_ns)
              ? static_cast<int64_t>((now_ns - ch.last_rx_ns) / 1000000)
              : -1;
    }
    // Phase 9: payload profile status (§42.1, §31.3).
    snap.payload_profile_name = payload_profile_ ? payload_profile_->name : "";
    snap.payload_profile_status =
        payload::payload_status_name(payload_status_);
    snap.payload_derated = payload_derated_;
    snap.payload_check_active = (phase_ == Phase::PayloadCheck);
    // v3 §50: what the screen needs in order to say *why* the turret is doing
    // what it is doing. The intent is reported as names plus its velocity scale,
    // so a derated tracking reference reads as "auto_track / los_direction /
    // coasting / 0.4" instead of a turret that quietly slowed down and a log with
    // nothing in it to explain itself.
    snap.operating_mode = operating_mode_name(mode_mgr_.mode());
    snap.supervisory_state = supervisory_state_name(mode_mgr_.supervisory());
    snap.mode_phase =
        mode_mgr_.mode() == OperatingMode::AutoRoam
            ? roam_state_name(roam_.state())
            : mode_mgr_.mode() == OperatingMode::AutoTrack
                  ? auto_track_state_name(at_out_.state)
                  : mode_phase_label();
    snap.roam_target_yaw_rad = roam_out_.target_yaw_rad;
    snap.roam_sweep_direction = roam_out_.direction;
    snap.manual_lease_active = manual_.lease_active();
    snap.manual_lease_remaining_ms = manual_out_.lease_remaining_ms;
    snap.manual_profile = manual_profile_name(manual_.profile());
    snap.confidence_band = confidence_band_name(at_out_.band);
    snap.selected_confidence = at_out_.selected_confidence;
    snap.intent_source = motion_source_name(last_intent_.source);
    snap.intent_type = intent_type_name(last_intent_.type);
    snap.intent_reason = last_intent_.reason;
    snap.intent_velocity_scale = last_intent_.velocity_scale;
    // §52. Published from the loop's own copy, so the answer exists even with no
    // web client connected: whoever pressed the button while the browser was
    // closed is exactly the person who wants to know what it did.
    snap.cmd_ack_command = last_ack_.command;
    snap.cmd_ack_accepted = last_ack_.command.empty()
                                ? static_cast<int8_t>(-1)
                                : static_cast<int8_t>(last_ack_.accepted ? 1 : 0);
    snap.cmd_ack_reason = last_ack_.reason;
    snap.cmd_ack_controller_state = last_ack_.controller_state;
    snap.cmd_ack_safety_state = last_ack_.safety_state;
    snap.cmd_ack_seq = last_ack_.seq;
    // Vision transport (observe-only; the ingest thread owns these counters).
    if (vision_link_) {
      const vision::VisionLink::Stats vs = vision_link_->stats();
      snap.vision_connected = vs.connected;
      snap.vision_frames = vs.frames;
      snap.vision_dropped = vs.dropped;
      snap.vision_last_frame_sequence = vs.last_frame_sequence;
    // §61: is the publisher speaking v3, and where is the time going? A TrackSet
    // count of zero against a connected publisher means it is still v1, which is a
    // different story from "v3 but slow" and needs a different fix.
    snap.vision_track_sets = vs.track_sets;
    snap.vision_sensor_age_ms =
        (vs.last_sensor_ns > 0 && now_ns >= vs.last_sensor_ns)
            ? static_cast<int64_t>((now_ns - vs.last_sensor_ns) / 1000000)
            : -1;
    snap.vision_publish_to_receive_ms =
        (vs.last_publish_ns > 0 && vs.last_arrival_ns >= vs.last_publish_ns)
            ? static_cast<int64_t>((vs.last_arrival_ns - vs.last_publish_ns) / 1000000)
            : -1;
      if (vs.last_arrival_ns > 0) {
        // Both stamps are in the host-monotonic domain (§6.2): the ingest thread
        // stamps arrival with now_monotonic_ns() and the daemon drives this
        // loop's clock from the same clock. A negative age can therefore only
        // mean a clock-domain mismatch (a simulated clock in tests): clamp it,
        // never publish nonsense to the dashboard.
        const int64_t age_ms = (now_ns - vs.last_arrival_ns) / 1000000;
        snap.vision_measurement_age_ms = age_ms > 0 ? age_ms : 0;
      } else {
        snap.vision_measurement_age_ms = -1;  // no measurement has ever arrived
      }
    }
    // Phase 7: world-frame telemetry (§29/§30) — base tilt + world-frame LOS
    // from the active R_W_B. Makes tracking world-correct for a tilted base.
    double az_base = 0.0, el_base = 0.0;
    if (tracking_) tracking_->predicted_los(az_base, el_base);
    fill_world_frame_telemetry(base_orientation_, az_base, el_base, snap);
    telemetry_.set_snapshot(snap);
  }

  // 8b. Phase 8: publish the command-validation state (§42.2) so the web thread
  //     can validate developer commands against the authoritative state.
  {
    std::lock_guard<std::mutex> lk(command_mutex_);
    command_state_.homed = homed_;
    command_state_.fault = (phase_ == Phase::Fault);
    // Homing is a sequence: between stages the phase is Hold while the station
    // is still travelling to the ready pose. `moving` alone cannot see that, and
    // a payload check requested in that window used to be accepted and then
    // aborted (the hold pose was a travel stop, so the safe region came out
    // empty). The gate now says so synchronously (§42.2).
    command_state_.at_ready = at_ready_;
    command_state_.tracking_enabled = tracking_ != nullptr;
    command_state_.tracking_active =
        tracking_ && tracking_ref_.is_tracking_reference;
    command_state_.search_enabled =
        tracking_ && tracking_->track_state() == tracking::TrackState::Search;
    command_state_.moving = (phase_ == Phase::Homing) ||
                            (phase_ == Phase::Parking) ||
                            (phase_ == Phase::PayloadCheck) ||
                            (last_decision_.action != SafetyAction::Allow &&
                             last_decision_.action != SafetyAction::Hold);
    command_state_.limits_valid =
        limits_[ix(AxisId::Pitch)].valid && limits_[ix(AxisId::Yaw)].valid;
    command_state_.q_min_rad[web::kPitchIx] =
        limits_[ix(AxisId::Pitch)].q_soft_min_rad;
    command_state_.q_max_rad[web::kPitchIx] =
        limits_[ix(AxisId::Pitch)].q_soft_max_rad;
    command_state_.q_min_rad[web::kYawIx] = limits_[ix(AxisId::Yaw)].q_soft_min_rad;
    command_state_.q_max_rad[web::kYawIx] = limits_[ix(AxisId::Yaw)].q_soft_max_rad;
  }

  return phase_;
}

// ---------------------------------------------------------------------------
// Phase 9: payload profiling / verification (§28.5, §31.3, §41).
// ---------------------------------------------------------------------------

void ControlLoop::set_payload_profile(payload::PayloadProfile pr,
                                      bool commissioned) {
  if (phase_ == Phase::PayloadCheck) return;  // never swap mid-check
  payload_profile_ = std::move(pr);
  if (commissioned) {
    // A commissioned profile is trusted (§28.5) until a verification says
    // otherwise; a fresh profile clears any earlier mismatch derate (this is
    // the repeatable-commissioning path, §28.5/§31.3).
    payload_status_ = payload::PayloadStatus::Ok;
    payload_detail_ = "profile loaded: " + payload_profile_->name;
  } else {
    // Runtime selection (§42.2): the profile CAPS station motion from now on,
    // but it is not believed until the check measures against it (§31.3).
    payload_status_ = payload::PayloadStatus::NoProfile;
    payload_detail_ = "profile '" + payload_profile_->name +
                      "' selected at runtime; run start_payload_verification "
                      "to confirm it against the installed mass (§31.3)";
  }
  apply_payload_derate(false);
}

double ControlLoop::hold_speed_effective() const {
  double v = cfg_.hold_speed_rad_s;
  if (payload_profile_) {
    // §28.5: the profiled safe v_max caps station motion.
    if (payload_profile_->pitch.v_max_rad_s > 0.0)
      v = std::min(v, payload_profile_->pitch.v_max_rad_s);
    if (payload_profile_->yaw.v_max_rad_s > 0.0)
      v = std::min(v, payload_profile_->yaw.v_max_rad_s);
  }
  if (payload_derated_) v *= cfg_.derate_factor;  // §31.3 conservative path
  return v;
}

// --- v3 mode plumbing (§43/§44/§53) ---------------------------------------

RoamEnvelope ControlLoop::safe_envelope() const {
  RoamEnvelope e;
  e.yaw_min_rad = limits_[ix(AxisId::Yaw)].q_soft_min_rad;
  e.yaw_max_rad = limits_[ix(AxisId::Yaw)].q_soft_max_rad;
  e.pitch_min_rad = limits_[ix(AxisId::Pitch)].q_soft_min_rad;
  e.pitch_max_rad = limits_[ix(AxisId::Pitch)].q_soft_max_rad;
  return e;
}

RoamConfig ControlLoop::roam_config() const {
  RoamConfig c = roam_.config();
  const AxisLimits& yl = limits_[ix(AxisId::Yaw)];
  const AxisLimits& pl = limits_[ix(AxisId::Pitch)];
  const double inset = cfg_.soft_margin_rad + cfg_.stop_margin_rad;
  const double ready_yaw = ready_raw_[ix(AxisId::Yaw)];
  c.envelope.yaw_min_rad =
      std::max(ready_yaw - cfg_.search_span_rad, yl.q_soft_min_rad + inset);
  c.envelope.yaw_max_rad =
      std::min(ready_yaw + cfg_.search_span_rad, yl.q_soft_max_rad - inset);
  c.envelope.pitch_min_rad = pl.q_soft_min_rad + inset;
  c.envelope.pitch_max_rad = pl.q_soft_max_rad - inset;
  // §30: one elevation for the whole sweep, and it is the elevation the station was
  // declared ready at — the pose the operator has already seen is safe under load. §72
  // will let a station name its own roam region and elevation; until that file exists,
  // deriving from the limits the station already obeys is the only source that cannot
  // drift from the truth.
  c.pitch_ref_rad = ready_raw_[ix(AxisId::Pitch)];
  c.v_max_rad_s = std::min(tracking_cfg_.search_v_max_rad_s, hold_speed_effective());
  // §33: a world-level scan needs gravity from the BNO085 expansion. This station has
  // the sensor, but core v3 does not depend on it, so the sweep stays in joint space and
  // says so rather than claiming a level scan it cannot honour.
  c.level_scan_available = false;
  return c;
}

ModeRequestContext ControlLoop::mode_context() const {
  ModeRequestContext c;
  // Allow/Derate are the "healthy" answers; Brake / Hold / FaultStop / Disable
  // all mean the supervisor has taken the axes away. Derived from the supervisor
  // rather than re-derived here so the two can never disagree about what "we are
  // allowed to move" means — a mode gate with its own opinion of health is how a
  // station ends up arguing with itself.
  const bool allowed = last_decision_.action == SafetyAction::Allow ||
                       last_decision_.action == SafetyAction::Derate;
  c.safety_healthy = allowed;
  // §44's "position valid": homed, *and* not being told the feedback is stale.
  // A station that cannot see its own encoders does not know where it is, however
  // homed its bookkeeping claims.
  c.position_valid = homed_ && allowed;
  // V3-1: the roam region is still v1's search band (computed at enable_tracking
  // from the homed soft limits minus margins). It counts as valid when that band
  // is non-empty. V3-6 replaces this with the configured inner roam envelope of
  // §32/§72 and a real validator; until then an empty band must refuse AUTO_ROAM
  // rather than let it fall back to sweeping the whole soft-limit span, which is
  // the tempting implementation and the one that finds the mechanical stops.
  // §32, checked rather than assumed: the roam region must be non-empty *and* sit
  // inside the safe envelope with room left over. The test here used to be "the v1 search
  // band is non-empty", which passes a configuration that sweeps right up to the
  // mechanical stops — the tempting implementation, and the one that finds them.
  {
    char unused[1] = {};
    const RoamConfig rc = roam_config();
    c.roam_envelope_valid =
        homed_ && RoamPlanner::validate_envelope(rc.envelope, safe_envelope(),
                                                 rc.pitch_ref_rad,
                                                 rc.min_inside_safe_rad, unused,
                                                 sizeof unused);
  }
  c.supervisory = mode_mgr_.supervisory();
  return c;
}

ModeResult ControlLoop::request_mode(OperatingMode target) {
  const std::string who = ack_in_flight_.empty() ? "request_mode" : ack_in_flight_;
  const ModeRequestContext ctx = mode_context();
  const ModeResult r = mode_mgr_.request(target, ctx);
  if (!r.ok && target == OperatingMode::AutoRoam && !ctx.roam_envelope_valid) {
    // The mode manager's reason is true and useless: "roam envelope invalid". Which
    // bound, by how much? Say it here, where the numbers are in scope, because this is
    // the message someone acts on from the other end of a network connection.
    char why[192] = {};
    if (!homed_) {
      std::snprintf(why, sizeof why,
                    "not homed, so the roam region is not known (home first)");
    } else {
      const RoamConfig rc = roam_config();
      RoamPlanner::validate_envelope(rc.envelope, safe_envelope(), rc.pitch_ref_rad,
                                     rc.min_inside_safe_rad, why, sizeof why);
    }
    std::snprintf(mode_refusal_reason_, sizeof mode_refusal_reason_,
                  "AUTO_ROAM refused: %s", why);
    ack_command(who, false, mode_refusal_reason_);
    return {false, false, mode_refusal_reason_};
  }
  if (!r.ok) {
    // §52: the reason that used to stop in the log now goes back to whoever asked.
    ack_command(who, false, r.reason);
    return r;
  }
  ack_command(who, true,
              std::string(r.changed ? "entered " : "already in ") +
                  operating_mode_name(target));
  sync_controllers_to_mode(target);
  if (r.changed)
    spdlog::info("MODE_CHANGED -> {} (supervisory {})",
                 operating_mode_name(target),
                 supervisory_state_name(mode_mgr_.supervisory()));
  return r;
}

ModeResult ControlLoop::stop_motion() {
  const OperatingMode was = mode_mgr_.mode();
  const ModeResult r = mode_mgr_.stop_motion(mode_context());
  ack_command(ack_in_flight_.empty() ? "stop_motion" : ack_in_flight_, true,
              std::string("cancelled ") + operating_mode_name(was) +
                  " intent; manual hold");
  sync_controllers_to_mode(OperatingMode::Manual);
  spdlog::warn("STOP_MOTION: {} intent cancelled, controlled hold in MANUAL "
               "(§27; not a disable, not a shutdown)", operating_mode_name(was));
  return r;
}

void ControlLoop::sync_controllers_to_mode(OperatingMode mode) {
  mode_hold_latched_ = false;  // §44: "here" is re-decided at a handover, not inherited
  mode_ramp_cycles_ = kModeRampCycles;  // §36/§44: see the ramp in step()
  // `mode_has_moved_` deliberately survives a handover. It is cleared only when the
  // station leaves Ready, below. Clearing it here would break the one handover that
  // matters most: STOP MOTION goes AUTO_ROAM -> MANUAL, and a flag reset by that
  // transition puts the hold pose back to the ready pose — so the turret stops, then
  // drives the whole way back. §52's button must stop the machine, not relocate it.
  // The v1 controllers are the mechanism; the mode is the authority. Search
  // (the sweep) is armed only for AUTO_ROAM — §111.5 freezes that AUTO_TRACK
  // does not roam looking for a target, so the flag follows the mode. Any stale
  // value left by the v1 enable_search command is corrected here rather than
  // argued with.
  search_override_ = (mode == OperatingMode::AutoRoam);
  switch (mode) {
    case OperatingMode::Manual:
      // MANUAL owns motion through jog/step (V3-5). Until those exist it holds,
      // and holding is exactly what dropping the tracking reference does.
      if (tracking_) disable_tracking();
      break;
    case OperatingMode::AutoTrack:
    case OperatingMode::AutoRoam: {
      std::string err;
      if (!tracking_ && !enable_tracking(tracking_cfg_, err))
        spdlog::warn("mode {}: tracking session could not start: {} — the mode "
                     "stays selected but will hold", operating_mode_name(mode),
                     err);
      else if (tracking_)
        tracking_->set_search_enabled(mode == OperatingMode::AutoRoam);
      break;
    }
  }
}

ReferenceManager::IntentLimits ControlLoop::intent_limits(TimeNs now_ns) const {
  ReferenceManager::IntentLimits l;
  l.now_ns = now_ns;
  // "Hold" has two meanings and v1 used only the first. v1's hold was *return to the
  // ready pose* — correct for the end of homing and for a tracking session winding down.
  // For the three operating modes it has to mean "stay where you are", and the
  // difference is not cosmetic: measured in simulation, a STOP MOTION taken 14 degrees
  // into a sweep moved the turret 14 degrees on the way to the ready pose. A button whose
  // name is STOP MOTION does not get to make a move of that size, and "it moved away
  // from me when I hit stop" is the sort of fact that ends trust in the panel.
  //
  // The pose is the last commanded reference rather than the measured feedback, so the
  // hold target does not wander with encoder noise; and this applies only while the
  // station is in one of the three modes at Ready. Homing, parking and fault are
  // Supervisory and carry their own explicit targets, where "go to the ready pose" is
  // exactly what is wanted.
  const bool mode_hold_in_place =
      mode_has_moved_ && mode_mgr_.supervisory() == SupervisoryState::Ready &&
      (mode_mgr_.mode() == OperatingMode::Manual ||
       mode_mgr_.mode() == OperatingMode::AutoTrack ||
       mode_mgr_.mode() == OperatingMode::AutoRoam);
  mode_hold_in_place_ = mode_hold_in_place;
  if (mode_hold_in_place) {
    // The pose is taken from the measured joints on the cycle the turret *stopped*
    // moving, and then held there. Two properties that only make sense together: no
    // creep (it is not re-read every cycle) and no journey (it is not the ready pose).
    if (!mode_hold_latched_ || last_intent_.type != IntentType::Hold) {
      const double q[2] = {last_positions()[ix(AxisId::Pitch)],
                           last_positions()[ix(AxisId::Yaw)]};
      mode_hold_pitch_rad_ = q[ix(AxisId::Pitch)];
      mode_hold_yaw_rad_ = q[ix(AxisId::Yaw)];
      mode_hold_latched_ = true;
    }
    l.q_yaw_hold_rad = mode_hold_yaw_rad_;
    l.q_pitch_hold_rad = mode_hold_pitch_rad_;
  } else {
    mode_hold_latched_ = false;  // homing / parking / fault will re-latch on return
    l.q_yaw_hold_rad = ready_raw_[ix(AxisId::Yaw)];
    l.q_pitch_hold_rad = ready_raw_[ix(AxisId::Pitch)];
  }
  // §28.5/§31.3: the profiled safe v_max, with any mismatch derate.
  const double cap = hold_speed_effective();
  l.hold_v_max_rad_s = cap;
  l.manual_v_max_rad_s = cap;
  l.track_v_max_rad_s = std::min(tracking_cfg_.track_v_max_rad_s, cap);
  l.roam_v_max_rad_s = std::min(tracking_cfg_.search_v_max_rad_s, cap);
  return l;
}

const char* ControlLoop::mode_phase_label() const {
  // v3 §45's substate vocabulary, translated from the v1 FSM while V3-4/V3-6
  // replace it. Translating rather than showing v1 names is deliberate: the
  // operator learns one vocabulary, and when AutoTrackController arrives it
  // reports these same strings from a real state machine instead of a table.
  if (!tracking_) return "HOLD";
  switch (tracking_->track_state()) {
    case tracking::TrackState::ReadyHold:    return "WAIT_TARGET";
    case tracking::TrackState::Tracking:     return "TRACK";
    case tracking::TrackState::Coasting:     return "COAST";
    case tracking::TrackState::BrakeToHold:  return "LOST_HOLD";
    case tracking::TrackState::TargetLost:   return "LOST_HOLD";
    case tracking::TrackState::Search:
      // The same internal state means two different things depending on the mode
      // (§111.5): a sweep in AUTO_ROAM, a refused one in AUTO_TRACK. Showing
      // "SWEEP" in both would be the UI lying on the FSM's behalf.
      return mode_mgr_.mode() == OperatingMode::AutoRoam ? "SWEEP" : "LOST_HOLD";
  }
  return "HOLD";
}

MotionIntent ControlLoop::build_mode_intent(TimeNs now_ns) const {
  MotionIntent in;
  in.timestamp_ns = now_ns;
  switch (mode_mgr_.mode()) {
    case OperatingMode::Manual: {
      // §38-§41. The lease and the step live in ManualController; what arrives here is
      // either a moving position reference (a jog) or a finite one (a step), both of
      // which go through the same v1 reference, envelope and trajectory generator as
      // everything else. A hold has no deadline; a jog carries one a cycle longer than
      // its lease, so an intent can never outlive the permission it was formed under.
      const MotionIntent& mi = manual_out_.intent;
      if (mi.type != IntentType::Hold) return mi;
      // The run_test_motion developer path (§42.2) is untouched below this switch: that
      // is a developer request in §16's sense, not one of the three modes' motion.
      return MotionIntent::hold(MotionSource::Manual, manual_out_.reason);
    }

    case OperatingMode::AutoTrack: {
      // §15-§20: this answer comes from AutoTrackController, not from the v1 FSM. What
      // is left here is mechanical — turn "follow the predicted LOS at this authority"
      // into an intent, or hold, saying which.
      //
      // The v1 FSM can no longer cause motion in this mode, which is also what finally
      // enforces §111.5: its sweep is reached through a search intent, and nothing here
      // ever asks for one. AUTO_ROAM owns roaming, and the operator owns AUTO_ROAM.
      if (!tracking_)
        return MotionIntent::hold(MotionSource::AutoTrack,
                                  at_out_.reason[0] ? at_out_.reason
                                                    : "no tracking session started");
      if (!at_out_.follow_los || !at_input_.estimator_ready)
        return MotionIntent::hold(MotionSource::AutoTrack,
                                  at_out_.reason[0] ? at_out_.reason : "hold");
      double az = 0.0, el = 0.0;
      tracking_->predicted_los_at_actuation(az, el);  // valid: estimator_ready
      in.source = MotionSource::AutoTrack;
      in.type = IntentType::LosDirection;
      in.has_los = true;
      in.los_az_rad = az;
      in.los_el_rad = el;
      // §19: the derating rides on the intent, so telemetry shows what was asked for
      // beside what was allowed, and an operator can see the turret was deliberately
      // gentle rather than wondering whether it was failing.
      in.velocity_scale = at_out_.velocity_scale;
      in.confidence = at_out_.selected_confidence;
      in.set_reason(at_out_.reason);
      return in;
    }

    case OperatingMode::AutoRoam: {
      // §29-§36: RoamPlanner's intent, verbatim — the waypoint it chose, the elevation
      // it decided to hold, and the reason it gave. The v1 search planner no longer owns
      // motion in any mode (it stays compiled for the legacy path the operator has not
      // retired), which is what §34's "RoamPlanner owns motion" means at this level.
      //
      // Note what is *not* tested here: `tracking_`. A sweep never needed a tracker, and
      // the station has to be able to roam with no camera and nothing in view.
      const MotionIntent& ri = roam_out_.intent;
      if (ri.type != IntentType::Hold) return ri;
      return MotionIntent::hold(MotionSource::AutoRoam,
                                roam_out_.reason[0] ? roam_out_.reason : "roam hold");
    }
  }
  return MotionIntent::hold(MotionSource::None, "unreachable mode value");
}

void ControlLoop::apply_payload_derate(bool derated) {
  payload_derated_ = derated;
  // Cap the safety-envelope v_max (it bounds the tracking reference, §15):
  // derated -> profile-capped hold speed * derate factor.
  double cap = derated ? cfg_.derate_factor * cfg_.hold_speed_rad_s
                       : cfg_.hold_speed_rad_s;
  if (payload_profile_) {
    if (payload_profile_->pitch.v_max_rad_s > 0.0)
      cap = std::min(cap, payload_profile_->pitch.v_max_rad_s);
    if (payload_profile_->yaw.v_max_rad_s > 0.0)
      cap = std::min(cap, payload_profile_->yaw.v_max_rad_s);
  }
  env_.set_v_max(cap);
}

void ControlLoop::start_payload_check(TimeNs now_ns, bool manual,
                                      const AxisSnapshot sp[kAxisCount]) {
  if (phase_ == Phase::Fault || phase_ == Phase::PayloadCheck) return;
  if (!homed_) return;
  // The check takes over the axes: drop any tracking reference first.
  if (tracking_) disable_tracking();
  // The position-mode check needs real torque authority: the post-homing
  // LimitCur (3 A pitch / 1 A yaw) is marginal for a 2 deg step (the yaw
  // creeps at 1 A; the 3 A pitch hold only just holds). Raise BOTH axes to
  // the check current (5 A, under the 10 A station cap) for the check and
  // LEAVE it there — the §33.2/Hold position holds are more authoritative
  // at 5 A, and the boot speed-mode hold already uses this same 5 A.
  for (int i = 0; i < kAxisCount; ++i)
    backend_->set_current_limit(static_cast<AxisId>(i),
                                cfg_.payload_check_current_a);
  // The stock drive speed-loop gains (SpdKp=1.0, SpdKi=0.002) are too weak to
  // hold the position-mode speed limit against a gravity load: on the pitch
  // axis the "against-gravity" half of the 2 deg check step creeps at a
  // fraction of the commanded rate (a few hundred milliamps, ~0.08 deg/s) and
  // never settles in the move budget, while the "with-gravity" half is
  // assisted and is fast. Raise the inner speed loop so it builds the torque
  // needed to hold the commanded rate against gravity and the step response is
  // the drive's controlled (mass-sensitive) response, not a gravity-dominated
  // creep. Capped by the current/torque limits, so safe. No-op in sim.
  for (int i = 0; i < kAxisCount; ++i)
    backend_->set_speed_loop_gains(static_cast<AxisId>(i),
                                   cfg_.payload_check_spd_kp,
                                   cfg_.payload_check_spd_ki);
  payload_check_cfg_ = payload::PayloadCheckConfig{};
  payload_check_cfg_.step_amplitude_rad = cfg_.payload_check_step_deg * kDeg2Rad;
  payload_check_cfg_.speed_rad_s = cfg_.payload_check_speed_deg_s * kDeg2Rad;
  const payload::PayloadProfile* prof =
      payload_profile_ ? &*payload_profile_ : nullptr;
  // The safe region is PER-AXIS, centered on that axis's CURRENT pose
  // (§44 "safe central region"): the check starts where the station holds
  // (the ready pose), which is not the static config default (0 +/- 20 deg)
  // on this rig (ready: pitch -1.496 rad, yaw -2.261 rad) — a shared static
  // region would make begin() fail "start pose outside the safe central
  // region". Intersecting with the homed soft limits keeps the region clear
  // of the travel stops; if the pose is too close to a stop the region
  // shrinks and begin() fails the min-amplitude guard instead of stepping
  // toward the boundary.
  for (int i = 0; i < kAxisCount; ++i) {
    payload_checks_[i] = payload::PayloadCheck(
        payload_check_cfg_, prof, static_cast<AxisId>(i));
    const double c = sp[i].has_feedback ? sp[i].q_rad : 0.0;
    double half_span = cfg_.payload_check_region_half_span_deg * kDeg2Rad;
    if (limits_[i].valid)
      half_span = std::min(
          half_span, std::min(c - limits_[i].q_soft_min_rad,
                              limits_[i].q_soft_max_rad - c));
    payload_checks_[i].set_region(c, half_span);
    // FIXED hold target: this axis's position right now. For the whole check
    // the axis is commanded to this (fixed) target with a NONZERO speed limit
    // so the position loop actively holds it against drift; re-pinning to the
    // live position would just follow the drift. The active axis's reference
    // is overridden each cycle by the check stepper (see the Phase::
    // PayloadCheck handler).
    payload_hold_target_[i] = c;
  }
  payload_axis_ix_ = 0;
  payload_check_manual_ = manual;
  phase_ = Phase::PayloadCheck;
  telemetry_.push_event(now_ns, telemetry::Event::PayloadVerifyStarted,
                        manual ? "manual (start_payload_verification)"
                               : "auto (§27 optional stage)");
  spdlog::info("payload check started (manual={}, profile={})", manual,
               payload_profile_ ? payload_profile_->name : std::string("none"));
}

void ControlLoop::finish_payload_check(TimeNs now_ns) {
  payload::VerifyResult vr;
  for (int i = 0; i < kAxisCount; ++i) vr.axes[i] = payload_checks_[i].axis_result();
  const bool profile_loaded = payload_profile_.has_value();
  vr.status = payload::overall_status(vr.axes, profile_loaded);
  std::string detail = payload::payload_status_name(vr.status);
  for (int i = 0; i < kAxisCount; ++i)
    for (const auto& v : vr.axes[i].violations) detail += "; " + v;
  vr.detail = detail;
  const auto d = payload::decide(vr, cfg_.derate_factor);
  payload_status_ = vr.status;
  payload_detail_ = d.action + (detail != payload::payload_status_name(vr.status)
                                    ? " — " + detail
                                    : std::string());
  // Audit trail of the MEASURED response (what a commissioned profile's
  // baseline must match, §28.5) — logged at info on every completed check.
  for (int i = 0; i < kAxisCount; ++i) {
    const auto& ax = vr.axes[i];
    if (!ax.measured) continue;
    const auto& p = ax.step_pos;
    const auto& n = ax.step_neg;
    spdlog::info(
        "payload check measured [{}]: +step amp={:.4f} rad rise={:.3f} s "
        "overshoot={:.4f} settle={:.3f} s rms={:.5f} rad peak_effort={:.3f} Nm; "
        "-step amp={:.4f} rad rise={:.3f} s overshoot={:.4f} settle={:.3f} s "
        "rms={:.5f} rad peak_effort={:.3f} Nm",
        i == 0 ? "pitch" : "yaw", p.amplitude_rad, p.rise_time_s, p.overshoot,
        p.settling_time_s, p.tracking_rms_rad, p.peak_effort_nm,
        n.amplitude_rad, n.rise_time_s, n.overshoot, n.settling_time_s,
        n.tracking_rms_rad, n.peak_effort_nm);
  }
  if (d.derate != payload_derated_) apply_payload_derate(d.derate);
  phase_ = Phase::Hold;
  if (vr.status == payload::PayloadStatus::Ok)
    telemetry_.push_event(now_ns, telemetry::Event::PayloadVerifyOk, detail);
  else if (vr.status == payload::PayloadStatus::Mismatch)
    telemetry_.push_event(now_ns, telemetry::Event::PayloadVerifyMismatch, detail);
  else
    telemetry_.push_event(now_ns, telemetry::Event::PayloadVerifyError, detail);
  spdlog::info("payload check complete: {} (derated={})", payload_detail_,
               payload_derated_);
}

void ControlLoop::abort_payload_check(TimeNs now_ns, const std::string& reason) {
  payload_status_ = payload::PayloadStatus::Error;
  payload_detail_ = reason;
  phase_ = Phase::Hold;
  telemetry_.push_event(now_ns, telemetry::Event::PayloadVerifyError, reason);
  spdlog::warn("payload check aborted: {}", reason);
}

// ---------------------------------------------------------------------------
// Phase 8: developer-command plumbing (§42.2).
// ---------------------------------------------------------------------------

web::CommandResult ControlLoop::submit_command(const std::string& name,
                                               const std::string& arg) {
  std::lock_guard<std::mutex> lk(command_mutex_);
  web::CommandResult r = web::validate_command(command_state_, name, arg);
  if (r.ok && name == "select_payload_profile") {
    // Reject an unknown profile SYNCHRONOUSLY. The web response is written
    // before the control thread executes anything, so an "ok" that later turns
    // into a log-only warning would lie to the operator (§42.1: what the UI
    // shows must be what the station did). This is a cheap stat() on the
    // CALLER's thread (the web server thread) — never on the control thread
    // (§46) — and `arg` is already name-validated by web::validate_command
    // (no '/', no "..", length-capped), so the path cannot escape the store.
    const std::string path =
        payload_profile_dir_.empty() ? std::string()
                                     : payload_profile_dir_ + "/" + arg + ".yaml";
    if (path.empty() || ::access(path.c_str(), R_OK) != 0) {
      r.ok = false;
      r.error = "no payload profile named '" + arg + "' in " +
                (payload_profile_dir_.empty() ? std::string("<unset>")
                                              : payload_profile_dir_);
    }
  }
  // Rejected here or executed there, either way controld answers (§52).
  command_queue_.push_back({name, arg, r.ok ? std::string() : r.error});
  return r;
}

void ControlLoop::process_commands() {
  std::deque<PendingCommand> cmds;
  {
    std::lock_guard<std::mutex> lk(command_mutex_);
    cmds.swap(command_queue_);
  }
  for (auto& c : cmds) {
    if (!c.gate_reject.empty()) {
      ack_command(c.name, false, "web gate: " + c.gate_reject);
      continue;
    }
    ack_in_flight_ = c.name;
    const uint64_t before = ack_seq_;
    execute_command(c.name, c.arg);
    if (ack_seq_ == before) {
      // Nobody claimed the answer, so this command has no failure path: the quiet
      // hold, the stop, the shutdown request. That default is honest only because
      // every path that CAN fail now answers for itself — which is why the no-op
      // commands below answer explicitly rather than inheriting "accepted" the
      // way v1's blanket acknowledge let them.
      ack_command(c.name, true, "accepted");
    }
    ack_in_flight_.clear();
  }
}

void ControlLoop::feed_track_set(const tracks::TrackSet& set, TimeNs receive_ns) {
  // The ingest thread's entire contribution: copy the frame's tracks across. Nothing
  // here decides anything. Selection, visibility and §21's reacquisition all read
  // control-thread state, and a socket thread running them would race the control
  // cycle that is using the same answers to move a turret.
  (void)receive_ns;  // §61 stamps are recorded by VisionLink, which owns that clock
  std::lock_guard<std::mutex> lk(measurement_mutex_);
  pending_set_ = set;
  has_pending_set_ = true;
}

const tracks::Track* ControlLoop::apply_track_set(const tracks::TrackSet& set,
                                                  TimeNs now) {
  last_set_receive_ns_ = now;
  // §17's chain, entered from the v3 door: the set is observed, a track is chosen, and
  // the result is handed to the v1 estimator as a pixel measurement. Everything from
  // "pixel" rightwards — ray, motor interpolation at the SensorTimestamp, LOS, the
  // TargetEstimator, the joint solver — is the v1 that has been tested on this
  // hardware (§111.18), which is the reason this function ends in a TargetMeasurement
  // instead of a parallel path beside it.
  selection_.observe(set, now);

  // Which candidate's measurement primes the estimator. With a selection, that is the
  // selected track and nothing else. Without one, it is the best-scoring CONFIRMED
  // person under v1's gates — which is a *measurement source*, not a decision to move:
  // §16 gives motion only to a selection, and AutoTrackController is what enforces that
  // (WAIT_TARGET holds). Priming is what lets a selection made at frame 400 act on
  // frame 401 instead of waiting out the estimator's own warm-up.
  //
  // Why it is still here rather than deleted: the estimator is v1's, and an estimator
  // with no history makes the first seconds after a selection worse, not better. Why it
  // is not a fallback for *motion*: a rule that grabs the highest-confidence target is
  // exactly how a turret ends up tracking a stranger after the selected one walks
  // behind a pillar. v1's class and confidence gates are kept because §14's validation
  // shares them, and a station that primes itself on any class the detector knows about
  // would be a wider target appetite than v1 ever had.
  const tracks::Track* pick = selection_.selected_track();
  const bool from_selection = pick != nullptr;
  if (!from_selection && !selection_.has_selection()) {
    constexpr int32_t kPreferredClassId = 1;  // v1 preferred_class_id: 'person'
    constexpr float kMinConfidence = 0.5f;    // v1 confidence_threshold
    double best_area = -1.0;
    for (int i = 0; i < set.count; ++i) {
      const tracks::Track& t = set.tracks[i];
      if (t.state != tracks::TrackState::Confirmed) continue;  // §8: only CONFIRMED
      if (t.class_id != kPreferredClassId) continue;
      if (t.track_confidence < kMinConfidence) continue;
      const double area = double(t.bbox.x_max - t.bbox.x_min) *
                          double(t.bbox.y_max - t.bbox.y_min);
      const double cand = double(t.track_confidence);
      const double cur = pick ? double(pick->track_confidence) : -1.0;
      // Confidence first, then size (a nearer, larger target is the better anchor for
      // LOS), then the smaller uuid — so two identical candidates resolve to the same
      // one every time instead of flickering between them frame to frame.
      if (!pick || cand > cur + 1e-6 ||
          (std::fabs(cand - cur) <= 1e-6 && area > best_area + 1e-9)) {
        pick = &t;
        best_area = area;
      }
    }
  }

  vision::TargetMeasurement m;  // invalid == "no target this frame" (§6.2)
  m.frame_sequence = set.frame_sequence;
  m.sensor_timestamp_ns = set.sensor_timestamp_ns;
  if (pick != nullptr) {
    if (set.width == 0 || set.height == 0) {
      // The §9 anchor is normalized (§60), so turning it into a pixel needs the
      // resolution it was normalized against. Without it the honest answer is "no
      // target": guessing a resolution would produce a confident, wrong LOS.
      static bool warned_geometry = false;
      if (!warned_geometry) {
        warned_geometry = true;
        spdlog::warn("TrackSet from visiond carries width=0/height=0: normalized "
                     "anchors cannot become pixels. Treating as no target until the "
                     "publisher fills them in (one warning).");
      }
      pick = nullptr;
    } else {
      m.valid = true;
      m.class_id = pick->class_id;
      m.confidence = pick->track_confidence;
      m.bbox_x_min_norm = pick->bbox.x_min;
      m.bbox_y_min_norm = pick->bbox.y_min;
      m.bbox_x_max_norm = pick->bbox.x_max;
      m.bbox_y_max_norm = pick->bbox.y_max;
      m.anchor_u_px = pick->anchor_x * static_cast<float>(set.width);
      m.anchor_v_px = pick->anchor_y * static_cast<float>(set.height);
      m.has_track_id = true;
      m.visual_track_id = pick->uuid.lo;
    }
  }
  // §19/§15: the facts the AUTO_TRACK state machine works from, refreshed once per
  // frame. Anything not refreshed is stale by construction — a track that stopped being
  // published is not counted visible, which is the whole basis of §20.
  {
    const auto& sel = selection_.selection();
    const bool visible = sel.visibility_state == tracks::Visibility::Visible;
    at_input_.just_reacquired =
        visible && !at_was_visible_ && sel.reacquisition_score > 0.0f;
    at_was_visible_ = visible;
    at_input_.has_selection = sel.has_selection;
    at_input_.target_visible = visible;
    at_input_.target_occluded = sel.visibility_state == tracks::Visibility::Occluded;
    at_input_.ambiguous = sel.ambiguous_reacquisition;
    at_input_.reacquisition_score = sel.reacquisition_score;
    if (pick != nullptr) {
      at_input_.detector_confidence = pick->detector_confidence;
      at_input_.track_confidence = pick->track_confidence;
      at_input_.visible_frames = pick->visible_frames;
      at_input_.missing_frames = pick->missing_frames;
    } else {
      // No target in hand: the confidence inputs go to zero rather than keeping the last
      // target's numbers, which would hold the state machine in a band it no longer has
      // evidence for.
      at_input_.detector_confidence = 0.0f;
      at_input_.track_confidence = 0.0f;
      at_input_.visible_frames = 0;
      at_input_.missing_frames = 0;
    }
  }
  if (m.valid) last_measurement_ns_ = now;

  if (pick != nullptr && from_selection) {
    static tracks::TrackUuid logged{};
    if (!(logged == pick->uuid)) {
      logged = pick->uuid;
      spdlog::info("following the selected target {} ({}) — sequence {}",
                   selection_.selection().selected_descriptor, pick->uuid.lo,
                   set.frame_sequence);
    }
  }
  // Applied here rather than queued through feed_measurement(), which is the ingest
  // thread's hand-off and would add a cycle of delay for no reason — this function is
  // already running on the control thread, in the same place the v1 path applies its
  // measurement. Queuing it also meant a measurement could be overtaken by the next
  // frame's set and never reach the estimator at all: invisible at 30 Hz against
  // 200 Hz, and exactly the kind of one-in-five-cycles bug that only shows up live.
  //
  // Nothing is applied while tracking is off, matching the rule the v1 path has always
  // enforced: a stale frame must not be waiting to fire the instant tracking starts.
  if (tracking_ && m.valid) tracking_->set_measurement(m);
  return pick;
}

void ControlLoop::ack_command(const std::string& name, bool accepted,
                              const std::string& why) {
  ++ack_seq_;
  last_ack_.command = name;
  last_ack_.accepted = accepted;
  last_ack_.reason = why;
  last_ack_.controller_state =
      std::string(phase_name(phase_)) + "/" + operating_mode_name(mode_mgr_.mode());
  last_ack_.safety_state = safety_action_name(last_decision_.action);
  last_ack_.seq = ack_seq_;
  if (!accepted)
    spdlog::warn("command '{}' rejected: {} (state {}, safety {})", name, why,
                 last_ack_.controller_state, last_ack_.safety_state);
}

void ControlLoop::disable_tracking() {
  tracking_.reset();
  tracking_ref_ = ReferenceRequest{};
}

void ControlLoop::execute_command(const std::string& name,
                                  const std::string& arg) {
  std::string err;
  if (name == "hold") {
    // v3 §2: HOLD is not a mode, it is what MANUAL does when nothing is asked of
    // it. The command stays — every script and habit uses it — but it now has to
    // *change the mode*, because dropping the tracking reference while AUTO_TRACK
    // is still authoritative would leave it re-acquiring on the next frame and
    // moving again. That restart-by-design is the stale intent §93 is about, and
    // this button used to be reachable proof of it.
    request_mode(OperatingMode::Manual);
    return;
  }
  if (name == "set_mode") {
    OperatingMode target;
    if (!operating_mode_from_name(arg.c_str(), target)) {
      ack_command(name, false, "unknown mode '" + arg +
                                  "'; expected MANUAL, AUTO_TRACK or AUTO_ROAM");
      return;
    }
    request_mode(target);
    return;
  }
  if (name == "stop_motion") {
    stop_motion();
    return;
  }
  if (name == "start_tracking") {
    // v3: this is SET_MODE(AUTO_TRACK). Kept as an alias because §16's old
    // semantics ("start tracking" as a separate action from "be in the tracking
    // mode") are what created the class of bug where a command reports success
    // and the mode never changes: it enabled a controller without taking
    // authority, so the operator's screen and the turret disagreed about who was
    // driving. Uses the COMMISSIONED configuration (turret.yaml `tracking:`
    // block + the calibration files), not built-in defaults (Part 2, S1).
    request_mode(OperatingMode::AutoTrack);
    return;
  }
  if (name == "stop_tracking") {
    disable_tracking();
    return;
  }
  if (name == "request_park") {
    if (!start_parking(err)) ack_command(name, false, "park rejected: " + err);
    return;
  }
  if (name == "request_shutdown") {
    shutdown_requested_.store(true);
    return;
  }
  if (name == "run_test_motion") {
    // Restricted one-shot move to `arg` radians on the yaw axis (already
    // envelope-checked by validate_command). The reference manager will clamp
    // it against the soft limits on the next cycle as well.
    test_motion_target_rad_ = std::stod(arg);
    has_test_motion_ = true;
    return;
  }
  if (name == "start_payload_verification") {
    // Phase 9 (§31.3, §42.2): begin the payload response check. The web gate
    // already required homed && !fault && !moving; the actual start happens in
    // step() (2c) so it runs on the control thread with the cycle timestamp.
    if (phase_ == Phase::Hold) {
      payload_check_requested_ = true;
    } else {
      ack_command(name, false,
                  std::string("payload check needs Hold; phase is ") +
                      phase_name(phase_));
    }
    return;
  }
  if (name == "select_payload_profile") {
    // §42.2 runtime profile switch (P6 follow-up: the mismatch -> clear cycle
    // used to require a config edit + a daemon restart). The new profile caps
    // station motion immediately; its status stays no_profile until the
    // operator runs start_payload_verification (§31.3), so selecting an
    // unverified profile can never silently raise a limit on trust.
    if (phase_ == Phase::PayloadCheck) {
      ack_command(name, false,
                  "a payload check is running; not swapping the profile mid-check "
                  "(§44)");
      return;
    }
    payload::PayloadProfileStore store(payload_profile_dir_);
    payload::PayloadProfile prof;
    std::string perr;
    if (!store.load(arg, prof, perr)) {
      ack_command(name, false, "profile '" + arg + "' not loaded: " + perr);
      return;
    }
    set_payload_profile(std::move(prof), /*commissioned=*/false);
    spdlog::info("payload profile: selected '{}' at runtime (status={}, "
                 "v_max pitch={:.1f} deg/s yaw={:.1f} deg/s); run "
                 "start_payload_verification to confirm it (§31.3)",
                 arg, payload::payload_status_name(payload_status_),
                 payload_profile_->pitch.v_max_rad_s / kDeg2Rad,
                 payload_profile_->yaw.v_max_rad_s / kDeg2Rad);
    return;
  }
  if (name == "enable_search" || name == "disable_search") {
    // See enable_tracking(): this used to be acknowledged and dropped. It arms
    // or disarms the sweep for the NEXT start_tracking. It deliberately cannot
    // retune a session already running — the FSM lives inside
    // TrackingController and rebuilding it mid-flight would throw away the
    // acquisition history — and "applies next time" is stated rather than left
    // for the operator to discover by watching a turret that keeps sweeping.
    const bool want = (name == "enable_search");
    search_override_ = want;
    if (tracking_) {
      // Live: the flag is consulted at the next transition, so arming it while
      // holding starts the sweep on the next cycle, and disarming it while
      // sweeping stops sweeping (the FSM hands back to the hold). Both are the
      // point of the buttons; a version that only affected a future session
      // could not be used to stop a turret that is already roaming.
      tracking_->set_search_enabled(want);
      spdlog::info("search mode {} for the RUNNING session (§36)",
                   want ? "ARMED" : "disarmed — returning to the hold");
    } else {
      spdlog::info("search mode {} for the next start_tracking (§36)",
                   want ? "ARMED" : "disarmed");
    }
    return;
  }
  if (name == "manual_jog_start" || name == "manual_jog_keepalive" ||
      name == "manual_jog_stop" || name == "manual_step") {
    // §38-§41. Every one of these is refused outside MANUAL with the reason §52 names
    // ("manual jog only available in MANUAL mode") — a jog that quietly started a
    // motion while AUTO_TRACK believed it owned the axes is the arbitration failure §26
    // exists to prevent, so the mode is checked here rather than assumed.
    if (mode_mgr_.mode() != OperatingMode::Manual) {
      ack_command(name, false,
                  "manual motion is only available in MANUAL (currently " +
                      std::string(operating_mode_name(mode_mgr_.mode())) + ")");
      return;
    }
    if (name == "manual_jog_keepalive") {
      // Answered honestly: a keepalive that renews nothing means the lease already
      // lapsed, and the operator holding a button needs to know that rather than
      // watching the turret refuse to move.
      const bool renewed = manual_.jog_keepalive(now_ns_);
      ack_command(name, renewed, renewed ? "lease renewed"
                                         : "no jog lease to renew (it expired)");
      return;
    }
    if (name == "manual_jog_stop") {
      manual_.jog_stop(now_ns_);
      ack_command(name, true, "jog stopped");
      return;
    }
    if (name == "manual_jog_start") {
      JogDirection dir;
      ManualProfile profile = manual_.profile();
      char why[96] = {};
      if (!ManualController::parse_jog_arg(arg.c_str(), dir, profile, why, sizeof why)) {
        ack_command(name, false, why);
        return;
      }
      if (!manual_.jog_start(dir, profile, now_ns_)) {
        ack_command(name, false, "jog direction was empty");
        return;
      }
      spdlog::info("MANUAL jog {}{} profile {} (lease {} ms)",
                   dir.yaw ? (dir.yaw > 0 ? "yaw+ " : "yaw- ") : "",
                   dir.pitch ? (dir.pitch > 0 ? "pitch+ " : "pitch- ") : "",
                   manual_profile_name(profile), manual_.config().lease_ms);
      ack_command(name, true,
                  std::string("jogging at ") + manual_profile_name(profile));
      return;
    }
    // manual_step <axis><sign><degrees>, e.g. "yaw+1" or "pitch-0.5"
    std::string a = arg;
    size_t k = 0;
    while (k < a.size() && ((a[k] >= 'a' && a[k] <= 'z') || (a[k] >= 'A' && a[k] <= 'Z')))
      ++k;
    const std::string axis_name = a.substr(0, k);
    const std::string rest = a.substr(k);
    if (axis_name.empty() || rest.empty()) {
      ack_command(name, false, "step needs an axis and degrees (yaw+1, pitch-0.5)");
      return;
    }
    int axis = -1;
    if (axis_name == "yaw" || axis_name == "yaw_axis") axis = 1;
    else if (axis_name == "pitch") axis = 0;
    if (axis < 0) {
      ack_command(name, false, "step axis must be yaw or pitch");
      return;
    }
    double sign = 1.0;
    size_t off = 0;
    if (!rest.empty() && (rest[0] == '+' || rest[0] == '-')) {
      sign = rest[0] == '-' ? -1.0 : 1.0;
      off = 1;
    }
    double deg = 0.0;
    try {
      deg = std::stod(rest.substr(off));
    } catch (...) {
      ack_command(name, false, "step degrees could not be read");
      return;
    }
    // §41's choices are 0.5 / 1 / 5 degrees. Anything larger is refused rather than
    // performed: an unbounded "move N degrees" on the operator page is the raw test
    // move the section says not to build, and a typo of one digit is a turret crossing
    // the room. §72 can widen the list at commissioning.
    const double allowed[3] = {0.5, 1.0, 5.0};
    bool ok_size = false;
    for (double d : allowed)
      if (std::fabs(deg - d) < 1e-9) ok_size = true;
    if (!ok_size) {
      ack_command(name, false, "step size must be one of 0.5, 1, or 5 degrees");
      return;
    }
    // Same trap as the jog: the step is relative to where the turret *is*. Reading the
    // reference here (cleared each cycle by section 2b) would make every step relative
    // to joint zero, so a station at 90 degrees asked for "+1" would be driven to 1
    // degree — and in a simulation that starts near zero, both numbers look the same.
    const double q_logical = last_positions()[axis];
    if (!manual_.step_move(axis, sign * deg * 0.017453292519943295, q_logical,
                           now_ns_)) {
      ack_command(name, false, "step was malformed");
      return;
    }
    ack_command(name, true, "step issued to v1 safety (41)");
    return;
  }
  if (name == "select_target" || name == "clear_target") {
    // §14. Validation is controld's, and this is the thread that can do it honestly:
    // the answer is computed against the TrackSet actually in hand, not against what
    // the web thread happened to see a moment ago. Reasons are the operator's — they
    // are acked verbatim, which is the only reason a refusal is survivable at all.
    if (name == "clear_target") {
      auto r = selection_.clear(now_ns_);
      if (r.changed) autotrack_.reset();  // §16: back to WAIT_TARGET
      spdlog::info("target selection: {}", r.reason);
      ack_command(name, r.ok, r.reason);
      return;
    }
    unsigned index = 0;
    try {
      index = static_cast<unsigned>(std::stoul(arg));
    } catch (...) {
      ack_command(name, false, "target id must be a number (the label on the screen)");
      return;
    }
    if (index == 0 || index > 65535) {
      ack_command(name, false, "target id must be a positive label number");
      return;
    }
    auto r = selection_.select_by_display_index(
        static_cast<uint16_t>(index), now_ns_);
    if (r.changed) autotrack_.reset();  // §15: acquisition restarts for a new subject
    spdlog::info("select_target {}: {} ({})", index, r.ok ? "ACCEPTED" : "REFUSED",
                 r.reason);
    ack_command(name, r.ok, r.reason);
    return;
  }
  if (name == "start_homing" || name == "start_installation_calibration") {
    // Both are advertised on the dashboard and neither is acted on: homing comes
    // from the boot sequence, and visual calibration is an offline tool that needs
    // a printed board (§29, P9). Acking them as accepted would keep a button that
    // does nothing looking like a button that worked, so until each is wired
    // deliberately or removed from the UI it says what it is.
    ack_command(name, false,
                name == "start_homing"
                    ? "controld does not start homing from a web command in this "
                      "build; homing runs from the boot sequence"
                    : "visual calibration is run by the offline tool "
                      "vision/installation_calibration.py, not by controld");
    return;
  }
  ack_command(name, false, "unknown command '" + name + "'");
}

}  // namespace ota
