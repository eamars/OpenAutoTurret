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
    {
      std::lock_guard<std::mutex> lk(measurement_mutex_);
      if (has_pending_measurement_) {
        m = pending_measurement_;
        has_pending_measurement_ = false;
        have = true;
      }
    }
    // Consume (and drop) measurements while tracking is off so a stale frame
    // cannot be applied the instant tracking is enabled.
    if (have && tracking_) tracking_->set_measurement(m);
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
  if (tracking_ && phase_ == Phase::Hold) {
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
    last_intent_ = build_mode_intent(now_ns);
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
    tracking_->record_pose(sp[ix(AxisId::Yaw)].q_rad);
    tracking_->record_reference(tracking_ref_);
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
      // Move to the safe ready pose (if not already there), then hold. Never
      // press against a stop.
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
        if (std::fabs(sp[i].q_rad - ready_raw_[i]) > kReadyPosTolRad) {
          q_ref[i] = ready_raw_[i];
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
    // Vision transport (observe-only; the ingest thread owns these counters).
    if (vision_link_) {
      const vision::VisionLink::Stats vs = vision_link_->stats();
      snap.vision_connected = vs.connected;
      snap.vision_frames = vs.frames;
      snap.vision_dropped = vs.dropped;
      snap.vision_last_frame_sequence = vs.last_frame_sequence;
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
  c.roam_envelope_valid =
      tracking_cfg_.search.yaw_high_rad > tracking_cfg_.search.yaw_low_rad;
  c.supervisory = mode_mgr_.supervisory();
  return c;
}

ModeResult ControlLoop::request_mode(OperatingMode target) {
  const ModeResult r = mode_mgr_.request(target, mode_context());
  if (!r.ok) {
    spdlog::warn("mode change to {} refused: {}", operating_mode_name(target),
                 r.reason);
    return r;
  }
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
  sync_controllers_to_mode(OperatingMode::Manual);
  spdlog::warn("STOP_MOTION: {} intent cancelled, controlled hold in MANUAL "
               "(§27; not a disable, not a shutdown)", operating_mode_name(was));
  return r;
}

void ControlLoop::sync_controllers_to_mode(OperatingMode mode) {
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
  l.q_yaw_hold_rad = ready_raw_[ix(AxisId::Yaw)];
  l.q_pitch_hold_rad = ready_raw_[ix(AxisId::Pitch)];
  // §28.5/§31.3: the profiled safe v_max, with any mismatch derate.
  const double cap = hold_speed_effective();
  l.hold_v_max_rad_s = cap;
  l.manual_v_max_rad_s = cap;
  l.track_v_max_rad_s = std::min(tracking_cfg_.track_v_max_rad_s, cap);
  l.roam_v_max_rad_s = std::min(tracking_cfg_.search_v_max_rad_s, cap);
  return l;
}

MotionIntent ControlLoop::build_mode_intent(TimeNs now_ns) const {
  MotionIntent in;
  in.timestamp_ns = now_ns;
  switch (mode_mgr_.mode()) {
    case OperatingMode::Manual:
      // V3-5 replaces this with jog/step/goto intents from ManualController. The
      // run_test_motion developer path (§42.2) is untouched below it: that is a
      // developer request in §16's sense, not one of the three modes' motion.
      return MotionIntent::hold(MotionSource::Manual, "manual hold");

    case OperatingMode::AutoTrack: {
      if (!tracking_)
        return MotionIntent::hold(MotionSource::AutoTrack,
                                  "no tracking session started");
      const tracking::TrackState st = tracking_->track_state();
      if (st == tracking::TrackState::Tracking ||
          st == tracking::TrackState::Coasting) {
        if (!tracking_->estimator_initialized())
          return MotionIntent::hold(MotionSource::AutoTrack,
                                    "estimator not initialised");
        double az = 0.0, el = 0.0;
        tracking_->predicted_los_at_actuation(az, el);
        in.source = MotionSource::AutoTrack;
        in.type = IntentType::LosDirection;
        in.has_los = true;
        in.los_az_rad = az;
        in.los_el_rad = el;
        // §19/§35: confidence derates, and it arrives as a scale on the intent so
        // telemetry can show what was asked for next to what was allowed.
        const double c = std::max(0.0, std::min(1.0, tracking_->confidence()));
        in.velocity_scale = c;
        in.confidence = c;
        in.set_reason(track_state_name(st));
        return in;
      }
      if (st == tracking::TrackState::Search) {
        // §111.5, and the reason this branch has to exist explicitly: a station
        // that lost its target in AUTO_TRACK must NOT start sweeping the
        // workspace looking for any target it happens to find. That is
        // AUTO_ROAM's job, and the operator chooses it. Without this branch the
        // inherited v1 search planner would happily keep sweeping, and the two
        // modes would mean the same thing.
        return MotionIntent::hold(MotionSource::AutoTrack,
                                  "target absent; roaming needs AUTO_ROAM (111.5)");
      }
      return MotionIntent::hold(MotionSource::AutoTrack,
                                track_state_name(st));
    }

    case OperatingMode::AutoRoam: {
      if (!tracking_)
        return MotionIntent::hold(MotionSource::AutoRoam,
                                  "sweep not started");
      if (tracking_->track_state() != tracking::TrackState::Search ||
          mode_proposal_.source != ReferenceSource::Search) {
        // Includes the lost-timeout grace before the sweep begins, and the
        // turnaround dwells. §36's "do not jump to a distant scan start" is
        // satisfied by the planner being re-seeded from the current yaw; RoamPlanner
        // owns that properly in V3-6.
        return MotionIntent::hold(MotionSource::AutoRoam,
                                  track_state_name(tracking_->track_state()));
      }
      in.source = MotionSource::AutoRoam;
      in.type = IntentType::JointPosition;
      in.has_joint_target = true;
      in.q_yaw_rad = mode_proposal_.q_yaw_rad;
      in.q_pitch_rad = mode_proposal_.q_pitch_rad;
      in.confidence = 1.0;  // no target involved; the sweep is not uncertain
      in.set_reason("bounded sweep");
      return in;
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
  if (r.ok) command_queue_.emplace_back(name, arg);
  return r;
}

void ControlLoop::process_commands() {
  std::deque<std::pair<std::string, std::string>> cmds;
  {
    std::lock_guard<std::mutex> lk(command_mutex_);
    cmds.swap(command_queue_);
  }
  for (auto& c : cmds) execute_command(c.first, c.second);
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
      spdlog::warn("set_mode rejected: unknown mode '{}' (expected MANUAL, "
                   "AUTO_TRACK or AUTO_ROAM)", arg);
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
    if (!start_parking(err)) spdlog::warn("request_park rejected: {}", err);
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
    if (phase_ == Phase::Hold) payload_check_requested_ = true;
    else spdlog::warn("start_payload_verification: not in hold (phase={})",
                      phase_name(phase_));
    return;
  }
  if (name == "select_payload_profile") {
    // §42.2 runtime profile switch (P6 follow-up: the mismatch -> clear cycle
    // used to require a config edit + a daemon restart). The new profile caps
    // station motion immediately; its status stays no_profile until the
    // operator runs start_payload_verification (§31.3), so selecting an
    // unverified profile can never silently raise a limit on trust.
    if (phase_ == Phase::PayloadCheck) {
      spdlog::warn("select_payload_profile: a payload check is running; not "
                   "swapping the profile mid-check (§44)");
      return;
    }
    payload::PayloadProfileStore store(payload_profile_dir_);
    payload::PayloadProfile prof;
    std::string perr;
    if (!store.load(arg, prof, perr)) {
      spdlog::warn("select_payload_profile: '{}' not loaded ({})", arg, perr);
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
  if (name == "select_target" || name == "start_homing" ||
      name == "start_installation_calibration") {
    // These are validated here but acted on by the tracking/vision subsystem or
    // an external tool (see webd). Acknowledge so the UI can proceed.
    return;
  }
  spdlog::warn("unhandled web command: {}", name);
}

}  // namespace ota
