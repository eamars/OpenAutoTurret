// OpenAutoTurret — ControlLoop implementation (§46 per-cycle engine).
#include "control/control_loop.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

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

bool ControlLoop::start_homing(HomingPlan plan, std::string& err) {
  if (phase_ == Phase::Fault) {
    err = "in fault; reset required";
    return false;
  }
  homing_.reset(new HomingPlan(std::move(plan)));
  std::string e;
  if (!enter_position_mode_all(cfg_.hold_speed_rad_s, e)) {
    err = e;
    return false;
  }
  homed_ = false;
  at_ready_ = false;
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
  std::string e;
  if (!enter_position_mode_all(cfg_.park.speed_deg_s * kDeg2Rad, e)) {
    err = e;
    return false;
  }
  phase_ = Phase::Parking;
  return true;
}

void ControlLoop::deenergize_all() {
  for (int i = 0; i < kAxisCount; ++i) backend_->deenergize(static_cast<AxisId>(i));
}

bool ControlLoop::enable_tracking(const TrackingController::Config& cfg,
                                  std::string& err) {
  if (tracking_) {
    err = "tracking already enabled";
    return false;
  }
  if (!homed_) {
    err = "cannot enable tracking: not homed (position validity unknown, §38.1)";
    return false;
  }
  tracking_.reset(new TrackingController(cfg));
  return true;
}

void ControlLoop::feed_measurement(const vision::TargetMeasurement& m) {
  if (!tracking_) return;
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

HomingFeedback ControlLoop::to_feedback(const AxisSnapshot& s) {
  HomingFeedback fb;
  fb.t_ns = s.rx_ns;
  fb.pos_rad = s.q_rad;
  fb.vel_rad_s = s.v_rad_s;
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
  }

  // 1a. Phase 8: execute any developer commands submitted via the web UI
  //     (§42.2). They were validated against the previous cycle's state on the
  //     web thread; execution happens HERE on the control thread so all state
  //     mutation stays single-threaded.
  process_commands();

  // 1b. Phase 6: feed the tracking controller the current pose (for §11
  //      timestamp interpolation) and consume any pending visiond measurement.
  if (tracking_) {
    tracking_->update_snapshots(now_ns, sp[ix(AxisId::Pitch)].q_rad,
                                sp[ix(AxisId::Yaw)].q_rad);
    if (has_pending_measurement_) {
      tracking_->set_measurement(pending_measurement_);
      has_pending_measurement_ = false;
    }
  }

  // 2. §39.3 deadline watchdog: a cycle longer than the control period is a
  //    miss; consecutive misses feed the supervisor.
  int64_t overrun_us = 0;
  if (period_ns > deadline_ns_) {
    overrun_us = (period_ns - deadline_ns_) / 1000;
    ++deadline_miss_count_;
  } else {
    deadline_miss_count_ = 0;
  }

  // 2b. Phase 6: compute the tracking reference (only while tracking is active,
  //     i.e. tracking enabled and in the ready-hold operation). This must
  //     precede the supervisor input so `tracking_enabled` reflects the cycle.
  tracking_ref_ = ReferenceRequest{};
  if (tracking_ && phase_ == Phase::Hold) {
    tracking_ref_ = tracking_->compute_reference(
        now_ns, ready_raw_[ix(AxisId::Pitch)], ready_raw_[ix(AxisId::Yaw)]);
    // §28.5/§31.3: the payload profile v_max (and any mismatch derate) caps
    // the tracking speed.
    tracking_ref_.v_max_rad_s =
        std::min(tracking_ref_.v_max_rad_s, hold_speed_effective());
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
    start_payload_check(now_ns, true);
  } else if (cfg_.payload_auto_verify && !payload_auto_done_ &&
             phase_ == Phase::Hold && !tracking_ && at_ready_ &&
             payload_profile_.has_value()) {
    bool at_rest = true;
    for (int i = 0; i < kAxisCount; ++i)
      if (std::fabs(sp[i].v_rad_s) > kAtRestVelRadS) at_rest = false;
    if (at_rest) {
      payload_auto_done_ = true;
      start_payload_check(now_ns, false);
    }
  }

  // 3. Build the supervisor input.
  SupervisorInput in;
  for (int i = 0; i < kAxisCount; ++i) {
    in.axes[i].q_raw_rad = sp[i].q_rad;
    in.axes[i].v_rad_s = sp[i].v_rad_s;
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

  // 5. Phase reference (per-axis q_ref, limit_spd). Default: hold in place.
  double q_ref[kAxisCount], lim[kAxisCount];
  for (int i = 0; i < kAxisCount; ++i) {
    q_ref[i] = sp[i].q_rad;
    lim[i] = 0.0;
  }
  switch (phase_) {
    case Phase::Homing: {
      const AxisId a = homing_->active_axis();
      DesiredState ds = homing_->step(to_feedback(sp[ix(a)]));
      q_ref[ix(a)] = ds.target_rad;
      lim[ix(a)] = ds.hold ? 0.0 : ds.speed_rad_s;
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
      if (tracking_) {
        // Phase 6: delegate to the tracking controller (tracking > search >
        // hold, §16). The reference was computed in step 2b. Constrain it with
        // the safety envelope (§15: reference manager -> SafetyEnvelope): clamp
        // the position to the soft limits and cap the speed at the largest
        // value from which a full stop still fits before the boundary.
        for (int i = 0; i < kAxisCount; ++i) {
          const double r = (i == ix(AxisId::Yaw)) ? tracking_ref_.q_yaw_rad
                                                  : tracking_ref_.q_pitch_rad;
          q_ref[i] = env_.constrain_reference(r, limits_[i]);
          lim[i] = std::min(tracking_ref_.v_max_rad_s,
                            env_.max_speed_at(q_ref[i], limits_[i]));
          at_ready_ = false;  // not at the ready pose while tracking/seeking
        }
        break;
      }
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
      for (int i = 0; i < kAxisCount; ++i) {
        q_ref[i] = sp[i].q_rad;
        lim[i] = 0.0;
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
        const auto out = check.step(now_ns, sp[payload_axis_ix_]);
        q_ref[payload_axis_ix_] = out.q_ref_rad;
        lim[payload_axis_ix_] = out.limit_spd_rad_s;
        if (!check.active()) {
          if (payload_axis_ix_ == 0) {
            payload_axis_ix_ = 1;  // begin the yaw check next cycle
          } else if (!check.failed()) {
            finish_payload_check(now_ns);
          } else {
            abort_payload_check(now_ns, check.fail_reason());
          }
        }
      } else if (check.failed()) {
        abort_payload_check(now_ns, check.fail_reason());
      }
      break;
    }
    case Phase::Parking: {
      ParkOutput po = park_->step(to_feedback(sp[ix(AxisId::Pitch)]),
                                  to_feedback(sp[ix(AxisId::Yaw)]));
      q_ref[ix(AxisId::Pitch)] = po.pitch.target_rad;
      lim[ix(AxisId::Pitch)] = po.pitch.hold ? 0.0 : po.pitch.speed_rad_s;
      q_ref[ix(AxisId::Yaw)] = po.yaw.target_rad;
      lim[ix(AxisId::Yaw)] = po.yaw.hold ? 0.0 : po.yaw.speed_rad_s;
      if (po.disable_pitch) backend_->deenergize(AxisId::Pitch);
      if (po.disable_yaw) backend_->deenergize(AxisId::Yaw);
      if (po.failed) fault("park failed: " + po.message);
      if (po.complete) phase_ = Phase::Parked;
      break;
    }
    case Phase::Fault: {
      // Controlled stop: keep braking while moving, then hold in place.
      for (int i = 0; i < kAxisCount; ++i) {
        if (std::fabs(sp[i].v_rad_s) > kAtRestVelRadS) {
          q_ref[i] = env_.emergency_stop_target(sp[i].q_rad, sp[i].v_rad_s,
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
        qr = env_.emergency_stop_target(sp[i].q_rad, sp[i].v_rad_s, limits_[i]);
        ls = cfg_.emergency_speed_rad_s;
        break;
      case SafetyAction::Disable:
        backend_->deenergize(a);
        any_disable = true;
        do_command = false;
        break;
    }
    if (do_command) backend_->command(a, qr, ls);
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
    // Phase 9: payload profile status (§42.1, §31.3).
    snap.payload_profile_name = payload_profile_ ? payload_profile_->name : "";
    snap.payload_profile_status =
        payload::payload_status_name(payload_status_);
    snap.payload_derated = payload_derated_;
    snap.payload_check_active = (phase_ == Phase::PayloadCheck);
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

void ControlLoop::set_payload_profile(payload::PayloadProfile pr) {
  if (phase_ == Phase::PayloadCheck) return;  // never swap mid-check
  payload_profile_ = std::move(pr);
  // A commissioned profile is trusted (§28.5) until a verification says
  // otherwise; a fresh profile clears any earlier mismatch derate (this is
  // the repeatable-commissioning path, §28.5/§31.3).
  payload_status_ = payload::PayloadStatus::Ok;
  payload_detail_ = "profile loaded: " + payload_profile_->name;
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

void ControlLoop::start_payload_check(TimeNs now_ns, bool manual) {
  if (phase_ == Phase::Fault || phase_ == Phase::PayloadCheck) return;
  if (!homed_) return;
  // The check takes over the axes: drop any tracking reference first.
  if (tracking_) disable_tracking();
  payload_check_cfg_ = payload::PayloadCheckConfig{};
  payload_check_cfg_.step_amplitude_rad = cfg_.payload_check_step_deg * kDeg2Rad;
  payload_check_cfg_.speed_rad_s = cfg_.payload_check_speed_deg_s * kDeg2Rad;
  const payload::PayloadProfile* prof =
      payload_profile_ ? &*payload_profile_ : nullptr;
  for (int i = 0; i < kAxisCount; ++i)
    payload_checks_[i] = payload::PayloadCheck(
        payload_check_cfg_, prof, static_cast<AxisId>(i));
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
    // Hold: drop any tracking reference so the loop settles at the ready pose.
    if (tracking_) disable_tracking();
    return;
  }
  if (name == "start_tracking") {
    if (tracking_) return;  // already enabled (validation should have caught it)
    TrackingController::Config tcfg;
    if (!enable_tracking(tcfg, err)) {
      spdlog::warn("start_tracking rejected: {}", err);
    }
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
  if (name == "select_target" || name == "enable_search" ||
      name == "disable_search" || name == "start_homing" ||
      name == "start_installation_calibration") {
    // These are validated here but acted on by the tracking/vision subsystem or
    // an external tool (see webd). Acknowledge so the UI can proceed.
    return;
  }
  spdlog::warn("unhandled web command: {}", name);
}

}  // namespace ota
