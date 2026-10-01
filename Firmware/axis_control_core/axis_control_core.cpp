#include "axis_control_core.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <new>
#include <vector>

namespace ota::axis {
bool valid(const Model& m) {
  if ((m.n != 5 && m.n != 8) || (m.periodic != 0 && m.periodic != 1) ||
      (m.periodic && m.n != 8)) return false;
  for (int k = 0; k < 7 + 6*m.n; ++k) if (!std::isfinite(m.theta[k])) return false;
  for (int k = 0; k < 3; ++k) {
    if (!(m.theta[k] > 0) || m.theta[3+k] < 0 || !std::isfinite(m.z[k])) return false;
    if (k && m.z[k] <= m.z[k-1]) return false;
  }
  for (int k = 0; k < m.n; ++k)
    if (!std::isfinite(m.q[k]) || (k && m.q[k] <= m.q[k-1])) return false;
  if (m.periodic && std::abs((m.q[1]-m.q[0])*8-2*std::numbers::pi)>1e-8) return false;
  return m.theta[6+6*m.n] >= 0;
}
bool coefficients(const Model& m, double q, double z, int d, double& a, double& b, double& h) {
  if (!std::isfinite(q) || !std::isfinite(z) || (d != -1 && d != 1) ||
      z < m.z[0] || z > m.z[2]) return false;
  if (m.periodic) {
    const double period = 2*std::numbers::pi;
    q = m.q[0] + std::fmod(std::fmod(q-m.q[0], period)+period, period);
  } else if (q < m.q[0]-1e-8 || q > m.q[m.n-1]+1e-8) return false;
  const int iz = z <= m.z[1] ? 0 : 1;
  const double wz = (z-m.z[iz])/(m.z[iz+1]-m.z[iz]);
  int iq = 0;
  while (iq < m.n-2 && q > m.q[iq+1]) ++iq;
  int next = iq+1;
  double right = m.q[next];
  if (m.periodic && q > m.q[m.n-1]) {iq=m.n-1;next=0;right=m.q[0]+2*std::numbers::pi;}
  const double wq = std::clamp((q-m.q[iq])/(right-m.q[iq]), 0., 1.);
  a = std::lerp(m.theta[iz],m.theta[iz+1],wz);
  b = std::lerp(m.theta[3+iz],m.theta[4+iz],wz);
  const int start = 6 + (d > 0 ? 3*m.n : 0);
  auto row = [&](int k) {return std::lerp(m.theta[start+k*m.n+iq],m.theta[start+k*m.n+next],wq);};
  h = std::lerp(row(iz),row(iz+1),wz);
  return true;
}

bool valid(const Parameters& p) {
  if (!valid(p.model)) return false;
  const double positive[] = {p.observer.encoder_variance,p.observer.gyro_variance,
    p.observer.process_variance,p.observer.max_encoder_age_s,p.observer.max_gyro_age_s,
    p.observer.initial_position_variance,p.observer.initial_velocity_variance,
    p.kp,p.kpos,p.kaw,p.current_cap,p.slew,p.integral_cap,p.velocity_cap,p.dt_min,p.dt_max,
    p.intent_threshold,p.rest_speed,p.sustained_s,p.start_timeout_s};
  for (double value:positive) if (!std::isfinite(value) || value<=0) return false;
  if (!std::isfinite(p.ki) || p.ki<0 || p.dt_max<p.dt_min || p.sustained_s>=p.start_timeout_s ||
      (p.observer.encoder_only_verified!=0 && p.observer.encoder_only_verified!=1)) return false;
  for(int k=0;k<6*p.model.n;++k)
    if (!std::isfinite(p.start_total[k]) || (p.start_censored[k]!=0 && p.start_censored[k]!=1)) return false;
  if(!std::isfinite(p.acceleration_cap) || p.acceleration_cap<0 ||
     !std::isfinite(p.acceleration_noise_sigma) || p.acceleration_noise_sigma<0 ||
     !std::isfinite(p.acceleration_sample_period_s) || p.acceleration_sample_period_s<0 ||
     (p.acceleration_current_window_enabled!=0 && p.acceleration_current_window_enabled!=1) ||
     (p.acceleration_cap>0 && !(p.acceleration_sample_period_s>0))) return false;
  return true;
}
bool Controller::configure(const Parameters& p) {
  if(ready_ || !valid(p)) return false;
  p_=p;configured_=true;return true;
}
bool Controller::reset(double t,double q,double v,double current,std::uint64_t generation,double current_time) {
  const bool actual_time=std::isfinite(current_time);
  if(evaluating_feedforward_ || !configured_ || !std::isfinite(t) || !std::isfinite(q) || !std::isfinite(v) ||
     !std::isfinite(current) || std::abs(current)>p_.current_cap || !generation ||
     (actual_time && current_time>t)) return false;
  time_=encoder_time_=gyro_time_=t;q_=q;v_=v;generation_=generation;
  p00_=p_.observer.initial_position_variance;p01_=0;p11_=p_.observer.initial_velocity_variance;
  integral_=last_applied_=current;error_previous_=0;encoder_seq_=gyro_seq_=0;
  motion_=Motion::Rest;direction_=1;sustained_since_=-1;pending_=false;
  quiet_initial_offset_=0;
  encoder_position_=motion_check_position_=q;gyro_travel_=motion_check_gyro_=0;
  last_gyro_rate_=v;motion_check_time_=t;
  running_integral_.fill(0.);
  transient_output_=false;
  shaped_velocity_=v; measured_acceleration_=0;acceleration_time_=t;acceleration_interval_=0;
  acceleration_fresh_=acceleration_valid_=current_history_actual_time_=false;
  current_history_.clear();
  current_history_.push_back({actual_time?current_time:t,current,actual_time});
  initialize_integral_=true;hard_abort_=false;ready_=true;return true;
}
bool Controller::observe(const Observation& o) {
  acceleration_fresh_=false;
  if(o.generation!=generation_ || !std::isfinite(o.now) ||
     (!current_history_.empty() && o.now<current_history_.back().time)) return false;
  dt_=o.now-time_;
  if(dt_<p_.dt_min || dt_>p_.dt_max) return false; // do not integrate across a scheduler gap
  q_+=dt_*v_;
  const double w=p_.observer.process_variance;
  p00_+=2*dt_*p01_+dt_*dt_*p11_+w*std::pow(dt_,4)/4;
  p01_+=dt_*p11_+w*std::pow(dt_,3)/2;p11_+=w*dt_*dt_;
  auto update=[&](double value,double hq,double hv,double variance) {
    const double c0=p00_*hq+p01_*hv,c1=p01_*hq+p11_*hv;
    const double s=hq*c0+hv*c1+variance;
    if (!(s>0) || !std::isfinite(s)) return false;
    const double innovation=value-hq*q_-hv*v_;
    q_+=c0/s*innovation;v_+=c1/s*innovation;
    p00_=std::max(p00_-c0*c0/s,0.);p01_-=c0*c1/s;p11_=std::max(p11_-c1*c1/s,0.);
    return std::isfinite(q_) && std::isfinite(v_);
  };
  if(o.encoder_valid && o.encoder_seq!=encoder_seq_) {
    if(o.encoder_seq<encoder_seq_ || !std::isfinite(o.encoder_time) ||
       !std::isfinite(o.position) || o.encoder_time>o.now || o.encoder_time<encoder_time_ ||
       (encoder_seq_ && o.encoder_time==encoder_time_)) return false;
    const double age=o.now-o.encoder_time;
    if(age>p_.observer.max_encoder_age_s || !update(o.position,1,-age,
       p_.observer.encoder_variance+w*std::pow(age,4)/4)) return false;
    encoder_seq_=o.encoder_seq;encoder_time_=o.encoder_time;encoder_position_=o.position;
  }
  if(o.gyro_valid && o.gyro_seq!=gyro_seq_) {
    if(o.gyro_seq<gyro_seq_ || !std::isfinite(o.gyro_time) ||
       !std::isfinite(o.gyro_rate) || o.gyro_time>o.now || o.gyro_time<gyro_time_ ||
       (gyro_seq_ && o.gyro_time==gyro_time_)) return false;
    const double age=o.now-o.gyro_time;
    if(age>p_.observer.max_gyro_age_s || !update(o.gyro_rate,0,1,
       p_.observer.gyro_variance+w*age*age)) return false;
    if(gyro_seq_) {
      acceleration_interval_=o.gyro_time-gyro_time_;
      measured_acceleration_=(o.gyro_rate-last_gyro_rate_)/acceleration_interval_;
      acceleration_time_=o.gyro_time;acceleration_fresh_=true;
      const double delay=p_.model.theta[6+6*p_.model.n];
      acceleration_valid_=interval_current(gyro_time_-delay,o.gyro_time-delay,
        delayed_applied_current_,current_history_actual_time_);
    }
    gyro_travel_+=(o.gyro_time-gyro_time_)*(last_gyro_rate_+o.gyro_rate)/2;
    last_gyro_rate_=o.gyro_rate;gyro_seq_=o.gyro_seq;gyro_time_=o.gyro_time;
  }
  return o.now-encoder_time_<=p_.observer.max_encoder_age_s &&
    (o.now-gyro_time_<=p_.observer.max_gyro_age_s || p_.observer.encoder_only_verified);
}
bool Controller::interval_current(double begin,double end,double& mean,bool& actual_time) const {
  if(current_history_.empty() || current_history_.front().time>begin || !(end>begin)) return false;
  auto event=current_history_.begin();
  while(std::next(event)!=current_history_.end() && std::next(event)->time<=begin) ++event;
  double at=begin,total=0.;actual_time=true;
  while(at<end) {
    const auto next=std::next(event);
    const double until=next==current_history_.end()?end:std::min(end,next->time);
    total+=(until-at)*event->current;actual_time&=event->actual_time;at=until;
    if(at<end) event=next;
  }
  mean=total/(end-begin);return std::isfinite(mean);
}
Output Controller::step(const Observation& o,const Reference& r) {
  return step_impl(o,r,nullptr);
}
void Controller::inhibit() {
  hard_abort_=true;ready_=false;pending_=false;
}
bool Controller::defer_destruction() {
  if(!evaluating_feedforward_) return false;
  destruction_requested_=true;inhibit();return true;
}
Output Controller::step_with_feedforward(const Observation& o,const Reference& r,double command_ff) {
  if(!std::isfinite(command_ff)) inhibit();
  return step_impl(o,r,&command_ff);
}
Output Controller::step_with_posterior_feedforward(const Observation& o,const Reference& r,
    FeedforwardCallback callback,void* context,int planned_start_intent) {
  return step_with_posterior_phase(o,r,callback,context,
      planned_start_intent?ReferencePhase::Departure:ReferencePhase::Legacy,planned_start_intent);
}
Output Controller::step_with_posterior_phase(const Observation& o,const Reference& r,
    FeedforwardCallback callback,void* context,ReferencePhase phase,int planned_start_intent) {
  const bool departure=phase==ReferencePhase::Departure;
  if(!callback || (phase!=ReferencePhase::Legacy && !departure && phase!=ReferencePhase::Braking) ||
     (departure?(planned_start_intent!=-1 && planned_start_intent!=1):planned_start_intent!=0) ||
     (departure && (planned_start_intent*r.velocity < 0 || planned_start_intent*r.acceleration < 0)) ||
     (phase==ReferencePhase::Braking && !(r.velocity*r.acceleration<0))) inhibit();
  return step_impl(o,r,nullptr,callback,context,planned_start_intent,phase);
}
Output Controller::step_impl(const Observation& o,const Reference& r,const double* command_ff,
    FeedforwardCallback callback,void* callback_context,int planned_start_intent,ReferencePhase phase) {
  Output result{};result.status=static_cast<int>(Status::DataInvalid);
  if(evaluating_feedforward_) inhibit();
  if(hard_abort_) {result.status=static_cast<int>(Status::HardAbort);return result;}
  if(!ready_ || pending_ || !std::isfinite(r.position) || !std::isfinite(r.velocity) ||
     !std::isfinite(r.acceleration) || !std::isfinite(r.posture) || !observe(o)) {
    ready_=false;return result;
  }
  time_=o.now;
  if(planned_start_intent && motion_==Motion::Start && planned_start_intent!=direction_) {
    // An opposite declaration cannot reuse an active START's floor or restart
    // its timeout. Braking/reversal must use the existing transition path.
    inhibit();result.status=static_cast<int>(Status::HardAbort);return result;
  }
  // A validated shaped departure can declare intent at its original anchor,
  // before its continuous velocity reaches the numerical intent threshold.
  // The existing Reverse/START/rest/timeout owner still handles that intent.
  const int intent=planned_start_intent?planned_start_intent:
                   r.velocity>p_.intent_threshold?1:r.velocity<-p_.intent_threshold?-1:0;
  double reference_velocity=r.velocity+p_.kpos*(r.position-q_);
  const auto previous_motion=motion_;
  const double encoder_motion_floor=3*std::sqrt(2*p_.observer.encoder_variance);
  // Position intent uses measured position noise. Applying the velocity noise
  // threshold here left a real 0.308-degree target error with no correction.
  const double position_error=r.position-q_;
  // A stationary one-count fluctuation also moves the observer estimate.
  // Include its current uncertainty before issuing another breakaway request.
  const double position_intent_floor=encoder_motion_floor+3*std::sqrt(p00_);
  const int wanted=intent?intent:(position_error>position_intent_floor?1:
                                      position_error<-position_intent_floor?-1:0);
  const double gyro_motion_floor=p_.rest_speed*p_.sustained_s;
  auto body_moved=[&]() {
    return direction_*(encoder_position_-motion_check_position_)>encoder_motion_floor &&
           direction_*(gyro_travel_-motion_check_gyro_)>gyro_motion_floor;
  };
  auto motion_window=[&]() {
    motion_check_time_=o.now;motion_check_position_=encoder_position_;motion_check_gyro_=gyro_travel_;
  };
  if(wanted && wanted!=direction_ && std::abs(v_)>p_.rest_speed) motion_=Motion::Reverse;
  if(motion_==Motion::Reverse) {
    reference_velocity=0;
    if(std::abs(v_)<=p_.rest_speed) motion_=Motion::Rest;
  }
  const bool planned_braking=phase==ReferencePhase::Braking;
  // An authoritative shaped deceleration uses the existing STOP path. Its
  // remaining signed velocity must not turn a slow body window into START.
  // HOLD uses Legacy and retains the existing position-correction intent.
  if(planned_braking && motion_!=Motion::Reverse) motion_=Motion::Stop;
  if(motion_!=Motion::Reverse) {
    if(wanted && motion_==Motion::Move && o.now-motion_check_time_>=p_.sustained_s) {
      // Quantized velocity spikes do not prove continued body travel.
      if(body_moved()) {
        // Continued measured motion distinguishes ordinary applied drive from
        // the first breakaway/slew impulse. Resume its existing back-calculation
        // so a real running-load error is not excluded indefinitely.
        transient_output_=false;
      } else if(direction_*last_gyro_rate_<=p_.rest_speed) motion_=Motion::Stop;
      motion_window();
    }
    if(!planned_braking && wanted && (motion_==Motion::Rest || motion_==Motion::Stop)) {
      // Running load corrections belong to their measured direction. Braking
      // and quiet hold cancellation must not oppose the next startup.
      motion_=Motion::Start;direction_=wanted;start_time_=o.now;sustained_since_=-1;
      motion_window();
      integral_=running_integral_[direction_>0?1:0];
      quiet_initial_offset_=0;
    } else if(!wanted && !planned_braking) {
      motion_=std::abs(v_)<=p_.rest_speed?Motion::Rest:Motion::Stop;
    }
  }
  double a,b,h;
  if(!coefficients(p_.model,q_,r.posture,direction_,a,b,h)) {
    ready_=false;result.status=static_cast<int>(Status::OperatingPointChanged);return result;
  }
  const bool braking=motion_==Motion::Reverse || motion_==Motion::Stop;
  // While moving, braking retains the moving direction's measured running load.
  // Dropping that compensation itself caused uncontrolled deceleration.
  const bool neutral=motion_==Motion::Rest;
  if(braking) {
    const int moving_direction=std::abs(last_gyro_rate_)>p_.rest_speed?(last_gyro_rate_>0?1:-1):direction_;
    if(!coefficients(p_.model,q_,r.posture,moving_direction,a,b,h)) {
      ready_=false;result.status=static_cast<int>(Status::OperatingPointChanged);return result;
    }
    if(motion_!=previous_motion) {
      integral_=running_integral_[moving_direction>0?1:0];transient_output_=true;
    }
  }
  if(neutral) {
    double other_a,other_b,other_h;
    if(!coefficients(p_.model,q_,r.posture,-direction_,other_a,other_b,other_h)) {
      ready_=false;result.status=static_cast<int>(Status::OperatingPointChanged);return result;
    }
    h=(h+other_h)/2;
  }
  reference_velocity=std::clamp(reference_velocity,-p_.velocity_cap,p_.velocity_cap);
  const double requested_reference_velocity=reference_velocity;
  double shaped_acceleration=r.acceleration;
  int acceleration_reason=0;
  if(p_.acceleration_cap>0) {
    const double previous=shaped_velocity_;
    reference_velocity=std::clamp(reference_velocity,previous-p_.acceleration_cap*dt_,
                                 previous+p_.acceleration_cap*dt_);
    shaped_acceleration=(reference_velocity-previous)/dt_;
    if(reference_velocity!=requested_reference_velocity) acceleration_reason|=1;
  }
  shaped_velocity_=reference_velocity;
  error_=reference_velocity-v_;
  double callback_ff=0.;
  if(callback) {
    const auto& accepted=current_history_.back();
    const PosteriorState state{o.now,dt_,q_,v_,encoder_time_,gyro_time_,accepted.current,accepted.time,
      encoder_seq_,gyro_seq_,generation_,o.now-gyro_time_>p_.observer.max_gyro_age_s,
      static_cast<int>(motion_),accepted.actual_time};
    evaluating_feedforward_=true;
    bool accepted_ff=false;
    try { accepted_ff=callback(callback_context,&state,&r,&callback_ff); }
    catch(...) { accepted_ff=false; }
    evaluating_feedforward_=false;
    if(!accepted_ff || !std::isfinite(callback_ff) || hard_abort_ || !ready_ || pending_) {
      inhibit();result.status=static_cast<int>(Status::HardAbort);
      result.position=q_;result.velocity=v_;result.motion=static_cast<int>(motion_);return result;
    }
    command_ff=&callback_ff;
  }
  const double ff=command_ff?*command_ff:(p_.acceleration_cap>0?a*shaped_acceleration+b*reference_velocity+h:
    a*(braking?0:r.acceleration)+b*(braking?0:r.velocity)+h);
  if(initialize_integral_) {
    // A zero-output reset may cancel load before intent, including noisy STOP.
    // It must not cancel the first requested motion or a direction change.
    if(last_applied_!=0. || !wanted) {
      integral_=std::clamp(last_applied_-p_.kp*error_-ff,-p_.integral_cap,p_.integral_cap);
      if(last_applied_==0.) quiet_initial_offset_=integral_;
      else running_integral_[direction_>0?1:0]=integral_;
    }
    initialize_integral_=false;
  }
  double base=p_.kp*error_+integral_+ff;
  double increment=0;
  result.status=static_cast<int>(Status::Ok);
  if(motion_==Motion::Start) {
    const bool moving=body_moved();
    if(moving && o.now-motion_check_time_>=p_.sustained_s) {
      motion_=Motion::Move;motion_window();
    }
    if(motion_==Motion::Start) {
      Model thresholds=p_.model;
      for(int k=0;k<6*thresholds.n;++k) thresholds.theta[6+k]=p_.start_total[k];
      double unused_a,unused_b,total;
      coefficients(thresholds,q_,r.posture,direction_,unused_a,unused_b,total);
      Model censor=p_.model;
      for(int k=0;k<6*censor.n;++k) censor.theta[6+k]=p_.start_censored[k];
      double censored;
      coefficients(censor,q_,r.posture,direction_,unused_a,unused_b,censored);
      if(censored>0 || o.now-start_time_>p_.start_timeout_s) {
        // A censored threshold or exhausted start duration ends the attempt.
        // No command token or pending output is created, and a reconnect or
        // subsequent reference cannot re-arm this controller.
        hard_abort_=true;ready_=false;pending_=false;
        result.status=static_cast<int>(Status::HardAbort);
        result.position=q_;result.velocity=v_;result.motion=static_cast<int>(motion_);
        return result;
      }
      // Assist responds to actual gyro speed while sustained encoder/gyro
      // evidence still controls START->MOVE. Waiting for that whole window
      // kept maximum drive queued after the body reached its requested speed.
      const bool speed_reached=direction_*last_gyro_rate_>p_.rest_speed &&
        direction_*last_gyro_rate_>=direction_*reference_velocity;
      if(!moving && !speed_reached) {
        increment=direction_*std::max(0.,direction_*(total-base));
      }
    }
  }
  if(increment!=0.) transient_output_=true;
  base_requested_=base;requested_=base+increment;
  double guarded=requested_,effective_slew=p_.slew;
  double current_min=-p_.current_cap,current_max=p_.current_cap,horizon=0.;
  if(p_.acceleration_cap>0 && p_.acceleration_current_window_enabled) {
    const double delay=p_.model.theta[6+6*p_.model.n];
    horizon=delay+std::max(p_.acceleration_sample_period_s,acceleration_interval_)+
            std::max(0.,o.now-gyro_time_);
    // One measured sigma is retained as uncertainty, not a physical guarantee.
    const double usable=std::max(0.,p_.acceleration_cap-p_.acceleration_noise_sigma);
    if(acceleration_valid_) {
      // The raw derivative spans this actual interval; use its interval-mean
      // delayed accepted current, not a newly requested or held gyro sample.
      double center=delayed_applied_current_-a*measured_acceleration_;
      if(motion_==Motion::Start && wanted && std::abs(last_gyro_rate_)<=p_.rest_speed) {
        // Static friction censors the running-load observation: zero alpha
        // under a prior-direction current does not make that current the new
        // direction's equilibrium. This is the existing model/load hypothesis,
        // not a measured breakaway threshold or a certified acceleration bound.
        center=h+b*last_gyro_rate_+running_integral_[direction_>0?1:0];
        acceleration_reason|=128;
      }
      current_min=std::max(-p_.current_cap,center-a*usable);
      current_max=std::min(p_.current_cap,center+a*usable);
      if(current_min>current_max) current_min=current_max=std::clamp(center,-p_.current_cap,p_.current_cap);
      guarded=std::clamp(guarded,current_min,current_max);
      if(guarded!=requested_) acceleration_reason|=2;
      if(std::abs(measured_acceleration_)>p_.acceleration_cap) acceleration_reason|=8;
    } else acceleration_reason|=16;
    if(motion_==Motion::Start || motion_==Motion::Rest) {
      // Identified current-per-acceleration divided by the real feedback
      // horizon bounds startup current change while static friction is unknown.
      effective_slew=std::min(effective_slew,a*usable/std::max(horizon,dt_));
      if(std::abs(guarded-last_applied_)>effective_slew*dt_) acceleration_reason|=4;
    }
  }
  limited_=std::clamp(std::clamp(guarded,last_applied_-effective_slew*dt_,last_applied_+effective_slew*dt_),
                      -p_.current_cap,p_.current_cap);
  if(p_.acceleration_cap>0 && p_.acceleration_current_window_enabled && acceleration_valid_ &&
     (limited_<current_min || limited_>current_max)) {
    const double electrical_min=std::max(-p_.current_cap,last_applied_-p_.slew*dt_);
    const double electrical_max=std::min(p_.current_cap,last_applied_+p_.slew*dt_);
    const double feasible_min=std::max(electrical_min,current_min);
    const double feasible_max=std::min(electrical_max,current_max);
    if(feasible_min<=feasible_max) {
      // A model-derived startup slew must not block a correction that the
      // existing electrical envelope can transmit.
      limited_=std::clamp(limited_,feasible_min,feasible_max);
      acceleration_reason|=32;
    } else {
      // Past applied input can make these hypotheses infeasible together.
      // Retain the real current/slew authority and explicitly record conflict.
      limited_=std::clamp(guarded,electrical_min,electrical_max);
      acceleration_reason|=64;
    }
  }
  if(p_.acceleration_cap>0 && !p_.acceleration_current_window_enabled) {
    horizon=p_.model.theta[6+6*p_.model.n]+
      std::max(p_.acceleration_sample_period_s,acceleration_interval_)+std::max(0.,o.now-gyro_time_);
    acceleration_reason|=256; // owner guidance: observe excursions, permit tracking drive
    if(acceleration_valid_ && std::abs(measured_acceleration_)>p_.acceleration_cap) acceleration_reason|=8;
  }
  if(sequence_==std::numeric_limits<std::uint64_t>::max()) {
    hard_abort_=true;ready_=false;
    result.status=static_cast<int>(Status::HardAbort);return result;
  }
  pending_=true;++sequence_;
  result.requested=requested_;result.limited=limited_;result.position=q_;result.velocity=v_;
  result.integral=integral_;result.feedforward=ff;result.start_increment=increment;
  result.sequence=sequence_;result.motion=static_cast<int>(motion_);
  result.encoder_only=o.now-gyro_time_>p_.observer.max_gyro_age_s;
  result.requested_reference_velocity=requested_reference_velocity;
  result.shaped_reference_velocity=reference_velocity;
  result.requested_reference_acceleration=r.acceleration;result.shaped_reference_acceleration=shaped_acceleration;
  result.measured_acceleration=measured_acceleration_;result.acceleration_sample_time=acceleration_time_;
  result.acceleration_interval_s=acceleration_interval_;result.acceleration_noise_sigma=p_.acceleration_noise_sigma;
  result.acceleration_feedback_horizon_s=horizon;result.delayed_applied_current=delayed_applied_current_;
  result.acceleration_current_min=current_min;result.acceleration_current_max=current_max;
  result.acceleration_limited_request=guarded;result.acceleration_fresh=acceleration_fresh_;
  result.acceleration_valid=acceleration_valid_;result.acceleration_limit_reason=acceleration_reason;
  result.current_history_actual_time=current_history_actual_time_;
  return result;
}
bool Controller::acknowledge(std::uint64_t seq,bool success,double applied) {
  return acknowledge_current(seq,success,applied,time_,false);
}
bool Controller::acknowledge_at(std::uint64_t seq,bool success,double applied,double accepted_time) {
  return acknowledge_current(seq,success,applied,accepted_time,true);
}
bool Controller::acknowledge_current(std::uint64_t seq,bool success,double applied,double accepted_time,bool actual_time) {
  if(evaluating_feedforward_ || !ready_ || !pending_ || seq!=sequence_ || !std::isfinite(applied) ||
     std::abs(applied)>p_.current_cap || !std::isfinite(accepted_time) || accepted_time<time_ ||
     (!current_history_.empty() && accepted_time<current_history_.back().time)) return false;
  pending_=false;
  if(!success) {ready_=false;return false;} // failed TX is never treated as applied current
  if(motion_==Motion::Start || transient_output_)
    // Friction stalls still provide real tracking error. Learn that error,
    // without absorbing the separate breakaway request or its slew transient.
    integral_=std::clamp(integral_+p_.ki*dt_*(error_+error_previous_)/2+
      p_.kaw*dt_*(std::clamp(base_requested_,-p_.current_cap,p_.current_cap)-base_requested_),
      -p_.integral_cap,p_.integral_cap);
  else
    integral_=std::clamp(integral_+p_.ki*dt_*(error_+error_previous_)/2+
                         p_.kaw*dt_*(applied-requested_),-p_.integral_cap,p_.integral_cap);
  if(motion_==Motion::Move || motion_==Motion::Start) running_integral_[direction_>0?1:0]=integral_;
  if(motion_!=Motion::Start && std::abs(applied-requested_)<=p_.slew*dt_)
    transient_output_=false;
  current_history_.push_back({accepted_time,applied,actual_time});
  const double retain=gyro_time_-p_.model.theta[6+6*p_.model.n]-
    std::max(p_.acceleration_sample_period_s,acceleration_interval_)-p_.observer.max_gyro_age_s;
  while(current_history_.size()>1 && current_history_[1].time<retain) current_history_.pop_front();
  error_previous_=error_;last_applied_=applied;return true;
}
bool Controller::switch_parameters(const Parameters& p,const Reference& r) {
  if(evaluating_feedforward_ || !ready_ || pending_ || !valid(p) || motion_!=Motion::Rest ||
     std::abs(last_applied_)>p.current_cap) return false;
  double a,b,h;
  if(!coefficients(p.model,q_,r.posture,direction_,a,b,h)) return false;
  const double error=std::clamp(r.velocity+p.kpos*(r.position-q_),-p.velocity_cap,p.velocity_cap)-v_;
  const double value=last_applied_-p.kp*error-a*r.acceleration-b*r.velocity-h;
  if(!std::isfinite(value) || std::abs(value)>p.integral_cap) return false;
  p_=p;integral_=value;error_previous_=error;return true;
}
}

extern "C" int ota_core_abi() { return 4; }
extern "C" void* ota_controller_create(const ota::axis::Parameters* p) {
  if(!p) return nullptr;
  auto* c=new(std::nothrow) ota::axis::Controller;
  if(!c) return nullptr;
  if(!c->configure(*p)) {delete c;return nullptr;}return c;
}
extern "C" void ota_controller_destroy(void* c) {
  auto* controller=static_cast<ota::axis::Controller*>(c);
  if(controller && !controller->defer_destruction()) delete controller;
}
extern "C" int ota_controller_reset(void* c,double t,double q,double v,double current,std::uint64_t gen) {
  return c && static_cast<ota::axis::Controller*>(c)->reset(t,q,v,current,gen);
}
extern "C" int ota_controller_reset_at(void* c,double t,double q,double v,double current,std::uint64_t gen,double accepted_time) {
  return c && static_cast<ota::axis::Controller*>(c)->reset(t,q,v,current,gen,accepted_time);
}
extern "C" int ota_controller_step(void* c,const ota::axis::Observation* o,
    const ota::axis::Reference* r,ota::axis::Output* out) {
  if(!c || !o || !r || !out) return 0;
  auto* controller=static_cast<ota::axis::Controller*>(c);
  *out=controller->step(*o,*r);
  if(controller->destruction_ready()) delete controller;
  return 1;
}
extern "C" int ota_controller_step_ff(void* c,const ota::axis::Observation* o,
    const ota::axis::Reference* r,double command_ff,ota::axis::Output* out) {
  if(!c || !o || !r || !out) return 0;
  auto* controller=static_cast<ota::axis::Controller*>(c);
  *out=controller->step_with_feedforward(*o,*r,command_ff);
  if(controller->destruction_ready()) delete controller;
  return 1;
}
extern "C" int ota_controller_step_posterior_ff(void* c,const ota::axis::Observation* o,
    const ota::axis::Reference* r,ota::axis::FeedforwardCallback callback,void* context,
    ota::axis::Output* out) {
  if(!c || !o || !r || !out) return 0;
  auto* controller=static_cast<ota::axis::Controller*>(c);
  *out=controller->step_with_posterior_feedforward(*o,*r,callback,context);
  if(controller->destruction_ready()) delete controller;
  return 1;
}
extern "C" int ota_controller_step_posterior_ff_intent(void* c,const ota::axis::Observation* o,
    const ota::axis::Reference* r,int planned_start_intent,ota::axis::FeedforwardCallback callback,
    void* context,ota::axis::Output* out) {
  if(!c || !o || !r || !out) return 0;
  auto* controller=static_cast<ota::axis::Controller*>(c);
  *out=controller->step_with_posterior_feedforward(*o,*r,callback,context,planned_start_intent);
  if(controller->destruction_ready()) delete controller;
  return 1;
}
extern "C" int ota_controller_step_posterior_ff_phase(void* c,const ota::axis::Observation* o,
    const ota::axis::Reference* r,int phase,int planned_start_intent,ota::axis::FeedforwardCallback callback,
    void* context,ota::axis::Output* out) {
  if(!c || !o || !r || !out) return 0;
  auto* controller=static_cast<ota::axis::Controller*>(c);
  *out=controller->step_with_posterior_phase(*o,*r,callback,context,
      static_cast<ota::axis::ReferencePhase>(phase),planned_start_intent);
  if(controller->destruction_ready()) delete controller;
  return 1;
}
extern "C" int ota_controller_inhibit(void* c) {
  if(!c) return 0;
  static_cast<ota::axis::Controller*>(c)->inhibit();return 1;
}
extern "C" int ota_controller_parameters(void* c,ota::axis::Parameters* out) {
  if(!c || !out) return 0;
  *out=static_cast<ota::axis::Controller*>(c)->parameters();return 1;
}
extern "C" int ota_controller_ack(void* c,std::uint64_t seq,int success,double applied) {
  return c && static_cast<ota::axis::Controller*>(c)->acknowledge(seq,success,applied);
}
extern "C" int ota_controller_ack_at(void* c,std::uint64_t seq,int success,double applied,double accepted_time) {
  return c && static_cast<ota::axis::Controller*>(c)->acknowledge_at(seq,success,applied,accepted_time);
}
extern "C" int ota_controller_switch(void* c,const ota::axis::Parameters* p,const ota::axis::Reference* r) {
  return c && p && r && static_cast<ota::axis::Controller*>(c)->switch_parameters(*p,*r);
}
extern "C" int ota_closed_rollout(const ota::axis::Parameters* params,
    const ota::axis::Parameters* plant,const OtaSimulation* settings,int n,
    const ota::axis::Reference* refs,double q,double v,double* trace) {
  using namespace ota::axis;
  if(!params || !plant || !settings || !refs || !trace || n<2 || !valid(*plant) ||
     !(settings->dt>0) || settings->encoder_period<1 || settings->gyro_period<1 ||
     settings->measurement_delay<0 || settings->gyro_filter_tau<0 ||
     settings->encoder_quantum<0 || settings->encoder_noise<0 || settings->gyro_noise<0) return 1;
  Controller controller;
  double a,b,h;
  if(!coefficients(plant->model,q,refs[0].posture,1,a,b,h) || !controller.configure(*params) ||
      !controller.reset(0,q,v,h,1)) return 1;
  const double dt=settings->dt,delay=plant->model.theta[6+6*plant->model.n];
  std::vector<double> currents(n+1,h),positions(n+1,q),velocities(n+1,v);
  std::uint64_t seed=settings->seed?settings->seed:1;
  auto noise=[&](double sigma) {seed^=seed<<13;seed^=seed>>7;seed^=seed<<17;
    return (static_cast<double>(seed>>11)/9007199254740992.-.5)*std::sqrt(12.)*sigma;};
  double gyro=v;
  int direction=1, command=0;
  Model start=plant->model;
  for(int j=0;j<6*start.n;++j) start.theta[6+j]=plant->start_total[j];
  for(int k=0;k<n;++k) {
    const double now=(k+1)*dt;
    auto domain_failure=[&]() {trace[0]=q;trace[1]=v;trace[2]=refs[k].posture;trace[3]=k;return 2;};
    // Integrate the last successfully applied output, respecting fractional TX delay.
    double at=k*dt;
    while(at<now-1e-13) {
      while(command+1<=k && (command+1)*dt+delay<=at+1e-13) ++command;
      double end=std::min(now,at+.001);
      if(command+1<=k && (command+1)*dt+delay>at+1e-13)
        end=std::min(end,(command+1)*dt+delay);
      const double step=end-at,u=currents[command];
      double upper,lower,aa,bb;
      if(!coefficients(start,q,refs[k].posture,1,aa,bb,upper) ||
         !coefficients(start,q,refs[k].posture,-1,aa,bb,lower)) return domain_failure();
      if(std::abs(v)<1e-7 && u>lower && u<upper) {v=0;at=end;continue;}
      direction=std::abs(v)>1e-7?(v>0?1:-1):(u>=upper?1:-1);
      auto acc=[&](double pos,double vel,double& out) {
        double ma,mb,mh;
        if(!coefficients(plant->model,pos,refs[k].posture,direction,ma,mb,mh)) return false;
        out=(u-mb*vel-mh)/ma;return std::isfinite(out);
      };
      double k1,k2,k3,k4;
      if(!acc(q,v,k1) || !acc(q+step*v/2,v+step*k1/2,k2) ||
         !acc(q+step*(v+step*k1/2)/2,v+step*k2/2,k3) ||
         !acc(q+step*(v+step*k2/2),v+step*k3,k4)) return domain_failure();
      q+=step*(v+2*(v+step*k1/2)+2*(v+step*k2/2)+(v+step*k3))/6;
      const double next=v+step*(k1+2*k2+2*k3+k4)/6;
      v=(v*next<0 && u>=lower && u<=upper)?0:next;
      at=end;
    }
    positions[k+1]=q;velocities[k+1]=v;
    const int index=std::clamp(static_cast<int>(std::floor((now-settings->measurement_delay)/dt+1e-9)),0,k+1);
    double measured_q=positions[index]+noise(settings->encoder_noise);
    if(settings->encoder_quantum>0) measured_q=std::round(measured_q/settings->encoder_quantum)*settings->encoder_quantum;
    gyro += dt/(settings->gyro_filter_tau+dt)*(velocities[index]-gyro);
    const bool enc=(k%settings->encoder_period==0),gyr=(k%settings->gyro_period==0);
    Observation o{now,index*dt,index*dt,measured_q,gyro+noise(settings->gyro_noise),
      static_cast<std::uint64_t>(index+1),static_cast<std::uint64_t>(index+1),1,enc,gyr};
    Output out=controller.step(o,refs[k]);
    if(out.status!=0 && out.status!=static_cast<int>(Status::EnvelopeLimited)) return 3;
    if(!controller.acknowledge(out.sequence,true,out.limited)) return 3;
    currents[k+1]=out.limited;
    const double row[]={q,v,out.position,out.velocity,out.requested,out.limited,
      out.integral,out.feedforward,out.start_increment,static_cast<double>(out.motion),
      static_cast<double>(out.status),currents[command]};
    std::copy(std::begin(row),std::end(row),trace+12*k);
  }
  return 0;
}
namespace {
int model_rollout(const ota::axis::Model* m, int n, const double* t,
    const double* z, const int* d, int nu, const double* ut, const double* u,
    double q, double v, double* out, bool actual_history) {
  if (!m || !t || !ut || !u || !z || !d || !out || n < 2 || nu < 1 || !ota::axis::valid(*m) ||
      !std::isfinite(q) || !std::isfinite(v)) return 1;
  const double delay = m->theta[6+6*m->n];
  if (!std::isfinite(t[0])) return 1;
  for (int j=0;j<nu;++j) {
    if (!std::isfinite(ut[j]) || !std::isfinite(u[j]) || (j && !(ut[j]>ut[j-1]))) return 1;
  }
  if (actual_history && ut[0]>t[0]-delay) return 3;
  int command = 0;
  out[0]=q;out[1]=v;
  for (int k=1;k<n;++k) {
    if (!(t[k]>t[k-1]) || !std::isfinite(t[k])) return 1;
    double now=t[k-1];
    while (now < t[k]-1e-13) {
      while (command+1<nu && ut[command+1]+delay<=now) ++command;
      double end=std::min(t[k],now+.0025); // numerical integration resolution, not controller tuning
      if (command+1<nu && ut[command+1]+delay>now) end=std::min(end,ut[command+1]+delay);
      const double dt=end-now;
      auto acceleration=[&](double pos,double vel,double& acc) {
        double a,b,h;
        if (!ota::axis::coefficients(*m,pos,z[k-1],d[k-1],a,b,h)) return false;
        acc=(u[command]-b*vel-h)/a;return std::isfinite(acc);
      };
      double k1,k2,k3,k4;
      if (!acceleration(q,v,k1) || !acceleration(q+dt*v/2,v+dt*k1/2,k2) ||
          !acceleration(q+dt*(v+dt*k1/2)/2,v+dt*k2/2,k3) ||
          !acceleration(q+dt*(v+dt*k2/2),v+dt*k3,k4)) return 2;
      q += dt*(v+2*(v+dt*k1/2)+2*(v+dt*k2/2)+(v+dt*k3))/6;
      v += dt*(k1+2*k2+2*k3+k4)/6;
      now=end;
    }
    out[2*k]=q;out[2*k+1]=v;
  }
  return 0;
}
}
extern "C" int ota_model_rollout(const ota::axis::Model* m, int n, const double* t,
    const double* u, const double* z, const int* d, double q, double v, double* out) {
  return model_rollout(m,n,t,z,d,n,t,u,q,v,out,false);
}
extern "C" int ota_model_rollout_with_history(const ota::axis::Model* m, int n,
    const double* t, const double* z, const int* d, int nu, const double* ut,
    const double* u, double q, double v, double* out) {
  return model_rollout(m,n,t,z,d,nu,ut,u,q,v,out,true);
}
