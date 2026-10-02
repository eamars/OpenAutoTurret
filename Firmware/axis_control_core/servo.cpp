#include "servo.hpp"
#include <algorithm>
#include <cmath>
#include <new>

namespace ota::axis {
bool valid(const ServoParameters& p) {
  const double positive[] = {p.encoder_variance,p.gyro_variance,p.process_variance,p.max_encoder_age_s,
    p.max_gyro_age_s,p.stribeck_speed,p.friction_band,p.current_cap,p.slew,p.dt_min,p.dt_max,
    p.following_error_limit,p.error_clamp,p.rms_limit,p.rms_tau_s};
  for (double value:positive) if (!std::isfinite(value) || value<=0) return false;
  const double nonnegative[] = {p.inertia,p.coulomb_positive,p.coulomb_negative,p.stribeck_positive,
    p.stribeck_negative,p.viscous,p.creep_drop,p.friction_correction_rate,p.friction_correction_deadband,p.dither_amplitude,p.kq,p.kv,p.ki,p.integral_cap,p.hold_band,p.hold_speed,p.hold_relax_tau_s};
  for (double value:nonnegative) if (!std::isfinite(value) || value<0) return false;
  for (int k=0;k<ServoParameters::kFrictionBins;++k)
    if (!std::isfinite(p.friction_map_positive[k]) || !std::isfinite(p.friction_map_negative[k])) return false;
  for (double value:{p.stall_error,p.stall_speed,p.stall_current,p.stall_time_s,p.rock_current,p.rock_s,p.stall_reference_speed})
    if (!std::isfinite(value) || value<0) return false;
  if (p.creep_drop>0 && !(std::isfinite(p.creep_speed) && p.creep_speed>0)) return false;
  if (!std::isfinite(p.crosstalk_delay_s) || p.crosstalk_delay_s<0 || p.crosstalk_delay_s>0.02) return false;
  for (int k=0;k<ServoParameters::kCrosstalkBins;++k) if (!std::isfinite(p.crosstalk_map[k])) return false;
  if (!std::isfinite(p.friction_learning_rate) || p.friction_learning_rate<0 ||
      !std::isfinite(p.friction_learning_speed) || p.friction_learning_speed<0 ||
      !std::isfinite(p.friction_map_limit) || p.friction_map_limit<0) return false;
  return std::isfinite(p.load) && std::isfinite(p.dither_period_s) && (p.dither_amplitude==0 || p.dither_period_s>0) &&
         p.dt_max>=p.dt_min && p.rms_limit<=p.current_cap && (p.use_gyro==0 || p.use_gyro==1);
}
bool Servo::configure(const ServoParameters& p) {
  if (!valid(p)) return false;
  p_=p; configured_=true; ready_=false; return true;
}
bool Servo::reset(double t,double q,double v,double current) {
  if (!configured_ || !std::isfinite(t) || !std::isfinite(q) || !std::isfinite(v) ||
      !std::isfinite(current) || std::abs(current)>p_.current_cap) return false;
  t_=encoder_time_=gyro_time_=step_time_=t; q_=q; v_=v;
  p00_=p_.encoder_variance; p01_=0; p11_=p_.gyro_variance;
  // Bumpless: whatever was applied becomes the integral's starting point.
  integral_=std::clamp(current-friction(0.)-p_.load,-p_.integral_cap,p_.integral_cap);
  applied_history_.clear(); applied_history_.push_back({t,current});
  last_applied_=last_output_=current; mean_square_=current*current; last_saturated_=0; ready_=true; return true;
}
double Servo::friction(double v) const {
  if (v==0.) return 0.;
  const double speed=std::abs(v);
  const double scale=std::min(1.,speed/p_.friction_band);
  const double creep=p_.creep_drop>0?p_.creep_drop*std::exp(-speed/p_.creep_speed):0.;
  const double magnitude=std::max(0.,(v>0?p_.coulomb_positive+p_.stribeck_positive*std::exp(-speed/p_.stribeck_speed):
                                          p_.coulomb_negative+p_.stribeck_negative*std::exp(-speed/p_.stribeck_speed))-creep);
  return (v>0?1.:-1.)*scale*magnitude+p_.viscous*v;
}
namespace {
// Linear interpolation over periodic angle bins.
void bins(double q,int& k0,int& k1,double& w) {
  constexpr int n=ServoParameters::kFrictionBins;
  const double x=std::fmod(std::fmod(q,2*M_PI)+2*M_PI,2*M_PI)/(2*M_PI)*n;
  k0=static_cast<int>(std::floor(x))%n; k1=(k0+1)%n; w=x-std::floor(x);
}
}
double Servo::friction_at(double v,double q) const {
  if (v==0.) return 0.;
  int k0,k1; double w; bins(q,k0,k1,w);
  const double* map=v>0?p_.friction_map_positive:p_.friction_map_negative;
  const double extra=(1-w)*map[k0]+w*map[k1];
  const double scale=std::min(1.,std::abs(v)/p_.friction_band);
  return friction(v)+(v>0?1.:-1.)*scale*extra;
}
double Servo::crosstalk_gain(double q) const {
  constexpr int n=ServoParameters::kCrosstalkBins;
  const double x=std::fmod(std::fmod(q,2*M_PI)+2*M_PI,2*M_PI)/(2*M_PI)*n;
  const int k0=static_cast<int>(std::floor(x))%n,k1=(k0+1)%n; const double w=x-std::floor(x);
  return (1-w)*p_.crosstalk_map[k0]+w*p_.crosstalk_map[k1];
}
void Servo::predict(double to) {
  const double dt=to-t_;
  if (!(dt>0)) return;
  const double w=p_.process_variance;
  q_+=dt*v_;
  p00_+=2*dt*p01_+dt*dt*p11_+w*dt*dt*dt*dt/4;
  p01_+=dt*p11_+w*dt*dt*dt/2; p11_+=w*dt*dt; t_=to;
}
bool Servo::update(double value,double hq,double hv,double variance) {
  const double c0=p00_*hq+p01_*hv,c1=p01_*hq+p11_*hv,s=hq*c0+hv*c1+variance;
  if (!(s>0) || !std::isfinite(s)) return false;
  const double innovation=value-hq*q_-hv*v_;
  q_+=c0/s*innovation; v_+=c1/s*innovation;
  p00_=std::max(p00_-c0*c0/s,0.); p01_-=c0*c1/s; p11_=std::max(p11_-c1*c1/s,0.);
  return std::isfinite(q_) && std::isfinite(v_);
}
bool Servo::observe_encoder(double ts,double q) {
  if (!ready_ || !std::isfinite(ts) || !std::isfinite(q)) return false;
  if (ts<=encoder_time_) { ++stale_samples_; return true; }  // reordered/duplicate: ignore, not fatal
  // Remove the current-induced reading error using the current applied at ts-delay.
  double current=applied_history_.front().second;
  for (const auto& [time,value]:applied_history_) if (time<=ts-p_.crosstalk_delay_s) current=value; else break;
  q-=crosstalk_gain(q)*current;
  // A sample older than the state is applied through its age (H=[1,-age]).
  if (ts>=t_) { predict(ts); if (!update(q,1,0,p_.encoder_variance)) return ready_=false; }
  else {
    const double age=t_-ts,w=p_.process_variance;
    if (!update(q,1,-age,p_.encoder_variance+w*age*age*age*age/4)) return ready_=false;
  }
  encoder_time_=ts; return true;
}
bool Servo::observe_gyro(double ts,double rate) {
  if (!ready_ || !std::isfinite(ts) || !std::isfinite(rate)) return false;
  if (ts<=gyro_time_) { ++stale_samples_; return true; }
  gyro_time_=ts;
  if (!p_.use_gyro) return true;
  if (ts>=t_) { predict(ts); if (!update(rate,0,1,p_.gyro_variance)) return ready_=false; }
  else {
    const double age=t_-ts;
    if (!update(rate,0,1,p_.gyro_variance+p_.process_variance*age*age)) return ready_=false;
  }
  return true;
}
ServoOutput Servo::step(double now,double q_ref,double v_ref,double a_ref) {
  ServoOutput out{}; out.status=static_cast<int>(ServoStatus::NotReady);
  if (!ready_) return out;
  // Event-driven callers may step sooner than dt_min after a frame burst; that
  // is harmless and is integrated as dt_min. Only a gap beyond dt_max is invalid.
  const double elapsed=now-step_time_;
  out.status=static_cast<int>(ServoStatus::DataInvalid);
  if (!std::isfinite(q_ref) || !std::isfinite(v_ref) || !std::isfinite(a_ref) ||
      !(elapsed>0) || elapsed>p_.dt_max || now-encoder_time_>p_.max_encoder_age_s ||
      (p_.use_gyro && now-gyro_time_>p_.max_gyro_age_s)) { ready_=false; return out; }
  step_time_=now;
  const double dt=std::max(elapsed,p_.dt_min);
  // Extrapolate the posterior to now without committing it.
  const double lead=std::max(0.,now-t_);
  const double q=q_+v_*lead, v=v_;
  const double raw_error=q_ref-q;
  out.position=q; out.velocity=v; out.error=raw_error;
  if (std::abs(raw_error)>p_.following_error_limit) {
    ready_=false; out.status=static_cast<int>(ServoStatus::FollowingError); return out;
  }
  const double e=std::clamp(raw_error,-p_.error_clamp,p_.error_clamp), ev=v_ref-v;
  // The friction FF direction follows the motion the servo is asking for: the
  // reference plus a correction toward it. A stationary reference with a stuck
  // position error therefore still receives breakaway help toward the target.
  const double beyond=std::max(0.,std::abs(e)-p_.friction_correction_deadband);
  const double fr=friction_at(v_ref+p_.friction_correction_rate*(e>0?beyond:-beyond),q);
  // Learning: steady tracking in one direction moves persistent integral into
  // the friction map at this angle; the sum (map+integral) is unchanged.
  if (p_.friction_learning_rate>0 && std::abs(v_ref)>=p_.friction_learning_speed && !last_saturated_) {
    int k0,k1; double w; bins(q,k0,k1,w);
    double* map=v_ref>0?p_.friction_map_positive:p_.friction_map_negative;
    const double sign=v_ref>0?1.:-1.;
    const double move=integral_*std::min(1.,p_.friction_learning_rate*dt);
    for (const auto& [k,share]:{std::pair{k0,1-w},std::pair{k1,w}}) {
      const double before=map[k];
      map[k]=std::clamp(map[k]+sign*move*share,-p_.friction_map_limit,p_.friction_map_limit);
      integral_-=sign*(map[k]-before);
    }
  }
  const double ff=p_.inertia*a_ref+fr+p_.load;
  // Conditional integration: no accumulation that pushes further into saturation,
  // and none inside the hold band while the reference is stationary.
  const bool hold=p_.hold_band>0 && std::abs(v_ref)<p_.hold_speed && std::abs(e)<p_.hold_band;
  if (hold) {
    // Yaw carries no gravity load: at rest the integral relaxes instead of
    // holding the axis loaded against static friction (heat, and a stuck
    // preloaded contact that needs far more current to restart).
    if (p_.hold_relax_tau_s>0) integral_*=std::exp(-dt/p_.hold_relax_tau_s);
  } else if (!(last_saturated_ && last_saturated_*e>0))
    integral_=std::clamp(integral_+p_.ki*e*dt,-p_.integral_cap,p_.integral_cap);
  out.proportional=p_.kq*e; out.derivative=p_.kv*ev; out.integral=integral_;
  out.feedforward=ff; out.friction=fr; out.velocity_error=ev;
  double dither=0.;
  if (p_.dither_amplitude>0)
    dither=std::fmod(now,p_.dither_period_s)<p_.dither_period_s/2?p_.dither_amplitude:-p_.dither_amplitude;
  out.requested=ff+out.proportional+out.derivative+integral_+dither;
  // Stall recovery (see header): loaded static contacts released whenever force reversed.
  if (p_.stall_time_s>0) {
    // "Not moving" is judged by displacement since the stall began (the observer's
    // instantaneous velocity is too noisy at rest), allowing two encoder counts.
    const bool settling=p_.stall_reference_speed<=0 || std::abs(v_ref)<p_.stall_reference_speed;
    const bool pushing=settling && std::abs(e)>p_.stall_error && std::abs(out.requested)>p_.stall_current && out.requested*e>0;
    if (!pushing || now<rock_until_) stall_since_=-1;
    else if (stall_since_<0) { stall_since_=now; stall_position_=q; }
    else if (std::abs(q-stall_position_)>p_.stall_speed*(now-stall_since_)+2*2*M_PI/8192) stall_since_=-1;
    if (stall_since_>=0 && now-stall_since_>=p_.stall_time_s) {
      rock_until_=now+p_.rock_s; rock_direction_=e>0?-1.:1.; stall_since_=-1; ++stall_events_;
    }
    if (now<rock_until_) { out.requested=rock_direction_*p_.rock_current; integral_=0.; out.rocking=1; }
  }
  out.stall_events=stall_events_;
  // Thermal budget: peaks up to current_cap, but once the exponentially
  // weighted RMS reaches rms_limit the authority itself falls to rms_limit.
  mean_square_+=(last_applied_*last_applied_-mean_square_)*(1-std::exp(-dt/p_.rms_tau_s));
  const double cap=std::sqrt(mean_square_)>=p_.rms_limit?p_.rms_limit:p_.current_cap;
  out.rms=std::sqrt(mean_square_); out.cap=cap;
  const double slewed=std::clamp(out.requested,last_output_-p_.slew*dt,last_output_+p_.slew*dt);
  out.limited=std::clamp(slewed,-cap,cap);
  last_output_=out.limited;
  out.saturated=out.limited<out.requested-1e-12?1:out.limited>out.requested+1e-12?-1:0;
  last_saturated_=out.saturated;
  out.status=static_cast<int>(ServoStatus::Ok);
  return out;
}
bool Servo::set_gains(double kq,double kv,double ki) {
  if (!std::isfinite(kq) || !std::isfinite(kv) || !std::isfinite(ki) || kq<0 || kv<0 || ki<0) return false;
  p_.kq=kq; p_.kv=kv; p_.ki=ki; return true;
}
void Servo::acknowledge(bool success,double applied) {
  if (!success || !std::isfinite(applied)) { ready_=false; return; }
  last_applied_=applied;
  applied_history_.push_back({step_time_,applied});
  while (applied_history_.size()>2 && applied_history_[1].first<step_time_-0.05) applied_history_.pop_front();
}
}

extern "C" {
void* ota_servo_create(const ota::axis::ServoParameters* p) {
  if (!p) return nullptr;
  auto* s=new(std::nothrow) ota::axis::Servo;
  if (s && !s->configure(*p)) { delete s; return nullptr; }
  return s;
}
void ota_servo_destroy(void* s) { delete static_cast<ota::axis::Servo*>(s); }
int ota_servo_reset(void* s,double t,double q,double v,double current) {
  return s && static_cast<ota::axis::Servo*>(s)->reset(t,q,v,current);
}
int ota_servo_observe_encoder(void* s,double t,double q) {
  return s && static_cast<ota::axis::Servo*>(s)->observe_encoder(t,q);
}
int ota_servo_observe_gyro(void* s,double t,double rate) {
  return s && static_cast<ota::axis::Servo*>(s)->observe_gyro(t,rate);
}
int ota_servo_step(void* s,double now,double q,double v,double a,ota::axis::ServoOutput* out) {
  if (!s || !out) return 0;
  *out=static_cast<ota::axis::Servo*>(s)->step(now,q,v,a); return 1;
}
int ota_servo_acknowledge(void* s,int success,double applied) {
  if (!s) return 0;
  static_cast<ota::axis::Servo*>(s)->acknowledge(success!=0,applied); return 1;
}
}
