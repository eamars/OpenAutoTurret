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
  a = m.theta[iz]*(1-wz)+m.theta[iz+1]*wz;
  b = m.theta[3+iz]*(1-wz)+m.theta[4+iz]*wz;
  const int start = 6 + (d > 0 ? 3*m.n : 0);
  auto row = [&](int k) {return m.theta[start+k*m.n+iq]*(1-wq)+m.theta[start+k*m.n+next]*wq;};
  h = row(iz)*(1-wz)+row(iz+1)*wz;
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
  return true;
}
bool Controller::configure(const Parameters& p) {
  if(ready_ || !valid(p)) return false;
  p_=p;configured_=true;return true;
}
bool Controller::reset(double t,double q,double v,double current,std::uint64_t generation) {
  if(!configured_ || !std::isfinite(t) || !std::isfinite(q) || !std::isfinite(v) ||
     !std::isfinite(current) || std::abs(current)>p_.current_cap || !generation) return false;
  time_=encoder_time_=gyro_time_=t;q_=q;v_=v;generation_=generation;
  p00_=p_.observer.initial_position_variance;p01_=0;p11_=p_.observer.initial_velocity_variance;
  integral_=last_applied_=current;error_previous_=0;encoder_seq_=gyro_seq_=sequence_=0;
  motion_=Motion::Rest;direction_=1;sustained_since_=-1;pending_=false;
  initialize_integral_=true;ready_=true;return true;
}
bool Controller::observe(const Observation& o) {
  if(o.generation!=generation_ || !std::isfinite(o.now)) return false;
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
    encoder_seq_=o.encoder_seq;encoder_time_=o.encoder_time;
  }
  if(o.gyro_valid && o.gyro_seq!=gyro_seq_) {
    if(o.gyro_seq<gyro_seq_ || !std::isfinite(o.gyro_time) ||
       !std::isfinite(o.gyro_rate) || o.gyro_time>o.now || o.gyro_time<gyro_time_ ||
       (gyro_seq_ && o.gyro_time==gyro_time_)) return false;
    const double age=o.now-o.gyro_time;
    if(age>p_.observer.max_gyro_age_s || !update(o.gyro_rate,0,1,
       p_.observer.gyro_variance+w*age*age)) return false;
    gyro_seq_=o.gyro_seq;gyro_time_=o.gyro_time;
  }
  return o.now-encoder_time_<=p_.observer.max_encoder_age_s &&
    (o.now-gyro_time_<=p_.observer.max_gyro_age_s || p_.observer.encoder_only_verified);
}
Output Controller::step(const Observation& o,const Reference& r) {
  Output result{};result.status=static_cast<int>(Status::DataInvalid);
  if(!ready_ || pending_ || !std::isfinite(r.position) || !std::isfinite(r.velocity) ||
     !std::isfinite(r.acceleration) || !std::isfinite(r.posture) || !observe(o)) {
    ready_=false;return result;
  }
  time_=o.now;
  const int intent=r.velocity>p_.intent_threshold?1:r.velocity<-p_.intent_threshold?-1:0;
  double reference_velocity=r.velocity+p_.kpos*(r.position-q_);
  // Direction follows intended motion with hysteresis, not noisy measured sign.
  const int wanted=intent?intent:(reference_velocity>p_.intent_threshold?1:
                                      reference_velocity<-p_.intent_threshold?-1:0);
  bool handoff=false;
  if(wanted && wanted!=direction_ && std::abs(v_)>p_.rest_speed) motion_=Motion::Reverse;
  if(motion_==Motion::Reverse) {
    reference_velocity=0;
    if(std::abs(v_)<=p_.rest_speed) {motion_=Motion::Rest;handoff=true;}
  }
  if(motion_!=Motion::Reverse) {
    if(wanted && (motion_==Motion::Rest || motion_==Motion::Stop)) {
      // Preserve the learned residual integral, not the old direction's load current.
      // Final slew arbitration makes the change continuous at the actuator.
      motion_=Motion::Start;direction_=wanted;start_time_=o.now;sustained_since_=-1;
    } else if(!wanted) {
      motion_=std::abs(v_)<=p_.rest_speed?Motion::Rest:Motion::Stop;
    }
  }
  double a,b,h;
  if(!coefficients(p_.model,q_,r.posture,direction_,a,b,h)) {
    ready_=false;result.status=static_cast<int>(Status::OperatingPointChanged);return result;
  }
  reference_velocity=std::clamp(reference_velocity,-p_.velocity_cap,p_.velocity_cap);
  error_=reference_velocity-v_;
  const double ff=a*(motion_==Motion::Reverse?0:r.acceleration)+b*(motion_==Motion::Reverse?0:r.velocity)+h;
  if(handoff || initialize_integral_) {
    integral_=std::clamp(last_applied_-p_.kp*error_-ff,-p_.integral_cap,p_.integral_cap);
    initialize_integral_=false;
  }
  double base=p_.kp*error_+integral_+ff;
  double increment=0;
  result.status=static_cast<int>(Status::Ok);
  if(motion_==Motion::Start) {
    if(direction_*v_>p_.rest_speed) {
      if(sustained_since_<0) sustained_since_=o.now;
      if(o.now-sustained_since_>=p_.sustained_s) {
        motion_=Motion::Move;
        integral_=std::clamp(last_applied_-p_.kp*error_-ff,-p_.integral_cap,p_.integral_cap);
        base=p_.kp*error_+integral_+ff;
      }
    } else sustained_since_=-1;
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
        result.status=static_cast<int>(Status::EnvelopeLimited);
      } else increment=direction_*std::max(0.,direction_*(total-base));
    }
  }
  requested_=base+increment;
  limited_=std::clamp(std::clamp(requested_,last_applied_-p_.slew*dt_,last_applied_+p_.slew*dt_),
                      -p_.current_cap,p_.current_cap);
  pending_=true;++sequence_;
  result.requested=requested_;result.limited=limited_;result.position=q_;result.velocity=v_;
  result.integral=integral_;result.feedforward=ff;result.start_increment=increment;
  result.sequence=sequence_;result.motion=static_cast<int>(motion_);
  result.encoder_only=o.now-gyro_time_>p_.observer.max_gyro_age_s;
  return result;
}
bool Controller::acknowledge(std::uint64_t seq,bool success,double applied) {
  if(!ready_ || !pending_ || seq!=sequence_ || !std::isfinite(applied) ||
     std::abs(applied)>p_.current_cap) return false;
  pending_=false;
  if(!success) {ready_=false;return false;} // failed TX is never treated as applied current
  integral_=std::clamp(integral_+p_.ki*dt_*(error_+error_previous_)/2+
                       p_.kaw*dt_*(applied-requested_),-p_.integral_cap,p_.integral_cap);
  error_previous_=error_;last_applied_=applied;return true;
}
bool Controller::switch_parameters(const Parameters& p,const Reference& r) {
  if(!ready_ || pending_ || !valid(p) || motion_!=Motion::Rest ||
     std::abs(last_applied_)>p.current_cap) return false;
  double a,b,h;
  if(!coefficients(p.model,q_,r.posture,direction_,a,b,h)) return false;
  const double error=std::clamp(r.velocity+p.kpos*(r.position-q_),-p.velocity_cap,p.velocity_cap)-v_;
  const double value=last_applied_-p.kp*error-a*r.acceleration-b*r.velocity-h;
  if(!std::isfinite(value) || std::abs(value)>p.integral_cap) return false;
  p_=p;integral_=value;error_previous_=error;return true;
}
}

extern "C" int ota_core_abi() { return 2; }
extern "C" void* ota_controller_create(const ota::axis::Parameters* p) {
  if(!p) return nullptr;
  auto* c=new(std::nothrow) ota::axis::Controller;
  if(!c) return nullptr;
  if(!c->configure(*p)) {delete c;return nullptr;}return c;
}
extern "C" void ota_controller_destroy(void* c) {delete static_cast<ota::axis::Controller*>(c);}
extern "C" int ota_controller_reset(void* c,double t,double q,double v,double current,std::uint64_t gen) {
  return c && static_cast<ota::axis::Controller*>(c)->reset(t,q,v,current,gen);
}
extern "C" int ota_controller_step(void* c,const ota::axis::Observation* o,
    const ota::axis::Reference* r,ota::axis::Output* out) {
  if(!c || !o || !r || !out) return 0;
  *out=static_cast<ota::axis::Controller*>(c)->step(*o,*r);return 1;
}
extern "C" int ota_controller_ack(void* c,std::uint64_t seq,int success,double applied) {
  return c && static_cast<ota::axis::Controller*>(c)->acknowledge(seq,success,applied);
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
extern "C" int ota_model_rollout(const ota::axis::Model* m, int n, const double* t,
    const double* u, const double* z, const int* d, double q, double v, double* out) {
  if (!m || !t || !u || !z || !d || !out || n < 2 || !ota::axis::valid(*m) ||
      !std::isfinite(q) || !std::isfinite(v)) return 1;
  const double delay = m->theta[6+6*m->n];
  int command = 0;
  out[0]=q;out[1]=v;
  for (int k=1;k<n;++k) {
    if (!(t[k]>t[k-1]) || !std::isfinite(t[k]) || !std::isfinite(u[k-1])) return 1;
    double now=t[k-1];
    while (now < t[k]-1e-13) {
      while (command+1<n && t[command+1]+delay<=now+1e-13) ++command;
      double end=std::min(t[k],now+.0025); // numerical integration resolution, not controller tuning
      if (command+1<n && t[command+1]+delay>now+1e-13) end=std::min(end,t[command+1]+delay);
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
