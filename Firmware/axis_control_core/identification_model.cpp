#include "identification_model.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace {
bool valid(const OtaIdentificationModel& m) {
  const double numbers[]={m.a,m.viscous,m.coulomb_negative,m.coulomb_positive,
    m.static_negative,m.static_positive,m.stribeck_negative,m.stribeck_positive,
    m.stribeck_power,m.load_offset,m.load_slope,m.q_origin,m.actuator_gain,
    m.actuator_bias,m.actuator_tau,m.transport_delay,m.gyro_bias,m.gyro_tau,
    m.gyro_delay,m.current_gain,m.current_bias,m.current_tau,m.current_delay,
    m.q_min,m.q_max,m.max_step};
  for(double value:numbers) if(!std::isfinite(value)) return false;
  return (m.actuator==0 || m.actuator==1) && (m.friction==0 || m.friction==1) &&
    (m.load==0 || m.load==1) && m.a>0 && m.viscous>=0 &&
    m.coulomb_negative>=0 && m.coulomb_positive>=0 &&
    m.static_negative>=m.coulomb_negative && m.static_positive>=m.coulomb_positive &&
    (m.friction==0 || (m.stribeck_negative>0 && m.stribeck_positive>0 && m.stribeck_power>=1)) &&
    m.actuator_gain>0 && m.actuator_tau>=0 && (m.actuator==0 || m.actuator_tau>0) &&
    m.transport_delay>=0 && m.gyro_tau>=0 && m.gyro_delay>=0 &&
    m.current_gain>0 && m.current_tau>=0 && m.current_delay>=0 &&
    m.q_max>m.q_min && m.max_step>0 && m.max_step<=.01;
}
double filtered(double previous,double target,double dt,double tau) {
  return tau==0?target:target+(previous-target)*std::exp(-dt/tau);
}
double filtered_current(double previous,double initial,double target,double dt,
    double actuator_tau,double sensor_tau) {
  if(sensor_tau==0) return filtered(initial,target,dt,actuator_tau);
  if(actuator_tau==0) return filtered(previous,target,dt,sensor_tau);
  // Exact response of the current sensor to a first-order regulated current.
  // The expm1 form keeps almost-equal time constants well conditioned.
  const double electrical=std::exp(-dt/actuator_tau),sensor=std::exp(-dt/sensor_tau);
  if(electrical==0 && sensor==0) return target;
  const double separation=dt*(1/sensor_tau-1/actuator_tau);
  double convolution;
  if(actuator_tau==sensor_tau || separation==0)
    convolution=(dt/sensor_tau)*sensor;
  else if(std::abs(separation)<.5)
    convolution=(dt/sensor_tau)*sensor*std::expm1(separation)/separation;
  else convolution=actuator_tau/(actuator_tau-sensor_tau)*(electrical-sensor);
  return target+(previous-target)*sensor+(initial-target)*convolution;
}
double moving_friction(const OtaIdentificationModel& m,double v,int direction) {
  const double fc=direction>0?m.coulomb_positive:m.coulomb_negative;
  const double fs=direction>0?m.static_positive:m.static_negative;
  const double vs=direction>0?m.stribeck_positive:m.stribeck_negative;
  const double magnitude=fc+(m.friction==1?(fs-fc)*std::exp(-std::pow(std::abs(v)/vs,m.stribeck_power)):0);
  return direction*magnitude;
}
struct Sample {double time,gyro,current,effective;};
double delayed(const std::vector<Sample>& history,double when,bool gyro) {
  if(when<=history.front().time) return gyro?history.front().gyro:history.front().current;
  auto right=std::lower_bound(history.begin(),history.end(),when,
    [](const Sample& sample,double at) {return sample.time<at;});
  if(right==history.end()) return gyro?history.back().gyro:history.back().current;
  if(right==history.begin()) return gyro?right->gyro:right->current;
  const auto& left=*(right-1);
  const double w=(when-left.time)/(right->time-left.time);
  return std::lerp(gyro?left.gyro:left.current,gyro?right->gyro:right->current,w);
}
double delayed_current(const std::vector<Sample>& history,double when,
    const OtaIdentificationModel& model,int nu,const double* ut,const double* u) {
  if(when<=history.front().time) return history.front().current;
  if(when>=history.back().time) return history.back().current;
  const auto right=std::upper_bound(history.begin(),history.end(),when,
    [](double at,const Sample& sample) {return at<sample.time;});
  const auto& left=*(right-1);
  // Accepted command changes already split the plant history. Within this
  // interval the current cascade has an exact response; querying it must not
  // change the mechanical integration mesh or its retained state.
  // A command at the query endpoint has not acted over the preceding interval.
  // Select the right-continuous command at the retained interval's left state.
  const auto next=std::upper_bound(ut,ut+nu,left.time-model.transport_delay+1e-13);
  const int command=std::max(0,static_cast<int>(next-ut)-1);
  const double target=model.actuator_gain*u[command]+model.actuator_bias;
  return filtered_current(left.current,left.effective,target,when-left.time,
    model.actuator==1?model.actuator_tau:0,model.current_tau);
}
}

extern "C" int ota_identification_rollout(const OtaIdentificationModel* model,int n,
    const double* t,int nu,const double* ut,const double* u,const double* initial,double* trace) {
  if(!model || !t || !ut || !u || !initial || !trace || n<2 || nu<1 || !valid(*model)) return 1;
  const auto& m=*model;
  for(int k=0;k<n;++k) if(!std::isfinite(t[k]) || (k && t[k]<=t[k-1])) return 1;
  for(int k=0;k<nu;++k) if(!std::isfinite(ut[k]) || !std::isfinite(u[k]) || (k && ut[k]<=ut[k-1])) return 1;
  for(int k=0;k<5;++k) if(!std::isfinite(initial[k])) return 1;
  if(ut[0]>t[0]-m.transport_delay) return 3;
  if(m.actuator==0 && m.current_tau==0 && ut[0]>t[0]-m.transport_delay-m.current_delay) return 3;
  double q=initial[0],v=initial[1],effective=initial[2],gyro=initial[3],current=initial[4];
  auto in_domain=[&](double pos) {return std::isfinite(pos) && pos>=m.q_min && pos<=m.q_max;};
  if(!in_domain(q)) return 2;
  int command=0;
  while(command+1<nu && ut[command+1]+m.transport_delay<=t[0]) ++command;
  if(m.actuator==0) effective=m.actuator_gain*u[command]+m.actuator_bias;
  if(m.gyro_tau==0) gyro=v;
  if(m.current_tau==0) current=effective;
  std::vector<Sample> history{{t[0],gyro,current,effective}};
  bool stick=std::abs(v)<1e-12;
  auto save=[&](int k) {
    double reported_current;
    if(m.actuator==0 && m.current_tau==0) {
      // Unfiltered current is a held accepted command, including at delayed
      // discontinuities. Interpolating adjacent integration samples invents ramps.
      const double when=t[k]-m.current_delay-m.transport_delay;
      auto next=std::upper_bound(ut,ut+nu,when+1e-13);
      const int index=std::max(0,static_cast<int>(next-ut)-1);
      reported_current=m.actuator_gain*u[index]+m.actuator_bias;
    } else reported_current=delayed_current(history,t[k]-m.current_delay,m,nu,ut,u);
    const double row[]={q,v,effective,delayed(history,t[k]-m.gyro_delay,true)+m.gyro_bias,
      m.current_gain*reported_current+m.current_bias,stick?1.:0.};
    std::copy(std::begin(row),std::end(row),trace+6*k);
  };
  save(0);
  for(int k=1;k<n;++k) {
    double at=t[k-1];
    while(at<t[k]-1e-13) {
      while(command+1<nu && ut[command+1]+m.transport_delay<=at+1e-13) ++command;
      const double target=m.actuator_gain*u[command]+m.actuator_bias;
      if(m.actuator==0) effective=target;
      double end=std::min(t[k],at+m.max_step);
      if(command+1<nu && ut[command+1]+m.transport_delay>at+1e-13)
        end=std::min(end,ut[command+1]+m.transport_delay);
      double dt=end-at;
      const double load=m.load_offset+(m.load?m.load_slope*(q-m.q_origin):0);
      const double net=effective-load;
      stick=std::abs(v)<1e-12 && net>=-m.static_negative && net<=m.static_positive;
      // At the exact boundary, a current response still increasing outward
      // breaks away immediately. Holding for a whole subsequent integration
      // step makes startup depend on the chosen numerical resolution.
      if(stick && m.actuator==1 &&
         ((net>=m.static_positive-1e-12 && target-load>m.static_positive) ||
          (net<=-m.static_negative+1e-12 && target-load<-m.static_negative))) stick=false;
      // Resolve a first-order actuator's exact threshold crossing while sticking.
      if(stick && m.actuator==1 && effective!=target) {
        const double threshold=target-load>m.static_positive?load+m.static_positive:
          (target-load<-m.static_negative?load-m.static_negative:target);
        const double ratio=(threshold-target)/(effective-target);
        if(ratio>0 && ratio<1) {
          const double crossing=-m.actuator_tau*std::log(ratio);
          if(crossing>1e-12 && crossing<dt) {dt=crossing;end=at+dt;}
        }
      }
      const double old_v=v,old_effective=effective;
      auto input=[&](double elapsed) {
        return m.actuator==0?target:filtered(old_effective,target,elapsed,m.actuator_tau);
      };
      if(stick) v=0;
      else {
        const int direction=std::abs(v)>1e-12?(v>0?1:-1):(net>0?1:-1);
        auto acceleration=[&](double pos,double vel,double elapsed) {
          const double spatial=m.load_offset+(m.load?m.load_slope*(pos-m.q_origin):0);
          return (input(elapsed)-spatial-m.viscous*vel-moving_friction(m,vel,direction))/m.a;
        };
        const double k1=acceleration(q,v,0),k2=acceleration(q+dt*v/2,v+dt*k1/2,dt/2);
        const double k3=acceleration(q+dt*(v+dt*k1/2)/2,v+dt*k2/2,dt/2);
        const double k4=acceleration(q+dt*(v+dt*k3/2),v+dt*k3,dt);
        const double next_v=v+dt*(k1+2*k2+2*k3+k4)/6;
        if(v*next_v<0) {
          // End exactly at the zero crossing and reconsider the holding interval.
          // Linear crossing interpolation is bounded by max_step; state is retained.
          const double fraction=std::clamp(std::abs(v)/(std::abs(v)+std::abs(next_v)),0.,1.);
          dt*=fraction;end=at+dt;
          q+=old_v*dt/2;v=0;
        } else {
          q+=dt*(v+2*(v+dt*k1/2)+2*(v+dt*k2/2)+(v+dt*k3))/6;
          v=next_v;
        }
      }
      effective=input(dt);
      gyro=filtered(gyro,(old_v+v)/2,dt,m.gyro_tau);
      if(m.gyro_tau==0) gyro=v;
      current=filtered_current(current,old_effective,target,dt,m.actuator==1?m.actuator_tau:0,m.current_tau);
      if(m.current_tau==0) current=effective;
      if(!in_domain(q) || !std::isfinite(v) || !std::isfinite(effective)) return 2;
      at=end;history.push_back({at,gyro,current,effective});
    }
    // Algebraic effective current is right-continuous at accepted event boundaries.
    while(command+1<nu && ut[command+1]+m.transport_delay<=t[k]+1e-13) ++command;
    if(m.actuator==0) {
      effective=m.actuator_gain*u[command]+m.actuator_bias;
      if(m.current_tau==0) {current=effective;history.back().current=current;}
    }
    const double load=m.load_offset+(m.load?m.load_slope*(q-m.q_origin):0);
    stick=std::abs(v)<1e-12 && effective-load>=-m.static_negative && effective-load<=m.static_positive;
    save(k);
  }
  return 0;
}
