#include "level1.hpp"
#include "control/reference_limiter.hpp"
#include <algorithm>
#include <cmath>

namespace ota::track {
bool valid(const Level1Parameters& p) {
  for (const auto& a:p.axis)
    if (!(std::isfinite(a.lambda) && a.lambda>0 && std::isfinite(a.v_max) && a.v_max>0 && std::isfinite(a.a_max) && a.a_max>0 &&
          std::isfinite(a.j_max) && a.j_max>0 && std::isfinite(a.lead_limit) && a.lead_limit>0 &&
          std::isfinite(a.q_min) && std::isfinite(a.q_max) && a.q_min<=a.q_max &&
          std::isfinite(a.dead_band) && a.dead_band>=0 && std::isfinite(a.feedforward_gain) &&
          a.feedforward_gain>=0 && a.feedforward_gain<=1)) return false;
  return std::isfinite(p.period_s) && p.period_s>0 && p.period_s<0.1 && std::isfinite(p.valid_s) && p.valid_s>=p.period_s;
}

void integrate(double& q,double& v,double& a,double j,double tau,double a_max,double v_max) {
  for (int events=0;tau>1e-12 && events<8;++events) {
    // Already at the acceleration bound and pushing outward: hold the bound.
    if (j!=0 && std::abs(a)>=a_max && (a>0)==(j>0)) { a=std::copysign(a_max,a); j=0; }
    double t=tau;
    if (j!=0) {
      const double ta=((j>0?a_max:-a_max)-a)/j;
      if (ta>0 && ta<t) t=ta;
    }
    // First time within (0, t] the speed reaches the ceiling while moving outward.
    double ts=-1;
    for (double bound:{v_max,-v_max}) {
      const double c=v-bound, A=0.5*j, B=a;
      double roots[2]={-1,-1};
      if (std::abs(A)<1e-15) { if (B!=0) roots[0]=-c/B; }
      else {
        const double disc=B*B-4*A*c;
        if (disc>=0) { const double s=std::sqrt(disc); roots[0]=(-B-s)/(2*A); roots[1]=(-B+s)/(2*A); }
      }
      for (double r:roots)
        if (r>1e-12 && r<=t && (ts<0 || r<ts)) {
          const double slope=a+j*r;  // dv/dt at the crossing: must point outward
          if ((bound>0 && slope>0) || (bound<0 && slope<0)) ts=r;
        }
    }
    const bool ceiling=ts>0;
    if (ceiling) t=ts;
    q+=v*t+a*t*t/2+j*t*t*t/6; v+=a*t+j*t*t/2; a+=j*t;
    tau-=t;
    if (ceiling) { v=std::copysign(v_max,v); a=0; j=0; }
    else if (j!=0 && std::abs(a)>=a_max-1e-12) { a=std::copysign(a_max,a); j=0; }
  }
  if (tau>1e-12) { q+=v*tau+a*tau*tau/2+j*tau*tau*tau/6; v+=a*tau+j*tau*tau/2; a+=j*tau; }
}

bool ReferenceSample::at(int i,int64_t t,double& q_out,double& v_out,double& a_out) const {
  if (!valid || i<0 || i>1) return false;
  const double tau=std::max(0.,(t-t_ns)*1e-9);
  if (tau>valid_s) return false;
  q_out=q[i]; v_out=v[i]; a_out=a[i];
  integrate(q_out,v_out,a_out,j[i],tau,a_max[i],v_max[i]);
  return true;
}

bool Level1Generator::configure(const Level1Parameters& p) {
  if (!valid(p)) return false;
  p_=p; configured_=true; initialized_=false; last_={}; return true;
}

bool Level1Generator::set_limits(int axis,double v_max,double a_max,double j_max,double q_min,double q_max) {
  if (axis<0 || axis>1 || !(std::isfinite(v_max) && v_max>0 && std::isfinite(a_max) && a_max>0 &&
      std::isfinite(j_max) && j_max>0 && std::isfinite(q_min) && std::isfinite(q_max) && q_min<=q_max)) return false;
  auto& a=p_.axis[axis];
  a.v_max=v_max; a.a_max=a_max; a.j_max=j_max; a.q_min=q_min; a.q_max=q_max;
  return true;
}

bool Level1Generator::set_speed_bounds(int axis,double negative_speed,double positive_speed) {
  if (axis<0 || axis>1 || std::isnan(negative_speed) || std::isnan(positive_speed)) return false;
  auto& a=p_.axis[axis];
  a.v_neg_cap=std::max(0.,negative_speed); a.v_pos_cap=std::max(0.,positive_speed);
  return true;
}

void Level1Generator::reset(int64_t t_ns,const std::array<double,2>& q,const std::array<double,2>& v,
                            const std::array<double,2>& a) {
  last_={};
  last_.t_ns=t_ns; last_.q=q; last_.v=v; last_.a=a; last_.valid=true; last_.valid_s=p_.valid_s; lead_hold_={};
  for (int i=0;i<2;++i) { last_.a_max[i]=p_.axis[i].a_max; last_.v_max[i]=p_.axis[i].v_max; }
  initialized_=true;
}

ReferenceSample Level1Generator::step(int64_t t_ns,const JointGoal& goal,const std::array<double,2>& q_measured) {
  if (!configured_) return {};
  if (!initialized_) reset(t_ns,q_measured,{0.,0.},{0.,0.});
  ReferenceSample s=last_;
  const double dt=(t_ns-last_.t_ns)*1e-9;
  if (dt<0) return last_;
  for (int i=0;i<2;++i) integrate(s.q[i],s.v[i],s.a[i],last_.j[i],dt,p_.axis[i].a_max,p_.axis[i].v_max);
  s.t_ns=t_ns; s.valid=true; s.valid_s=p_.valid_s;
  for (int i=0;i<2;++i) {
    const auto& c=p_.axis[i];
    uint32_t flags=0;
    const double q=s.q[i], v=s.v[i], a=s.a[i];
    double qt=q, vt=0;
    if (!goal.valid) flags|=kGoalInvalid;
    else {
      qt=goal.q[i];
      if (goal.velocity_valid[i] && std::isfinite(goal.v[i])) vt=goal.v[i]; else flags|=kVelocityInvalid;
    }
    const bool bounded=c.q_min<c.q_max;
    if (bounded && (qt<c.q_min || qt>c.q_max)) { qt=std::clamp(qt,c.q_min,c.q_max); vt=0; flags|=kBoundary; }
    vt*=c.feedforward_gain;
    if (c.dead_band>0) {
      const double e=qt-q;
      if (std::abs(e)<=c.dead_band) { qt=q; vt=0; } else qt-=std::copysign(c.dead_band,e);
    }
    // The axis cannot keep up (saturation, an obstruction): beyond lead_limit the reference
    // stops advancing and holds where it is, until the axis has closed half of the limit;
    // then it continues from there toward the current target (nothing queued is replayed).
    const double lead=q-q_measured[i];
    if (std::abs(lead)>c.lead_limit) lead_hold_[i]=true;
    else if (std::abs(lead)<c.lead_limit/2) lead_hold_[i]=false;
    if (lead_hold_[i] && (qt-q)*lead>=0) { qt=q; vt=0; flags|=kLeadLimited; }
    double request=c.lambda*c.lambda*(qt-q)+2*c.lambda*(vt-v);
    s.a_request[i]=request;
    if (bounded && v!=0) {
      const double room=v>0?c.q_max-q:q-c.q_min;
      if (control::stopping_distance_rad(v,a,c.a_max,c.j_max)>=room-std::abs(v)*p_.period_s) {
        request=v>0?std::min(request,-c.a_max):std::max(request,c.a_max); flags|=kBoundary;
      }
    }
    // Speed bounds: v_max both ways, tightened by the host's directional caps. Above a cap the
    // request is pulled down along the same jerk-limited curve that approaches it from below (a
    // cap that falls as the axis nears its end must slow the reference, not merely stop it
    // accelerating).
    const double vp=std::min(c.v_max,c.v_pos_cap), vn=std::min(c.v_max,c.v_neg_cap);
    const auto toward=[&](double gap) { return std::copysign(std::sqrt(2*c.j_max*std::abs(gap)),gap); };
    const double up=toward(vp-v), down=-toward(vn+v);
    if (request>up || request<down) { request=std::clamp(request,down,up); flags|=kVelocityLimited; }
    if (std::abs(request)>c.a_max) { request=std::copysign(c.a_max,request); flags|=kAccelerationLimited; }
    double j=(request-a)/p_.period_s;
    if (std::abs(j)>c.j_max) { j=std::copysign(c.j_max,j); flags|=kJerkLimited; }
    s.j[i]=j; s.flags[i]=flags; s.a_max[i]=c.a_max; s.v_max[i]=c.v_max;
  }
  last_=s;
  return s;
}
}
