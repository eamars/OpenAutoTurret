#include "estimator.hpp"
#include <algorithm>
#include <cmath>

namespace ota::track {
namespace {
double wrap(double a) { return std::remainder(a,2*M_PI); }

struct Prediction { double theta, omega, pp, pv, vv; };

Prediction predict(const AxisState& x,double dt,double q) {
  const double d2=dt*dt, d3=d2*dt;
  return {x.theta+x.omega*dt, x.omega,
          x.pp+2*dt*x.pv+d2*x.vv+q*d3/3, x.pv+dt*x.vv+q*d2/2, x.vv+q*dt};
}

// Joseph-form scalar update of a [theta, omega] state with innovation e and variance r.
AxisState joseph(const Prediction& p,double e,double r) {
  const double s=p.pp+r, kp=p.pp/s, kv=p.pv/s, a=1-kp;
  return {p.theta+kp*e, p.omega+kv*e,
          a*a*p.pp+kp*kp*r,
          a*(p.pv-kv*p.pp)+kp*kv*r,
          p.vv-2*kv*p.pv+kv*kv*(p.pp+r)};
}
}

bool valid(const EstimatorParameters& p) {
  for (int i=0;i<2;++i)
    if (!(std::isfinite(p.process_density[i]) && p.process_density[i]>0 && std::isfinite(p.measurement_floor[i]) &&
          p.measurement_floor[i]>0 && std::isfinite(p.scale_max[i]) && p.scale_max[i]>=1)) return false;
  return std::isfinite(p.initial_rate_sigma) && p.initial_rate_sigma>0 && std::isfinite(p.scale_tau_s) && p.scale_tau_s>0 &&
         std::isfinite(p.rate_domain) && p.rate_domain>0 && std::isfinite(p.fresh_s) && p.fresh_s>=0 &&
         std::isfinite(p.horizon_s) && p.horizon_s>=p.fresh_s && p.horizon_s>0 &&
         std::isfinite(p.position_sigma_limit) && p.position_sigma_limit>=0;
}

double coast_fade(double age,double fresh,double horizon) {
  if (age<=fresh) return 1.;
  if (age>=horizon) return 0.;
  const double u=(age-fresh)/(horizon-fresh);
  return 1-3*u*u+2*u*u*u;
}

double coast_integral(double age,double fresh,double horizon) {
  age=std::max(age,0.);
  if (age<=fresh) return age;
  if (horizon<=fresh) return fresh;
  const double u=std::min(1.,(age-fresh)/(horizon-fresh));
  return fresh+(horizon-fresh)*(u-u*u*u+u*u*u*u/2);
}

double velocity_weight(double omega,double sigma) {
  if (!std::isfinite(omega) || !std::isfinite(sigma) || sigma<0) return 0.;
  if (sigma==0) return omega!=0 ? 1. : 0.;  // exact zero variance: mathematical tests only
  const double s=std::clamp((std::abs(omega)/sigma-1)/2,0.,1.);
  return s*s*(3-2*s);
}

bool TargetEstimator::configure(const EstimatorParameters& p) {
  if (!valid(p)) return false;
  p_=p; configured_=true; reset(); return true;
}

void TargetEstimator::reset() {
  initialized_=false; x_={}; t_ns_=0; beyond_domain_=0;
  const auto counts=d_; d_={};
  d_.accepted=counts.accepted; d_.rejected=counts.rejected; d_.downweighted=counts.downweighted;
  d_.reacquired=counts.reacquired; d_.gap_resets=counts.gap_resets;
}

void TargetEstimator::initialise(const LosObservation& z,const double r[2]) {
  const double angle[2]={z.az,z.el};
  for (int i=0;i<2;++i) x_[i]={angle[i],0.,r[i],0.,p_.initial_rate_sigma*p_.initial_rate_sigma};
  t_ns_=z.t_ns; initialized_=true; beyond_domain_=0; d_.scale={1.,1.};
}

bool TargetEstimator::update(const LosObservation& z) {
  d_.last_accepted=false;
  if (!configured_ || !std::isfinite(z.az) || !std::isfinite(z.el) || std::abs(z.el)>M_PI/2 ||
      !std::isfinite(z.var_az) || !std::isfinite(z.var_el) || z.var_az<0 || z.var_el<0 || z.t_ns<=0 ||
      (initialized_ && z.t_ns<=t_ns_)) {
    ++d_.rejected; return false;
  }
  const double r[2]={std::max(p_.measurement_floor[0],z.var_az),std::max(p_.measurement_floor[1],z.var_el)};
  const double dt=initialized_?(z.t_ns-t_ns_)*1e-9:0.;
  if (!initialized_ || dt>p_.horizon_s) {
    // Beyond the prediction domain the old motion is not evidence: start again, velocity unknown.
    if (initialized_) ++d_.gap_resets;
    initialise(z,r);
    d_.last_accepted=true; ++d_.accepted; return true;
  }
  const double angle[2]={z.az,z.el};
  double scale[2], e[2], nis=0;
  Prediction p[2];
  for (int i=0;i<2;++i) {
    scale[i]=1+(d_.scale[i]-1)*std::exp(-dt/p_.scale_tau_s);
    p[i]=predict(x_[i],dt,scale[i]*p_.process_density[i]);
    e[i]=i==0?wrap(angle[i]-p[i].theta):angle[i]-p[i].theta;
    nis+=e[i]*e[i]/(p[i].pp+r[i]);
  }
  if (!std::isfinite(nis)) { ++d_.rejected; return false; }
  if (nis>kNisGate) {
    // Raise the manoeuvre uncertainty once, from the pre-update state (Q is not added twice).
    const double raise=nis/kNisGate;
    nis=0;
    for (int i=0;i<2;++i) {
      scale[i]=std::max(scale[i],std::min(p_.scale_max[i],raise));
      p[i]=predict(x_[i],dt,scale[i]*p_.process_density[i]);
      nis+=e[i]*e[i]/(p[i].pp+r[i]);
    }
  }
  // Robust update: inflate R by 1/w for a large whitened innovation (an equivalent noise model,
  // applied to the covariance too, so the published uncertainty is honest).
  const double norm=std::sqrt(nis);
  const double w=std::min(1.,std::sqrt(kNisGate)/std::max(norm,1e-12));
  if (w<1) ++d_.downweighted;
  for (int i=0;i<2;++i) {
    x_[i]=joseph(p[i],e[i],r[i]/w);
    d_.innovation[i]=e[i]; d_.scale[i]=scale[i];
  }
  x_[1].theta=std::clamp(x_[1].theta,-M_PI/2,M_PI/2);
  d_.nis=nis; d_.weight=w; t_ns_=z.t_ns;
  // Outside the validated motion domain twice in a row: reacquire with the same identity.
  if (std::abs(x_[0].omega)>p_.rate_domain || std::abs(x_[1].omega)>p_.rate_domain) {
    if (++beyond_domain_>=2) {
      for (auto& a:x_) { a.omega=0; a.pv=0; a.vv=p_.initial_rate_sigma*p_.initial_rate_sigma; }
      beyond_domain_=0; ++d_.reacquired;
    }
  } else beyond_domain_=0;
  d_.last_accepted=true; ++d_.accepted;
  return true;
}

LosGoal TargetEstimator::query(int64_t t_eval_ns,bool use_motion) const {
  LosGoal g;
  if (!initialized_) return g;
  g.state_ns=t_ns_;
  g.age_s=std::max(0.,(t_eval_ns-t_ns_)*1e-9);
  if (g.age_s>p_.horizon_s) return g;  // beyond H: no extrapolation; the caller holds (LostHold)
  g.fade=coast_fade(g.age_s,p_.fresh_s,p_.horizon_s);
  const double travel=coast_integral(g.age_s,p_.fresh_s,p_.horizon_s);
  g.position_valid=true; g.velocity_valid=true;
  for (int i=0;i<2;++i) {
    const auto& a=x_[i];
    g.rate[i]=a.omega; g.rate_sigma[i]=std::sqrt(std::max(0.,a.vv));
    const double w=use_motion?velocity_weight(a.omega,g.rate_sigma[i]):0.;
    g.ff_weight[i]=w;
    const double v=w*a.omega;
    g.theta[i]=a.theta+v*travel;
    g.omega[i]=v*g.fade;
    if (p_.position_sigma_limit>0) {
      const auto q=predict(a,g.age_s,d_.scale[i]*p_.process_density[i]);
      if (std::sqrt(std::max(0.,q.pp))>p_.position_sigma_limit) g.position_valid=false;
    }
    if (!std::isfinite(g.rate_sigma[i])) g.velocity_valid=false;
  }
  g.theta[1]=std::clamp(g.theta[1],-M_PI/2,M_PI/2);
  if (!g.position_valid) g.velocity_valid=false;
  return g;
}
}
