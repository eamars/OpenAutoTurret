#include "plant.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace ota::axis {
namespace {
constexpr double kStep=1e-4;  // integration substep, s
template <std::size_t N>
void table(const YAML::Node& node,const char* key,double (&target)[N]) {
  std::fill(target,target+N,0.);
  if (!node[key]) return;
  const auto values=node[key].as<std::vector<double>>();
  if (values.size()!=N) throw std::runtime_error(std::string("DATA_INVALID: plant table size: ")+key);
  std::copy(values.begin(),values.end(),target);
}
double periodic(const double* map,int n,double q) {
  const double x=std::fmod(std::fmod(q,2*M_PI)+2*M_PI,2*M_PI)/(2*M_PI)*n;
  const int k0=static_cast<int>(std::floor(x))%n,k1=(k0+1)%n; const double w=x-std::floor(x);
  return (1-w)*map[k0]+w*map[k1];
}
double latest(const std::deque<std::pair<double,double>>& history,double time) {
  double value=0.;
  for (const auto& [t,v]:history) { if (t<=time) value=v; else break; }
  return value;
}
void prune(std::deque<std::pair<double,double>>& history,double time) {
  while (history.size()>1 && history[1].first<=time-0.1) history.pop_front();
}
void check(bool ok,const char* what) { if (!ok) throw std::runtime_error(std::string("DATA_INVALID: plant ")+what); }
}

YawPlantParameters yaw_plant_from_yaml(const YAML::Node& n) {
  YawPlantParameters p{};
  p.inertia=n["inertia"].as<double>();
  p.coulomb_positive=n["coulomb_positive"].as<double>(); p.coulomb_negative=n["coulomb_negative"].as<double>();
  p.stribeck_positive=n["stribeck_positive"].as<double>(); p.stribeck_negative=n["stribeck_negative"].as<double>();
  p.stribeck_speed=n["stribeck_speed"].as<double>(); p.viscous=n["viscous"].as<double>();
  p.creep_drop=n["creep_drop"].as<double>(0.); p.creep_speed=n["creep_speed"].as<double>(0.01);
  p.presliding_stiffness=n["presliding_stiffness"].as<double>(); p.presliding_damping=n["presliding_damping"].as<double>();
  p.load=n["load"].as<double>(0.);
  table(n,"friction_map_positive",p.friction_map_positive); table(n,"friction_map_negative",p.friction_map_negative);
  p.actuation_delay_s=n["actuation_delay_s"].as<double>(); p.current_tau_s=n["current_tau_s"].as<double>();
  p.encoder_delay_s=n["encoder_delay_s"].as<double>(); p.crosstalk_delay_s=n["crosstalk_delay_s"].as<double>(0.);
  table(n,"crosstalk_map",p.crosstalk_map);
  p.counts_per_rev=n["counts_per_rev"].as<double>(8192.);
  check(p.inertia>0 && p.stribeck_speed>0 && p.counts_per_rev>0 && p.presliding_stiffness>0,"inertia/stribeck_speed/counts/stiffness");
  check(p.creep_speed>0,"creep_speed");
  for (double v:{p.coulomb_positive,p.coulomb_negative,p.stribeck_positive,p.stribeck_negative,p.viscous,p.presliding_damping,p.creep_drop,
                 p.actuation_delay_s,p.current_tau_s,p.encoder_delay_s,p.crosstalk_delay_s})
    check(std::isfinite(v) && v>=0,"nonnegative terms");
  return p;
}

YawPlant::YawPlant(const YawPlantParameters& p,double position) : p_(p), q_(position) { commands_.push_back({-1e9,0.}); }
void YawPlant::command(double time,double current) { commands_.push_back({time,current}); }
double YawPlant::commanded(double time) const { return latest(commands_,time); }
// Steady sliding friction magnitude at speed v (the LuGre g(v)), at least a trace above zero.
double YawPlant::friction(double v,double q) const {
  const bool positive=v>=0;
  const double magnitude=(positive?p_.coulomb_positive:p_.coulomb_negative)+
    (positive?p_.stribeck_positive:p_.stribeck_negative)*std::exp(-std::abs(v)/p_.stribeck_speed)-
    p_.creep_drop*std::exp(-std::abs(v)/p_.creep_speed)+
    periodic(positive?p_.friction_map_positive:p_.friction_map_negative,YawPlantParameters::kFrictionBins,q);
  return std::max(1e-3,magnitude);
}
void YawPlant::advance(double to) {
  // LuGre: dz/dt = v - sigma0*|v|*z/g(v);  F = sigma0*z + sigma1*dz/dt + viscous*v.
  // z is integrated exactly for the substep's frozen v (stiff when sliding fast).
  const double s0=p_.presliding_stiffness,s1=p_.presliding_damping;
  while (t_<to-1e-12) {
    const double h=std::min(kStep,to-t_);
    const double u=commanded(t_+h-p_.actuation_delay_s);
    i_=p_.current_tau_s>0?i_+(u-i_)*(1-std::exp(-h/p_.current_tau_s)):u;
    const double k=s0*std::abs(v_)/friction(v_,q_);
    const double z_next=k>1e-12?v_/k+(z_-v_/k)*std::exp(-k*h):z_+v_*h;
    const double dz=(z_next-z_)/h;
    const double force=s0*z_next+s1*dz+p_.viscous*v_;
    // Semi-implicit in the damping terms keeps the stiff contact stable at this step.
    const double damping=s1*(k>1e-12?(1-std::exp(-k*h))/(k*h):1.)+p_.viscous;
    const double next=(p_.inertia*v_+h*(i_-p_.load-force+damping*v_))/(p_.inertia+h*damping);
    q_+=0.5*(v_+next)*h; v_=next; z_=z_next; t_+=h;
  }
  prune(commands_,t_-p_.actuation_delay_s-p_.crosstalk_delay_s);
}
double YawPlant::reading(double sampled,double receipt) const {
  const double u=commanded(receipt-p_.crosstalk_delay_s);
  const double q=sampled+periodic(p_.crosstalk_map,YawPlantParameters::kCrosstalkBins,sampled)*u;
  const double quantum=2*M_PI/p_.counts_per_rev;
  return std::round(q/quantum)*quantum;
}

PitchPlantParameters pitch_plant_from_yaml(const YAML::Node& n) {
  PitchPlantParameters p{};
  p.speed_gain=n["speed_gain"].as<double>(1.);
  p.speed_delay_s=n["speed_delay_s"].as<double>(); p.speed_tau_s=n["speed_tau_s"].as<double>();
  check(p.speed_gain>0,"pitch speed gain");
  p.acceleration_limit=n["acceleration_limit_rad_s2"].as<double>(0.);
  p.reply_delay_s=n["reply_delay_s"].as<double>(); p.position_quantum_rad=n["position_quantum_rad"].as<double>(8*M_PI/65535);
  for (double v:{p.speed_delay_s,p.speed_tau_s,p.acceleration_limit,p.reply_delay_s,p.position_quantum_rad})
    check(std::isfinite(v) && v>=0,"pitch terms");
  return p;
}
PitchPlant::PitchPlant(const PitchPlantParameters& p,double position) : p_(p), q_(position) { commands_.push_back({-1e9,0.}); }
void PitchPlant::command(double time,double speed) { commands_.push_back({time,speed}); }
void PitchPlant::advance(double to) {
  while (t_<to-1e-12) {
    const double h=std::min(kStep,to-t_);
    const double target=p_.speed_gain*latest(commands_,t_+h-p_.speed_delay_s);
    double next=p_.speed_tau_s>0?v_+(target-v_)*(1-std::exp(-h/p_.speed_tau_s)):target;
    if (p_.acceleration_limit>0) next=std::clamp(next,v_-p_.acceleration_limit*h,v_+p_.acceleration_limit*h);
    q_+=0.5*(v_+next)*h; v_=next; t_+=h;
  }
  prune(commands_,t_-p_.speed_delay_s);
}
double PitchPlant::reading() const {
  return p_.position_quantum_rad>0?std::round(q_/p_.position_quantum_rad)*p_.position_quantum_rad:q_;
}
}
