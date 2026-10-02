#include "tracking_config.hpp"
#include <sstream>
#include <stdexcept>

namespace ota::track {
namespace {
double number(const YAML::Node& n,const char* key) {
  if (!n[key]) throw std::runtime_error(std::string("DATA_INVALID: tracking parameter missing: ")+key);
  return n[key].as<double>();
}
std::array<double,2> pair(const YAML::Node& n,const char* key) {
  if (!n[key] || !n[key].IsSequence() || n[key].size()!=2)
    throw std::runtime_error(std::string("DATA_INVALID: tracking parameter needs [azimuth, elevation]: ")+key);
  return {n[key][0].as<double>(),n[key][1].as<double>()};
}
Level1Axis axis(const YAML::Node& n) {
  if (!n) throw std::runtime_error("DATA_INVALID: level1 axis missing");
  Level1Axis a;
  a.lambda=number(n,"lambda_rad_s"); a.v_max=number(n,"v_max_rad_s"); a.a_max=number(n,"a_max_rad_s2");
  a.j_max=number(n,"j_max_rad_s3"); a.lead_limit=number(n,"lead_limit_rad");
  a.q_min=number(n,"q_min_rad"); a.q_max=number(n,"q_max_rad");
  return a;
}
}

TrackerParameters tracker_from_yaml(const YAML::Node& n) {
  if (!n || n["schema"].as<std::string>("")!="ota.tracking/1") throw std::runtime_error("DATA_INVALID: tracking schema");
  TrackerParameters p;
  const auto e=n["estimator"];
  if (!e) throw std::runtime_error("DATA_INVALID: estimator missing");
  p.estimator.process_density=pair(e,"process_density_rad2_s3");
  p.estimator.measurement_floor=pair(e,"measurement_floor_rad2");
  p.estimator.initial_rate_sigma=number(e,"initial_rate_sigma_rad_s");
  p.estimator.scale_max=pair(e,"scale_max");
  p.estimator.scale_tau_s=number(e,"scale_tau_s");
  p.estimator.rate_domain=number(e,"rate_domain_rad_s");
  p.estimator.fresh_s=number(e,"fresh_s");
  p.estimator.horizon_s=number(e,"horizon_s");
  p.estimator.position_sigma_limit=number(e,"position_sigma_limit_rad");
  const auto l=n["level1"];
  if (!l) throw std::runtime_error("DATA_INVALID: level1 missing");
  p.level1.period_s=number(l,"period_s"); p.level1.valid_s=number(l,"valid_s");
  p.level1.axis[0]=axis(l["yaw"]); p.level1.axis[1]=axis(l["pitch"]);
  const auto t=n["timing"];
  if (!t) throw std::runtime_error("DATA_INVALID: timing missing");
  p.timing.fixed_offset_s=number(t,"fixed_offset_s"); p.timing.exposure_fraction=number(t,"exposure_fraction");
  p.timing.row_time_s=number(t,"row_time_s");
  p.execution_horizon_s=number(n,"execution_horizon_s");
  p.pixel_sigma=number(n,"pixel_sigma_px");
  if (!n["target_motion"]) throw std::runtime_error("DATA_INVALID: target_motion missing");
  p.target_motion=n["target_motion"].as<bool>();
  p.jacobian_det_min=number(n,"jacobian_det_min");
  if (!valid(p)) throw std::runtime_error("DATA_INVALID: tracking parameters out of range");
  return p;
}

std::string tracker_to_json(const TrackerParameters& p) {
  std::ostringstream o; o.precision(9);
  const auto pr=[&](const std::array<double,2>& a) { o<<'['<<a[0]<<','<<a[1]<<']'; };
  const auto& e=p.estimator;
  o<<"{\"schema\":\"ota.tracking/1\",\"estimator\":{\"process_density_rad2_s3\":"; pr(e.process_density);
  o<<",\"measurement_floor_rad2\":"; pr(e.measurement_floor);
  o<<",\"initial_rate_sigma_rad_s\":"<<e.initial_rate_sigma<<",\"scale_max\":"; pr(e.scale_max);
  o<<",\"scale_tau_s\":"<<e.scale_tau_s<<",\"rate_domain_rad_s\":"<<e.rate_domain<<",\"fresh_s\":"<<e.fresh_s
   <<",\"horizon_s\":"<<e.horizon_s<<",\"position_sigma_limit_rad\":"<<e.position_sigma_limit<<"},\"level1\":{\"period_s\":"
   <<p.level1.period_s<<",\"valid_s\":"<<p.level1.valid_s;
  const char* names[2]={"yaw","pitch"};
  for (int i=0;i<2;++i) {
    const auto& a=p.level1.axis[i];
    o<<",\""<<names[i]<<"\":{\"lambda_rad_s\":"<<a.lambda<<",\"v_max_rad_s\":"<<a.v_max<<",\"a_max_rad_s2\":"<<a.a_max
     <<",\"j_max_rad_s3\":"<<a.j_max<<",\"lead_limit_rad\":"<<a.lead_limit<<",\"q_min_rad\":"<<a.q_min<<",\"q_max_rad\":"<<a.q_max<<'}';
  }
  o<<"},\"timing\":{\"fixed_offset_s\":"<<p.timing.fixed_offset_s<<",\"exposure_fraction\":"<<p.timing.exposure_fraction
   <<",\"row_time_s\":"<<p.timing.row_time_s<<"},\"execution_horizon_s\":"<<p.execution_horizon_s<<",\"pixel_sigma_px\":"
   <<p.pixel_sigma<<",\"target_motion\":"<<(p.target_motion?"true":"false")<<",\"jacobian_det_min\":"<<p.jacobian_det_min<<'}';
  return o.str();
}
}
