#include "replay.hpp"
#include "axis_control_core.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace ota::axis {
namespace {
bool read_parameters(std::istream& in,Parameters& p) {
  in>>p.model.n>>p.model.periodic;
  if(p.model.n!=5 && p.model.n!=8) return false;
  for(int j=0;j<p.model.n;++j) in>>p.model.q[j];
  for(auto& x:p.model.z) in>>x;
  for(int j=0;j<7+6*p.model.n;++j) in>>p.model.theta[j];
  auto& o=p.observer;
  in>>o.encoder_variance>>o.gyro_variance>>o.process_variance>>o.max_encoder_age_s
    >>o.max_gyro_age_s>>o.initial_position_variance>>o.initial_velocity_variance>>o.encoder_only_verified;
  in>>p.kp>>p.ki>>p.kpos>>p.kaw>>p.current_cap>>p.slew>>p.integral_cap>>p.velocity_cap
    >>p.dt_min>>p.dt_max>>p.intent_threshold>>p.rest_speed>>p.sustained_s>>p.start_timeout_s;
  for(int j=0;j<6*p.model.n;++j) in>>p.start_total[j];
  for(int j=0;j<6*p.model.n;++j) in>>p.start_censored[j];
  return bool(in) && valid(p);
}
}
int replay_file(const char* path) {
  std::ifstream in(path);std::string magic;in>>magic;
  if(!in || magic!="ADR0022_SYNTHETIC_REPLAY_V2") {std::cerr<<"DATA_INVALID: replay header\n";return 2;}
  Parameters control{},plant{};
  if(!read_parameters(in,control) || !read_parameters(in,plant)) {
    std::cerr<<"DATA_INVALID: complete replay parameters\n";return 2;
  }
  OtaSimulation s{};int n;double q,v;
  in>>s.dt>>s.encoder_quantum>>s.encoder_noise>>s.gyro_noise>>s.measurement_delay>>s.gyro_filter_tau
    >>s.encoder_period>>s.gyro_period>>s.seed>>n>>q>>v;
  if(!in || n<2 || n>1000000) {std::cerr<<"DATA_INVALID: bounded replay length\n";return 2;}
  std::vector<Reference> refs(n);std::vector<double> trace(static_cast<std::size_t>(n)*12);
  for(auto& r:refs) in>>r.position>>r.velocity>>r.acceleration>>r.posture;
  if(!in) {std::cerr<<"DATA_INVALID: truncated reference\n";return 2;}
  std::string tail;if(in>>tail) {std::cerr<<"DATA_INVALID: trailing replay data\n";return 2;}
  const int status=ota_closed_rollout(&control,&plant,&s,n,refs.data(),q,v,trace.data());
  if(status) {std::cerr<<"DATA_INVALID: replay failed "<<status<<'\n';return 2;}
  std::cout<<"# execution=SYNTHETIC route=offline_shared_core_replay physical_qualification=NOT_RUN\n";
  std::cout<<std::setprecision(17);
  for(int k=0;k<n;++k) {
    for(int j=0;j<12;++j) {if(j) std::cout<<',';std::cout<<trace[12*k+j];}
    std::cout<<'\n';
  }
  return 0;
}
}
