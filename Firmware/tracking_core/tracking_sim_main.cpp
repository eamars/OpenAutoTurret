// tracking-sim REQUEST.json TICKS.npy FRAMES.npy -- ADR-003 closed-loop tracking simulation
// (tracking_sim.hpp). Writes the control-tick rows and the camera-frame rows as float64 .npy
// matrices and one JSON status line (columns and the applied parameters) on stdout.
#include "npy.hpp"
#include "tracking_sim.hpp"
#include <iostream>

namespace {
void columns(const std::vector<std::string>& names) {
  std::cout<<'[';
  for (std::size_t k=0;k<names.size();++k) std::cout<<(k?",":"")<<'"'<<names[k]<<'"';
  std::cout<<']';
}
}

int main(int argc,char** argv) {
  if (argc!=4) { std::cerr<<"tracking-sim REQUEST.json TICKS.npy FRAMES.npy\n"; return 2; }
  try {
    const auto result=ota::track::simulate_tracking(YAML::LoadFile(argv[1]));
    ota::axis::write_npy(argv[2],result.ticks,result.tick_columns.size());
    ota::axis::write_npy(argv[3],result.frames,result.frame_columns.size());
    std::cout<<"{\"status\":\""<<result.status<<"\",\"tick_columns\":"; columns(result.tick_columns);
    std::cout<<",\"frame_columns\":"; columns(result.frame_columns);
    std::cout<<",\"parameters\":"<<result.parameters<<"}\n";
    return 0;
  } catch (const std::exception& e) {
    std::cout<<"{\"status\":\"INVALID\",\"detail\":\""<<e.what()<<"\"}\n"; return 1;
  }
}
