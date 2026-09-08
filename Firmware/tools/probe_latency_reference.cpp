// Offline measurement of the production tracking reference. No device code.
#include <iostream>
#include "control/tracking_reference.hpp"
#include "tracking/auto_track_controller.hpp"

int main(int argc, char**) {
  constexpr double rad = 3.14159265358979323846/180;
  if (argc>1) {
    std::cout << "frame_rate_hz,tracking_after_ms\n";
    for (const int hz : {26,52}) {
      ota::AutoTrackController controller;
      ota::AutoTrackInput in;
      in.has_selection=in.target_visible=in.estimator_ready=true;
      in.track_confidence=1; in.visible_frames=20;
      for (int tick=0; tick<100; ++tick) {
        const int64_t elapsed=tick*5'000'000LL;
        const int64_t frame=elapsed*hz/1'000'000'000LL;
        in.measurement_timestamp_ns=10'000'000'000LL+frame*1'000'000'000LL/hz-60'000'000LL;
        in.measurement_age_ms=(10'000'000'000LL+elapsed-in.measurement_timestamp_ns)/1'000'000;
        if (controller.update(in,10'000'000'000LL+elapsed).state==ota::AutoTrackState::Tracking) {
          std::cout << hz << ',' << tick*5 << '\n';
          break;
        }
      }
    }
    return 0;
  }
  std::cout << "step_deg,time_s,reference_deg,reference_speed_deg_s\n";
  for (const double amplitude : {1.,5.,10.}) {
    ota::control::ReferenceLimiter state;
    state.reset_at(0);
    for (int tick=0; tick<=1200; ++tick) {
      if (tick) ota::control::track_reference(state,amplitude*rad,0,.005,
          20*rad,30*rad,100*rad);
      std::cout << amplitude << ',' << tick*.005 << ',' << state.q_rad/rad
                << ',' << state.v_rad_s/rad << '\n';
    }
  }
}
