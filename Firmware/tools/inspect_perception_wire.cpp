// Decode a saved vision packet with the production C++ decoder. No hardware I/O.
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>
#include "tracks/track_wire.hpp"
#include "tracks/perception_wire.hpp"
#include "tracking/target_measurement.hpp"

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  std::ifstream input(argv[1], std::ios::binary);
  if (!input) return 2;
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
  ota::tracks::TrackSet set;
  if (ota::tracks::decode_perception_frame(bytes.data(), bytes.size(), set) ||
      ota::tracks::decode_track_set(bytes.data(), bytes.size(), set)) {
    std::cout << "{\"native\":" << set.observation.native
              << ",\"generation\":" << set.observation.generation
              << ",\"selected_lo\":" << set.observation.selected.lo
              << ",\"measurement_valid\":" << set.observation.valid
              << ",\"count\":" << set.count << ",\"tracks\":[";
    for (int i = 0; i < set.count; ++i) {
      const auto& t = set.tracks[i];
      if (i) std::cout << ',';
      std::cout << "{\"class_id\":" << t.class_id << ",\"uuid_lo\":" << t.uuid.lo
                << ",\"visible_frames\":" << t.visible_frames
                << ",\"velocity_x\":" << t.velocity_x_norm_s << '}';
    }
    std::cout << "]}\n";
    return 0;
  }
  ota::vision::TargetMeasurement measurement;
  if (!ota::vision::TargetMeasurement::decode(bytes.data(), bytes.size(), measurement)) return 1;
  std::cout << "{\"valid\":" << (measurement.valid ? "true" : "false")
            << ",\"class_id\":" << measurement.class_id
            << ",\"track_id\":" << measurement.visual_track_id << "}\n";
}
