// Recorded predicted goals -> two production reference generators, no hardware.
// CSV columns (no header): seconds, yaw goal, pitch goal, yaw velocity,
// pitch velocity, measured yaw, measured pitch. Angular units are radians.
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include "control/tracking_reference.hpp"

int main(int argc, char** argv) {
  if (argc != 2) { std::cerr << "usage: replay-guidance recorded-goals.csv\n"; return 2; }
  std::ifstream file(argv[1]);
  std::string line;
  std::vector<std::array<double,7>> rows;
  while (std::getline(file,line)) {
    std::replace(line.begin(),line.end(),',',' ');
    std::istringstream input(line);
    std::array<double,7> row{};
    for (auto& value : row)
      if (!(input >> value) || !std::isfinite(value)) return 2;
    if (row[0] < 0 || (!rows.empty() && row[0] <= rows.back()[0])) return 2;
    rows.push_back(row);
  }
  if (rows.empty() || rows.back()[0] > 600) return 2;
  ota::control::ReferenceLimiter baseline[2], damped[2];
  for (int axis=0;axis<2;++axis) {
    baseline[axis].reset_at(rows[0][5+axis]);
    damped[axis].reset_at(rows[0][5+axis]);
  }
  constexpr double deg=3.14159265358979323846/180, dt=.005;
  std::cout << std::setprecision(12)
            << "time,axis,target,baseline,damped,baseline_v,damped_v\n";
  size_t cursor=0;
  // Hold the last observed goal with zero target velocity for 15 seconds.
  // Both variants receive exactly the same sampled input; this comparison
  // isolates guidance, not the estimator or physical closed-loop response.
  for (int tick=0;tick*dt<rows.back()[0]+15;++tick) {
    const double t=tick*dt;
    while (cursor+1<rows.size() && rows[cursor+1][0]<=t) ++cursor;
    const auto& row=rows[cursor];
    for (int axis=0;axis<2;++axis) {
      const double velocity=t>rows.back()[0] ? 0 : row[3+axis];
      ota::control::limit_reference(baseline[axis],row[1+axis],dt,15*deg,15*deg,60*deg);
      ota::control::track_reference(damped[axis],row[1+axis],velocity,dt,15*deg,15*deg,60*deg);
      if (tick%10==0)
        std::cout << t << ',' << axis << ',' << row[1+axis] << ','
                  << baseline[axis].q_rad << ',' << damped[axis].q_rad << ','
                  << baseline[axis].v_rad_s << ',' << damped[axis].v_rad_s << '\n';
    }
  }
}
