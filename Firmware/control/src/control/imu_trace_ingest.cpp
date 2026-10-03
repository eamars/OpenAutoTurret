#include "control/imu_trace_ingest.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>
#include <vector>

#include "common/thread_class.hpp"
#include "common/time.hpp"

namespace ota::control {
namespace {

bool field_start(const std::string& row, const char* key, size_t& pos) {
  const std::string token = std::string("\"") + key + "\"";
  pos = row.find(token);
  if (pos == std::string::npos) return false;
  pos = row.find(':', pos + token.size());
  if (pos == std::string::npos) return false;
  ++pos;
  while (pos < row.size() && (row[pos] == ' ' || row[pos] == '\t')) ++pos;
  return pos < row.size();
}

bool number_field(const std::string& row, const char* key, int64_t& value) {
  size_t pos;
  if (!field_start(row, key, pos)) return false;
  char* end = nullptr;
  errno = 0;
  const long long parsed = std::strtoll(row.c_str() + pos, &end, 10);
  if (errno || end == row.c_str() + pos) return false;
  value = static_cast<int64_t>(parsed);
  return true;
}

bool string_field(const std::string& row, const char* key, std::string& value) {
  size_t pos;
  if (!field_start(row, key, pos) || row[pos] != '"') return false;
  const size_t begin = ++pos;
  const size_t end = row.find('"', begin);
  if (end == std::string::npos) return false;
  value.assign(row, begin, end - begin);
  return true;
}

template <size_t N>
bool array_field(const std::string& row, const char* key,
                 std::array<double, N>& values) {
  size_t pos;
  if (!field_start(row, key, pos) || row[pos] != '[') return false;
  ++pos;
  for (size_t i = 0; i < N; ++i) {
    while (pos < row.size() && (row[pos] == ' ' || row[pos] == '\t')) ++pos;
    char* end = nullptr;
    errno = 0;
    const double parsed = std::strtod(row.c_str() + pos, &end);
    if (errno || end == row.c_str() + pos || !std::isfinite(parsed)) return false;
    values[i] = parsed;
    pos = static_cast<size_t>(end - row.c_str());
    while (pos < row.size() && (row[pos] == ' ' || row[pos] == '\t')) ++pos;
    if (i + 1 < N) {
      if (pos >= row.size() || row[pos] != ',') return false;
      ++pos;
    }
  }
  return pos < row.size() && row[pos] == ']';
}

bool valid_quaternion(const std::array<double, 4>& q) {
  double n2 = 0;
  for (double v : q) n2 += v * v;
  return n2 >= 0.9801 && n2 <= 1.0201;
}

bool valid_generation(int64_t generation) {
  return generation >= 0 && generation <= UINT32_MAX;
}

}  // namespace

bool ImuTraceIngest::start(std::string path, std::string& err) {
  if (running_.load() || reader_thread_.joinable()) {
    err = "IMU trace ingest already started";
    return false;
  }
  if (path.empty()) {
    err = "IMU trace path is empty";
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mu_);
    state_ = ImuTraceSnapshot{};
  }
  path_ = std::move(path);
  running_.store(true);
  reader_thread_ = std::thread([this] { reader_loop(); });
  err.clear();
  return true;
}

void ImuTraceIngest::stop() {
  running_.store(false);
  if (reader_thread_.joinable()) reader_thread_.join();
}

ImuTraceSnapshot ImuTraceIngest::snapshot(TimeNs now_ns) const {
  std::lock_guard<std::mutex> lock(state_mu_);
  ImuTraceSnapshot result = state_;
  result.game_rv_fresh = result.game_rv_present && now_ns >= result.game_rv_sample_ns &&
      now_ns - result.game_rv_sample_ns <= freshness_ns_;
  result.gyro_fresh = result.gyro_present && now_ns >= result.gyro_sample_ns &&
      now_ns - result.gyro_sample_ns <= freshness_ns_;
  return result;
}

void ImuTraceIngest::reader_loop() {
  apply_thread_class("imu-observer", ThreadClass::Background);
  while (running_.load()) {
    std::ifstream input(path_, std::ios::in | std::ios::binary);
    if (!input) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(state_mu_);
      state_.trace_open = true;
      state_.trace_ended = false;
    }
    std::string line;
    uintmax_t offset = 0;
    while (running_.load()) {
      if (std::getline(input, line)) {
        const std::streampos next = input.tellg();
        if (next >= 0) offset = static_cast<uintmax_t>(next);
        consume_line(line, now_monotonic_ns());
        continue;
      }
      input.clear();
      std::error_code ec;
      const uintmax_t size = std::filesystem::file_size(path_, ec);
      if (!ec && size < offset) {
        // A launcher-side truncation starts a new trace epoch. Old samples and
        // tare must not masquerade as data from the replacement contents.
        input.seekg(0);
        offset = 0;
        std::lock_guard<std::mutex> lock(state_mu_);
        state_.tare_valid = false;
        state_.game_rv_present = false;
        state_.gyro_present = false;
        state_.gap_seen = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    {
      std::lock_guard<std::mutex> lock(state_mu_);
      state_.trace_open = false;
    }
  }
}

void ImuTraceIngest::consume_line(const std::string& line, TimeNs read_ns) {
  std::string kind;
  if (!string_field(line, "kind", kind)) return;
  int64_t generation = 0;
  (void)number_field(line, "generation", generation);
  std::lock_guard<std::mutex> lock(state_mu_);
  state_.last_trace_rx_ns = read_ns;

  if (kind == "gap" || kind == "trace_reset") {
    state_.gap_seen = true;
    state_.tare_valid = false;
    state_.game_rv_present = false;
    state_.gyro_present = false;
    return;
  }
  if (kind == "summary") {
    state_.trace_ended = true;
    return;
  }
  if (kind == "tare") {
    std::array<double, 4> q{};
    if (!number_field(line, "generation", generation) ||
        !valid_generation(generation) ||
        !number_field(line, "rx_ns", state_.tare_rx_ns) || state_.tare_rx_ns <= 0 ||
        !array_field(line, "q_ref_xyzw", q) || !valid_quaternion(q)) {
      state_.tare_valid = false;
      return;
    }
    state_.generation = static_cast<uint32_t>(generation);
    state_.tare_generation = static_cast<uint32_t>(generation);
    state_.tare_valid = true;
    state_.tare_xyzw = q;
    state_.gap_seen = false;
    return;
  }
  if (kind != "sample") return;

  std::string sensor;
  if (!string_field(line, "sensor", sensor) ||
      !number_field(line, "generation", generation)) return;
  std::array<double, 3> gyro{};
  std::array<double, 4> q{};
  int64_t sample_ns = 0, rx_ns = 0, status = 0;
  if (!valid_generation(generation) ||
      !number_field(line, "sample_ns", sample_ns) || sample_ns <= 0 ||
      !number_field(line, "rx_ns", rx_ns) || rx_ns <= 0) return;
  state_.generation = static_cast<uint32_t>(generation);
  if (sensor == "game_rv") {
    if (!number_field(line, "status", status) || status < 0 || status > 3 ||
        !array_field(line, "values", q) || !valid_quaternion(q)) return;
    state_.game_rv_xyzw = q;
    state_.game_rv_generation = static_cast<uint32_t>(generation);
    state_.game_rv_sample_ns = sample_ns;
    state_.game_rv_rx_ns = rx_ns;
    state_.game_rv_present = true;
    state_.game_rv_accuracy = static_cast<uint32_t>(status);
    state_.game_rv_tared =
        array_field(line, "relative_xyzw", state_.relative_xyzw) &&
        state_.tare_valid && state_.tare_generation == state_.game_rv_generation;
    return;
  }
  if (sensor == "gyro") {
    if (!array_field(line, "values", gyro)) return;
    state_.gyro_rad_s = gyro;
    state_.gyro_generation = static_cast<uint32_t>(generation);
    state_.gyro_sample_ns = sample_ns;
    state_.gyro_rx_ns = rx_ns;
    state_.gyro_present = true;
  }
}

}  // namespace ota::control
