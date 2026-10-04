#include "mode/watch_point.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

namespace ota {

std::string watch_point_to_json(const WatchPoint& w) {
  std::ostringstream o;
  o << std::setprecision(17);
  o << "{\"schema\":1"
    << ",\"yaw_frame\":\"" << (w.yaw_absolute ? "gm6020_absolute" : "homed") << "\""
    << ",\"yaw_rad\":" << w.yaw_rad
    << ",\"pitch_homed_rad\":" << w.pitch_homed_rad
    << ",\"saved_unix_s\":" << w.saved_unix_s << "}\n";
  return o.str();
}

bool watch_point_from_json(const std::string& text, WatchPoint& out, std::string& err) {
  out = WatchPoint{};
  try {
    const YAML::Node n = YAML::Load(text);
    if (!n.IsMap()) { err = "not a JSON object"; return false; }
    if (!n["schema"] || n["schema"].as<int>() != 1) { err = "schema is not 1"; return false; }
    const std::string frame = n["yaw_frame"] ? n["yaw_frame"].as<std::string>() : "";
    if (frame != "gm6020_absolute" && frame != "homed") {
      err = "yaw_frame must be gm6020_absolute or homed, not '" + frame + "'";
      return false;
    }
    if (!n["yaw_rad"] || !n["pitch_homed_rad"]) { err = "yaw_rad and pitch_homed_rad are required"; return false; }
    const double yaw = n["yaw_rad"].as<double>();
    const double pitch = n["pitch_homed_rad"].as<double>();
    if (!std::isfinite(yaw) || !std::isfinite(pitch)) { err = "non-finite angle"; return false; }
    out.yaw_absolute = frame == "gm6020_absolute";
    out.yaw_rad = yaw;
    out.pitch_homed_rad = pitch;
    out.saved_unix_s = n["saved_unix_s"] ? n["saved_unix_s"].as<int64_t>() : 0;
    out.valid = true;
    err.clear();
    return true;
  } catch (const std::exception& e) {
    err = std::string("unparseable: ") + e.what();
    return false;
  }
}

bool load_watch_point(const std::string& path, WatchPoint& out, std::string& err) {
  out = WatchPoint{};
  err.clear();
  std::ifstream f(path);
  if (!f) return false;  // no point saved yet: not an error
  std::stringstream ss;
  ss << f.rdbuf();
  return watch_point_from_json(ss.str(), out, err);
}

bool save_watch_point(const std::string& path, const WatchPoint& w, std::string& err) {
  const std::string tmp = path + ".tmp";
  {
    std::ofstream f(tmp, std::ios::trunc);
    if (!f) { err = "cannot open " + tmp + " for writing"; return false; }
    f << watch_point_to_json(w);
    f.flush();
    if (!f.good()) { err = "write failed for " + tmp; return false; }
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    err = "rename " + tmp + " -> " + path + " failed";
    return false;
  }
  err.clear();
  return true;
}

WatchPointStore::WatchPointStore(std::string path) : path_(std::move(path)) {
  if (persistent()) thread_ = std::thread([this] { run(); });
}

WatchPointStore::~WatchPointStore() {
  {
    std::lock_guard lk(mutex_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void WatchPointStore::save(const WatchPoint& w) {
  if (!persistent()) {
    last_result_.store(0);
    return;
  }
  {
    std::lock_guard lk(mutex_);
    pending_ = w;
  }
  cv_.notify_all();
}

void WatchPointStore::run() {
  std::unique_lock lk(mutex_);
  for (;;) {
    cv_.wait(lk, [this] { return stop_ || pending_.has_value(); });
    if (pending_) {
      const WatchPoint w = *pending_;
      pending_.reset();
      lk.unlock();
      std::string err;
      const bool ok = save_watch_point(path_, w, err);
      last_result_.store(ok ? 1 : 0);
      if (ok) spdlog::info("watch point saved to {}", path_);
      else spdlog::error("watch point NOT saved ({}); it holds for this session only", err);
      lk.lock();
      continue;  // a save queued during the write runs before stopping
    }
    if (stop_) return;
  }
}

}  // namespace ota
