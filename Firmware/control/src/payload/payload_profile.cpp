// OpenAutoTurret — payload profile storage implementation (§28.5, §41).
#include "payload/payload_profile.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

namespace ota {
namespace payload {
namespace {

bool is_safe_name(const std::string& name) {
  if (name.empty() || name.size() > 64) return false;
  if (name == "." || name == "..") return false;
  for (char c : name) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' &&
        c != '-')
      return false;
  }
  return true;
}

double finite_or(const YAML::Node& n, double dflt) {
  if (!n.IsDefined() || !n.IsScalar()) return dflt;
  try {
    const double v = n.as<double>();
    return std::isfinite(v) ? v : dflt;
  } catch (...) {
    return dflt;
  }
}

YAML::Node metrics_node(const StepMetrics& m) {
  YAML::Node n;
  n["valid"] = m.valid;
  n["amplitude_rad"] = m.amplitude_rad;
  n["rise_time_s"] = m.rise_time_s;
  n["overshoot"] = m.overshoot;
  n["settling_time_s"] = m.settling_time_s;
  n["settle_band_rad"] = m.settle_band_rad;
  n["tracking_rms_rad"] = m.tracking_rms_rad;
  n["peak_effort_nm"] = m.peak_effort_nm;
  return n;
}

StepMetrics metrics_from(const YAML::Node& n) {
  StepMetrics m;
  if (!n.IsDefined() || !n.IsMap()) return m;
  m.valid = n["valid"] && n["valid"].IsScalar() && n["valid"].as<bool>();
  m.amplitude_rad = finite_or(n["amplitude_rad"], 0.0);
  m.rise_time_s = finite_or(n["rise_time_s"], 0.0);
  m.overshoot = finite_or(n["overshoot"], 0.0);
  m.settling_time_s = finite_or(n["settling_time_s"], 0.0);
  m.settle_band_rad = finite_or(n["settle_band_rad"], 0.0);
  m.tracking_rms_rad = finite_or(n["tracking_rms_rad"], 0.0);
  m.peak_effort_nm = finite_or(n["peak_effort_nm"], 0.0);
  return m;
}

YAML::Node axis_node(const AxisPayloadProfile& a) {
  YAML::Node n;
  n["v_max_rad_s"] = a.v_max_rad_s;
  n["a_max_rad_s2"] = a.a_max_rad_s2;
  n["j_max_rad_s3"] = a.j_max_rad_s3;
  n["gain_kp"] = a.gain_kp;
  n["gain_ki"] = a.gain_ki;
  n["gain_kd"] = a.gain_kd;
  n["baseline"] = YAML::Node();
  YAML::Node b;
  b["step_pos"] = metrics_node(a.baseline.step_pos);
  b["step_neg"] = metrics_node(a.baseline.step_neg);
  b["triangle_rms_rad"] = a.baseline.triangle_rms_rad;
  b["hold_effort_nm"] = a.baseline.hold_effort_nm;
  b["max_verified_speed_rad_s"] = a.baseline.max_verified_speed_rad_s;
  YAML::Node br;
  br["valid"] = a.baseline.brake.valid;
  br["v0_rad_s"] = a.baseline.brake.v0_rad_s;
  br["stop_distance_rad"] = a.baseline.brake.stop_distance_rad;
  br["stop_time_s"] = a.baseline.brake.stop_time_s;
  b["brake"] = br;
  b["valid"] = a.baseline.valid;
  n["baseline"] = b;
  return n;
}

AxisPayloadProfile axis_from(const YAML::Node& n) {
  AxisPayloadProfile a;
  if (!n.IsDefined() || !n.IsMap()) return a;
  a.v_max_rad_s = finite_or(n["v_max_rad_s"], 0.0);
  a.a_max_rad_s2 = finite_or(n["a_max_rad_s2"], 0.0);
  a.j_max_rad_s3 = finite_or(n["j_max_rad_s3"], 0.0);
  a.gain_kp = finite_or(n["gain_kp"], 0.0);
  a.gain_ki = finite_or(n["gain_ki"], 0.0);
  a.gain_kd = finite_or(n["gain_kd"], 0.0);
  const YAML::Node b = n["baseline"];
  a.baseline.step_pos = metrics_from(b["step_pos"]);
  a.baseline.step_neg = metrics_from(b["step_neg"]);
  a.baseline.triangle_rms_rad = finite_or(b["triangle_rms_rad"], 0.0);
  a.baseline.hold_effort_nm = finite_or(b["hold_effort_nm"], 0.0);
  a.baseline.max_verified_speed_rad_s =
      finite_or(b["max_verified_speed_rad_s"], 0.0);
  const YAML::Node br = b["brake"];
  a.baseline.brake.valid =
      br["valid"] && br["valid"].IsScalar() && br["valid"].as<bool>();
  a.baseline.brake.v0_rad_s = finite_or(br["v0_rad_s"], 0.0);
  a.baseline.brake.stop_distance_rad =
      finite_or(br["stop_distance_rad"], 0.0);
  a.baseline.brake.stop_time_s = finite_or(br["stop_time_s"], 0.0);
  a.baseline.valid = b["valid"] && b["valid"].IsScalar() &&
                     b["valid"].as<bool>() && a.baseline.step_pos.valid &&
                     a.baseline.step_neg.valid;
  return a;
}

}  // namespace

// Split a path on '/', returning non-empty components.
static std::vector<std::string> path_components(const std::string& p) {
  std::vector<std::string> parts;
  std::string cur;
  for (char c : p) {
    if (c == '/') {
      if (!cur.empty()) parts.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) parts.push_back(cur);
  return parts;
}

// Create `path` (and any missing parent directories) if it does not exist.
// Returns 0 on success or the errno of the failed mkdir.
static int ensure_dir(const std::string& path) {
  if (path.empty() || ::access(path.c_str(), F_OK) == 0) return 0;
  std::string base = path;
  if (base[0] != '/') {
    char cwd[4096];
    if (getcwd(cwd, sizeof(cwd)) == nullptr) return errno;
    base = std::string(cwd) + "/" + base;
  }
  // Walk the components, creating each missing ancestor in turn.
  std::string prefix;
  for (const auto& part : path_components(base)) {
    if (part == ".") continue;
    prefix += "/" + part;
    if (part == "..") continue;  // not expected; nothing to create
    if (::mkdir(prefix.c_str(), 0755) != 0 && errno != EEXIST) return errno;
  }
  return 0;
}

std::string payload_profile_to_yaml(const PayloadProfile& p) {
  YAML::Node n;
  n["schema_version"] = p.schema_version;
  n["name"] = p.name;
  n["created_ns"] = p.created_ns;
  n["config_revision"] = p.config_revision;
  n["hardware"] = p.hardware;
  n["notes"] = p.notes;
  n["pitch"] = axis_node(p.pitch);
  n["yaw"] = axis_node(p.yaw);
  YAML::Emitter out;
  out << n;
  return std::string(out.c_str());
}

bool payload_profile_from_yaml(const std::string& text, PayloadProfile& out,
                               std::string& err) {
  YAML::Node n;
  try {
    n = YAML::Load(text);
  } catch (const std::exception& e) {
    err = std::string("parse error: ") + e.what();
    return false;
  }
  if (!n.IsMap()) {
    err = "not a mapping";
    return false;
  }
  const int ver = finite_or(n["schema_version"], -1.0);
  if (ver != kPayloadProfileSchemaVersion) {
    err = "unsupported schema_version " + std::to_string(ver) +
          " (this build knows " +
          std::to_string(kPayloadProfileSchemaVersion) + ")";
    return false;
  }
  PayloadProfile p;
  p.schema_version = ver;
  p.name = n["name"].IsScalar() ? n["name"].as<std::string>() : "";
  if (p.name.empty() || !is_safe_name(p.name)) {
    err = "missing or unsafe profile name";
    return false;
  }
  p.created_ns = static_cast<int64_t>(finite_or(n["created_ns"], 0.0));
  p.config_revision =
      n["config_revision"].IsScalar() ? n["config_revision"].as<std::string>()
                                      : "";
  p.hardware =
      n["hardware"].IsScalar() ? n["hardware"].as<std::string>() : "";
  p.notes = n["notes"].IsScalar() ? n["notes"].as<std::string>() : "";
  p.pitch = axis_from(n["pitch"]);
  p.yaw = axis_from(n["yaw"]);
  out = std::move(p);
  return true;
}

bool PayloadProfileStore::load(const std::string& name, PayloadProfile& out,
                               std::string& err) const {
  if (!is_safe_name(name)) {
    err = "unsafe profile name '" + name + "'";
    return false;
  }
  const std::string path = dir_ + "/" + name + ".yaml";
  std::ifstream f(path);
  if (!f.good()) {
    err = "no such profile: " + path;
    return false;
  }
  std::string text((std::istreambuf_iterator<char>(f)),
                   std::istreambuf_iterator<char>());
  return payload_profile_from_yaml(text, out, err);
}

bool PayloadProfileStore::save(const PayloadProfile& p, std::string& err) const {
  if (!is_safe_name(p.name)) {
    err = "unsafe profile name '" + p.name + "'";
    return false;
  }
  const std::string final_path = dir_ + "/" + p.name + ".yaml";
  const std::string prev_path = final_path + ".prev";
  const std::string tmp_path = final_path + ".tmp";

  // Ensure the profile directory exists (commissioning on a fresh checkout).
  if (ensure_dir(dir_) != 0) {
    err = "cannot create profile directory " + dir_;
    return false;
  }

  // Write the tmp file fully and fsync it before publishing (§41 atomicity).
  {
    std::ofstream f(tmp_path, std::ios::trunc);
    if (!f.good()) {
      err = "cannot open " + tmp_path;
      return false;
    }
    f << payload_profile_to_yaml(p);
    f.flush();
    if (!f.good()) {
      err = "write error: " + tmp_path;
      return false;
    }
  }
  // fsync the tmp file before publishing it (rename is atomic on POSIX).
  {
    const int fd = ::open(tmp_path.c_str(), O_WRONLY);
    if (fd >= 0) {
      if (::fsync(fd) != 0) {
        err = "fsync failed: " + tmp_path;
        ::close(fd);
        ::unlink(tmp_path.c_str());
        return false;
      }
      ::close(fd);
    }
  }

  // Keep the previous good file as last-known-good, then publish atomically.
  struct stat st{};
  if (::stat(final_path.c_str(), &st) == 0) {
    if (::rename(final_path.c_str(), prev_path.c_str()) != 0) {
      err = "cannot keep last-known-good copy of " + final_path;
      ::unlink(tmp_path.c_str());
      return false;
    }
  }
  if (::rename(tmp_path.c_str(), final_path.c_str()) != 0) {
    err = "cannot rename " + tmp_path + " into place";
    ::unlink(tmp_path.c_str());
    return false;
  }
  return true;
}

std::vector<std::string> PayloadProfileStore::list() const {
  std::vector<std::string> names;
  DIR* d = ::opendir(dir_.c_str());
  if (d == nullptr) return names;
  while (dirent* e = ::readdir(d)) {
    const std::string f = e->d_name;
    if (f.size() > 5 && f.substr(f.size() - 5) == ".yaml") {
      names.push_back(f.substr(0, f.size() - 5));
    }
  }
  ::closedir(d);
  std::sort(names.begin(), names.end());
  return names;
}

}  // namespace payload
}  // namespace ota
