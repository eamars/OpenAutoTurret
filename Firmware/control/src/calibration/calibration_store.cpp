#include "calibration/calibration_store.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <system_error>

#include <yaml-cpp/yaml.h>

#include "calibration/camera_calibration.hpp"
#include "position_loop.hpp"
#include "servo.hpp"
#include "servo_config.hpp"

namespace ota::calibration {

namespace fs = std::filesystem;

namespace {

constexpr const char* kParts[] = {"yaw", "pitch", "accuracy", "extrinsic", "bore"};
constexpr double kRadToDeg = 180.0 / M_PI;
// bore_alignment needs a finite depth; at this distance a 75 mm offset is 75 nrad, i.e. none.
constexpr double kInfiniteDepthM = 1e6;

std::string id_dir(int id) {
  char buf[16];
  std::snprintf(buf, sizeof buf, "%04d", id);
  return buf;
}

int dir_id(const fs::path& p) {
  const std::string n = p.filename().string();
  if (n.empty() || !std::all_of(n.begin(), n.end(), [](char c) { return c >= '0' && c <= '9'; })) return 0;
  try { return std::stoi(n); } catch (...) { return 0; }
}

std::vector<int> ids_in(const fs::path& dir) {
  std::vector<int> ids;
  std::error_code ec;
  if (!fs::is_directory(dir, ec)) return ids;
  for (const auto& e : fs::directory_iterator(dir, ec))
    if (e.is_directory())
      if (const int id = dir_id(e.path()); id > 0) ids.push_back(id);
  std::sort(ids.begin(), ids.end());
  return ids;
}

std::string str(const YAML::Node& n) { return n && n.IsScalar() ? n.as<std::string>() : std::string(); }

BoreCamera read_bore_camera(const YAML::Node& n, const std::string& name) {
  if (!n || !n.IsMap()) throw std::runtime_error("bore.yaml has no " + name + " camera");
  BoreCamera c;
  c.base = {n["right_mm"].as<double>(), n["up_mm"].as<double>(), n["forward_mm"].as<double>(),
            n["right_deg"].as<double>(), n["up_deg"].as<double>()};
  c.trim_right_mrad = n["trim_right_mrad"] ? n["trim_right_mrad"].as<double>() : 0.0;
  c.trim_up_mrad = n["trim_up_mrad"] ? n["trim_up_mrad"].as<double>() : 0.0;
  return c;
}

// The part's file parses into what controld would make of it; the reason when it does not.
std::string check_part(const std::string& part, const fs::path& file, Bore& bore) {
  std::error_code ec;
  if (!fs::is_regular_file(file, ec)) return file.filename().string() + " is missing";
  try {
    if (part == "yaw") {
      axis::Servo servo;
      if (!servo.configure(axis::servo_from_yaml(YAML::LoadFile(file.string())["servo_parameters"])))
        return "yaw_servo.json does not make a servo";
    } else if (part == "pitch") {
      const auto trial = YAML::LoadFile(file.string())["servo_trial"];
      axis::PositionLoop loop;
      // The speed limit is the profile's (100 RPM), not the asset's; any positive value checks the rest.
      if (!loop.configure({trial["kp_per_s"].as<double>(), trial["ki_per_s2"].as<double>(),
                           trial["integral_clamp_rad_s"].as<double>(), 1.0}) ||
          !(trial["following_error_rad"].as<double>() > 0))
        return "pitch_servo.json does not make a position loop";
    } else if (part == "accuracy") {
      if (!YAML::LoadFile(file.string())["limits"]) return "yaw_accuracy.json has no limits";
    } else if (part == "extrinsic") {
      std::string detail;
      load_camera_extrinsics(file.string(), detail);
      if (detail != "R_P_C loaded from file") return "camera_extrinsics.yaml: " + detail;
    } else if (part == "bore") {
      const auto doc = YAML::LoadFile(file.string());
      const std::string mode = str(doc["mode"]);
      if (mode != "off" && mode != "manual_depth") return "bore.yaml mode must be off or manual_depth";
      bore.enabled = mode == "manual_depth";
      bore.zero_m = doc["zero_m"].as<double>();
      if (!(bore.zero_m > 0)) return "bore.yaml zero_m must be a positive distance or .inf";
      bore.wide = read_bore_camera(doc["wide"], "wide");
      bore.detail = read_bore_camera(doc["detail"], "detail");
      // The same validity rule the tracker applies, checked here so a bad bore refuses the record
      // rather than the tracker (which would refuse to track).
      const geo::CameraIntrinsics probe;   // any valid camera: the offsets are what is checked
      auto cfg = bore.alignment();
      cfg.enabled = true;
      for (auto role : {geo::CameraRole::Wide, geo::CameraRole::Detail})
        if (!geo::bore_alignment(cfg, role, probe).valid) return "bore.yaml does not make a valid bore";
    }
  } catch (const std::exception& e) {
    return file.filename().string() + " does not parse (" + e.what() + ")";
  }
  return {};
}

}  // namespace

geo::CameraOffset BoreCamera::effective() const {
  geo::CameraOffset o = base;
  o.right_deg -= trim_right_mrad * 1e-3 * kRadToDeg;
  o.up_deg -= trim_up_mrad * 1e-3 * kRadToDeg;
  return o;
}

geo::BoreAlignmentConfig Bore::alignment() const {
  geo::BoreAlignmentConfig c;
  c.enabled = enabled;
  c.assumed_depth_m = std::isfinite(zero_m) ? zero_m : kInfiniteDepthM;
  c.wide = wide.effective();
  c.detail = detail.effective();
  return c;
}

fs::path Record::path(const std::string& part) const {
  const auto it = parts.find(part);
  return dir / (it == parts.end() ? std::string() : it->second.file);
}

Record read_record(const fs::path& dir, int id) {
  Record r;
  r.id = id;
  r.dir = dir;
  try {
    const auto doc = YAML::LoadFile((dir / "record.json").string());
    r.name = str(doc["name"]);
    for (const char* part : kParts) {
      const auto p = doc["parts"][part];
      if (!p || !p.IsMap()) { r.unusable = std::string("record.json has no ") + part + " part"; return r; }
      PartInfo info{str(p["file"]), str(p["origin"]), str(p["date"]), str(p["run"]), str(p["rev"]),
                    p["from"] ? p["from"].as<int>() : 0};
      if (info.file.empty() || info.file.find('/') != std::string::npos) {
        r.unusable = std::string("record.json names no file for ") + part;
        return r;
      }
      r.parts[part] = info;
    }
  } catch (const std::exception& e) {
    r.unusable = std::string("record.json does not parse (") + e.what() + ")";
    return r;
  }
  if (r.name.empty()) r.name = "#" + std::to_string(id);
  for (const char* part : kParts) {
    r.unusable = check_part(part, r.path(part), r.bore);
    if (!r.unusable.empty()) return r;
  }
  return r;
}

void Store::adopt_incoming() {
  const fs::path incoming = root_ / "incoming", records = root_ / "records";
  std::error_code ec;
  if (!fs::is_directory(incoming, ec)) return;
  std::vector<fs::path> staged;
  for (const auto& e : fs::directory_iterator(incoming, ec))
    if (e.is_directory() && e.path().filename().string().rfind('.', 0) != 0) staged.push_back(e.path());
  std::sort(staged.begin(), staged.end());   // their names begin with the time they were staged
  for (const auto& from : staged) {
    // Ids are never reused: a removed record keeps its number in deleted/.
    const auto live = ids_in(records), gone = ids_in(root_ / "deleted");
    const int next = 1 + std::max(live.empty() ? 0 : live.back(), gone.empty() ? 0 : gone.back());
    fs::create_directories(records, ec);
    fs::rename(from, records / id_dir(next), ec);
    if (ec) adopted_.push_back("could not adopt " + from.filename().string() + ": " + ec.message());
    else adopted_.push_back("adopted " + from.filename().string() + " as #" + std::to_string(next));
  }
}

bool Store::open(const fs::path& state_dir, std::string& err) {
  root_ = state_dir / "calibration";
  records_.clear();
  adopted_.clear();
  active_ = 0;
  std::error_code ec;
  if (!fs::exists(root_, ec)) return true;
  if (!fs::is_directory(root_, ec)) {
    err = root_.string() + " is not a directory";
    return false;
  }
  adopt_incoming();
  for (const int id : ids_in(root_ / "records")) records_.push_back(read_record(root_ / "records" / id_dir(id), id));
  try {
    const auto doc = YAML::LoadFile((root_ / "active.json").string());
    active_ = doc["active"].as<int>();
  } catch (const std::exception&) {
    active_ = 0;   // reported by in_force()
  }
  return true;
}

const Record* Store::find(int id) const {
  for (const auto& r : records_)
    if (r.id == id) return &r;
  return nullptr;
}

const Record* Store::in_force(std::string& why) const {
  if (records_.empty()) { why = "no calibration records"; return nullptr; }
  if (active_ <= 0) { why = "no record is chosen (active.json is missing or unreadable)"; return nullptr; }
  const Record* r = find(active_);
  if (r == nullptr) { why = "record #" + std::to_string(active_) + " named in active.json does not exist"; return nullptr; }
  if (!r->unusable.empty()) { why = "record #" + std::to_string(r->id) + ": " + r->unusable; return nullptr; }
  return r;
}

}  // namespace ota::calibration
