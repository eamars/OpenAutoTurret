#include "calibration/calibration_store.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <limits>
#include <iterator>
#include <fstream>
#include <system_error>

#include <unistd.h>

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
    unverified_ = doc["unverified"] && doc["unverified"].as<bool>();
  } catch (const std::exception&) {
    active_ = 0;   // reported by in_force()
  }
  return true;
}

bool prepare(const Record& record, Prepared& out, std::string& err) {
  if (!record.unusable.empty()) {
    err = "record #" + std::to_string(record.id) + ": " + record.unusable;
    return false;
  }
  Prepared p;
  p.record = record;
  if (!parse_servo_assets(record.path("yaw").string(), record.path("pitch").string(), p.servo, err)) {
    err = "record #" + std::to_string(record.id) + ": " + err;
    return false;
  }
  std::string detail;
  p.kinematics = load_camera_extrinsics(record.path("extrinsic").string(), detail);
  if (detail != "R_P_C loaded from file") {
    err = "record #" + std::to_string(record.id) + ": camera_extrinsics.yaml: " + detail;
    return false;
  }
  p.alignment = record.bore.alignment();
  out = std::move(p);
  return true;
}

namespace {
std::string esc(const std::string& s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (static_cast<unsigned char>(c) < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); o += b; }
    else o += c;
  }
  return o;
}
std::string num(double v) {
  if (!std::isfinite(v)) return "null";
  char b[32];
  std::snprintf(b, sizeof b, "%.6g", v);
  return b;
}
std::string camera_json(const BoreCamera& c) {
  return "{\"trim_right_mrad\":" + num(c.trim_right_mrad) + ",\"trim_up_mrad\":" + num(c.trim_up_mrad) +
         ",\"right_mm\":" + num(c.base.right_mm) + ",\"up_mm\":" + num(c.base.up_mm) + "}";
}
}  // namespace

std::string records_json(const std::vector<Record>& records) {
  std::string o = "[";
  for (const auto& r : records) {
    if (o.size() > 1) o += ',';
    o += "{\"id\":" + std::to_string(r.id) + ",\"name\":\"" + esc(r.name) + "\",\"unusable\":\"" +
         esc(r.unusable) + "\",\"parts\":{";
    bool first = true;
    for (const auto& [part, info] : r.parts) {
      if (!first) o += ',';
      first = false;
      o += "\"" + part + "\":{\"origin\":\"" + esc(info.origin) + "\",\"date\":\"" + esc(info.date) +
           "\",\"run\":\"" + esc(info.run) + "\",\"from\":" + std::to_string(info.from) + "}";
    }
    // zero_m null = infinity (no parallax).
    o += "},\"bore\":{\"mode\":\"" + std::string(r.bore.enabled ? "manual_depth" : "off") +
         "\",\"zero_m\":" + num(r.bore.zero_m) + ",\"wide\":" + camera_json(r.bore.wide) +
         ",\"detail\":" + camera_json(r.bore.detail) + "}}";
  }
  return o + "]";
}

namespace {
// Temp file, flushed to the disk, then renamed over: a reader never sees half a file, and a power
// cut never leaves an empty one (the WSL crash of 2026-10-24 showed what unflushed writes become).
bool write_file(const fs::path& final, const std::string& text, std::string& err) {
  const fs::path tmp = final.string() + ".tmp";
  std::FILE* f = std::fopen(tmp.string().c_str(), "wb");
  if (f == nullptr) { err = "cannot write " + tmp.string(); return false; }
  const bool ok = std::fwrite(text.data(), 1, text.size(), f) == text.size() && std::fflush(f) == 0 &&
                  ::fsync(fileno(f)) == 0;
  std::fclose(f);
  if (!ok) { err = "cannot write " + tmp.string(); return false; }
  std::error_code ec;
  fs::rename(tmp, final, ec);
  if (ec) { err = "cannot replace " + final.string() + ": " + ec.message(); return false; }
  return true;
}

bool json_number(const std::string& s) {
  std::size_t i = 0;
  if (i < s.size() && s[i] == '-') ++i;
  const std::size_t digits = i;
  while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) ++i;
  if (i == digits) return false;
  if (i < s.size() && s[i] == '.') {
    const std::size_t f = ++i;
    while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) ++i;
    if (i == f) return false;
  }
  if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
    ++i;
    if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
    const std::size_t e = i;
    while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) ++i;
    if (i == e) return false;
  }
  return i == s.size();
}

}  // namespace

// yaml-cpp reads JSON but writes YAML; record.json is read by the Python tool with json.loads, so
// it is written back as JSON. A quoted scalar stays a string ("!" tag); a plain one is written as it
// was when it is a JSON number, true, false or null.
std::string to_json(const YAML::Node& n, int indent) {
  const std::string pad(indent + 1, ' '), end(indent, ' ');
  if (n.IsMap()) {
    std::string o = "{";
    bool first = true;
    for (const auto& kv : n) {
      o += (first ? "\n" : ",\n") + pad + "\"" + esc(kv.first.as<std::string>()) + "\": " + to_json(kv.second, indent + 1);
      first = false;
    }
    return first ? "{}" : o + "\n" + end + "}";
  }
  if (n.IsSequence()) {
    std::string o = "[";
    for (std::size_t i = 0; i < n.size(); ++i) o += (i ? ", " : "") + to_json(n[i], indent + 1);
    return o + "]";
  }
  if (n.IsNull() || !n.IsDefined()) return "null";
  const std::string v = n.Scalar();
  if (n.Tag() != "!" && (json_number(v) || v == "true" || v == "false" || v == "null")) return v;
  return "\"" + esc(v) + "\"";
}

namespace {

std::string now_local() {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char b[40];
  std::strftime(b, sizeof b, "%Y-%m-%dT%H:%M:%S%z", &tm);
  std::string s = b;
  if (s.size() > 2) s.insert(s.size() - 2, ":");   // +1300 -> +13:00, as the Python tool writes it
  return s;
}
}  // namespace

bool write_active(const fs::path& dir, int id, bool unverified, std::string& err) {
  return write_file(dir / "active.json", "{\"schema\": 1, \"active\": " + std::to_string(id) +
                    ", \"unverified\": " + (unverified ? "true" : "false") + "}\n", err);
}

std::string today_local() {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char b[16];
  std::strftime(b, sizeof b, "%Y-%m-%d", &tm);
  return b;
}

std::string bore_yaml(const Bore& b) {
  auto cam = [](const char* name, const BoreCamera& c) {
    return std::string(name) + ": {right_mm: " + num(c.base.right_mm) + ", up_mm: " + num(c.base.up_mm) +
           ", forward_mm: " + num(c.base.forward_mm) + ", right_deg: " + num(c.base.right_deg) +
           ", up_deg: " + num(c.base.up_deg) + ", trim_right_mrad: " + num(c.trim_right_mrad) +
           ", trim_up_mrad: " + num(c.trim_up_mrad) + "}\n";
  };
  return "# ADR-005 D9: this record's bore. mode off = the tracker aims the camera centre.\n"
         "schema: 1\nmode: \"" + std::string(b.enabled ? "manual_depth" : "off") + "\"\n"
         "zero_m: " + (std::isfinite(b.zero_m) ? num(b.zero_m) : std::string(".inf")) +
         "   # zeroing distance; .inf = no parallax\n" + cam("wide", b.wide) + cam("detail", b.detail);
}

// record.json is rewritten by a rename (the web thread) and by a trim (the writer thread): each reads,
// changes one field and writes it back, so they take turns or one would undo the other.
static std::mutex record_json_mutex;

bool write_bore(const fs::path& dir, const Bore& b, const std::string& date, std::string& err) {
  if (!write_file(dir / "bore.yaml", bore_yaml(b), err)) return false;
  std::lock_guard lk(record_json_mutex);
  try {
    YAML::Node doc = YAML::LoadFile((dir / "record.json").string());
    YAML::Node part = doc["parts"]["bore"];
    part.remove("from");
    part.remove("rev");
    part["origin"] = "trimmed";
    part["origin"].SetTag("!");
    part["date"] = date;
    part["date"].SetTag("!");
    return write_file(dir / "record.json", to_json(doc, 0) + "\n", err);
  } catch (const std::exception& e) {
    err = std::string("record.json does not parse (") + e.what() + ")";
    return false;
  }
}

std::string record_name(const std::string& raw) {
  std::string name;
  for (char c : raw) name += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  while (!name.empty() && name.back() == ' ') name.pop_back();
  while (!name.empty() && name.front() == ' ') name.erase(name.begin());
  if (name.empty() || name.size() > 24) return {};
  for (char c : name)
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == ' ' || c == '-' || c == '_')) return {};
  return name;
}

bool rename_record(const fs::path& dir, const std::string& name, std::string& err) {
  std::lock_guard lk(record_json_mutex);
  try {
    YAML::Node doc = YAML::LoadFile((dir / "record.json").string());
    doc["name"] = name;
    doc["name"].SetTag("!");
    return write_file(dir / "record.json", to_json(doc, 0) + "\n", err);
  } catch (const std::exception& e) {
    err = std::string("record.json does not parse (") + e.what() + ")";
    return false;
  }
}

bool install_parts(const fs::path& dir, int id, const fs::path& from, const std::string& run, const std::string& date,
                   std::vector<std::string>& installed, std::string& err) {
  static const std::pair<const char*, const char*> kMeasured[] = {
      {"yaw", "yaw_servo.json"}, {"pitch", "pitch_servo.json"}, {"accuracy", "yaw_accuracy.json"}};
  std::error_code ec;
  installed.clear();
  for (const auto& [part, file] : kMeasured)
    if (fs::is_regular_file(from / file, ec)) installed.push_back(part);
  if (installed.empty()) { err = "nothing to install in " + from.string(); return false; }
  // Staged: the record as it would be, read and checked like any record before anything changes.
  const fs::path staging = dir.parent_path().parent_path() / (".install-" + dir.filename().string());
  fs::remove_all(staging, ec);
  fs::create_directories(staging, ec);
  for (const auto& e : fs::directory_iterator(dir, ec))
    if (e.is_regular_file()) fs::copy_file(e.path(), staging / e.path().filename(), fs::copy_options::overwrite_existing, ec);
  for (const auto& [part, file] : kMeasured)
    if (fs::is_regular_file(from / file)) fs::copy_file(from / file, staging / file, fs::copy_options::overwrite_existing, ec);
  if (ec) { err = "cannot stage the install: " + ec.message(); fs::remove_all(staging, ec); return false; }
  const Record staged = read_record(staging, id);
  fs::remove_all(staging, ec);
  if (!staged.unusable.empty()) { err = "the measured parts would make the record unusable: " + staged.unusable; return false; }
  // In place: the replaced files to history/, the new ones written atomically, record.json last.
  std::string stamp = run;
  for (char& c : stamp) if (c == '/' || c == ' ') c = '_';
  const fs::path history = dir / "history" / stamp;
  fs::create_directories(history, ec);
  if (ec) { err = "cannot create " + history.string() + ": " + ec.message(); return false; }
  std::lock_guard lk(record_json_mutex);
  try {
    YAML::Node doc = YAML::LoadFile((dir / "record.json").string());
    fs::copy_file(dir / "record.json", history / "record.json", fs::copy_options::overwrite_existing, ec);
    for (const auto& [part, file] : kMeasured) {
      if (!fs::is_regular_file(from / file)) continue;
      const std::string current = doc["parts"][part]["file"] ? doc["parts"][part]["file"].as<std::string>() : file;
      if (fs::is_regular_file(dir / current)) fs::copy_file(dir / current, history / current, fs::copy_options::overwrite_existing, ec);
      std::ifstream in(from / file);
      const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
      if (!write_file(dir / file, text, err)) return false;
      YAML::Node p;
      p["file"] = file;
      p["file"].SetTag("!");
      p["origin"] = "measured";
      p["origin"].SetTag("!");
      p["date"] = date;
      p["date"].SetTag("!");
      p["run"] = run;
      p["run"].SetTag("!");
      doc["parts"][part] = p;
    }
    return write_file(dir / "record.json", to_json(doc, 0) + "\n", err);
  } catch (const std::exception& e) {
    err = std::string("record.json does not parse (") + e.what() + ")";
    return false;
  }
}

bool write_last_run(const fs::path& dir, const std::string& kind, const std::string& verdict, const std::string& run,
                    const std::string& date, std::string& err) {
  std::lock_guard lk(record_json_mutex);
  try {
    YAML::Node doc = YAML::LoadFile((dir / "record.json").string());
    YAML::Node r;
    for (const auto& [k, v] : {std::pair{"kind", kind}, {"verdict", verdict}, {"run", run}, {"date", date}}) {
      r[k] = v;
      r[k].SetTag("!");
    }
    doc["last_run"] = r;
    return write_file(dir / "record.json", to_json(doc, 0) + "\n", err);
  } catch (const std::exception& e) {
    err = std::string("record.json does not parse (") + e.what() + ")";
    return false;
  }
}

bool copy_record(const Record& from, const fs::path& dir, const std::string& name, std::string& err) {
  // Made beside, then renamed into place: a half-made record is never in records/.
  const fs::path staging = dir.parent_path().parent_path() / (".new-" + dir.filename().string());
  std::error_code ec;
  fs::remove_all(staging, ec);
  fs::create_directories(staging, ec);
  if (ec) { err = "cannot create " + staging.string() + ": " + ec.message(); return false; }
  std::string parts;
  for (const char* part : kParts) {
    const auto& file = from.parts.at(part).file;
    fs::copy_file(from.dir / file, staging / file, fs::copy_options::overwrite_existing, ec);
    if (ec) { err = "cannot copy " + file + ": " + ec.message(); fs::remove_all(staging, ec); return false; }
    parts += std::string(parts.empty() ? "" : ",\n") + "  \"" + part + "\": {\"file\": \"" + esc(file) +
             "\", \"origin\": \"inherited\", \"from\": " + std::to_string(from.id) + "}";
  }
  const std::string record = "{\n \"schema\": \"ota.calibration-record/1\",\n \"name\": \"" + esc(name) +
      "\",\n \"created\": \"" + now_local() + "\",\n \"source\": \"copy of #" + std::to_string(from.id) +
      "\",\n \"extrinsic_frame\": \"pitch_from_homed_low\",\n \"parts\": {\n" + parts + "\n },\n \"last_run\": null\n}\n";
  if (!write_file(staging / "record.json", record, err)) { fs::remove_all(staging, ec); return false; }
  fs::create_directories(dir.parent_path(), ec);
  fs::rename(staging, dir, ec);
  if (ec) { err = "cannot place " + dir.string() + ": " + ec.message(); fs::remove_all(staging, ec); return false; }
  return true;
}

bool remove_record(const fs::path& root, int id, std::string& err) {
  std::error_code ec;
  fs::create_directories(root / "deleted", ec);
  fs::path to = root / "deleted" / id_dir(id);
  for (int n = 1; fs::exists(to, ec); ++n) to = root / "deleted" / (id_dir(id) + "." + std::to_string(n));
  fs::rename(root / "records" / id_dir(id), to, ec);
  if (ec) { err = "cannot remove #" + std::to_string(id) + ": " + ec.message(); return false; }
  return true;
}

fs::path Store::dir_of(int id) const { return root_ / "records" / id_dir(id); }

void Store::set_bore(int id, const Bore& b, const std::string& date) {
  for (auto& r : records_)
    if (r.id == id) {
      r.bore = b;
      r.parts["bore"] = PartInfo{"bore.yaml", "trimmed", date, "", "", 0};
    }
}

int Store::next_id() const {
  const auto live = ids_in(root_ / "records"), gone = ids_in(root_ / "deleted");
  return 1 + std::max(live.empty() ? 0 : live.back(), gone.empty() ? 0 : gone.back());
}

void Store::refresh(int id) {
  Record r = read_record(dir_of(id), id);
  for (auto& existing : records_)
    if (existing.id == id) { existing = std::move(r); return; }
  records_.push_back(std::move(r));
  std::sort(records_.begin(), records_.end(), [](const Record& a, const Record& b) { return a.id < b.id; });
}

void Store::drop(int id) {
  records_.erase(std::remove_if(records_.begin(), records_.end(), [id](const Record& r) { return r.id == id; }),
                 records_.end());
}

Writer::Writer() : thread_([this] {
  std::unique_lock lk(m_);
  for (;;) {
    cv_.wait(lk, [this] { return stop_ || !jobs_.empty(); });
    if (jobs_.empty()) return;   // stopping, and nothing left to write
    auto job = std::move(jobs_.front());
    jobs_.pop_front();
    lk.unlock();
    job();
    lk.lock();
  }
}) {}

Writer::~Writer() {
  { std::lock_guard lk(m_); stop_ = true; }
  cv_.notify_all();
  thread_.join();
}

void Writer::post(std::function<void()> job) {
  { std::lock_guard lk(m_); jobs_.push_back(std::move(job)); }
  cv_.notify_one();
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
