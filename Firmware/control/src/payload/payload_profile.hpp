// OpenAutoTurret — payload profile storage (architecture §28.5, §41, §31.5).
//
// A payload profile is what commissioning produces for one meaningful payload
// configuration: a controller gain profile, the safe velocity/acceleration/
// jerk limits, and the measured response baseline (§28.5). Yaw and pitch have
// SEPARATE profiles (§31.5) — they live as per-axis sections of one file.
//
// Integrity rules (§41):
//   * explicit schema version (reject unknown major versions);
//   * creation timestamp + config revision + hardware ids in the file;
//   * atomic write: tmp file + fsync + rename;
//   * the previous good file is kept as `<name>.yaml.prev` (last-known-good).
//
// Profiles live under `config/payload_profiles/<name>.yaml` (doc file layout).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "payload/response_metrics.hpp"

namespace ota {
namespace payload {

constexpr int kPayloadProfileSchemaVersion = 1;

// Per-axis section (yaw and pitch separate, §31.5).
struct AxisPayloadProfile {
  // Safe motion limits this payload was verified against (§28.5).
  double v_max_rad_s = 0.0;
  double a_max_rad_s2 = 0.0;
  double j_max_rad_s3 = 0.0;
  // CyberGear internal-loop gain profile (§58.18). Informational: the daemon
  // NEVER rewrites motor gains online (§59.19) — it only derates its own
  // motion limits.
  double gain_kp = 0.0;
  double gain_ki = 0.0;
  double gain_kd = 0.0;
  // Measured response baseline (§28.5 / §44).
  AxisProfileMetrics baseline;

  bool valid() const {
    return baseline.valid && v_max_rad_s > 0.0;
  }
};

// One payload profile (one meaningful payload configuration, §28.5).
struct PayloadProfile {
  int schema_version = kPayloadProfileSchemaVersion;
  std::string name = "conservative";
  int64_t created_ns = 0;         // §41: creation timestamp
  std::string config_revision;    // §41: config revision at profiling time
  std::string hardware;           // §41: hardware ids (motor unique ids)
  std::string notes;              // free-form operator notes
  AxisPayloadProfile pitch;
  AxisPayloadProfile yaw;

  const AxisPayloadProfile& axis(AxisId a) const {
    return (a == AxisId::Pitch) ? pitch : yaw;
  }
  AxisPayloadProfile& axis(AxisId a) {
    return (a == AxisId::Pitch) ? pitch : yaw;
  }
};

// Load / save payload profiles from a directory.
class PayloadProfileStore {
 public:
  explicit PayloadProfileStore(std::string dir) : dir_(std::move(dir)) {}
  const std::string& dir() const { return dir_; }

  // Load `<dir>/<name>.yaml`. ok=false (with err set) when the file is
  // missing, unparseable, or fails schema validation.
  bool load(const std::string& name, PayloadProfile& out,
            std::string& err) const;

  // Atomic save (§41): write `<dir>/<name>.yaml.tmp`, fsync, rename over
  // `<dir>/<name>.yaml`; if a previous file existed it is first renamed to
  // `<name>.yaml.prev` (last-known-good). The profile name is taken from the
  // profile itself (it must be a safe file name).
  bool save(const PayloadProfile& p, std::string& err) const;

  // All profile names currently in the directory (without extension).
  std::vector<std::string> list() const;

 private:
  std::string dir_;
};

// Serialize / parse a profile to / from YAML text.
std::string payload_profile_to_yaml(const PayloadProfile& p);
bool payload_profile_from_yaml(const std::string& text, PayloadProfile& out,
                               std::string& err);

}  // namespace payload
}  // namespace ota
