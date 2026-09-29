#pragma once
// What can actually be changed while controld runs, stated by the code that changes it.
//
// ADR-002.1 D2 forbids two things at once: a hot-tunable parameter may not need a rebuild, and a
// parameter the firmware cannot really change may not be written into a YAML and pretended to be
// applied. Both rules need one document listing every real control knob with its unit, its bounds,
// where the value actually lives and how it is read back. A hand-typed table drifts: it is a copy
// of the code, and the code moved. So this table is compiled against the fields it names — the
// binding is a member access, not a string — and `controld --dump-parameter-registry` emits it as
// JSON next to the binary hash that enforces it.
//
// Four mutability classes, and the difference is not a label:
//   experiment_writable   the campaign may request a new value in this boot, through the
//                         prepare/apply/readback transaction; bounds below are the ones the
//                         enforcing validator actually applies.
//   fixed_in_campaign     writable in principle, deliberately not a search dimension this round.
//                         Still hot-appliable, so a later campaign can open it without a rebuild.
//   protected_read_only   the optimizer must not widen or bypass it: current caps, thermal and
//                         feedback limits, lease durations, the control period itself.
//   unsupported           the drive or this backend cannot write/read it. `reason` is mandatory:
//                         a silent omission is how a no-op write gets reported as success.
//
// `readback_source` is separate from mutability because a value can be writable and still not
// verifiable: the yaw current-loop gains live in host memory, so the only readback is the host's
// own echo, while the CyberGear speed gains come back from a register. Reporting an echo as
// "verified" is the exact lie behind the kp2 folder that contained Kp=1.
#include <string>
#include <vector>

#include "../config/mixed_hardware_profile.hpp"
#include "../config/turret_config.hpp"

namespace ota::control {

enum class Mutability { ExperimentWritable, FixedInCampaign, ProtectedReadOnly, Unsupported };
enum class Readback { DriveRegister, HostEcho, ConfigFile, None };

const char* mutability_name(Mutability m);
const char* readback_source_name(Readback r);

struct ParameterEntry {
  std::string name;                // "yaw.current_kp_a_per_rad_s"
  std::string group;               // ADR-002.1 docs/02's groups, used to sort and to filter
  std::string type;                // "double" | "int" | "bool" | "string" | "enum"
  std::string unit;                // physical unit, or "dimensionless"; never blank for a physical quantity
  std::string default_literal;     // JSON literal: the number, or a quoted string
  std::string actual_literal;      // JSON literal: what this boot is running
  std::string supported_range;     // the bound the enforcing validator applies, quoted in reason if subtle
  std::string allowed_test_range;  // empty when this round does not search it (say why in reason)
  std::string mode;                // the mode/phase a change is even allowed in
  Mutability mutability = Mutability::ProtectedReadOnly;
  std::string apply_condition;     // what must be true at the moment of exchange
  std::string source_binding;      // the C++ member this row reads, checked by the compiler
  Readback readback = Readback::None;
  std::string encoding;            // how the value reaches hardware ("host variable", "register 0x7014", ...)
  bool restart_required = false;
  std::string reason;              // mandatory for fixed_in_campaign / protected_read_only / unsupported
};

struct ExclusionRule {
  std::string config_struct;       // a config struct whose scalar fields are not control knobs
  std::string reason;
};

// The table, in ADR-002.1's group order. `profile` may be null: a --sim boot has no mixed hardware
// profile, and then the current-mode yaw rows report their binding plus the reason they have no
// value, instead of inventing one.
std::vector<ParameterEntry> build_parameter_registry(const config::TurretConfig& cfg,
                                                     const config::mixed::Profile* profile);
std::vector<ExclusionRule> build_exclusion_rules();

// The document. `config_path` and `profile_path` are recorded so the file can be re-derived; the
// generator adds binary_sha256/source_rev, which a running binary cannot know about itself.
std::string parameter_registry_json(const config::TurretConfig& cfg,
                                    const config::mixed::Profile* profile,
                                    const std::string& config_path,
                                    const std::string& profile_path);

}  // namespace ota::control
