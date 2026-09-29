#pragma once
// One parameter set, one revision, and no motion until the firmware has seen what it actually wrote.
//
// ADR-002.1 D3 is a sentence about ordering: prepare the whole set, apply it inside the existing
// control owner, read the values back, and only then allow RUN. The failure it exists to prevent is
// not exotic — the ADR-002 run report has a folder named `kp2-fine` that contains Kp=1, because the
// apply was refused and the next script started a jog anyway. Any runner can forget; the controller
// cannot, if the controller is the one holding the door.
//
// So this is a record-keeper, not a writer: the control loop owns the backend and does the actual
// writes, and reports them here. What this class guarantees is the state machine around those writes
// — what was requested, what was on the hardware before, what came back, and therefore whether a
// motion command may start. It has no threads and no hardware, which is why the kp2 failure mode is
// testable instead of being something a campaign discovers.
//
// Values travel as `ParamValue` with a `canonical` text: the value in the representation the firmware
// actually stores (`%.9g`, `true`/`false`, an enum token). The hash is computed over the readback,
// never over the request string, because the thing a trial must be able to prove it ran under is the
// effective value, not what somebody asked for.
#include <cstdint>
#include <string>
#include <vector>

namespace ota::control {

struct ParamValue {
  std::string name;       // the registry name, e.g. "yaw.current_kp_a_per_rad_s"
  std::string canonical;  // effective value as stored: "%.9g", "true"/"false", or an enum token
};

// Canonical text for a numeric setting. One formatter, so a prepare and a readback of the same value
// hash the same on the first try; a fuzzy float string is how a verification fails for appearance.
std::string canonical_number(double value);
std::string canonical_bool(bool value);

// FNV-1a over sorted "name=canonical" pairs. A checksum for one campaign's consistency — see
// docs/02 §7: it proves two documents agree, it does not authorise anything.
std::string effective_hash(const std::vector<ParamValue>& values);

class ParameterTransaction {
 public:
  enum class State { Idle, Prepared, AppliedUnverified, Restoring, Failed };

  static constexpr uint64_t kInitialRevision = 0;

  // Stage a complete candidate set. Rejects an empty set, a repeated name, a non-canonical value,
  // or a re-prepare while an apply is still unverified — each with the reason, because "prepare
  // failed" without a reason restarts the guessing this whole package was written to end.
  std::string prepare(std::vector<ParamValue> requested, const std::string& request_id);

  // A begin_apply call carries the values read back BEFORE writing, so a failed apply has something
  // honest to restore to. Staged values move to `pending_`; revision does not move yet.
  void begin_apply(std::vector<ParamValue> previous_readback, const std::string& request_id);

  // Compare what came back from the hardware/host after the write. On a match the revision advances
  // and motion is allowed again. On a mismatch the transaction demands a restore, and motion stays
  // blocked until the restore verifies: a half-written set is not a candidate, and it is not free.
  std::string verify(const std::vector<ParamValue>& readback);

  bool restore_required() const { return state_ == State::Restoring; }
  const std::vector<ParamValue>& restore_values() const { return previous_; }

  // Motion gate. True while any exchange is incomplete: prepared-but-unapplied (the set the runner
  // means to run under is not the set the hardware holds), applied-unverified (the kp2 case), or
  // mid-restore (nothing about this candidate is trustworthy right now).
  bool blocks_motion() const;

  State state() const { return state_; }
  const char* state_name() const;
  uint64_t revision() const { return revision_; }
  const std::string& expected_hash() const { return expected_hash_; }
  const std::string& applied_hash() const { return applied_hash_; }
  const std::string& request_id() const { return request_id_; }
  const std::string& last_reason() const { return last_reason_; }
  const std::vector<ParamValue>& staged() const { return staged_; }

 private:
  State state_ = State::Idle;
  uint64_t revision_ = kInitialRevision;
  std::vector<ParamValue> staged_;      // what prepare accepted, in canonical order
  std::vector<ParamValue> pending_;     // what is being applied, awaiting readback
  std::vector<ParamValue> previous_;    // what the hardware held before this apply (restore target)
  std::string expected_hash_;
  std::string applied_hash_;
  std::string request_id_;
  std::string last_reason_;
};

// Sort a set into the order the hash is defined over, so callers cannot produce two hashes for one
// candidate by asking in a different order.
std::vector<ParamValue> canonical_order(std::vector<ParamValue> values);

}  // namespace ota::control
