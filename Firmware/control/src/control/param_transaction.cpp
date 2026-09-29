#include "param_transaction.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>

namespace ota::control {

std::string canonical_number(double value) {
  if (!std::isfinite(value)) return "nan";
  char buf[40];
  std::snprintf(buf, sizeof buf, "%.9g", value);
  return buf;
}

std::string canonical_bool(bool value) { return value ? "true" : "false"; }

std::vector<ParamValue> canonical_order(std::vector<ParamValue> values) {
  std::sort(values.begin(), values.end(),
            [](const ParamValue& a, const ParamValue& b) { return a.name < b.name; });
  return values;
}

std::string effective_hash(const std::vector<ParamValue>& values) {
  uint64_t hash = 1469598103934665603ULL;  // FNV-1a offset basis
  for (const ParamValue& value : canonical_order(values)) {
    const std::string line = value.name + "=" + value.canonical + "\n";
    for (const char c : line) {
      hash ^= static_cast<uint64_t>(static_cast<unsigned char>(c));
      hash *= 1099511628211ULL;
    }
  }
  char buf[24];
  std::snprintf(buf, sizeof buf, "%016llx", static_cast<unsigned long long>(hash));
  return buf;
}

std::string ParameterTransaction::prepare(std::vector<ParamValue> requested,
                                          const std::string& request_id) {
  if (requested.empty()) {
    last_reason_ = "prepare received no parameters; an empty set would silently mean 'run under "
                   "whatever is loaded', which is not a candidate";
    return last_reason_;
  }
  if (state_ == State::AppliedUnverified || state_ == State::Restoring) {
    last_reason_ = std::string("an exchange is already ") + state_name() +
                   "; no new candidate may be staged until the readback verifies (a second prepare "
                   "here is how one trial's motion runs under another trial's gains)";
    return last_reason_;
  }
  for (const ParamValue& value : requested) {
    if (value.name.empty()) {
      last_reason_ = "prepare received a value with no name";
      return last_reason_;
    }
    if (value.canonical.empty()) {
      last_reason_ = value.name + " was prepared with no value";
      return last_reason_;
    }
  }
  staged_ = canonical_order(std::move(requested));
  for (size_t i = 1; i < staged_.size(); ++i) {
    if (staged_[i].name == staged_[i - 1].name) {
      last_reason_ = staged_[i].name + " appears twice in one prepare; one candidate states each "
                                      "parameter once, or it is two candidates";
      staged_.clear();
      return last_reason_;
    }
  }
  // A repeat of the same request is idempotent, and says so by not changing anything: a runner that
  // retries after a lost ack must not cause a second integral-state transition downstream.
  if (state_ == State::Prepared && expected_hash_ == effective_hash(staged_)) {
    state_ = State::Prepared;
    last_reason_.clear();
    return "";
  }
  expected_hash_ = effective_hash(staged_);
  request_id_ = request_id;
  state_ = State::Prepared;
  last_reason_.clear();
  return "";
}

void ParameterTransaction::begin_apply(std::vector<ParamValue> previous_readback,
                                       const std::string& request_id) {
  if (state_ == State::AppliedUnverified && request_id == request_id_) return;  // retry of the same
  if (state_ != State::Prepared) {
    state_ = State::Failed;
    last_reason_ = std::string("apply arrived while ") + state_name() +
                   " with no prepared set behind it; refusing to write half a candidate. The "
                   "request_id must follow a prepare that was accepted";
    return;
  }
  previous_ = canonical_order(std::move(previous_readback));
  pending_ = staged_;
  staged_.clear();
  if (!request_id.empty()) request_id_ = request_id;
  state_ = State::AppliedUnverified;
  last_reason_.clear();
}

std::string ParameterTransaction::verify(const std::vector<ParamValue>& readback) {
  if (state_ != State::AppliedUnverified && state_ != State::Restoring) {
    last_reason_ = "readback reported while " + std::string(state_name()) +
                   "; nothing is waiting to be verified";
    return last_reason_;
  }
  const bool restoring = state_ == State::Restoring;
  const std::vector<ParamValue>& wanted = restoring ? previous_ : pending_;
  std::map<std::string, std::string> got;
  for (const ParamValue& value : readback) got[value.name] = value.canonical;
  for (const ParamValue& want : wanted) {
    const auto found = got.find(want.name);
    if (found == got.end()) {
      last_reason_ = "readback did not include " + want.name +
                     "; an absent value is not a confirmed one";
      return last_reason_;
    }
    if (found->second != want.canonical) {
      last_reason_ = want.name + " came back " + found->second + ", expected " + want.canonical +
                     (restoring ? " (the restore itself did not verify)" : "");
      state_ = State::Restoring;      // demanded again, motion stays blocked
      return last_reason_;
    }
  }
  applied_hash_ = effective_hash(wanted);
  ++revision_;
  state_ = State::Idle;
  pending_.clear();
  previous_.clear();
  last_reason_.clear();
  return "";
}

bool ParameterTransaction::blocks_motion() const { return state_ != State::Idle; }

const char* ParameterTransaction::state_name() const {
  switch (state_) {
    case State::Idle: return "idle";
    case State::Prepared: return "prepared";
    case State::AppliedUnverified: return "applied_unverified";
    case State::Restoring: return "restoring";
    case State::Failed: return "failed";
  }
  return "failed";
}

}  // namespace ota::control
