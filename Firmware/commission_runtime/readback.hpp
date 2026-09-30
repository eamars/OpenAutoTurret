#pragma once
#include "capture.hpp"
#include "can/cybergear_protocol.hpp"
#include <optional>

namespace ota::commission {
struct ReadObservation {
  cybergear::Reg reg;
  double value;
  uint64_t request_sequence;
  int64_t request_begin_ns, request_accepted_ns, receive_ns;
};

// One outstanding read, no retry after timeout. Source, host, register, type and
// host receive interval all correlate the reply. Protocol 17 has no transaction
// ID: these checks do not pretend to recover a device sampling timestamp.
class Readback {
 public:
  Readback(uint8_t motor, uint8_t host) : motor_(motor), host_(host) {}
  can::RawFrame begin(cybergear::Reg reg, int64_t now, int64_t timeout_ns);
  void accepted(int64_t now, bool success);
  ReadObservation observe(const Receipt&);
  void check_deadline(int64_t now);
  bool pending() const { return pending_; }
  uint64_t sequence() const { return sequence_; }
 private:
  uint8_t motor_, host_;
  bool pending_{false}, accepted_{false}, failed_{false};
  cybergear::Reg reg_{};
  uint64_t sequence_{};
  int64_t begin_{}, accept_{}, deadline_{};
};
std::string readback_json(const ReadObservation&);
} // namespace ota::commission
