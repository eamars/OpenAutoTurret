#include "readback.hpp"
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace ota::commission {
can::RawFrame Readback::begin(cybergear::Reg reg, int64_t now, int64_t timeout) {
  if (failed_ || pending_ || now<=0 || timeout<=0 || !cybergear::reg_info(reg))
    throw std::runtime_error("DATA_INVALID: overlapping/invalid register transaction");
  const auto source=cybergear::make_read_reg(reg,host_,motor_);
  can::RawFrame result; result.extended=true; result.id=source.id;
  std::memcpy(result.data,source.data,8);
  reg_=reg; begin_=now; deadline_=now+timeout; accept_=0;
  pending_=true; accepted_=false; ++sequence_;
  return result;
}
void Readback::accepted(int64_t now, bool success) {
  if (!pending_ || accepted_ || !success || now<begin_ || now>=deadline_) {
    failed_=true; throw std::runtime_error("DATA_INVALID: register request TX failed");
  }
  accept_=now; accepted_=true;
}
void Readback::check_deadline(int64_t now) {
  if (failed_ || (pending_ && now>=deadline_)) {
    failed_=true; throw std::runtime_error("MEASUREMENT_LIMITED: register read timeout; no retry");
  }
}
ReadObservation Readback::observe(const Receipt& r) {
  const auto id=cybergear::unpack_ext_id(r.frame.id);
  cybergear::CanFrame wire; wire.id=r.frame.id; wire.dlc=r.frame.dlc;
  std::memcpy(wire.data,r.frame.data,8);
  cybergear::Reg reg; double value;
  // A response can reach the socket before send() returns. Bound it by the
  // pre-send timestamp, not a false assertion that it followed that return.
  if (failed_ || !pending_ || !accepted_ || !r.frame.extended || r.frame.error || r.frame.rtr ||
      id.comm_type!=17 || id.target!=host_ || id.data2!=motor_ ||
      r.kernel_monotonic_ns<begin_ || r.kernel_monotonic_ns>=deadline_ ||
      !cybergear::parse_reg_response(wire,reg,value) || reg!=reg_ || !std::isfinite(value)) {
    failed_=true; throw std::runtime_error("DATA_INVALID: uncorrelated, stale or invalid register response");
  }
  pending_=false;
  return {reg,value,sequence_,begin_,accept_,r.kernel_monotonic_ns};
}
std::string readback_json(const ReadObservation& o) {
  std::ostringstream s; s.precision(17);
  s << "{\"kind\":\"register_read\",\"axis\":\"pitch\",\"index\":" << unsigned(o.reg)
    << ",\"value\":" << o.value << ",\"request_sequence\":" << o.request_sequence
    << ",\"request_begin_ns\":" << o.request_begin_ns << ",\"request_accepted_ns\":" << o.request_accepted_ns
    << ",\"receive_ns\":" << o.receive_ns << ",\"device_sample_ns\":null,\"source\":\"type17_readback\"}";
  return s.str();
}
} // namespace ota::commission
