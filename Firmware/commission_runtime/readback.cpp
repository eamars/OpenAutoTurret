#include "readback.hpp"
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace ota::commission {
can::RawFrame Readback::begin(cybergear::Reg reg, int64_t now, int64_t timeout) {
  if (failed_ || pending_ || now<=0 || timeout<=0 || !cybergear::reg_info(reg) || rejected_registers_.count(reg))
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
  const auto requested_index=uint16_t(r.frame.data[0]) | (uint16_t(r.frame.data[1])<<8);
  // A response can reach the socket before send() returns. Bound it by the
  // pre-send timestamp, not a false assertion that it followed that return.
  if (failed_ || !pending_ || !accepted_ || !r.frame.extended || r.frame.error || r.frame.rtr ||
      r.frame.dlc!=8 || id.comm_type!=17 || id.target!=host_ || (id.data2&255)!=motor_ ||
      r.kernel_monotonic_ns<begin_ || r.kernel_monotonic_ns>=deadline_ ||
      requested_index!=static_cast<uint16_t>(reg_) ||
      ((id.data2&0xff00)!=0 && (id.data2&0xff00)!=0x0100)) {
    failed_=true; throw std::runtime_error("DATA_INVALID: uncorrelated, stale or invalid register response");
  }
  if ((id.data2&0xff00)==0x0100 && !wire.data[2] && !wire.data[3]) {
    // The captured factory response 0x11017f00 rejects a read and leaves stale
    // bytes in the value field. It is capability evidence, never a float value.
    pending_=false; rejected_registers_.insert(reg_);
    return {reg_,std::nullopt,sequence_,begin_,accept_,r.kernel_monotonic_ns,1};
  }
  if (!cybergear::parse_reg_response(wire,reg,value) || reg!=reg_ || !std::isfinite(value)) {
    failed_=true; throw std::runtime_error("DATA_INVALID: uncorrelated, stale or invalid register response");
  }
  pending_=false;
  return {reg,value,sequence_,begin_,accept_,r.kernel_monotonic_ns};
}
std::string readback_json(const ReadObservation& o) {
  std::ostringstream s; s.precision(17);
  s << "{\"kind\":\"" << (o.value?"register_read":"register_rejected") << "\",\"axis\":\"pitch\",\"index\":" << unsigned(o.reg)
    << ",\"value\":";
  if (o.value) s << *o.value; else s << "null";
  s << ",\"request_sequence\":" << o.request_sequence
    << ",\"request_begin_ns\":" << o.request_begin_ns << ",\"request_accepted_ns\":" << o.request_accepted_ns
    << ",\"receive_ns\":" << o.receive_ns << ",\"device_sample_ns\":null,\"source\":\""
    << (o.value?"type17_readback":"type17_negative_reply") << "\"";
  if (!o.value) s << ",\"device_error_flag\":" << unsigned(o.device_error_flag);
  s << "}";
  return s.str();
}
} // namespace ota::commission
