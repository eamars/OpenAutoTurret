#include "capture.hpp"
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <linux/can.h>
#include <linux/sock_diag.h>
#include <sstream>
#include <stdexcept>
#include <time.h>
#include <unistd.h>
#include "can/gm6020_protocol.hpp"
#include "can/cybergear_protocol.hpp"

namespace ota::commission {
namespace {
int64_t read_clock(clockid_t clock) {
  timespec t{};
  if (clock_gettime(clock, &t)) throw std::runtime_error("clock_gettime failed");
  return int64_t(t.tv_sec)*1000000000 + t.tv_nsec;
}
void option(int fd, int key) {
  int one = 1;
  if (setsockopt(fd, SOL_SOCKET, key, &one, sizeof(one)))
    throw std::runtime_error("required receive metadata unavailable: " + std::string(strerror(errno)));
}
}
int64_t monotonic_ns() { return read_clock(CLOCK_MONOTONIC); }
ClockBracket ClockBracket::sample() {
  const auto before = monotonic_ns();
  const auto realtime = read_clock(CLOCK_REALTIME);
  const auto after = monotonic_ns();
  return {realtime - (before + (after-before)/2), (after-before+1)/2};
}
TimestampedReceiver::TimestampedReceiver(int fd, int64_t max_clock_error_ns)
    : fd_(fd), max_clock_error_ns_(max_clock_error_ns), origin_(ClockBracket::sample()) {
  if (max_clock_error_ns <= 0 || origin_.uncertainty_ns > max_clock_error_ns)
    throw std::runtime_error("DATA_INVALID: clock mapping uncertainty");
  option(fd, SO_TIMESTAMPNS);
  option(fd, SO_RXQ_OVFL);
  if (kernel_drops()) throw std::runtime_error("DATA_INVALID: socket already lost packets before capture");
}
uint32_t TimestampedReceiver::kernel_drops() const {
  std::array<uint32_t,SK_MEMINFO_VARS> counters{};
  socklen_t size=sizeof(counters);
  if (getsockopt(fd_,SOL_SOCKET,SO_MEMINFO,counters.data(),&size) ||
      size<(SK_MEMINFO_DROPS+1)*sizeof(uint32_t))
    throw std::runtime_error("DATA_INVALID: final socket loss counter unavailable");
  return counters[SK_MEMINFO_DROPS];
}
bool TimestampedReceiver::receive(Receipt& out) {
  can_frame wire{};
  iovec io{&wire, sizeof(wire)};
  alignas(cmsghdr) std::array<char, CMSG_SPACE(sizeof(timespec)) + CMSG_SPACE(sizeof(uint32_t))> control{};
  msghdr msg{};
  msg.msg_iov = &io; msg.msg_iovlen = 1;
  msg.msg_control = control.data(); msg.msg_controllen = control.size();
  ssize_t n;
  do { n = recvmsg(fd_, &msg, MSG_DONTWAIT); } while (n < 0 && errno == EINTR);
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return false;
  if (n != CAN_MTU || (msg.msg_flags & (MSG_TRUNC|MSG_CTRUNC)))
    throw std::runtime_error("DATA_INVALID: truncated or invalid CAN datagram");
  out = {};
  out.dequeue_ns = monotonic_ns();
  bool timestamp = false;
  out.socket_drops = drops_;
  for (auto* c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
    if (c->cmsg_level != SOL_SOCKET) continue;
    if (c->cmsg_type == SCM_TIMESTAMPNS && c->cmsg_len == CMSG_LEN(sizeof(timespec))) {
      timespec stamp{}; std::memcpy(&stamp, CMSG_DATA(c), sizeof(stamp));
      if (stamp.tv_sec <= 0 || stamp.tv_nsec < 0 || stamp.tv_nsec >= 1000000000)
        throw std::runtime_error("DATA_INVALID: malformed kernel timestamp");
      out.kernel_realtime_ns = int64_t(stamp.tv_sec)*1000000000 + stamp.tv_nsec;
      timestamp = true;
    }
    if (c->cmsg_type == SO_RXQ_OVFL && c->cmsg_len == CMSG_LEN(sizeof(uint32_t)))
      std::memcpy(&out.socket_drops, CMSG_DATA(c), sizeof(uint32_t));
  }
  if (!timestamp) throw std::runtime_error("DATA_INVALID: kernel timestamp missing");
  const auto now = ClockBracket::sample();
  // Retain a fixed transform for the capture; bound its drift on every receive.
  // A clock step invalidates the capture, it is not fitted as motor latency.
  out.clock_uncertainty_ns = origin_.uncertainty_ns + now.uncertainty_ns
                            + std::abs(now.offset_ns - origin_.offset_ns);
  out.kernel_monotonic_ns = out.kernel_realtime_ns - origin_.offset_ns;
  if (out.clock_uncertainty_ns > max_clock_error_ns_ ||
      out.kernel_monotonic_ns > out.dequeue_ns + out.clock_uncertainty_ns ||
      out.kernel_monotonic_ns <= previous_ns_)
    throw std::runtime_error("DATA_INVALID: receive clock discontinuity or uncertainty");
  previous_ns_ = out.kernel_monotonic_ns;
  out.drop_delta = out.socket_drops - drops_; drops_ = out.socket_drops;
  if (wire.can_dlc > 8) throw std::runtime_error("DATA_INVALID: CAN DLC");
  out.frame.id = wire.can_id & CAN_EFF_MASK;
  out.frame.extended = wire.can_id & CAN_EFF_FLAG;
  out.frame.rtr = wire.can_id & CAN_RTR_FLAG;
  out.frame.error = wire.can_id & CAN_ERR_FLAG;
  if (!out.frame.extended && !out.frame.error && out.frame.id>CAN_SFF_MASK)
    throw std::runtime_error("DATA_INVALID: standard CAN identifier out of range");
  out.frame.dlc = wire.can_dlc;
  out.frame.rx_ns = out.kernel_monotonic_ns;
  std::memcpy(out.frame.data, wire.data, 8);
  return true;
}

Journal::Journal(const std::string& path, const std::string& header, size_t capacity)
    : queue_(capacity) {
  if (capacity < 2) throw std::invalid_argument("journal capacity must be at least two");
  fd_ = open(path.c_str(), O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW, 0600);
  if (fd_ < 0) throw std::runtime_error("cannot create immutable capture: " + std::string(strerror(errno)));
  if (!write_line(header.data(), header.size()) || fsync(fd_)) {
    close(fd_); fd_ = -1;
    throw std::runtime_error("capture header not durable");
  }
  const auto parent=std::filesystem::path(path).parent_path();
  const int directory=open(parent.empty()?".":parent.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC);
  const bool durable=directory>=0 && fsync(directory)==0;
  if (directory>=0) close(directory);
  if (!durable) { close(fd_); fd_=-1; throw std::runtime_error("capture directory not durable"); }
  try { writer_ = std::thread(&Journal::drain, this); }
  catch (...) { close(fd_); fd_ = -1; throw; }
}
Journal::~Journal() {
  done_ = true;
  if (writer_.joinable()) writer_.join();
  if (fd_ >= 0) close(fd_);
  // No footer on an abandoned session: the offline reviewer must reject it.
}
bool Journal::write_bytes(const char* data, size_t size) {
  while (size) {
    const auto n = write(fd_, data, size);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) { failure_code_=3; failed_ = true; return false; }
    size -= n; data += n;
  }
  return true;
}
bool Journal::write_line(const char* data, size_t size) {
  if (!write_bytes(data,size) || !write_bytes("\n",1)) return false;
  ++written_; return true;
}
bool Journal::append(const std::string& text) {
  if (failed_ || done_ || text.size() >= sizeof(Line::bytes) || text.find('\n') != std::string::npos) {
    if (!failed_) failure_code_=2;
    failed_ = true; return false;
  }
  const auto head = head_.load(std::memory_order_relaxed);
  const auto next = (head+1)%queue_.size();
  const auto tail=tail_.load(std::memory_order_acquire);
  if (next == tail) { failure_code_=1; failed_ = true; return false; }
  const auto depth=(next+queue_.size()-tail)%queue_.size();
  if (depth>high_water_) high_water_=depth;
  auto& row = queue_[head]; row.size = text.size();
  std::memcpy(row.bytes, text.data(), row.size);
  head_.store(next, std::memory_order_release);
  return true;
}
void Journal::drain() {
  // One bounded staging buffer turns small sensor records into large writes.
  // Acquisition never waits on disk, even on a slow mounted filesystem.
  std::array<char,256*1024> buffer;
  size_t bytes=0, records=0;
  auto last=std::chrono::steady_clock::now();
  auto flush=[&]() {
    if (bytes && !write_bytes(buffer.data(),bytes)) return false;
    written_+=records; bytes=0; records=0; last=std::chrono::steady_clock::now(); return true;
  };
  for (;;) {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      if (done_) { flush(); return; }
      if (std::chrono::steady_clock::now()-last>=std::chrono::milliseconds(20) && !flush()) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    const auto& row = queue_[tail];
    if (bytes+row.size+1>buffer.size() && !flush()) return;
    std::memcpy(buffer.data()+bytes,row.bytes,row.size); bytes+=row.size; buffer[bytes++]='\n'; ++records;
    tail_.store((tail+1)%queue_.size(), std::memory_order_release);
    if (bytes>=64*1024 && !flush()) return;
  }
}
bool Journal::finish(const std::string& footer) {
  if (finished_) return false;
  done_ = true; writer_.join(); finished_ = true;
  // A footer can record failure, but a disk/queue failure cannot claim completion.
  if (failed_) return false;
  if (!write_line(footer.data(), footer.size()) || fsync(fd_)) { failure_code_=3; failed_ = true; return false; }
  return true;
}
const char* Journal::failure_reason() const {
  switch (failure_code_.load()) {
    case 1: return "capture queue full";
    case 2: return "capture line invalid or append after finish";
    case 3: return "capture disk write/flush failed";
    default: return "capture recording interrupted";
  }
}
std::string receipt_json(const Receipt& r, const std::string& axis, uint64_t sequence) {
  if (axis != "yaw" && axis != "pitch") throw std::invalid_argument("axis");
  std::ostringstream s; s.precision(17);
  s << "{\"kind\":\"can_rx\",\"axis\":\"" << axis << "\",\"sequence\":" << sequence
    << ",\"generation\":1,\"kernel_realtime_ns\":" << r.kernel_realtime_ns
    << ",\"kernel_monotonic_ns\":" << r.kernel_monotonic_ns << ",\"dequeue_ns\":" << r.dequeue_ns
    << ",\"clock_uncertainty_ns\":" << r.clock_uncertainty_ns << ",\"socket_drops\":" << r.socket_drops
    << ",\"drop_delta\":" << r.drop_delta << ",\"id\":" << r.frame.id
    << ",\"extended\":" << (r.frame.extended?"true":"false")
    << ",\"rtr\":" << (r.frame.rtr?"true":"false") << ",\"error\":" << (r.frame.error?"true":"false")
    << ",\"dlc\":" << int(r.frame.dlc) << ",\"bytes\":[";
  for (int i=0; i<8; ++i) s << (i?",":"") << int(r.frame.data[i]);
  s << "]";
  gm6020::Feedback yaw;
  if (axis == "yaw" && gm6020::decode(r.frame, 1, yaw))
    s << ",\"angle_raw\":" << yaw.angle_count << ",\"speed_rpm\":" << yaw.speed_rpm
      // The command scale is documented; baseline capture has no bound feedback
      // calibration. Retain raw counts rather than applying the legacy conversion.
      << ",\"current_raw\":" << yaw.current_raw << ",\"current_A\":null"
      << ",\"temperature_raw\":" << int(yaw.temperature_raw) << ",\"temperature_C\":null";
  // Preserve pitch's wire fields, including temperature. Its feedback torque is
  // NOT Iq; current needs an independently correlated register observation.
  if (axis == "pitch" && r.frame.extended && !r.frame.error && !r.frame.rtr && r.frame.dlc == 8 &&
      cybergear::unpack_ext_id(r.frame.id).comm_type == 2) {
    auto word = [&](int n) { return (unsigned(r.frame.data[n])<<8)|r.frame.data[n+1]; };
    s << ",\"angle_raw\":" << word(0) << ",\"velocity_raw\":" << word(2)
      << ",\"torque_raw\":" << word(4) << ",\"temperature_raw\":" << word(6)
      << ",\"temperature_C\":" << word(6)*0.1;
  }
  s << "}"; return s.str();
}
} // namespace ota::commission
