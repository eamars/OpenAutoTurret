#include "capture/journal.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <stdexcept>
#include <unistd.h>

namespace ota::capture {

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
bool Journal::append(std::string_view text) {
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

}  // namespace ota::capture
