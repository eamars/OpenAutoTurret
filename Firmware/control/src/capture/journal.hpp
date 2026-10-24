#pragma once
// A capture journal: JSON lines appended by one real-time producer and written by a disk thread of
// its own. commissiond's sessions and controld's calibration sessions (ADR-005 D7) write the same
// journals, which tools/servo_commission/journal.py reads.
#include <atomic>
#include <cstdint>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace ota::capture {

// One acquisition/event-loop producer; one disk writer. A full queue fails the
// session instead of delaying acquisition or silently discarding evidence.
class Journal {
 public:
  Journal(const std::string& path, const std::string& header, size_t capacity = 4096);
  ~Journal();
  Journal(const Journal&) = delete;
  Journal& operator=(const Journal&) = delete;
  bool append(std::string_view json_line);
  bool finish(const std::string& footer);
  bool healthy() const { return !failed_.load(); }
  uint64_t written() const { return written_.load(); }
  size_t high_water() const { return high_water_.load(); }
  const char* failure_reason() const;
 private:
  struct Line { uint32_t size{}; char bytes[4096]{}; };
  bool write_line(const char*, size_t);
  bool write_bytes(const char*, size_t);
  void drain();
  int fd_{-1};
  std::vector<Line> queue_;
  std::atomic<size_t> head_{0}, tail_{0};
  std::atomic<bool> failed_{false}, done_{false};
  std::atomic<uint64_t> written_{0};
  std::atomic<size_t> high_water_{0};
  std::atomic<int> failure_code_{0};
  std::thread writer_;
  bool finished_{false};
};

}  // namespace ota::capture
