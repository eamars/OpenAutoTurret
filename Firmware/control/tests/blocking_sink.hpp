#pragma once
// A storage device that does not answer, for tests of the evidence writer.
//
// A FIFO made where an evidence file will be written blocks whoever opens it for writing until
// somebody opens it for reading: exactly a disk that has stopped answering, and real file I/O
// rather than a hook. Code that writes in the caller's thread hangs on it; code that hands the
// write to a background thread does not. pump_fifos() is the reader that finally answers.
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <future>
#include <string>
#include <thread>
#include <vector>

namespace ota::test {

inline bool make_blocking_sink(const std::string& path) { return ::mkfifo(path.c_str(), 0600) == 0; }

// Opens every FIFO for reading (non-blocking, so a FIFO nobody writes to does not hang the test)
// and reads them until `done` is ready, then drains what is left. Returns each FIFO's bytes.
template <typename T>
std::vector<std::string> pump_fifos(const std::vector<std::string>& paths, std::future<T>& done) {
  std::vector<int> fds;
  for (const auto& p : paths) fds.push_back(::open(p.c_str(), O_RDONLY | O_NONBLOCK));
  std::vector<std::string> got(paths.size());
  auto read_all = [&] {
    char buf[65536];
    for (std::size_t i = 0; i < fds.size(); ++i) {
      if (fds[i] < 0) continue;
      for (;;) {
        const ssize_t n = ::read(fds[i], buf, sizeof buf);
        if (n <= 0) break;  // EAGAIN (writer open, nothing yet) or no writer at all
        got[i].append(buf, static_cast<std::size_t>(n));
      }
    }
  };
  while (done.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) read_all();
  read_all();
  for (int fd : fds)
    if (fd >= 0) ::close(fd);
  return got;
}

}  // namespace ota::test
