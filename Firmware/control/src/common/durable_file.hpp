#pragma once
// OpenAutoTurret — a small state file written so it survives a power cut (ADR-008 D5).
//
// Temp file, fsync it, rename it over the old one, fsync the directory. A reader never sees half a
// file; a power cut leaves either the old file or the new one, never an empty one (ext4's delayed
// allocation does exactly that to a rename over unsynced data, and the station's supply is marginal).
// The directory fsync makes the rename itself durable.
//
// Blocking disk I/O: call it from a writer thread, never from the control thread.
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

namespace ota {

inline bool write_file_durably(const std::string& path, const std::string& text, std::string& err,
                               const std::string& tmp_suffix = ".tmp") {
  const std::string tmp = path + tmp_suffix;
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) {
    err = "cannot open " + tmp + " for writing: " + std::strerror(errno);
    return false;
  }
  std::size_t off = 0;
  bool ok = true;
  while (off < text.size()) {
    const ssize_t n = ::write(fd, text.data() + off, text.size() - off);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) { ok = false; break; }
    off += static_cast<std::size_t>(n);
  }
  ok = ok && ::fsync(fd) == 0;
  int e = errno;
  if (::close(fd) != 0 && ok) { ok = false; e = errno; }
  if (!ok) {
    err = "write failed for " + tmp + ": " + std::strerror(e);
    return false;
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    err = "rename " + tmp + " -> " + path + " failed: " + std::strerror(errno);
    return false;
  }
  const std::string::size_type slash = path.find_last_of('/');
  const std::string dir = slash == std::string::npos ? std::string(".")
                          : slash == 0               ? std::string("/")
                                                     : path.substr(0, slash);
  const int dfd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  const bool dir_ok = dfd >= 0 && ::fsync(dfd) == 0;
  e = errno;
  if (dfd >= 0) ::close(dfd);
  err.clear();
  if (!dir_ok) {
    // The new content is in place and readable: the write succeeded. Only its survival of a power
    // cut is in doubt, so this is a warning, not a failure -- a caller that stopped part-way on it
    // would leave a half-updated set of files (ADR-008 review, 2026-11-10).
    std::fprintf(stderr, "warning: replaced %s but could not sync %s: %s\n", path.c_str(), dir.c_str(),
                 std::strerror(e));
  }
  return true;
}

}  // namespace ota
