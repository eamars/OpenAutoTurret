#pragma once
#include <array>
#include <fstream>
#include <string>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/vfs.h>
#include <unistd.h>
#include "common/logical_coordinates.hpp"
#include "control/safety_envelope.hpp"

namespace ota {
// Retained calibration survives application restarts, but never enables a
// motor: adoption separately requires fresh, already-energized drive feedback.
// Keep the command-path invalidation in pinned tmpfs memory: a shared SD-backed
// mapping can fault into filesystem writeback even on a single atomic store.
// A full host reboot discards this cache and conservatively requires homing.
class RetainedHoming {
  struct Data {
    uint32_t valid, version;
    uint64_t config_hash, ids[2];
    uint64_t checksum;
    std::array<AxisLogicalModel, kAxisCount> models;
    std::array<AxisLimits, kAxisCount> limits;
  };
  Data* data_ = nullptr;
  uint64_t hash_ = 1469598103934665603ULL;
  std::array<uint64_t, 2> ids_;
  static uint64_t checksum(const Data& data) {
    uint64_t hash=1469598103934665603ULL;
    const auto add=[&](const void* p, size_t n) {
      const auto* bytes=static_cast<const unsigned char*>(p);
      for (size_t i=0; i<n; ++i) { hash ^= bytes[i]; hash *= 1099511628211ULL; }
    };
    add(&data.models, sizeof data.models); add(&data.limits, sizeof data.limits);
    return hash;
  }
 public:
  RetainedHoming(const std::string& config, std::array<uint64_t, 2> ids,
                 const std::string& cache_path = "") : ids_(ids) {
    std::ifstream input(config, std::ios::binary);
    char c;
    while (input.get(c)) { hash_ ^= static_cast<uint8_t>(c); hash_ *= 1099511628211ULL; }
    if (!input.eof()) return;
    const auto path = cache_path.empty() ? "/dev/shm/ota-homing-" + std::to_string(getuid()) : cache_path;
    const int fd = open(path.c_str(), O_RDWR|O_CREAT|O_NOFOLLOW|O_CLOEXEC, 0600);
    if (fd < 0) return;
    struct stat st{};
    struct statfs fs{};
    constexpr long tmpfs_magic = 0x01021994;
    if (fstat(fd, &st) || st.st_uid != getuid() || !S_ISREG(st.st_mode) ||
        fstatfs(fd, &fs) || fs.f_type != tmpfs_magic || fchmod(fd, 0600) ||
        ftruncate(fd, sizeof(Data))) { close(fd); return; }
    void* memory = mmap(nullptr, sizeof(Data), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (memory == MAP_FAILED) return;
    if (mlock(memory, sizeof(Data))) { munmap(memory, sizeof(Data)); return; }
    data_ = static_cast<Data*>(memory);
    if (data_->version != 2 || data_->config_hash != hash_ ||
        data_->ids[0] != ids_[0] || data_->ids[1] != ids_[1] ||
        data_->checksum != checksum(*data_)) invalidate();
  }
  ~RetainedHoming() {
    if (data_) { munlock(data_, sizeof(Data)); munmap(data_, sizeof(Data)); }
  }
  bool valid() const { return data_ && __atomic_load_n(&data_->valid, __ATOMIC_ACQUIRE) == 1; }
  void invalidate() { if (data_) __atomic_store_n(&data_->valid, 0, __ATOMIC_RELEASE); }
  bool load(std::array<AxisLogicalModel, 2>& models, std::array<AxisLimits, 2>& limits) const {
    if (!valid()) return false;
    models=data_->models; limits=data_->limits;
    return valid();
  }
  void save(const std::array<AxisLogicalModel, 2>& models, const std::array<AxisLimits, 2>& limits) {
    if (!data_) return;
    invalidate(); data_->version=2; data_->config_hash=hash_;
    data_->ids[0]=ids_[0]; data_->ids[1]=ids_[1];
    data_->models=models; data_->limits=limits;
    data_->checksum=checksum(*data_);
    __atomic_store_n(&data_->valid, 1, __ATOMIC_RELEASE);
  }
};
}
