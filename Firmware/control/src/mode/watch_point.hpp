#pragma once
// SURVEILLANCE's watch point (owner, 2026-10-05): a saved pose that survives restarts and deploys.
//
// What is stored is chosen so that it means the same physical direction in the next session, which
// rules out the obvious number. Yaw's joint position on the continuous station is session-relative
// -- its zero is wherever the axis stood when controld opened the drive -- so it is saved as the
// GM6020's own absolute angle (the motor drives the turret directly, no gearing, so one motor turn
// is one turret turn). A bounded yaw, which is homed, is saved in its homed frame, and so is pitch,
// which is homed against its end stop on every boot. The yaw angle stays valid as long as the base
// does not turn: moving the tripod moves the watch point with it, which is the owner's accepted cost.
//
// The control loop never touches the file. The pose is loaded once at startup, before the loop
// runs, and a save hands a copy to WatchPointStore's own thread, which writes it beside the target
// and renames it over (a crash mid-write leaves the previous point, never a torn file).
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace ota {

struct WatchPoint {
  bool valid = false;
  // True: yaw_rad is the yaw drive's absolute angle, wrapped into [0, 2*pi). False: the homed frame.
  bool yaw_absolute = false;
  double yaw_rad = 0.0;
  double pitch_homed_rad = 0.0;   // pitch in its homed (logical) frame
  int64_t saved_unix_s = 0;
};

std::string watch_point_to_json(const WatchPoint& w);
// Parse; false (and `err`) for anything that is not a complete schema-1 record with finite numbers.
bool watch_point_from_json(const std::string& text, WatchPoint& out, std::string& err);
// Load a file. A missing file is not an error (out.valid stays false, err stays empty); anything
// unreadable or malformed is, and is reported rather than half-loaded.
bool load_watch_point(const std::string& path, WatchPoint& out, std::string& err);
// Write `w` to a temp file beside `path` and rename it over. Blocking; WatchPointStore calls it.
bool save_watch_point(const std::string& path, const WatchPoint& w, std::string& err);

// The one writer. save() never blocks the caller: the newest point wins, and a failure is counted
// and logged, not thrown. With an empty path nothing is written and save() reports that.
class WatchPointStore {
 public:
  explicit WatchPointStore(std::string path = {});
  ~WatchPointStore();
  WatchPointStore(const WatchPointStore&) = delete;
  WatchPointStore& operator=(const WatchPointStore&) = delete;

  const std::string& path() const { return path_; }
  bool persistent() const { return !path_.empty(); }
  void save(const WatchPoint& w);
  // 1: the last save reached the disk; 0: it failed; -1: nothing saved yet in this session.
  int last_result() const { return last_result_.load(); }

 private:
  void run();
  std::string path_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<WatchPoint> pending_;
  bool stop_ = false;
  std::atomic<int> last_result_{-1};
  std::thread thread_;
};

}  // namespace ota
