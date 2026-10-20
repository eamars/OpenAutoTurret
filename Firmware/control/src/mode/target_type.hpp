#pragma once
// What the turret follows: HUMAN, HEAD, CAT or DOG (owner, 2026-10-20;
// docs/DESIGN_TARGET_TYPE_2026-10-20.md). One at a time, chosen in the web's TARGETS drawer.
//
// controld owns the operator's choice: it persists it (run/state/target_type.json, like the watch
// point), publishes it in its telemetry, and holds the turret while perception changes model. visiond
// reads the request from that telemetry, applies it between frames, and stamps every perception frame
// with the type it applied (native header flag bits 2-4). The codes are perception/target_type.py's.
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace ota {

enum class TargetType : uint8_t { None = 0, Human = 1, Head = 2, Cat = 3, Dog = 4 };

// Today's behaviour (the pose model aiming at the head) is the default (owner, 2026-10-20).
constexpr TargetType kDefaultTargetType = TargetType::Head;

const char* target_type_name(TargetType t);  // "HUMAN", ...; "" for None
// Case-insensitive; false (and `t` untouched) for anything that is not one of the four.
bool target_type_from_name(const std::string& name, TargetType& t);
inline TargetType target_type_from_code(uint8_t code) {
  return code >= 1 && code <= 4 ? static_cast<TargetType>(code) : TargetType::None;
}

std::string target_type_to_json(TargetType t);
bool target_type_from_json(const std::string& text, TargetType& out, std::string& err);
// A missing file is not an error (returns false, err empty): the default applies.
bool load_target_type(const std::string& path, TargetType& out, std::string& err);

// The one writer, off the control thread: write beside the target and rename over it.
class TargetTypeStore {
 public:
  explicit TargetTypeStore(std::string path = {});
  ~TargetTypeStore();
  TargetTypeStore(const TargetTypeStore&) = delete;
  TargetTypeStore& operator=(const TargetTypeStore&) = delete;

  bool persistent() const { return !path_.empty(); }
  const std::string& path() const { return path_; }
  void save(TargetType t);
  // 1: the last save reached the disk; 0: it failed; -1: nothing saved yet in this session.
  int last_result() const { return last_result_.load(); }

 private:
  void run();
  std::string path_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<TargetType> pending_;
  bool stop_ = false;
  std::atomic<int> last_result_{-1};
  std::thread thread_;
};

}  // namespace ota
