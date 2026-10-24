#pragma once
// ADR-005: the station's calibration records. A record is a payload set -- the yaw and pitch servo
// assets, the yaw accuracy limits, the camera->pitch extrinsic and the bore -- kept in
// <state>/calibration/records/<id>/ beside active.json, which names the one in force.
// tools/calibration_records.py seeds #1 into an empty store and stages imports in incoming/; this
// store adopts those at start and is otherwise the only writer.
//
// There are no hashes (owner, 2026-10-24): a record is usable when its files parse and make a servo,
// a position loop, a rotation and a valid bore. A file edited by hand needs no bookkeeping.
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "control/servo_assets.hpp"
#include "geometry/bore_alignment.hpp"
#include "geometry/turret_kinematics.hpp"

namespace YAML { class Node; }

namespace ota::calibration {

// Where one part of a record came from (record.json "parts"): measured, inherited, trimmed, shipped.
struct PartInfo {
  std::string file, origin, date, run, rev;
  int from = 0;
};

// bore.yaml (ADR-005 D9). The operator's trims are kept apart from the base angles, in mrad, so the
// record says what was configured and what was adjusted. A positive trim moves the bore mark right
// (or up) on the picture: the bore turns right of where the camera looks.
struct BoreCamera {
  geo::CameraOffset base;
  double trim_right_mrad = 0, trim_up_mrad = 0;
  geo::CameraOffset effective() const;
};

struct Bore {
  bool enabled = false;      // mode manual_depth: the tracker aims the bore, not the camera centre
  double zero_m = 10;        // the zeroing distance; infinity = no parallax
  BoreCamera wide, detail;
  geo::BoreAlignmentConfig alignment() const;
};

struct Record {
  int id = 0;
  std::string name;
  std::filesystem::path dir;
  std::map<std::string, PartInfo> parts;   // yaw, pitch, accuracy, extrinsic, bore
  Bore bore;
  // record.json's last_run: the latest run on this record (ADR-005 D6, LAST RUN). Empty kind: none.
  struct LastRun { std::string kind, verdict, run, date; } last_run;
  std::string unusable;                    // why it cannot be put in force; empty when it can
  std::filesystem::path path(const std::string& part) const;
};

// Reads and checks one record directory; a problem is reported in `unusable`, never thrown.
Record read_record(const std::filesystem::path& dir, int id);

// A record parsed into what is applied: the servo assets for the backend, the extrinsic (in its
// homed frame) and the bore for the tracking geometry. Made off the control thread (file I/O).
struct Prepared {
  Record record;
  ServoAssets servo;
  geo::TurretKinematics kinematics;
  geo::BoreAlignmentConfig alignment;
};
bool prepare(const Record& record, Prepared& out, std::string& err);

// The records for telemetry: id, name, why unusable, each part's origin and the bore.
std::string records_json(const std::vector<Record>& records);
// active.json, written atomically: the record in force, and whether it has been VERIFYed since it was
// chosen (the nudge survives a restart).
bool write_active(const std::filesystem::path& state_calibration_dir, int id, bool unverified, std::string& err);
// bore.yaml, written whole, and record.json's bore part marked trimmed on `date` (ADR-005 D9).
std::string bore_yaml(const Bore& b);
bool write_bore(const std::filesystem::path& dir, const Bore& b, const std::string& date, std::string& err);
std::string today_local();
// A YAML (or JSON) document as JSON: what record.json, and a session journal's manifest, are written
// as (json.loads reads them).
std::string to_json(const YAML::Node& n, int indent = 0);

// Record files, changed on disk (call off the control thread; each is atomic). Records leave the way
// they came (owner, 2026-10-24): rename, a copy made from another, removal to deleted/ (kept, not
// erased: a measured record holds an hour of station time). #1 is the shipped backup -- the caller
// refuses it; these do what they are asked.
bool rename_record(const std::filesystem::path& dir, const std::string& name, std::string& err);
bool copy_record(const Record& from, const std::filesystem::path& dir, const std::string& name, std::string& err);
bool remove_record(const std::filesystem::path& state_calibration_dir, int id, std::string& err);
// ADR-005 D7.6: a run's accepted parts (yaw_servo.json, pitch_servo.json, yaw_accuracy.json, whichever
// `from` holds) into the record at `dir`. Checked first, on a staged copy: parts that would make the
// record unusable install nothing. Each replaced file is kept in history/<stamp>/, and record.json
// marks the part measured by `run` on `date`. `installed` names the parts.
bool install_parts(const std::filesystem::path& dir, int id, const std::filesystem::path& from, const std::string& run,
                   const std::string& date, std::vector<std::string>& installed, std::string& err);
// record.json's last_run: what the latest run on this record was, how it ended and when.
bool write_last_run(const std::filesystem::path& dir, const std::string& kind, const std::string& verdict,
                    const std::string& run, const std::string& date, std::string& err);
// A name: 1-24 letters, digits, spaces, '-' or '_', stored upper case. Empty when it is not one.
std::string record_name(const std::string& raw);

// File work off the 200 Hz control thread: jobs run in order on one thread of their own.
class Writer {
 public:
  Writer();
  ~Writer();
  Writer(const Writer&) = delete;
  Writer& operator=(const Writer&) = delete;
  void post(std::function<void()> job);

 private:
  std::mutex m_;
  std::condition_variable cv_;
  std::deque<std::function<void()>> jobs_;
  bool stop_ = false;
  std::thread thread_;
};

class Store {
 public:
  // Opens <state_dir>/calibration. A missing store is an empty one, not an error; false only when
  // the directory exists and cannot be read.
  bool open(const std::filesystem::path& state_dir, std::string& err);
  bool empty() const { return records_.empty(); }
  const std::vector<Record>& records() const { return records_; }
  const std::vector<std::string>& adopted() const { return adopted_; }
  int active_id() const { return active_; }
  const Record* find(int id) const;
  // The record in force when it is usable; otherwise null, and `why` says what is wrong. Never
  // another record instead (ADR-005 D4): a servo designed for another payload is the failure this
  // feature exists to prevent.
  const Record* in_force(std::string& why) const;
  const std::filesystem::path& root() const { return root_; }
  void set_active(int id) { active_ = id; }
  bool unverified() const { return unverified_; }
  // After a trim: the record's bore in memory, its part marked trimmed (the file is written apart).
  void set_bore(int id, const Bore& b, const std::string& date);
  // A run's end, in memory (the file is written apart, write_last_run).
  void set_last_run(int id, const Record::LastRun& last);
  std::filesystem::path dir_of(int id) const;
  // The next id: never one in records/ or deleted/.
  int next_id() const;
  // After a change on disk: read one record again (added if new), or forget a removed one. refresh()
  // reads the files; a caller that holds a lock the control loop takes reads the record first
  // (read_record) and only puts it here, in memory.
  void refresh(int id);
  void put(Record record);
  void drop(int id);

 private:
  void adopt_incoming();
  std::filesystem::path root_;
  std::vector<Record> records_;
  std::vector<std::string> adopted_;
  int active_ = 0;
  bool unverified_ = false;
};

}  // namespace ota::calibration
