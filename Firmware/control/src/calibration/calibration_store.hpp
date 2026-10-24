#pragma once
// ADR-005: the station's calibration records. A record is a payload set -- the yaw and pitch servo
// assets, the yaw accuracy limits, the camera->pitch extrinsic and the bore -- kept in
// <state>/calibration/records/<id>/ beside active.json, which names the one in force.
// tools/calibration_records.py seeds #1 into an empty store and stages imports in incoming/; this
// store adopts those at start and is otherwise the only writer.
//
// There are no hashes (owner, 2026-10-24): a record is usable when its files parse and make a servo,
// a position loop, a rotation and a valid bore. A file edited by hand needs no bookkeeping.
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include "geometry/bore_alignment.hpp"

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
  std::string unusable;                    // why it cannot be put in force; empty when it can
  std::filesystem::path path(const std::string& part) const;
};

// Reads and checks one record directory; a problem is reported in `unusable`, never thrown.
Record read_record(const std::filesystem::path& dir, int id);

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

 private:
  void adopt_incoming();
  std::filesystem::path root_;
  std::vector<Record> records_;
  std::vector<std::string> adopted_;
  int active_ = 0;
};

}  // namespace ota::calibration
