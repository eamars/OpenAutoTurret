#include "mode/target_type.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>

#include <spdlog/spdlog.h>

namespace ota {

const char* target_type_name(TargetType t) {
  switch (t) {
    case TargetType::Human: return "HUMAN";
    case TargetType::Head: return "HEAD";
    case TargetType::Cat: return "CAT";
    case TargetType::Dog: return "DOG";
    case TargetType::None: break;
  }
  return "";
}

bool target_type_from_name(const std::string& name, TargetType& t) {
  std::string up;
  for (char c : name)
    if (!std::isspace(static_cast<unsigned char>(c))) up += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  for (TargetType k : {TargetType::Human, TargetType::Head, TargetType::Cat, TargetType::Dog})
    if (up == target_type_name(k)) {
      t = k;
      return true;
    }
  return false;
}

std::string target_type_to_json(TargetType t) {
  return std::string("{\"schema\":1,\"target_type\":\"") + target_type_name(t) + "\"}\n";
}

bool target_type_from_json(const std::string& text, TargetType& out, std::string& err) {
  // One flat record written by this file; a full JSON parser would be a dependency for one string.
  if (text.find("\"schema\":1") == std::string::npos) {
    err = "not a schema-1 target type record";
    return false;
  }
  const std::string key = "\"target_type\":\"";
  const auto at = text.find(key);
  const auto end = at == std::string::npos ? at : text.find('"', at + key.size());
  if (end == std::string::npos) {
    err = "no target_type in the record";
    return false;
  }
  TargetType t;
  if (!target_type_from_name(text.substr(at + key.size(), end - at - key.size()), t)) {
    err = "unknown target type '" + text.substr(at + key.size(), end - at - key.size()) + "'";
    return false;
  }
  out = t;
  err.clear();
  return true;
}

bool load_target_type(const std::string& path, TargetType& out, std::string& err) {
  err.clear();
  std::ifstream f(path);
  if (!f) return false;  // missing: the default applies, not an error
  std::stringstream s;
  s << f.rdbuf();
  return target_type_from_json(s.str(), out, err);
}

TargetTypeStore::TargetTypeStore(std::string path) : path_(std::move(path)) {
  if (persistent()) thread_ = std::thread([this] { run(); });
}

TargetTypeStore::~TargetTypeStore() {
  {
    std::lock_guard lk(mutex_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void TargetTypeStore::save(TargetType t) {
  if (!persistent()) {
    last_result_.store(0);
    return;
  }
  {
    std::lock_guard lk(mutex_);
    pending_ = t;
  }
  cv_.notify_all();
}

void TargetTypeStore::run() {
  std::unique_lock lk(mutex_);
  for (;;) {
    cv_.wait(lk, [this] { return stop_ || pending_.has_value(); });
    if (pending_) {
      const TargetType t = *pending_;
      pending_.reset();
      lk.unlock();
      const std::string tmp = path_ + ".tmp";
      bool ok = false;
      {
        std::ofstream f(tmp, std::ios::trunc);
        if (f) {
          f << target_type_to_json(t);
          f.flush();
          ok = f.good();
        }
      }
      ok = ok && std::rename(tmp.c_str(), path_.c_str()) == 0;
      last_result_.store(ok ? 1 : 0);
      if (ok) spdlog::info("target type {} saved to {}", target_type_name(t), path_);
      else spdlog::error("target type {} NOT saved to {}; it holds for this session only", target_type_name(t), path_);
      lk.lock();
      continue;  // a save queued during the write runs before stopping
    }
    if (stop_) return;
  }
}

}  // namespace ota
