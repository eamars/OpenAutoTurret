#include "telemetry/evidence_writer.hpp"

#include "common/thread_class.hpp"

namespace ota::telemetry {

void EvidenceWriter::start() {
  std::lock_guard<std::mutex> lk(mu_);
  if (thread_.joinable()) return;
  stopping_ = false;
  thread_ = std::thread([this] { run(); });
}

void EvidenceWriter::stop() {
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!thread_.joinable()) return;
    stopping_ = true;
  }
  work_cv_.notify_all();
  thread_.join();
  std::lock_guard<std::mutex> lk(mu_);
  thread_ = std::thread();
}

bool EvidenceWriter::running() const {
  std::lock_guard<std::mutex> lk(mu_);
  return thread_.joinable() && !stopping_;
}

bool EvidenceWriter::post(Job job) {
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!thread_.joinable() || stopping_ || count_ == jobs_.size()) {
      ++dropped_;
      return false;
    }
    jobs_[(head_ + count_) % jobs_.size()] = std::move(job);
    ++count_;
  }
  work_cv_.notify_one();
  return true;
}

void EvidenceWriter::drain() {
  std::unique_lock<std::mutex> lk(mu_);
  if (!thread_.joinable()) return;
  idle_cv_.wait(lk, [this] { return count_ == 0 && !busy_; });
}

uint64_t EvidenceWriter::dropped() const {
  std::lock_guard<std::mutex> lk(mu_);
  return dropped_;
}

uint64_t EvidenceWriter::completed() const {
  std::lock_guard<std::mutex> lk(mu_);
  return completed_;
}

void EvidenceWriter::run() {
  apply_thread_class("evidence-write", ThreadClass::Background);
  std::unique_lock<std::mutex> lk(mu_);
  for (;;) {
    work_cv_.wait(lk, [this] { return stopping_ || count_ != 0; });
    if (count_ == 0) break;  // stopping, and everything queued has run
    Job job = std::move(jobs_[head_]);
    jobs_[head_] = nullptr;
    head_ = (head_ + 1) % jobs_.size();
    --count_;
    busy_ = true;
    lk.unlock();
    // The file I/O, outside the lock: a producer never waits on it. Evidence is best-effort,
    // so a job that throws costs its own file and not the writer.
    try {
      job();
    } catch (...) {
    }
    job = nullptr;
    lk.lock();
    busy_ = false;
    ++completed_;
    if (count_ == 0) idle_cv_.notify_all();
  }
  idle_cv_.notify_all();
}

}  // namespace ota::telemetry
