#pragma once
// The writer behind every evidence file the control loop leaves (trip traces, stop records).
//
// §46 says the 200 Hz loop does no file I/O, and the loggers were moved off it for that reason
// in 2026-09. The evidence files were not: a supervisor BRAKE on 2026-10-20 15:34:13 latched a
// fault whose trip trace was formatted and written in the control step itself, and that step took
// 79.6 ms while visiond loaded its model. The owner's scheduling ruling (2026-10-03) keeps real
// time for motor control only, so the write belongs on a background thread, and this is that
// thread.
//
// The contract is the async logger's: a producer never waits for storage. The queue is bounded
// and a full queue drops the new job and counts it, because a stalled disk may cost evidence,
// never a control cycle. The critical section a producer shares with the writer is the move of
// one job, nothing that can touch a file.
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace ota::telemetry {

class EvidenceWriter {
 public:
  using Job = std::function<void()>;
  // A trip and its stop records are a handful of jobs; sixteen is room for a burst of them
  // while one file is still being written, and small enough that a stuck disk is visible as
  // drops within seconds rather than as memory.
  static constexpr std::size_t kCapacity = 16;

  explicit EvidenceWriter(std::size_t capacity = kCapacity) : jobs_(capacity) {}
  ~EvidenceWriter() { stop(); }
  EvidenceWriter(const EvidenceWriter&) = delete;
  EvidenceWriter& operator=(const EvidenceWriter&) = delete;

  // Spawns the writer thread (Background class: nice 10, never real time). Called from setup,
  // not from the loop: a thread created by a SCHED_FIFO thread would start life as one.
  // Idempotent.
  void start();
  // Runs what is already queued, then joins. A shutdown keeps its last stop record.
  void stop();
  bool running() const;

  // Never waits on I/O. False, and counted, when the queue is full or the writer is not
  // running; the job is then not run at all.
  bool post(Job job);
  // Blocks until every job queued before the call has run. For shutdown and tests, never
  // for the loop.
  void drain();

  uint64_t dropped() const;
  uint64_t completed() const;

 private:
  void run();

  mutable std::mutex mu_;
  std::condition_variable work_cv_;
  std::condition_variable idle_cv_;
  std::vector<Job> jobs_;     // fixed-capacity ring, sized once
  std::size_t head_ = 0;      // next job to run
  std::size_t count_ = 0;     // jobs queued
  bool busy_ = false;         // a job is running outside the lock
  bool stopping_ = false;
  uint64_t dropped_ = 0;
  uint64_t completed_ = 0;
  std::thread thread_;
};

}  // namespace ota::telemetry
