// The evidence writer (telemetry/evidence_writer.hpp): the control loop's file I/O goes to a
// background thread through a bounded queue, and a producer never waits on it.
#include <gtest/gtest.h>

#include "telemetry/evidence_writer.hpp"

#include <atomic>
#include <chrono>
#include <future>

namespace {
using ota::telemetry::EvidenceWriter;
using Clock = std::chrono::steady_clock;

TEST(EvidenceWriter, AStuckWriteNeverStallsTheProducerAndAFullQueueDropsAndCounts) {
  // The 2026-10-20 15:34:13 stall in miniature: the first job is a write that does not
  // come back. Everything the producer does after it -- queueing up to capacity and
  // being refused beyond it -- has to return without waiting for that write.
  EvidenceWriter w(4);
  w.start();
  std::promise<void> disk_answers;
  std::shared_future<void> answered = disk_answers.get_future().share();
  std::promise<void> stuck_in_write;
  std::atomic<int> ran{0};
  ASSERT_TRUE(w.post([&] {
    stuck_in_write.set_value();
    answered.wait();
    ++ran;
  }));
  stuck_in_write.get_future().wait();  // the writer is now inside the stuck job

  const auto t0 = Clock::now();
  int queued = 0, refused = 0;
  for (int i = 0; i < 10; ++i) (w.post([&] { ++ran; }) ? queued : refused)++;
  const auto spent = Clock::now() - t0;

  EXPECT_EQ(queued, 4) << "the queue holds its capacity while the writer is stuck";
  EXPECT_EQ(refused, 6);
  EXPECT_EQ(w.dropped(), 6u) << "a refusal is counted, not silent";
  EXPECT_LT(spent, std::chrono::milliseconds(50)) << "posting waited on the stuck write";
  EXPECT_EQ(ran.load(), 0);

  disk_answers.set_value();
  w.drain();
  EXPECT_EQ(ran.load(), 5) << "the stuck job and the four queued behind it";
  EXPECT_EQ(w.completed(), 5u);
}

TEST(EvidenceWriter, StoppingRunsWhatIsAlreadyQueued) {
  // A shutdown's last stop record is queued just before the process ends; it must land.
  std::atomic<int> ran{0};
  {
    EvidenceWriter w;
    w.start();
    for (int i = 0; i < 8; ++i) ASSERT_TRUE(w.post([&] { ++ran; }));
  }
  EXPECT_EQ(ran.load(), 8);
}

TEST(EvidenceWriter, AWriterThatWasNeverStartedRefusesRatherThanQueueingForever) {
  EvidenceWriter w;
  EXPECT_FALSE(w.post([] {}));
  EXPECT_EQ(w.dropped(), 1u);
  w.drain();  // returns: nothing will ever run
}

TEST(EvidenceWriter, AJobThatThrowsCostsOnlyItself) {
  EvidenceWriter w;
  w.start();
  std::atomic<int> ran{0};
  ASSERT_TRUE(w.post([] { throw std::runtime_error("disk full"); }));
  ASSERT_TRUE(w.post([&] { ++ran; }));
  w.drain();
  EXPECT_EQ(ran.load(), 1);
}

}  // namespace
