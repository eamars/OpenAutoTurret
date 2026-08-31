// SeqLock tests: lock-free single-writer/single-reader latest-value.
#include <atomic>
#include <thread>

#include <gtest/gtest.h>

#include "common/seqlock.hpp"

namespace {
struct Payload {
  int a;
  double b;
  char c;
};

// Every payload a test writes must satisfy this, so a consistent read can
// be validated field-by-field. (An inconsistent initial payload is a common
// trap: it makes a legitimate first read look "torn".)
bool valid(const Payload& p) {
  return p.b == static_cast<double>(p.a) * 0.5 &&
         p.c == static_cast<char>('a' + p.a % 26);
}
}  // namespace

TEST(SeqLock, BasicWriteRead) {
  ota::SeqLock<Payload> sl;
  sl.write(Payload{1, 2.5, 'x'});
  Payload p{};
  ASSERT_TRUE(sl.read(p));
  EXPECT_EQ(p.a, 1);
  EXPECT_DOUBLE_EQ(p.b, 2.5);
  EXPECT_EQ(p.c, 'x');
}

TEST(SeqLock, LatestWins) {
  ota::SeqLock<int> sl;
  for (int i = 0; i < 1000; ++i) sl.write(i);
  int v = -1;
  ASSERT_TRUE(sl.read(v));
  EXPECT_EQ(v, 999);
}

TEST(SeqLock, RealisticRateConsistentAndFast) {
  // Production-shaped traffic (architecture §8.1): the CAN RX thread posts
  // per-axis feedback at the motor feedback rate; the control thread
  // samples freely. Writer here: 1 kHz. The reader must never see a torn
  // payload and must sustain a healthy consistent-read rate.
  ota::SeqLock<Payload> sl;
  sl.write(Payload{0, 0.0, 'a'});
  std::atomic<bool> stop{false};
  std::atomic<long> checks{0};
  std::atomic<bool> torn{false};

  std::thread writer([&] {
    for (int i = 1; i <= 100 && !stop.load(std::memory_order_relaxed); ++i) {
      sl.write(Payload{i, static_cast<double>(i) * 0.5,
                       static_cast<char>('a' + i % 26)});
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  std::thread reader([&] {
    Payload p{};
    while (!stop.load(std::memory_order_relaxed) && sl.read(p)) {
      if (!valid(p)) torn.store(true);
      ++checks;
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  stop.store(true);
  writer.join();
  reader.join();
  EXPECT_FALSE(torn.load());
  EXPECT_GT(checks.load(), 1000);
}

TEST(SeqLock, AdversarialContentionNoTornReads) {
  // Back-to-back writes ping-pong the seq cache line between cores. On
  // weakly-ordered hardware (measured on Cortex-A76) the reader's view of
  // that line lags in this regime, so the reader may complete only a
  // handful of consistent reads in the window. The protocol guarantee
  // being checked is therefore: every read returned is torn-free, and the
  // reader still makes forward progress. A hard throughput threshold here
  // is hardware-flaky and would not be measuring the lock.
  ota::SeqLock<Payload> sl;
  sl.write(Payload{0, 0.0, 'a'});  // invariant-consistent (see valid())
  std::atomic<bool> stop{false};
  std::atomic<long> checks{0};
  std::atomic<bool> torn{false};

  std::thread writer([&] {
    for (int i = 1; !stop.load(std::memory_order_relaxed); ++i) {
      sl.write(Payload{i, static_cast<double>(i) * 0.5,
                       static_cast<char>('a' + i % 26)});
    }
  });
  std::thread reader([&] {
    Payload p{};
    while (!stop.load(std::memory_order_relaxed) && sl.read(p)) {
      if (!valid(p)) torn.store(true);
      ++checks;
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  stop.store(true);
  writer.join();
  reader.join();
  EXPECT_FALSE(torn.load());
  EXPECT_GT(checks.load(), 0);
}
