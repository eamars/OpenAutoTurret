// Per-thread scheduling classes (common/thread_class.hpp; docs/operations/os-setup.md).
// Each case runs on its own thread so the test process keeps its own class.
#include <gtest/gtest.h>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "common/thread_class.hpp"

namespace {

struct RtEnv {
  explicit RtEnv(const char* value) {
    if (value) ::setenv("OTA_RT", value, 1); else ::unsetenv("OTA_RT");
  }
  ~RtEnv() { ::unsetenv("OTA_RT"); }
};

std::string thread_name() {
  char buf[32] = {};
  pthread_getname_np(pthread_self(), buf, sizeof buf);
  return buf;
}

int own_nice() {
  return ::getpriority(PRIO_PROCESS, static_cast<id_t>(::syscall(SYS_gettid)));
}

}  // namespace

TEST(ThreadClass, WithoutTheLauncherFlagOnlyTheNameChanges) {
  RtEnv env(nullptr);
  std::thread([] {
    EXPECT_FALSE(ota::apply_thread_class("web-client", ota::ThreadClass::Background));
    EXPECT_EQ(thread_name(), "web-client");
    EXPECT_EQ(own_nice(), 0);
    EXPECT_FALSE(ota::apply_thread_class("rx-can0", ota::ThreadClass::Motor, 48));
    EXPECT_EQ(::sched_getscheduler(0), SCHED_OTHER);
  }).join();
}

TEST(ThreadClass, BackgroundThreadsAreNicedWithoutPrivilege) {
  RtEnv env("1");
  std::thread([] {
    EXPECT_TRUE(ota::apply_thread_class("log-writer", ota::ThreadClass::Background));
    EXPECT_EQ(own_nice(), 10);
  }).join();
}

TEST(ThreadClass, AMotorThreadIsRealTimeOnlyWhenGrantedAndItsChildrenAreNot) {
  RtEnv env("1");
  std::thread([] {
    const bool granted = ota::apply_thread_class("pitch-servo", ota::ThreadClass::Motor, 46);
    const int policy = ::sched_getscheduler(0) & ~SCHED_RESET_ON_FORK;
    if (!granted) {
      // A workstation, or a station without the OS grant: refused, and nothing else changed.
      EXPECT_EQ(policy, SCHED_OTHER);
      return;
    }
    EXPECT_EQ(policy, SCHED_FIFO);
    sched_param sp{};
    ::sched_getparam(0, &sp);
    EXPECT_EQ(sp.sched_priority, 46);
    // A thread created here does not silently become a second real-time thread.
    std::thread([] { EXPECT_EQ(::sched_getscheduler(0) & ~SCHED_RESET_ON_FORK, SCHED_OTHER); }).join();
  }).join();
}

TEST(ThreadClass, NormalAndBackgroundLeaveAnInheritedRealTimeClass) {
  RtEnv env("1");
  std::thread([] {
    sched_param sp{};
    sp.sched_priority = 10;
    // Inherit-by-hand: plain SCHED_FIFO without reset-on-fork, as a thread created before its
    // creator's class was set would have it. Needs the grant; without it there is nothing to leave.
    if (::sched_setscheduler(0, SCHED_FIFO, &sp) != 0) GTEST_SKIP() << "no real-time grant here";
    std::thread([] {
      EXPECT_EQ(::sched_getscheduler(0), SCHED_FIFO);
      ota::apply_thread_class("vision-rx", ota::ThreadClass::Normal);
      EXPECT_EQ(::sched_getscheduler(0), SCHED_OTHER);
    }).join();
    std::thread([] {
      ota::apply_thread_class("imu-observer", ota::ThreadClass::Background);
      EXPECT_EQ(::sched_getscheduler(0), SCHED_OTHER);
      EXPECT_EQ(own_nice(), 10);
    }).join();
  }).join();
}
