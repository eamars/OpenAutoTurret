#pragma once
// Per-thread scheduling classes (owner ruling 2026-10-03, docs/operations/os-setup.md).
//
// Real time is a property of a thread, never of a process: only the threads that close a motor loop
// or watch one run SCHED_FIFO. Tracking input stays at the normal class, and the web, IMU observer
// and log writer threads are niced down, so the controller's own housekeeping cannot delay a servo
// step either.
//
// The policy is opt-in: nothing changes unless the launcher sets OTA_RT=1 for controld. Tests and
// workstation runs therefore never create a SCHED_FIFO thread. Without the RLIMIT_RTPRIO grant from
// the OS setup, the request is refused, logged once per thread and the thread keeps SCHED_OTHER: a
// station that is not set up runs exactly as it did before, only slower to react.
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>

#include <spdlog/spdlog.h>

namespace ota {

enum class ThreadClass {
  Motor,       // closes or guards a motor loop: SCHED_FIFO at its own priority, 1 ns timer slack
  Normal,      // feeds the control loop (vision input): default class, nice 0
  Background,  // web, IMU observer, log writer: nice +10
};

// SCHED_FIFO priorities. The kernel's threaded IRQ handlers (the MCP2518FD CAN controllers on SPI,
// and the SPI message pumps) run at FIFO 50; every motor thread stays below them, because those are
// the threads that deliver this process its feedback and carry its commands to the bus.
namespace rt_priority {
inline constexpr int kYawRx = 48;       // can0 RX: GM6020 feedback, and the yaw servo steps on it
inline constexpr int kPitchRx = 47;     // can1 RX: CyberGear feedback
inline constexpr int kPitchServo = 46;  // the pitch servo's 1 kHz host loop
inline constexpr int kGuard = 45;       // yaw guard and the CyberGear watchdog
inline constexpr int kControl = 44;     // the 200 Hz control loop
}  // namespace rt_priority

// A thread created by a real-time thread may have inherited its class; dropping back to SCHED_OTHER
// needs no privilege.
inline void leave_realtime() {
  if (::sched_getscheduler(0) == SCHED_OTHER) return;
  sched_param sp{};
  ::sched_setscheduler(0, SCHED_OTHER, &sp);
}

inline bool realtime_policy_enabled() {
  const char* v = std::getenv("OTA_RT");
  return v != nullptr && std::strcmp(v, "1") == 0;
}

// Names the calling thread (always; it is how `ps -T` and top tell the threads apart) and, when the
// launcher asked for the policy, applies its class. Returns true when the class is in force.
inline bool apply_thread_class(const char* name, ThreadClass cls, int fifo_priority = 0) {
  pthread_setname_np(pthread_self(), name);  // truncated by the kernel to 15 characters
  if (!realtime_policy_enabled()) return false;
  const pid_t tid = static_cast<pid_t>(::syscall(SYS_gettid));
  switch (cls) {
    case ThreadClass::Motor: {
      // Without SCHED_FIFO a sleeping thread wakes up to 50 us late by default (timer slack); a
      // motor thread asks for exact wake-ups either way.
      ::prctl(PR_SET_TIMERSLACK, 1UL, 0, 0, 0);
      sched_param sp{};
      sp.sched_priority = fifo_priority;
      // SCHED_RESET_ON_FORK: a thread this one creates starts in the normal class, not as a second
      // real-time thread nobody chose (sched_setscheduler with pid 0 acts on the calling thread).
      if (::sched_setscheduler(0, SCHED_FIFO | SCHED_RESET_ON_FORK, &sp) == 0) {
        spdlog::info("thread {} (tid {}): SCHED_FIFO {}", name, tid, fifo_priority);
        return true;
      }
      const int e = errno;
      rlimit rl{};
      ::getrlimit(RLIMIT_RTPRIO, &rl);
      spdlog::warn("thread {} (tid {}): SCHED_FIFO {} refused ({}; RLIMIT_RTPRIO={}); stays "
                   "SCHED_OTHER. The station's OS setup grants it: docs/operations/os-setup.md",
                   name, tid, fifo_priority, std::strerror(e), static_cast<long>(rl.rlim_cur));
      return false;
    }
    case ThreadClass::Normal:
      leave_realtime();
      return true;
    case ThreadClass::Background:
      leave_realtime();
      // Raising one's own niceness needs no privilege; it is per thread on Linux.
      if (::setpriority(PRIO_PROCESS, static_cast<id_t>(tid), 10) == 0) return true;
      spdlog::warn("thread {} (tid {}): nice 10 refused ({})", name, tid, std::strerror(errno));
      return false;
  }
  return false;
}

// Keeps controld's pages resident so a motor thread never waits on a page fault. MCL_ONFAULT locks a
// page when it is first touched, so the eight-megabyte thread stacks are not pinned whole.
inline void lock_process_memory() {
  if (!realtime_policy_enabled()) return;
  if (::mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT) == 0) {
    spdlog::info("memory locked (mlockall, on fault)");
    return;
  }
  const int e = errno;
  rlimit rl{};
  ::getrlimit(RLIMIT_MEMLOCK, &rl);
  spdlog::warn("mlockall refused ({}; RLIMIT_MEMLOCK={} bytes); pages stay swappable. The station's "
               "OS setup grants it: docs/operations/os-setup.md",
               std::strerror(e), static_cast<long long>(rl.rlim_cur));
}

}  // namespace ota
