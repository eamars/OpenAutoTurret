#pragma once
// Minimal netlink(RTNETLINK) helpers for CAN interfaces (architecture §8.1).
//
// Linux does not expose an SIOCSIFCANBITRATE-style ioctl: bitrate changes and
// "bring up" go through RTM_NEWLINK (what `ip link set ... type can bitrate N
// up` does), and interface state/bitrate are read with RTM_GETLINK
// (what `ip -details link show can0` reads).
//
// These helpers are used only on the slow setup path (open / commissioning),
// never from the control loop.
#include <cstdint>
#include <string>

namespace ota::can {

// Operational state of a CAN interface (linux/can/netlink.h enum can_state).
enum class CanIfState : int {
  Unknown = -1,
  ErrorActive = 0,
  ErrorWarning = 1,
  ErrorPassive = 2,
  BusOff = 3,
  Stopped = 4,
  Sleeping = 5,
};

struct CanIfInfo {
  bool exists{false};
  int ifindex{0};
  bool up{false};          // IFF_UP
  CanIfState state{CanIfState::Unknown};
  uint32_t bitrate{0};     // 0 if the driver does not report one (e.g. vcan)
  bool is_can{false};      // IFLA_INFO_KIND == "can"
};

// Query one interface. `exists` stays false if the interface is not there.
// Never fails hard on "not found" — the caller decides what that means.
bool netlink_query_can(const std::string& iface, CanIfInfo& out, std::string& err);

// Bring a DOWN CAN interface up at the given bitrate (RTM_NEWLINK with
// IFLA_INFO_KIND="can" + IFLA_CAN_BITTIMING.bitrate). Fails if the interface
// is already UP (bitrate changes require down) or not a CAN interface.
bool netlink_can_set_bitrate_up(const std::string& iface, uint32_t bitrate,
                                std::string& err);

}  // namespace ota::can
