#include "can/can_netlink.hpp"

#include <cerrno>
#include <cstring>
#include <cstdio>
#include <net/if.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <poll.h>

#include <chrono>

#include <linux/can/netlink.h>
#include <linux/if_arp.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>

#include <cinttypes>

namespace ota::can {

namespace {

constexpr std::size_t kRxSize = 8192;
// Netlink is a setup-path control channel; a hung reply must never wedge the
// caller, so every receive is bounded by this deadline.
constexpr int kNetlinkTimeoutMs = 2000;

int open_netlink(std::string& err) {
  const int fd = ::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
  if (fd < 0) {
    err = std::string("socket(AF_NETLINK): ") + std::strerror(errno);
  }
  return fd;
}

void put_attr(char* at, std::size_t len, int type, const void* data) {
  auto* a = reinterpret_cast<struct rtattr*>(at);
  a->rta_len = static_cast<unsigned short>(len);
  a->rta_type = static_cast<unsigned short>(type);
  if (data) std::memcpy(RTA_DATA(a), data, len - sizeof(struct rtattr));
}

// Receive one netlink datagram, bounded by a timeout. Returns the number of
// bytes read, 0 on timeout, or -1 on transport error (sets err).
int netlink_recv_one(int fd, uint8_t* buf, std::size_t cap, int timeout_ms,
                     std::string& err) {
  struct pollfd pf{};
  pf.fd = fd;
  pf.events = POLLIN;
  const int pr = ::poll(&pf, 1, timeout_ms);
  if (pr < 0) {
    if (errno == EINTR) return 0;
    err = std::string("netlink poll: ") + std::strerror(errno);
    return -1;
  }
  if (pr == 0) return 0;  // timed out with no datagram
  const ssize_t n = ::recv(fd, buf, cap, 0);
  if (n < 0) {
    if (errno == EINTR) return 0;
    err = std::string("netlink recv: ") + std::strerror(errno);
    return -1;
  }
  return static_cast<int>(n);
}

// Wait for the kernel's acknowledgement of a request issued with NLM_F_ACK.
// The kernel answers NLMSG_ERROR (error == 0 on success, -errno on failure)
// and/or NLMSG_DONE. Bounded by a timeout so a missing ack can never wedge
// the caller. (Without NLM_F_ACK the kernel sends no success reply at all,
// which is why the request must set that flag.)
bool netlink_recv_ack(int fd, std::string& err) {
  uint8_t buf[kRxSize];
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(kNetlinkTimeoutMs);
  for (;;) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      err = "netlink ack timeout";
      return false;
    }
    const int remain = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
            .count());
    const int n = netlink_recv_one(fd, buf, sizeof(buf), remain, err);
    if (n < 0) return false;
    if (n == 0) continue;  // no datagram yet; recompute and retry
    if (static_cast<std::size_t>(n) < NLMSG_HDRLEN) {
      err = "short netlink message";
      return false;
    }
    std::size_t len = static_cast<std::size_t>(n);
    for (auto* h = reinterpret_cast<struct nlmsghdr*>(buf);
         NLMSG_OK(h, len); h = NLMSG_NEXT(h, len)) {
      if (h->nlmsg_type == NLMSG_DONE) return true;
      if (h->nlmsg_type == NLMSG_ERROR) {
        auto* e = reinterpret_cast<struct nlmsgerr*>(NLMSG_DATA(h));
        if (e->error != 0) {
          err = "netlink error " + std::to_string(e->error) + ": " +
                std::strerror(-e->error);
          return false;
        }
        return true;  // error == 0: success ack
      }
    }
  }
}

}  // namespace

bool netlink_query_can(const std::string& iface, CanIfInfo& out,
                       std::string& err) {
  out = CanIfInfo{};
  const int ifindex = ::if_nametoindex(iface.c_str());
  if (ifindex == 0) {
    if (errno != 0) err = std::string("if_nametoindex: ") + std::strerror(errno);
    out.exists = false;
    return true;  // "not found" is a valid answer, not a transport error
  }
  out.exists = true;
  out.ifindex = ifindex;

  err.clear();
  const int fd = open_netlink(err);
  if (fd < 0) return false;

  alignas(uint64_t) uint8_t req[NLMSG_HDRLEN + sizeof(struct ifinfomsg)] = {};
  auto* nh = reinterpret_cast<struct nlmsghdr*>(req);
  nh->nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
  nh->nlmsg_type = RTM_GETLINK;
  nh->nlmsg_flags = NLM_F_REQUEST;
  nh->nlmsg_seq = 1;
  auto* ifi = reinterpret_cast<struct ifinfomsg*>(NLMSG_DATA(nh));
  ifi->ifi_family = AF_UNSPEC;
  ifi->ifi_index = ifindex;

  if (::sendto(fd, req, nh->nlmsg_len, 0, nullptr, 0) < 0) {
    err = std::string("netlink send: ") + std::strerror(errno);
    ::close(fd);
    return false;
  }

  // Scan the reply for our interface. The kernel answers with an RTM_NEWLINK
  // message for the requested index. Note: some kernel versions do NOT emit
  // the trailing NLMSG_DONE for a single-interface GETLINK, so completion is
  // defined as "we received the interface's NEWLINK", not "we saw a
  // terminator". The receive is still deadline-bounded so a silent kernel can
  // never wedge the caller.
  bool found = false;
  {
    uint8_t buf[kRxSize];
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kNetlinkTimeoutMs);
    for (;;) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        err = "netlink query timeout";
        break;
      }
      const int remain = static_cast<int>(
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
              .count());
      const int n = netlink_recv_one(fd, buf, sizeof(buf), remain, err);
      if (n < 0) break;
      if (n == 0) continue;  // no datagram yet
      if (static_cast<std::size_t>(n) < NLMSG_HDRLEN) break;
      std::size_t len = static_cast<std::size_t>(n);
      bool terminator = false;
      for (auto* h = reinterpret_cast<struct nlmsghdr*>(buf);
           NLMSG_OK(h, len); h = NLMSG_NEXT(h, len)) {
        if (h->nlmsg_type == NLMSG_DONE || h->nlmsg_type == NLMSG_ERROR) {
          terminator = true;
          break;
        }
        if (h->nlmsg_type != RTM_NEWLINK) continue;
        auto* i = reinterpret_cast<struct ifinfomsg*>(NLMSG_DATA(h));
        if (i->ifi_index != static_cast<unsigned short>(ifindex)) continue;
        found = true;
        out.up = (i->ifi_flags & IFF_UP) != 0;
        out.is_can = (i->ifi_type == ARPHRD_CAN);

        // Walk IFLA_* attributes.
        std::size_t rlen = NLMSG_PAYLOAD(h, sizeof(struct ifinfomsg));
        auto* r = reinterpret_cast<struct rtattr*>(
            reinterpret_cast<char*>(i) + NLMSG_ALIGN(sizeof(struct ifinfomsg)));
        for (; RTA_OK(r, rlen); r = RTA_NEXT(r, rlen)) {
          if (r->rta_type != IFLA_LINKINFO) continue;
          // Walk IFLA_INFO_* (kernel emits no IFLA_INFO_UNSPEC placeholder).
          std::size_t ilen = r->rta_len - sizeof(struct rtattr);
          auto* ia = reinterpret_cast<struct rtattr*>(RTA_DATA(r));
          for (; RTA_OK(ia, ilen); ia = RTA_NEXT(ia, ilen)) {
            if (ia->rta_type == IFLA_INFO_KIND) {
              out.is_can =
                  std::string(reinterpret_cast<char*>(RTA_DATA(ia))) == "can";
            } else if (ia->rta_type == IFLA_INFO_DATA) {
              // Walk IFLA_CAN_* data attributes.
              std::size_t dlen = ia->rta_len - sizeof(struct rtattr);
              auto* da = reinterpret_cast<struct rtattr*>(RTA_DATA(ia));
              for (; RTA_OK(da, dlen); da = RTA_NEXT(da, dlen)) {
                if (da->rta_type == IFLA_CAN_STATE) {
                  uint32_t st = 0;
                  std::memcpy(&st, RTA_DATA(da), sizeof(st));
                  out.state =
                      (st < CAN_STATE_MAX) ? static_cast<CanIfState>(st)
                                           : CanIfState::Unknown;
                } else if (da->rta_type == IFLA_CAN_BITTIMING &&
                           da->rta_len >= sizeof(struct rtattr) +
                                               sizeof(uint32_t)) {
                  // struct can_bittiming; bitrate is the first field.
                  uint32_t br = 0;
                  std::memcpy(&br, RTA_DATA(da), sizeof(br));
                  out.bitrate = br;
                }
              }
            }
          }
        }
      }
      if (found || terminator) break;
    }
  }
  ::close(fd);
  if (!found) {
    if (err.empty()) err = "interface " + iface + " not found in netlink reply";
    out.exists = false;
    return false;
  }
  return true;
}

bool netlink_can_set_bitrate_up(const std::string& iface, uint32_t bitrate,
                                std::string& err) {
  CanIfInfo info{};
  if (!netlink_query_can(iface, info, err)) return false;
  if (!info.exists) {
    err = "interface " + iface + " does not exist";
    return false;
  }
  if (!info.is_can) {
    err = "interface " + iface + " is not a CAN interface";
    return false;
  }
  if (info.up) {
    err = "interface " + iface +
          " is already up; bring it down before changing bitrate";
    return false;
  }

  const int fd = open_netlink(err);
  if (fd < 0) return false;

  // RTM_NEWLINK: IFF_UP + type can + IFLA_CAN_BITTIMING{bitrate}.
  // (Same message `ip link set <if> up type can bitrate N` generates.)
  // NLM_F_ACK requests a reply on both success and failure; without it the
  // kernel sends no success acknowledgement and the recv would time out.
  constexpr std::size_t kKindLen = sizeof(struct rtattr) + 4;  // "can\0"
  constexpr std::size_t kBtLen = sizeof(struct rtattr) + sizeof(struct can_bittiming);
  constexpr std::size_t kDataLen = sizeof(struct rtattr) + kBtLen;
  constexpr std::size_t kInfoLen = sizeof(struct rtattr) + kKindLen + kDataLen;
  constexpr std::size_t kBody =
      NLMSG_ALIGN(NLMSG_HDRLEN + sizeof(struct ifinfomsg));

  alignas(uint64_t) uint8_t buf[512] = {};
  auto* nh = reinterpret_cast<struct nlmsghdr*>(buf);
  nh->nlmsg_len = static_cast<unsigned short>(kBody + kInfoLen);
  nh->nlmsg_type = RTM_NEWLINK;
  nh->nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
  nh->nlmsg_seq = 2;
  auto* ifi = reinterpret_cast<struct ifinfomsg*>(buf + NLMSG_HDRLEN);
  ifi->ifi_family = AF_UNSPEC;
  ifi->ifi_type = ARPHRD_CAN;
  ifi->ifi_index = static_cast<unsigned short>(info.ifindex);
  ifi->ifi_flags = IFF_UP;
  ifi->ifi_change = IFF_UP;

  char* p = reinterpret_cast<char*>(buf + kBody);
  put_attr(p, kInfoLen, IFLA_LINKINFO, nullptr);
  p += sizeof(struct rtattr);
  put_attr(p, kKindLen, IFLA_INFO_KIND, "can");
  p += kKindLen;
  put_attr(p, kDataLen, IFLA_INFO_DATA, nullptr);
  p += sizeof(struct rtattr);
  struct can_bittiming bt{};
  bt.bitrate = bitrate;
  put_attr(p, kBtLen, IFLA_CAN_BITTIMING, &bt);

  if (::sendto(fd, buf, nh->nlmsg_len, 0, nullptr, 0) < 0) {
    err = std::string("netlink send: ") + std::strerror(errno);
    ::close(fd);
    return false;
  }
  const bool ok = netlink_recv_ack(fd, err);
  ::close(fd);
  return ok;
}

}  // namespace ota::can
