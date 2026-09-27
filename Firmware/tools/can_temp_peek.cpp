// Read-only SocketCAN peek at GM6020 status broadcasts (0x200+id group):
// prints the raw temperature byte. No frames are sent, ownership untouched.
// Deliberately stdlib-only: buildable anywhere with g++ and linux/can.h.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/ioctl.h>
#include <poll.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <unistd.h>

int main(int argc, char** argv) {
  const char* iface = argc > 1 ? argv[1] : "can0";
  const int ms = argc > 2 ? std::atoi(argv[2]) : 3000;
  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) { std::perror("socket"); return 1; }
  ifreq r{};
  std::strncpy(r.ifr_name, iface, IFNAMSIZ - 1);
  if (ioctl(s, SIOCGIFINDEX, &r) < 0) { std::perror("ioctl"); return 1; }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = r.ifr_ifindex;
  if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::perror("bind"); return 1;
  }
  can_filter f{0x200u, 0x7F0u};  // any 0x20x status frame
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &f, sizeof(f));
  pollfd p{ s, POLLIN, 0 };
  int seen = 0;
  while (poll(&p, 1, ms) > 0) {
    can_frame fr{};
    const ssize_t n = read(s, &fr, sizeof(fr));
    if (n != sizeof(can_frame) || fr.can_id > 0x207 || fr.len != 8) continue;
    const uint16_t angle = (uint16_t(fr.data[0]) << 8) | fr.data[1];
    const int16_t speed = int16_t((uint16_t(fr.data[2]) << 8) | fr.data[3]);
    const int16_t current = int16_t((uint16_t(fr.data[4]) << 8) | fr.data[5]);
    std::printf("id=0x%X angle=%u speed_rpm=%d current_raw=%d temp_raw=%u\n",
                fr.can_id, angle, speed, current, fr.data[6]);
    if (++seen >= 5) return 0;
  }
  std::printf("no GM6020 status frames within %d ms\n", ms);
  return seen > 0 ? 0 : 2;
}
