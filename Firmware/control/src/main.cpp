// controld — sole owner of can0 and the motors (architecture §4.1).
//
// NOTE: temporary boot skeleton. The full daemon (config load, boot state
// machine, 200 Hz control loop, homing, telemetry) is implemented in
// phases 2/3/6 — see PROGRESS.md. Kept as a placeholder so the build is
// green while the stack is assembled.
#include <cstdio>

int main() {
  std::printf("controld: skeleton (full daemon under construction)\n");
  return 0;
}
