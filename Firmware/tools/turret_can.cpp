// turret-can — commissioning/probe CLI for the CyberGear bus.
//
// Phase-1 deliverable tool (PROGRESS.md): discover both motors, watch
// high-rate feedback, and exercise each axis independently. It is a
// developer instrument, not the control path: it may only be used when
// controld is not running (single-owner-of-motors rule, architecture §4.1).
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "can/cybergear_system.hpp"
#include "can/cybergear_protocol.hpp"
#include "common/time.hpp"

using namespace ota;
using namespace ota::can;
namespace cg = ota::cybergear;

namespace {

volatile std::sig_atomic_t g_stop_flag = 0;
void on_signal(int) { g_stop_flag = 1; }

void print_usage(const char* prog) {
  std::printf(
      "usage: %s [options] <command> [args]\n"
      "\n"
      "options:\n"
      "  --iface NAME        CAN interface (default can0)\n"
      "  --bitrate N         bitrate, only applied if iface is DOWN\n"
      "  --bring-up          bring a DOWN interface up at the given bitrate\n"
      "  --pitch-id N        pitch motor CAN id (default 100)\n"
      "  --yaw-id N          yaw motor CAN id (default 101)\n"
      "  --timeout MS        discover/read timeout (default 500)\n"
      "\n"
      "commands:\n"
      "  discover                       discover both motors, print unique ids\n"
      "  feedback [SECS]                live feedback table (default 5 s)\n"
      "  stats                          bus stats\n"
      "  enable   [pitch|yaw|both]      issue enable command\n"
      "  stop     [pitch|yaw|both]      issue stop command\n"
      "  setzero  [pitch|yaw|both]      set mechanical zero (convenience)\n"
      "  read     <axis> <REG>          read register (name or 0xNNNN)\n"
      "  write    <axis> <REG> <VAL>    write register (RunMode: 0..3, else float)\n"
      "  jog      <axis> <DEG_S> <SECS> position-mode move of (DEG_S*SECS) degrees\n"
      "  pos      <axis> <DEG> [MS]     position-mode move (RunMode=1, LocRef)\n",
      prog);
}

AxisId parse_axis(const std::string& s) {
  if (s == "pitch") return AxisId::Pitch;
  if (s == "yaw") return AxisId::Yaw;
  std::fprintf(stderr, "bad axis '%s' (want pitch|yaw)\n", s.c_str());
  std::exit(2);
}

std::vector<AxisId> parse_axes(const std::string& s) {
  if (s == "both") return {AxisId::Pitch, AxisId::Yaw};
  return {parse_axis(s)};
}

cg::Reg parse_reg(const std::string& s, bool& ok) {
  ok = true;
  if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
    const unsigned addr = std::stoul(s, nullptr, 16);
    if (auto* info = cg::reg_info(static_cast<cg::Reg>(addr))) {
      return info->reg;
    }
    ok = false;
    return cg::Reg::RunMode;
  }
  for (auto r : {cg::Reg::RunMode, cg::Reg::IqRef, cg::Reg::SpdRef,
                 cg::Reg::LimitTorque, cg::Reg::CurKp, cg::Reg::CurKi,
                 cg::Reg::CurFiltGain, cg::Reg::LocRef, cg::Reg::LimitSpd,
                 cg::Reg::LimitCur, cg::Reg::MechPos, cg::Reg::Iqf,
                 cg::Reg::MechVel, cg::Reg::VBus, cg::Reg::Rotation,
                 cg::Reg::LocKp, cg::Reg::SpdKp, cg::Reg::SpdKi}) {
    if (s == cg::reg_name(r)) return r;
  }
  ok = false;
  return cg::Reg::RunMode;
}

float deg2rad(double d) { return static_cast<float>(d * M_PI / 180.0); }
double rad2deg(float r) { return static_cast<double>(r) * 180.0 / M_PI; }

void print_feedback_row(const CyberGearSystem& sys, AxisId a) {
  AxisLatest l;
  if (!sys.axis(a).latest(l) || !l.has_feedback) {
    std::printf("%-6s no feedback yet\n", axis_name(a));
    return;
  }
  const TimeNs age = now_monotonic_ns() - l.rx_ns;
  std::printf(
      "%-6s mode=%d q=%9.3f deg  v=%9.3f deg/s  tq=%7.3f N.m  T=%6.1f C  "
      "faults=0x%04x  age=%5.1f ms\n",
      axis_name(a), static_cast<int>(l.mode), rad2deg(l.q_rad),
      rad2deg(l.v_rad_s), l.torque_nm, l.temp_c, l.faults,
      static_cast<double>(age) / 1e6);
}

// Send a MIT (COMM_TYPE_1) hold frame: zero torque, fixed target position,
// zero velocity, small Kp/Kd. Two purposes at once (CyberGear_AI_Reference.md
// §25.1): it keeps the axis actively holding `target_rad`, and — because the
// CyberGear does NOT free-run COMM_TYPE_2 feedback — every such command is
// answered by one feedback frame, which is what the display below shows.
void send_mit_hold(CyberGearSystem& sys, AxisId a, float target_rad,
                   float kp = 80.0f, float kd = 2.0f) {
  cg::CanFrame f =
      cg::make_mit_command(0.0f, target_rad, 0.0f, kp, kd, sys.motor_id(a));
  std::string e;
  if (!sys.send(f.id, f.data, &e)) {
    std::fprintf(stderr, "MIT hold %s: %s\n", axis_name(a), e.c_str());
  }
}

int cmd_feedback(CyberGearSystem& sys, int secs) {
  std::printf(
      "Driving §25.1 MIT active-hold to elicit COMM_TYPE_2 feedback "
      "(the CyberGear does not free-run feedback). Axes are left holding "
      "their start position (run_mode=0) when this finishes.\n");
  // Setup: enable, read the current position (safe register read), switch to
  // motion-control (MIT) mode, send an initial hold. The target is fixed at
  // the start position so the hold resists any sag.
  float target[2] = {0.0f, 0.0f};
  for (AxisId a : {AxisId::Pitch, AxisId::Yaw}) {
    std::string e;
    sys.send_enable(a, &e);
    double pos = 0.0;
    if (!sys.read_register(a, cg::Reg::MechPos, pos, 500, &e)) {
      std::fprintf(stderr, "%s: read mechPos failed: %s\n", axis_name(a),
                   e.c_str());
      return 1;
    }
    target[static_cast<int>(a)] = static_cast<float>(pos);
    cg::CanFrame m = cg::make_write_reg_u8(
        cg::Reg::RunMode, static_cast<uint8_t>(cg::RunMode::MotionControl),
        sys.host_id(), sys.motor_id(a));
    sys.send(m.id, m.data, &e);
    send_mit_hold(sys, a, target[static_cast<int>(a)]);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::printf("time+s  "
              "pitch: mode q(deg) v(deg/s) tq(N.m) T(C) faults | yaw: same\n");
  const TimeNs t0 = now_monotonic_ns();
  TimeNs last_send = 0;
  while (!g_stop_flag) {
    const TimeNs now = now_monotonic_ns();
    if (now - t0 > ms_to_ns(static_cast<double>(secs) * 1000.0)) break;
    if (now - last_send >= ms_to_ns(50.0)) {
      last_send = now;  // ~20 Hz hold pace keeps feedback age < ~50 ms
      send_mit_hold(sys, AxisId::Pitch, target[0]);
      send_mit_hold(sys, AxisId::Yaw, target[1]);
    }
    std::printf("t=%6.2fs  ", static_cast<double>(now - t0) / 1e9);
    for (AxisId a : {AxisId::Pitch, AxisId::Yaw}) {
      AxisLatest l;
      if (sys.axis(a).latest(l) && l.has_feedback) {
        const TimeNs age = now - l.rx_ns;
        std::printf("mode=%d q=%9.3f v=%9.3f tq=%7.3f T=%6.1f f=0x%04x age=%5.1fms | ",
                    static_cast<int>(l.mode), rad2deg(l.q_rad), rad2deg(l.v_rad_s),
                    l.torque_nm, l.temp_c, l.faults,
                    static_cast<double>(age) / 1e6);
      } else {
        std::printf("%-95s | ", "no feedback");
      }
    }
    std::printf("\n");
    std::fflush(stdout);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::fprintf(stderr,
               "note: axes left in MIT hold mode (run_mode=0). Use 'stop' or "
               "'jog'/'pos' to change mode.\n");
  return 0;
}

// Safe position-mode jog. A speed-mode jog (run_mode=2 + SpdRef) was found not
// to drive the loaded axis at the commanded rate (measured ~1% of SpdRef with
// default gains), so this moves to a TARGET instead, which the firmware drives
// reliably (verified: it swept the full −37° to loc_ref on first contact).
// Follows MODE_SWITCH_SAFETY ("stop before switching control mode") and §25.4:
//   stop -> position mode -> enable -> limit_spd (safe cap) -> read current ->
//   loc_ref = current + delta -> wait for arrival -> hold.
// Entering position mode WITHOUT first pinning loc_ref drives the motor to a
// stale target (this is how an earlier revision moved the axis unexpectedly),
// so the target is always set from the freshly-read current position.
// `deg_s` × `secs` is interpreted as the total move (degrees).
int cmd_jog(CyberGearSystem& sys, AxisId a, double deg_s, double secs, int timeout_ms) {
  (void)timeout_ms;
  std::string e;
  const double delta = deg2rad(deg_s * secs);
  auto send_reg = [&](cg::CanFrame f) {
    if (!sys.send(f.id, f.data, &e))
      std::fprintf(stderr, "TX failed (%s): %s\n", axis_name(a), e.c_str());
  };
  auto sleep_ms = [](int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); };
  // --- Enter position mode (§25.4): stop -> run_mode=1 -> enable -> limit_spd.
  sys.send_stop(a, &e);
  sleep_ms(50);
  send_reg(cg::make_write_reg_u8(cg::Reg::RunMode, 1, sys.host_id(), sys.motor_id(a)));
  sys.send_enable(a, &e);
  sleep_ms(50);
  send_reg(cg::make_write_reg_float(cg::Reg::LimitSpd, 2.0f, sys.host_id(), sys.motor_id(a)));
  // --- Read the current position, then set the target = current + delta.
  double cur = 0.0;
  if (!sys.read_register(a, cg::Reg::MechPos, cur, 500, &e)) {
    std::fprintf(stderr, "could not read position (%s)\n", e.c_str());
    return 1;
  }
  const double target = cur + delta;
  send_reg(cg::make_write_reg_float(cg::Reg::LocRef, static_cast<float>(target),
                                    sys.host_id(), sys.motor_id(a)));
  // --- Wait for arrival (poll mechPos). The loaded axis moves slowly
  // (measured ~0.002 rad/s in position mode), so allow up to 30 s. The
  // arrival band scales with the move size (5%, floored just above the
  // 0.00038 rad position quantization) so a small jog is not declared
  // "arrived" before it has actually travelled.
  const double band = 0.0005 + 0.05 * std::fabs(delta);
  const TimeNs t0 = now_monotonic_ns();
  bool arrived = false;
  while (!g_stop_flag && now_monotonic_ns() - t0 < ms_to_ns(30000.0)) {
    double p = 0.0;
    if (sys.read_register(a, cg::Reg::MechPos, p, 300, &e)) {
      std::printf("t=%5.2fs  q=%9.4f rad  target=%9.4f rad  err=%+.4f\n",
                  static_cast<double>(now_monotonic_ns() - t0) / 1e9, p, target,
                  p - target);
      if (std::fabs(p - target) < band) { arrived = true; break; }
    }
    sleep_ms(100);
  }
  std::printf("jog %s: %s, %.4f -> %.4f rad (%+.2f deg)\n", axis_name(a),
              arrived ? "arrived" : "TIMEOUT", cur, target,
              rad2deg(static_cast<float>(delta)));
  return arrived ? 0 : 1;
}

int cmd_pos(CyberGearSystem& sys, AxisId a, double deg, int hold_ms, int timeout_ms) {
  auto f = cg::make_write_reg_u8(cg::Reg::RunMode, 1, sys.host_id(), sys.motor_id(a));
  std::string e;
  if (!sys.send(f.id, f.data, &e)) { std::fprintf(stderr, "write RunMode: %s\n", e.c_str()); return 1; }

  const TimeNs t0 = now_monotonic_ns();
  const TimeNs dur = ms_to_ns(hold_ms);
  bool sent = false;
  while (!g_stop_flag && (hold_ms < 0 || now_monotonic_ns() - t0 < dur)) {
    if (!sent) {
      f = cg::make_write_reg_float(cg::Reg::LocRef, deg2rad(deg), sys.host_id(), sys.motor_id(a));
      if (!sys.send(f.id, f.data, &e)) { std::fprintf(stderr, "write LocRef: %s\n", e.c_str()); return 1; }
      sent = true;
    }
    print_feedback_row(sys, a);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  print_feedback_row(sys, a);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  CyberGearSystemConfig cfg;
  int timeout_ms = 500;
  int i = 1;
  for (; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--iface" && i + 1 < argc) cfg.iface = argv[++i];
    else if (a == "--bitrate" && i + 1 < argc) cfg.bitrate = std::stoul(argv[++i]);
    else if (a == "--bring-up") cfg.bring_up_if_down = true;
    else if (a == "--pitch-id" && i + 1 < argc) cfg.pitch_motor_id = static_cast<uint8_t>(std::stoul(argv[++i]));
    else if (a == "--yaw-id" && i + 1 < argc) cfg.yaw_motor_id = static_cast<uint8_t>(std::stoul(argv[++i]));
    else if (a == "--timeout" && i + 1 < argc) timeout_ms = std::stoi(argv[++i]);
    else if (a == "-h" || a == "--help") { print_usage(argv[0]); return 0; }
    else break;
  }
  if (i >= argc) {
    print_usage(argv[0]);
    return 2;
  }
  const std::string cmd = argv[i++];

  std::string err;
  CyberGearSystem sys;
  if (!sys.open(cfg, err)) {
    std::fprintf(stderr, "open: %s\n", err.c_str());
    return 1;
  }

  int rc = 0;
  if (cmd == "discover") {
    for (AxisId a : {AxisId::Pitch, AxisId::Yaw}) {
      uint64_t uid = 0;
      if (sys.discover(a, uid, timeout_ms, &err)) {
        std::printf("%-6s id=%u unique_id=0x%016llx\n", axis_name(a),
                    sys.motor_id(a), static_cast<unsigned long long>(uid));
      } else {
        std::fprintf(stderr, "%-6s %s\n", axis_name(a), err.c_str());
        rc = 1;
      }
    }
  } else if (cmd == "feedback") {
    const int secs = i < argc ? std::stoi(argv[i++]) : 5;
    rc = cmd_feedback(sys, secs);
  } else if (cmd == "stats") {
    const auto s = sys.bus().stats();
    std::printf("iface=%s rx=%llu err_rx=%llu tx=%llu tx_fail=%llu last_rx_age=%.1f ms\n",
                sys.bus().device().c_str(),
                static_cast<unsigned long long>(s.rx_frames),
                static_cast<unsigned long long>(s.rx_error_frames),
                static_cast<unsigned long long>(s.tx_frames),
                static_cast<unsigned long long>(s.tx_failed),
                s.last_rx_ns == 0 ? -1.0
                                  : static_cast<double>(now_monotonic_ns() - s.last_rx_ns) / 1e6);
  } else if (cmd == "enable" || cmd == "stop" || cmd == "setzero") {
    const std::string which = i < argc ? argv[i++] : "both";
    for (AxisId a : parse_axes(which)) {
      std::string e;
      bool ok = cmd == "enable"   ? sys.send_enable(a, &e)
                : cmd == "stop"   ? sys.send_stop(a, &e)
                                  : sys.send_set_zero(a, &e);
      if (!ok) {
        std::fprintf(stderr, "%s %s: %s\n", cmd.c_str(), axis_name(a), e.c_str());
        rc = 1;
      } else {
        std::printf("%s %s ok\n", cmd.c_str(), axis_name(a));
      }
    }
  } else if (cmd == "read") {
    if (i + 1 >= argc) { print_usage(argv[0]); return 2; }
    AxisId a = parse_axis(argv[i++]);
    bool ok = false;
    cg::Reg r = parse_reg(argv[i++], ok);
    if (!ok) { std::fprintf(stderr, "unknown register\n"); return 2; }
    double v = 0.0;
    if (!sys.read_register(a, r, v, timeout_ms, &err)) {
      std::fprintf(stderr, "%s %s: %s\n", axis_name(a), cg::reg_name(r), err.c_str());
      return 1;
    }
    std::printf("%-6s %-14s = %g\n", axis_name(a), cg::reg_name(r), v);
  } else if (cmd == "write") {
    if (i + 2 >= argc) { print_usage(argv[0]); return 2; }
    AxisId a = parse_axis(argv[i++]);
    bool ok = false;
    cg::Reg r = parse_reg(argv[i++], ok);
    if (!ok) { std::fprintf(stderr, "unknown register\n"); return 2; }
    const double v = std::stod(argv[i++]);
    cg::CanFrame f;
    if (r == cg::Reg::RunMode) {
      f = cg::make_write_reg_u8(r, static_cast<uint8_t>(v), sys.host_id(), sys.motor_id(a));
    } else if (r == cg::Reg::Rotation) {
      std::fprintf(stderr, "rotation register is not supported by this CLI\n");
      return 2;
    } else {
      f = cg::make_write_reg_float(r, static_cast<float>(v), sys.host_id(), sys.motor_id(a));
    }
    if (!sys.send(f.id, f.data, &err)) {
      std::fprintf(stderr, "write %s: %s\n", cg::reg_name(r), err.c_str());
      return 1;
    }
    std::printf("wrote %s %s = %g\n", axis_name(a), cg::reg_name(r), v);
  } else if (cmd == "jog") {
    if (i + 2 >= argc) { print_usage(argv[0]); return 2; }
    AxisId a = parse_axis(argv[i++]);
    const double deg_s = std::stod(argv[i++]);
    const double secs = std::stod(argv[i++]);
    rc = cmd_jog(sys, a, deg_s, secs, timeout_ms);
  } else if (cmd == "pos") {
    if (i + 1 >= argc) { print_usage(argv[0]); return 2; }
    AxisId a = parse_axis(argv[i++]);
    const double deg = std::stod(argv[i++]);
    const int hold_ms = i < argc ? std::stoi(argv[i++]) : 2000;
    rc = cmd_pos(sys, a, deg, hold_ms, timeout_ms);
  } else {
    std::fprintf(stderr, "unknown command '%s'\n", cmd.c_str());
    print_usage(argv[0]);
    return 2;
  }

  sys.close();
  return rc;
}
