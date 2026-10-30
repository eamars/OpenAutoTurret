#pragma once
// ADR-007: the one-shot actuator, a hobby servo on an RP1 PWM0 pin (GPIO15 on this station, owner
// 2026-10-30). It rests at A; ACTUATE moves it to B, waits the dwell, and brings it back to A.
//
// controld owns the pin (owner, 2026-10-30), but the control loop never writes it: every request is
// accepted or refused at once on the caller's thread, and Actuator's own Background thread writes the
// PWM and keeps the shot's time. The 200 Hz loop takes the mutex only to post a request or copy the
// status, and the worker never holds it across a sysfs write.
//
// What the servo does (ADR-007 D4): at start it goes to A once, from wherever it was (a crash may have
// left it at B); between shots it is limp at A, or powered there with `hold_at_rest`; STOP MOTION
// ends a shot at A at once; the process's stop drives A, lets it travel, and switches the PWM off.
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace ota {

// The bounds every pulse is checked against, wherever it comes from (the file, the page, a test).
inline constexpr int kActuatorMinPulseUs = 500;
inline constexpr int kActuatorMaxPulseUs = 2500;
inline constexpr double kActuatorMaxDwellMs = 5000.0;

// run/state/actuator.json, schema 1 (written first by tools/actuator_calibrate.py).
struct ActuatorConfig {
  bool valid = false;
  int gpio = 15;
  int period_us = 20000;
  int rest_us = 0;     // A
  int active_us = 0;   // B
  double dwell_ms = 0;
  bool hold_at_rest = false;  // false (the default, owner 2026-10-30): limp at A between shots
  std::string saved;          // when, as the writer put it (ISO 8601, local)
};

std::string actuator_config_to_json(const ActuatorConfig& c);
// The whole record or nothing: false (and `err`) for anything that is not a schema-1 record with
// pulses in 500..2500 us, a dwell in 0..5000 ms, and a 50 Hz-class period (2.5..50 ms).
bool actuator_config_from_json(const std::string& text, ActuatorConfig& out, std::string& err);
// A missing file is not an error (returns false, err empty): the actuator is unconfigured.
bool load_actuator_config(const std::string& path, ActuatorConfig& out, std::string& err);
// Temp file beside `path`, renamed over it. Blocking; the worker calls it.
bool save_actuator_config(const std::string& path, const ActuatorConfig& c, std::string& err);
// "" when (rest, active, dwell) may be used; otherwise why not.
std::string actuator_values_refusal(int rest_us, int active_us, double dwell_ms);

// One PWM output. set() switches it on at `pulse_us` or, with enabled=false, off.
class PwmOutput {
 public:
  virtual ~PwmOutput() = default;
  virtual bool set(int pulse_us, bool enabled, std::string& err) = 0;
};

// RP1 PWM0 through sysfs. The chip is found by its address (1f00098000), never by its index, which
// the kernel does not promise. The launcher muxes the pin (pinctrl); this only drives the channel.
class SysfsPwm : public PwmOutput {
 public:
  // nullptr (and `err`) when the GPIO has no PWM0 channel, no chip has the address (the overlay is not
  // loaded), or the channel cannot be exported or written. `class_root` is /sys/class/pwm on a station.
  static std::unique_ptr<SysfsPwm> open(const std::string& class_root, int gpio, int period_us, std::string& err);
  bool set(int pulse_us, bool enabled, std::string& err) override;
  const std::string& channel_dir() const { return dir_; }

 private:
  SysfsPwm(std::string dir, bool enabled) : dir_(std::move(dir)), enabled_(enabled) {}
  std::string dir_;
  bool enabled_ = false;
};
// GPIO -> PWM0 channel (0..3), or -1. From `pinctrl funcs` on the station.
int pwm0_channel_for_gpio(int gpio);

class Actuator {
 public:
  struct Options {
    int settle_ms = 500;  // the travel allowed to reach A before the servo goes limp
  };
  // `pwm` null: unavailable, with `unavailable_reason`. `path` "": nothing is saved across restarts.
  // `config` is what was loaded from `path` (valid=false: unconfigured, `config_problem` says why if
  // the file was there but refused).
  Actuator(std::unique_ptr<PwmOutput> pwm, std::string unavailable_reason, std::string path, int gpio,
           ActuatorConfig config, std::string config_problem, Options opt);
  Actuator(std::unique_ptr<PwmOutput> pwm, std::string unavailable_reason, std::string path, int gpio,
           ActuatorConfig config, std::string config_problem)
      : Actuator(std::move(pwm), std::move(unavailable_reason), std::move(path), gpio, std::move(config),
                 std::move(config_problem), Options{}) {}
  ~Actuator();
  Actuator(const Actuator&) = delete;
  Actuator& operator=(const Actuator&) = delete;

  // Each returns "" when accepted, otherwise the reason. None blocks on the PWM.
  std::string fire();
  void abort();  // STOP MOTION: a shot in progress returns to A now; nothing else changes
  std::string begin_calibration(std::optional<int> start_us);
  std::string set_pulse(int us);
  std::string test(int rest_us, int active_us, double dwell_ms);
  std::string save(int rest_us, int active_us, double dwell_ms);
  std::string end_calibration();
  void request_shutdown();  // non-blocking: A, settle, PWM off, then the worker ends
  void shutdown();          // request_shutdown() and wait for it

  // The page's object (ADR-007 D7), rebuilt on every change; never null.
  std::shared_ptr<const std::string> status_json() const;
  std::string state_name() const;

 private:
  enum class Mode { Unavailable, Unconfigured, Settling, Idle, Active, Calibrating, Stopped };
  using Clock = std::chrono::steady_clock;
  void run();
  void publish_locked();
  const char* mode_name_locked() const;

  std::unique_ptr<PwmOutput> pwm_;
  const std::string path_;
  const int gpio_;
  const Options opt_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  Mode mode_ = Mode::Unavailable;
  std::string reason_;            // why unavailable or unconfigured
  ActuatorConfig cfg_;            // in force
  int pulse_us_ = 0;              // wanted output
  bool powered_ = false;
  int shot_rest_us_ = 0;          // the shot's A (a calibration test may use unsaved values)
  bool back_to_calibration_ = false;
  Clock::time_point until_{};     // end of the dwell (Active) or of the travel (Settling)
  bool stopping_ = false;
  std::optional<ActuatorConfig> pending_save_;
  int saved_ = -1;                // 1: the last save reached the disk; 0: it failed; -1: none yet
  int64_t shots_ = 0;
  std::shared_ptr<const std::string> status_;
  std::thread thread_;
};

}  // namespace ota
