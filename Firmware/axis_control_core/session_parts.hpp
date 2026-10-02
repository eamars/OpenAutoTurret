#pragma once
// Building blocks shared by the commissiond servo sessions and the simulator:
// the reference table, the identification sweep and the limit-cycle guard.
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace ota::axis {
// One ADR-003 reference sample: script time and {q, v, a} relative to the start.
struct ReferenceSample { double time, position, velocity, acceleration; };

// A time-ordered reference table, linearly interpolated; before the first sample
// it returns the first, after the last it returns the last (end_at_rest zeroes
// its velocity and acceleration). Shared by commissiond and the simulator.
class ReferenceTable {
 public:
  void add(const ReferenceSample& s) {
    if (!std::isfinite(s.time) || !std::isfinite(s.position) || !std::isfinite(s.velocity) ||
        !std::isfinite(s.acceleration) || (samples_.empty() ? s.time!=0. : s.time<=samples_.back().time))
      throw std::runtime_error("DATA_INVALID: reference table sample");
    samples_.push_back(s);
  }
  bool empty() const { return samples_.empty(); }
  std::size_t size() const { return samples_.size(); }
  double duration() const { return samples_.empty()?0.:samples_.back().time; }
  ReferenceSample at(double t,bool end_at_rest=false) const {
    const auto upper=std::upper_bound(samples_.begin(),samples_.end(),t,
      [](double time,const ReferenceSample& s) { return time<s.time; });
    if (upper==samples_.begin()) return samples_.front();
    if (upper==samples_.end()) {
      auto last=samples_.back(); if (end_at_rest) last.velocity=last.acceleration=0.; return last;
    }
    const auto& lower=*(upper-1); const double f=(t-lower.time)/(upper->time-lower.time);
    return {t,std::lerp(lower.position,upper->position,f),std::lerp(lower.velocity,upper->velocity,f),
            std::lerp(lower.acceleration,upper->acceleration,f)};
  }
 private:
  std::vector<ReferenceSample> samples_;
};

// Identification excitation: a log-swept sine from f0 to f1 over duration_s,
// starting begin_s into the script; zero outside that window.
struct LogSweep {
  double amplitude=0, f0=0, f1=0, begin_s=0, duration_s=0;
  bool valid(double max_amplitude) const {
    return amplitude>0 && amplitude<=max_amplitude && f0>0 && f1>f0 && f1<=200 && duration_s>0 && begin_s>=0;
  }
  double at(double script_time) const {
    const double elapsed=script_time-begin_s;
    if (amplitude<=0 || elapsed<0 || elapsed>=duration_s) return 0.;
    const double k=std::log(f1/f0)/duration_s;
    return amplitude*std::sin(2*std::numbers::pi*f0*(std::exp(k*elapsed)-1)/k);
  }
};

// Limit-cycle guard: RMS over ~0.25 s of the current's fast part (the current
// minus its 15 ms exponential average) held above the limit for sustain_s.
// Normal tracking stays below ~0.15 A; the crosstalk/inertia limit cycle reaches
// 0.3 A within ~0.2 s of onset and stays there. A stall-recovery rock is a single
// deliberate swing that can touch 0.3 A for a moment (station 2026-10-02): the caller
// marks it (and its settling) as deliberate. A short sustain absorbs single spikes;
// a long one let a real cycle grow into the speed trip (station, same day).
class OscillationMonitor {
 public:
  explicit OscillationMonitor(double limit=0.,double sustain_s=0.05) : limit_(limit), sustain_(sustain_s) {}
  // True once the RMS has stayed above the limit for sustain_s (never when the limit is 0).
  // `deliberate`: the servo is rocking a stall free (or just did); that swing is not counted.
  bool update(double dt,double current,bool deliberate=false) {
    if (!(dt>0)) return false;
    if (!started_) { average_=current; started_=true; }
    average_+=(current-average_)*(1-std::exp(-dt/0.015));
    if (deliberate) { above_=0.; return false; }
    const double fast=current-average_;
    mean_square_+=(fast*fast-mean_square_)*(1-std::exp(-dt/0.25));
    above_=limit_>0 && rms()>limit_?above_+dt:0.;
    return limit_>0 && above_>=sustain_;
  }
  double rms() const { return std::sqrt(mean_square_); }
 private:
  double limit_, sustain_, average_=0., mean_square_=0., above_=0.;
  bool started_=false;
};
}
