// OpenAutoTurret — payload mismatch detection implementation (§31.3).
#include "payload/payload_verifier.hpp"

#include <algorithm>
#include <cmath>

namespace ota {
namespace payload {

namespace {

// The worst (largest) of the +/− step for one metric.
struct PairWorst {
  const StepMetrics& pos;
  const StepMetrics& neg;
  bool valid() const { return pos.valid || neg.valid; }
};

double worst(const PairWorst& p, double (*get)(const StepMetrics&)) {
  double v = get(p.pos);
  if (p.neg.valid) v = std::max(v, get(p.neg));
  else if (!p.pos.valid) v = 0.0;
  return v;
}

// Amplitude-normalized rise time (s/rad) — makes the comparison robust to
// different step amplitudes between profiling and verification.
double rise_norm(const StepMetrics& m) {
  return (m.valid && m.amplitude_rad > 1e-6)
             ? m.rise_time_s / m.amplitude_rad
             : 0.0;
}
double settle_norm(const StepMetrics& m) {
  return (m.valid && m.amplitude_rad > 1e-6)
             ? m.settling_time_s / m.amplitude_rad
             : 0.0;
}

void check_ratio(const std::string& what, double measured, double base,
                 double ratio_min, double ratio_max,
                 std::vector<std::string>& violations) {
  if (base <= 1e-9) return;  // nothing to compare against
  const double r = measured / base;
  if (r < ratio_min || r > ratio_max) {
    violations.push_back(what + " ratio " + std::to_string(r).substr(0, 4) +
                         " outside [" + std::to_string(ratio_min).substr(0, 3) +
                         "," + std::to_string(ratio_max).substr(0, 4) + "]");
  }
}

}  // namespace

AxisVerifyResult compare_axis(const AxisPayloadProfile& base,
                              const StepMetrics& step_pos,
                              const StepMetrics& step_neg,
                              const VerifyTolerances& tol) {
  AxisVerifyResult r;
  r.step_pos = step_pos;
  r.step_neg = step_neg;
  const PairWorst m{step_pos, step_neg};
  if (!m.valid()) {
    r.violations.push_back("no measured step response");
    return r;  // measured=false, ok=false
  }
  r.measured = true;

  const PairWorst b{base.baseline.step_pos, base.baseline.step_neg};
  if (!b.valid()) {
    r.violations.push_back("no stored baseline for this axis");
    return r;
  }

  // Rise / settling: amplitude-normalized (s/rad), ratio window.
  check_ratio("rise_time", worst(m, rise_norm), worst(b, rise_norm),
              tol.rise_ratio_min, tol.rise_ratio_max, r.violations);
  check_ratio("settling_time", worst(m, settle_norm), worst(b, settle_norm),
              tol.settle_ratio_min, tol.settle_ratio_max, r.violations);

  // Overshoot: absolute excursion beyond the stored baseline.
  const double m_overshoot = worst(m, [](const StepMetrics& x) { return x.overshoot; });
  const double b_overshoot = worst(b, [](const StepMetrics& x) { return x.overshoot; });
  if (m_overshoot - b_overshoot > tol.overshoot_abs_max) {
    r.violations.push_back("overshoot " +
                           std::to_string(m_overshoot).substr(0, 5) +
                           " exceeds baseline + " +
                           std::to_string(tol.overshoot_abs_max).substr(0, 4));
  }

  // Peak effort: the payload mass shifts the gravity term, so allow a ratio
  // plus a small absolute floor.
  const double m_eff = worst(m, [](const StepMetrics& x) { return x.peak_effort_nm; });
  const double b_eff = worst(b, [](const StepMetrics& x) { return x.peak_effort_nm; });
  if (m_eff > b_eff * tol.peak_effort_ratio_max + tol.peak_effort_abs_max_nm) {
    r.violations.push_back("peak effort " +
                           std::to_string(m_eff).substr(0, 4) +
                           " Nm exceeds " +
                           std::to_string(tol.peak_effort_ratio_max).substr(0, 3) +
                           "x baseline (" +
                           std::to_string(b_eff).substr(0, 4) + " Nm)");
  }

  // Tracking RMS: ratio of the stored value plus an absolute floor.
  const double m_rms = worst(m, [](const StepMetrics& x) { return x.tracking_rms_rad; });
  const double b_rms = worst(b, [](const StepMetrics& x) { return x.tracking_rms_rad; });
  if (m_rms > b_rms * tol.tracking_rms_ratio_max + tol.tracking_rms_abs_max_rad) {
    r.violations.push_back("tracking rms " +
                           std::to_string(m_rms).substr(0, 6) +
                           " rad exceeds " +
                           std::to_string(tol.tracking_rms_ratio_max).substr(0, 3) +
                           "x baseline + " +
                           std::to_string(tol.tracking_rms_abs_max_rad).substr(0, 6));
  }

  r.ok = r.violations.empty();
  return r;
}

PayloadStatus overall_status(const AxisVerifyResult (&axes)[kAxisCount],
                             bool profile_loaded) {
  bool any_unmeasured = false;
  bool any_violation = false;
  for (int i = 0; i < kAxisCount; ++i) {
    if (!axes[i].measured) any_unmeasured = true;
    if (axes[i].measured && !axes[i].ok) any_violation = true;
  }
  if (!profile_loaded) return PayloadStatus::NoProfile;
  if (any_unmeasured) return PayloadStatus::Error;
  if (any_violation) return PayloadStatus::Mismatch;
  return PayloadStatus::Ok;
}

DerateDecision decide(const VerifyResult& r, double derate_factor) {
  DerateDecision d;
  switch (r.status) {
    case PayloadStatus::Ok:
      d.action = "ok";
      break;
    case PayloadStatus::Mismatch:
      d.derate = true;
      d.factor = derate_factor;
      d.action = "derate " + std::to_string(derate_factor).substr(0, 3);
      break;
    case PayloadStatus::NoProfile:
      d.action = "no_profile";
      break;
    case PayloadStatus::Error:
      d.action = "error";
      break;
  }
  return d;
}

}  // namespace payload
}  // namespace ota
