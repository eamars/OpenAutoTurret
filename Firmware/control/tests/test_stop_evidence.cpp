#include "control/stop_evidence.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <string>

// These tests exist because the contract's two sharpest sentences are about what may
// NOT be said, and a type that permits the mistake will eventually make it. If an axis
// were a bool, someone would AND the two of them and call the result "safely powered
// down"; if `Unsupported` were `false`, a GM6020 would look like a motor we had proven
// to be energised. Neither sentence is enforceable in prose, so it is enforced here.
namespace {

using ota::AxisStopEvidence;
using ota::Evidence;
using ota::StopEvidence;

AxisStopEvidence Honest(const char* name) {
  AxisStopEvidence a;
  a.axis = name;
  a.zero_requested = Evidence::Requested;
  a.disable_requested = Evidence::Requested;
  a.stationary_observed = Evidence::Observed;
  a.stationary_window_ns = 1'000'000'000LL;
  a.feedback_age_ms = 1.2;
  a.last_neutral_request_ns = 12345678901234567LL;
  return a;
}

StopEvidence TwoAxes() {
  StopEvidence e;
  e.stop_id = "stop-1";
  e.reason = "operator_stop";
  e.stage = "parked";
  e.requested_at_ns = 12345678901234000LL;
  e.axes[0] = Honest("pitch");
  e.axes[1] = Honest("yaw");
  // CyberGear tells us it disabled; the GM6020 has no enable bit in its frame.
  e.axes[0].disable_confirmed = Evidence::Confirmed;
  e.axes[1].disable_confirmed = Evidence::Unsupported;
  return e;
}

TEST(StopEvidence, ADriveThatCannotTellUsIsNotEvidenceEitherWay) {
  StopEvidence e = TwoAxes();
  e.finalise();
  EXPECT_EQ(e.completion_quality, "verified") << "GM6020 cannot report disable; that is a "
                                                "fact about the drive, not a gap in this stop";
  EXPECT_TRUE(e.missing_evidence.empty());
  EXPECT_EQ(std::string(e.axes[1].disable_confirmed == Evidence::Unsupported ? "u" : "?"), "u");
}

TEST(StopEvidence, TheGoodAxisCannotCarryTheBadOne) {
  StopEvidence e = TwoAxes();
  e.axes[1].stationary_observed = Evidence::Absent;   // we never looked at yaw
  e.finalise();
  EXPECT_EQ(e.completion_quality, "partial");
  ASSERT_EQ(e.missing_evidence.size(), 1u);
  EXPECT_EQ(e.missing_evidence[0], "yaw:no_stationary_observation")
      << "pitch being confirmed must not turn into a green that covers yaw";
}

TEST(StopEvidence, NothingAtAllIsUnverifiedNotFailed) {
  StopEvidence e = TwoAxes();
  for (int i = 0; i < 2; ++i) {
    e.axes[i].zero_requested = Evidence::Absent;
    e.axes[i].disable_requested = Evidence::Absent;
    e.axes[i].stationary_observed = Evidence::Absent;
    e.axes[i].disable_confirmed = Evidence::Absent;
    e.axes[i].feedback_age_ms = -1.0;
  }
  e.finalise();
  EXPECT_EQ(e.completion_quality, "unverified");
  EXPECT_EQ(e.missing_evidence.size(), 6u);   // three claims per axis, both axes
}

TEST(StopEvidence, UnknownFeedbackAgeIsPublishedAsNullNotAsZero) {
  StopEvidence e = TwoAxes();
  e.axes[0].feedback_age_ms = -1.0;
  e.finalise();
  const std::string line = e.to_json_line();
  EXPECT_NE(line.find("\"feedback_age_ms\":null"), std::string::npos) << line;
  EXPECT_EQ(line.find("\"feedback_age_ms\":0.000"), std::string::npos) << line;
  bool named = false;
  for (const std::string& m : e.missing_evidence)
    named |= m == "pitch:feedback_age_unknown";
  EXPECT_TRUE(named);
}

TEST(StopEvidence, TheLineIsOneJsonObjectWithNsQuoted) {
  StopEvidence e = TwoAxes();
  e.finalise();
  const std::string line = e.to_json_line();
  // Adjacency and quote parity, the lesson from the first on-station trip file: a key
  // can be present and the line still unparseable.
  EXPECT_NE(line.find("\"requested_at_ns\":\"12345678901234000\""), std::string::npos) << line;
  EXPECT_NE(line.find("\"last_neutral_request_ns\":\"12345678901234567\""), std::string::npos)
      << line;
  EXPECT_NE(line.find("\"disable_confirmed\":\"unsupported\""), std::string::npos) << line;
  EXPECT_NE(line.find("\"power_isolated_confirmed\":\"unsupported\""), std::string::npos) << line;
  int quotes = 0;
  for (char c : line) quotes += (c == '"');
  EXPECT_EQ(quotes % 2, 0) << line;
  EXPECT_EQ(line.back(), '\n');
}

}  // namespace
