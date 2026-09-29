// The orchestration failures, tested instead of remembered.
//
// ADR-002.1 exists because a campaign ran a jog under gains it believed were applied but were not:
// the folder was named kp2-fine and the trace said Kp=1. No amount of process text prevents that
// once the sequence is spread over scripts, so the controller holds the door and these tests prove
// the door holds — including the failure path, which is the one that actually occurs at 1 am.
#include <gtest/gtest.h>

#include "control/param_transaction.hpp"

namespace ota::control {
namespace {

ParamValue value(const char* name, const char* canonical) {
  return ParamValue{name, canonical};
}

std::vector<ParamValue> candidate(const char* kp, const char* ki) {
  return {value("yaw.current_kp_a_per_rad_s", kp), value("yaw.current_ki_a_per_rad_s", ki)};
}

TEST(ParameterTransaction, PrepareApplyVerifyAdvancesExactlyOneRevision) {
  ParameterTransaction tx;
  EXPECT_EQ(tx.revision(), ParameterTransaction::kInitialRevision);
  EXPECT_FALSE(tx.blocks_motion());

  EXPECT_EQ("", tx.prepare(candidate("2", "0.6"), "r1"));
  EXPECT_STREQ("prepared", tx.state_name());
  EXPECT_TRUE(tx.blocks_motion())
      << "the set the runner intends is not yet the set the hardware holds";

  tx.begin_apply(candidate("1", "0.6"), "r1");   // what the hardware held before
  EXPECT_STREQ("applied_unverified", tx.state_name());

  EXPECT_EQ("", tx.verify(candidate("2", "0.6")));
  EXPECT_STREQ("idle", tx.state_name());
  EXPECT_FALSE(tx.blocks_motion());
  EXPECT_EQ(tx.revision(), ParameterTransaction::kInitialRevision + 1);
  EXPECT_EQ(tx.applied_hash(), effective_hash(candidate("2", "0.6")));
}

TEST(ParameterTransaction, TheUnverifiedApplyThatTheAdrWasWrittenAboutStillBlocksMotion) {
  ParameterTransaction tx;
  ASSERT_EQ("", tx.prepare(candidate("2", "0.6"), "r1"));
  tx.begin_apply(candidate("1", "0.6"), "r1");
  // The backend refused or lost the write and nothing has been confirmed. This is the moment a
  // script used to start a jog anyway.
  EXPECT_TRUE(tx.blocks_motion());
  EXPECT_STREQ("applied_unverified", tx.state_name());
}

TEST(ParameterTransaction, AMismatchedReadbackDemandsARestoreAndKeepsMotionBlocked) {
  ParameterTransaction tx;
  ASSERT_EQ("", tx.prepare(candidate("2", "0.6"), "r1"));
  tx.begin_apply(candidate("1", "0.6"), "r1");

  EXPECT_NE("", tx.verify(candidate("1", "0.6")));       // the write never took
  EXPECT_TRUE(tx.restore_required());
  EXPECT_TRUE(tx.blocks_motion());
  EXPECT_NE(std::string::npos, tx.last_reason().find("expected 2")) << tx.last_reason();

  EXPECT_EQ("", tx.verify(candidate("1", "0.6")));       // the restore is confirmed
  EXPECT_FALSE(tx.restore_required());
  EXPECT_FALSE(tx.blocks_motion());
}

TEST(ParameterTransaction, AnAbsentReadbackValueIsNotAConfirmedOne) {
  ParameterTransaction tx;
  ASSERT_EQ("", tx.prepare(candidate("2", "0.6"), "r1"));
  tx.begin_apply(candidate("1", "0.6"), "r1");
  const std::string reason = tx.verify({value("yaw.current_kp_a_per_rad_s", "2")});
  EXPECT_NE(std::string::npos, reason.find("yaw.current_ki_a_per_rad_s")) << reason;
  EXPECT_NE(std::string::npos, reason.find("absent")) << reason;
  EXPECT_TRUE(tx.blocks_motion());
}

TEST(ParameterTransaction, ApplyWithoutAPreparedSetIsRefusedRatherThanHalfWritten) {
  ParameterTransaction tx;
  tx.begin_apply(candidate("1", "0.6"), "orphan");       // a runner that skipped prepare
  EXPECT_STREQ("failed", tx.state_name());
  EXPECT_TRUE(tx.blocks_motion()) << "a controller that does not know what it holds must not move";
  EXPECT_NE(std::string::npos, tx.last_reason().find("prepare")) << tx.last_reason();
}

TEST(ParameterTransaction, TheSameRequestRetriedDoesNotStageAnythingTwice) {
  ParameterTransaction tx;
  ASSERT_EQ("", tx.prepare(candidate("2", "0.6"), "r1"));
  const std::string first_hash = tx.expected_hash();
  EXPECT_EQ("", tx.prepare(candidate("2", "0.6"), "r1"));   // a retry after a lost ack
  EXPECT_EQ(first_hash, tx.expected_hash());
  tx.begin_apply(candidate("1", "0.6"), "r1");
  tx.begin_apply(candidate("9", "9"), "r1");                // the same apply again
  EXPECT_EQ("", tx.verify(candidate("2", "0.6")));          // still verifies against the first set
}

TEST(ParameterTransaction, OneCandidateMayNotStateTheSameParameterTwice) {
  ParameterTransaction tx;
  std::vector<ParamValue> two = {value("yaw.current_kp_a_per_rad_s", "2"),
                                 value("yaw.current_kp_a_per_rad_s", "4"),
                                 value("yaw.current_ki_a_per_rad_s", "0.6")};
  EXPECT_NE("", tx.prepare(two, "r1"));
  EXPECT_STREQ("idle", tx.state_name()) << "a refused prepare must not leave a half-staged set";
}

TEST(ParameterTransaction, AnEmptyPrepareIsRefusedRatherThanMeaningWhateverIsLoaded) {
  ParameterTransaction tx;
  EXPECT_NE("", tx.prepare({}, "r1"));
  EXPECT_NE(std::string::npos, tx.last_reason().find("whatever is loaded")) << tx.last_reason();
}

TEST(ParameterTransaction, APreparedSetMayNotBeSwappedWhileAnApplyIsUnverified) {
  ParameterTransaction tx;
  ASSERT_EQ("", tx.prepare(candidate("2", "0.6"), "r1"));
  tx.begin_apply(candidate("1", "0.6"), "r1");
  EXPECT_NE("", tx.prepare(candidate("8", "2"), "r2"))
      << "staging the next candidate over an unconfirmed write is how two trials merge into one trace";
}

TEST(ParameterTransaction, TheHashIgnoresTheOrderParametersWereAskedFor) {
  const std::vector<ParamValue> a = {value("yaw.current_ki_a_per_rad_s", "0.6"),
                                     value("yaw.current_kp_a_per_rad_s", "2")};
  const std::vector<ParamValue> b = {value("yaw.current_kp_a_per_rad_s", "2"),
                                     value("yaw.current_ki_a_per_rad_s", "0.6")};
  EXPECT_EQ(effective_hash(a), effective_hash(b));
  EXPECT_NE(effective_hash(a), effective_hash(candidate("2", "0.61")));
}

TEST(ParameterTransaction, CanonicalTextIsWhatTheFirmwareStoresNotWhatSomeoneTyped) {
  // The register-backed identity of 0.03 is the float32 nearest it; recording that here keeps the
  // reason a station confirms a write from being folklore.
  EXPECT_EQ("0.0299999993", control::canonical_number(static_cast<float>(0.03)));
  EXPECT_EQ("2", canonical_number(2.0));
  EXPECT_EQ("0.6", canonical_number(0.6));
  // The trial command's three-count displacement threshold, in the unit the firmware stores it in:
  // 0.00230 rad, which reads as 0.132 deg only after a conversion somebody has to remember.
  EXPECT_EQ("0.00230097118", canonical_number(3 * 2 * 3.14159265358979323846 / 8192));
  EXPECT_EQ("nan", canonical_number(0.0 / 0.0));
  EXPECT_EQ("true", canonical_bool(true));
}

}  // namespace
}  // namespace ota::control
