#include <gtest/gtest.h>
#include "can/gm6020_rx_velocity.hpp"
#include "can/gm6020_velocity.hpp"

using namespace ota::gm6020;

TEST(Adr002Yaw, FreshRxWindowIgnoresDuplicateTicksAndSessionOffset) {
  RxVelocity velocity;
  for (int i=1; i<=100; ++i) {
    ASSERT_TRUE(velocity.observe(5.0+.2*i*.001, i*1'000'000LL));
    EXPECT_FALSE(velocity.observe(999, i*1'000'000LL));
  }
  for (int window : {20,30,40}) EXPECT_NEAR(velocity.estimate(window),.2,1e-10);
  EXPECT_FALSE(velocity.observe(NAN,101'000'000));
  EXPECT_FALSE(velocity.observe(5,1));
  EXPECT_TRUE(std::isnan(velocity.estimate(0)));
}

FrictionConfig measured_fixture() { return {true,.55,.60,.35,.40,1,.0023,.0087,4,4}; }

TEST(Adr002Yaw, SessionGainChangePreservesQuietSupportingCurrent) {
  VelocityLoop loop; loop.reset(0,1'000'000);
  for(int n=1;n<=100;++n)
    loop.update_amps(.1,0,1'000'000+n*5'000'000LL,.524,.8,1,.6,0);
  const double held=loop.update_amps(0,0,506'000'000,.524,.8,1,.6,0);
  ASSERT_GT(held,0);
  loop.prepare_current_tuning(0,2,.8);
  EXPECT_NEAR(loop.update_amps(0,0,511'000'000,.524,.8,2,.6,0),held,1e-12);
}

TEST(Adr002Yaw, FinalSlewAndCapBlockIntegratorWindup) {
  VelocityLoop loop; loop.reset(0,1'000'000);
  auto f=measured_fixture();
  double last=0;
  for(int n=1;n<=80;++n) {
    const double out=loop.update_amps(.3,0,1'000'000+n*5'000'000LL,.524,.8,1,.6,0,&f,true,n);
    ASSERT_TRUE(loop.valid());
    EXPECT_LE(std::abs(out-last),.0200001);
    EXPECT_LE(std::abs(out),.8);
    EXPECT_NEAR(loop.integral(),0,1e-12);
    last=out;
  }
}

TEST(Adr002Yaw, MovingAndQuietHandoffsUseDeliveredCurrent) {
  VelocityLoop loop; loop.reset(0,1'000'000);
  auto f=measured_fixture(); double last=0;
  for(int n=1;n<=70;++n) {
    double q=n<21 ? 0 : .003;
    double out=loop.update_amps(.1,q,1'000'000+n*5'000'000LL,.524,.8,1,.6,0,&f,true,n);
    EXPECT_LE(std::abs(out-last),.0200001); last=out;
  }
  ASSERT_EQ(loop.friction_output().state,FrictionState::Moving);
  const double held=loop.update_amps(0,.003,356'000'000,.524,.8,1,.6,0,&f,false,71);
  EXPECT_LE(std::abs(held-last),.0200001);
  const auto supporting_i=loop.integral();
  EXPECT_GT(supporting_i,.1);
  loop.update_amps(0,.003,361'000'000,.524,.8,1,.6,0,&f,false,72);
  EXPECT_DOUBLE_EQ(loop.integral(),supporting_i);
  EXPECT_EQ(loop.friction_output().feedforward_target_a,0);
}

TEST(Adr002Yaw, NoisyZeroCrossingDoesNotRearmAnAttempt) {
  VelocityLoop loop; loop.reset(0,1'000'000);
  auto f=measured_fixture(); int attempts=0;
  for(int n=1;n<=300;++n) {
    const double reference=n==1 ? -1e-7 : n==150 ? 0 : .1;
    loop.update_amps(reference,0,1'000'000+n*5'000'000LL,.524,.8,1,.6,0,&f,true,n);
    attempts+=loop.friction_output().new_attempt;
  }
  EXPECT_EQ(attempts,1);
  EXPECT_TRUE(loop.friction_output().attempt_exhausted);
}

TEST(Adr002Yaw, ReducedCapStillPermitsOpposingBrakeCurrent) {
  VelocityLoop loop; loop.reset(0,1'000'000); auto f=measured_fixture();
  for(int n=1;n<=60;++n)
    loop.update_amps(.2,0,1'000'000+n*5'000'000LL,.524,.8,1,.6,0,&f,true,n);
  double out=0;
  for(int n=61;n<=160;++n) {
    out=loop.update_amps(-.2,0,1'000'000+n*5'000'000LL,.524,.3,1,.6,0,&f,true,n);
    ASSERT_TRUE(loop.valid()); EXPECT_LE(std::abs(out),.3000001);
  }
  EXPECT_LT(out,0);
}
