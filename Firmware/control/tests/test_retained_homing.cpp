#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include "calibration/retained_homing.hpp"

TEST(RetainedHoming, PersistsOnlyMatchingCalibrationAndInvalidatesBeforeReuse) {
  const auto path="/dev/shm/ota-cache-test-"+std::to_string(getpid());
  const auto config=path+".yaml", cache=path+".bin";
  { std::ofstream f(config); f << "fixture configuration"; }
  std::array<ota::AxisLogicalModel, 2> models{};
  std::array<ota::AxisLimits, 2> limits{};
  {
    ota::RetainedHoming saved(config, {100,101}, cache);
    EXPECT_FALSE(saved.valid()); saved.save(models, limits); EXPECT_TRUE(saved.valid());
  }
  {
    ota::RetainedHoming restored(config, {100,101}, cache);
    EXPECT_TRUE(restored.load(models, limits)); restored.invalidate();
  }
  {
    ota::RetainedHoming stopped(config, {100,101}, cache);
    EXPECT_FALSE(stopped.valid()); stopped.save(models, limits);
  }
  {
    ota::RetainedHoming wrong_motor(config, {100,102}, cache);
    EXPECT_FALSE(wrong_motor.valid());
  }
  std::remove(config.c_str()); std::remove(cache.c_str());
}
