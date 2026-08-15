#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

#include "precision_landing/landing_search_core.hpp"

namespace {

using precision_landing::WorldTargetFilter;
using precision_landing::WorldTargetFilterConfig;

WorldTargetFilterConfig testConfig() {
  WorldTargetFilterConfig config;
  config.stable_samples = 4U;
  config.max_sample_gap_sec = 0.20;
  config.max_position_jump_m = 0.30;
  config.max_spread_m = 0.05;
  return config;
}

TEST(LandingSearchCore, CameraPointUsesBothExtrinsicsAndBodyPose) {
  const Eigen::Vector3d result = precision_landing::cameraPointToWorld(
      Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.1, 0.2, 0.3),
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d(10.0, 20.0, 30.0),
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  EXPECT_NEAR(result.x(), 8.8, 1.0e-9);
  EXPECT_NEAR(result.y(), 20.1, 1.0e-9);
  EXPECT_NEAR(result.z(), 30.3, 1.0e-9);
}

TEST(LandingSearchCore, RequiresCompleteStableWindow) {
  WorldTargetFilter filter(testConfig());
  EXPECT_FALSE(filter.add(7, Eigen::Vector3d(1.00, 2.00, 0.0), 1.00));
  EXPECT_FALSE(filter.add(7, Eigen::Vector3d(1.01, 2.00, 0.0), 1.05));
  EXPECT_FALSE(filter.add(7, Eigen::Vector3d(0.99, 2.01, 0.0), 1.10));
  EXPECT_TRUE(filter.add(7, Eigen::Vector3d(1.00, 1.99, 0.0), 1.15));
  EXPECT_TRUE(filter.stable());
  EXPECT_EQ(filter.markerId(), 7);
  EXPECT_NEAR(filter.filteredPoint().x(), 1.0, 0.01);
  EXPECT_LT(filter.spread(), 0.05);
}

TEST(LandingSearchCore, ResetsOnMarkerChangeTimeGapAndJump) {
  WorldTargetFilter filter(testConfig());
  filter.add(1, Eigen::Vector3d::Zero(), 1.0);
  filter.add(1, Eigen::Vector3d::Zero(), 1.1);
  EXPECT_FALSE(filter.add(2, Eigen::Vector3d::Zero(), 1.2));
  EXPECT_EQ(filter.markerId(), 2);
  EXPECT_FALSE(filter.add(2, Eigen::Vector3d::Zero(), 1.5));
  EXPECT_FALSE(filter.add(2, Eigen::Vector3d(1.0, 0.0, 0.0), 1.6));
  EXPECT_FALSE(filter.stable());
}

TEST(LandingSearchCore, RejectsNoisyWindow) {
  WorldTargetFilter filter(testConfig());
  filter.add(3, Eigen::Vector3d(0.00, 0.0, 0.0), 1.00);
  filter.add(3, Eigen::Vector3d(0.04, 0.0, 0.0), 1.05);
  filter.add(3, Eigen::Vector3d(0.08, 0.0, 0.0), 1.10);
  EXPECT_FALSE(filter.add(3, Eigen::Vector3d(0.12, 0.0, 0.0), 1.15));
  EXPECT_GT(filter.spread(), 0.05);
}

} // namespace
