#include <gtest/gtest.h>
#include <cmath>

#include "plan_manage/corridor_dynamic_rechecker.h"

namespace {

std::vector<Eigen::Vector3d> frame(double ball_y) {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 20; ++i) {
    const double x = -1.0 + 0.1 * (i % 10);
    const double z = 0.35 + 0.06 * (i / 10);
    points.emplace_back(x, -0.75, z);
    points.emplace_back(x, 0.75, z);
  }
  for (int i = 0; i < 4; ++i)
    points.emplace_back(0.5 + 0.02 * i, ball_y, 0.65 + 0.02 * (i % 2));
  return points;
}
}  // namespace

TEST(CorridorDynamicRechecker, StaticWallsDoNotCreateObject) {
  diff_planner::CorridorDynamicRecheckerConfig config;
  config.min_wall_points = 4;
  diff_planner::CorridorDynamicRechecker rechecker(config);
  for (int i = 0; i < 8; ++i) {
    const auto objects = rechecker.process(frame(0.0), ros::Time(i * 0.1),
                                            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    EXPECT_TRUE(objects.empty());
  }
}

TEST(CorridorDynamicRechecker, LateralMotionNeedsConfirmation) {
  diff_planner::CorridorDynamicRecheckerConfig config;
  config.min_wall_points = 4;
  diff_planner::CorridorDynamicRechecker rechecker(config);
  EXPECT_TRUE(rechecker.process(frame(-0.20), ros::Time(0.0), Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero()).empty());
  EXPECT_TRUE(rechecker.process(frame(-0.05), ros::Time(0.1), Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero()).empty());
  EXPECT_TRUE(rechecker.process(frame(0.10), ros::Time(0.2), Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero()).empty());
  const auto objects = rechecker.process(frame(0.25), ros::Time(0.3), Eigen::Vector3d::Zero(),
                                         Eigen::Vector3d::Zero());
  ASSERT_EQ(objects.size(), 1u);
  EXPECT_EQ(objects.front().id, 1u);
  EXPECT_GT(std::abs(objects.front().velocity.y()), 0.10);
}
