#include <gtest/gtest.h>

#include <optimizer/suspended_obstacle_underpass.h>

#include <limits>

namespace diff_planner
{
namespace
{

TEST(SuspendedObstacleUnderpass, AcceptsPathThatActuallyDescends)
{
  const Eigen::Vector3d start(0.0, 0.0, 0.6);
  const Eigen::Vector3d end(1.0, 0.0, 0.6);
  const std::vector<Eigen::Vector3d> path = {
      start, Eigen::Vector3d(0.5, 0.0, 0.45), end};

  EXPECT_TRUE(SuspendedObstacleUnderpass::hasRequiredDescent(
      path, start, end, 0.10));
}

TEST(SuspendedObstacleUnderpass, RejectsFlatSidePath)
{
  const Eigen::Vector3d start(0.0, 0.0, 0.6);
  const Eigen::Vector3d end(1.0, 0.0, 0.6);
  const std::vector<Eigen::Vector3d> path = {
      start, Eigen::Vector3d(0.5, 0.1, 0.6), end};

  EXPECT_FALSE(SuspendedObstacleUnderpass::hasRequiredDescent(
      path, start, end, 0.10));
}

TEST(SuspendedObstacleUnderpass, RejectsNonFinitePath)
{
  const Eigen::Vector3d start(0.0, 0.0, 0.6);
  const Eigen::Vector3d end(1.0, 0.0, 0.6);
  const std::vector<Eigen::Vector3d> path = {
      start,
      Eigen::Vector3d(0.5, 0.0, std::numeric_limits<double>::quiet_NaN()),
      end};

  EXPECT_FALSE(SuspendedObstacleUnderpass::hasRequiredDescent(
      path, start, end, 0.10));
}

TEST(SuspendedObstacleUnderpass, ClassifiesFreeSpaceBelowSuspendedObstacle)
{
  const std::vector<int> downward_occupancy = {1, 1, 0, 0, 0, 0, 0, 1};
  EXPECT_TRUE(SuspendedObstacleUnderpass::hasVerticalGap(
      downward_occupancy, 0.10, 0.35));
}

TEST(SuspendedObstacleUnderpass, RejectsGroundConnectedObstacle)
{
  const std::vector<int> downward_occupancy = {1, 1, 1, 1, 1, 1, 1, -1};
  EXPECT_FALSE(SuspendedObstacleUnderpass::hasVerticalGap(
      downward_occupancy, 0.10, 0.35));
}

TEST(SuspendedObstacleUnderpass, RejectsGapThatIsTooLow)
{
  const std::vector<int> downward_occupancy = {1, 0, 0, 0, 1};
  EXPECT_FALSE(SuspendedObstacleUnderpass::hasVerticalGap(
      downward_occupancy, 0.10, 0.35));
}

TEST(SuspendedObstacleUnderpass, AcceptsEnoughGapBeforeVirtualGround)
{
  const std::vector<int> downward_occupancy = {1, 0, 0, 0, 0, -1};
  EXPECT_TRUE(SuspendedObstacleUnderpass::hasVerticalGap(
      downward_occupancy, 0.10, 0.35));
}

TEST(SuspendedObstacleUnderpass, ChoosesLowHeightInsideSharedGap)
{
  double target_z = 0.0;
  EXPECT_TRUE(SuspendedObstacleUnderpass::choosePassageHeight(
      0.55, 0.20, 0.60, 0.60, 0.20, target_z));
  EXPECT_DOUBLE_EQ(target_z, 0.375);
}

TEST(SuspendedObstacleUnderpass, RejectsGapAboveRequiredDescent)
{
  double target_z = 0.0;
  EXPECT_FALSE(SuspendedObstacleUnderpass::choosePassageHeight(
      0.55, 0.45, 0.60, 0.60, 0.20, target_z));
}

} // namespace
} // namespace diff_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
