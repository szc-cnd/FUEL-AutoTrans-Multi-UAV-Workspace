#include <gtest/gtest.h>

#include <optimizer/vertical_obstacle_side_lock.h>

namespace diff_planner
{
namespace
{

TEST(VerticalObstacleSideLock, AcceptsSimilarUsableWidths)
{
  EXPECT_TRUE(VerticalObstacleSideLock::widthsAreComparable(
      0.42, 0.50, 0.20, 0.15));
}

TEST(VerticalObstacleSideLock, RejectsClearlyUnequalWidths)
{
  EXPECT_FALSE(VerticalObstacleSideLock::widthsAreComparable(
      0.25, 0.55, 0.20, 0.15));
}

TEST(VerticalObstacleSideLock, RejectsBlockedSide)
{
  EXPECT_FALSE(VerticalObstacleSideLock::widthsAreComparable(
      0.10, 0.12, 0.20, 0.15));
}

TEST(VerticalObstacleSideLock, ChoosesSideUsedByInitialPath)
{
  const Eigen::Vector3d center(1.0, 0.0, 0.6);
  const Eigen::Vector3d lateral(0.0, 1.0, 0.0);
  const std::vector<Eigen::Vector3d> path = {
      Eigen::Vector3d(0.0, 0.0, 0.6),
      Eigen::Vector3d(1.0, -0.35, 0.6),
      Eigen::Vector3d(2.0, 0.0, 0.6)};
  EXPECT_LT(VerticalObstacleSideLock::chooseSideNormal(
                path, center, lateral, 0.45, 0.45).dot(lateral),
            0.0);
}

TEST(VerticalObstacleSideLock, UsesWiderSideForFlatPathFallback)
{
  const Eigen::Vector3d center(1.0, 0.0, 0.6);
  const Eigen::Vector3d lateral(0.0, 1.0, 0.0);
  const std::vector<Eigen::Vector3d> path = {
      Eigen::Vector3d(0.0, 0.0, 0.6),
      Eigen::Vector3d(2.0, 0.0, 0.6)};
  EXPECT_GT(VerticalObstacleSideLock::chooseSideNormal(
                path, center, lateral, 0.50, 0.45).dot(lateral),
            0.0);
}

TEST(VerticalObstacleSideLock, UsesMidpointOfObstacleAndWallBoundaries)
{
  EXPECT_DOUBLE_EQ(
      VerticalObstacleSideLock::computePassageCenter(0.10, 0.75), 0.425);
}

} // namespace
} // namespace diff_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
