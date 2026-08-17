#include <gtest/gtest.h>

#include <plan_manage/swing_obstacle_guard.h>

#include <cmath>
#include <vector>

namespace diff_planner
{
namespace
{

std::vector<SwingTrajectorySample> straightTrajectory(double altitude)
{
  std::vector<SwingTrajectorySample> samples;
  for (int i = 0; i <= 50; ++i)
  {
    const double time = 0.1 * i;
    samples.push_back({time, Eigen::Vector3d(0.2 * time, 0.0, altitude)});
  }
  return samples;
}

SwingObstacleObservation ballAt(double z)
{
  SwingObstacleObservation ball;
  ball.id = 7;
  ball.position = Eigen::Vector3d(0.5, 0.0, z);
  ball.velocity = Eigen::Vector3d(0.0, 0.4, 0.0);
  ball.size = Eigen::Vector3d(0.3, 0.3, 0.3);
  return ball;
}

TEST(SwingObstacleGuard, ReflectedPredictionNeverLeavesCorridor)
{
  constexpr double half_width = 0.60;
  for (double time = 0.0; time <= 30.0; time += 0.01)
  {
    const double coordinate =
        SwingObstacleGuard::reflectedCoordinate(0.25, 0.9, time, half_width);
    EXPECT_LE(std::abs(coordinate), half_width + 1.0e-12);
  }
  EXPECT_GT(SwingObstacleGuard::reflectedCoordinate(-half_width, -0.4, 0.1,
                                                     half_width),
            -half_width);
}

TEST(SwingObstacleGuard, AllowsCompleteVehicleEnvelopeBelowBall)
{
  SwingObstacleGuard guard;
  for (double time = 10.0; time <= 13.0; time += 0.5)
    guard.update({ballAt(1.20)}, time);

  SwingCollisionResult result;
  EXPECT_FALSE(guard.findCollision(straightTrajectory(0.60), 13.0, &result));
}

TEST(SwingObstacleGuard, BlocksUnderpassUntilLowestHeightIsLearned)
{
  SwingObstacleGuard guard;
  guard.update({ballAt(1.20)}, 10.0);

  EXPECT_TRUE(guard.findCollision(straightTrajectory(0.60), 10.0, nullptr));
}

TEST(SwingObstacleGuard, UsesLowestBottomFromContinuousHistory)
{
  SwingObstacleGuard guard;
  for (double time = 10.0; time <= 13.0; time += 0.5)
    guard.update({ballAt(time == 11.5 ? 0.90 : 1.20)}, time);

  EXPECT_TRUE(guard.findCollision(straightTrajectory(0.60), 13.0, nullptr));
}

TEST(SwingObstacleGuard, BlocksSameHeightTimedIntersection)
{
  SwingObstacleGuard guard;
  guard.update({ballAt(0.55)}, 10.0);

  SwingCollisionResult result;
  ASSERT_TRUE(guard.findCollision(straightTrajectory(0.60), 10.0, &result));
  EXPECT_EQ(result.obstacle_id, 7U);
  EXPECT_GE(result.time_from_now, 0.0);
  EXPECT_LE(result.time_from_now, 5.0);
}

TEST(SwingObstacleGuard, DropsExpiredObservation)
{
  SwingObstacleGuard guard;
  guard.update({ballAt(0.55)}, 10.0);
  EXPECT_FALSE(guard.findCollision(straightTrajectory(0.60), 11.0, nullptr));
}

} // namespace
} // namespace diff_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
