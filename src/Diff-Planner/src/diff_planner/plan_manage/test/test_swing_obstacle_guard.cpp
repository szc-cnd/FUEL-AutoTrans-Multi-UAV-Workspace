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

TEST(SwingObstacleGuard, HarmonicPredictionStopsAndReversesAtEndpoint)
{
  constexpr double center = 0.10;
  constexpr double amplitude = 0.45;
  constexpr double half_period = 0.50;
  const double initial_velocity = amplitude * std::acos(-1.0) / half_period;

  EXPECT_NEAR(SwingObstacleGuard::harmonicCoordinate(
                  center, initial_velocity, 0.25, center, amplitude, half_period),
              center + amplitude, 1.0e-9);
  const double before_endpoint = SwingObstacleGuard::harmonicCoordinate(
      center, initial_velocity, 0.24, center, amplitude, half_period);
  const double after_endpoint = SwingObstacleGuard::harmonicCoordinate(
      center, initial_velocity, 0.26, center, amplitude, half_period);
  EXPECT_NEAR(before_endpoint, after_endpoint, 1.0e-9);
  EXPECT_LT(after_endpoint, center + amplitude);

  for (double time = 0.0; time <= 10.0; time += 0.01)
  {
    const double coordinate = SwingObstacleGuard::harmonicCoordinate(
        center, initial_velocity, time, center, amplitude, half_period);
    EXPECT_GE(coordinate, center - amplitude - 1.0e-12);
    EXPECT_LE(coordinate, center + amplitude + 1.0e-12);
  }
}

TEST(SwingObstacleGuard, LearnedHarmonicPredictionIsUsedForCollisionTiming)
{
  SwingObstacleGuard::Config harmonic_config;
  harmonic_config.enable_harmonic_prediction = true;
  harmonic_config.vehicle_radius = 0.0;
  harmonic_config.horizontal_margin = 0.0;
  harmonic_config.harmonic_reversal_velocity_epsilon = 0.02;
  SwingObstacleGuard harmonic_guard(harmonic_config);

  SwingObstacleGuard::Config reflected_config = harmonic_config;
  reflected_config.enable_harmonic_prediction = false;
  SwingObstacleGuard reflected_guard(reflected_config);

  constexpr double amplitude = 0.45;
  constexpr double half_period = 0.50;
  const double omega = std::acos(-1.0) / half_period;
  for (int index = 0; index <= 60; ++index)
  {
    const double time = 0.05 * index;
    SwingObstacleObservation ball;
    ball.id = 9;
    ball.position = Eigen::Vector3d(0.5, amplitude * std::sin(omega * time), 0.6);
    ball.velocity = Eigen::Vector3d(0.0, amplitude * omega * std::cos(omega * time), 0.0);
    ball.size = Eigen::Vector3d(0.1, 0.1, 0.1);
    harmonic_guard.update({ball}, time);
    reflected_guard.update({ball}, time);
  }

  const std::vector<SwingTrajectorySample> trajectory = {
      {0.0, Eigen::Vector3d(0.0, 0.0, 0.6)},
      {0.5, Eigen::Vector3d(0.5, 0.0, 0.6)}};
  SwingCollisionResult result;
  EXPECT_TRUE(harmonic_guard.findCollision(trajectory, 3.0, &result));
  EXPECT_NEAR(result.time_from_now, 0.5, 1.0e-12);
  EXPECT_FALSE(reflected_guard.findCollision(trajectory, 3.0, nullptr));
}

TEST(SwingObstacleGuard, NewIdInheritsRecentSwingHistory)
{
  SwingObstacleGuard::Config config;
  config.enable_harmonic_prediction = true;
  config.observation_retention = 2.0;
  config.identity_handoff_max_gap = 2.0;
  config.vehicle_radius = 0.0;
  config.horizontal_margin = 0.0;
  config.harmonic_min_samples = 8;
  config.harmonic_min_motion_span = 0.25;
  config.harmonic_reversal_velocity_epsilon = 0.02;
  SwingObstacleGuard guard(config);

  constexpr double amplitude = 0.45;
  constexpr double half_period = 0.50;
  const double omega = std::acos(-1.0) / half_period;
  for (int index = 0; index <= 30; ++index)
  {
    const double time = 0.05 * index;
    SwingObstacleObservation ball;
    ball.id = 7;
    ball.position = Eigen::Vector3d(0.5, amplitude * std::sin(omega * time), 0.6);
    ball.velocity = Eigen::Vector3d(0.0, amplitude * omega * std::cos(omega * time), 0.0);
    ball.size = Eigen::Vector3d(0.3, 0.3, 0.3);
    guard.update({ball}, time);
  }

  // LDOP 换了 external ID，但新观测仍在旧球的空间/尺寸/运动门内。
  SwingObstacleObservation replacement;
  replacement.id = 8;
  replacement.position = Eigen::Vector3d(0.5, amplitude * std::sin(omega * 1.60), 0.6);
  replacement.velocity = Eigen::Vector3d(
      0.0, amplitude * omega * std::cos(omega * 1.60), 0.0);
  replacement.size = Eigen::Vector3d(0.3, 0.3, 0.3);
  guard.update({replacement}, 1.60);

  const std::vector<SwingTrajectorySample> trajectory = {
      {0.0, Eigen::Vector3d(0.0, 0.0, 0.6)},
      {0.5, Eigen::Vector3d(0.5, 0.0, 0.6)}};
  SwingCollisionResult result;
  ASSERT_TRUE(guard.findCollision(trajectory, 2.0, &result));
  EXPECT_EQ(result.obstacle_id, 1U);
  EXPECT_NEAR(result.time_from_now, 0.5, 1.0e-12);
}

TEST(SwingObstacleGuard, LogicalTrackIgnoresLdopId)
{
  SwingObstacleGuard::Config config;
  config.enable_harmonic_prediction = true;
  config.realtime_observation_only = true;
  config.observation_retention = 2.0;
  config.vehicle_radius = 0.0;
  config.horizontal_margin = 0.0;
  SwingObstacleGuard guard(config);

  SwingObstacleObservation static_fragment;
  static_fragment.id = 21;
  static_fragment.position = Eigen::Vector3d(0.5, 0.2, 0.6);
  static_fragment.velocity = Eigen::Vector3d::Zero();
  static_fragment.size = Eigen::Vector3d(0.02, 0.02, 0.02);
  for (int index = 0; index < 6; ++index)
    guard.update({static_fragment}, 0.1 * index);

  SwingObstacleObservation new_fragment = static_fragment;
  new_fragment.id = 22;
  new_fragment.position.y() = 0.24;
  new_fragment.velocity.y() = 0.4;
  guard.update({new_fragment}, 0.7);

  const std::vector<SwingTrajectorySample> trajectory = {
      {0.0, Eigen::Vector3d(0.0, 0.0, 0.6)},
      {0.5, Eigen::Vector3d(0.5, 0.24, 0.6)}};
  SwingCollisionResult result;
  ASSERT_TRUE(guard.findCollision(trajectory, 0.7, &result));
  // LDOP 编号改变不创建第二条规划轨迹，规划器始终使用逻辑编号1。
  EXPECT_EQ(result.obstacle_id, 1U);
}

TEST(SwingObstacleGuard, RealtimeModeKeepsCurrentLateralPosition)
{
  SwingObstacleGuard::Config config;
  config.realtime_observation_only = true;
  config.vehicle_radius = 0.0;
  config.horizontal_margin = 0.0;
  config.observation_retention = 0.5;
  SwingObstacleGuard guard(config);

  SwingObstacleObservation ball;
  ball.id = 12;
  ball.position = Eigen::Vector3d(0.5, -0.45, 0.6);
  ball.velocity = Eigen::Vector3d(0.0, 0.4, 0.0);
  ball.size = Eigen::Vector3d(0.1, 0.1, 0.1);
  guard.update({ball}, 10.0);

  const std::vector<SwingTrajectorySample> trajectory = {
      {0.0, Eigen::Vector3d(0.0, 0.0, 0.6)},
      {1.0, Eigen::Vector3d(0.5, 0.0, 0.6)}};
  // The obstacle stays at y=-0.45 in realtime mode; a reflected trajectory
  // would cross the vehicle near the far end of this sample path.
  EXPECT_FALSE(guard.findCollision(trajectory, 10.0, nullptr));
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
  EXPECT_EQ(result.obstacle_id, 1U);
  EXPECT_GE(result.time_from_now, 0.0);
  EXPECT_LE(result.time_from_now, 5.0);
}

TEST(SwingObstacleGuard, DropsExpiredObservation)
{
  SwingObstacleGuard guard;
  guard.update({ballAt(0.55)}, 10.0);
  EXPECT_FALSE(guard.findCollision(straightTrajectory(0.60), 11.6, nullptr));
}

TEST(SwingObstacleGuard, EmptyFramesKeepOnePredictionOnlyTrack)
{
  SwingObstacleGuard guard;
  guard.update({ballAt(0.55)}, 10.0);
  guard.update({}, 10.6);

  EXPECT_EQ(guard.observationState(10.6),
            SwingObstacleGuard::ObservationState::PREDICTION_ONLY);
  EXPECT_TRUE(guard.hasLogicalTrack(10.6));
  EXPECT_FALSE(guard.hasRecentMeasurement(10.6, 0.25));
  SwingObstacleObservation observation;
  ASSERT_TRUE(guard.logicalTrackSnapshot(10.6, &observation));
  EXPECT_EQ(observation.id, 1U);
}

TEST(SwingObstacleGuard, LogicalTrackBecomesStaleThenResets)
{
  SwingObstacleGuard guard;
  guard.update({ballAt(0.55)}, 10.0);

  EXPECT_EQ(guard.observationState(11.6),
            SwingObstacleGuard::ObservationState::STALE);
  EXPECT_TRUE(guard.hasLogicalTrack(12.9));
  EXPECT_FALSE(guard.hasLogicalTrack(13.1));
}

TEST(SwingObstacleGuard, AmbiguousFrameDoesNotReplaceLogicalTrack)
{
  SwingObstacleGuard guard;
  SwingObstacleObservation original = ballAt(0.55);
  original.position.y() = 0.0;
  guard.update({original}, 10.0);

  SwingObstacleObservation left = original;
  left.id = 20;
  left.position.y() = -0.10;
  SwingObstacleObservation right = original;
  right.id = 21;
  right.position.y() = 0.10;
  guard.update({left, right}, 10.1);

  SwingObstacleObservation retained;
  ASSERT_TRUE(guard.logicalTrackSnapshot(10.1, &retained));
  EXPECT_NEAR(retained.position.y(), 0.0, 1.0e-12);
  EXPECT_EQ(retained.id, 1U);
}

} // namespace
} // namespace diff_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
