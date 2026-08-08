#include <gtest/gtest.h>

#include "entry_command_reference_limiter.h"

using PayloadMPC::EntryCommandReferenceLimiter;

TEST(EntryCommandReferenceLimiter, LimitsVelocityAndAcceleration)
{
  EntryCommandReferenceLimiter limiter;
  limiter.configure(0.20, 0.30);
  limiter.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d(2.0, 0.0, 0.0));

  double max_speed = 0.0;
  for (int i = 0; i < 2000 && (limiter.reference().position - Eigen::Vector3d(2.0, 0.0, 0.0)).norm() > 1.0e-6; ++i)
  {
    ASSERT_TRUE(limiter.update(0.01));
    max_speed = std::max(max_speed, limiter.reference().velocity.norm());
    EXPECT_LE(limiter.reference().velocity.norm(), 0.20 + 1.0e-9);
    EXPECT_LE(limiter.reference().acceleration.norm(), 0.30 + 1.0e-9);
  }
  EXPECT_NEAR(limiter.reference().position.x(), 2.0, 1.0e-6);
  EXPECT_LE(max_speed, 0.20 + 1.0e-9);
}

TEST(EntryCommandReferenceLimiter, NewTargetDoesNotJumpReference)
{
  EntryCommandReferenceLimiter limiter;
  limiter.configure(0.20, 0.30);
  limiter.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 0.0));
  ASSERT_TRUE(limiter.update(0.1));
  const Eigen::Vector3d before = limiter.reference().position;
  limiter.setTarget(Eigen::Vector3d(-1.0, 0.0, 0.0));
  ASSERT_TRUE(limiter.update(0.1));
  EXPECT_LT((limiter.reference().position - before).norm(), 0.20 * 0.1 + 1.0e-9);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
