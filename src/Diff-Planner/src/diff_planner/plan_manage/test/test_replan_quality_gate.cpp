#include <gtest/gtest.h>

#include <plan_manage/replan_quality_gate.h>

namespace diff_planner
{
namespace
{

ReplanTrajectoryMetrics metrics(double clearance, double length, double jerk)
{
  ReplanTrajectoryMetrics result;
  result.valid = true;
  result.min_clearance = clearance;
  result.length = length;
  result.jerk_cost = jerk;
  return result;
}

TEST(ReplanQualityGate, RejectsCandidateWithoutMeaningfulImprovement)
{
  const auto decision = ReplanQualityGate::decide(
      metrics(0.30, 4.0, 10.0), metrics(0.30, 4.0, 10.0),
      0.01, 0.01, ReplanQualityConfig());
  EXPECT_FALSE(decision.accept);
  EXPECT_EQ(decision.reason, "candidate has no meaningful improvement");
}

TEST(ReplanQualityGate, RejectsShorterCandidateThatLosesClearance)
{
  const auto decision = ReplanQualityGate::decide(
      metrics(0.30, 4.0, 10.0), metrics(0.25, 3.0, 8.0),
      0.01, 0.01, ReplanQualityConfig());
  EXPECT_FALSE(decision.accept);
  EXPECT_EQ(decision.reason, "minimum clearance decreased");
}

TEST(ReplanQualityGate, AcceptsMeaningfullyShorterCandidate)
{
  const auto decision = ReplanQualityGate::decide(
      metrics(0.30, 4.0, 10.0), metrics(0.29, 3.7, 11.0),
      0.01, 0.01, ReplanQualityConfig());
  EXPECT_TRUE(decision.accept);
}

TEST(ReplanQualityGate, ClearanceImprovementMayUseLongerPath)
{
  const auto decision = ReplanQualityGate::decide(
      metrics(0.25, 4.0, 10.0), metrics(0.32, 4.5, 18.0),
      0.01, 0.01, ReplanQualityConfig());
  EXPECT_TRUE(decision.accept);
}

TEST(ReplanQualityGate, RejectsDiscontinuousHandoff)
{
  const auto decision = ReplanQualityGate::decide(
      metrics(0.30, 4.0, 10.0), metrics(0.40, 3.0, 5.0),
      0.08, 0.01, ReplanQualityConfig());
  EXPECT_FALSE(decision.accept);
  EXPECT_EQ(decision.reason, "handoff position discontinuity");
}

}  // namespace
}  // namespace diff_planner

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
