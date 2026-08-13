#include <cassert>
#include <cmath>
#include <deque>

#include "exploration_manager/inflation_history_escape_policy.h"

int main() {
  using Eigen::Vector3d;
  using fast_planner::inflation_escape::buildBackwardHistoryPath;
  using fast_planner::inflation_escape::buildForwardTangentPath;
  using fast_planner::inflation_escape::buildDirectionalPath;
  using fast_planner::inflation_escape::shouldStartFromOdometry;

  assert(shouldStartFromOdometry(true));
  assert(!shouldStartFromOdometry(false));

  const auto lateral = buildDirectionalPath(
      Vector3d(0.0, 0.0, 0.75), Vector3d(0.0, -2.0, 0.4), 0.30, 0.05);
  assert(lateral.size() >= 3);
  assert(std::fabs(lateral.back().x()) < 1e-9);
  assert(std::fabs(lateral.back().y() + 0.30) < 1e-9);
  assert(std::fabs(lateral.back().z() - 0.75) < 1e-9);

  std::deque<Vector3d> history{
      Vector3d(0.00, 0.00, 0.75), Vector3d(0.10, 0.00, 0.75),
      Vector3d(0.20, 0.02, 0.75), Vector3d(0.30, 0.02, 0.75),
      Vector3d(0.40, 0.02, 0.75)};
  const Vector3d current(0.42, 0.02, 0.75);

  const auto backward = buildBackwardHistoryPath(current, history, 0.45, 0.05);
  assert(backward.size() >= 3);
  assert((backward.front() - current).norm() < 1e-9);
  assert(fast_planner::inflation_escape::pathLength(backward) <= 0.45 + 1e-9);
  assert(backward.back().x() < current.x());

  const auto forward = buildForwardTangentPath(current, history, 0.45, 0.05);
  assert(forward.size() >= 3);
  assert((forward.front() - current).norm() < 1e-9);
  assert(std::fabs(fast_planner::inflation_escape::pathLength(forward) - 0.45) < 1e-6);
  assert(forward.back().x() > current.x());

  const auto empty_forward =
      buildForwardTangentPath(current, std::deque<Vector3d>{current}, 0.45, 0.05);
  assert(empty_forward.empty());

  // 旧航迹先沿X、最近已经转向Y时，前向脱困必须沿最近切线，不能被远处旧方向拉偏。
  std::deque<Vector3d> turning_history{
      Vector3d(0.00, 0.00, 0.75), Vector3d(0.40, 0.00, 0.75),
      Vector3d(0.40, 0.10, 0.75), Vector3d(0.40, 0.20, 0.75),
      Vector3d(0.40, 0.30, 0.75)};
  const auto turning_forward = buildForwardTangentPath(
      Vector3d(0.40, 0.32, 0.75), turning_history, 0.45, 0.05);
  assert(turning_forward.back().y() > 0.70);
  assert(std::fabs(turning_forward.back().x() - 0.40) < 0.05);
  return 0;
}
