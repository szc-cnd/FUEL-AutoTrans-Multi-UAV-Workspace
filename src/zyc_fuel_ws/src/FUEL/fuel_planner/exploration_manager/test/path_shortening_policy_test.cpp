#include <cassert>

#include "exploration_manager/path_shortening_policy.h"

int main() {
  using fast_planner::path_shortening::preserveWaypointForClearance;
  using fast_planner::path_shortening::preserveWaypointForObstacleTurn;
  using fast_planner::path_shortening::turnAngleRadians;

  // A shortcut that gives up more than 5 cm of clearance must keep the A* turn.
  assert(preserveWaypointForClearance(0.35, 0.29, 0.05));

  // Small map/sampling fluctuations must not stop ordinary path shortening.
  assert(!preserveWaypointForClearance(0.35, 0.30, 0.05));
  assert(!preserveWaypointForClearance(0.18, 0.16, 0.05));

  // A shortcut with equal or better clearance is always allowed.
  assert(!preserveWaypointForClearance(0.20, 0.20, 0.05));
  assert(!preserveWaypointForClearance(0.20, 0.28, 0.05));

  const Eigen::Vector3d p0(0.0, 0.0, 0.5);
  const Eigen::Vector3d p1(1.0, 0.0, 0.5);
  const Eigen::Vector3d p2(1.0, 1.0, 0.5);
  assert(std::fabs(turnAngleRadians(p0, p1, p2) - M_PI_2) < 1e-9);
  // 障碍附近的90度A*转角不能被缩成提前斜切；开阔区同角度仍允许普通平滑。
  assert(preserveWaypointForObstacleTurn(p0, p1, p2, 0.35, 0.50,
                                         30.0 * M_PI / 180.0));
  assert(!preserveWaypointForObstacleTurn(p0, p1, p2, 0.70, 0.50,
                                          30.0 * M_PI / 180.0));
  return 0;
}
