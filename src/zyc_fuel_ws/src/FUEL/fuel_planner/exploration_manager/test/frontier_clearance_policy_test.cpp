#include <cassert>
#include <cmath>

#include "exploration_manager/frontier_clearance_policy.h"

int main() {
  using fast_planner::frontier_clearance::clearanceReward;
  using fast_planner::frontier_clearance::minimumKnownRadialClearance;

  assert(clearanceReward(0.29, 0.30, 0.60, 5.0) == 0.0);
  assert(clearanceReward(0.30, 0.30, 0.60, 5.0) == 0.0);
  assert(std::fabs(clearanceReward(0.45, 0.30, 0.60, 5.0) - 2.5) < 1e-9);
  assert(clearanceReward(0.60, 0.30, 0.60, 5.0) == 5.0);
  assert(clearanceReward(0.90, 0.30, 0.60, 5.0) == 5.0);

  const double open_clearance = minimumKnownRadialClearance(
      0.05, 0.60, 8, [](int, double) { return true; });
  assert(std::fabs(open_clearance - 0.60) < 1e-9);

  const double narrow_direction = minimumKnownRadialClearance(
      0.05, 0.60, 8,
      [](int direction, double distance) {
        return direction != 3 || distance < 0.40 - 1e-9;
      });
  assert(std::fabs(narrow_direction - 0.35) < 1e-9);

  const double unknown_nearby = minimumKnownRadialClearance(
      0.05, 0.60, 8,
      [](int direction, double distance) {
        return direction != 5 || distance < 0.20 - 1e-9;
      });
  assert(std::fabs(unknown_nearby - 0.15) < 1e-9);
  assert(clearanceReward(unknown_nearby, 0.30, 0.60, 5.0) == 0.0);
  return 0;
}
