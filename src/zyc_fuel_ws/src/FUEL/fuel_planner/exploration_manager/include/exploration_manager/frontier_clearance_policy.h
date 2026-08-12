#pragma once

#include <algorithm>
#include <cmath>

namespace fast_planner {
namespace frontier_clearance {

inline double clearanceReward(double clearance, double reward_start,
                              double reward_full, double max_reward) {
  reward_start = std::max(0.0, reward_start);
  reward_full = std::max(reward_start + 1e-6, reward_full);
  max_reward = std::max(0.0, max_reward);
  if (clearance <= reward_start) return 0.0;
  const double ratio = std::min(1.0, (clearance - reward_start) /
                                        (reward_full - reward_start));
  return max_reward * ratio;
}

template <typename IsKnownFree>
double minimumKnownRadialClearance(double step, double max_distance,
                                   int direction_count,
                                   IsKnownFree&& is_known_free) {
  step = std::max(1e-3, step);
  max_distance = std::max(0.0, max_distance);
  direction_count = std::max(1, direction_count);
  double minimum_clearance = max_distance;
  for (int direction = 0; direction < direction_count; ++direction) {
    double free_distance = 0.0;
    for (double distance = step; distance <= max_distance + 1e-9;
         distance += step) {
      if (!is_known_free(direction, distance)) break;
      free_distance = std::min(distance, max_distance);
    }
    minimum_clearance = std::min(minimum_clearance, free_distance);
  }
  return minimum_clearance;
}

}  // namespace frontier_clearance
}  // namespace fast_planner
