#pragma once

#include <algorithm>

namespace fast_planner {
namespace path_shortening {

inline bool preserveWaypointForClearance(double original_min_clearance,
                                         double shortcut_min_clearance,
                                         double loss_tolerance) {
  loss_tolerance = std::max(0.0, loss_tolerance);
  return shortcut_min_clearance + loss_tolerance + 1e-9 < original_min_clearance;
}

}  // namespace path_shortening
}  // namespace fast_planner
