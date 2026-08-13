#pragma once

#include <Eigen/Eigen>

#include <algorithm>

namespace fast_planner {
namespace directional_progress {

inline bool isAllowed(const Eigen::Vector3d& start,
                      const Eigen::Vector3d& candidate,
                      const Eigen::Vector3d& forward_direction,
                      double maximum_regression) {
  if (forward_direction.head<2>().norm() < 1e-6) return true;
  const Eigen::Vector2d forward = forward_direction.head<2>().normalized();
  const double progress = (candidate.head<2>() - start.head<2>()).dot(forward);
  return progress + 1e-6 >= -std::max(0.0, maximum_regression);
}

}  // namespace directional_progress
}  // namespace fast_planner
