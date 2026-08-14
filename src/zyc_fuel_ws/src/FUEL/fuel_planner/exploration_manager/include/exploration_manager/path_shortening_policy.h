#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>

namespace fast_planner {
namespace path_shortening {

inline bool preserveWaypointForClearance(double original_min_clearance,
                                         double shortcut_min_clearance,
                                         double loss_tolerance) {
  loss_tolerance = std::max(0.0, loss_tolerance);
  return shortcut_min_clearance + loss_tolerance + 1e-9 < original_min_clearance;
}

inline double turnAngleRadians(const Eigen::Vector3d& previous,
                               const Eigen::Vector3d& corner,
                               const Eigen::Vector3d& next) {
  Eigen::Vector2d incoming = (corner - previous).head<2>();
  Eigen::Vector2d outgoing = (next - corner).head<2>();
  if (incoming.norm() < 1e-6 || outgoing.norm() < 1e-6) return 0.0;
  const double cosine = std::max(
      -1.0, std::min(1.0, incoming.normalized().dot(outgoing.normalized())));
  return std::acos(cosine);
}

inline bool preserveWaypointForObstacleTurn(
    const Eigen::Vector3d& previous, const Eigen::Vector3d& corner,
    const Eigen::Vector3d& next, double original_min_clearance,
    double obstacle_clearance_threshold, double minimum_turn_angle) {
  return original_min_clearance <= std::max(0.0, obstacle_clearance_threshold) + 1e-9 &&
         turnAngleRadians(previous, corner, next) + 1e-9 >=
             std::max(0.0, minimum_turn_angle);
}

}  // namespace path_shortening
}  // namespace fast_planner
