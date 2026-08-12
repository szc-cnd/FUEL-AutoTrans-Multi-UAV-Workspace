#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>

namespace fast_planner {
namespace task_search {
inline bool boundaryPathRejectRequiresGoalSwitch() { return true; }

inline double viewpointDirectionScoreAdjustment(double forward_alignment,
                                                double forward_bonus,
                                                double backward_penalty) {
  return -forward_bonus * std::max(0.0, forward_alignment) +
         backward_penalty * std::max(0.0, -forward_alignment);
}

inline bool insideForwardHalfPlane(const Eigen::Vector2d& direction,
                                   const Eigen::Vector2d& mission_inside_direction) {
  if (direction.norm() < 1e-6 || mission_inside_direction.norm() < 1e-6) return true;
  return direction.normalized().dot(mission_inside_direction.normalized()) >= -1e-6;
}

inline bool holdYawForLateralTranslation(double heading_alignment) {
  return heading_alignment <= 0.70;
}

inline bool mappedContourSupportsTurn(bool front_wall_blocked,
                                      int left_wall_support,
                                      int right_wall_support,
                                      int minimum_wall_support) {
  minimum_wall_support = std::max(1, minimum_wall_support);
  return front_wall_blocked &&
         std::max(left_wall_support, right_wall_support) >= minimum_wall_support;
}

inline bool passesMissionBoundaryNoReturn(double door_progress,
                                          double inside_return_margin,
                                          bool final_exit_guard_active,
                                          double exit_side) {
  if (door_progress < -inside_return_margin) return false;
  if (final_exit_guard_active && exit_side < -0.05) return false;
  return true;
}

inline Eigen::Vector2d forwardReference(bool entry_forward_phase, bool have_recent_motion,
                                        const Eigen::Vector2d& corridor_direction,
                                        const Eigen::Vector2d& recent_motion,
                                        const Eigen::Vector2d& yaw_direction) {
  if (entry_forward_phase && corridor_direction.norm() > 1e-3)
    return corridor_direction.normalized();
  if (have_recent_motion && recent_motion.norm() > 0.15)
    return recent_motion.normalized();
  if (corridor_direction.norm() > 1e-3) return corridor_direction.normalized();
  return yaw_direction.normalized();
}
}  // namespace task_search
}  // namespace fast_planner
