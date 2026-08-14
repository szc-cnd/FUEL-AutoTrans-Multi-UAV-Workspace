#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>

namespace fast_planner {
namespace task_search {
class TurnYawFollowLatch {
public:
  bool arm(const Eigen::Vector2d& direction) {
    if (direction.norm() < 1e-6) return false;
    direction_ = direction.normalized();
    active_ = true;
    return true;
  }

  bool directionForYaw(double current_yaw, double release_angle,
                       Eigen::Vector2d& direction) {
    if (!active_) return false;
    direction = direction_;
    const double target_yaw = std::atan2(direction_.y(), direction_.x());
    const double error = std::atan2(std::sin(target_yaw - current_yaw),
                                    std::cos(target_yaw - current_yaw));
    // 对准阈值内的当前规划周期仍保持墙体轴线目标，下一周期才恢复普通横移锁航向。
    if (std::fabs(error) <= std::max(0.0, release_angle)) active_ = false;
    return true;
  }

  bool active() const { return active_; }

private:
  Eigen::Vector2d direction_{1.0, 0.0};
  bool active_{false};
};

inline bool isStationaryVerticalMotion(const Eigen::Vector3d& start,
                                       const Eigen::Vector3d& target,
                                       double xy_tolerance,
                                       double minimum_height_change = 0.01) {
  return (target - start).head<2>().norm() <=
             std::max(0.0, xy_tolerance) + 1e-9 &&
         std::fabs(target.z() - start.z()) + 1e-9 >=
             std::max(0.0, minimum_height_change);
}

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

inline bool passesLatestTurnNoReturn(const Eigen::Vector2d& candidate,
                                     const Eigen::Vector2d& turn_anchor,
                                     const Eigen::Vector2d& incoming_direction,
                                     const Eigen::Vector2d& outgoing_direction,
                                     double backward_margin) {
  if (incoming_direction.norm() < 1e-6) return true;
  const Eigen::Vector2d relative = candidate - turn_anchor;
  if (relative.dot(incoming_direction.normalized()) + 1e-6 >=
      -std::max(0.0, backward_margin))
    return true;
  // 只有大于90度的真实弯道才允许自然落到旧方向锚点之后，而且位移必须主要沿
  // 新通道方向。仅仅在新方向上有少量投影，不能放行数米长的旧通道回退目标。
  if (outgoing_direction.norm() < 1e-6) return false;
  const Eigen::Vector2d incoming = incoming_direction.normalized();
  const Eigen::Vector2d outgoing = outgoing_direction.normalized();
  if (incoming.dot(outgoing) >= -1e-3 || relative.norm() < 1e-6) return false;
  return relative.normalized().dot(outgoing) >= 0.70 &&
         relative.dot(outgoing) > std::max(0.0, backward_margin);
}

inline bool holdYawForLateralTranslation(double heading_alignment) {
  return heading_alignment <= 0.70;
}

inline bool holdYawInCorridor(bool corridor_search_active,
                              bool mapped_turn_detected) {
  return corridor_search_active && !mapped_turn_detected;
}

inline bool longRangeTurnViewSupported(
    double farthest_free_depth, int free_sections,
    double farthest_left_wall_depth, int left_wall_sections,
    double farthest_right_wall_depth, int right_wall_sections,
    double farthest_paired_wall_depth, int paired_wall_sections,
    int consistent_paired_wall_sections,
    double minimum_visible_depth, int minimum_free_sections,
    int minimum_wall_sections, int minimum_paired_wall_sections) {
  const bool far_free =
      farthest_free_depth + 1e-6 >= minimum_visible_depth &&
      free_sections >= std::max(1, minimum_free_sections);
  const bool far_left_wall =
      farthest_left_wall_depth + 1e-6 >= minimum_visible_depth &&
      left_wall_sections >= std::max(1, minimum_wall_sections);
  const bool far_right_wall =
      farthest_right_wall_depth + 1e-6 >= minimum_visible_depth &&
      right_wall_sections >= std::max(1, minimum_wall_sections);
  const bool paired_corridor =
      farthest_paired_wall_depth + 1e-6 >= minimum_visible_depth &&
      paired_wall_sections >= std::max(1, minimum_paired_wall_sections) &&
      consistent_paired_wall_sections >=
          std::max(1, minimum_paired_wall_sections);
  // 一侧墙只能说明有墙，不能说明它是新通道。真正转弯允许近角处一侧被遮挡，
  // 但远端必须重新看到同截面的左右墙，并且墙宽和中心保持一致。
  return far_free && far_left_wall && far_right_wall && paired_corridor;
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
