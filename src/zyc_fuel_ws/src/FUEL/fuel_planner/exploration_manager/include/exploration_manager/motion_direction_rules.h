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

  // 分段原地转向期间由FSM依据真实里程计yaw显式结束会话，不能让一次规划查询
  // 因为“接近目标”提前消费锁存方向。
  bool lockedDirection(Eigen::Vector2d& direction) const {
    if (!active_) return false;
    direction = direction_;
    return true;
  }

  void clear() { active_ = false; }

  bool active() const { return active_; }

private:
  Eigen::Vector2d direction_{1.0, 0.0};
  bool active_{false};
};

inline bool stationaryTurnReady(double speed, double max_speed,
                                double stable_elapsed,
                                double required_stable_time) {
  return std::isfinite(speed) && std::isfinite(stable_elapsed) &&
         speed <= std::max(0.0, max_speed) &&
         stable_elapsed >= std::max(0.0, required_stable_time);
}

inline bool corridorYawCorrectionNeeded(const Eigen::Vector2d& corridor_direction,
                                        double current_yaw,
                                        double correction_threshold) {
  if (corridor_direction.norm() < 1e-6) return false;
  const Eigen::Vector2d direction = corridor_direction.normalized();
  const double target_yaw = std::atan2(direction.y(), direction.x());
  const double error = std::atan2(std::sin(target_yaw - current_yaw),
                                  std::cos(target_yaw - current_yaw));
  return std::fabs(error) > std::max(0.0, correction_threshold);
}

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

inline double preferredDistanceCost(double distance, double preferred_min,
                                    double preferred_max, double near_weight,
                                    double far_weight) {
  const double minimum = std::max(0.0, preferred_min);
  const double maximum = std::max(minimum, preferred_max);
  if (distance < minimum)
    return std::max(0.0, near_weight) * (minimum - distance);
  if (distance > maximum)
    return std::max(0.0, far_weight) * (distance - maximum);
  return 0.0;
}

inline int directionSector(const Eigen::Vector2d& direction,
                           const Eigen::Vector2d& reference,
                           double sector_width_rad) {
  if (direction.norm() < 1e-6 || reference.norm() < 1e-6) return 0;
  const double width = std::max(1.0 * M_PI / 180.0,
                                std::min(2.0 * M_PI, sector_width_rad));
  const int sector_count = std::max(1, static_cast<int>(std::ceil(2.0 * M_PI / width)));
  const Eigen::Vector2d unit_direction = direction.normalized();
  const Eigen::Vector2d unit_reference = reference.normalized();
  const double relative_angle = std::atan2(
      unit_reference.x() * unit_direction.y() -
          unit_reference.y() * unit_direction.x(),
      unit_reference.dot(unit_direction));
  int sector = static_cast<int>(
      std::floor((relative_angle + M_PI + 0.5 * width) / width));
  sector %= sector_count;
  if (sector < 0) sector += sector_count;
  return sector;
}

inline bool insideForwardHalfPlane(const Eigen::Vector2d& direction,
                                   const Eigen::Vector2d& mission_inside_direction) {
  if (direction.norm() < 1e-6 || mission_inside_direction.norm() < 1e-6) return true;
  return direction.normalized().dot(mission_inside_direction.normalized()) >= -1e-6;
}

inline Eigen::Vector2d corridorGridCenter(
    const Eigen::Vector2d& obstacle_station,
    const Eigen::Vector2d& corridor_origin,
    const Eigen::Vector2d& corridor_direction) {
  if (corridor_direction.norm() < 1e-6) return obstacle_station;
  const Eigen::Vector2d direction = corridor_direction.normalized();
  return corridor_origin +
         (obstacle_station - corridor_origin).dot(direction) * direction;
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
