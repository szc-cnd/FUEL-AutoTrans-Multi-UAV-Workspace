#pragma once

#include <algorithm>

namespace fast_planner {
namespace exploration_policy {

inline bool shouldInterruptCurrentTrajectory(bool replacement_succeeded) {
  return replacement_succeeded;
}

inline bool shouldMonitorPublishedTrajectory(bool published_trajectory_active,
                                             bool replacement_pending,
                                             bool brake_already_requested = false) {
  (void)replacement_pending;
  return published_trajectory_active && !brake_already_requested;
}

inline bool shouldBrakePublishedTrajectory(bool published_trajectory_active,
                                           bool collision_predicted,
                                           bool brake_already_requested) {
  return published_trajectory_active && collision_predicted && !brake_already_requested;
}

inline bool shouldReplanNearTrajectoryEnd(double time_to_end, double threshold,
                                          double trajectory_duration) {
  if (threshold < 0.0 || trajectory_duration <= 0.0) return false;
  // 短局部轨迹的总时长可能小于全局1s阈值。此时只在最后25%启动接续规划，
  // 不能刚发布就因为“剩余不足1s”立即把已确认的侧绕短步刷掉。
  const double effective_threshold =
      std::min(threshold, std::max(0.05, 0.25 * trajectory_duration));
  return time_to_end < effective_threshold;
}

inline bool shouldContinueCurrentTrajectoryAfterReplacementFailure(
    bool active_trajectory_valid, bool active_trajectory_braked,
    bool active_trajectory_safe, double trajectory_time,
    double trajectory_duration) {
  return active_trajectory_valid && !active_trajectory_braked &&
         active_trajectory_safe && trajectory_time < trajectory_duration;
}

inline bool shouldPeriodicReplan(double trajectory_time, double threshold,
                                 bool periodic_replan_enabled) {
  return periodic_replan_enabled && threshold >= 0.0 && trajectory_time > threshold;
}

inline bool shouldReplanForCoveredFrontier(bool frontier_covered,
                                           bool narrow_corridor_stage,
                                           double trajectory_time,
                                           double minimum_time) {
  return frontier_covered && !narrow_corridor_stage && trajectory_time > minimum_time;
}

}  // namespace exploration_policy
}  // namespace fast_planner
