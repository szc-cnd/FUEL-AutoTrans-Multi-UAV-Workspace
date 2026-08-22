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

inline bool shouldReplanNearTrajectoryEnd(double time_to_end, double threshold) {
  return threshold >= 0.0 && time_to_end < threshold;
}

inline bool shouldHoldAtTrajectoryEnd(bool published_trajectory_active,
                                      bool replacement_published,
                                      bool brake_already_requested,
                                      double time_to_end,
                                      double hold_lead_time) {
  return published_trajectory_active && !replacement_published &&
         !brake_already_requested &&
         time_to_end <= std::max(0.0, hold_lead_time);
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

inline bool shouldActivateExternalExploration(bool use_diff_for_exploration,
                                              bool entry_trigger_received,
                                              bool external_exploration_active) {
  return use_diff_for_exploration && entry_trigger_received &&
         !external_exploration_active;
}

}  // namespace exploration_policy
}  // namespace fast_planner
