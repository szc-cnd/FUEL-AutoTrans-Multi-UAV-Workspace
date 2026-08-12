#pragma once

namespace fast_planner {
namespace exploration_policy {

inline bool shouldInterruptCurrentTrajectory(bool replacement_succeeded) {
  return replacement_succeeded;
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
