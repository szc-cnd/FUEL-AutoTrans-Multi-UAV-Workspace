#include <plan_manage/replan_quality_gate.h>

#include <cmath>

namespace diff_planner
{

ReplanQualityDecision ReplanQualityGate::decide(
    const ReplanTrajectoryMetrics &current,
    const ReplanTrajectoryMetrics &candidate,
    const double handoff_position_error,
    const double handoff_velocity_error,
    const ReplanQualityConfig &config)
{
  if (!current.valid || !candidate.valid)
    return {false, "invalid trajectory metrics"};
  if (!std::isfinite(handoff_position_error) ||
      handoff_position_error > config.max_handoff_position_error)
    return {false, "handoff position discontinuity"};
  if (!std::isfinite(handoff_velocity_error) ||
      handoff_velocity_error > config.max_handoff_velocity_error)
    return {false, "handoff velocity discontinuity"};
  if (candidate.colliding)
    return {false, "candidate trajectory is occupied"};
  if (current.colliding)
    return {true, "current trajectory is occupied"};

  if (candidate.min_clearance + config.max_clearance_drop < current.min_clearance)
    return {false, "minimum clearance decreased"};

  const bool clearance_improved =
      candidate.min_clearance >=
      current.min_clearance + config.min_clearance_improvement;
  const bool length_improved =
      candidate.length <=
      current.length * (1.0 - config.min_length_improvement_ratio);
  const bool jerk_improved =
      candidate.jerk_cost <=
      current.jerk_cost * (1.0 - config.min_jerk_improvement_ratio);

  if (!clearance_improved &&
      candidate.length > current.length * config.max_length_ratio)
    return {false, "path length increased"};
  if (!clearance_improved &&
      candidate.jerk_cost > current.jerk_cost * config.max_jerk_ratio)
    return {false, "jerk cost increased"};
  if (!clearance_improved && !length_improved && !jerk_improved)
    return {false, "candidate has no meaningful improvement"};

  return {true, "candidate improves the current trajectory"};
}

}  // namespace diff_planner
