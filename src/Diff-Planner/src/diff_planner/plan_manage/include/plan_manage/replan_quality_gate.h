#ifndef _REPLAN_QUALITY_GATE_H_
#define _REPLAN_QUALITY_GATE_H_

#include <string>

namespace diff_planner
{

struct ReplanTrajectoryMetrics
{
  bool valid{false};
  bool colliding{false};
  double min_clearance{0.0};
  double length{0.0};
  double jerk_cost{0.0};
};

struct ReplanQualityConfig
{
  double max_clearance_drop{0.03};
  double min_clearance_improvement{0.05};
  double max_length_ratio{1.05};
  double min_length_improvement_ratio{0.05};
  double max_jerk_ratio{1.50};
  double min_jerk_improvement_ratio{0.20};
  double max_handoff_position_error{0.06};
  double max_handoff_velocity_error{0.10};
};

struct ReplanQualityDecision
{
  bool accept{false};
  std::string reason;
};

class ReplanQualityGate
{
public:
  static ReplanQualityDecision decide(
      const ReplanTrajectoryMetrics &current,
      const ReplanTrajectoryMetrics &candidate,
      double handoff_position_error,
      double handoff_velocity_error,
      const ReplanQualityConfig &config);
};

}  // namespace diff_planner

#endif
