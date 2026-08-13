#include <cassert>

#include "exploration_manager/replan_execution_policy.h"

int main() {
  using fast_planner::exploration_policy::shouldInterruptCurrentTrajectory;
  using fast_planner::exploration_policy::shouldReplanNearTrajectoryEnd;
  using fast_planner::exploration_policy::shouldPeriodicReplan;
  using fast_planner::exploration_policy::shouldBrakePublishedTrajectory;
  using fast_planner::exploration_policy::shouldMonitorPublishedTrajectory;
  using fast_planner::exploration_policy::shouldReplanForCoveredFrontier;

  // 新规划失败时，仍然安全的当前轨迹不能被提前截断。
  assert(!shouldInterruptCurrentTrajectory(false));
  assert(shouldInterruptCurrentTrajectory(true));

  // FSM 已进入 PLAN_TRAJ 时，traj_server 仍在执行已发布轨迹，安全检查不能停。
  assert(shouldMonitorPublishedTrajectory(true, true));
  assert(shouldMonitorPublishedTrajectory(true, false));
  assert(!shouldMonitorPublishedTrajectory(true, true, true));
  assert(!shouldMonitorPublishedTrajectory(false, true));

  // 等待替代轨迹期间一旦旧轨迹预测碰撞，只发一次短刹车指令。
  assert(shouldBrakePublishedTrajectory(true, true, false));
  assert(!shouldBrakePublishedTrajectory(true, false, false));
  assert(!shouldBrakePublishedTrajectory(true, true, true));
  assert(!shouldBrakePublishedTrajectory(false, true, false));

  assert(shouldReplanNearTrajectoryEnd(0.99, 1.0));
  assert(!shouldReplanNearTrajectoryEnd(1.01, 1.0));

  // 通道模式关闭无条件周期重规划；碰撞与临近终点由FSM的独立分支处理。
  assert(!shouldPeriodicReplan(30.0, 3.0, false));
  assert(shouldPeriodicReplan(3.1, 3.0, true));
  assert(!shouldPeriodicReplan(2.9, 3.0, true));

  assert(!shouldReplanForCoveredFrontier(true, true, 1.0, 0.5));
  assert(shouldReplanForCoveredFrontier(true, false, 1.0, 0.5));
  assert(!shouldReplanForCoveredFrontier(false, false, 1.0, 0.5));
  return 0;
}
