#include <cassert>

#include "exploration_manager/replan_execution_policy.h"

int main() {
  using fast_planner::exploration_policy::shouldInterruptCurrentTrajectory;
  using fast_planner::exploration_policy::shouldHoldAtTrajectoryEnd;
  using fast_planner::exploration_policy::shouldReplanNearTrajectoryEnd;
  using fast_planner::exploration_policy::shouldPeriodicReplan;
  using fast_planner::exploration_policy::shouldBrakePublishedTrajectory;
  using fast_planner::exploration_policy::shouldMonitorPublishedTrajectory;
  using fast_planner::exploration_policy::shouldReplanForCoveredFrontier;
  using fast_planner::exploration_policy::shouldActivateExternalExploration;

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

  // 下一条轨迹尚未发布时只在旧轨迹真正到达终点才锁点，不能提前一秒刹停。
  assert(!shouldHoldAtTrajectoryEnd(true, false, false, 0.06, 0.05));
  assert(shouldHoldAtTrajectoryEnd(true, false, false, 0.05, 0.05));
  assert(!shouldHoldAtTrajectoryEnd(true, true, false, 0.00, 0.05));
  assert(!shouldHoldAtTrajectoryEnd(true, false, true, 0.00, 0.05));

  // 通道模式关闭无条件周期重规划；碰撞与临近终点由FSM的独立分支处理。
  assert(!shouldPeriodicReplan(30.0, 3.0, false));
  assert(shouldPeriodicReplan(3.1, 3.0, true));
  assert(!shouldPeriodicReplan(2.9, 3.0, true));

  assert(!shouldReplanForCoveredFrontier(true, true, 1.0, 0.5));
  assert(shouldReplanForCoveredFrontier(true, false, 1.0, 0.5));
  assert(!shouldReplanForCoveredFrontier(false, false, 1.0, 0.5));

  // 初始 SEARCH_CORRIDOR 状态不能启动混合探索；必须收到入口完成触发。
  assert(!shouldActivateExternalExploration(true, false, false));
  assert(shouldActivateExternalExploration(true, true, false));
  assert(!shouldActivateExternalExploration(true, true, true));
  assert(!shouldActivateExternalExploration(false, true, false));
  return 0;
}
