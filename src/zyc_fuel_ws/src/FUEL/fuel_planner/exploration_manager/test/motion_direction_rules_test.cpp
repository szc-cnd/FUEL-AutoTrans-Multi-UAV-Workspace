#include "exploration_manager/motion_direction_rules.h"
#include "path_searching/directional_progress_policy.h"

#include <cassert>

int main() {
  Eigen::Vector2d corridor(1.0, 0.0);
  Eigen::Vector2d recent(0.0, 1.0);
  Eigen::Vector2d yaw(-1.0, 0.0);
  assert((fast_planner::task_search::forwardReference(true, true, corridor, recent, yaw) - corridor).norm() < 1e-9);
  assert((fast_planner::task_search::forwardReference(false, true, corridor, recent, yaw) - recent).norm() < 1e-9);

  // 入口/最终出口平面始终单向；局部横移或侧绕不能仅因靠近旧航迹被拒绝。
  assert(fast_planner::task_search::passesMissionBoundaryNoReturn(
      0.30, 0.10, false, 0.0));
  assert(!fast_planner::task_search::passesMissionBoundaryNoReturn(
      -0.11, 0.10, false, 0.0));
  assert(fast_planner::task_search::passesMissionBoundaryNoReturn(
      0.30, 0.10, true, -0.05));
  assert(!fast_planner::task_search::passesMissionBoundaryNoReturn(
      0.30, 0.10, true, -0.051));
  assert(fast_planner::task_search::boundaryPathRejectRequiresGoalSwitch());
  assert(std::fabs(fast_planner::task_search::viewpointDirectionScoreAdjustment(
                       1.0, 5.0, 5.0) + 5.0) < 1e-9);
  assert(std::fabs(fast_planner::task_search::viewpointDirectionScoreAdjustment(
                       -1.0, 5.0, 5.0) - 5.0) < 1e-9);
  assert(fast_planner::task_search::insideForwardHalfPlane(
      Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(1.0, 0.0)));
  assert(fast_planner::task_search::insideForwardHalfPlane(
      Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(1.0, 0.0)));
  assert(!fast_planner::task_search::insideForwardHalfPlane(
      Eigen::Vector2d(-0.01, 1.0), Eigen::Vector2d(1.0, 0.0)));
  // A* 搜索阶段允许纯横移和向前斜移，但不能先向旧通道退一步再绕障。
  assert(fast_planner::directional_progress::isAllowed(
      Eigen::Vector3d(3.2, 0.3, 0.5), Eigen::Vector3d(3.2, 1.1, 0.5),
      Eigen::Vector3d(1.0, 0.0, 0.0), 0.0));
  assert(fast_planner::directional_progress::isAllowed(
      Eigen::Vector3d(3.2, 0.3, 0.5), Eigen::Vector3d(3.4, 1.1, 0.5),
      Eigen::Vector3d(1.0, 0.0, 0.0), 0.0));
  assert(!fast_planner::directional_progress::isAllowed(
      Eigen::Vector3d(3.2, 0.3, 0.5), Eigen::Vector3d(3.0, 1.1, 0.5),
      Eigen::Vector3d(1.0, 0.0, 0.0), 0.0));
  // 确认左转后，最新拐点仍允许新通道内横移，但不能沿刚完成的+X通道退回去。
  assert(fast_planner::task_search::passesLatestTurnNoReturn(
      Eigen::Vector2d(5.0, 1.0), Eigen::Vector2d(5.0, 0.0),
      Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(0.0, 1.0), 0.20));
  assert(fast_planner::task_search::passesLatestTurnNoReturn(
      Eigen::Vector2d(4.80, 0.5), Eigen::Vector2d(5.0, 0.0),
      Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(0.0, 1.0), 0.20));
  assert(!fast_planner::task_search::passesLatestTurnNoReturn(
      Eigen::Vector2d(4.79, 0.0), Eigen::Vector2d(5.0, 0.0),
      Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(0.0, 1.0), 0.20));
  // 旧故障目标虽然在新方向上有投影，主体仍沿旧通道退回数米，必须拒绝。
  assert(!fast_planner::task_search::passesLatestTurnNoReturn(
      Eigen::Vector2d(2.55, 0.53), Eigen::Vector2d(5.44, -0.43),
      Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(0.0, 1.0), 0.20));
  // 105度真实弯道沿新轴线前进时允许自然落到旧方向锚点之后。
  const Eigen::Vector2d obtuse_outgoing(
      std::cos(105.0 * M_PI / 180.0), std::sin(105.0 * M_PI / 180.0));
  assert(fast_planner::task_search::passesLatestTurnNoReturn(
      Eigen::Vector2d(5.0, 0.0) + 1.0 * obtuse_outgoing,
      Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(1.0, 0.0),
      obtuse_outgoing, 0.20));
  assert(fast_planner::task_search::holdYawForLateralTranslation(0.0));
  assert(fast_planner::task_search::holdYawForLateralTranslation(0.70));
  assert(!fast_planner::task_search::holdYawForLateralTranslation(0.71));

  // 拐角近场允许只有外墙连续；远场还会单独要求新方向重新形成稳定双墙。
  assert(fast_planner::task_search::mappedContourSupportsTurn(true, 3, 0, 2));
  assert(fast_planner::task_search::mappedContourSupportsTurn(true, 0, 3, 2));
  // An isolated obstacle has no continuous wall contour and must remain a lateral detour.
  assert(!fast_planner::task_search::mappedContourSupportsTurn(true, 1, 0, 2));
  assert(!fast_planner::task_search::mappedContourSupportsTurn(false, 3, 0, 2));

  // 地图确认转弯后，位置恢复先读取新方向不能把偏航跟随状态消费掉。
  fast_planner::task_search::TurnYawFollowLatch turn_latch;
  assert(turn_latch.arm(Eigen::Vector2d(0.0, 1.0)));
  Eigen::Vector2d turn_direction;
  assert(turn_latch.directionForYaw(0.0, 15.0 * M_PI / 180.0,
                                    turn_direction));
  assert((turn_direction - Eigen::Vector2d(0.0, 1.0)).norm() < 1e-9);
  assert(turn_latch.active());

  // 接近新通道方向的这一帧仍跟随轨迹，随后才退出转弯状态。
  assert(turn_latch.directionForYaw(80.0 * M_PI / 180.0,
                                    15.0 * M_PI / 180.0, turn_direction));
  assert(!turn_latch.active());
  assert(!turn_latch.directionForYaw(M_PI_2, 15.0 * M_PI / 180.0,
                                     turn_direction));
  assert(!turn_latch.arm(Eigen::Vector2d::Zero()));

  // 分段转向查询不会自行消费锁存方向，只能在实际yaw稳定达标后显式释放。
  assert(turn_latch.arm(Eigen::Vector2d(0.0, 1.0)));
  assert(turn_latch.lockedDirection(turn_direction));
  assert(turn_latch.active());
  turn_latch.clear();
  assert(!turn_latch.lockedDirection(turn_direction));

  // 90度转向拆成不超过30度的连续小段，并正确处理跨越+/-PI的最短转向。
  const double max_yaw_step = 30.0 * M_PI / 180.0;
  assert(std::fabs(fast_planner::task_search::boundedYawStep(
                       0.0, M_PI_2, max_yaw_step) - max_yaw_step) < 1e-9);
  assert(std::fabs(fast_planner::task_search::boundedYawStep(
                       60.0 * M_PI / 180.0, M_PI_2, max_yaw_step) - M_PI_2) < 1e-9);
  assert(std::fabs(fast_planner::task_search::boundedYawStep(
                       170.0 * M_PI / 180.0, -170.0 * M_PI / 180.0,
                       max_yaw_step) - 190.0 * M_PI / 180.0) < 1e-9);

  // 分段建立后，只有机头明显偏离实际通道轴线才校正；该判断不创建新分段。
  assert(fast_planner::task_search::corridorYawCorrectionNeeded(
      Eigen::Vector2d(0.0, 1.0), 60.0 * M_PI / 180.0,
      15.0 * M_PI / 180.0));
  assert(!fast_planner::task_search::corridorYawCorrectionNeeded(
      Eigen::Vector2d(0.0, 1.0), 80.0 * M_PI / 180.0,
      15.0 * M_PI / 180.0));
  assert(fast_planner::task_search::corridorYawCorrectionNeeded(
      Eigen::Vector2d(-1.0, 0.0), -170.0 * M_PI / 180.0,
      5.0 * M_PI / 180.0));

  // 悬空障碍下探只豁免原地改变高度，带明显XY移动的斜向俯冲仍不能走该通道。
  assert(fast_planner::task_search::isStationaryVerticalMotion(
      Eigen::Vector3d(4.95, 1.28, 0.59),
      Eigen::Vector3d(4.95, 1.28, 0.10), 0.08));
  assert(!fast_planner::task_search::isStationaryVerticalMotion(
      Eigen::Vector3d(4.95, 1.28, 0.59),
      Eigen::Vector3d(5.10, 1.28, 0.10), 0.08));
  assert(!fast_planner::task_search::isStationaryVerticalMotion(
      Eigen::Vector3d(4.95, 1.28, 0.59),
      Eigen::Vector3d(4.95, 1.28, 0.59), 0.08));

  // 通道搜索中只有地图确认的真实转弯可以解除yaw锁定。
  assert(fast_planner::task_search::holdYawInCorridor(true, false));
  assert(!fast_planner::task_search::holdYawInCorridor(true, true));
  // 入口、出口和降落等非通道搜索状态沿用各自已有的yaw规则。
  assert(!fast_planner::task_search::holdYawInCorridor(false, false));

  // 只有约1m的近场侧绕空间不能被当成新通道。
  assert(!fast_planner::task_search::longRangeTurnViewSupported(
      1.0, 5, 1.0, 3, 0.8, 2, 0.8, 2, 2,
      2.0, 5, 3, 4));
  // 只有单侧长墙时可能只是圆柱边缘或障碍物后的旧墙，不能确认转弯。
  assert(!fast_planner::task_search::longRangeTurnViewSupported(
      3.5, 9, 3.2, 5, 0.8, 1, 0.8, 1, 1,
      2.0, 5, 3, 4));
  // 允许中间被障碍物遮挡，但远端必须重新形成宽度/中心稳定的双墙通道。
  assert(fast_planner::task_search::longRangeTurnViewSupported(
      3.5, 9, 3.2, 6, 3.0, 5, 3.0, 5, 4,
      2.0, 5, 3, 4));
  // 双墙虽然都有零散命中，但没有足够的同截面稳定配对，仍按普通绕障处理。
  assert(!fast_planner::task_search::longRangeTurnViewSupported(
      3.5, 9, 3.2, 5, 3.0, 4, 3.0, 3, 2,
      2.0, 5, 3, 4));
  return 0;
}
