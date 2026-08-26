#include "exploration_manager/motion_direction_rules.h"
#include "path_searching/directional_progress_policy.h"

#include <cassert>
#include <limits>

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
  // 1-2m是零附加代价的优选带；过近和过远只受各自一侧的线性惩罚。
  assert(std::fabs(fast_planner::task_search::preferredDistanceCost(
                       1.0, 1.0, 2.0, 1.5, 1.2)) < 1e-9);
  assert(std::fabs(fast_planner::task_search::preferredDistanceCost(
                       2.0, 1.0, 2.0, 1.5, 1.2)) < 1e-9);
  assert(std::fabs(fast_planner::task_search::preferredDistanceCost(
                       0.5, 1.0, 2.0, 1.5, 1.2) - 0.75) < 1e-9);
  assert(std::fabs(fast_planner::task_search::preferredDistanceCost(
                       3.0, 1.0, 2.0, 1.5, 1.2) - 1.2) < 1e-9);
  // 扇区以稳定前进方向为零度中心，边界两侧的小角度候选不会被拆开。
  const double sector_width = 30.0 * M_PI / 180.0;
  const Eigen::Vector2d minus_five(
      std::cos(-5.0 * M_PI / 180.0), std::sin(-5.0 * M_PI / 180.0));
  const Eigen::Vector2d plus_five(
      std::cos(5.0 * M_PI / 180.0), std::sin(5.0 * M_PI / 180.0));
  const Eigen::Vector2d plus_forty(
      std::cos(40.0 * M_PI / 180.0), std::sin(40.0 * M_PI / 180.0));
  const int forward_sector = fast_planner::task_search::directionSector(
      Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(1.0, 0.0), sector_width);
  assert(fast_planner::task_search::directionSector(
             minus_five, Eigen::Vector2d(1.0, 0.0), sector_width) ==
         forward_sector);
  assert(fast_planner::task_search::directionSector(
             plus_five, Eigen::Vector2d(1.0, 0.0), sector_width) ==
         forward_sector);
  assert(fast_planner::task_search::directionSector(
             plus_forty, Eigen::Vector2d(1.0, 0.0), sector_width) !=
         forward_sector);
  assert(fast_planner::task_search::insideForwardHalfPlane(
      Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(1.0, 0.0)));
  assert(fast_planner::task_search::insideForwardHalfPlane(
      Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(1.0, 0.0)));
  assert(!fast_planner::task_search::insideForwardHalfPlane(
      Eigen::Vector2d(-0.01, 1.0), Eigen::Vector2d(1.0, 0.0)));

  // 九宫格中心投影在通道轴线上，不能跟着偏到一侧的无人机射线移动。
  const Eigen::Vector2d grid_center =
      fast_planner::task_search::corridorGridCenter(
          Eigen::Vector2d(3.75, 0.50), Eigen::Vector2d(0.95, 0.01),
          Eigen::Vector2d(1.0, 0.0));
  assert((grid_center - Eigen::Vector2d(3.75, 0.01)).norm() < 1e-9);

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

  // 转向查询不会自行消费锁存方向，只能在实际yaw稳定达标后显式释放。
  assert(turn_latch.arm(Eigen::Vector2d(0.0, 1.0)));
  assert(turn_latch.lockedDirection(turn_direction));
  assert(turn_latch.active());
  turn_latch.clear();
  assert(!turn_latch.lockedDirection(turn_direction));

  // 原地转向必须先连续静止；速度和稳定时间任一不满足都不能建立转向轨迹。
  assert(!fast_planner::task_search::stationaryTurnReady(0.13, 0.12, 0.20, 0.15));
  assert(!fast_planner::task_search::stationaryTurnReady(0.10, 0.12, 0.14, 0.15));
  assert(fast_planner::task_search::stationaryTurnReady(0.12, 0.12, 0.15, 0.15));
  assert(!fast_planner::task_search::stationaryTurnReady(
      std::numeric_limits<double>::quiet_NaN(), 0.12, 0.20, 0.15));

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

  return 0;
}
