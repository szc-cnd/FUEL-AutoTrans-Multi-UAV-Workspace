#include "exploration_manager/motion_direction_rules.h"

#include <cassert>

int main() {
  Eigen::Vector2d corridor(1.0, 0.0);
  Eigen::Vector2d recent(0.0, 1.0);
  Eigen::Vector2d yaw(-1.0, 0.0);
  assert((fast_planner::task_search::forwardReference(true, true, corridor, recent, yaw) - corridor).norm() < 1e-9);
  assert((fast_planner::task_search::forwardReference(false, true, corridor, recent, yaw) - recent).norm() < 1e-9);

  // 全局禁回只约束入口/最终出口平面；窄通道内靠近旧航迹本身不能否决横移或侧绕。
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
  assert(fast_planner::task_search::holdYawForLateralTranslation(0.0));
  assert(fast_planner::task_search::holdYawForLateralTranslation(0.70));
  assert(!fast_planner::task_search::holdYawForLateralTranslation(0.71));

  // A mapped corner may have only the outer wall continuous while the inner wall ends.
  assert(fast_planner::task_search::mappedContourSupportsTurn(true, 3, 0, 2));
  assert(fast_planner::task_search::mappedContourSupportsTurn(true, 0, 3, 2));
  // An isolated obstacle has no continuous wall contour and must remain a lateral detour.
  assert(!fast_planner::task_search::mappedContourSupportsTurn(true, 1, 0, 2));
  assert(!fast_planner::task_search::mappedContourSupportsTurn(false, 3, 0, 2));
  return 0;
}
