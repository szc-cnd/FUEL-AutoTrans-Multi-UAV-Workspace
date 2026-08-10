// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <lkh_tsp_solver/lkh_interface.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>
#include <exploration_manager/task_search_manager.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {
}

FastExplorationManager::~FastExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

bool FastExplorationManager::pointInBox(const Vector3d& pt, const Vector3d& box_min,
                                        const Vector3d& box_max) const {
  return pt.x() >= box_min.x() && pt.x() <= box_max.x() && pt.y() >= box_min.y() &&
         pt.y() <= box_max.y() && pt.z() >= box_min.z() && pt.z() <= box_max.z();
}

bool FastExplorationManager::cameraOccupancyColumn(const Vector3d& point,
                                                   double reference_z) const {
  if (!sdf_map_) return false;
  // 2026-07-24: 相机障碍物触发只看相对当前高度的短柱，避免FAST-LIO绝对z漂移导致漏检。
  const double step = std::max(0.08, sdf_map_->getResolution());
  int occupied_layers = 0;
  for (double dz = -0.30; dz <= 0.30 + 1e-6; dz += step) {
    Vector3d probe(point.x(), point.y(), reference_z + dz);
    if (sdf_map_->isInMap(probe) &&
        sdf_map_->getOccupancy(probe) == SDFMap::OCCUPIED)
      ++occupied_layers;
  }
  return occupied_layers >= 2;
}

bool FastExplorationManager::cameraWallSupported(
    const Vector3d& point, const Eigen::Vector2d& travel_dir) const {
  if (!sdf_map_ || travel_dir.norm() < 1e-3) return false;
  // 2026-07-24: 沿局部通道方向持续占据的是墙；允许一个栅格横向误差以适应弯曲/离散墙面。
  const double step = std::max(0.10, sdf_map_->getResolution());
  const Eigen::Vector2d dir = travel_dir.normalized();
  const Eigen::Vector2d lateral(-dir.y(), dir.x());
  const double half_length = 0.5 * std::max(0.50, camera_wall_support_length_);
  int supported = 0;
  int total = 0;
  for (double along = -half_length; along <= half_length + 1e-6; along += step) {
    ++total;
    bool hit = false;
    for (double offset : { -step, 0.0, step }) {
      Vector3d probe = point;
      probe.head<2>() += along * dir + offset * lateral;
      if (cameraOccupancyColumn(probe, point.z())) {
        hit = true;
        break;
      }
    }
    if (hit) ++supported;
  }
  return total > 0 &&
         static_cast<double>(supported) / static_cast<double>(total) >=
             camera_wall_support_ratio_;
}

bool FastExplorationManager::cameraObstacleAlreadyScanned(
    const Vector3d& obstacle_center) const {
  if (camera_obstacle_scan_active_ &&
      (camera_active_obstacle_.head<2>() - obstacle_center.head<2>()).norm() <
          camera_obstacle_dedup_radius_)
    return true;
  for (const auto& scanned : camera_scanned_obstacles_) {
    if ((scanned.head<2>() - obstacle_center.head<2>()).norm() <
        camera_obstacle_dedup_radius_)
      return true;
  }
  return false;
}

bool FastExplorationManager::detectCameraObstacle(
    const Vector3d& pos, const Eigen::Vector2d& travel_dir,
    Vector3d& obstacle_center) const {
  if (!sdf_map_ || travel_dir.norm() < 1e-3) return false;
  const Eigen::Vector2d dir = travel_dir.normalized();
  const Eigen::Vector2d lateral(-dir.y(), dir.x());
  const double step = std::max(0.10, sdf_map_->getResolution());
  vector<Vector3d> residual_points;

  // 2026-07-24: 只检查前方近场；连续墙带被剔除，剩余柱子/箱体/向通道内突出的占据簇才触发相机环扫。
  for (double forward = camera_obstacle_min_forward_;
       forward <= camera_obstacle_max_forward_ + 1e-6; forward += step) {
    for (double side = -camera_obstacle_lateral_range_;
         side <= camera_obstacle_lateral_range_ + 1e-6; side += step) {
      Vector3d sample = pos;
      sample.head<2>() += forward * dir + side * lateral;
      if (!cameraOccupancyColumn(sample, pos.z())) continue;
      if (cameraWallSupported(sample, dir)) continue;
      residual_points.push_back(sample);
    }
  }
  if (residual_points.empty()) return false;

  // 2026-07-24: 以最近剩余点为种子做轻量连通聚合，至少三个栅格才接受，过滤零散点云噪声。
  auto nearest_it = std::min_element(
      residual_points.begin(), residual_points.end(),
      [&pos](const Vector3d& lhs, const Vector3d& rhs) {
        return (lhs.head<2>() - pos.head<2>()).squaredNorm() <
               (rhs.head<2>() - pos.head<2>()).squaredNorm();
      });
  const Vector3d seed = *nearest_it;
  Vector3d sum = Vector3d::Zero();
  int cluster_count = 0;
  const double cluster_radius = std::max(0.30, 2.5 * step);
  for (const auto& point : residual_points) {
    if ((point.head<2>() - seed.head<2>()).norm() <= cluster_radius) {
      sum += point;
      ++cluster_count;
    }
  }
  if (cluster_count < 3) return false;
  obstacle_center = sum / static_cast<double>(cluster_count);
  obstacle_center.z() = pos.z();
  return !cameraObstacleAlreadyScanned(obstacle_center);
}

void FastExplorationManager::updateMissionRegionState(const Vector3d& pos) {
  if (mission_entered_search_region_) return;
  if (pointInBox(pos, ep_->mission_search_region_min_, ep_->mission_search_region_max_)) {
    // 2026-07-08 19:26: 一旦机体真正进入比赛搜索区，就锁存状态，后续不再退回“起飞区引导”模式。
    mission_entered_search_region_ = true;
    ROS_INFO("[mission_exploration] entered search region at %.2f %.2f %.2f.", pos.x(), pos.y(),
             pos.z());
  }
}

bool FastExplorationManager::shouldUseMissionEntryTransit(const Vector3d& pos) const {
  if (!ep_->mission_use_entry_transit_ || mission_entered_search_region_) return false;
  // 2026-07-10: 动态门平面锁只会在 corridor_search_manager 完成找门/穿门阶段后发布。
  // 此时 FUEL 已经处于门后搜索阶段，禁止继续使用旧的静态 entry_goal，避免穿门后又被拉回起飞区。
  if (ep_->mission_use_workspace_lock_ && mission_workspace_lock_received_) return false;
  return (pos - ep_->mission_entry_goal_).norm() > ep_->mission_entry_arrive_dist_;
}

void FastExplorationManager::workspaceLockCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  mission_workspace_origin_ =
      Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  const auto& q = msg->pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  mission_workspace_dir_ = Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
  mission_workspace_lock_received_ = true;
  // 2026-07-13: 入口坐标只作为通道坐标系和初始推进参考，深处拐弯由任务覆盖历史决定。
  if (task_search_manager_)
    task_search_manager_->setCorridorFrame(mission_workspace_origin_, mission_workspace_dir_);
  ROS_WARN_THROTTLE(1.0,
                    "[workspace_lock] received door plane origin=(%.2f, %.2f) yaw=%.1fdeg.",
                    mission_workspace_origin_.x(), mission_workspace_origin_.y(),
                    yaw * 180.0 / M_PI);
}

bool FastExplorationManager::pointInsideWorkspaceLock(const Vector3d& pt) const {
  if (!ep_->mission_global_no_return_ || !ep_->mission_use_workspace_lock_ ||
      !mission_workspace_lock_received_)
    return true;
  // 2026-07-23: 动态workspace_lock描述的是入口门，不是整个U形赛道的世界半平面。
  // TaskSearchManager确认已穿入口后，该锁对frontier、fallback和A*路径统一退休。
  if (task_search_manager_ && !task_search_manager_->entryWorkspaceLockActive())
    return true;

  const Vector3d rel = pt - mission_workspace_origin_;
  const double progress = rel.x() * mission_workspace_dir_.x() + rel.y() * mission_workspace_dir_.y();
  return progress >= -ep_->mission_door_back_margin_;
}

bool FastExplorationManager::pathInsideWorkspaceLock(const vector<Vector3d>& path) const {
  if (!ep_->mission_global_no_return_ || !ep_->mission_use_workspace_lock_ ||
      !mission_workspace_lock_received_)
    return true;
  for (const auto& pt : path) {
    if (!pointInsideWorkspaceLock(pt)) return false;
  }
  return true;
}

void FastExplorationManager::applyMissionFrontierFilter() {
  const auto old_frontiers = ed_->frontiers_;
  const auto old_boxes = ed_->frontier_boxes_;
  const auto old_points = ed_->points_;
  const auto old_yaws = ed_->yaws_;
  const auto old_averages = ed_->averages_;

  vector<vector<Vector3d>> filtered_frontiers;
  vector<pair<Vector3d, Vector3d>> filtered_boxes;
  vector<Vector3d> filtered_points;
  vector<double> filtered_yaws;
  vector<Vector3d> filtered_averages;
  int rejected_workspace = 0;
  int rejected_takeoff = 0;
  int rejected_region = 0;
  // 2026-07-23: 日志和“空集合是否硬锁存”必须使用实际状态，不能继续只看参数开关；
  // 否则入口已退休后仍会把空frontier误当成门外硬拒绝。
  const bool workspace_lock_active =
      ep_->mission_use_workspace_lock_ && mission_workspace_lock_received_ &&
      (!task_search_manager_ || task_search_manager_->entryWorkspaceLockActive());

  // 2026-07-10: 收到动态门平面后，门后搜索方向由 workspace_lock 决定；旧的静态 search_region
  // 只适合早期固定地图验证，不能再误杀当前门方向上的候选点。
  const bool use_static_search_region =
      ep_->mission_prefer_search_region_frontiers_ && !mission_workspace_lock_received_;

  bool has_search_region_candidate = false;
  if (use_static_search_region) {
    for (size_t i = 0; i < old_points.size(); ++i) {
      if (pointInBox(old_points[i], ep_->mission_search_region_min_,
                     ep_->mission_search_region_max_)) {
        has_search_region_candidate = true;
        break;
      }
    }
  }

  for (size_t i = 0; i < old_points.size(); ++i) {
    const bool in_takeoff_box = ep_->mission_use_takeoff_exclusion_box_ &&
                                pointInBox(old_points[i], ep_->mission_takeoff_exclusion_min_,
                                           ep_->mission_takeoff_exclusion_max_);
    const bool in_search_region =
        pointInBox(old_points[i], ep_->mission_search_region_min_, ep_->mission_search_region_max_);
    // 2026-07-10: 正常情况下 averages 与 viewpoints 等长；这里兜底防止异常日志/旧数据导致越界。
    const Vector3d frontier_ref = i < old_averages.size() ? old_averages[i] : old_points[i];
    const bool inside_workspace =
        pointInsideWorkspaceLock(old_points[i]) && pointInsideWorkspaceLock(frontier_ref);

    if (!inside_workspace) {
      ++rejected_workspace;
      continue;
    }
    if (in_takeoff_box && (mission_entered_search_region_ || has_search_region_candidate)) {
      ++rejected_takeoff;
      continue;
    }
    if (use_static_search_region && has_search_region_candidate && !in_search_region) {
      ++rejected_region;
      continue;
    }

    filtered_frontiers.push_back(old_frontiers[i]);
    filtered_boxes.push_back(old_boxes[i]);
    filtered_points.push_back(old_points[i]);
    filtered_yaws.push_back(old_yaws[i]);
    filtered_averages.push_back(frontier_ref);
  }

  if (!filtered_points.empty() || workspace_lock_active) {
    // 2026-07-10: workspace_lock 是硬约束，即使过滤后为空也不能回退到门外 frontier；
    // 空集合由上层 hold_on_no_frontier 处理，避免 FUEL 为补起飞区地图掉头出门。
    ed_->frontiers_.swap(filtered_frontiers);
    ed_->frontier_boxes_.swap(filtered_boxes);
    ed_->points_.swap(filtered_points);
    ed_->yaws_.swap(filtered_yaws);
    ed_->averages_.swap(filtered_averages);
  }
  ROS_WARN_THROTTLE(1.0,
                    "[workspace_lock] frontier kept=%zu rejected_workspace=%d rejected_takeoff=%d "
                    "rejected_region=%d lock=%d received=%d static_region=%d.",
                    ed_->points_.size(), rejected_workspace, rejected_takeoff, rejected_region,
                    static_cast<int>(workspace_lock_active),
                    static_cast<int>(mission_workspace_lock_received_),
                    static_cast<int>(use_static_search_region));
}

bool FastExplorationManager::buildMissionForwardFallback(const Vector3d& pos, double cur_yaw,
                                                         Vector3d& next_pos,
                                                         double& next_yaw) {
  if (!ep_->mission_use_forward_fallback_ || !ep_->mission_use_workspace_lock_ ||
      !mission_workspace_lock_received_) {
    return false;
  }

  const double step = std::max(0.2, ep_->mission_forward_fallback_step_);
  const double max_dist = std::max(step, ep_->mission_forward_fallback_max_);
  // 2026-07-13: 在通道深处按实际运动方向向前、侧向和转弯方向尝试，解决固定门方向在折线通道末端卡死。
  auto directions = task_search_manager_
                              ? task_search_manager_->recoveryDirections(cur_yaw)
                              : vector<Vector3d>{mission_workspace_dir_};
  const Vector3d recovery_forward = task_search_manager_
                                        ? task_search_manager_->recoveryForwardDirection(cur_yaw)
                                        : Vector3d(std::cos(cur_yaw), std::sin(cur_yaw), 0.0);
  // 2026-07-28: 所有前向方向仍保持任务层顺序；短回撤候选改按目标点ESDF净空降序，
  // 让无人机优先退向通道中部，而不是因为±135度枚举顺序固定地退向某一侧墙。
  std::stable_sort(directions.begin(), directions.end(), [&](const Vector3d& lhs,
                                                              const Vector3d& rhs) {
    const bool lhs_back = task_search_manager_ &&
        task_search_manager_->isRecoveryDirectionBackward(lhs, cur_yaw);
    const bool rhs_back = task_search_manager_ &&
        task_search_manager_->isRecoveryDirectionBackward(rhs, cur_yaw);
    if (lhs_back != rhs_back) return !lhs_back;
    if (!lhs_back) return false;
    const double probe_dist = std::max(0.20, short_backtrack_max_distance_);
    const Vector3d lhs_probe = pos + lhs.normalized() * probe_dist;
    const Vector3d rhs_probe = pos + rhs.normalized() * probe_dist;
    return sdf_map_->getDistance(lhs_probe) > sdf_map_->getDistance(rhs_probe);
  });
  // 2026-07-28: 正常向前恢复后完全解锁；若仍被困，冷却后最多再尝试一次且必须换方向，
  // 既允许视频中的“小退再绕”，又不能把多个0.45m回撤串成同方向持续倒飞。
  if (short_backtrack_latched_ &&
      (pos - short_backtrack_release_origin_).dot(short_backtrack_forward_dir_) >=
          short_backtrack_forward_release_) {
    short_backtrack_latched_ = false;
    short_backtrack_chain_count_ = 0;
    short_backtrack_last_dir_.setZero();
    ROS_WARN("[mission_exploration] short backtrack lock released after %.2fm forward recovery.",
             short_backtrack_forward_release_);
  }
  const bool timed_direction_change_retry =
      short_backtrack_latched_ && short_backtrack_chain_count_ < short_backtrack_max_chain_ &&
      !short_backtrack_last_time_.isZero() &&
      (ros::Time::now() - short_backtrack_last_time_).toSec() >= short_backtrack_retry_cooldown_;
  // 2026-07-23: 记录前向扇区各级拒绝原因，区分“方向没试到”和真实无安全路径，
  // 避免再次只看到笼统的no safe fallback而无法判断卡在哪一层。
  int rejected_workspace = 0;
  int rejected_progress = 0;
  int rejected_position = 0;
  int rejected_astar = 0;
  int rejected_path = 0;
  // 2026-07-28: 平面被低障碍截断后必须主动枚举上方自由层，不再把Z永久锁在当前高度。
  std::vector<double> recovery_heights{
      task_search_manager_ ? task_search_manager_->clampSearchHeight(pos.z()) : pos.z()};
  if (task_search_manager_) {
    const double max_recovery_z = task_search_manager_->clampSearchHeight(1e6);
    for (double z = recovery_heights.front() + 0.30; z <= max_recovery_z + 1e-3;
         z += 0.30)
      recovery_heights.push_back(std::min(z, max_recovery_z));
    // 2026-07-28: 0.30m离散步长未必恰好落在上限，显式补入最高可用层。
    if (max_recovery_z - recovery_heights.back() > 0.05)
      recovery_heights.push_back(max_recovery_z);
  }
  for (const auto& direction : directions) {
    const bool short_backtrack = task_search_manager_ &&
        task_search_manager_->isRecoveryDirectionBackward(direction, cur_yaw);
    if (short_backtrack && short_backtrack_latched_) {
      if (!timed_direction_change_retry) continue;
      const double min_change_rad = short_backtrack_min_direction_change_deg_ * M_PI / 180.0;
      // 2026-07-28: 第二次回撤与上一次夹角不足阈值时拒绝，硬性防止沿一条线继续往后退。
      if (short_backtrack_last_dir_.norm() > 1e-3 &&
          direction.normalized().dot(short_backtrack_last_dir_.normalized()) >
              std::cos(min_change_rad))
        continue;
    }
    const double direction_max_dist = short_backtrack
        ? std::min(max_dist, std::max(step, short_backtrack_max_distance_))
        : max_dist;
    for (double dist = direction_max_dist; dist >= step - 1e-3; dist -= step) {
      for (const double recovery_z : recovery_heights) {
      Vector3d candidate = pos + direction * dist;
      candidate.z() = recovery_z;
      if (!pointInsideWorkspaceLock(candidate)) {
        ++rejected_workspace;
        continue;
      }
      if (task_search_manager_ && !task_search_manager_->isRecoveryCandidateUseful(candidate)) {
        ++rejected_progress;
        continue;
      }
      // 2026-07-13: 恢复点不能只检查中心栅格，必须容纳 0.15m 膨胀外的完整 Iris 足迹。
      if (!planner_manager_->isPositionSafe(candidate) ||
          sdf_map_->getOccupancy(candidate) == SDFMap::UNKNOWN) {
        ++rejected_position;
        continue;
      }
      // 2026-07-28: 短回撤必须提升或至少维持当前ESDF净空；拒绝虽然终点可用、
      // 但实际继续靠近墙/柱子的斜向回撤，保证恢复动作服务于回到通道中部。
      if (short_backtrack &&
          sdf_map_->getDistance(candidate) + 0.02 < sdf_map_->getDistance(pos)) {
        ++rejected_position;
        continue;
      }

      planner_manager_->path_finder_->reset();
      if (planner_manager_->path_finder_->search(pos, candidate) != Astar::REACH_END) {
        ++rejected_astar;
        continue;
      }
      const auto candidate_path = planner_manager_->path_finder_->getPath();
      // 2026-07-13: A* 中心线可达不代表旋翼包络可达，危险路径直接换下一个恢复方向。
      if (!pathInsideWorkspaceLock(candidate_path) ||
          !planner_manager_->isPathSafe(candidate_path) ||
          (task_search_manager_ &&
           !task_search_manager_->isRecoveryPathAllowed(candidate_path, short_backtrack))) {
        ++rejected_path;
        continue;
      }

      next_pos = candidate;
      next_yaw = std::atan2(direction.y(), direction.x());
      if (task_search_manager_) task_search_manager_->recordSelectedGoal(next_pos);
      if (short_backtrack) {
        short_backtrack_latched_ = true;
        short_backtrack_release_origin_ = next_pos;
        short_backtrack_forward_dir_ = recovery_forward.normalized();
        short_backtrack_last_dir_ = direction.normalized();
        short_backtrack_last_time_ = ros::Time::now();
        ++short_backtrack_chain_count_;
        ROS_ERROR("[mission_exploration] LIMITED SHORT BACKTRACK %d/%d target=(%.2f, %.2f, %.2f) "
                  "dist=%.2fm; next retry needs %.1fs cooldown and %.1fdeg direction change.",
                  short_backtrack_chain_count_, short_backtrack_max_chain_, next_pos.x(),
                  next_pos.y(), next_pos.z(), dist, short_backtrack_retry_cooldown_,
                  short_backtrack_min_direction_change_deg_);
      }
      ROS_WARN_THROTTLE(0.5,
                        "[task_search] 3D recovery target=(%.2f, %.2f, %.2f) dist=%.2f yaw=%.1fdeg.",
                        next_pos.x(), next_pos.y(), next_pos.z(), dist,
                        next_yaw * 180.0 / M_PI);
      return true;
      }
    }
  }

  // 2026-07-28: 日志同时给出短回撤锁状态，区分“没有安全回撤点”和“已回撤、等待重新前进”。
  ROS_WARN_THROTTLE(
      1.0,
      "[mission_exploration] no safe recovery fallback: directions=%zu short_backtrack_locked=%d "
      "workspace=%d progress=%d position=%d astar=%d path=%d.",
      directions.size(), static_cast<int>(short_backtrack_latched_), rejected_workspace,
      rejected_progress, rejected_position, rejected_astar, rejected_path);
  next_yaw = cur_yaw;
  return false;
}

void FastExplorationManager::initialize(ros::NodeHandle& nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // 2026-07-13: 初始化比赛第二阶段任务搜索层，颜色/二维码/温度接口与 frontier 生命周期解耦。
  task_search_manager_.reset(new TaskSearchManager);
  task_search_manager_->initialize(nh);
  // 2026-07-28: 短回撤仅用于前向失败后的有限脱困；距离、冷却、次数和换向角均可在launch调节。
  nh.param("mission/short_backtrack_max_distance", short_backtrack_max_distance_, 0.45);
  nh.param("mission/short_backtrack_forward_release", short_backtrack_forward_release_, 0.60);
  nh.param("mission/short_backtrack_retry_cooldown", short_backtrack_retry_cooldown_, 1.5);
  nh.param("mission/short_backtrack_max_chain", short_backtrack_max_chain_, 2);
  nh.param("mission/short_backtrack_min_direction_change_deg",
           short_backtrack_min_direction_change_deg_, 35.0);
  // 2026-07-13: 出口推断必须读取与规划器完全相同的累计 SDFMap，不能退化为实时雷达点云判断。
  task_search_manager_->setMap(sdf_map_);
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);
  nh.param("cmu_exploration", cmu_exploration, false);
  // 2026-07-08 19:26: 为比赛 2.1 增加最小任务层参数。
  // 这层不替代后续二维码/颜色/温度识别，但先把“20s 进搜索区、起飞区禁搜、不能因 frontier 清空就结束”落实。
  nh.param("mission/use_entry_transit", ep_->mission_use_entry_transit_, false);
  nh.param("mission/prefer_entry_heading", ep_->mission_prefer_entry_heading_, true);
  nh.param("mission/use_takeoff_exclusion_box", ep_->mission_use_takeoff_exclusion_box_, false);
  nh.param("mission/prefer_search_region_frontiers", ep_->mission_prefer_search_region_frontiers_,
           false);
  nh.param("mission/hold_on_no_frontier", ep_->mission_hold_on_no_frontier_, false);
  // 2026-07-10: FUEL 接管后锁在门内作业区，门外起飞区地图只保留避障用途，不再贡献探索收益。
  nh.param("mission/use_workspace_lock", ep_->mission_use_workspace_lock_, true);
  nh.param("mission/global_no_return", ep_->mission_global_no_return_, true);
  // 2026-07-16: 相机接入后可打开该接口恢复viewpoint航向；当前Mid360模式关闭，优先实际平移。
  nh.param("mission/use_camera_viewpoint_yaw", use_camera_viewpoint_yaw_, false);
  // 2026-07-24: 全程连续环扫改为调试回退项；比赛默认清晰直段轻摆头、新障碍物触发快速环扫。
  double camera_head_sweep_amplitude_deg = 30.0;
  double camera_clear_sweep_amplitude_deg = 35.0;
  double camera_turn_threshold_deg = 30.0;
  double camera_head_scan_direction = 1.0;
  nh.param("mission/camera_head_sweep/enabled", camera_head_sweep_enabled_, true);
  nh.param("mission/camera_head_sweep/full_rotation", camera_head_full_rotation_, false);
  nh.param("mission/camera_head_sweep/amplitude_deg", camera_head_sweep_amplitude_deg, 30.0);
  nh.param("mission/camera_head_sweep/period", camera_head_sweep_period_, 10.0);
  nh.param("mission/camera_head_sweep/direction", camera_head_scan_direction, 1.0);
  nh.param("mission/camera_obstacle_scan/enabled", camera_obstacle_scan_enabled_, true);
  nh.param("mission/camera_obstacle_scan/clear_sweep_amplitude_deg",
           camera_clear_sweep_amplitude_deg, 35.0);
  nh.param("mission/camera_obstacle_scan/clear_sweep_period",
           camera_clear_sweep_period_, 3.5);
  nh.param("mission/camera_obstacle_scan/full_rotation_period",
           camera_obstacle_scan_period_, 6.0);
  nh.param("mission/camera_obstacle_scan/turn_threshold_deg",
           camera_turn_threshold_deg, 30.0);
  nh.param("mission/camera_obstacle_scan/turn_hold_time",
           camera_turn_hold_time_, 1.2);
  nh.param("mission/camera_obstacle_scan/min_forward",
           camera_obstacle_min_forward_, 0.55);
  nh.param("mission/camera_obstacle_scan/max_forward",
           camera_obstacle_max_forward_, 1.60);
  nh.param("mission/camera_obstacle_scan/lateral_range",
           camera_obstacle_lateral_range_, 0.90);
  nh.param("mission/camera_obstacle_scan/dedup_radius",
           camera_obstacle_dedup_radius_, 0.55);
  nh.param("mission/camera_obstacle_scan/wall_support_length",
           camera_wall_support_length_, 0.90);
  nh.param("mission/camera_obstacle_scan/wall_support_ratio",
           camera_wall_support_ratio_, 0.68);
  camera_head_sweep_amplitude_rad_ =
      std::max(0.0, std::min(75.0, camera_head_sweep_amplitude_deg)) * M_PI / 180.0;
  camera_head_sweep_period_ = std::max(6.0, camera_head_sweep_period_);
  camera_head_scan_direction_ = camera_head_scan_direction < 0.0 ? -1.0 : 1.0;
  camera_clear_sweep_amplitude_rad_ =
      std::max(0.0, std::min(65.0, camera_clear_sweep_amplitude_deg)) * M_PI / 180.0;
  camera_clear_sweep_period_ = std::max(2.5, camera_clear_sweep_period_);
  camera_obstacle_scan_period_ = std::max(4.0, camera_obstacle_scan_period_);
  camera_turn_threshold_rad_ =
      std::max(15.0, std::min(70.0, camera_turn_threshold_deg)) * M_PI / 180.0;
  camera_turn_hold_time_ = std::max(0.5, camera_turn_hold_time_);
  camera_obstacle_min_forward_ = std::max(0.30, camera_obstacle_min_forward_);
  camera_obstacle_max_forward_ =
      std::max(camera_obstacle_min_forward_ + 0.30, camera_obstacle_max_forward_);
  camera_obstacle_lateral_range_ = std::max(0.35, camera_obstacle_lateral_range_);
  camera_obstacle_dedup_radius_ = std::max(0.30, camera_obstacle_dedup_radius_);
  camera_wall_support_length_ = std::max(0.50, camera_wall_support_length_);
  camera_wall_support_ratio_ =
      std::max(0.45, std::min(0.90, camera_wall_support_ratio_));
  camera_head_sweep_start_ = ros::Time(0);
  camera_turning_until_ = ros::Time(0);
  camera_obstacle_scan_start_ = ros::Time(0);
  // 2026-07-10: 门后第二阶段以“向通道/作业区内部推进”为主，frontier 只作为建图收益参考。
  nh.param("mission/use_forward_progress_bias", ep_->mission_use_forward_progress_bias_, true);
  nh.param("mission/use_forward_fallback", ep_->mission_use_forward_fallback_, true);
  // 2026-07-22: 远端frontier只提供方向，实际轨迹每次最多发布一段短安全路线点。
  nh.param("mission/max_route_segment_length", ep_->mission_max_route_segment_length_, 0.80);
  nh.param("mission/entry_arrive_dist", ep_->mission_entry_arrive_dist_, 0.6);
  nh.param("mission/entry_yaw", ep_->mission_entry_yaw_, 0.0);
  nh.param("mission/door_back_margin", ep_->mission_door_back_margin_, 0.25);
  nh.param("mission/forward_progress_weight", ep_->mission_forward_progress_weight_, 1.8);
  nh.param("mission/forward_min_gain", ep_->mission_forward_min_gain_, 0.25);
  nh.param("mission/forward_fallback_step", ep_->mission_forward_fallback_step_, 0.45);
  nh.param("mission/forward_fallback_max", ep_->mission_forward_fallback_max_, 1.8);
  std::string workspace_lock_topic;
  nh.param("mission/workspace_lock_topic", workspace_lock_topic,
           std::string("/corridor_search/workspace_lock"));
  std::vector<double> mission_entry_goal{0.0, 0.0, 0.8};
  std::vector<double> takeoff_exclusion_min{-100.0, -100.0, -100.0};
  std::vector<double> takeoff_exclusion_max{100.0, 100.0, 100.0};
  std::vector<double> search_region_min{-100.0, -100.0, -100.0};
  std::vector<double> search_region_max{100.0, 100.0, 100.0};
  nh.getParam("mission/entry_goal", mission_entry_goal);
  nh.getParam("mission/takeoff_exclusion_min", takeoff_exclusion_min);
  nh.getParam("mission/takeoff_exclusion_max", takeoff_exclusion_max);
  nh.getParam("mission/search_region_min", search_region_min);
  nh.getParam("mission/search_region_max", search_region_max);
  if (mission_entry_goal.size() != 3 || takeoff_exclusion_min.size() != 3 ||
      takeoff_exclusion_max.size() != 3 || search_region_min.size() != 3 ||
      search_region_max.size() != 3) {
    ROS_ERROR("[mission_exploration] mission box/vector params must all have exactly 3 elements.");
    mission_entry_goal = {0.0, 0.0, 0.8};
    takeoff_exclusion_min = {-100.0, -100.0, -100.0};
    takeoff_exclusion_max = {100.0, 100.0, 100.0};
    search_region_min = {-100.0, -100.0, -100.0};
    search_region_max = {100.0, 100.0, 100.0};
  }
  ep_->mission_entry_goal_ =
      Vector3d(mission_entry_goal[0], mission_entry_goal[1], mission_entry_goal[2]);
  ep_->mission_takeoff_exclusion_min_ =
      Vector3d(takeoff_exclusion_min[0], takeoff_exclusion_min[1], takeoff_exclusion_min[2]);
  ep_->mission_takeoff_exclusion_max_ =
      Vector3d(takeoff_exclusion_max[0], takeoff_exclusion_max[1], takeoff_exclusion_max[2]);
  ep_->mission_search_region_min_ =
      Vector3d(search_region_min[0], search_region_min[1], search_region_min[2]);
  ep_->mission_search_region_max_ =
      Vector3d(search_region_max[0], search_region_max[1], search_region_max[2]);
  workspace_lock_sub_ =
      nh.subscribe(workspace_lock_topic, 1, &FastExplorationManager::workspaceLockCallback, this);

  endpose_pub_ = nh.advertise<geometry_msgs::Point>("/chen/plan_pose", 10);
  cmu_pose_pub_ = nh.advertise<geometry_msgs::PointStamped>("/way_point", 10);
  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // planner_manager_->path_finder_->max_search_time_ = 0.05;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 1\n";

  // Analysis
  // ofstream fout;
  // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
  // fout.close();
}

int FastExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;
  ed_->views_.clear();
  ed_->global_tour_.clear();

  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  updateMissionRegionState(pos);
  // 2026-07-22: 真实航迹已由FSM里程计回调连续写入；这里的pos可能是未来重规划起点，禁止重复记入历史。

  Vector3d next_pos;
  double next_yaw = yaw[0];
  bool use_forced_entry_target = false;
  bool use_stage3_target = false;
  if (task_search_manager_ && task_search_manager_->landingRequested()) {
    // 2026-07-13: AUTO.LAND 已由控制器接管后，探索器不再发布新的平移轨迹。
    ROS_WARN_THROTTLE(1.0, "[exit_mission] landing is active; exploration planning stopped.");
    return NO_FRONTIER;
  }
  if (shouldUseMissionEntryTransit(pos)) {
    // 2026-07-08 19:26: 在真正进入搜索区前，先强制把无人机送到门口/作业区入口，不再允许 frontier 收益把它绑在起飞区。
    use_forced_entry_target = true;
    next_pos = ep_->mission_entry_goal_;
    next_yaw = ep_->mission_prefer_entry_heading_ ? atan2(next_pos.y() - pos.y(), next_pos.x() - pos.x())
                                                  : ep_->mission_entry_yaw_;
    ROS_WARN_THROTTLE(1.0,
                      "[mission_exploration] force entry transit to %.2f %.2f %.2f before frontier search.",
                      next_pos.x(), next_pos.y(), next_pos.z());
  }
  if (!use_forced_entry_target && task_search_manager_ && task_search_manager_->stage3Active()) {
    // 2026-07-16: 只有已经正式进入第三阶段才在frontier搜索前取终点目标；搜索期必须先看本轮地图覆盖。
    use_stage3_target =
        task_search_manager_->buildStage3Goal(pos, yaw[0], false, next_pos, next_yaw);
  }

  // Search frontiers and group them into clusters
  frontier_finder_->searchFrontiers();

  double frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
  frontier_finder_->computeFrontiersToVisit();
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
  // 2026-07-20: 不能仅凭原始frontier非空就取消搜索耗尽计时；全部候选都被重复访问、
  // 失败冷却或门平面约束过滤时并没有可执行路径，旧逻辑会让FSM原地空转十几秒。
  if (!use_forced_entry_target && !use_stage3_target && task_search_manager_ &&
      task_search_manager_->buildStage3Goal(pos, yaw[0], false, next_pos, next_yaw)) {
    // 2026-07-16: 搜索期仅在三个任务目标确实全部找到时允许主动切换；普通出口推测不能抢占frontier。
    use_stage3_target = true;
  }
  const bool use_mission_filtered_direct_view =
      !use_forced_entry_target && !use_stage3_target &&
      (ep_->mission_use_takeoff_exclusion_box_ || ep_->mission_prefer_search_region_frontiers_ ||
       ep_->mission_use_workspace_lock_);
  bool use_mission_forward_fallback = false;
  if (!use_forced_entry_target && !use_stage3_target) {
    applyMissionFrontierFilter();
  }

  if (!use_forced_entry_target && !use_stage3_target &&
      (ed_->frontiers_.empty() || ed_->points_.empty())) {
    ROS_WARN("No coverable frontier after mission filter.");
    // 2026-07-13: 搜索候选耗尽时优先进入地图出口阶段，不能继续在通道内盲目前探。
    if (task_search_manager_ &&
        task_search_manager_->buildStage3Goal(pos, yaw[0], true, next_pos, next_yaw)) {
      use_stage3_target = true;
      ed_->global_tour_ = {pos, next_pos};
      ed_->refined_tour_.clear();
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();
    // 2026-07-20: 已确认远端终点时，无frontier分支同样不能用无约束前探绕过
    // distance_to_exit保护；终点未确认时仍完整保留原前探和U形掉头能力。
    } else if ((!task_search_manager_ ||
                !task_search_manager_->finalExitFrontierGuardActive()) &&
               buildMissionForwardFallback(pos, yaw[0], next_pos, next_yaw)) {
      // 2026-07-10: 门后 frontier 被过滤为空时，先沿动态门方向短距离前探，给雷达制造新的可见地图和搜索空间。
      use_mission_forward_fallback = true;
      ed_->global_tour_ = {pos, next_pos};
      ed_->refined_tour_.clear();
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();
    } else {
      if (ep_->mission_hold_on_no_frontier_) {
        // 2026-07-08 19:26: 比赛任务尚未接入“目标已全部找到”的外部完成信号前，先禁止 exploration 因 frontier 见底而直接 FINISH。
        return FAIL;
      }
      return NO_FRONTIER;
    }
  }

  // 2026-07-13: 门后由任务覆盖和重复访问选点；现有 frontier 全是已搜索局部点时主动切换多方向恢复。
  int task_candidate_idx = -1;
  if (!use_forced_entry_target && !use_stage3_target && use_mission_filtered_direct_view &&
      !use_mission_forward_fallback && task_search_manager_ && task_search_manager_->enabled()) {
    task_candidate_idx = task_search_manager_->selectSearchCandidate(
        ed_->points_, ed_->yaws_, ed_->frontiers_, pos, yaw[0]);
    if (task_candidate_idx < 0) {
      // 2026-07-20: 原始frontier存在但没有任何可执行候选时也启动配置时长的耗尽确认；
      // 优先使用已经稳定确认并锁存的最终出口，确认期间才允许安全短步恢复。
      if (task_search_manager_->buildStage3Goal(pos, yaw[0], true, next_pos, next_yaw)) {
        use_stage3_target = true;
        ed_->global_tour_ = {pos, next_pos};
        ed_->refined_tour_.clear();
        ed_->refined_views1_.clear();
        ed_->refined_views2_.clear();
      // 2026-07-20: 最终出口保护开启后，通用前探可能绕过frontier距离约束并再次远离终点；
      // 2026-07-21: 此时只等待耗尽确认切入门内接近阶段，不再发布无约束恢复步。
      } else if (!task_search_manager_->finalExitFrontierGuardActive() &&
                 buildMissionForwardFallback(pos, yaw[0], next_pos, next_yaw)) {
        use_mission_forward_fallback = true;
        ed_->global_tour_ = {pos, next_pos};
      } else {
        // 2026-07-14: FSM 会以高频重试，节流该诊断避免一次卡点刷出数千行并拖慢规划回调。
        ROS_WARN_THROTTLE(1.0,
                          "[task_search] searched frontiers exhausted and no safe recovery target exists.");
        return FAIL;
      }
    } else {
      // 2026-07-20: 只有真正选出了可执行候选，才证明搜索覆盖已经恢复并取消耗尽确认。
      task_search_manager_->reportSearchCoverageAvailable();
    }
  }

  for (int i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(
        ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

  double view_time = (ros::Time::now() - t1).toSec();
  // 2026-07-16: 规划失败时FSM会高频重试，frontier数量诊断节流到1Hz，避免日志I/O反过来拖慢脱困搜索。
  ROS_WARN_THROTTLE(
      1.0,
      "Frontier: %d, t: %lf, viewpoint: %d, t: %lf", ed_->frontiers_.size(), frontier_time,
      ed_->points_.size(), view_time);

  // Do global and local tour planning and retrieve the next viewpoint
  if (use_stage3_target) {
    // 2026-07-13: 出口接近、二维码环扫和降落接近都走同一条安全 A*/轨迹生成后端。
    if (task_search_manager_) task_search_manager_->recordSelectedGoal(next_pos);
    ed_->global_tour_ = {pos, next_pos};
    ed_->refined_tour_.clear();
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();
    // 2026-07-14: 第三阶段目标是正常任务状态而非错误；节流输出，真正的 A*/足迹失败仍保留 ERROR。
    ROS_INFO_THROTTLE(1.0, "[exit_mission] stage-3 goal=(%.2f, %.2f, %.2f), yaw=%.1fdeg.",
                      next_pos.x(), next_pos.y(), next_pos.z(), next_yaw * 180.0 / M_PI);
  } else if (use_mission_forward_fallback) {
    ROS_INFO("[mission_exploration] use forward fallback view pos=(%.2f, %.2f, %.2f).",
             next_pos.x(), next_pos.y(), next_pos.z());
  } else if (use_mission_filtered_direct_view) {
    // 2026-07-08 19:26: 任务过滤后的候选集合只存在于 exploration_manager 的副本里，
    // 不能再继续走 FrontierFinder 内部那套基于原始 frontiers_ 链表的 cost matrix / TSP / viewpoint-id 索引，
    // 否则会在 frontier 数量被压缩后出现索引错位，最终在 updateFrontierCostMatrix() 里触发 invalid free。
    // 2026-07-13: 任务搜索启用时使用覆盖评分；关闭时保留第一个 FUEL 视点作为兼容回退。
    const int best_idx = task_search_manager_ && task_search_manager_->enabled()
                             ? task_candidate_idx
                             : (ed_->points_.empty() ? -1 : 0);
    if (best_idx < 0) {
      ROS_WARN("[mission_exploration] no valid filtered candidate after mission selection.");
      return FAIL;
    }
    next_pos = ed_->points_[best_idx];
    if (task_search_manager_ && task_search_manager_->enabled()) {
      // 2026-07-13: 将高处 frontier 投影到当前巡航层，优先横向绕障而不是不断爬高。
      next_pos.z() = task_search_manager_->projectSearchHeight(next_pos.z(), pos.z());
    }
    next_yaw = ed_->yaws_[best_idx];
    if (task_search_manager_) task_search_manager_->recordSelectedGoal(next_pos);
    ed_->global_tour_ = {pos, next_pos};
    ed_->refined_tour_.clear();
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();
    ROS_INFO("[task_search] task view selected idx=%d pos=(%.2f, %.2f, %.2f).",
             best_idx, next_pos.x(), next_pos.y(), next_pos.z());
  } else if (!use_forced_entry_target && ed_->points_.size() > 1) {
    // Find the global tour passing through all viewpoints
    // Create TSP and solve by LKH
    // Optimal tour is returned as indices of frontier
    vector<int> indices;
    findGlobalTour(pos, vel, yaw, indices);

    if (ep_->refine_local_) {
      // Do refinement for the next few viewpoints in the global tour
      // Idx of the first K frontier in optimal tour
      t1 = ros::Time::now();

      ed_->refined_ids_.clear();
      ed_->unrefined_points_.clear();
      int knum = min(int(indices.size()), ep_->refined_num_);
      for (int i = 0; i < knum; ++i) {
        auto tmp = ed_->points_[indices[i]];
        ed_->unrefined_points_.push_back(tmp);
        ed_->refined_ids_.push_back(indices[i]);
        if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2) break;
      }

      // Get top N viewpoints for the next K frontiers
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      ed_->refined_points_.clear();
      ed_->refined_views_.clear();
      vector<double> refined_yaws;
      refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
      next_pos = ed_->refined_points_[0];
      next_yaw = refined_yaws[0];

      // Get marker for view visualization
      for (int i = 0; i < ed_->refined_points_.size(); ++i) {
        Vector3d view =
            ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
        ed_->refined_views_.push_back(view);
      }
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();
      for (int i = 0; i < ed_->refined_points_.size(); ++i) {
        vector<Vector3d> v1, v2;
        frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
        frontier_finder_->percep_utils_->getFOV(v1, v2);
        ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
        ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
      }
      double local_time = (ros::Time::now() - t1).toSec();
      ROS_WARN("Local refine time: %lf", local_time);

    } else {
      // Choose the next viewpoint from global tour
      next_pos = ed_->points_[indices[0]];
      next_yaw = ed_->yaws_[indices[0]];
    }
  } else if (!use_forced_entry_target && ed_->points_.size() == 1) {
    // Only 1 destination, no need to find global tour through TSP
    // frontier_finder_->updateFrontierCostMatrix();该这里
    frontier_finder_->updateFrontierCostMatrix();
    ed_->global_tour_ = { pos, ed_->points_[0] };
    ed_->refined_tour_.clear();
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();

    if (ep_->refine_local_) {
      // Find the min cost viewpoint for next frontier
      ed_->refined_ids_ = { 0 };
      ed_->unrefined_points_ = { ed_->points_[0] };
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, { 0 }, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      double min_cost = 100000;
      int min_cost_id = -1;
      vector<Vector3d> tmp_path;
      for (int i = 0; i < ed_->n_points_[0].size(); ++i) {
        auto tmp_cost = ViewNode::computeCost(
            pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
        if (tmp_cost < min_cost) {
          min_cost = tmp_cost;
          min_cost_id = i;
        }
      }
      next_pos = ed_->n_points_[0][min_cost_id];
      next_yaw = n_yaws[0][min_cost_id];
      ed_->refined_points_ = { next_pos };
      ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
    } else {
      next_pos = ed_->points_[0];
      next_yaw = ed_->yaws_[0];
    }
  } else if (!use_forced_entry_target)
    ROS_ERROR("Empty destination.");

  // 2026-07-16: 比赛使用360度Mid360时航向直接对准运动方向；相机模式保留原始viewpoint航向接口。
  const Eigen::Vector2d motion_delta = next_pos.head<2>() - pos.head<2>();
  if (!use_camera_viewpoint_yaw_ && motion_delta.norm() > 0.15)
    next_yaw = std::atan2(motion_delta.y(), motion_delta.x());

  // 2026-07-24: 只在前机正常SEARCH_CORRIDOR平移阶段启用相机扫描；入口/出口任务朝向优先。
  // 默认直段小幅快扫、转弯关闭附加扫描、新障碍物才做一次6秒全向扫描。
  bool camera_head_sweep_active = false;
  bool camera_continuous_rotation_active = false;
  double camera_rotation_rate =
      camera_head_scan_direction_ * 2.0 * M_PI / camera_head_sweep_period_;
  // 2026-07-24: 最新实测中出口候选1/5后，门框被相机障碍扫描误触发60deg/s环扫，
  // 下一帧雷达切面随即center_blocked并清票。候选验证期间立即取消环扫并沿路径朝前。
  const bool exit_verification_active =
      task_search_manager_ && task_search_manager_->exitVerificationActive();
  if (exit_verification_active && camera_obstacle_scan_active_) {
    camera_obstacle_scan_active_ = false;
    camera_route_heading_valid_ = false;
    ROS_WARN("[camera_scan] CANCEL obstacle rotation: exit portal verification has priority.");
  }
  if (camera_head_sweep_enabled_ && !use_forced_entry_target && !use_stage3_target &&
      task_search_manager_ && task_search_manager_->enabled() &&
      !task_search_manager_->stage3Active() &&
      !task_search_manager_->finalExitFrontierGuardActive() &&
      !exit_verification_active && motion_delta.norm() > 0.15) {
    const ros::Time now = ros::Time::now();
    const double base_yaw = std::atan2(motion_delta.y(), motion_delta.x());
    if (camera_route_heading_valid_) {
      const double heading_change =
          std::atan2(std::sin(base_yaw - camera_last_route_heading_),
                     std::cos(base_yaw - camera_last_route_heading_));
      if (std::fabs(heading_change) >= camera_turn_threshold_rad_)
        camera_turning_until_ = now + ros::Duration(camera_turn_hold_time_);
    }
    camera_last_route_heading_ = base_yaw;
    camera_route_heading_valid_ = true;
    const bool turning =
        !camera_turning_until_.isZero() && now < camera_turning_until_;

    if (camera_head_full_rotation_) {
      // 2026-07-24: 仅供调试的旧全程环扫回退项，正常比赛launch默认关闭。
      camera_head_sweep_active = true;
      camera_continuous_rotation_active = true;
      ROS_INFO_THROTTLE(
          1.0,
          "[camera_full_scan] current=%.1fdeg route_heading=%.1fdeg rate=%+.1fdeg/s "
          "period=%.1fs.",
          yaw[0] * 180.0 / M_PI, base_yaw * 180.0 / M_PI,
          camera_head_scan_direction_ * 360.0 / camera_head_sweep_period_,
          camera_head_sweep_period_);
    } else {
      // 2026-07-24: 活动障碍物完成一圈后按中心去重，后续地图更新不能反复触发同一物体。
      if (camera_obstacle_scan_active_ &&
          (now - camera_obstacle_scan_start_).toSec() >= camera_obstacle_scan_period_) {
        camera_scanned_obstacles_.push_back(camera_active_obstacle_);
        if (camera_scanned_obstacles_.size() > 64)
          camera_scanned_obstacles_.erase(camera_scanned_obstacles_.begin());
        ROS_WARN("[camera_obstacle_scan] COMPLETE center=(%.2f,%.2f) duration=%.1fs.",
                 camera_active_obstacle_.x(), camera_active_obstacle_.y(),
                 camera_obstacle_scan_period_);
        camera_obstacle_scan_active_ = false;
      }

      if (camera_obstacle_scan_enabled_ && !camera_obstacle_scan_active_ && !turning) {
        Vector3d detected_obstacle;
        if (detectCameraObstacle(pos, motion_delta.head<2>(), detected_obstacle)) {
          camera_active_obstacle_ = detected_obstacle;
          camera_obstacle_scan_start_ = now;
          camera_obstacle_scan_active_ = true;
          ROS_WARN("[camera_obstacle_scan] NEW center=(%.2f,%.2f,%.2f) "
                   "range=%.2fm; start one %.1fs rotation.",
                   detected_obstacle.x(), detected_obstacle.y(), detected_obstacle.z(),
                   (detected_obstacle.head<2>() - pos.head<2>()).norm(),
                   camera_obstacle_scan_period_);
        }
      }

      if (camera_obstacle_scan_active_) {
        // 2026-07-24: 障碍物扫描用60deg/s级连续yaw轨迹；无人机仍沿原安全位置轨迹移动，
        // 通过接近到越过的视点变化覆盖障碍物正面、侧面和背面。
        camera_head_sweep_active = true;
        camera_continuous_rotation_active = true;
        camera_rotation_rate =
            camera_head_scan_direction_ * 2.0 * M_PI / camera_obstacle_scan_period_;
        ROS_INFO_THROTTLE(
            1.0,
            "[camera_obstacle_scan] ACTIVE center=(%.2f,%.2f) elapsed=%.1f/%.1fs "
            "rate=%+.1fdeg/s.",
            camera_active_obstacle_.x(), camera_active_obstacle_.y(),
            (now - camera_obstacle_scan_start_).toSec(), camera_obstacle_scan_period_,
            camera_rotation_rate * 180.0 / M_PI);
      } else if (turning) {
        // 2026-07-24: A/B/C之间的弯道只看运动方向，不把正常转弯计入扫描任务。
        next_yaw = base_yaw;
        ROS_INFO_THROTTLE(
            1.0, "[camera_scan] TURNING route_heading=%.1fdeg; extra scan disabled.",
            base_yaw * 180.0 / M_PI);
      } else {
        // 2026-07-24: 无障碍直段只做小幅快速摆头，兼顾两侧墙面目标而不再持续背向飞行。
        if (camera_head_sweep_start_.isZero()) camera_head_sweep_start_ = now;
        const double elapsed = std::max(0.0, (now - camera_head_sweep_start_).toSec());
        const double phase = 2.0 * M_PI * elapsed / camera_clear_sweep_period_;
        const double sweep_offset =
            camera_clear_sweep_amplitude_rad_ * std::sin(phase);
        next_yaw = std::atan2(std::sin(base_yaw + sweep_offset),
                              std::cos(base_yaw + sweep_offset));
        camera_head_sweep_active = true;
        ROS_INFO_THROTTLE(
            1.0,
            "[camera_scan] CLEAR_STRAIGHT base=%.1fdeg offset=%+.1fdeg "
            "target=%.1fdeg amplitude=%.1fdeg period=%.1fs.",
            base_yaw * 180.0 / M_PI, sweep_offset * 180.0 / M_PI,
            next_yaw * 180.0 / M_PI,
            camera_clear_sweep_amplitude_rad_ * 180.0 / M_PI,
            camera_clear_sweep_period_);
      }
    }
  } else if (exit_verification_active && motion_delta.norm() > 0.15) {
    // 2026-07-24: 连续5帧门验证期间只看飞行方向，不保持上一环扫角，也不叠加左右摆头。
    next_yaw = std::atan2(motion_delta.y(), motion_delta.x());
    camera_route_heading_valid_ = false;
    ROS_INFO_THROTTLE(
        1.0, "[camera_scan] EXIT_VERIFY forward yaw=%.1fdeg; all camera sweeps suspended.",
        next_yaw * 180.0 / M_PI);
  }

  // 2026-07-14: 记录任务层原始目的地，后续路径截断不能覆盖碰撞失败应冷却的目标。
  last_requested_goal_ = next_pos;
  has_last_requested_goal_ = true;
  std::cout << "Next view: " << next_pos.transpose() << ", " << "next_yaw:" << next_yaw << std::endl;
  // Plan trajectory (position and yaw) to the next viewpoint
  t1 = ros::Time::now();

  // 2026-07-16: Mid360模式不让转头时间拖慢位置轨迹；相机接口打开后才恢复观测航向时间下界。
  const double yaw_diff = std::fabs(next_yaw - yaw[0]);
  const double wrapped_yaw_diff = std::min(yaw_diff, 2.0 * M_PI - yaw_diff);
  const double time_lb = use_camera_viewpoint_yaw_ ? wrapped_yaw_diff / ViewNode::yd_ : 0.0;

  // Generate trajectory of x,y,z
  // 2026-07-23: 已验证出口的门内观察/穿门目标由任务状态机沿锁定门法向生成，不能再被
  // 面向普通FUEL frontier的“旧航迹禁回头”过滤器误杀；占据、A*和机体足迹安全检查仍完整保留。
  const bool exit_transit_active =
      task_search_manager_ && task_search_manager_->exitTransitActive();
  if (task_search_manager_ && !exit_transit_active &&
      !task_search_manager_->isTaskMotionAllowed(next_pos)) {
    ROS_ERROR_THROTTLE(1.0,
                       "[task_progress] reject goal in completed route %.2f %.2f %.2f.",
                       next_pos.x(), next_pos.y(), next_pos.z());
    task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  // 2026-07-13: 所有任务点先做机体足迹检查，避免合法中心点贴柱、贴墙导致旋翼碰撞。
  if (!planner_manager_->isPositionSafe(next_pos)) {
    // 2026-07-14: 保留安全拒绝，但限制重复目标的错误输出频率。
    ROS_ERROR_THROTTLE(1.0, "[footprint_safety] reject unsafe goal %.2f %.2f %.2f before A*.",
                       next_pos.x(), next_pos.y(), next_pos.z());
    if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  planner_manager_->path_finder_->reset();
  if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END) {
    // 2026-07-27: 远frontier跨越尚未建成的已知连通域时，先在当前前向扇区找安全短步；
    // 不再立即切换另一个远目标并触发长时间safety hold，让雷达随局部推进逐段补齐拐弯地图。
    Vector3d local_recovery_pos;
    double local_recovery_yaw = next_yaw;
    const bool allow_local_recovery =
        !use_forced_entry_target && !use_stage3_target && !exit_transit_active;
    if (allow_local_recovery &&
        buildMissionForwardFallback(pos, yaw[0], local_recovery_pos, local_recovery_yaw)) {
      ROS_WARN("[task_route] remote goal (%.2f,%.2f,%.2f) disconnected; use connected "
               "local step (%.2f,%.2f,%.2f).",
               next_pos.x(), next_pos.y(), next_pos.z(), local_recovery_pos.x(),
               local_recovery_pos.y(), local_recovery_pos.z());
      next_pos = local_recovery_pos;
      next_yaw = local_recovery_yaw;
      // 2026-07-27: 碰撞反馈必须冷却实际发布的局部目标，不能继续指向已被替换的远frontier。
      last_requested_goal_ = next_pos;
      planner_manager_->path_finder_->reset();
      if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END) {
        ROS_ERROR_THROTTLE(
            1.0, "Connected local recovery became unavailable before trajectory generation");
        if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
        return FAIL;
      }
    } else {
      // 2026-07-16: Astar内部会区分TIMEOUT和DISCONNECTED；这里只保留1Hz汇总，禁止失败时百Hz刷屏。
      ROS_ERROR_THROTTLE(1.0, "No path to next viewpoint");
      if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
      return FAIL;
    }
  }
  ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  // 2026-07-14: 起点和目标落在同一 A* 栅格时 getPath() 可能只返回一个点；去重并显式保留真实目标。
  vector<Vector3d> normalized_path;
  normalized_path.reserve(ed_->path_next_goal_.size() + 1);
  for (const auto& point : ed_->path_next_goal_) {
    if (normalized_path.empty() || (point - normalized_path.back()).norm() >= 1e-3)
      normalized_path.push_back(point);
  }
  // 2026-07-14: 同栅格回溯可能只含目标栅格中心，不含真实起点；显式补起点才能保留实际短距离。
  if (normalized_path.empty())
    normalized_path.push_back(pos);
  else if ((normalized_path.front() - pos).norm() >= 1e-3)
    normalized_path.insert(normalized_path.begin(), pos);
  if ((next_pos - normalized_path.back()).norm() >= 1e-3) normalized_path.push_back(next_pos);
  ed_->path_next_goal_.swap(normalized_path);

  // 2026-07-23: 普通搜索路径继续执行全局禁回头；出口任务路径允许跨越最近实飞航迹和门平面，
  // 否则CROSS_EXIT会在门口被“completed route”永久拒绝。
  if (task_search_manager_ && !exit_transit_active &&
      !task_search_manager_->isTaskPathAllowed(ed_->path_next_goal_)) {
    ROS_ERROR_THROTTLE(1.0,
                       "[global_no_return] reject A* path returning through entrance/"
                       "completed corridor or starting backward.");
    task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }

  // 2026-07-22: FUEL远端frontier只决定前进方向；沿A*路径截取短安全段作为本轮路线点，
  // 避免刚穿门就发布跨越数米未知/窄区的整条轨迹，并让新点云在每段之间更新地图。
  const double route_horizon = std::max(0.20, ep_->mission_max_route_segment_length_);
  if (Astar::pathLength(ed_->path_next_goal_) > route_horizon) {
    vector<Vector3d> local_segment{ed_->path_next_goal_.front()};
    double accumulated = 0.0;
    for (size_t i = 1; i < ed_->path_next_goal_.size(); ++i) {
      const Vector3d delta = ed_->path_next_goal_[i] - ed_->path_next_goal_[i - 1];
      const double segment_length = delta.norm();
      if (segment_length < 1e-6) continue;
      if (accumulated + segment_length >= route_horizon) {
        const double ratio = (route_horizon - accumulated) / segment_length;
        local_segment.push_back(ed_->path_next_goal_[i - 1] +
                                std::max(0.0, std::min(1.0, ratio)) * delta);
        break;
      }
      local_segment.push_back(ed_->path_next_goal_[i]);
      accumulated += segment_length;
    }
    ed_->path_next_goal_.swap(local_segment);
    ROS_WARN("[task_route] truncate remote goal to %.2fm safe segment endpoint=(%.2f, %.2f, %.2f).",
             route_horizon, ed_->path_next_goal_.back().x(), ed_->path_next_goal_.back().y(),
             ed_->path_next_goal_.back().z());
  }
  // 2026-07-23: 截断后的出口任务短步同样豁免旧航迹门控，但不豁免后续障碍物/足迹检查。
  if (task_search_manager_ && !exit_transit_active &&
      !task_search_manager_->isTaskMotionAllowed(ed_->path_next_goal_.back())) {
    ROS_ERROR_THROTTLE(1.0, "[task_progress] reject route segment ending in completed corridor.");
    task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  if (Astar::pathLength(ed_->path_next_goal_) < 0.03) {
    // 2026-07-14: 三厘米内视为已到达，冷却该视点并换点，不能生成零时长多项式轨迹。
    ROS_WARN_THROTTLE(1.0, "[trajectory_input] goal is already reached; select another viewpoint.");
    if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  // 2026-07-13: 几何路径在缩短前按完整机体足迹复核，避免优化器把中心安全路径贴到障碍物上。
  if (!planner_manager_->isPathSafe(ed_->path_next_goal_)) {
    // 2026-07-14: 路径失败会反馈给任务层更换出口接近点，不再以 100 Hz 输出同一错误。
    ROS_ERROR_THROTTLE(1.0,
                       "[footprint_safety] reject A* path whose Iris footprint intersects obstacles.");
    if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  shortenPath(ed_->path_next_goal_);
  // 2026-07-14: shortenPath 会用中心射线删除 A* 转折点，删除后必须重新做完整机体足迹检查。
  if (!planner_manager_->isPathSafe(ed_->path_next_goal_)) {
    ROS_ERROR_THROTTLE(
        1.0, "[footprint_safety] reject shortened path whose Iris footprint intersects obstacles.");
    if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  // 2026-07-23: shortenPath后只对普通探索路径再次做禁回头判定，任务出口路径由门状态机约束方向。
  if (task_search_manager_ && !exit_transit_active &&
      !task_search_manager_->isTaskPathAllowed(ed_->path_next_goal_)) {
    ROS_ERROR_THROTTLE(1.0,
                       "[global_no_return] reject shortened path returning to completed corridor.");
    task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }
  if (!use_forced_entry_target && !pathInsideWorkspaceLock(ed_->path_next_goal_)) {
    // 2026-07-10: 即使 viewpoint 在门内，A* 也可能绕回起飞区；轨迹生成前必须拒绝穿门外路径。
    ROS_ERROR("[workspace_lock] reject path to next viewpoint because it crosses outside the door plane.");
    if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
    return FAIL;
  }

  const double radius_far = 5.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (len < radius_close) {
    // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
    // optimization
    if (!planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb)) {
      if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
      return FAIL;
    }
    ed_->next_goal_ = next_pos;

  } else if (len > radius_far) {
    // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
    // dead end)
    std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
    for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    if (!planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb)) {
      if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
      return FAIL;
    }
    // if (!planner_manager_->kinodynamicReplan(
    //         pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
    //   return FAIL;
    // ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  } else {
    // Search kino path to exactly next viewpoint and optimize
    std::cout << "Mid goal" << std::endl;
    ed_->next_goal_ = next_pos;

    if (!planner_manager_->kinodynamicReplan(
            pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb)) {
      if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
      return FAIL;
    }
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
    ROS_ERROR("Lower bound not satified!");

  // 2026-07-14: 优化后的 B-spline 可能切入柱体/墙面；发布前拒绝并切换任务目标，而不是交给控制器反复急停。
  if (!planner_manager_->isTrajectorySafe()) {
    // 2026-07-16: 远目标平滑轨迹切弯失败时，沿已经通过足迹检查的A*路径先走0.8m安全前缀。
    // 比赛中宁可分段推进并重规划，也不能在宽通道对同一远目标永久FAIL/悬停。
    constexpr double conservative_advance = 0.80;
    vector<Vector3d> safe_prefix;
    if (!ed_->path_next_goal_.empty()) safe_prefix.push_back(ed_->path_next_goal_.front());
    double prefix_length = 0.0;
    for (size_t i = 1; i < ed_->path_next_goal_.size() && prefix_length < conservative_advance;
         ++i) {
      const Vector3d delta = ed_->path_next_goal_[i] - safe_prefix.back();
      const double remaining = conservative_advance - prefix_length;
      if (delta.norm() > remaining && remaining > 1e-3) {
        safe_prefix.push_back(safe_prefix.back() + delta.normalized() * remaining);
        prefix_length = conservative_advance;
      } else {
        safe_prefix.push_back(ed_->path_next_goal_[i]);
        prefix_length += delta.norm();
      }
    }
    if (safe_prefix.size() == 2)
      safe_prefix.insert(safe_prefix.begin() + 1, 0.5 * (safe_prefix[0] + safe_prefix[1]));

    const bool prefix_generated = safe_prefix.size() >= 3 &&
                                  planner_manager_->isPathSafe(safe_prefix) &&
                                  planner_manager_->planExploreTraj(safe_prefix, vel, acc, 0.0);
    if (!prefix_generated || !planner_manager_->isTrajectorySafe()) {
      ROS_ERROR_THROTTLE(1.0,
                         "[stuck_recovery] full trajectory and 0.8m safe-prefix trajectory both failed.");
      if (task_search_manager_) task_search_manager_->reportGoalFailure(next_pos);
      return FAIL;
    }
    ed_->next_goal_ = safe_prefix.back();
    ROS_WARN("[stuck_recovery] full trajectory unsafe; publish %.2fm A* safe prefix to (%.2f %.2f %.2f).",
             prefix_length, ed_->next_goal_.x(), ed_->next_goal_.y(), ed_->next_goal_.z());
  }

  // 2026-07-24: 扫描段关闭“整段始终看前方”的约束；全向模式在位置轨迹全程插入单方向
  // 连续yaw约束，非扫描阶段完全保留原有沿路径看前方或任务固定朝向的行为。
  planner_manager_->planYawExplore(
      yaw, next_yaw, !camera_head_sweep_active, ep_->relax_time_,
      camera_continuous_rotation_active, camera_rotation_rate);

  double traj_plan_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  double yaw_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
  double total = (ros::Time::now() - t2).toSec();
  ROS_WARN("Total time: %lf", total);
  ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

  return SUCCEED;
}

void FastExplorationManager::reportTrajectoryCollision() {
  // 2026-07-14: 冷却任务层原始目标而非远路径的截断中间点，阻断同一出口路线反复发布/急停。
  if (task_search_manager_ && has_last_requested_goal_)
    task_search_manager_->reportGoalFailure(last_requested_goal_);
}

void FastExplorationManager::updateMissionOdometry(const Vector3d& pos, double yaw) {
  // 2026-07-22: 连续里程计航迹是任务完成路段的唯一依据；规划器预测起点不再污染真实历史。
  if (task_search_manager_) task_search_manager_->updateRobotPose(pos, yaw);
}

void FastExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastExplorationManager::findGlobalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
    vector<int>& indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->updateFrontierCostMatrix();
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
  //     "\nEDGE_WEIGHT_TYPE : "
  //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;
  if (false) {
    // Use symmetric TSP
    for (int i = 1; i < dimension; ++i) {
      for (int j = 0; j < i; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }

  } else {
    // Use Asymmetric TSP
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  if (false) {
    // Read path for Symmetric TSP formulation
    getline(res_file, res);  // Skip current pose
    getline(res_file, res);
    int id = stoi(res);
    bool rev = (id == dimension);  // The next node is virutal depot?

    while (id != -1) {
      indices.push_back(id - 2);
      getline(res_file, res);
      id = stoi(res);
    }
    if (rev) reverse(indices.begin(), indices.end());
    indices.pop_back();  // Remove the depot

  } else {
    // Read path for ATSP formulation
    while (getline(res_file, res)) {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1)  // Ignore the current state
        continue;
      if (id == -1) break;
      indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
    }
  }

  res_file.close();

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

void FastExplorationManager::refineLocalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
    const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
    vector<Vector3d>& refined_pts, vector<double>& refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local tour graph: ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group)
        g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

}  // namespace fast_planner
