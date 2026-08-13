#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <bspline/Bspline.h>

using std::vector;
using Eigen::Vector3d;

namespace fast_planner {
struct FSMData {
  // FSM data
  bool trigger_, have_odom_, static_state_;
  vector<string> state_str_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  vector<Eigen::Vector3d> start_poss;
  bspline::Bspline newest_traj_;
};

struct FSMParam {
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_time_;  // second
  // 2026-07-13: 实际里程计偏离样条过大时先悬停重规划，避免轨迹安全但机体跟踪误差导致擦碰。
  double max_tracking_error_xy_;
  double max_tracking_error_z_;
  // 2026-07-13: 跟踪误差采用持续确认和硬急停两级阈值，避免单次抖动反复清空轨迹。
  double tracking_error_confirm_time_;
  double tracking_error_grace_time_;
  double hard_tracking_error_xy_;
  double hard_tracking_error_z_;
  // 2026-07-22: 失败重试周期与100Hz FSM定时器解耦，悬停期间等待地图真正更新。
  double plan_failure_retry_interval_;
  // 2026-07-28: 新轨迹需经过刷新地图上的连续确认，并限制轨迹起点与真实里程计的交接误差。
  double trajectory_release_confirm_time_;
  double trajectory_release_check_interval_;
  double trajectory_release_max_start_error_;
  // 预测旧轨迹将碰撞时，traj_server 只沿当前样条再执行这段时间，然后停在短段末端。
  double emergency_brake_horizon_;
};

struct ExplorationData {
  vector<vector<Vector3d>> frontiers_;
  vector<vector<Vector3d>> dead_frontiers_;
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<Vector3d> points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_;
  vector<double> yaws_;
  vector<Vector3d> global_tour_;

  vector<int> refined_ids_;
  vector<vector<Vector3d>> n_points_;
  vector<Vector3d> unrefined_points_;
  vector<Vector3d> refined_points_;
  vector<Vector3d> refined_views_;  // points + dir(yaw)
  vector<Vector3d> refined_views1_, refined_views2_;
  vector<Vector3d> refined_tour_;

  Vector3d next_goal_;
  vector<Vector3d> path_next_goal_;

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;
};

struct ExplorationParam {
  // params
  bool refine_local_;
  int refined_num_;
  double refined_radius_;
  int top_view_num_;
  double max_decay_;
  string tsp_dir_;  // resource dir of tsp solver
  double relax_time_;

  // 2026-07-08 19:26: 为未来飞行器大赛 2.1 的“先进入搜索区、再做目标搜索”任务语义新增任务约束参数，
  // 避免 frontier exploration 继续把起飞大房间当主收益区，导致无人机迟迟不愿穿过窄通道。
  bool mission_use_entry_transit_;
  bool mission_prefer_entry_heading_;
  bool mission_use_takeoff_exclusion_box_;
  bool mission_prefer_search_region_frontiers_;
  bool mission_hold_on_no_frontier_;
  // 2026-07-10: 第二阶段启用门平面锁，FUEL 只能在入口内侧/作业区侧选择 frontier、viewpoint 和路径。
  bool mission_use_workspace_lock_;
  // 入口确认后对所有目标来源和完整A*路径统一禁止返回门外/已完成通道；默认开启。
  bool mission_global_no_return_;
  // 2026-07-10: 门后搜索阶段不再只选最近 frontier，优先沿动态门方向向前推进；frontier 为空时允许短距离安全前探。
  bool mission_use_forward_progress_bias_;
  bool mission_use_forward_fallback_;
  // 2026-07-22: 任务路线按短安全段滚动发布，禁止刚过门就直接优化到数米外FUEL视点。
  double mission_max_route_segment_length_;
  double mission_entry_arrive_dist_;
  double mission_entry_yaw_;
  double mission_door_back_margin_;
  double mission_forward_progress_weight_;
  double mission_forward_min_gain_;
  double mission_forward_fallback_step_;
  double mission_forward_fallback_max_;
  Eigen::Vector3d mission_entry_goal_;
  Eigen::Vector3d mission_takeoff_exclusion_min_;
  Eigen::Vector3d mission_takeoff_exclusion_max_;
  Eigen::Vector3d mission_search_region_min_;
  Eigen::Vector3d mission_search_region_max_;
};

}  // namespace fast_planner

#endif
