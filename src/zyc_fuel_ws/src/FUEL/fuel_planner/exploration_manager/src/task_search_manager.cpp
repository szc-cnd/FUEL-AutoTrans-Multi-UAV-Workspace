// 2026-07-13: 实现比赛第二阶段任务搜索、重复站点抑制、目标登记和多方向搜索恢复排序。
#include <exploration_manager/task_search_manager.h>
#include <exploration_manager/motion_direction_rules.h>
#include <exploration_manager/frontier_clearance_policy.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <plan_env/sdf_map.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>

namespace fast_planner {

void TaskSearchManager::initialize(ros::NodeHandle& nh) {
  nh.param("mission/task_search/enabled", enabled_, true);
  nh.param("fsm/use_diff_for_fuel_exploration",
           hybrid_constraints_enabled_, false);
  // 2026-07-16: false只关闭三类识别对任务状态切换的阻塞，不删除话题订阅和后续接入接口。
  nh.param("mission/task_search/require_stage2_detections", require_stage2_detections_, true);
  nh.param("mission/task_search/world_frame", world_frame_, std::string("world"));
  nh.param("mission/task_search/visit_spacing", visit_spacing_, 0.35);
  nh.param("mission/task_search/revisit_radius", revisit_radius_, 0.65);
  nh.param("mission/task_search/repeat_goal_radius", repeat_goal_radius_, 0.45);
  nh.param("mission/task_search/max_goal_repeats", max_goal_repeats_, 2);
  nh.param("mission/task_search/max_history_size", max_history_size_, 120);
  nh.param("mission/task_search/cruise_height", cruise_height_, 0.75);
  nh.param("mission/task_search/min_height", min_search_height_, 0.55);
  nh.param("mission/task_search/max_height", max_search_height_, 1.00);
  // 2026-07-13: 比赛窄道采用水平优先搜索，目标高度不能随 frontier 一次跳到顶层。
  nh.param("mission/task_search/max_goal_climb", max_goal_climb_, 0.06);
  nh.param("mission/task_search/max_goal_descent", max_goal_descent_, 0.10);
  nh.param("mission/task_search/novelty_weight", novelty_weight_, 2.5);
  nh.param("mission/task_search/travel_weight", travel_weight_, 0.65);
  nh.param("mission/task_search/yaw_weight", yaw_weight_, 0.20);
  nh.param("mission/task_search/height_weight", height_weight_, 2.0);
  nh.param("mission/task_search/repeat_penalty", repeat_penalty_, 6.0);
  nh.param("mission/task_search/frontier_gain_weight", frontier_gain_weight_, 0.10);
  nh.param("mission/task_search/clearance_reward_enabled",
           clearance_reward_enabled_, true);
  nh.param("mission/task_search/clearance_reward_start",
           clearance_reward_start_, 0.30);
  nh.param("mission/task_search/clearance_reward_full",
           clearance_reward_full_, 0.60);
  nh.param("mission/task_search/clearance_reward_max",
           clearance_reward_max_, 5.0);
  clearance_reward_start_ = std::max(0.0, clearance_reward_start_);
  clearance_reward_full_ =
      std::max(clearance_reward_start_ + 0.05, clearance_reward_full_);
  clearance_reward_max_ = std::max(0.0, clearance_reward_max_);
  nh.param("mission/task_search/entry_forward_distance", entry_forward_distance_, 2.0);
  nh.param("mission/task_search/entry_forward_weight", entry_forward_weight_, 1.5);
  nh.param("mission/task_search/forward_viewpoint_bonus", forward_viewpoint_bonus_, 5.0);
  nh.param("mission/task_search/backward_viewpoint_penalty",
           backward_viewpoint_penalty_, 5.0);
  // 2026-07-21: 单通道任务不能把单帧frontier耗尽当成死路；默认只接受前向/侧向候选。
  nh.param("mission/task_search/prefer_motion_forward", prefer_motion_forward_, true);
  nh.param("mission/task_search/allow_search_backtrack", allow_search_backtrack_, false);
  nh.param("mission/global_no_return", global_no_return_, true);
  nh.param("mission/task_search/backward_cos_threshold", backward_cos_threshold_, -0.15);
  // 2026-07-13: 选点保持和失败冷却抑制柱体附近每 0.25s 来回切换目标。
  nh.param("mission/task_search/min_goal_hold_time", min_goal_hold_time_, 1.2);
  // 2026-07-14: 活动目标未到达且未失败时惩罚远距离换点，防止前机突然倒车撞向后机。
  nh.param("mission/task_search/goal_switch_weight", goal_switch_weight_, 1.2);
  nh.param("mission/task_search/failed_goal_radius", failed_goal_radius_, 0.55);
  nh.param("mission/task_search/failed_goal_cooldown", failed_goal_cooldown_, 2.0);
  nh.param("mission/task_search/inside_return_margin", inside_return_margin_, 0.10);
  nh.param("mission/task_search/entry_path_direction_grace_distance",
           entry_path_direction_grace_distance_, 1.50);
  // 2026-07-28: 正常拐弯不能继续把最近实飞切线当作唯一“前方”；累计占据地图负责识别新通道轴线。
  nh.param("mission/task_search/recovery/occupancy_turn_enabled",
           recovery_occupancy_turn_enabled_, true);
  nh.param("mission/task_search/recovery/turn_probe_length",
           recovery_turn_probe_length_, 1.50);
  nh.param("mission/task_search/recovery/turn_probe_step",
           recovery_turn_probe_step_, 0.10);
  nh.param("mission/task_search/recovery/turn_min_angle_deg",
           recovery_turn_min_angle_deg_, 30.0);
  nh.param("mission/task_search/recovery/turn_max_angle_deg",
           recovery_turn_max_angle_deg_, 120.0);
  nh.param("mission/task_search/recovery/turn_confirmation_count",
           recovery_turn_confirmation_count_, 1);
  nh.param("mission/task_search/recovery/turn_confirmation_min_interval",
           recovery_turn_confirmation_min_interval_, 0.15);
  nh.param("mission/task_search/recovery/turn_confirmation_angle_deg",
           recovery_turn_confirmation_angle_deg_, 15.0);
  nh.param("mission/task_search/recovery/turn_confirmation_accumulation_window",
           recovery_turn_confirmation_accumulation_window_, 8.0);
  nh.param("mission/task_search/recovery/turn_no_return_margin",
           recovery_turn_no_return_margin_, 0.20);
  nh.param("mission/task_search/recovery/turn_yaw_release_angle_deg",
           recovery_turn_yaw_release_angle_deg_, 15.0);
  recovery_turn_confirmation_count_ =
      std::max(1, recovery_turn_confirmation_count_);
  recovery_turn_confirmation_min_interval_ =
      std::max(0.05, recovery_turn_confirmation_min_interval_);
  recovery_turn_confirmation_angle_deg_ =
      std::max(3.0, std::min(30.0, recovery_turn_confirmation_angle_deg_));
  recovery_turn_confirmation_accumulation_window_ =
      std::max(0.5, recovery_turn_confirmation_accumulation_window_);

  // 2026-07-13: 第三阶段基于累计占据地图推断出口，ArUco平台未确认时只搜索不降落。
  nh.param("mission/task_search/exit/enabled", exit_detection_enabled_, true);
  nh.param("mission/task_search/exit/grid_resolution", exit_grid_resolution_, 0.20);
  nh.param("mission/task_search/exit/inference_period", exit_inference_period_, 1.0);
  nh.param("mission/task_search/exit/min_geodesic_distance", exit_min_geodesic_distance_, 4.0);
  // 2026-07-23: 5/5只代表局部结构稳定；累计通道里程达到该值后才像入口一样锁门停检。
  nh.param("mission/task_search/exit/lock_min_geodesic_distance",
           exit_lock_min_geodesic_distance_, 8.0);
  nh.param("mission/task_search/exit/stability_radius", exit_candidate_stability_radius_, 0.80);
  nh.param("mission/task_search/exit/confirmation_count", exit_confirmation_count_, 3);
  // 2026-07-28: 允许远处候选累计结构证据，但最终确认必须进入门平面近场。
  nh.param("mission/task_search/exit/confirmation_max_distance",
           exit_confirmation_max_distance_, 1.00);
  // 2026-07-28: 只在累计实飞足够深、最近航迹近似直线且门面位于近场时启用局部出口降级。
  nh.param("mission/task_search/exit/near_fallback_min_travel",
           exit_near_fallback_min_travel_, 10.0);
  nh.param("mission/task_search/exit/near_fallback_max_distance",
           exit_near_fallback_max_distance_, 1.0);
  nh.param("mission/task_search/exit/near_fallback_max_turn_deg",
           exit_near_fallback_max_turn_deg_, 25.0);
  // 2026-07-24: 出口证据间隔约1秒；保留2.5秒前视窗口可跨过一帧雷达抖动，
  // 又不会在普通通道里长期关闭相机扫描。
  nh.param("mission/task_search/exit/verification_yaw_hold_time",
           exit_verification_yaw_hold_time_, 2.5);
  // 2026-07-22: 出口与入口相反，只允许更靠拓扑远端的稳定双墙终止截面修正已确认出口。
  nh.param("mission/task_search/exit/supersede_confirmation_count",
           exit_supersede_confirmation_count_, 2);
  nh.param("mission/task_search/exit/supersede_min_path_increase",
           exit_supersede_min_path_increase_, 0.60);
  // 2026-07-16: 搜索耗尽采用时间确认；候选失败冷却或单周期地图抖动不能触发第三阶段。
  nh.param("mission/task_search/exit/search_exhausted_confirm_time",
           search_exhausted_confirm_time_, 3.0);
  nh.param("mission/task_search/exit/standoff", exit_standoff_, 0.45);
  nh.param("mission/task_search/exit/arrive_distance", exit_arrive_distance_, 0.50);
  // 2026-07-21: 出口确认必须看到可容纳机体的开口、左右边界及门外非占据延伸。
  nh.param("mission/task_search/exit/portal_min_half_width", exit_portal_min_half_width_, 0.42);
  nh.param("mission/task_search/exit/portal_max_half_width", exit_portal_max_half_width_, 1.10);
  nh.param("mission/task_search/exit/portal_side_sample_step",
           exit_portal_side_sample_step_, 0.10);
  // 2026-07-22: 门洞核心净空止于门框搜索起点内侧，避免XY邻域采样同时碰到真实门框。
  nh.param("mission/task_search/exit/portal_core_margin", exit_portal_core_margin_, 0.10);
  // 2026-07-22: 只把工作高度带内具有多层竖向支撑的占据当成门框，排除地板和单个漂移体素。
  nh.param("mission/task_search/exit/column_lower_margin", exit_column_lower_margin_, 0.15);
  nh.param("mission/task_search/exit/column_upper_margin", exit_column_upper_margin_, 0.15);
  nh.param("mission/task_search/exit/column_sample_step", exit_column_sample_step_, 0.10);
  nh.param("mission/task_search/exit/column_min_z", exit_column_min_z_, 0.18);
  nh.param("mission/task_search/exit/column_max_z", exit_column_max_z_, 1.55);
  nh.param("mission/task_search/exit/column_min_occupied_layers",
           exit_column_min_occupied_layers_, 2);
  nh.param("mission/task_search/exit/column_min_vertical_span",
           exit_column_min_vertical_span_, 0.18);
  nh.param("mission/task_search/exit/transition_inside_probe",
           exit_transition_inside_probe_, 0.45);
  nh.param("mission/task_search/exit/transition_outside_probe",
           exit_transition_outside_probe_, 0.65);
  // 2026-07-22: 不再用“大开放区”定义出口；排除正常拐弯后，识别横向门框或双侧通道墙同步变化。
  nh.param("mission/task_search/exit/max_centerline_turn_deg",
           exit_max_centerline_turn_deg_, 35.0);
  nh.param("mission/task_search/exit/turn_window_distance",
           exit_turn_window_distance_, 0.60);
  nh.param("mission/task_search/exit/wall_support_length",
           exit_wall_support_length_, 0.90);
  nh.param("mission/task_search/exit/wall_support_min_ratio",
           exit_wall_support_min_ratio_, 0.55);
  nh.param("mission/task_search/exit/frame_outward_support_length",
           exit_frame_outward_support_length_, 0.30);
  nh.param("mission/task_search/exit/min_bilateral_wall_shift",
           exit_min_bilateral_wall_shift_, 0.18);
  // 2026-07-22: 出口采用“门内稳定双墙 -> 门外多截面共同终止 + 已观测FREE”的任务语义。
  nh.param("mission/task_search/exit/wall_track_width_tolerance",
           exit_wall_track_width_tolerance_, 0.25);
  nh.param("mission/task_search/exit/wall_end_probe_step",
           exit_wall_end_probe_step_, 0.25);
  nh.param("mission/task_search/exit/wall_end_probe_count",
           exit_wall_end_probe_count_, 4);
  nh.param("mission/task_search/exit/wall_end_min_changed_count",
           exit_wall_end_min_changed_count_, 3);
  nh.param("mission/task_search/exit/outside_min_free_ratio",
           exit_outside_min_free_ratio_, 0.75);
  nh.param("mission/task_search/exit/candidate_max_behind_distance",
           exit_candidate_max_behind_distance_, 0.35);
  nh.param("mission/task_search/exit/progress_reverse_reject_deg",
           exit_progress_reverse_reject_deg_, 120.0);
  nh.param("mission/task_search/exit/max_direction_jump_deg",
           exit_max_direction_jump_deg_, 100.0);
  // 2026-07-23: 拓扑层已经确认是正常拐弯的局部截面不能再被雷达直线射线判成出口。
  nh.param("mission/task_search/exit/normal_turn_reject_radius",
           exit_normal_turn_reject_radius_, 0.90);
  // 2026-07-23: 出口局部结构改由机体系 Mid360 短时累积确认；这些参数均不引用世界绝对z。
  nh.param("mission/task_search/exit/radar/enabled", radar_exit_detection_enabled_, true);
  nh.param("mission/task_search/exit/radar/body_cloud_topic",
           radar_exit_body_cloud_topic_,
           // 2026-07-27: 出口雷达复核默认订阅前机 UAV0 的机体系点云。
           std::string("/UAV0/fast_lio/cloud_registered_body"));
  nh.param("mission/task_search/exit/radar/accumulation_time",
           radar_exit_accumulation_time_, 1.50);
  nh.param("mission/task_search/exit/radar/max_elevation_deg",
           radar_exit_max_elevation_deg_, 12.0);
  nh.param("mission/task_search/exit/radar/min_range", radar_exit_min_range_, 0.25);
  nh.param("mission/task_search/exit/radar/max_range", radar_exit_max_range_, 12.0);
  nh.param("mission/task_search/exit/radar/max_points_per_frame",
           radar_exit_max_points_per_frame_, 1800);
  nh.param("mission/task_search/exit/radar/search_min_forward",
           radar_exit_search_min_forward_, -0.15);
  nh.param("mission/task_search/exit/radar/search_max_forward",
           radar_exit_search_max_forward_, 1.25);
  nh.param("mission/task_search/exit/radar/search_step", radar_exit_search_step_, 0.10);
  nh.param("mission/task_search/exit/radar/wall_bin_step",
           radar_exit_wall_bin_step_, 0.12);
  nh.param("mission/task_search/exit/radar/inside_length",
           radar_exit_inside_length_, 0.90);
  nh.param("mission/task_search/exit/radar/min_inside_bins",
           radar_exit_min_inside_bins_, 5);
  nh.param("mission/task_search/exit/radar/wall_track_tolerance",
           radar_exit_wall_track_tolerance_, 0.22);
  nh.param("mission/task_search/exit/radar/min_wall_shift",
           radar_exit_min_wall_shift_, 0.18);
  nh.param("mission/task_search/exit/radar/outside_probe_count",
           radar_exit_outside_probe_count_, 4);
  nh.param("mission/task_search/exit/radar/min_changed_probes",
           radar_exit_min_changed_probes_, 3);
  nh.param("mission/task_search/exit/radar/clear_half_width",
           radar_exit_clear_half_width_, 0.28);
  nh.param("mission/task_search/exit/radar/clear_depth",
           radar_exit_clear_depth_, 0.80);
  nh.param("mission/task_search/exit/radar/min_clear_ray_frames",
           radar_exit_min_clear_ray_frames_, 2);
  // 2026-07-22: 门后至少累计1m已知FREE拓扑分支，才说明远端预测点已经落在通道外侧。
  nh.param("mission/task_search/exit/endpoint_outside_route_min_length",
           exit_endpoint_outside_route_min_length_, 1.00);
  // 2026-07-22: 门口局部地图抖动采用计数衰减，并在高稳定候选处生成门内补扫目标。
  nh.param("mission/task_search/exit/candidate_reject_decay",
           exit_candidate_reject_decay_, 1);
  nh.param("mission/task_search/exit/pending_observation_min_hits",
           exit_pending_observation_min_hits_, 3);
  nh.param("mission/task_search/exit/pending_observation_lateral_offset",
           exit_pending_observation_lateral_offset_, 0.25);
  nh.param("mission/task_search/exit/pending_observation_min_move",
           exit_pending_observation_min_move_, 0.22);
  nh.param("mission/task_search/exit/outside_probe_distance", exit_outside_probe_distance_, 0.90);
  nh.param("mission/task_search/exit/outside_min_nonoccupied_samples",
           exit_outside_min_nonoccupied_samples_, 3);
  nh.param("mission/task_search/exit/cross_step", exit_cross_step_, 0.35);
  nh.param("mission/task_search/exit/cross_target_distance", exit_cross_target_distance_, 0.90);
  nh.param("mission/task_search/exit/cross_confirm_distance", exit_cross_confirm_distance_, 0.45);
  nh.param("mission/task_search/exit/footprint_extra_radius", exit_footprint_extra_radius_, 0.22);
  nh.param("mission/task_search/exit/footprint_samples", exit_footprint_samples_, 8);
  nh.param("mission/task_search/exit/scan_radius", scan_radius_, 0.35);
  nh.param("mission/task_search/exit/scan_dwell_time", scan_dwell_time_, 1.2);
  nh.param("mission/task_search/exit/external_landing_planner",
           external_landing_planner_, false);
  nh.param("mission/task_search/exit/require_final_landing_marker",
           require_final_landing_marker_, true);
  // 2026-07-20: 最终出口确认且路径足够远后，普通frontier不得把机体明显拉离出口；
  // 出口附近保留局部绕障和姿态调整空间。
  nh.param("mission/task_search/exit/frontier_local_adjust_radius",
           exit_frontier_local_adjust_radius_, 1.20);
  nh.param("mission/task_search/exit/frontier_max_distance_increase",
           exit_frontier_max_distance_increase_, 0.45);
  nh.param("mission/task_search/exit/final_landing_marker_confirmation_count",
           final_landing_marker_confirmation_count_, 3);
  nh.param("mission/task_search/exit/final_landing_marker_consistency_radius",
           final_landing_marker_consistency_radius_, 0.40);
  nh.param("mission/task_search/exit/landing_approach_height", landing_approach_height_, 0.70);
  nh.param("mission/task_search/exit/landing_trigger_distance", landing_trigger_distance_, 0.35);
  nh.param("mission/task_search/exit/landing_column_bottom_clearance",
           landing_column_bottom_clearance_, 0.20);
  nh.param("mission/task_search/exit/landing_column_step", landing_column_step_, 0.10);

  nh.param("mission/task_search/exit/rc_trigger/enabled",
           rc_search_landing_enabled_, false);
  nh.param("mission/task_search/exit/rc_trigger/topic",
           rc_search_landing_topic_, std::string("/UAV0/mavros/rc/in"));
  nh.param("mission/task_search/exit/rc_trigger/channel_index",
           rc_search_landing_channel_, 8);
  nh.param("mission/task_search/exit/rc_trigger/low_pwm",
           rc_search_landing_low_pwm_, 1300);
  nh.param("mission/task_search/exit/rc_trigger/high_pwm",
           rc_search_landing_high_pwm_, 1800);
  nh.param("mission/task_search/exit/rc_trigger/hold_sec",
           rc_search_landing_hold_sec_, 0.5);
  rc_search_landing_channel_ = std::max(0, rc_search_landing_channel_);
  rc_search_landing_hold_sec_ = std::max(0.1, rc_search_landing_hold_sec_);

  std::string color_topic, qrcode_topic, thermal_topic, final_landing_marker_topic;
  nh.param("mission/task_search/color_topic", color_topic, std::string("/mission/detection/color"));
  nh.param("mission/task_search/qrcode_topic", qrcode_topic,
           std::string("/mission/detection/qrcode"));
  nh.param("mission/task_search/thermal_topic", thermal_topic,
           std::string("/mission/detection/thermal"));
  nh.param("mission/task_search/final_landing_marker_topic", final_landing_marker_topic,
           std::string("/mission/detection/final_aruco"));
  color_detection_sub_ = nh.subscribe(color_topic, 5, &TaskSearchManager::colorDetectionCallback, this);
  qrcode_detection_sub_ =
      nh.subscribe(qrcode_topic, 5, &TaskSearchManager::qrcodeDetectionCallback, this);
  thermal_detection_sub_ =
      nh.subscribe(thermal_topic, 5, &TaskSearchManager::thermalDetectionCallback, this);
  final_landing_marker_sub_ = nh.subscribe(
      final_landing_marker_topic, 5,
      &TaskSearchManager::finalLandingMarkerCallback, this);
  // 2026-07-23: 直接订阅FAST-LIO已外参校正的IMU-body点云，避免再经漂移世界z筛选墙体。
  if (radar_exit_detection_enabled_) {
    body_cloud_sub_ = nh.subscribe(radar_exit_body_cloud_topic_, 5,
                                   &TaskSearchManager::bodyCloudCallback, this);
  }
  if (rc_search_landing_enabled_) {
    rc_search_landing_sub_ = nh.subscribe(
        rc_search_landing_topic_, 10,
        &TaskSearchManager::rcSearchLandingCallback, this);
  }
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/mission/search_markers", 10, true);
  status_pub_ = nh.advertise<std_msgs::String>("/mission/task_status", 2, true);
  exit_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mission/exit_candidate", 2, true);
  // 2026-07-28: 与可撤销exit_candidate分离；后机只能接收最终锁存门，禁止追第一弯墙端候选。
  final_exit_pose_pub_ =
      nh.advertise<geometry_msgs::PoseStamped>("/mission/final_exit", 1, true);
  landing_target_pub_ =
      nh.advertise<geometry_msgs::PoseStamped>("/mission/landing_target", 2, true);
  landing_request_pub_ = nh.advertise<std_msgs::Bool>("/mission/landing_request", 2, true);
  publishLandingRequest(false);

  ROS_WARN("[task_search] stage-2 task search enabled=%d detections_required=%d, "
           "z=[%.2f, %.2f], revisit=%.2f, route_constraints=%s.",
           static_cast<int>(enabled_), static_cast<int>(require_stage2_detections_),
           min_search_height_, max_search_height_, revisit_radius_,
           hybrid_constraints_enabled_ ? "FUEL_DIFF" : "NATIVE_FUEL");
  // 2026-07-22: 启动日志明确输出“终点预测不等于目标”和门后FREE分支门槛，便于现场核对参数生效。
  ROS_WARN("[exit_mission] endpoint-hint-only + corridor-wall-end validation enabled, grid=%.2f "
           "min_path=%.2f outside_route=%.2f confirm=%d landing_marker_required=%d "
           "portal_half_width=[%.2f,%.2f] wall_end=%d/%d max_behind=%.2f.",
           exit_grid_resolution_, exit_min_geodesic_distance_,
           exit_endpoint_outside_route_min_length_, exit_confirmation_count_,
           static_cast<int>(require_final_landing_marker_),
           exit_portal_min_half_width_, exit_portal_max_half_width_,
           exit_wall_end_min_changed_count_, exit_wall_end_probe_count_,
           exit_candidate_max_behind_distance_);
  // 2026-07-23: 启动时明确打印局部雷达出口检测数据源和硬门槛，便于从日志判断是否真正接到点云。
  ROS_WARN("[exit_mission] body-radar wall-end detector enabled=%d topic=%s window=%.2fs "
           "elevation=+/-%.1fdeg inside_bins>=%d changed>=%d clear_frames>=%d.",
           static_cast<int>(radar_exit_detection_enabled_),
           radar_exit_body_cloud_topic_.c_str(), radar_exit_accumulation_time_,
           radar_exit_max_elevation_deg_, radar_exit_min_inside_bins_,
           radar_exit_min_changed_probes_, radar_exit_min_clear_ray_frames_);
  // 2026-07-23: 明确启动后的证据优先级，防止误以为body雷达仍可否决RViz累计地图里的清晰双墙断面。
  ROS_WARN("[exit_mission] evidence priority=LOCAL_OCCUPANCY_XY_THEN_BODY_RADAR; "
           "local map uses current_z+/-0.40m and requires bilateral wall end + >=60%% known FREE.");
  ROS_WARN("[exit_mission] RC search-landing trigger enabled=%d topic=%s CH%d "
           "low<=%d high>=%d hold=%.2fs; startup requires LOW before HIGH.",
           static_cast<int>(rc_search_landing_enabled_),
           rc_search_landing_topic_.c_str(), rc_search_landing_channel_ + 1,
           rc_search_landing_low_pwm_, rc_search_landing_high_pwm_,
           rc_search_landing_hold_sec_);
}

void TaskSearchManager::rcSearchLandingCallback(
    const mavros_msgs::RCInConstPtr& msg) {
  if (!enabled_ || !rc_search_landing_enabled_ ||
      rc_search_landing_triggered_) {
    return;
  }
  if (msg->channels.size() <=
      static_cast<std::size_t>(rc_search_landing_channel_)) {
    ROS_WARN_THROTTLE(2.0,
                      "[exit_mission] RC trigger waiting for CH%d; received only %zu channels.",
                      rc_search_landing_channel_ + 1, msg->channels.size());
    rc_search_landing_high_since_ = ros::Time(0);
    return;
  }

  const int pwm = static_cast<int>(msg->channels[rc_search_landing_channel_]);
  if (pwm <= rc_search_landing_low_pwm_) {
    if (!rc_search_landing_armed_) {
      ROS_WARN("[exit_mission] RC CH%d LOW observed (%d us); search-landing trigger armed.",
               rc_search_landing_channel_ + 1, pwm);
    }
    rc_search_landing_armed_ = true;
    rc_search_landing_high_since_ = ros::Time(0);
    return;
  }

  if (pwm < rc_search_landing_high_pwm_) {
    rc_search_landing_high_since_ = ros::Time(0);
    return;
  }
  if (!rc_search_landing_armed_) {
    ROS_WARN_THROTTLE(2.0,
                      "[exit_mission] ignore RC CH%d HIGH (%d us): move switch LOW first.",
                      rc_search_landing_channel_ + 1, pwm);
    return;
  }

  const ros::Time now = msg->header.stamp.isZero() ? ros::Time::now()
                                                    : msg->header.stamp;
  if (rc_search_landing_high_since_.isZero()) {
    rc_search_landing_high_since_ = now;
    return;
  }
  if ((now - rc_search_landing_high_since_).toSec() <
      rc_search_landing_hold_sec_) {
    return;
  }
  activateRcSearchLanding(now);
}

void TaskSearchManager::activateRcSearchLanding(const ros::Time& stamp) {
  if (mission_stage_ != SEARCH_CORRIDOR) {
    ROS_WARN("[exit_mission] ignore RC search-landing trigger: mission stage is already %d.",
             static_cast<int>(mission_stage_));
    rc_search_landing_triggered_ = true;
    rc_search_landing_armed_ = false;
    return;
  }
  if (!latest_robot_pose_valid_) {
    ROS_ERROR_THROTTLE(1.0,
                       "[exit_mission] reject RC search-landing trigger: robot pose unavailable.");
    rc_search_landing_high_since_ = ros::Time(0);
    return;
  }

  const ros::Time now = stamp.isZero() ? ros::Time::now() : stamp;
  const Eigen::Vector2d outward(std::cos(latest_robot_yaw_),
                               std::sin(latest_robot_yaw_));
  mission_stage_ = SEARCH_OUTSIDE_LANDING;
  mission_stage_start_ = now;
  active_goal_valid_ = false;
  search_exhausted_since_ = ros::Time(0);
  outside_search_anchor_ = latest_robot_pos_;
  outside_search_anchor_.z() = cruise_height_;

  // 人工拨杆表示 UAV0 已经位于出口外：当前位置兼作双机接力的最终出口锚点。
  exit_candidate_ = latest_robot_pos_;
  exit_portal_center_ = latest_robot_pos_;
  exit_outward_direction_ = outward;
  exit_candidate_confirmed_ = true;
  exit_portal_locked_ = true;
  exit_portal_outside_verified_ = true;

  geometry_msgs::PoseStamped final_exit_pose;
  final_exit_pose.header.stamp = now;
  final_exit_pose.header.frame_id = world_frame_;
  final_exit_pose.pose.position.x = latest_robot_pos_.x();
  final_exit_pose.pose.position.y = latest_robot_pos_.y();
  final_exit_pose.pose.position.z = latest_robot_pos_.z();
  final_exit_pose.pose.orientation.w = std::cos(0.5 * latest_robot_yaw_);
  final_exit_pose.pose.orientation.z = std::sin(0.5 * latest_robot_yaw_);
  final_exit_pose_pub_.publish(final_exit_pose);
  exit_pose_pub_.publish(final_exit_pose);

  rc_search_landing_triggered_ = true;
  rc_search_landing_start_requested_ = true;
  rc_search_landing_armed_ = false;
  rc_search_landing_high_since_ = ros::Time(0);
  publishSearchState();
  ROS_ERROR("[exit_mission] RC CH%d TRIGGERED search landing at current outside pose "
            "(%.2f, %.2f, %.2f), yaw=%.1fdeg; automatic exit detection bypassed.",
            rc_search_landing_channel_ + 1, latest_robot_pos_.x(),
            latest_robot_pos_.y(), latest_robot_pos_.z(),
            latest_robot_yaw_ * 180.0 / M_PI);
}

bool TaskSearchManager::consumeRcSearchLandingStartRequest() {
  if (!rc_search_landing_start_requested_) return false;
  rc_search_landing_start_requested_ = false;
  return true;
}

void TaskSearchManager::setMap(const std::shared_ptr<SDFMap>& map) {
  // 2026-07-13: 与 exploration_manager 共用同一累计 SDFMap，避免另订阅实时点云造成无记忆判断。
  sdf_map_ = map;
}

void TaskSearchManager::setCorridorFrame(const Eigen::Vector3d& origin,
                                         const Eigen::Vector3d& inside_dir) {
  // 入口内外是任务开始时的一次性定义：初始机头前方为门内，后方为门外。
  // corridor_search_manager 会周期性重发锁存的 workspace_lock；如果每次都覆盖这里，
  // 通道内已经随转弯更新的 stable_progress_direction_ 会被不断拉回初始航向。
  if (corridor_frame_received_) return;
  corridor_origin_ = origin;
  corridor_dir_ = inside_dir;
  corridor_dir_.z() = 0.0;
  if (corridor_dir_.head<2>().norm() < 1e-3) corridor_dir_ = Eigen::Vector3d::UnitX();
  corridor_dir_.normalize();
  entry_inside_yaw_ = std::atan2(corridor_dir_.y(), corridor_dir_.x());
  segment_origin_ = corridor_origin_.head<2>();
  segment_direction_ = corridor_dir_.head<2>().normalized();
  segment_high_water_ = 0.0;
  stable_progress_direction_ = corridor_dir_.head<2>().normalized();
  stable_progress_direction_valid_ = true;
  corridor_frame_received_ = true;
  ROS_WARN("[task_search] entry frame frozen once: origin=(%.2f,%.2f) inside_yaw=%.1fdeg; "
           "initial heading forward is permanently INSIDE.",
           corridor_origin_.x(), corridor_origin_.y(),
           std::atan2(corridor_dir_.y(), corridor_dir_.x()) * 180.0 / M_PI);
}

void TaskSearchManager::updateRobotPose(const Eigen::Vector3d& pos, double yaw) {
  if (!enabled_) return;
  {
    // 2026-07-23: 点云回调只借用连续里程计的XY和yaw落入短时局部平面；世界z有意不参与。
    std::lock_guard<std::mutex> lock(body_cloud_mutex_);
    latest_robot_pos_ = pos;
    latest_robot_yaw_ = yaw;
    latest_robot_pose_valid_ = true;
    ++latest_robot_pose_sequence_;
  }
  if (hybrid_constraints_enabled_ && segment_direction_.norm() > 1e-3) {
    const double progress =
        (pos.head<2>() - segment_origin_).dot(segment_direction_.normalized());
    if (!transition_active_)
      segment_high_water_ = std::max(segment_high_water_, progress);
    else if ((pos.head<2>() - transition_anchor_).dot(transition_outgoing_direction_) >= 0.60) {
      transition_active_ = false;
      segment_origin_ = transition_anchor_;
      segment_direction_ = transition_outgoing_direction_;
      segment_high_water_ = std::max(
          segment_high_water_,
          (pos.head<2>() - segment_origin_).dot(segment_direction_));
      ROS_INFO("[task_search] turn transition released after 0.60m forward progress.");
    }
  }
  // 2026-07-23: 用入口门平面只做一次“起点外 -> 通道内”锁存；之后不再根据局部几何
  // 反复重定义内外。0.35m滞回避免刚过门平面时的里程计抖动。
  if (corridor_frame_received_ && !corridor_entry_crossed_) {
    const double entry_progress = (pos - corridor_origin_).dot(corridor_dir_);
    if (entry_progress >= std::max(0.35, inside_return_margin_ + 0.25)) {
      corridor_entry_crossed_ = true;
      ROS_ERROR("[mission_region] START_OUTSIDE -> INSIDE_CORRIDOR, "
                "entry plane crossed by %.2fm; region is now latched INSIDE and "
                "the entry workspace lock is retired.",
                entry_progress);
    }
  }
  if (visited_positions_.empty() ||
      (pos.head<2>() - visited_positions_.back().head<2>()).norm() >= visit_spacing_) {
    // 实际位移方向不能重定义通道正方向。向右前方斜飞只是局部避障动作；若在这里
    // 跟随里程计切线，恢复器会把斜向误当成新的“直行”，随后无法回到原通道轴线。
    // stable_progress_direction_ 只在下方累计地图确认真实双墙拐弯后更新。
    visited_positions_.push_back(pos);
    while (static_cast<int>(visited_positions_.size()) > max_history_size_)
      visited_positions_.pop_front();
    publishSearchState();
  }
  // 2026-07-13: 出口候选在第二阶段持续后台更新，frontier 耗尽时无需临时等待重新建图。
  updateExitCandidate(pos);

  // 2026-07-21: 拓扑远端和门框确认只允许触发穿门状态机；通道内任何位置均不得直接发布降落。

  if (mission_stage_ == APPROACH_LANDING && final_landing_marker_.found) {
    const Eigen::Vector3d marker(final_landing_marker_.pose.pose.position.x,
                                 final_landing_marker_.pose.pose.position.y,
                                 final_landing_marker_.pose.pose.position.z);
    const Eigen::Vector3d approach(marker.x(), marker.y(),
                                   marker.z() + landing_approach_height_);
    if ((pos.head<2>() - marker.head<2>()).norm() <= landing_trigger_distance_ &&
        landingColumnSafe(marker, approach)) {
      mission_stage_ = LANDING;
      mission_stage_start_ = ros::Time::now();
      publishSearchState();
      publishLandingRequest(true);
      ROS_ERROR("[exit_mission] landing ArUco aligned at %.2f %.2f; "
                "landing request published.", marker.x(), marker.y());
    }
  }
  (void)yaw;
}

void TaskSearchManager::freezeExplorationInitialYaw(double yaw) {
  if (exploration_initial_yaw_frozen_) return;
  if (!std::isfinite(yaw)) return;
  exploration_initial_yaw_ = yaw;
  exploration_initial_yaw_frozen_ = true;
  ROS_INFO("[fuel_diff] exploration initial yaw frozen at %.1fdeg.",
           exploration_initial_yaw_ * 180.0 / M_PI);
}

void TaskSearchManager::fillExplorationConstraint(
    quadrotor_msgs::ExplorationMotionConstraint& msg) const {
  msg.enabled = corridor_frame_received_ || exploration_initial_yaw_frozen_;
  msg.exploration_initial_yaw = exploration_initial_yaw_;
  msg.entry_inside_yaw = entry_inside_yaw_;
  msg.entry_plane_active = corridor_frame_received_ && !corridor_entry_crossed_;
  msg.entry_origin.x = corridor_origin_.x();
  msg.entry_origin.y = corridor_origin_.y();
  msg.entry_origin.z = 0.0;
  msg.entry_direction.x = corridor_dir_.x();
  msg.entry_direction.y = corridor_dir_.y();
  msg.entry_direction.z = 0.0;
  msg.entry_back_margin = std::max(0.0, inside_return_margin_);
  msg.segment_origin.x = segment_origin_.x();
  msg.segment_origin.y = segment_origin_.y();
  msg.segment_origin.z = 0.0;
  msg.segment_direction.x = segment_direction_.x();
  msg.segment_direction.y = segment_direction_.y();
  msg.segment_direction.z = 0.0;
  msg.segment_high_water = segment_high_water_;
  msg.segment_regression_margin = 0.25;
  msg.transition_active = transition_active_;
  msg.transition_anchor.x = transition_anchor_.x();
  msg.transition_anchor.y = transition_anchor_.y();
  msg.transition_anchor.z = 0.0;
  msg.transition_incoming_direction.x = transition_incoming_direction_.x();
  msg.transition_incoming_direction.y = transition_incoming_direction_.y();
  msg.transition_incoming_direction.z = 0.0;
  msg.transition_outgoing_direction.x = transition_outgoing_direction_.x();
  msg.transition_outgoing_direction.y = transition_outgoing_direction_.y();
  msg.transition_outgoing_direction.z = 0.0;
  msg.transition_old_high_water = transition_old_high_water_;
  msg.transition_regression_margin = 0.45;
  msg.transition_radius = 0.80;
  msg.transition_corridor_half_width = 1.25;
  msg.completed_gates.clear();
  for (const auto& gate : completed_gates_) {
    quadrotor_msgs::ExplorationGate out;
    out.sequence = gate.sequence;
    out.center.x = gate.center.x();
    out.center.y = gate.center.y();
    out.center.z = 0.0;
    out.normal.x = gate.normal.x();
    out.normal.y = gate.normal.y();
    out.normal.z = 0.0;
    out.left_extent = gate.left_extent;
    out.right_extent = gate.right_extent;
    out.thickness = gate.thickness;
    msg.completed_gates.push_back(out);
  }
}

void TaskSearchManager::recordExplorationReached(const Eigen::Vector3d& goal) {
  recordSelectedGoal(goal);
  clearActiveGoal();
}

void TaskSearchManager::clearPendingTurnEvidence() {
  pending_turn_confirmations_ = 0;
  pending_turn_probe_origin_valid_ = false;
  pending_turn_last_evidence_ = ros::Time(0);
  pending_turn_pose_sequence_ = 0;
}

bool TaskSearchManager::confirmCorridorTurnEvidence(
    const Eigen::Vector3d& turn_direction) {
  const ros::Time now = ros::Time::now();
  if (turn_direction.head<2>().norm() < 1e-3) return false;

  const Eigen::Vector2d observed = turn_direction.head<2>().normalized();
  const double elapsed = pending_turn_last_evidence_.isZero()
                             ? std::numeric_limits<double>::infinity()
                             : (now - pending_turn_last_evidence_).toSec();
  const double angular_tolerance =
      recovery_turn_confirmation_angle_deg_ * M_PI / 180.0;
  const bool direction_consistent =
      pending_turn_confirmations_ > 0 &&
      std::acos(std::max(-1.0, std::min(1.0,
                                      pending_turn_direction_.dot(observed)))) <=
          angular_tolerance;
  // 在时间窗内累计同方向证据，不要求相邻规划周期连续命中。中间地图暂时缺失
  // 不清票；只有方向明显改变或时间窗过期才重新计数。
  if (!direction_consistent ||
      elapsed > recovery_turn_confirmation_accumulation_window_) {
    pending_turn_direction_ = observed;
    pending_turn_probe_origin_ = visited_positions_.empty()
                                     ? latest_robot_pos_
                                     : visited_positions_.back();
    pending_turn_probe_origin_valid_ = true;
    pending_turn_confirmations_ = 1;
    pending_turn_last_evidence_ = now;
    pending_turn_pose_sequence_ = latest_robot_pose_sequence_;
  } else if (elapsed >= recovery_turn_confirmation_min_interval_ &&
             latest_robot_pose_sequence_ != pending_turn_pose_sequence_) {
    pending_turn_direction_ =
        (pending_turn_direction_ * pending_turn_confirmations_ + observed).normalized();
    ++pending_turn_confirmations_;
    pending_turn_last_evidence_ = now;
    pending_turn_pose_sequence_ = latest_robot_pose_sequence_;
  }

  if (pending_turn_confirmations_ < recovery_turn_confirmation_count_) {
    ROS_WARN_THROTTLE(0.3,
                      "[task_search] mapped turn pending %d/%d yaw=%.1fdeg; "
                      "freeze probe=(%.2f,%.2f) and keep current corridor direction.",
                      pending_turn_confirmations_, recovery_turn_confirmation_count_,
                      std::atan2(observed.y(), observed.x()) * 180.0 / M_PI,
                      pending_turn_probe_origin_.x(), pending_turn_probe_origin_.y());
    return false;
  }
  clearPendingTurnEvidence();
  return true;
}

Eigen::Vector2d TaskSearchManager::stableProgressDirection() const {
  if (stable_progress_direction_valid_ && stable_progress_direction_.norm() > 1e-3)
    return stable_progress_direction_.normalized();
  if (corridor_frame_received_ && corridor_dir_.head<2>().norm() > 1e-3)
    return corridor_dir_.head<2>().normalized();
  return Eigen::Vector2d(std::cos(latest_robot_yaw_), std::sin(latest_robot_yaw_));
}

bool TaskSearchManager::corridorYawCorrectionDirection(
    double cur_yaw, Eigen::Vector3d& direction) const {
  if (!stable_progress_direction_valid_ && !corridor_frame_received_) return false;
  const Eigen::Vector2d stable = stableProgressDirection();
  const double correction_threshold =
      std::max(0.0, recovery_turn_yaw_release_angle_deg_) * M_PI / 180.0;
  if (!task_search::corridorYawCorrectionNeeded(
          stable, cur_yaw, correction_threshold))
    return false;
  direction = Eigen::Vector3d(stable.x(), stable.y(), 0.0);
  return true;
}

// 2026-07-23: 机体系点云仅按仰角保留近水平射线，再用最新XY/yaw放入1.5秒局部平面；
// 不使用点的世界z，因此FAST-LIO高度整体上飘不会把真实墙体移出检测层。
void TaskSearchManager::bodyCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  if (!radar_exit_detection_enabled_ || !msg || msg->width * msg->height == 0) return;

  Eigen::Vector2d sensor_xy;
  double yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(body_cloud_mutex_);
    if (!latest_robot_pose_valid_) return;
    sensor_xy = latest_robot_pos_.head<2>();
    yaw = latest_robot_yaw_;
  }

  BodyCloudFrame frame;
  frame.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  frame.sensor_xy = sensor_xy;
  frame.endpoints_xy.reserve(std::min<int>(
      radar_exit_max_points_per_frame_, static_cast<int>(msg->width * msg->height)));
  const double max_elevation =
      std::max(1.0, radar_exit_max_elevation_deg_) * M_PI / 180.0;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const int total_points = static_cast<int>(msg->width * msg->height);
  const int stride = std::max(
      1, total_points / std::max(1, radar_exit_max_points_per_frame_));

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (int index = 0; iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z, ++index) {
      if (index % stride != 0) continue;
      const double x = *iter_x;
      const double y = *iter_y;
      const double z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      const double horizontal_range = std::hypot(x, y);
      const double range = std::hypot(horizontal_range, z);
      if (range < radar_exit_min_range_ || range > radar_exit_max_range_ ||
          horizontal_range < 1e-3 ||
          std::fabs(std::atan2(z, horizontal_range)) > max_elevation)
        continue;
      frame.endpoints_xy.emplace_back(
          sensor_xy.x() + cos_yaw * x - sin_yaw * y,
          sensor_xy.y() + sin_yaw * x + cos_yaw * y);
      if (static_cast<int>(frame.endpoints_xy.size()) >=
          radar_exit_max_points_per_frame_)
        break;
    }
  } catch (const std::runtime_error& error) {
    ROS_ERROR_THROTTLE(2.0,
                       "[exit_radar] body cloud lacks float x/y/z fields: %s.",
                       error.what());
    return;
  }
  if (frame.endpoints_xy.empty()) return;

  std::lock_guard<std::mutex> lock(body_cloud_mutex_);
  body_cloud_frames_.push_back(std::move(frame));
  const ros::Time newest = body_cloud_frames_.back().stamp;
  while (!body_cloud_frames_.empty() &&
         (newest - body_cloud_frames_.front().stamp).toSec() >
             radar_exit_accumulation_time_)
    body_cloud_frames_.pop_front();
}

// 2026-07-23: 出口定义为“门内一段时间持续存在的左右通道墙，在同一前向截面后共同消失/外移，
// 且多帧真实雷达射线穿过中部净空”。孤立箱体没有足够纵向支撑，普通拐弯没有直穿自由射线。
bool TaskSearchManager::detectRadarExit(const Eigen::Vector3d& cur_pos,
                                        const Eigen::Vector2d& forward_hint,
                                        RadarExitResult& result) {
  if (!radar_exit_detection_enabled_ || forward_hint.norm() < 1e-3) return false;

  std::deque<BodyCloudFrame> frames;
  {
    std::lock_guard<std::mutex> lock(body_cloud_mutex_);
    const ros::Time now = ros::Time::now();
    while (!body_cloud_frames_.empty() &&
           (now - body_cloud_frames_.front().stamp).toSec() >
               radar_exit_accumulation_time_)
      body_cloud_frames_.pop_front();
    frames = body_cloud_frames_;
  }
  if (frames.size() < static_cast<size_t>(radar_exit_min_clear_ray_frames_)) {
    ROS_WARN_THROTTLE(2.0, "[exit_radar] waiting body cloud frames: %zu/%d.",
                      frames.size(), radar_exit_min_clear_ray_frames_);
    return false;
  }

  const Eigen::Vector2d forward = forward_hint.normalized();
  const Eigen::Vector2d lateral(-forward.y(), forward.x());
  struct LocalPoint {
    double longitudinal;
    double lateral;
  };
  std::vector<LocalPoint> points;
  for (const auto& frame : frames) {
    for (const auto& endpoint : frame.endpoints_xy) {
      const Eigen::Vector2d relative = endpoint - cur_pos.head<2>();
      points.push_back({relative.dot(forward), relative.dot(lateral)});
    }
  }
  if (points.size() < 80) {
    ROS_WARN_THROTTLE(2.0, "[exit_radar] too few horizontal returns: %zu.", points.size());
    return false;
  }

  auto median = [](std::vector<double> values) {
    if (values.empty()) return std::numeric_limits<double>::infinity();
    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    return values[middle];
  };
  const double bin_step = std::max(0.08, radar_exit_wall_bin_step_);
  const double min_side = std::max(0.25, 0.75 * exit_portal_min_half_width_);
  const double max_side = std::max(min_side + 0.20, exit_portal_max_half_width_ + 0.55);
  auto nearestSides = [&](double longitudinal, double& left, double& right) {
    left = std::numeric_limits<double>::infinity();
    right = std::numeric_limits<double>::infinity();
    for (const auto& point : points) {
      if (std::fabs(point.longitudinal - longitudinal) > 0.55 * bin_step) continue;
      if (point.lateral <= -min_side && -point.lateral <= max_side)
        left = std::min(left, -point.lateral);
      if (point.lateral >= min_side && point.lateral <= max_side)
        right = std::min(right, point.lateral);
    }
  };

  bool found = false;
  double best_score = -std::numeric_limits<double>::infinity();
  // 2026-07-23: 分项记录局部雷达拒绝原因，现场日志可直接区分墙历史不足、障碍堵门和门外射线不足。
  int rejected_inside_wall = 0;
  int rejected_wall_not_ended = 0;
  int rejected_center_blocked = 0;
  int rejected_clear_rays = 0;
  const double search_step = std::max(0.05, radar_exit_search_step_);
  for (double section = radar_exit_search_min_forward_;
       section <= radar_exit_search_max_forward_ + 1e-6; section += search_step) {
    std::vector<double> raw_left;
    std::vector<double> raw_right;
    const int inside_bins = std::max(
        radar_exit_min_inside_bins_,
        static_cast<int>(std::floor(radar_exit_inside_length_ / bin_step)));
    for (int bin = 1; bin <= inside_bins; ++bin) {
      double left, right;
      nearestSides(section - bin * bin_step, left, right);
      if (std::isfinite(left)) raw_left.push_back(left);
      if (std::isfinite(right)) raw_right.push_back(right);
    }
    if (static_cast<int>(raw_left.size()) < radar_exit_min_inside_bins_ ||
        static_cast<int>(raw_right.size()) < radar_exit_min_inside_bins_) {
      ++rejected_inside_wall;
      continue;
    }
    const double wall_left = median(raw_left);
    const double wall_right = median(raw_right);
    int left_support = 0;
    int right_support = 0;
    for (double value : raw_left)
      if (std::fabs(value - wall_left) <= radar_exit_wall_track_tolerance_)
        ++left_support;
    for (double value : raw_right)
      if (std::fabs(value - wall_right) <= radar_exit_wall_track_tolerance_)
        ++right_support;
    if (left_support < radar_exit_min_inside_bins_ ||
        right_support < radar_exit_min_inside_bins_) {
      ++rejected_inside_wall;
      continue;
    }

    int bilateral_changed = 0;
    const int outside_probes = std::max(1, radar_exit_outside_probe_count_);
    for (int probe = 1; probe <= outside_probes; ++probe) {
      double left, right;
      nearestSides(section + probe * bin_step * 1.5, left, right);
      const bool left_changed =
          !std::isfinite(left) || left >= wall_left + radar_exit_min_wall_shift_;
      const bool right_changed =
          !std::isfinite(right) || right >= wall_right + radar_exit_min_wall_shift_;
      if (left_changed && right_changed) ++bilateral_changed;
    }
    if (bilateral_changed < radar_exit_min_changed_probes_) {
      ++rejected_wall_not_ended;
      continue;
    }

    // 中心近处的水平回波意味着箱体/墙挡住开口；允许极少量离群点，但不把未知空间当FREE。
    const double center_offset = 0.5 * (wall_right - wall_left);
    int blocking_returns = 0;
    for (const auto& point : points) {
      if (point.longitudinal >= section + 0.08 &&
          point.longitudinal <= section + radar_exit_clear_depth_ &&
          std::fabs(point.lateral - center_offset) <= radar_exit_clear_half_width_)
        ++blocking_returns;
    }
    if (blocking_returns > std::max<int>(3, frames.size())) {
      ++rejected_center_blocked;
      continue;
    }

    // 每一帧至少有一条回波射线穿过候选截面的中央并打到更远处，才算真实观测到门外净空。
    int clear_ray_frames = 0;
    for (const auto& frame : frames) {
      const Eigen::Vector2d sensor_relative =
          frame.sensor_xy - cur_pos.head<2>();
      const double sensor_s = sensor_relative.dot(forward);
      const double sensor_l = sensor_relative.dot(lateral);
      bool frame_clear = false;
      for (const auto& endpoint : frame.endpoints_xy) {
        const Eigen::Vector2d endpoint_relative =
            endpoint - cur_pos.head<2>();
        const double endpoint_s = endpoint_relative.dot(forward);
        if (sensor_s >= section - 0.03 ||
            endpoint_s <= section + std::max(0.45, radar_exit_clear_depth_))
          continue;
        const double ratio = (section - sensor_s) / (endpoint_s - sensor_s);
        const double endpoint_l = endpoint_relative.dot(lateral);
        const double crossing_l = sensor_l + ratio * (endpoint_l - sensor_l);
        if (std::fabs(crossing_l - center_offset) <= radar_exit_clear_half_width_) {
          frame_clear = true;
          break;
        }
      }
      if (frame_clear) ++clear_ray_frames;
    }
    if (clear_ray_frames < radar_exit_min_clear_ray_frames_) {
      ++rejected_clear_rays;
      continue;
    }

    const double score = 0.35 * std::min(left_support, right_support) +
                         0.80 * bilateral_changed + 0.25 * clear_ray_frames -
                         0.50 * std::fabs(wall_left - wall_right) -
                         0.10 * std::max(0.0, section);
    if (!found || score > best_score) {
      found = true;
      best_score = score;
      result.portal_center = cur_pos;
      result.portal_center.head<2>() += section * forward + center_offset * lateral;
      result.outward_direction = forward;
      result.inside_left_support = left_support;
      result.inside_right_support = right_support;
      result.bilateral_changed = bilateral_changed;
      result.clear_ray_frames = clear_ray_frames;
      result.confidence = std::min(
          0.98, 0.40 + 0.04 * std::min(left_support, right_support) +
                    0.06 * bilateral_changed + 0.025 * clear_ray_frames);
    }
  }
  if (!found) {
    ROS_WARN_THROTTLE(2.0,
                      "[exit_radar] no local wall-end: horizontal_points=%zu frames=%zu "
                      "reject_inside_wall=%d wall_not_ended=%d center_blocked=%d "
                      "clear_rays=%d.",
                      points.size(), frames.size(), rejected_inside_wall,
                      rejected_wall_not_ended, rejected_center_blocked,
                      rejected_clear_rays);
    return false;
  }
  ROS_INFO_THROTTLE(
      1.0,
      "[exit_radar] local portal center=(%.2f,%.2f) wall_support=L%d/R%d "
      "changed=%d/%d clear_ray_frames=%d confidence=%.2f.",
      result.portal_center.x(), result.portal_center.y(),
      result.inside_left_support, result.inside_right_support,
      result.bilateral_changed, radar_exit_outside_probe_count_,
      result.clear_ray_frames, result.confidence);
  return true;
}

double TaskSearchManager::minDistance2D(
    const Eigen::Vector3d& point, const std::deque<Eigen::Vector3d>& history) const {
  double min_dist = std::numeric_limits<double>::infinity();
  for (const auto& item : history)
    min_dist = std::min(min_dist, (point.head<2>() - item.head<2>()).norm());
  return history.empty() ? 100.0 : min_dist;
}

double TaskSearchManager::wrapYaw(double yaw) const {
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

double TaskSearchManager::knownHorizontalClearance(
    const Eigen::Vector3d& point) const {
  if (!sdf_map_ || !sdf_map_->isInMap(point) ||
      sdf_map_->getOccupancy(point) != SDFMap::FREE)
    return 0.0;
  constexpr int kDirections = 16;
  const double step = std::max(0.05, sdf_map_->getResolution());
  return frontier_clearance::minimumKnownRadialClearance(
      step, clearance_reward_full_, kDirections,
      [&](int direction, double distance) {
        const double angle = 2.0 * M_PI * static_cast<double>(direction) /
                             static_cast<double>(kDirections);
        Eigen::Vector3d probe = point;
        probe.x() += distance * std::cos(angle);
        probe.y() += distance * std::sin(angle);
        for (double z_offset : {-0.06, 0.0, 0.06}) {
          probe.z() = point.z() + z_offset;
          if (!sdf_map_->isInMap(probe) ||
              sdf_map_->getOccupancy(probe) != SDFMap::FREE)
            return false;
        }
        return true;
      });
}

int TaskSearchManager::selectSearchCandidate(
    const std::vector<Eigen::Vector3d>& points, const std::vector<double>& yaws,
    const std::vector<std::vector<Eigen::Vector3d>>& frontiers,
    const Eigen::Vector3d& cur_pos, double cur_yaw) {
  if (!enabled_ || points.empty()) return -1;

  const double current_progress = corridor_frame_received_
                                      ? (cur_pos - corridor_origin_).dot(corridor_dir_)
                                      : 0.0;
  const bool entry_forward_phase =
      corridor_frame_received_ && current_progress < entry_forward_distance_;
  // frontier的前后语义与恢复器使用同一条持久通道轴线。机头转动和避障斜飞
  // 都不能让候选排序跟着偏转；真实拐弯由累计地图确认后才更新该轴线。
  Eigen::Vector2d motion_forward = stableProgressDirection();
  if (hybrid_constraints_enabled_ && exploration_initial_yaw_frozen_ &&
      !corridor_frame_received_)
    motion_forward = Eigen::Vector2d(std::cos(exploration_initial_yaw_),
                                     std::sin(exploration_initial_yaw_));
  if (motion_forward.norm() < 1e-3)
    motion_forward = Eigen::Vector2d(std::cos(cur_yaw), std::sin(cur_yaw));
  motion_forward.normalize();
  int best_idx = -1;
  int best_non_backward_idx = -1;
  int rejected_revisit = 0;
  int rejected_failed = 0;
  int rejected_door_return = 0;
  int rejected_exit_regression = 0;
  double best_score = std::numeric_limits<double>::infinity();
  double best_non_backward_score = std::numeric_limits<double>::infinity();
  double best_clearance = 0.0;
  double best_clearance_reward = 0.0;
  double best_non_backward_clearance = 0.0;
  double best_non_backward_clearance_reward = 0.0;

  // 2026-07-20: 终点未确认（或拓扑距离不足）时该保护完全关闭，下面原有的全体最优回退
  // 继续允许U形通道掉头；只有可信最终出口出现后才开始约束普通frontier。
  const bool final_exit_guard = finalExitFrontierGuardActive();
  const double current_distance_to_exit = final_exit_guard
      ? (cur_pos.head<2>() - exit_candidate_.head<2>()).norm()
      : -1.0;

  // 2026-07-13: 当前目标仍存在且尚未到达时至少保持一段时间，避免碰撞检查触发后在柱子两侧来回跳点。
  const Eigen::Vector2d active_delta = active_goal_.head<2>() - cur_pos.head<2>();
  const bool active_goal_is_backward =
      active_delta.norm() > 0.35 &&
      active_delta.normalized().dot(motion_forward) < backward_cos_threshold_;
  const double active_goal_distance_to_exit = final_exit_guard
      ? (active_goal_.head<2>() - exit_candidate_.head<2>()).norm()
      : 0.0;
  const bool active_goal_is_exit_regression = final_exit_guard &&
      active_goal_distance_to_exit > exit_frontier_local_adjust_radius_ &&
      active_goal_distance_to_exit >
          current_distance_to_exit + exit_frontier_max_distance_increase_;
  // 2026-07-23: 最终出口锁存后，普通FUEL活动目标不得越过门平面；穿门只由CROSS_EXIT发布。
  const bool active_goal_crosses_locked_exit =
      exit_portal_locked_ && exit_outward_direction_.norm() > 1e-3 &&
      (active_goal_.head<2>() - exit_portal_center_.head<2>())
              .dot(exit_outward_direction_.normalized()) >= -0.05;
  if (active_goal_valid_ && !active_goal_is_backward && !active_goal_is_exit_regression &&
      !active_goal_crosses_locked_exit &&
      (ros::Time::now() - active_goal_stamp_).toSec() < min_goal_hold_time_ &&
      (cur_pos.head<2>() - active_goal_.head<2>()).norm() > 0.35 &&
      !goalTemporarilyBlocked(active_goal_)) {
    int matched = -1;
    double matched_dist = 0.55;
    for (size_t i = 0; i < points.size(); ++i) {
      const double dist = (points[i].head<2>() - active_goal_.head<2>()).norm();
      if (dist < matched_dist) {
        matched_dist = dist;
        matched = static_cast<int>(i);
      }
    }
    if (matched >= 0) {
      ROS_WARN("[task_search] keep active goal idx=%d for %.2fs, match=%.2fm.", matched,
               (ros::Time::now() - active_goal_stamp_).toSec(), matched_dist);
      return matched;
    }
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3d& point = points[i];
    if (goalTemporarilyBlocked(point)) {
      ++rejected_failed;
      continue;
    }

    if (corridor_frame_received_) {
      const double candidate_progress = (point - corridor_origin_).dot(corridor_dir_);
      // 门后初始推进阶段只准保持或增加入口法向进度。外部空间更开阔时，不能仅靠代价
      // 惩罚期待FUEL自觉向前，否则大frontier收益仍会诱导其掉头。
      if (entry_forward_phase && candidate_progress < current_progress - 0.10) {
        ++rejected_door_return;
        continue;
      }
      // 2026-07-13: 深入通道后禁止再选入口门平面附近的点，修复日志中重新选择 (1.70,-0.19) 的回头轨迹。
      if (current_progress >= entry_forward_distance_ &&
          candidate_progress < inside_return_margin_) {
        ++rejected_door_return;
        continue;
      }
    }

    // 2026-07-20: 最终出口已经确认且拓扑距离达到直降门槛后，普通frontier若既不在
    // 出口局部调整区，又会让distance_to_exit显著增加，则拒绝该回头候选。
    if (final_exit_guard) {
      // 2026-07-23: 锁门后先关掉跨门的普通frontier，避免无人机先飞到门外再把门框方向带乱；
      // 门内局部调整仍保留，真正穿门由出口状态机单独执行。
      if (exit_portal_locked_ && exit_outward_direction_.norm() > 1e-3) {
        const double candidate_exit_side =
            (point.head<2>() - exit_portal_center_.head<2>())
                .dot(exit_outward_direction_.normalized());
        if (candidate_exit_side >= -0.05) {
          ++rejected_exit_regression;
          continue;
        }
      }
      const double candidate_distance_to_exit =
          (point.head<2>() - exit_candidate_.head<2>()).norm();
      const bool local_exit_adjustment =
          candidate_distance_to_exit <= exit_frontier_local_adjust_radius_;
      if (!local_exit_adjustment &&
          candidate_distance_to_exit >
              current_distance_to_exit + exit_frontier_max_distance_increase_) {
        ++rejected_exit_regression;
        continue;
      }
    }
    const double travel = (point - cur_pos).norm();
    const double novelty = minDistance2D(point, visited_positions_);
    const double goal_distance = minDistance2D(point, selected_goals_);
    int repeat_count = 0;
    for (const auto& goal : selected_goals_)
      if ((point.head<2>() - goal.head<2>()).norm() < repeat_goal_radius_) ++repeat_count;

    // 2026-07-13: 已搜索且反复选中的局部 frontier 直接失去任务资格，防止在通道拐弯处生成零长度轨迹。
    if (novelty < revisit_radius_ && repeat_count >= max_goal_repeats_) {
      ++rejected_revisit;
      continue;
    }

    const double candidate_yaw = i < yaws.size() ? yaws[i] : cur_yaw;
    const double yaw_cost = std::fabs(wrapYaw(candidate_yaw - cur_yaw));
    const double height_cost = std::fabs(clampSearchHeight(point.z()) - cruise_height_);
    const double frontier_gain =
        i < frontiers.size() ? std::log1p(static_cast<double>(frontiers[i].size())) : 0.0;
    const double clearance = clearance_reward_enabled_
                                 ? knownHorizontalClearance(point)
                                 : 0.0;
    const double clearance_reward = clearance_reward_enabled_
        ? frontier_clearance::clearanceReward(
              clearance, clearance_reward_start_, clearance_reward_full_,
              clearance_reward_max_)
        : 0.0;
    double score = travel_weight_ * travel + yaw_weight_ * yaw_cost +
                   height_weight_ * height_cost - novelty_weight_ * std::min(2.0, novelty) -
                   frontier_gain_weight_ * frontier_gain - clearance_reward;
    // 前向只作为软偏好：正前方获得完整加分，斜前方按投影加分，横向和后方不加分也不拒绝。
    // 使用稳定通道方向而非瞬时机头角，避免避障转头时把合法横移误判成回头。
    const Eigen::Vector2d candidate_delta = point.head<2>() - cur_pos.head<2>();
    if (prefer_motion_forward_ && candidate_delta.norm() > 1e-3) {
      const double forward_alignment =
          candidate_delta.normalized().dot(motion_forward.normalized());
      score += task_search::viewpointDirectionScoreAdjustment(
          forward_alignment, forward_viewpoint_bonus_, backward_viewpoint_penalty_);
    }
    // 2026-07-14: 仅对仍有效且尚未到达的活动目标施加连续性代价；失败目标已由
    // reportGoalFailure 失效，不会阻止规划器绕开真正不可达的位置。
    if (active_goal_valid_ && !goalTemporarilyBlocked(active_goal_) &&
        (cur_pos.head<2>() - active_goal_.head<2>()).norm() > 0.35) {
      score += goal_switch_weight_ *
               (point.head<2>() - active_goal_.head<2>()).norm();
    }
    if (goal_distance < repeat_goal_radius_) score += repeat_penalty_;

    // 2026-07-13: 固定门方向只用于刚穿门后的短距离；进入通道深处后由未搜索覆盖决定方向，允许正常拐弯。
    if (entry_forward_phase) {
      const double progress = (point - corridor_origin_).dot(corridor_dir_);
      score -= entry_forward_weight_ * std::max(0.0, progress - current_progress);
      if (progress < current_progress - 0.10) score += repeat_penalty_;
    }
    if (score < best_score) {
      best_score = score;
      best_idx = static_cast<int>(i);
      best_clearance = clearance;
      best_clearance_reward = clearance_reward;
    }
    const bool is_backward =
        candidate_delta.norm() > 0.35 &&
        candidate_delta.normalized().dot(motion_forward) < backward_cos_threshold_;
    if (!is_backward && score < best_non_backward_score) {
      best_non_backward_score = score;
      best_non_backward_idx = static_cast<int>(i);
      best_non_backward_clearance = clearance;
      best_non_backward_clearance_reward = clearance_reward;
    }
  }

  // 后方候选仍执行硬拒绝；前向加分只负责在合法的前向/横向/斜向候选中加速排序。
  if (prefer_motion_forward_) {
    if (best_non_backward_idx >= 0) {
      best_idx = best_non_backward_idx;
      best_score = best_non_backward_score;
      best_clearance = best_non_backward_clearance;
      best_clearance_reward = best_non_backward_clearance_reward;
    } else if (best_idx >= 0 && !allow_search_backtrack_) {
      ROS_ERROR_THROTTLE(1.0,
                         "[task_search] reject backward-only frontier set; hold for forward map "
                         "update instead of returning through completed corridor.");
      best_idx = -1;
      best_score = std::numeric_limits<double>::infinity();
    }
  }

  ROS_WARN("[task_search] candidates=%zu selected=%d rejected_revisit=%d rejected_failed=%d "
           "rejected_door=%d rejected_exit_regression=%d exit_guard=%d "
           "current_exit_distance=%.2f entry_forward=%d forward_candidate=%d score=%.2f "
           "clearance=%.2f reward=%.2f.",
           points.size(), best_idx, rejected_revisit, rejected_failed, rejected_door_return,
           rejected_exit_regression, static_cast<int>(final_exit_guard),
           current_distance_to_exit, static_cast<int>(entry_forward_phase),
           best_non_backward_idx, best_score, best_clearance,
           best_clearance_reward);
  return best_idx;
}

// 2026-07-28: 用累计占据地图识别“旧前向封闭、侧向通道连续”的正常弯道；
// 允许一侧墙在拐角中断，但另一侧必须有连续地图轮廓，避免孤立障碍物误触发。
bool TaskSearchManager::inferOccupancyTurnDirection(
    const Eigen::Vector3d& travel_direction, Eigen::Vector3d& turn_direction,
    double& forward_free_length, double& turn_free_length) const {
  forward_free_length = 0.0;
  turn_free_length = 0.0;
  if (!recovery_occupancy_turn_enabled_ || !sdf_map_ || visited_positions_.empty() ||
      travel_direction.head<2>().norm() < 1e-3)
    return false;

  // 第一票出现后冻结当时看到拐角的地图观察点。无人机随后斜移/进入新通道时，
  // 若继续用最新位置发射旧前向射线，前墙会落到身后，可靠的第一票反而无法复核。
  const Eigen::Vector3d origin =
      pending_turn_probe_origin_valid_ ? pending_turn_probe_origin_
                                       : visited_positions_.back();
  const Eigen::Vector2d travel = travel_direction.head<2>().normalized();
  const double probe_step = std::max(0.05, recovery_turn_probe_step_);
  auto knownFreeLength = [&](const Eigen::Vector3d& ray_origin,
                             const Eigen::Vector2d& direction) {
    double free_length = 0.0;
    for (double distance = probe_step; distance <= recovery_turn_probe_length_ + 1e-6;
         distance += probe_step) {
      Eigen::Vector3d probe = ray_origin;
      probe.head<2>() += distance * direction;
      // 这里只识别地图轮廓，不签发飞行许可。UNKNOWN和占据都终止射线，足迹安全仍由A*复核。
      if (!sdf_map_->isInMap(probe) ||
          sdf_map_->getOccupancy(probe) != SDFMap::FREE)
        break;
      free_length = distance;
    }
    return free_length;
  };
  forward_free_length = knownFreeLength(origin, travel);
  Eigen::Vector3d forward_end = origin;
  forward_end.head<2>() +=
      std::min(recovery_turn_probe_length_, forward_free_length + probe_step) * travel;
  // 旧通道前方必须有雷达已建图的实际占据，UNKNOWN不算死路。
  // 确认旧轴已终止后，不再用新旧自由长度差或固定最小延伸距离否决转弯。
  const bool old_direction_blocked =
      forward_free_length + probe_step < recovery_turn_probe_length_ + 1e-6 &&
      mapRelativeColumnOccupied(forward_end, origin.z());
  if (!old_direction_blocked) return false;

  // 不从无人机当前位置横向打射线，而把虚拟观察点提前放到前方墙前。这样地图刚形成明显
  // L形/弧形轮廓时就能看到侧向通道，无需先横移进入新通道1m以上。
  Eigen::Vector3d contour_origin = origin;
  contour_origin.head<2>() += std::max(0.0, forward_free_length - 0.20) * travel;
  double best_score = -std::numeric_limits<double>::infinity();
  Eigen::Vector2d best_direction = travel;
  const double min_angle = recovery_turn_min_angle_deg_ * M_PI / 180.0;
  const double max_angle = recovery_turn_max_angle_deg_ * M_PI / 180.0;
  for (double angle = min_angle; angle <= max_angle + 1e-6; angle += M_PI / 12.0) {
    for (double sign : {-1.0, 1.0}) {
      const double signed_angle = sign * angle;
      const Eigen::Vector2d direction(
          std::cos(signed_angle) * travel.x() - std::sin(signed_angle) * travel.y(),
          std::sin(signed_angle) * travel.x() + std::cos(signed_angle) * travel.y());
      if (!hybrid_constraints_enabled_ && corridor_frame_received_ &&
          !task_search::insideForwardHalfPlane(direction,
                                               corridor_dir_.head<2>()))
        continue;
      const double free_length = knownFreeLength(contour_origin, direction);
      if (free_length <= 1e-6) continue;
      const double score = free_length - 0.05 * std::fabs(angle);
      if (score > best_score) {
        best_score = score;
        best_direction = direction;
        turn_free_length = free_length;
      }
    }
  }
  if (!std::isfinite(best_score)) return false;
  turn_direction = Eigen::Vector3d(best_direction.x(), best_direction.y(), 0.0);
  ROS_ERROR_THROTTLE(
      0.5,
      "[task_search] OCCUPANCY TURN selected yaw=%.1fdeg old_free=%.2fm "
      "branch_free=%.2fm contour_probe=(%.2f,%.2f); old direction stopped and "
      "the single free branch was selected.",
      std::atan2(best_direction.y(), best_direction.x()) * 180.0 / M_PI,
      forward_free_length, turn_free_length, contour_origin.x(), contour_origin.y());
  return true;
}

void TaskSearchManager::commitCorridorTurn(
    const Eigen::Vector3d& turn_direction) {
  if (turn_direction.head<2>().norm() < 1e-3) return;
  Eigen::Vector2d incoming = stableProgressDirection();
  if (incoming.norm() < 1e-3) incoming = corridor_dir_.head<2>();
  incoming.normalize();
  const Eigen::Vector2d outgoing = turn_direction.head<2>().normalized();
  const double signed_angle = std::atan2(
      incoming.x() * outgoing.y() - incoming.y() * outgoing.x(),
      incoming.dot(outgoing));

  Eigen::Vector2d anchor = visited_positions_.empty()
                               ? corridor_origin_.head<2>()
                               : visited_positions_.back().head<2>();
  {
    std::lock_guard<std::mutex> lock(body_cloud_mutex_);
    if (latest_robot_pose_valid_) anchor = latest_robot_pos_.head<2>();
  }
  latest_turn_anchor_ = anchor;
  latest_turn_incoming_direction_ = incoming;
  latest_turn_anchor_valid_ = true;
  if (hybrid_constraints_enabled_) {
    CompletedGate gate;
    gate.sequence = next_gate_sequence_++;
    gate.center = anchor;
    gate.normal = incoming;
    gate.left_extent = 1.45;
    gate.right_extent = 1.45;
    gate.thickness = 0.30;
    completed_gates_.push_back(gate);
    if (completed_gates_.size() > 64)
      completed_gates_.erase(completed_gates_.begin());
    transition_active_ = true;
    transition_anchor_ = anchor;
    transition_incoming_direction_ = incoming;
    transition_outgoing_direction_ = outgoing;
    transition_old_high_water_ = segment_high_water_;
    segment_origin_ = anchor;
    segment_direction_ = outgoing;
    segment_high_water_ = 0.0;
  }
  stable_progress_direction_ = outgoing;
  stable_progress_direction_valid_ = true;
  // A confirmed turn starts a new FUEL segment. Keep the yaw reference tied to
  // that segment instead of letting the instantaneous body yaw redefine
  // which frontier is considered "in front".
  if (hybrid_constraints_enabled_) {
    exploration_initial_yaw_ = std::atan2(outgoing.y(), outgoing.x());
    exploration_initial_yaw_frozen_ = true;
  }
  turn_yaw_follow_latch_.arm(outgoing);
  ROS_ERROR("[task_search] confirmed %s turn %.1fdeg at (%.2f,%.2f); "
            "new wall-parallel yaw=%.1fdeg and latest completed direction is no-return.",
            signed_angle >= 0.0 ? "LEFT" : "RIGHT",
            std::fabs(signed_angle) * 180.0 / M_PI, anchor.x(), anchor.y(),
            std::atan2(outgoing.y(), outgoing.x()) * 180.0 / M_PI);
}

bool TaskSearchManager::mappedCorridorDirection(
    double cur_yaw, Eigen::Vector3d& direction) {
  // 入口后的第一段直线尚未走完时，门外开放区和门框遮挡不是拐弯。
  if (entryForwardPhaseActive()) {
    clearPendingTurnEvidence();
    return false;
  }

  Eigen::Vector2d latched_direction;
  if (turn_yaw_follow_latch_.lockedDirection(latched_direction)) {
    direction = Eigen::Vector3d(latched_direction.x(), latched_direction.y(), 0.0);
    return true;
  }

  // 已确认的转弯尚未沿新通道离开转角时，不得把同一片转角地图再次解释成
  // 另一条新通道。保持刚提交的通道方向，走出既有0.60m过渡区后再恢复检测。
  if (transition_active_) {
    clearPendingTurnEvidence();
    ROS_INFO_THROTTLE(
        0.5,
        "[task_search] turn transition active; keep committed corridor direction "
        "and suppress repeated turn detection."
    );
    return false;
  }

  Eigen::Vector2d stable = stableProgressDirection();
  if (stable.norm() < 1e-3)
    stable = Eigen::Vector2d(std::cos(cur_yaw), std::sin(cur_yaw));
  const Eigen::Vector3d travel(stable.x(), stable.y(), 0.0);
  double forward_free_length = 0.0;
  double turn_free_length = 0.0;
  if (!inferOccupancyTurnDirection(
          travel, direction, forward_free_length, turn_free_length))
    return false;
  if (!confirmCorridorTurnEvidence(direction)) return false;
  commitCorridorTurn(direction);
  // 新确认的这一周期立即生效。锁存方向只能由实际yaw连续达标后的FSM显式释放，
  // 这样30度分段之间不会重新做地图转弯检测。
  if (!turn_yaw_follow_latch_.lockedDirection(latched_direction))
    return false;
  direction = Eigen::Vector3d(latched_direction.x(), latched_direction.y(), 0.0);
  return true;
}

bool TaskSearchManager::entryForwardPhaseActive() const {
  if (!corridor_frame_received_) return true;
  Eigen::Vector3d position = visited_positions_.empty()
                                 ? corridor_origin_
                                 : visited_positions_.back();
  {
    std::lock_guard<std::mutex> lock(body_cloud_mutex_);
    if (latest_robot_pose_valid_) position = latest_robot_pos_;
  }
  return (position - corridor_origin_).dot(corridor_dir_) <
         entry_forward_distance_;
}

// 局部恢复只读取已确认的通道轴线，不在候选排序过程中检测或提交转弯。
Eigen::Vector3d TaskSearchManager::recoveryForwardDirection(double cur_yaw) {
  Eigen::Vector2d latched_direction;
  if (turn_yaw_follow_latch_.lockedDirection(latched_direction))
    return Eigen::Vector3d(latched_direction.x(), latched_direction.y(), 0.0);

  Eigen::Vector2d stable = stableProgressDirection();
  if (stable.norm() < 1e-3)
    stable = Eigen::Vector2d(std::cos(cur_yaw), std::sin(cur_yaw));
  if (!hybrid_constraints_enabled_ && corridor_frame_received_ &&
      !task_search::insideForwardHalfPlane(stable, corridor_dir_.head<2>()))
    stable = corridor_dir_.head<2>().normalized();
  return Eigen::Vector3d(stable.x(), stable.y(), 0.0);
}

// 2026-07-28: 与普通候选使用相同角度阈值，避免恢复器和frontier对“后退”的定义不一致。
bool TaskSearchManager::isRecoveryDirectionBackward(
    const Eigen::Vector3d& direction, double cur_yaw) {
  if (!hybrid_constraints_enabled_ && corridor_frame_received_ &&
      !task_search::insideForwardHalfPlane(direction.head<2>(),
                                           corridor_dir_.head<2>()))
    return true;
  const Eigen::Vector3d forward = recoveryForwardDirection(cur_yaw);
  return direction.head<2>().norm() > 1e-3 &&
         direction.head<2>().normalized().dot(forward.head<2>()) <
             backward_cos_threshold_;
}

std::vector<Eigen::Vector3d> TaskSearchManager::recoveryDirections(double cur_yaw) {
  const Eigen::Vector3d forward = recoveryForwardDirection(cur_yaw);
  const double travel_yaw = std::atan2(forward.y(), forward.x());

  // 2026-07-23: 最新视频中直行短路径被膨胀栅格拒绝后，旧恢复器只试0/±45/±90度，
  // 会漏掉仍在前方半平面内的可行绕障方向；改为每15度扫描前方半圆，仍不生成后退方向。
  std::vector<double> offsets = {
      0.0,
      M_PI / 12.0, -M_PI / 12.0,
      M_PI / 6.0, -M_PI / 6.0,
      M_PI_4, -M_PI_4,
      M_PI / 3.0, -M_PI / 3.0,
      5.0 * M_PI / 12.0, -5.0 * M_PI / 12.0,
      M_PI_2, -M_PI_2};
  // 2026-07-28: 始终把±135度和180度交给“短回撤”候选池；上层将其严格限制为一次、最多0.45m，
  // 不会让普通FUEL frontier获得向后探索资格，也不会形成连续直线倒车。
  offsets.push_back(3.0 * M_PI_4);
  offsets.push_back(-3.0 * M_PI_4);
  offsets.push_back(M_PI);
  std::vector<std::pair<double, Eigen::Vector3d>> scored;
  for (double offset : offsets) {
    const double yaw = wrapYaw(travel_yaw + offset);
    Eigen::Vector3d dir(std::cos(yaw), std::sin(yaw), 0.0);
    if (!hybrid_constraints_enabled_ && corridor_frame_received_ &&
        !task_search::insideForwardHalfPlane(dir.head<2>(),
                                             corridor_dir_.head<2>()))
      continue;
    const Eigen::Vector3d probe =
        visited_positions_.empty() ? dir : visited_positions_.back() + 1.2 * dir;
    const double novelty = minDistance2D(probe, visited_positions_);
    // 2026-07-22: fallback首先保持真实运动方向，只有同一转角级别的左右候选才比较新颖度；
    // 旧权重会让“更陌生的±90度”压过可行直行，并在通道末端主动拐回刚走过的支路。
    scored.emplace_back(-4.0 * std::fabs(offset) +
                            0.10 * std::min(2.0, novelty),
                        dir);
  }
  std::sort(scored.begin(), scored.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.first > rhs.first; });

  std::vector<Eigen::Vector3d> directions;
  for (const auto& item : scored) directions.push_back(item.second);
  return directions;
}

bool TaskSearchManager::isRecoveryCandidateUseful(const Eigen::Vector3d& candidate) const {
  // 恢复点只受入口和最终出口边界约束；历史航迹距离不能否决窄通道中的侧移绕障。
  if (!hybrid_constraints_enabled_ && corridor_frame_received_ &&
      !visited_positions_.empty()) {
    const Eigen::Vector2d motion =
        candidate.head<2>() - visited_positions_.back().head<2>();
    if (!task_search::insideForwardHalfPlane(motion, stableProgressDirection()))
      return false;
  }
  return isTaskMotionAllowed(candidate);
}

bool TaskSearchManager::astarNoReturnDirection(Eigen::Vector3d& direction) const {
  if (!global_no_return_ || !corridor_frame_received_) return false;
  const Eigen::Vector2d progress = stableProgressDirection();
  if (progress.norm() < 1e-6) return false;
  direction = Eigen::Vector3d(progress.x(), progress.y(), 0.0);
  return true;
}

bool TaskSearchManager::isMissionBoundaryMotionAllowed(
    const Eigen::Vector3d& candidate) const {
  if (!global_no_return_ || !corridor_frame_received_) return true;
  // 普通搜索永远不能重新穿回初始入口外侧。最终出口的专用状态由上层
  // exit_transit_active 绕过普通任务路径过滤，不受这里影响。
  const double door_progress = (candidate - corridor_origin_).dot(corridor_dir_);
  // 2026-07-23: 最终出口只允许穿越一次。CROSS_EXIT确认完成后，出口外的所有区域
  // 统一视为通道外；候选若重新落到门内侧就直接拒绝，不再做任何复杂区域分类。
  const bool final_exit_guard_active =
      mission_stage_ >= SEARCH_OUTSIDE_LANDING && exit_candidate_confirmed_ &&
      exit_outward_direction_.norm() > 1e-3;
  double exit_side = 0.0;
  if (final_exit_guard_active) {
    exit_side =
        (candidate.head<2>() - exit_portal_center_.head<2>())
            .dot(exit_outward_direction_.normalized());
  }
  if (!task_search::passesMissionBoundaryNoReturn(
          door_progress, inside_return_margin_, final_exit_guard_active, exit_side))
    return false;
  return true;
}

bool TaskSearchManager::isTaskMotionAllowed(const Eigen::Vector3d& candidate) const {
  if (!isMissionBoundaryMotionAllowed(candidate)) return false;
  if (!global_no_return_ || !corridor_frame_received_) return true;
  if (!hybrid_constraints_enabled_) {
    return !latest_turn_anchor_valid_ ||
           task_search::passesLatestTurnNoReturn(
               candidate.head<2>(), latest_turn_anchor_,
               latest_turn_incoming_direction_, stableProgressDirection(),
               recovery_turn_no_return_margin_);
  }
  if (transition_active_ &&
      (candidate.head<2>() - transition_anchor_).norm() <= 0.80 + 1e-6)
    return true;
  for (const auto& gate : completed_gates_) {
    const Eigen::Vector2d normal = gate.normal.normalized();
    const Eigen::Vector2d lateral(-normal.y(), normal.x());
    const Eigen::Vector2d relative = candidate.head<2>() - gate.center;
    const double side = relative.dot(normal);
    const double across = relative.dot(lateral);
    if (std::fabs(side) <= 0.5 * gate.thickness + 1e-6 &&
        across <= gate.left_extent + 1e-6 && across >= -gate.right_extent - 1e-6)
      return false;
  }
  if (transition_active_) {
    const Eigen::Vector2d incoming = transition_incoming_direction_.normalized();
    const Eigen::Vector2d outgoing = transition_outgoing_direction_.normalized();
    const Eigen::Vector2d in_lateral(-incoming.y(), incoming.x());
    const Eigen::Vector2d out_lateral(-outgoing.y(), outgoing.x());
    const Eigen::Vector2d from_segment = candidate.head<2>() - segment_origin_;
    const Eigen::Vector2d from_anchor = candidate.head<2>() - transition_anchor_;
    const bool in_corridor = from_segment.dot(incoming) >=
                                 transition_old_high_water_ - 0.45 &&
                             std::fabs(from_segment.dot(in_lateral)) <= 1.25;
    const bool out_corridor = from_anchor.dot(outgoing) >= -0.45 &&
                              std::fabs(from_anchor.dot(out_lateral)) <= 1.25;
    return (from_anchor.norm() <= 0.80 + 1e-6) || in_corridor || out_corridor;
  }
  const Eigen::Vector2d direction = segment_direction_.norm() > 1e-3
                                        ? segment_direction_.normalized()
                                        : stableProgressDirection();
  const double progress = (candidate.head<2>() - segment_origin_).dot(direction);
  return progress >= segment_high_water_ - 0.25 - 1e-6;
}

bool TaskSearchManager::isTaskPathAllowed(
    const std::vector<Eigen::Vector3d>& path) const {
  if (!global_no_return_ || !corridor_frame_received_ || path.empty()) return true;

  // 完整路径逐点检查入口、最近已完成转角和最终出口边界，避免目标合法但中途越界。
  for (const auto& point : path) {
    if (!isTaskMotionAllowed(point)) return false;
  }

  if (!hybrid_constraints_enabled_) return true;

  // 解析检查每一段，保证端点合法但中途穿过历史门的路径仍被拒绝。
  // During the short turn transition the outgoing segment is allowed to leave
  // the anchor through the completed gate; the corridor envelope above is the
  // active restriction until the new segment is established.
  if (transition_active_) return true;
  for (size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector2d start = path[i - 1].head<2>();
    const Eigen::Vector2d end = path[i].head<2>();
    for (const auto& gate : completed_gates_) {
      const Eigen::Vector2d normal = gate.normal.normalized();
      const Eigen::Vector2d lateral(-normal.y(), normal.x());
      const double s0 = (start - gate.center).dot(normal);
      const double s1 = (end - gate.center).dot(normal);
      if (std::fabs(s0) <= 0.5 * gate.thickness ||
          std::fabs(s1) <= 0.5 * gate.thickness || s0 * s1 < 0.0) {
        const double denom = s0 - s1;
        const double ratio = std::fabs(denom) < 1e-9 ? 0.0 : s0 / denom;
        const Eigen::Vector2d crossing = start +
            std::max(0.0, std::min(1.0, ratio)) * (end - start);
        const double across = (crossing - gate.center).dot(lateral);
        if (across <= gate.left_extent + 1e-6 &&
            across >= -gate.right_extent - 1e-6)
          return false;
      }
    }
  }
  return true;
}

bool TaskSearchManager::isRecoveryPathAllowed(
    const std::vector<Eigen::Vector3d>& path, bool allow_initial_reverse) const {
  if (!hybrid_constraints_enabled_) {
    if (corridor_frame_received_ && path.size() >= 2) {
      const Eigen::Vector2d progress_direction = stableProgressDirection();
      const double start_progress =
          path.front().head<2>().dot(progress_direction);
      for (const auto& point : path) {
        if (point.head<2>().dot(progress_direction) < start_progress - 1e-3)
          return false;
      }
    }
    if (!allow_initial_reverse) return isTaskPathAllowed(path);
    if (!global_no_return_ || !corridor_frame_received_ || path.empty())
      return true;
    for (const auto& point : path) {
      if (!isTaskMotionAllowed(point)) return false;
    }
    return true;
  }
  if (!allow_initial_reverse) return isTaskPathAllowed(path);
  if (!global_no_return_ || !corridor_frame_received_ || path.empty()) return true;

  return isTaskPathAllowed(path);
}

double TaskSearchManager::clampSearchHeight(double z) const {
  return std::max(min_search_height_, std::min(max_search_height_, z));
}

double TaskSearchManager::preferredSearchHeight() const {
  return clampSearchHeight(cruise_height_);
}

double TaskSearchManager::projectSearchHeight(double candidate_z, double current_z) const {
  // 2026-07-13: frontier 的 z 只说明观测点来源，不应直接成为窄道飞行高度；按当前高度缓慢回到巡航层。
  (void)candidate_z;
  const double current = clampSearchHeight(current_z);
  const double target = clampSearchHeight(cruise_height_);
  const double delta = target - current;
  if (delta > 0.0) return current + std::min(delta, std::max(0.0, max_goal_climb_));
  return current + std::max(delta, -std::max(0.0, max_goal_descent_));
}

void TaskSearchManager::recordSelectedGoal(const Eigen::Vector3d& goal) {
  // 2026-07-13: 同一目标的重规划不再重复写满历史，否则几次安全重规划就会被误判为已搜索完成。
  if (selected_goals_.empty() ||
      (selected_goals_.back().head<2>() - goal.head<2>()).norm() > 0.15)
    selected_goals_.push_back(goal);
  while (static_cast<int>(selected_goals_.size()) > max_history_size_)
    selected_goals_.pop_front();
  active_goal_ = goal;
  active_goal_stamp_ = ros::Time::now();
  active_goal_valid_ = true;
}

void TaskSearchManager::reportGoalFailure(const Eigen::Vector3d& goal) {
  // 2026-07-13: A*/足迹检查失败的目标短时拉黑，规划器不能以 100Hz 重试同一点。
  failed_goals_.emplace_back(goal, ros::Time::now());
  while (failed_goals_.size() > 20) failed_goals_.pop_front();
  if (active_goal_valid_ &&
      (active_goal_.head<2>() - goal.head<2>()).norm() < failed_goal_radius_)
    active_goal_valid_ = false;
}

bool TaskSearchManager::goalTemporarilyBlocked(const Eigen::Vector3d& goal) const {
  const ros::Time now = ros::Time::now();
  for (const auto& failed : failed_goals_) {
    if ((now - failed.second).toSec() <= failed_goal_cooldown_ &&
        (failed.first.head<2>() - goal.head<2>()).norm() < failed_goal_radius_)
      return true;
  }
  return false;
}

void TaskSearchManager::colorDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  registerDetection(0, *msg);
}
void TaskSearchManager::qrcodeDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  registerDetection(1, *msg);
}
void TaskSearchManager::thermalDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  registerDetection(2, *msg);
}

void TaskSearchManager::finalLandingMarkerCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  // 通道内看到相似图案也不得触发降落；只有确认穿过出口后才登记平台 ArUco。
  if (mission_stage_ != SEARCH_OUTSIDE_LANDING &&
      mission_stage_ != APPROACH_LANDING) {
    ROS_WARN_THROTTLE(
        1.0, "[exit_mission] ignore landing ArUco before confirmed exit crossing.");
    return;
  }
  const ros::Time now = ros::Time::now();
  const Eigen::Vector2d point(msg->pose.position.x, msg->pose.position.y);
  const Eigen::Vector2d previous(final_landing_marker_.pose.pose.position.x,
                                 final_landing_marker_.pose.pose.position.y);
  const bool continuous = final_landing_marker_hits_ > 0 &&
                          (now - final_landing_marker_.stamp).toSec() <= 1.5 &&
                          (point - previous).norm() <=
                              final_landing_marker_consistency_radius_;
  final_landing_marker_hits_ = continuous ? final_landing_marker_hits_ + 1 : 1;
  final_landing_marker_.pose = *msg;
  final_landing_marker_.stamp = now;

  if (final_landing_marker_hits_ >=
      final_landing_marker_confirmation_count_) {
    final_landing_marker_.found = true;
    ROS_ERROR("[exit_mission] LANDING ARUCO confirmed %d/%d at "
              "world=(%.2f, %.2f, %.2f).",
              final_landing_marker_hits_,
              final_landing_marker_confirmation_count_, msg->pose.position.x,
              msg->pose.position.y, msg->pose.position.z);
  } else {
    ROS_WARN("[exit_mission] landing ArUco confirmation %d/%d at "
             "(%.2f, %.2f).",
             final_landing_marker_hits_,
             final_landing_marker_confirmation_count_, point.x(), point.y());
  }
  publishSearchState();
}

void TaskSearchManager::registerDetection(int type, const geometry_msgs::PoseStamped& msg) {
  if (type < 0 || type >= 3) return;
  targets_[type].found = true;
  targets_[type].pose = msg;
  targets_[type].stamp = ros::Time::now();

  const char* names[] = {"COLOR", "QRCODE", "THERMAL"};
  Eigen::Vector3d point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  Eigen::Vector3d corridor_point = point;
  if (corridor_frame_received_) {
    const Eigen::Vector3d lateral(-corridor_dir_.y(), corridor_dir_.x(), 0.0);
    const Eigen::Vector3d rel = point - corridor_origin_;
    corridor_point = Eigen::Vector3d(rel.dot(corridor_dir_), rel.dot(lateral), rel.z());
  }
  ROS_WARN("[task_search] target %s registered world=(%.2f, %.2f, %.2f) corridor=(%.2f, %.2f, %.2f).",
           names[type], point.x(), point.y(), point.z(), corridor_point.x(), corridor_point.y(),
           corridor_point.z());
  publishSearchState();
}

bool TaskSearchManager::allStage2TargetsFound() const {
  return targets_[0].found && targets_[1].found && targets_[2].found;
}

bool TaskSearchManager::mapPointSafe(const Eigen::Vector3d& point) const {
  if (!sdf_map_ || !sdf_map_->isInMap(point) ||
      sdf_map_->getOccupancy(point) != SDFMap::FREE ||
      sdf_map_->getInflateOccupancy(point) != 0)
    return false;

  // 2026-07-13: 与轨迹安全层一致，中心查 0.15m 膨胀、机体圆周查原始占据，禁止双重膨胀堵死通道。
  for (int sample = 0; sample < exit_footprint_samples_; ++sample) {
    const double angle = 2.0 * M_PI * static_cast<double>(sample) /
                         static_cast<double>(exit_footprint_samples_);
    Eigen::Vector3d probe = point;
    probe.x() += exit_footprint_extra_radius_ * std::cos(angle);
    probe.y() += exit_footprint_extra_radius_ * std::sin(angle);
    if (!sdf_map_->isInMap(probe) || sdf_map_->getOccupancy(probe) == SDFMap::OCCUPIED)
      return false;
  }
  return true;
}

bool TaskSearchManager::landingColumnSafe(const Eigen::Vector3d& marker,
                                          const Eigen::Vector3d& approach) const {
  if (approach.z() <= marker.z()) return false;
  const double start_z =
      marker.z() + std::max(0.05, landing_column_bottom_clearance_);
  const double step = std::max(0.05, landing_column_step_);
  for (double z = start_z; z <= approach.z() + 1e-6; z += step) {
    if (!mapPointSafe(Eigen::Vector3d(marker.x(), marker.y(), z))) return false;
  }
  return mapPointSafe(approach);
}

// 2026-07-23: 只在当前里程计高度上下各0.40m内投影占据，地图和里程计共同漂移时仍保持相对一致；
// 不再用固定世界高度找“门柱”，同时避开明显低于机体的地面和明显高于机体的顶棚。
bool TaskSearchManager::mapRelativeColumnOccupied(const Eigen::Vector3d& point,
                                                  double reference_z) const {
  if (!sdf_map_) return false;
  Eigen::Vector3d box_min, box_max;
  sdf_map_->getBox(box_min, box_max);
  const double min_z = std::max(box_min.z() + 0.05, reference_z - 0.40);
  const double max_z = std::min(box_max.z() - 0.05, reference_z + 0.40);
  if (max_z < min_z) return false;
  const double xy_offsets[][2] = {{0.0, 0.0}, {0.06, 0.0}, {-0.06, 0.0},
                                  {0.0, 0.06}, {0.0, -0.06}};
  int occupied_layers = 0;
  for (double z = min_z; z <= max_z + 1e-6; z += 0.10) {
    bool occupied = false;
    for (const auto& offset : xy_offsets) {
      Eigen::Vector3d probe(point.x() + offset[0], point.y() + offset[1], z);
      if (sdf_map_->isInMap(probe) &&
          sdf_map_->getOccupancy(probe) == SDFMap::OCCUPIED) {
        occupied = true;
        break;
      }
    }
    if (occupied) ++occupied_layers;
  }
  return occupied_layers >= 1;
}

// 2026-07-23: 对单通道任务，局部地形只需解释为“旧双墙继续”“旧双墙共同结束后进入更宽区域”
// 或“前方被墙/障碍封闭”。累计地图中旧双墙共同结束且门外中心已有FREE时，直接形成出口证据；
// body雷达的世界XY拼接和远距离命中只作辅助，不能否决RViz里已经清楚成形的断面。
bool TaskSearchManager::detectLocalMapExit(const Eigen::Vector3d& cur_pos,
                                           const Eigen::Vector2d& forward_hint,
                                           RadarExitResult& result) const {
  if (!sdf_map_ || forward_hint.norm() < 1e-3) return false;

  auto median = [](std::vector<double> values) {
    if (values.empty()) return std::numeric_limits<double>::infinity();
    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    return values[middle];
  };
  auto rotate = [](const Eigen::Vector2d& vector, double angle) {
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return Eigen::Vector2d(cosine * vector.x() - sine * vector.y(),
                           sine * vector.x() + cosine * vector.y());
  };

  bool found = false;
  double best_score = -std::numeric_limits<double>::infinity();
  int rejected_inside = 0;
  int rejected_not_ended = 0;
  int rejected_outside_not_free = 0;
  const double bin_step = std::max(0.10, radar_exit_wall_bin_step_);
  const double min_side = std::max(0.22, 0.65 * exit_portal_min_half_width_);
  const double max_side = std::max(min_side + 0.35, exit_portal_max_half_width_ + 0.65);
  const int inside_bins = std::max(
      radar_exit_min_inside_bins_,
      static_cast<int>(std::floor(radar_exit_inside_length_ / bin_step)));

  // 最近航迹经过弯角时切向可能偏斜，围绕它做小角度搜索；不允许90度旋转去把普通拐弯解释成出口。
  for (double angle_deg : {-25.0, -12.5, 0.0, 12.5, 25.0}) {
    const Eigen::Vector2d forward =
        rotate(forward_hint.normalized(), angle_deg * M_PI / 180.0).normalized();
    const Eigen::Vector2d lateral(-forward.y(), forward.x());
    auto nearestSides = [&](const Eigen::Vector2d& center, double& left, double& right) {
      left = std::numeric_limits<double>::infinity();
      right = std::numeric_limits<double>::infinity();
      for (double side_distance = min_side;
           side_distance <= max_side + 1e-6; side_distance += 0.05) {
        Eigen::Vector3d left_probe = cur_pos;
        left_probe.head<2>() = center - side_distance * lateral;
        if (!std::isfinite(left) &&
            mapRelativeColumnOccupied(left_probe, cur_pos.z()))
          left = side_distance;
        Eigen::Vector3d right_probe = cur_pos;
        right_probe.head<2>() = center + side_distance * lateral;
        if (!std::isfinite(right) &&
            mapRelativeColumnOccupied(right_probe, cur_pos.z()))
          right = side_distance;
        if (std::isfinite(left) && std::isfinite(right)) break;
      }
    };
    auto knownFree = [&](const Eigen::Vector2d& xy) {
      // 相邻三个相对高度层任一为FREE即可证明射线已经实际清过，UNKNOWN不冒充开放区。
      for (double z_offset : {-0.10, 0.0, 0.10}) {
        Eigen::Vector3d probe(xy.x(), xy.y(), cur_pos.z() + z_offset);
        if (sdf_map_->isInMap(probe) &&
            sdf_map_->getOccupancy(probe) == SDFMap::FREE)
          return true;
      }
      return false;
    };

    for (double section = radar_exit_search_min_forward_ - 0.10;
         section <= radar_exit_search_max_forward_ + 1e-6;
         section += std::max(0.08, radar_exit_search_step_)) {
      std::vector<double> inside_left;
      std::vector<double> inside_right;
      for (int bin = 1; bin <= inside_bins; ++bin) {
        const Eigen::Vector2d center =
            cur_pos.head<2>() + (section - bin * bin_step) * forward;
        double left, right;
        nearestSides(center, left, right);
        if (std::isfinite(left)) inside_left.push_back(left);
        if (std::isfinite(right)) inside_right.push_back(right);
      }
      if (static_cast<int>(inside_left.size()) < radar_exit_min_inside_bins_ ||
          static_cast<int>(inside_right.size()) < radar_exit_min_inside_bins_) {
        ++rejected_inside;
        continue;
      }
      const double wall_left = median(inside_left);
      const double wall_right = median(inside_right);
      int left_support = 0;
      int right_support = 0;
      for (double value : inside_left)
        if (std::fabs(value - wall_left) <= radar_exit_wall_track_tolerance_)
          ++left_support;
      for (double value : inside_right)
        if (std::fabs(value - wall_right) <= radar_exit_wall_track_tolerance_)
          ++right_support;
      if (left_support < radar_exit_min_inside_bins_ ||
          right_support < radar_exit_min_inside_bins_) {
        ++rejected_inside;
        continue;
      }

      const double center_offset = 0.5 * (wall_right - wall_left);
      int bilateral_changed = 0;
      const int outside_probes = std::max(1, radar_exit_outside_probe_count_);
      for (int probe_index = 1; probe_index <= outside_probes; ++probe_index) {
        const double depth = 1.5 * bin_step * probe_index;
        const Eigen::Vector2d center =
            cur_pos.head<2>() + (section + depth) * forward +
            center_offset * lateral;
        double left, right;
        nearestSides(center, left, right);
        const bool left_changed =
            !std::isfinite(left) || left >= wall_left + radar_exit_min_wall_shift_;
        const bool right_changed =
            !std::isfinite(right) || right >= wall_right + radar_exit_min_wall_shift_;
        if (left_changed && right_changed) ++bilateral_changed;
      }
      if (bilateral_changed < radar_exit_min_changed_probes_) {
        ++rejected_not_ended;
        continue;
      }

      // 门外不要求命中远墙，只要求中心走廊确实被累计射线清成FREE；普通转弯外侧墙和死胡同会降低该比例。
      int known_free = 0;
      int free_samples = 0;
      for (double depth = 0.12; depth <= radar_exit_clear_depth_ + 1e-6;
           depth += 0.12) {
        for (double lateral_offset : {-0.18, 0.0, 0.18}) {
          const Eigen::Vector2d probe =
              cur_pos.head<2>() + (section + depth) * forward +
              (center_offset + lateral_offset) * lateral;
          ++free_samples;
          if (knownFree(probe)) ++known_free;
        }
      }
      const double free_ratio = free_samples > 0
          ? static_cast<double>(known_free) / free_samples : 0.0;
      if (free_ratio < 0.60) {
        ++rejected_outside_not_free;
        continue;
      }

      const double score = 0.50 * std::min(left_support, right_support) +
                           1.20 * bilateral_changed + 2.0 * free_ratio -
                           0.02 * std::fabs(angle_deg) -
                           0.10 * std::max(0.0, section);
      if (!found || score > best_score) {
        found = true;
        best_score = score;
        result.portal_center = cur_pos;
        result.portal_center.head<2>() +=
            section * forward + center_offset * lateral;
        result.outward_direction = forward;
        result.inside_left_support = left_support;
        result.inside_right_support = right_support;
        result.bilateral_changed = bilateral_changed;
        result.clear_ray_frames = known_free;
        result.confidence = std::min(
            0.98, 0.48 + 0.04 * std::min(left_support, right_support) +
                      0.06 * bilateral_changed + 0.20 * free_ratio);
      }
    }
  }

  if (!found) {
    ROS_WARN_THROTTLE(
        2.0,
        "[exit_local_map] no local XY wall-end: reject_inside=%d "
        "wall_not_ended=%d outside_not_free=%d.",
        rejected_inside, rejected_not_ended, rejected_outside_not_free);
    return false;
  }
  ROS_INFO_THROTTLE(
      1.0,
      "[exit_local_map] portal center=(%.2f,%.2f) direction=%.1fdeg "
      "wall_support=L%d/R%d changed=%d/%d known_free_samples=%d confidence=%.2f.",
      result.portal_center.x(), result.portal_center.y(),
      std::atan2(result.outward_direction.y(), result.outward_direction.x()) *
          180.0 / M_PI,
      result.inside_left_support, result.inside_right_support,
      result.bilateral_changed, radar_exit_outside_probe_count_,
      result.clear_ray_frames, result.confidence);
  return true;
}

// 2026-07-22: 出口结构只接受工作高度带内跨多个z层的竖向支撑；地面、天花板和单个噪声体素
// 即使落入XY十字采样邻域，也不能再把整个门洞截面误判为“立柱占据”。
bool TaskSearchManager::mapColumnOccupied(const Eigen::Vector3d& point) const {
  if (!sdf_map_) return false;
  Eigen::Vector3d box_min, box_max;
  sdf_map_->getBox(box_min, box_max);
  const double min_z = std::max(box_min.z() + std::max(0.0, exit_column_lower_margin_),
                                exit_column_min_z_);
  const double max_z = std::min(box_max.z() - std::max(0.0, exit_column_upper_margin_),
                                exit_column_max_z_);
  if (max_z < min_z) return false;
  const double step = std::max(0.05, exit_column_sample_step_);
  // 对XY做一个小十字邻域，避免0.1m栅格采样恰好落在稀疏墙体体素之间。
  const double xy_offsets[][2] = {{0.0, 0.0}, {0.06, 0.0}, {-0.06, 0.0},
                                  {0.0, 0.06}, {0.0, -0.06}};
  int occupied_layers = 0;
  double first_occupied_z = std::numeric_limits<double>::infinity();
  double last_occupied_z = -std::numeric_limits<double>::infinity();
  for (double z = min_z; z <= max_z + 1e-6; z += step) {
    bool layer_occupied = false;
    for (const auto& offset : xy_offsets) {
      Eigen::Vector3d probe = point;
      probe.x() += offset[0];
      probe.y() += offset[1];
      probe.z() = z;
      if (sdf_map_->isInMap(probe) &&
          sdf_map_->getOccupancy(probe) == SDFMap::OCCUPIED) {
        layer_occupied = true;
        break;
      }
    }
    if (!layer_occupied) continue;
    ++occupied_layers;
    first_occupied_z = std::min(first_occupied_z, z);
    last_occupied_z = std::max(last_occupied_z, z);
  }
  return occupied_layers >= std::max(1, exit_column_min_occupied_layers_) &&
         last_occupied_z - first_occupied_z + 1e-6 >=
             std::max(0.0, exit_column_min_vertical_span_);
}

// 2026-07-22: 墙面必须在XY主方向上连续占据一段距离；单个箱体边缘、柱子或稀疏噪声即使有
// 竖向支撑，也不能凭一个占据柱冒充通道边界。该判定只使用累计地图，不依赖退化的单帧z值。
bool TaskSearchManager::mapWallRaySupported(const Eigen::Vector3d& start,
                                            const Eigen::Vector2d& direction,
                                            double length, double min_ratio) const {
  if (!sdf_map_ || direction.norm() < 1e-3 || length <= 0.0) return false;
  const Eigen::Vector2d unit = direction.normalized();
  const double step = std::max(0.08, exit_portal_side_sample_step_);
  int samples = 0;
  int occupied = 0;
  int consecutive = 0;
  int max_consecutive = 0;
  for (double distance = 0.0; distance <= length + 1e-6; distance += step) {
    Eigen::Vector3d probe = start;
    probe.head<2>() += distance * unit;
    ++samples;
    if (mapColumnOccupied(probe)) {
      ++occupied;
      max_consecutive = std::max(max_consecutive, ++consecutive);
    } else {
      consecutive = 0;
    }
  }
  if (samples <= 0) return false;
  const double ratio = static_cast<double>(occupied) / static_cast<double>(samples);
  // 至少三个连续采样点，避免两个相邻体素组成的紧凑障碍被解释成长墙。
  return ratio + 1e-6 >= std::max(0.0, std::min(1.0, min_ratio)) &&
         max_consecutive >= std::min(3, samples);
}

// 2026-07-21: 拓扑搜索只负责给出远端提示及末段方向；是否为出口由门框开口验证决定。
bool TaskSearchManager::inferExitFromMap(const Eigen::Vector3d& cur_pos,
                                         Eigen::Vector3d& candidate,
                                         Eigen::Vector3d& portal_center,
                                         Eigen::Vector2d& outward_direction,
                                         Eigen::Vector3d& endpoint_prediction,
                                         bool& endpoint_outside_verified,
                                         double& geodesic_distance,
                                         double& confidence) {
  // 2026-07-23: 每轮累计地图推理重新生成正常拐弯截面，随后与同周期局部雷达候选做XY关联。
  last_topology_turn_centers_.clear();
  if (!sdf_map_ || !corridor_frame_received_) return false;

  Eigen::Vector3d box_min, box_max;
  sdf_map_->getBox(box_min, box_max);
  const double resolution = std::max(0.10, exit_grid_resolution_);
  // 2026-07-21: 拓扑可达层跟随当前地图/里程计高度，避免z漂移后继续死守绝对0.65m切片。
  const double analysis_z = clampSearchHeight(cur_pos.z());
  const int width = std::max(1, static_cast<int>(std::ceil((box_max.x() - box_min.x()) / resolution)));
  const int height = std::max(1, static_cast<int>(std::ceil((box_max.y() - box_min.y()) / resolution)));
  if (width * height > 200000) {
    ROS_ERROR_THROTTLE(2.0, "[exit_mission] topology grid too large: %dx%d.", width, height);
    return false;
  }

  auto index = [width](int x, int y) { return y * width + x; };
  auto gridPoint = [&](int x, int y) {
    return Eigen::Vector3d(box_min.x() + (x + 0.5) * resolution,
                           box_min.y() + (y + 0.5) * resolution, analysis_z);
  };

  std::vector<unsigned char> free_cell(width * height, 0);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const Eigen::Vector3d point = gridPoint(x, y);
      const double door_progress = (point - corridor_origin_).dot(corridor_dir_);
      if (door_progress >= inside_return_margin_ && mapPointSafe(point))
        free_cell[index(x, y)] = 1;
    }
  }

  // 2026-07-13: 入口内侧作为拓扑根节点，找不到精确栅格时选择其附近最近自由单元。
  const Eigen::Vector3d root_hint = corridor_origin_ + 0.65 * corridor_dir_;
  int root = -1;
  double root_distance = std::numeric_limits<double>::infinity();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (!free_cell[index(x, y)]) continue;
      const double distance = (gridPoint(x, y).head<2>() - root_hint.head<2>()).norm();
      if (distance < root_distance) {
        root_distance = distance;
        root = index(x, y);
      }
    }
  }
  if (root < 0 || root_distance > 1.2) {
    ROS_WARN_THROTTLE(2.0,
                      "[exit_mission] TOPOLOGY_EARLY_RETURN root_missing nearest=%.2fm "
                      "analysis_z=%.2fm.",
                      root_distance, analysis_z);
    return false;
  }

  std::vector<double> distance(width * height, std::numeric_limits<double>::infinity());
  std::vector<int> parent(width * height, -1);
  using QueueItem = std::pair<double, int>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
  distance[root] = 0.0;
  queue.emplace(0.0, root);
  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!queue.empty()) {
    const double current_distance = queue.top().first;
    const int current = queue.top().second;
    queue.pop();
    if (current_distance > distance[current] + 1e-6) continue;
    const int cx = current % width;
    const int cy = current / width;
    for (int n = 0; n < 8; ++n) {
      const int nx = cx + dx[n];
      const int ny = cy + dy[n];
      if (nx < 0 || nx >= width || ny < 0 || ny >= height || !free_cell[index(nx, ny)])
        continue;
      // 2026-07-13: 对角移动不能穿过两个占据角点，保证拓扑路径真实可飞。
      if (n >= 4 && (!free_cell[index(nx, cy)] || !free_cell[index(cx, ny)])) continue;
      const double step = resolution * (n < 4 ? 1.0 : std::sqrt(2.0));
      const int next = index(nx, ny);
      if (current_distance + step + 1e-6 < distance[next]) {
        distance[next] = current_distance + step;
        parent[next] = current;
        queue.emplace(distance[next], next);
      }
    }
  }

  double farthest_distance = 0.0;
  for (double item : distance)
    if (std::isfinite(item)) farthest_distance = std::max(farthest_distance, item);
  if (farthest_distance < exit_min_geodesic_distance_) {
    ROS_WARN_THROTTLE(2.0,
                      "[exit_mission] TOPOLOGY_EARLY_RETURN reachable_free_short=%.2fm < %.2fm "
                      "analysis_z=%.2fm.",
                      farthest_distance, exit_min_geodesic_distance_, analysis_z);
    return false;
  }

  // 2026-07-13: 只在拓扑最远的 15% 区域中寻找开口/宽阔终端，避免选到入口附近欧氏距离较远的墙角。
  const double candidate_distance_min =
      std::max(exit_min_geodesic_distance_, 0.85 * farthest_distance);
  int best = -1;
  double best_score = -std::numeric_limits<double>::infinity();
  int best_unknown_support = 0;
  int best_local_free = 0;
  Eigen::Vector2d best_direction = corridor_dir_.head<2>();

  for (int cell = 0; cell < width * height; ++cell) {
    if (!std::isfinite(distance[cell]) || distance[cell] < candidate_distance_min ||
        parent[cell] < 0)
      continue;
    const int x = cell % width;
    const int y = cell / width;
    const int px = parent[cell] % width;
    const int py = parent[cell] / width;
    Eigen::Vector2d travel_direction(x - px, y - py);
    if (travel_direction.norm() < 1e-6) continue;
    travel_direction.normalize();
    const Eigen::Vector2d lateral(-travel_direction.y(), travel_direction.x());
    const Eigen::Vector3d world = gridPoint(x, y);

    int unknown_support = 0;
    for (double angle_offset : {-M_PI_4, -M_PI_4 * 0.5, 0.0, M_PI_4 * 0.5, M_PI_4}) {
      const double c = std::cos(angle_offset);
      const double s = std::sin(angle_offset);
      const Eigen::Vector2d ray(c * travel_direction.x() - s * travel_direction.y(),
                                s * travel_direction.x() + c * travel_direction.y());
      for (double depth = 0.35; depth <= 1.05; depth += 0.20) {
        Eigen::Vector3d probe = world;
        probe.head<2>() += depth * ray;
        if (!sdf_map_->isInMap(probe) || sdf_map_->getOccupancy(probe) == SDFMap::UNKNOWN)
          ++unknown_support;
      }
    }

    int local_free = 0;
    const int local_radius = std::max(1, static_cast<int>(std::round(0.60 / resolution)));
    for (int oy = -local_radius; oy <= local_radius; ++oy) {
      for (int ox = -local_radius; ox <= local_radius; ++ox) {
        const int nx = x + ox;
        const int ny = y + oy;
        if (nx >= 0 && nx < width && ny >= 0 && ny < height && free_cell[index(nx, ny)])
          ++local_free;
      }
    }

    int side_wall_support = 0;
    for (double side : {-1.0, 1.0}) {
      for (double offset = 0.35; offset <= 0.85; offset += 0.20) {
        Eigen::Vector3d probe = world;
        probe.head<2>() += side * offset * lateral;
        if (sdf_map_->isInMap(probe) && sdf_map_->getOccupancy(probe) == SDFMap::OCCUPIED)
          ++side_wall_support;
      }
    }

    const double score = distance[cell] + 0.08 * unknown_support +
                         0.01 * local_free + 0.04 * side_wall_support -
                         0.02 * (world.head<2>() - cur_pos.head<2>()).norm();
    if (score > best_score) {
      best_score = score;
      best = cell;
      best_unknown_support = unknown_support;
      best_local_free = local_free;
      best_direction = travel_direction;
    }
  }
  if (best < 0) {
    ROS_WARN_THROTTLE(2.0,
                      "[exit_mission] TOPOLOGY_EARLY_RETURN no_far_endpoint farthest=%.2fm.",
                      farthest_distance);
    return false;
  }
  // 2026-07-22: 最远可达自由点只是终点预测锚点，绝不直接作为出口或降落目标。
  endpoint_prediction = gridPoint(best % width, best / width);
  endpoint_outside_verified = false;

  // 2026-07-21: 最远自由点通常已经位于门外房间，不能直接充当门框中心。
  // 沿入口到最远点的拓扑父链，寻找“门内仍有双侧墙、门口双边夹自由开口、门外明显变宽”的转变截面。
  std::vector<int> route;
  for (int cell = best; cell >= 0; cell = parent[cell]) {
    route.push_back(cell);
    if (cell == root) break;
  }
  if (route.empty() || route.back() != root) {
    ROS_WARN_THROTTLE(2.0, "[exit_mission] TOPOLOGY_EARLY_RETURN parent_chain_broken.");
    return false;
  }
  std::reverse(route.begin(), route.end());

  auto firstBoundaryDistance = [&](const Eigen::Vector3d& center,
                                   const Eigen::Vector2d& lateral,
                                   double side, double max_distance) {
    for (double offset = exit_portal_min_half_width_;
         offset <= max_distance + 1e-6;
         offset += std::max(0.05, exit_portal_side_sample_step_)) {
      Eigen::Vector3d probe = center;
      probe.head<2>() += side * offset * lateral;
      if (mapColumnOccupied(probe)) return offset;
    }
    return std::numeric_limits<double>::infinity();
  };

  int transition_cell = -1;
  Eigen::Vector2d transition_direction = best_direction;
  // 2026-07-22: 入口取最先建立双墙的截面，出口相反，应取最靠拓扑远端的双墙共同终止截面。
  double transition_distance = -std::numeric_limits<double>::infinity();
  double transition_structure_score = -std::numeric_limits<double>::infinity();
  int rejected_normal_turn = 0;
  int rejected_nonwall_boundary = 0;
  int rejected_no_portal_transition = 0;
  int rejected_gap_blocked = 0;
  int rejected_missing_boundaries = 0;
  int rejected_outside_axis = 0;
  int rejected_outside_route = 0;
  int rejected_passed_history = 0;
  int rejected_outside_unknown = 0;
  // 2026-07-22: 起搜下限只表示已经离开入口任务区，不能随外部地图继续变远；否则真实门框会在
  // 外部房间扩图后落到“最远路径60%”之前并凭空消失，随后又把房间边缘误认成新出口。
  const double route_min_distance = exit_min_geodesic_distance_;
  const int turn_window_cells = std::max(
      2, static_cast<int>(std::ceil(exit_turn_window_distance_ / resolution)));
  for (size_t i = 2; i + 2 < route.size(); ++i) {
    const int cell = route[i];
    if (distance[cell] < route_min_distance) continue;
    const Eigen::Vector3d center = gridPoint(cell % width, cell / width);
    // 门截面方向只能由门内来向决定。父链进入较大盒子后会立即斜向远角；若混入门外
    // route[i+2]，真实直通门会被伪装成普通拐弯，左右墙采样也会整体转歪。
    const size_t incoming_index =
        i >= static_cast<size_t>(turn_window_cells) ? i - turn_window_cells : i - 2;
    const Eigen::Vector3d incoming_start =
        gridPoint(route[incoming_index] % width, route[incoming_index] / width);
    Eigen::Vector2d direction = center.head<2>() - incoming_start.head<2>();
    if (direction.norm() < 0.20) continue;
    direction.normalize();
    const Eigen::Vector2d lateral(-direction.y(), direction.x());

    // 正常弯角只看候选之前的中心线是否已经转弯；门外盒子里的分叉/斜向父链不参与。
    if (i >= static_cast<size_t>(2 * turn_window_cells)) {
      const Eigen::Vector3d turn_before = gridPoint(
          route[i - 2 * turn_window_cells] % width,
          route[i - 2 * turn_window_cells] / width);
      const Eigen::Vector3d turn_middle = gridPoint(
          route[i - turn_window_cells] % width,
          route[i - turn_window_cells] / width);
      Eigen::Vector2d incoming = turn_middle.head<2>() - turn_before.head<2>();
      Eigen::Vector2d outgoing = center.head<2>() - turn_middle.head<2>();
      if (incoming.norm() > 0.20 && outgoing.norm() > 0.20) {
        const double cosine = std::max(-1.0, std::min(
            1.0, incoming.normalized().dot(outgoing.normalized())));
        const double turn_deg = std::acos(cosine) * 180.0 / M_PI;
        if (turn_deg > exit_max_centerline_turn_deg_) {
          // 2026-07-23: 不只统计normal_turn数量，还保留其位置，堵住“拓扑拒绝后雷达又放行”的旁路。
          last_topology_turn_centers_.push_back(center);
          ++rejected_normal_turn;
          continue;
        }
      }
    }

    bool gap_clear = true;
    // 2026-07-22: 推断与最终验证使用同一个门洞核心半宽，避免一个阶段放行、下阶段在边缘拒绝。
    const double core_half_width =
        std::max(0.10, exit_portal_min_half_width_ - exit_portal_core_margin_);
    for (double offset = -core_half_width;
         offset <= core_half_width + 1e-6; offset += 0.10) {
      Eigen::Vector3d probe = center;
      probe.head<2>() += offset * lateral;
      if (mapColumnOccupied(probe)) {
        gap_clear = false;
        break;
      }
    }
    if (!gap_clear) {
      ++rejected_gap_blocked;
      continue;
    }

    // 2026-07-22: 单通道里飞机已经越过的截面不可能再成为出口。该硬约束比候选稳定计数优先，
    // 防止历史点云缺口在数秒后累计到5/5并把门内接近点放到飞机身后。
    const double passed_distance = (cur_pos.head<2>() - center.head<2>()).dot(direction);
    if (passed_distance > exit_candidate_max_behind_distance_) {
      ++rejected_passed_history;
      continue;
    }

    const double side_search = exit_portal_max_half_width_ + 0.60;
    // 2026-07-22: 门内连续取三个截面跟踪左右墙距。只有距离平滑且向后连续的双墙才叫通道墙；
    // 单个柱子/箱体只会命中一两个截面，不能再提供出口门的“墙体”证据。
    const int inside_probe_count = 3;
    double inside_left = 0.0;
    double inside_right = 0.0;
    bool inside_tracks_stable = true;
    for (int probe_index = 1; probe_index <= inside_probe_count; ++probe_index) {
      const double depth = std::max(0.20, exit_wall_end_probe_step_) * probe_index;
      Eigen::Vector3d inside_probe = center;
      inside_probe.head<2>() -= depth * direction;
      const double left = firstBoundaryDistance(inside_probe, lateral, -1.0, side_search);
      const double right = firstBoundaryDistance(inside_probe, lateral, 1.0, side_search);
      if (!std::isfinite(left) || !std::isfinite(right)) {
        inside_tracks_stable = false;
        break;
      }
      if (probe_index == 1) {
        inside_left = left;
        inside_right = right;
      } else if (std::fabs(left - inside_left) > exit_wall_track_width_tolerance_ ||
                 std::fabs(right - inside_right) > exit_wall_track_width_tolerance_) {
        inside_tracks_stable = false;
        break;
      }
    }
    if (!inside_tracks_stable) {
      ++rejected_missing_boundaries;
      continue;
    }

    Eigen::Vector3d inside = center;
    inside.head<2>() -= std::max(0.20, exit_wall_end_probe_step_) * direction;
    Eigen::Vector3d inside_left_wall = inside;
    inside_left_wall.head<2>() -= inside_left * lateral;
    Eigen::Vector3d inside_right_wall = inside;
    inside_right_wall.head<2>() += inside_right * lateral;
    const bool left_corridor_wall = mapWallRaySupported(
        inside_left_wall, -direction, exit_wall_support_length_,
        exit_wall_support_min_ratio_);
    const bool right_corridor_wall = mapWallRaySupported(
        inside_right_wall, -direction, exit_wall_support_length_,
        exit_wall_support_min_ratio_);
    if (!left_corridor_wall || !right_corridor_wall) {
      ++rejected_nonwall_boundary;
      continue;
    }

    // 2026-07-22: 门框横向占据只作为结构加分。比赛里的“门”是通道入口/出口截面，
    // 不要求地图里一定存在标准门柱形状，因此不能把门框存在设成硬门槛。
    const double portal_left =
        firstBoundaryDistance(center, lateral, -1.0, exit_portal_max_half_width_);
    const double portal_right =
        firstBoundaryDistance(center, lateral, 1.0, exit_portal_max_half_width_);
    Eigen::Vector3d left_post = center;
    left_post.head<2>() -= (std::isfinite(portal_left) ? portal_left : inside_left) * lateral;
    Eigen::Vector3d right_post = center;
    right_post.head<2>() += (std::isfinite(portal_right) ? portal_right : inside_right) * lateral;
    const bool transverse_frame =
        std::isfinite(portal_left) && std::isfinite(portal_right) &&
        mapWallRaySupported(left_post, -lateral,
                            exit_frame_outward_support_length_,
                            exit_wall_support_min_ratio_) &&
        mapWallRaySupported(right_post, lateral,
                            exit_frame_outward_support_length_,
                            exit_wall_support_min_ratio_);
    // 2026-07-28: 门必须具有双侧同截面的横向墙端/门框证据；弧形墙外侧的自由区不能再冒充门洞。
    if (!transverse_frame) {
      ++rejected_nonwall_boundary;
      continue;
    }

    // 2026-07-22: 出口定义为旧左右墙在多个门外截面共同终止/外移，并且这些截面是雷达
    // 实际清出的FREE；UNKNOWN不是开阔区证据。连续采样可抑制点云空洞，双侧共同变化可抑制障碍物。
    const double min_shift = std::max(0.0, exit_min_bilateral_wall_shift_);
    int bilateral_changed_count = 0;
    int observed_free_samples = 0;
    int total_free_samples = 0;
    double accumulated_left_shift = 0.0;
    double accumulated_right_shift = 0.0;
    const int outside_probe_count = std::max(1, exit_wall_end_probe_count_);
    for (int probe_index = 1; probe_index <= outside_probe_count; ++probe_index) {
      const double depth = std::max(0.15, exit_wall_end_probe_step_) * probe_index;
      Eigen::Vector3d outside = center;
      outside.head<2>() += depth * direction;
      const double outside_left = firstBoundaryDistance(outside, lateral, -1.0, side_search);
      const double outside_right = firstBoundaryDistance(outside, lateral, 1.0, side_search);
      const bool left_changed = !std::isfinite(outside_left) ||
          outside_left >= inside_left + min_shift;
      const bool right_changed = !std::isfinite(outside_right) ||
          outside_right >= inside_right + min_shift;
      if (left_changed && right_changed) ++bilateral_changed_count;
      accumulated_left_shift += !std::isfinite(outside_left)
          ? side_search - inside_left : std::max(0.0, outside_left - inside_left);
      accumulated_right_shift += !std::isfinite(outside_right)
          ? side_search - inside_right : std::max(0.0, outside_right - inside_right);
      for (double lateral_scale : {-0.45, 0.0, 0.45}) {
        Eigen::Vector3d free_probe = outside;
        free_probe.head<2>() += lateral_scale * core_half_width * lateral;
        ++total_free_samples;
        if (sdf_map_->isInMap(free_probe) &&
            sdf_map_->getOccupancy(free_probe) == SDFMap::FREE)
          ++observed_free_samples;
      }
    }
    const int min_changed = std::min(outside_probe_count,
                                     std::max(1, exit_wall_end_min_changed_count_));
    if (bilateral_changed_count < min_changed) {
      ++rejected_no_portal_transition;
      continue;
    }
    const double outside_free_ratio = total_free_samples > 0
        ? static_cast<double>(observed_free_samples) / total_free_samples : 0.0;
    if (outside_free_ratio + 1e-6 < exit_outside_min_free_ratio_) {
      ++rejected_outside_unknown;
      continue;
    }

    // 2026-07-22: 门后必须确实存在由累计FREE栅格组成的拓扑分支；只有UNKNOWN射线不算通道外。
    const double outside_route_length = distance[best] - distance[cell];
    if (outside_route_length + 1e-6 < exit_endpoint_outside_route_min_length_) {
      ++rejected_outside_route;
      continue;
    }

    const double left_shift = accumulated_left_shift / outside_probe_count;
    const double right_shift = accumulated_right_shift / outside_probe_count;
    const double structure_score = (transverse_frame ? 2.0 : 0.0) +
                                   0.5 * bilateral_changed_count + outside_free_ratio +
                                   std::min(1.0, std::max(0.0, left_shift)) +
                                   std::min(1.0, std::max(0.0, right_shift)) -
                                   0.25 * std::fabs(inside_left - inside_right);
    // 2026-07-22: 出口与入口方向相反：入口取双墙开始处，出口取最靠远端的双墙结束处。
    const bool more_endpoint_side = distance[cell] > transition_distance + 0.5 * resolution;
    const bool same_cross_section =
        std::fabs(distance[cell] - transition_distance) <= 0.5 * resolution;
    if (more_endpoint_side ||
        (same_cross_section && structure_score > transition_structure_score)) {
      transition_distance = distance[cell];
      transition_structure_score = structure_score;
      transition_cell = cell;
      transition_direction = direction;
    }
  }
  if (transition_cell < 0) {
    ROS_WARN_THROTTLE(2.0,
                      "[exit_mission] far topology branch exists but no structural portal "
                      "is verified: normal_turn=%d nonwall_boundary=%d no_wall_transition=%d "
                      "gap=%d missing_boundary=%d outside_axis=%d outside_route=%d "
                      "passed_history=%d outside_unknown=%d.",
                      rejected_normal_turn, rejected_nonwall_boundary,
                      rejected_no_portal_transition, rejected_gap_blocked,
                      rejected_missing_boundaries, rejected_outside_axis,
                      rejected_outside_route, rejected_passed_history,
                      rejected_outside_unknown);
    // 2026-07-23: 世界z退化时累计地图的竖直柱检测会漏掉真门。只要入口根到远端的FREE拓扑
    // 仍成立，就把远端保留为候选锚点，真正门截面必须交给短时机体系雷达确认，绝不在这里确认。
    if (!radar_exit_detection_enabled_) return false;
    portal_center = cur_pos;
    outward_direction =
        best_direction.norm() > 1e-3 ? best_direction.normalized()
                                     : corridor_dir_.head<2>().normalized();
    candidate = cur_pos;
    endpoint_outside_verified = false;
    geodesic_distance = farthest_distance;
    confidence = 0.20;
    return true;
  }

  portal_center = gridPoint(transition_cell % width, transition_cell / width);
  outward_direction = transition_direction;
  endpoint_outside_verified =
      distance[best] - distance[transition_cell] + 1e-6 >=
      exit_endpoint_outside_route_min_length_;
  candidate = portal_center;
  candidate.head<2>() -= exit_standoff_ * transition_direction;
  if (!mapPointSafe(candidate)) return false;
  geodesic_distance = distance[transition_cell];
  confidence = std::min(0.99, 0.45 + 0.015 * best_unknown_support +
                                  0.002 * best_local_free +
                                  0.10 * geodesic_distance / farthest_distance);
  return mapPointSafe(candidate);
}

// 2026-07-21: 出口是“左右占据边界夹着自由开口并向外延伸”，而不是一个稳定最远自由栅格。
bool TaskSearchManager::validateExitPortal(
    const Eigen::Vector3d& inside_hint, const Eigen::Vector3d& portal_center,
    const Eigen::Vector2d& outward_direction) const {
  if (!sdf_map_ || outward_direction.norm() < 1e-3 || !mapPointSafe(inside_hint) ||
      !mapPointSafe(portal_center)) {
    ROS_WARN_THROTTLE(1.0, "[exit_mission] EXIT_REJECT unsafe_inside_or_center.");
    return false;
  }

  const Eigen::Vector2d direction = outward_direction.normalized();
  const Eigen::Vector2d lateral(-direction.y(), direction.x());

  // 门内接近点到门框中心必须整段保持可飞，避免把墙后拓扑点误当门洞。
  const double approach_length =
      (portal_center.head<2>() - inside_hint.head<2>()).norm();
  for (double distance = 0.0; distance <= approach_length + 1e-3; distance += 0.10) {
    Eigen::Vector3d probe = inside_hint;
    probe.head<2>() += std::min(distance, approach_length) * direction;
    if (!mapPointSafe(probe)) {
      ROS_WARN_THROTTLE(1.0, "[exit_mission] EXIT_REJECT inside_approach_blocked.");
      return false;
    }
  }

  // 2026-07-22: 门洞核心净空与推断阶段一致，并给门框起搜边界保留采样邻域余量。
  const double core_half_width =
      std::max(0.10, exit_portal_min_half_width_ - exit_portal_core_margin_);
  for (double offset = 0.10; offset <= core_half_width + 1e-6; offset += 0.10) {
    for (double side : {-1.0, 1.0}) {
      Eigen::Vector3d clearance_probe = portal_center;
      clearance_probe.head<2>() += side * offset * lateral;
      if (mapColumnOccupied(clearance_probe)) {
        ROS_WARN_THROTTLE(1.0, "[exit_mission] EXIT_REJECT gap_center_column_occupied.");
        return false;
      }
    }
  }

  bool left_frame = false;
  bool right_frame = false;
  double left_frame_offset = std::numeric_limits<double>::infinity();
  double right_frame_offset = std::numeric_limits<double>::infinity();
  for (double offset = exit_portal_min_half_width_;
       offset <= exit_portal_max_half_width_ + 1e-6;
       offset += std::max(0.05, exit_portal_side_sample_step_)) {
    for (double side : {-1.0, 1.0}) {
      Eigen::Vector3d probe = portal_center;
      probe.head<2>() += side * offset * lateral;
      const bool occupied = mapColumnOccupied(probe);
      if (occupied) {
        if (side < 0.0) {
          left_frame = true;
          left_frame_offset = std::min(left_frame_offset, offset);
        } else {
          right_frame = true;
          right_frame_offset = std::min(right_frame_offset, offset);
        }
      }
    }
  }
  // 2026-07-22: 最终复核以通道墙的生命周期为准。门框点云允许缺失，因为任务中的“门”是
  // 通道出口截面；左右门柱只作辅助显示/加分，不能替代门内连续双墙和门外共同终止证据。
  auto firstBoundaryDistance = [&](const Eigen::Vector3d& center,
                                   double side, double max_distance) {
    for (double offset = exit_portal_min_half_width_;
         offset <= max_distance + 1e-6;
         offset += std::max(0.05, exit_portal_side_sample_step_)) {
      Eigen::Vector3d probe = center;
      probe.head<2>() += side * offset * lateral;
      if (mapColumnOccupied(probe)) return offset;
    }
    return std::numeric_limits<double>::infinity();
  };
  const double side_search = exit_portal_max_half_width_ + 0.60;
  const int inside_probe_count = 3;
  double inside_left = 0.0;
  double inside_right = 0.0;
  bool inside_tracks_stable = true;
  for (int probe_index = 1; probe_index <= inside_probe_count; ++probe_index) {
    const double depth = std::max(0.20, exit_wall_end_probe_step_) * probe_index;
    Eigen::Vector3d inside_probe = portal_center;
    inside_probe.head<2>() -= depth * direction;
    const double left = firstBoundaryDistance(inside_probe, -1.0, side_search);
    const double right = firstBoundaryDistance(inside_probe, 1.0, side_search);
    if (!std::isfinite(left) || !std::isfinite(right)) {
      inside_tracks_stable = false;
      break;
    }
    if (probe_index == 1) {
      inside_left = left;
      inside_right = right;
    } else if (std::fabs(left - inside_left) > exit_wall_track_width_tolerance_ ||
               std::fabs(right - inside_right) > exit_wall_track_width_tolerance_) {
      inside_tracks_stable = false;
      break;
    }
  }
  if (!inside_tracks_stable) {
    ROS_WARN_THROTTLE(1.0, "[exit_mission] EXIT_REJECT unstable_corridor_wall_tracks.");
    return false;
  }
  Eigen::Vector3d inside_section = portal_center;
  inside_section.head<2>() -= std::max(0.20, exit_wall_end_probe_step_) * direction;
  Eigen::Vector3d inside_left_wall = inside_section;
  inside_left_wall.head<2>() -= inside_left * lateral;
  Eigen::Vector3d inside_right_wall = inside_section;
  inside_right_wall.head<2>() += inside_right * lateral;
  if (!mapWallRaySupported(inside_left_wall, -direction,
                           exit_wall_support_length_, exit_wall_support_min_ratio_) ||
      !mapWallRaySupported(inside_right_wall, -direction,
                           exit_wall_support_length_, exit_wall_support_min_ratio_)) {
    ROS_WARN_THROTTLE(1.0, "[exit_mission] EXIT_REJECT side_boundary_is_obstacle_not_wall.");
    return false;
  }

  Eigen::Vector3d left_post = portal_center;
  left_post.head<2>() -= (left_frame ? left_frame_offset : inside_left) * lateral;
  Eigen::Vector3d right_post = portal_center;
  right_post.head<2>() += (right_frame ? right_frame_offset : inside_right) * lateral;
  const bool transverse_frame =
      left_frame && right_frame &&
      mapWallRaySupported(left_post, -lateral,
                          exit_frame_outward_support_length_,
                          exit_wall_support_min_ratio_) &&
      mapWallRaySupported(right_post, lateral,
                          exit_frame_outward_support_length_,
                          exit_wall_support_min_ratio_);
  // 2026-07-28: 最终复核同样把双侧横向门框设为硬条件，堵住局部自由空洞绕过拓扑筛选的旁路。
  if (!transverse_frame) {
    ROS_WARN_THROTTLE(1.0, "[exit_mission] EXIT_REJECT missing_straight_two_sided_frame.");
    return false;
  }
  const double min_shift = std::max(0.0, exit_min_bilateral_wall_shift_);
  int bilateral_changed_count = 0;
  int observed_free_samples = 0;
  int total_free_samples = 0;
  const int outside_probe_count = std::max(1, exit_wall_end_probe_count_);
  for (int probe_index = 1; probe_index <= outside_probe_count; ++probe_index) {
    const double depth = std::max(0.15, exit_wall_end_probe_step_) * probe_index;
    Eigen::Vector3d outside_section = portal_center;
    outside_section.head<2>() += depth * direction;
    const double outside_left = firstBoundaryDistance(outside_section, -1.0, side_search);
    const double outside_right = firstBoundaryDistance(outside_section, 1.0, side_search);
    const bool left_changed = !std::isfinite(outside_left) ||
        outside_left >= inside_left + min_shift;
    const bool right_changed = !std::isfinite(outside_right) ||
        outside_right >= inside_right + min_shift;
    if (left_changed && right_changed) ++bilateral_changed_count;
    for (double lateral_scale : {-0.45, 0.0, 0.45}) {
      Eigen::Vector3d free_probe = outside_section;
      free_probe.head<2>() += lateral_scale * core_half_width * lateral;
      ++total_free_samples;
      if (sdf_map_->isInMap(free_probe) &&
          sdf_map_->getOccupancy(free_probe) == SDFMap::FREE)
        ++observed_free_samples;
    }
  }
  const int min_changed = std::min(outside_probe_count,
                                   std::max(1, exit_wall_end_min_changed_count_));
  if (bilateral_changed_count < min_changed) {
    ROS_WARN_THROTTLE(1.0,
                      "[exit_mission] EXIT_REJECT walls_do_not_end_together changed=%d/%d "
                      "frame=%d.", bilateral_changed_count, outside_probe_count,
                      static_cast<int>(transverse_frame));
    return false;
  }
  const double free_ratio = total_free_samples > 0
      ? static_cast<double>(observed_free_samples) / total_free_samples : 0.0;
  if (free_ratio + 1e-6 < exit_outside_min_free_ratio_) {
    ROS_WARN_THROTTLE(1.0,
                      "[exit_mission] EXIT_REJECT outside_not_observed_free ratio=%.2f < %.2f.",
                      free_ratio, exit_outside_min_free_ratio_);
    return false;
  }
  return true;
}

// 出口结构已经确认后，沿锁定法向做短步穿越。已知FREE优先；若门外仍是UNKNOWN，
// 只要目标足迹和整段轴线都没有原始OCCUPIED，也允许一步受控探测，打破“必须先看到FREE
// 才肯出门、但不出门又看不到FREE”的死锁。该放宽仅存在于 CROSS_EXIT。
bool TaskSearchManager::buildSafeExitCrossingGoal(const Eigen::Vector3d& cur_pos,
                                                  Eigen::Vector3d& goal) const {
  if (!exit_candidate_confirmed_ || exit_outward_direction_.norm() < 1e-3) return false;
  const Eigen::Vector2d direction = exit_outward_direction_.normalized();
  const auto raw_footprint_clear = [this](const Eigen::Vector3d& point) {
    if (!sdf_map_ || !sdf_map_->isInMap(point) ||
        sdf_map_->getOccupancy(point) == SDFMap::OCCUPIED)
      return false;
    for (int sample = 0; sample < exit_footprint_samples_; ++sample) {
      const double angle = 2.0 * M_PI * static_cast<double>(sample) /
                           static_cast<double>(exit_footprint_samples_);
      Eigen::Vector3d probe = point;
      probe.x() += exit_footprint_extra_radius_ * std::cos(angle);
      probe.y() += exit_footprint_extra_radius_ * std::sin(angle);
      if (!sdf_map_->isInMap(probe) ||
          sdf_map_->getOccupancy(probe) == SDFMap::OCCUPIED)
        return false;
    }
    return true;
  };
  const auto raw_segment_clear = [&raw_footprint_clear](const Eigen::Vector3d& start,
                                                        const Eigen::Vector3d& end) {
    const double length = (end - start).norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.08)));
    for (int index = 1; index <= samples; ++index) {
      const double ratio = static_cast<double>(index) / static_cast<double>(samples);
      if (!raw_footprint_clear(start + ratio * (end - start))) return false;
    }
    return true;
  };
  const double current_side =
      (cur_pos.head<2>() - exit_portal_center_.head<2>()).dot(direction);
  const double desired_side = std::min(exit_cross_target_distance_,
                                       std::max(0.0, current_side) + exit_cross_step_);
  bool found = false;
  for (double side = desired_side; side >= -0.05; side -= 0.10) {
    Eigen::Vector3d candidate = exit_portal_center_;
    candidate.head<2>() += side * direction;
    candidate.z() = cruise_height_;
    const bool known_free = mapPointSafe(candidate);
    if ((!known_free &&
         (!raw_footprint_clear(candidate) || !raw_segment_clear(cur_pos, candidate))) ||
        goalTemporarilyBlocked(candidate))
      continue;
    goal = candidate;
    if (!known_free)
      ROS_WARN_THROTTLE(1.0,
                        "[exit_mission] CROSS_EXIT uses occupied-free UNKNOWN probe "
                        "side=%.2fm step=%.2fm.",
                        side, (candidate.head<2>() - cur_pos.head<2>()).norm());
    found = true;
    break;
  }
  return found;
}

// 2026-07-22: 候选门框已经积累较高稳定度、但尚差少量确认时，只在门内观察点附近做小范围横移。
// 该目标不确认出口、不穿入UNKNOWN，也不调用旧frontier回头；作用仅是改变雷达视角补齐门框竖向支撑。
bool TaskSearchManager::buildPendingExitObservationGoal(
    const Eigen::Vector3d& cur_pos, Eigen::Vector3d& goal, double& goal_yaw) const {
  if (exit_candidate_confirmed_ ||
      exit_candidate_hits_ < std::max(1, exit_pending_observation_min_hits_) ||
      !pending_exit_portal_outside_verified_ ||
      // 2026-07-28: 拓扑端点仍在门内时不为疑似弧墙生成观察目标，避免主动靠近错误截面。
      !pending_exit_endpoint_outside_verified_ ||
      pending_exit_outward_direction_.norm() < 1e-3)
    return false;

  const Eigen::Vector2d direction = pending_exit_outward_direction_.normalized();
  const Eigen::Vector2d progress_direction = stableProgressDirection();
  const double direction_dot = std::max(
      -1.0, std::min(1.0, direction.dot(progress_direction)));
  const double direction_change_deg = std::acos(direction_dot) * 180.0 / M_PI;
  const double portal_forward =
      (pending_exit_portal_center_.head<2>() - cur_pos.head<2>())
          .dot(progress_direction);
  if (direction_change_deg > exit_max_direction_jump_deg_ ||
      portal_forward < -exit_candidate_max_behind_distance_) {
    ROS_ERROR_THROTTLE(
        1.0,
        "[exit_mission] reject pending observation behind progress: "
        "direction_jump=%.1f/%.1fdeg portal_forward=%.2f/%.2fm.",
        direction_change_deg, exit_max_direction_jump_deg_, portal_forward,
        -exit_candidate_max_behind_distance_);
    return false;
  }
  const Eigen::Vector2d lateral(-direction.y(), direction.x());
  const double lateral_offset = std::max(0.10, exit_pending_observation_lateral_offset_);
  const double min_move = std::max(0.10, exit_pending_observation_min_move_);
  std::vector<Eigen::Vector3d> candidates;
  for (double inward_offset : {0.0, 0.20}) {
    for (double side : {0.0, 1.0, -1.0}) {
      Eigen::Vector3d candidate = pending_exit_candidate_;
      candidate.head<2>() -= inward_offset * direction;
      candidate.head<2>() += side * lateral_offset * lateral;
      candidate.z() = clampSearchHeight(pending_exit_candidate_.z());
      candidates.push_back(candidate);
    }
  }

  double best_travel = std::numeric_limits<double>::infinity();
  bool found = false;
  for (const auto& candidate : candidates) {
    const double travel = (candidate.head<2>() - cur_pos.head<2>()).norm();
    if (travel + 1e-6 < min_move || !mapPointSafe(candidate) ||
        goalTemporarilyBlocked(candidate))
      continue;
    if (travel < best_travel) {
      best_travel = travel;
      goal = candidate;
      found = true;
    }
  }
  if (!found) return false;
  goal_yaw = std::atan2(pending_exit_portal_center_.y() - goal.y(),
                        pending_exit_portal_center_.x() - goal.x());
  return true;
}

void TaskSearchManager::updateExitCandidate(const Eigen::Vector3d& cur_pos) {
  if (!exit_detection_enabled_) return;
  if (!sdf_map_ || !corridor_frame_received_) return;
  // 2026-07-23: 尚未明确穿过入口时禁止运行出口检测；任务顺序固定为先入通道、后找最终出口。
  if (!corridor_entry_crossed_) return;
  // 2026-07-21: 进入门内接近/穿越后锁定已验证门平面；门外地图扩张产生的新最远点不能拖动穿门法向。
  if (mission_stage_ != SEARCH_CORRIDOR) return;
  // 2026-07-23: 最终出口满足稳定度和累计里程后采用入口同样的一次锁存语义；
  // 后续点云只更新占据地图，不再旋转、平移或撤销已锁定门框。
  if (exit_portal_locked_) return;
  // 2026-07-22: 搜索通道期间持续用累计地图复核；更远外部分支只更新预测点，不能拖动门框。
  const ros::Time now = ros::Time::now();
  if (!last_exit_inference_.isZero() &&
      (now - last_exit_inference_).toSec() < exit_inference_period_)
    return;
  last_exit_inference_ = now;

  // 2026-07-22: SEARCH_CORRIDOR阶段若飞机已经越过候选门平面，说明它仍在同一通道继续推进；
  // 该截面立即永久失效。不能保留4/5历史票数，更不能等frontier耗尽后命令返回门内点。
  if (exit_candidate_hits_ > 0 && pending_exit_outward_direction_.norm() > 1e-3) {
    const double pending_passed =
        (cur_pos.head<2>() - pending_exit_portal_center_.head<2>())
            .dot(pending_exit_outward_direction_.normalized());
    if (pending_passed > exit_candidate_max_behind_distance_) {
      ROS_ERROR("[exit_mission] discard passed portal candidate center=(%.2f, %.2f): "
                "leader is %.2fm beyond it while still SEARCH_CORRIDOR.",
                pending_exit_portal_center_.x(), pending_exit_portal_center_.y(),
                pending_passed);
      exit_candidate_hits_ = 0;
      pending_exit_endpoint_outside_verified_ = false;
      pending_exit_portal_outside_verified_ = false;
    }
  }
  if (exit_candidate_confirmed_ && exit_outward_direction_.norm() > 1e-3) {
    const double confirmed_passed =
        (cur_pos.head<2>() - exit_portal_center_.head<2>())
            .dot(exit_outward_direction_.normalized());
    if (confirmed_passed > exit_candidate_max_behind_distance_) {
      ROS_ERROR("[exit_mission] revoke passed EXIT PORTAL center=(%.2f, %.2f): leader "
                "continued %.2fm through the same corridor; never command a return.",
                exit_portal_center_.x(), exit_portal_center_.y(), confirmed_passed);
      exit_candidate_confirmed_ = false;
      exit_endpoint_outside_verified_ = false;
      exit_portal_outside_verified_ = false;
      exit_candidate_hits_ = 0;
      active_goal_valid_ = false;
    }
  }

  Eigen::Vector3d inferred, inferred_portal, inferred_endpoint;
  Eigen::Vector2d inferred_direction;
  bool inferred_endpoint_outside = false;
  // 2026-07-23: 门外FREE空间和拓扑预测点所属侧必须分别记录，不能因雷达确认开口
  // 就无条件把任意紫色endpoint_hint写成outside。
  bool inferred_portal_outside = false;
  double geodesic = 0.0;
  double confidence = 0.0;
  bool local_terminal_fallback = false;
  RadarExitResult fallback_radar_result;
  if (!inferExitFromMap(cur_pos, inferred, inferred_portal, inferred_direction,
                        inferred_endpoint, inferred_endpoint_outside,
                        geodesic, confidence)) {
    // 2026-07-28: U形通道末端可能因旧FREE被占据层覆盖而从入口拓扑根断开；此时不能把4m
    // 门槛盲目降低到入口附近，而是仅允许“累计实飞足够深+最近直线+1m内双墙共同终止”接管。
    double travelled = 0.0;
    for (size_t i = 1; i < visited_positions_.size(); ++i)
      travelled += (visited_positions_[i].head<2>() -
                    visited_positions_[i - 1].head<2>()).norm();

    Eigen::Vector2d recent_direction = corridor_dir_.head<2>();
    Eigen::Vector2d previous_direction = recent_direction;
    bool have_recent = false;
    bool have_previous = false;
    Eigen::Vector2d recent_anchor = cur_pos.head<2>();
    for (auto it = visited_positions_.rbegin(); it != visited_positions_.rend(); ++it) {
      const double arc = (recent_anchor - it->head<2>()).norm();
      if (!have_recent && arc >= 0.45) {
        recent_direction = (recent_anchor - it->head<2>()).normalized();
        have_recent = true;
      }
      if (arc >= 1.15) {
        Eigen::Vector2d middle = recent_anchor - 0.55 * recent_direction;
        Eigen::Vector2d older_segment = middle - it->head<2>();
        if (older_segment.norm() > 0.20) {
          previous_direction = older_segment.normalized();
          have_previous = true;
        }
        break;
      }
    }
    const double max_turn_rad = std::max(0.0, exit_near_fallback_max_turn_deg_) * M_PI / 180.0;
    const bool recent_straight = have_recent && have_previous &&
        recent_direction.dot(previous_direction) >= std::cos(max_turn_rad);
    const bool local_structure = travelled >= exit_near_fallback_min_travel_ &&
        recent_straight && detectLocalMapExit(cur_pos, recent_direction, fallback_radar_result);
    const double local_portal_distance = local_structure
        ? (cur_pos.head<2>() - fallback_radar_result.portal_center.head<2>()).norm()
        : std::numeric_limits<double>::infinity();
    if (!local_structure || local_portal_distance > exit_near_fallback_max_distance_) {
      ROS_WARN_THROTTLE(2.0,
                        "[exit_mission] no topology hint and near fallback rejected: "
                        "travel=%.2f/%.2fm straight=%d portal_distance=%.2f/%.2fm.",
                        travelled, exit_near_fallback_min_travel_,
                        static_cast<int>(recent_straight), local_portal_distance,
                        exit_near_fallback_max_distance_);
      return;
    }
    local_terminal_fallback = true;
    inferred_portal = fallback_radar_result.portal_center;
    inferred_direction = fallback_radar_result.outward_direction.normalized();
    inferred = inferred_portal;
    inferred_endpoint = inferred_portal;
    inferred_endpoint.head<2>() += exit_outside_probe_distance_ * inferred_direction;
    inferred_endpoint_outside = true;
    inferred_portal_outside = true;
    geodesic = travelled;
    confidence = fallback_radar_result.confidence;
    ROS_ERROR("[exit_mission] NEAR_TERMINAL_FALLBACK accepted: travel=%.2fm "
              "portal=(%.2f,%.2f) distance=%.2fm; still requires close multi-frame confirmation.",
              travelled, inferred_portal.x(), inferred_portal.y(), local_portal_distance);
  }
  if (radar_exit_detection_enabled_) {
    // 2026-07-23: 局部门法向优先使用最近实际航迹切向，而不是朝向远端预测点的直线；
    // U形通道中后者会穿墙，机头yaw又可能因观测任务转动，二者都不能代表通道推进方向。
    // 使用独立维护的推进方向，而不是最近0.45m位移；短回撤不能再把门的内外方向翻转。
    // 正常90度/U形转弯会由连续航迹段逐步更新该方向。
    Eigen::Vector2d forward_hint = stableProgressDirection();
    RadarExitResult radar_result;
    // 2026-07-23: RViz累计地图已形成清晰XY双墙断面时由局部地图主判；
    // body点云只在局部地图尚未成形时补充，不能再以坐标拼接误差否决明显出口。
    const bool local_map_confirmed = local_terminal_fallback
        ? (radar_result = fallback_radar_result, true)
        : detectLocalMapExit(cur_pos, forward_hint, radar_result);
    const bool body_radar_confirmed =
        local_map_confirmed ? false
                            : detectRadarExit(cur_pos, forward_hint, radar_result);
    if (!local_map_confirmed && !body_radar_confirmed) {
      const int previous_hits = exit_candidate_hits_;
      exit_candidate_hits_ = std::max(
          0, exit_candidate_hits_ - std::max(1, exit_candidate_reject_decay_));
      ROS_WARN_THROTTLE(
          2.0,
          "[exit_mission] topology endpoint exists but neither local occupancy XY "
          "wall-end nor body-radar confirmed the portal; hits decay %d->%d.",
          previous_hits, exit_candidate_hits_);
      return;
    }
    const Eigen::Vector2d detected_direction =
        radar_result.outward_direction.normalized();
    const double progress_dot = std::max(
        -1.0, std::min(1.0, detected_direction.dot(forward_hint)));
    const double progress_jump_deg = std::acos(progress_dot) * 180.0 / M_PI;
    const double portal_forward =
        (radar_result.portal_center.head<2>() - cur_pos.head<2>()).dot(forward_hint);
    if (progress_jump_deg > exit_max_direction_jump_deg_ ||
        portal_forward < -exit_candidate_max_behind_distance_) {
      const int previous_hits = exit_candidate_hits_;
      exit_candidate_hits_ = std::max(
          0, exit_candidate_hits_ - std::max(1, exit_candidate_reject_decay_));
      ROS_ERROR_THROTTLE(
          1.0,
          "[exit_mission] reject backward/abrupt portal center=(%.2f,%.2f): "
          "direction_jump=%.1f/%.1fdeg portal_forward=%.2f/%.2fm hits %d->%d.",
          radar_result.portal_center.x(), radar_result.portal_center.y(),
          progress_jump_deg, exit_max_direction_jump_deg_, portal_forward,
          -exit_candidate_max_behind_distance_, previous_hits, exit_candidate_hits_);
      return;
    }
    if (exit_candidate_hits_ > 0 && pending_exit_outward_direction_.norm() > 1e-3) {
      const double pending_dot = std::max(
          -1.0, std::min(1.0, detected_direction.dot(
              pending_exit_outward_direction_.normalized())));
      const double pending_jump_deg = std::acos(pending_dot) * 180.0 / M_PI;
      if (pending_jump_deg > exit_max_direction_jump_deg_) {
        const int previous_hits = exit_candidate_hits_;
        exit_candidate_hits_ = std::max(
            0, exit_candidate_hits_ - std::max(1, exit_candidate_reject_decay_));
        ROS_ERROR_THROTTLE(
            1.0,
            "[exit_mission] reject portal direction discontinuity %.1f/%.1fdeg; "
            "keep previous candidate, hits %d->%d.",
            pending_jump_deg, exit_max_direction_jump_deg_, previous_hits,
            exit_candidate_hits_);
        return;
      }
    }
    // 2026-07-23: 最新视频中第一个弯角已被拓扑报告normal_turn=1/2，但BODY_RADAR仍在
    // (4.31,0.09)累计到5/5。局部门框若落在同一弯角邻域，直接衰减并撤销该候选。
    double nearest_turn_distance = std::numeric_limits<double>::infinity();
    for (const auto& turn_center : last_topology_turn_centers_) {
      nearest_turn_distance = std::min(
          nearest_turn_distance,
          (radar_result.portal_center.head<2>() - turn_center.head<2>()).norm());
    }
    if (nearest_turn_distance <= exit_normal_turn_reject_radius_) {
      const int previous_hits = exit_candidate_hits_;
      exit_candidate_hits_ = std::max(
          0, exit_candidate_hits_ - std::max(1, exit_candidate_reject_decay_));
      if (exit_candidate_confirmed_ &&
          (exit_portal_center_.head<2>() -
           radar_result.portal_center.head<2>()).norm() <=
              exit_candidate_stability_radius_) {
        exit_candidate_confirmed_ = false;
        exit_endpoint_outside_verified_ = false;
        exit_portal_outside_verified_ = false;
        active_goal_valid_ = false;
      }
      ROS_ERROR_THROTTLE(
          1.0,
          "[exit_mission] reject local portal (%.2f, %.2f): topology marks the "
          "same cross-section as a normal turn (distance=%.2fm); hits %d->%d.",
          radar_result.portal_center.x(), radar_result.portal_center.y(),
          nearest_turn_distance, previous_hits, exit_candidate_hits_);
      return;
    }
    ROS_INFO_THROTTLE(1.0, "[exit_mission] portal evidence source=%s.",
                      local_map_confirmed ? "LOCAL_OCCUPANCY_XY" : "BODY_RADAR");
    inferred_portal = radar_result.portal_center;
    inferred_direction = radar_result.outward_direction.normalized();
    inferred = inferred_portal;
    bool safe_inside_found = false;
    for (double standoff = exit_standoff_; standoff >= 0.15; standoff -= 0.10) {
      Eigen::Vector3d inside = inferred_portal;
      inside.head<2>() -= standoff * inferred_direction;
      if (mapPointSafe(inside)) {
        inferred = inside;
        safe_inside_found = true;
        break;
      }
    }
    if (!safe_inside_found) {
      ROS_WARN_THROTTLE(2.0,
                        "[exit_mission] radar portal found but no safe inside standoff point.");
      return;
    }
    // 2026-07-23: 局部穿门射线只证明门法向前方存在开放空间；紫色拓扑预测点必须
    // 真实投影到门外侧才显示outside，不能继续无条件置true。
    inferred_portal_outside = true;
    const double endpoint_signed_side =
        (inferred_endpoint.head<2>() - inferred_portal.head<2>())
            .dot(inferred_direction);
    inferred_endpoint_outside = endpoint_signed_side >= 0.20;
    if (!inferred_endpoint_outside) {
      ROS_WARN_THROTTLE(
          1.0,
          "[exit_mission] portal outside space verified, but endpoint hint stays "
          "INSIDE signed_side=%.2fm; endpoint is visualization only.",
          endpoint_signed_side);
    }
    confidence = radar_result.confidence;
  } else if (!validateExitPortal(inferred, inferred_portal, inferred_direction)) {
    // 2026-07-22: 一帧地面/稀疏点抖动不能抹掉此前4/5的门框证据；连续失败仍会逐步降到0。
    const int previous_hits = exit_candidate_hits_;
    exit_candidate_hits_ = std::max(0, exit_candidate_hits_ -
                                           std::max(1, exit_candidate_reject_decay_));
    ROS_WARN_THROTTLE(2.0,
                      "[exit_mission] topology hint (%.2f, %.2f) rejected: no verified "
                      "two-sided portal/free opening/outside extension yet; hits decay %d->%d.",
                      inferred_portal.x(), inferred_portal.y(), previous_hits,
                      exit_candidate_hits_);
    return;
  } else {
    // 2026-07-23: 非雷达分支的旧验证器同时验证门后延伸，故沿用其外部分支结论。
    inferred_portal_outside = inferred_endpoint_outside;
  }
  const Eigen::Vector2d normalized_direction = inferred_direction.normalized();
  const bool direction_stable = exit_candidate_hits_ > 0 &&
      normalized_direction.dot(pending_exit_outward_direction_) >= std::cos(20.0 * M_PI / 180.0);
  if (exit_candidate_hits_ > 0 &&
      (inferred.head<2>() - pending_exit_candidate_.head<2>()).norm() <=
          exit_candidate_stability_radius_ && direction_stable) {
    ++exit_candidate_hits_;
    pending_exit_candidate_ = 0.65 * pending_exit_candidate_ + 0.35 * inferred;
    pending_exit_portal_center_ =
        0.65 * pending_exit_portal_center_ + 0.35 * inferred_portal;
    pending_exit_outward_direction_ =
        (0.65 * pending_exit_outward_direction_ + 0.35 * normalized_direction).normalized();
    pending_exit_endpoint_prediction_ = inferred_endpoint;
    pending_exit_endpoint_outside_verified_ =
        pending_exit_endpoint_outside_verified_ && inferred_endpoint_outside;
    pending_exit_portal_outside_verified_ =
        pending_exit_portal_outside_verified_ && inferred_portal_outside;
  } else {
    pending_exit_candidate_ = inferred;
    pending_exit_portal_center_ = inferred_portal;
    pending_exit_outward_direction_ = normalized_direction;
    pending_exit_endpoint_prediction_ = inferred_endpoint;
    pending_exit_endpoint_outside_verified_ = inferred_endpoint_outside;
    pending_exit_portal_outside_verified_ = inferred_portal_outside;
    exit_candidate_hits_ = 1;
  }
  // 2026-07-24: 每次有效门结构证据都延长前视窗口；即便下一帧票数衰减，
  // 相机环扫也不会立即重启并反过来破坏后续出口切面。
  exit_verification_hold_until_ =
      now + ros::Duration(std::max(0.0, exit_verification_yaw_hold_time_));

  // 2026-07-22: 只有更靠拓扑远端、并完整通过双墙终止和门后FREE复核的截面才能修正出口；
  // 入口侧旧截面永不覆盖任务推进方向上的新证据。
  const double pending_shift_from_confirmed =
      (pending_exit_candidate_.head<2>() - exit_candidate_.head<2>()).norm();
  const bool stable_more_endpoint_side_candidate =
      exit_candidate_confirmed_ &&
      exit_candidate_hits_ >= exit_supersede_confirmation_count_ &&
      geodesic > exit_geodesic_distance_ + exit_supersede_min_path_increase_ &&
      pending_shift_from_confirmed > exit_candidate_stability_radius_;
  if (stable_more_endpoint_side_candidate) {
    ROS_ERROR("[exit_mission] correct confirmed exit (%.2f, %.2f), path=%.2fm: "
              "more-endpoint-side wall termination is stable %d/%d at (%.2f, %.2f), path=%.2fm.",
              exit_candidate_.x(), exit_candidate_.y(), exit_geodesic_distance_,
              exit_candidate_hits_, exit_confirmation_count_,
              pending_exit_candidate_.x(), pending_exit_candidate_.y(), geodesic);
    exit_candidate_confirmed_ = false;
    active_goal_valid_ = false;
  }

  const double pending_portal_distance =
      (cur_pos.head<2>() - pending_exit_portal_center_.head<2>()).norm();
  // 2026-07-28: 远距离只保留候选票，不发布确认门；近场仍持续建图并会在阈值内立即复核。
  if (exit_candidate_hits_ >= exit_confirmation_count_ &&
      pending_portal_distance > exit_confirmation_max_distance_) {
    ROS_WARN_THROTTLE(
        1.0,
        "[exit_mission] portal evidence %d/%d held as candidate: distance=%.2fm > "
        "confirmation_max_distance=%.2fm; keep mapping until close verification.",
        exit_candidate_hits_, exit_confirmation_count_, pending_portal_distance,
        exit_confirmation_max_distance_);
  } else if (exit_candidate_hits_ >= exit_confirmation_count_) {
    const bool first_confirmation = !exit_candidate_confirmed_;
    const bool more_endpoint_side_branch = exit_candidate_confirmed_ &&
        geodesic > exit_geodesic_distance_ + exit_supersede_min_path_increase_;
    const bool same_branch = exit_candidate_confirmed_ &&
        (pending_exit_candidate_.head<2>() - exit_candidate_.head<2>()).norm() <= 1.0;
    // 2026-07-28: 真门以近场双侧墙端+门外FREE射线为硬证据；拓扑endpoint只是
    // 远端搜索提示，U形通道内它可能因拐弯投影落在门内，不得否决已经5/5的真实门框。
    // 防误检仍由拐弯邻域排除、近距离确认和最小航程锁定三层条件负责。
    if ((first_confirmation || more_endpoint_side_branch || same_branch) &&
        pending_exit_portal_outside_verified_) {
      const double candidate_shift =
          (pending_exit_candidate_.head<2>() - exit_candidate_.head<2>()).norm();
      exit_candidate_ = pending_exit_candidate_;
      exit_portal_center_ = pending_exit_portal_center_;
      exit_outward_direction_ = pending_exit_outward_direction_;
      exit_endpoint_prediction_ = pending_exit_endpoint_prediction_;
      exit_endpoint_outside_verified_ = pending_exit_endpoint_outside_verified_;
      exit_portal_outside_verified_ = pending_exit_portal_outside_verified_;
      exit_geodesic_distance_ = geodesic;
      exit_confidence_ = confidence;
      exit_candidate_confirmed_ = true;
      // 2026-07-23: 最新视频第一弯的稳定墙端约5.06m，只保留为可撤销候选；
      // 末端真门约11.13m达到锁门里程后冻结中心/法向，并清除仍想跨门的普通活动目标。
      if (!exit_portal_locked_ &&
          exit_geodesic_distance_ + 1e-6 >= exit_lock_min_geodesic_distance_) {
        exit_portal_locked_ = true;
        active_goal_valid_ = false;
        search_exhausted_since_ = ros::Time(0);
        const double locked_yaw =
            std::atan2(exit_outward_direction_.y(), exit_outward_direction_.x());
        ROS_ERROR("[exit_mission] FINAL EXIT PORTAL LOCKED center=(%.2f, %.2f), "
                  "outward_yaw=%.1fdeg path=%.2fm >= %.2fm; stop exit detection "
                  "and ordinary cross-door frontiers.",
                  exit_portal_center_.x(), exit_portal_center_.y(),
                  locked_yaw * 180.0 / M_PI, exit_geodesic_distance_,
                  exit_lock_min_geodesic_distance_);
        // 2026-07-28: 最终出口门心/外向法向只在永久锁门瞬间发布一次；锁存话题保证后机晚启动也能收到。
        geometry_msgs::PoseStamped final_exit_pose;
        final_exit_pose.header.stamp = now;
        final_exit_pose.header.frame_id = world_frame_;
        final_exit_pose.pose.position.x = exit_portal_center_.x();
        final_exit_pose.pose.position.y = exit_portal_center_.y();
        final_exit_pose.pose.position.z = exit_portal_center_.z();
        final_exit_pose.pose.orientation.w = std::cos(0.5 * locked_yaw);
        final_exit_pose.pose.orientation.z = std::sin(0.5 * locked_yaw);
        final_exit_pose_pub_.publish(final_exit_pose);
        ROS_ERROR("[exit_mission] PUBLISH FINAL EXIT relay pose center=(%.2f, %.2f, %.2f) "
                  "outward_yaw=%.1fdeg.",
                  exit_portal_center_.x(), exit_portal_center_.y(),
                  exit_portal_center_.z(), locked_yaw * 180.0 / M_PI);
      }
      geometry_msgs::PoseStamped exit_pose;
      exit_pose.header.stamp = now;
      exit_pose.header.frame_id = world_frame_;
      exit_pose.pose.position.x = exit_portal_center_.x();
      exit_pose.pose.position.y = exit_portal_center_.y();
      exit_pose.pose.position.z = exit_portal_center_.z();
      const double exit_yaw = std::atan2(exit_outward_direction_.y(), exit_outward_direction_.x());
      exit_pose.pose.orientation.w = std::cos(0.5 * exit_yaw);
      exit_pose.pose.orientation.z = std::sin(0.5 * exit_yaw);
      exit_pose_pub_.publish(exit_pose);
      // 2026-07-14: 稳定地图每次推理都会再次确认同一出口，这不是错误；首次确认、
      // 更远分支或位置明显变化才输出 INFO，其余重复确认仅保留 DEBUG。
      if (first_confirmation || more_endpoint_side_branch || candidate_shift > 0.25) {
        // 2026-07-23: SEARCH_CORRIDOR中的稳定结构仍只是通道内墙端证据；只有
        // buildStage3Goal在任务完成/稳定耗尽后才把它升级为最终出口。
        ROS_INFO("[exit_mission] WALL-END STRUCTURE stable while INSIDE center=(%.2f, %.2f, %.2f), "
                 "inside=(%.2f, %.2f), endpoint_hint=(%.2f, %.2f) endpoint_side=%s, "
                 "outside_space=%d, "
                 "outward_yaw=%.1fdeg path=%.2fm confidence=%.2f.",
                 exit_portal_center_.x(), exit_portal_center_.y(), exit_portal_center_.z(),
                 exit_candidate_.x(), exit_candidate_.y(), exit_endpoint_prediction_.x(),
                 exit_endpoint_prediction_.y(),
                 exit_endpoint_outside_verified_ ? "OUT" : "IN",
                 static_cast<int>(exit_portal_outside_verified_),
                 exit_yaw * 180.0 / M_PI,
                 exit_geodesic_distance_, exit_confidence_);
      } else {
        ROS_DEBUG_THROTTLE(2.0,
                           "[exit_mission] stable EXIT candidate (%.2f, %.2f), confidence=%.2f.",
                           exit_candidate_.x(), exit_candidate_.y(), exit_confidence_);
      }
    } else if (exit_candidate_hits_ >= exit_confirmation_count_ &&
               !pending_exit_portal_outside_verified_) {
      // 2026-07-28: 只在真正缺少门外FREE证据时拒绝；endpoint内外不再参与门框确认。
      ROS_ERROR_THROTTLE(1.0,
                         "[exit_mission] EXIT_REJECT no verified free space beyond portal; "
                         "keep mapping before confirmation.");
    }
  } else {
    ROS_WARN("[exit_mission] wall-end portal waiting confirmation %d/%d center=(%.2f, %.2f), "
             "endpoint_hint=(%.2f, %.2f) outside=%d path=%.2fm confidence=%.2f.",
             exit_candidate_hits_, exit_confirmation_count_, inferred_portal.x(),
             inferred_portal.y(), inferred_endpoint.x(), inferred_endpoint.y(),
             static_cast<int>(inferred_endpoint_outside), geodesic, confidence);
  }
  publishSearchState();
}

bool TaskSearchManager::buildStage3Goal(const Eigen::Vector3d& cur_pos, double cur_yaw,
                                        bool search_exhausted, Eigen::Vector3d& goal,
                                        double& goal_yaw) {
  // 出口总开关关闭时，保持普通通道探索，不推断出口、不切换出口任务阶段。
  if (!enabled_ || !exit_detection_enabled_ || landing_requested_) return false;
  updateExitCandidate(cur_pos);

  if (mission_stage_ == SEARCH_CORRIDOR) {
    // 2026-07-16: 无摄像头时三类接口不阻塞任务，但也不能把“关闭识别”冒充目标全部找到，
    // 否则会在第一个中途出口候选处过早切换；当前直降模式由最终出口距离和到达条件独立触发。
    const bool targets_complete = require_stage2_detections_ && allStage2TargetsFound();
    if (!targets_complete) {
      if (!search_exhausted) {
        // 2026-07-16: false调用也用于正常搜索期刷新出口推测，不能在这里清零耗尽计时；
        // 只有reportSearchCoverageAvailable确认本轮确有frontier/viewpoint时才取消计时。
        return false;
      }
      if (search_exhausted_since_.isZero()) {
        search_exhausted_since_ = ros::Time::now();
        ROS_WARN("[exit_mission] no coverable frontier observed; start %.1fs exhaustion confirmation.",
                 search_exhausted_confirm_time_);
        return false;
      }
      const double exhausted_duration =
          (ros::Time::now() - search_exhausted_since_).toSec();
      if (exhausted_duration < search_exhausted_confirm_time_) {
        ROS_WARN_THROTTLE(1.0,
                          "[exit_mission] search exhaustion pending %.1f/%.1fs; keep searching.",
                          exhausted_duration, search_exhausted_confirm_time_);
        return false;
      }
    }
    if (!exit_candidate_confirmed_ || !exit_portal_locked_) {
      // 2026-07-22: frontier稳定耗尽且门框已有较高累计证据时，去门内安全观察位补扫，
      // 不再因为一次复核抖动清零后在出口原地safety_hold。
      if (!exit_candidate_confirmed_ &&
          buildPendingExitObservationGoal(cur_pos, goal, goal_yaw)) {
        ROS_WARN_THROTTLE(1.0,
                          "[exit_mission] pending portal %d/%d: use inside observation "
                          "goal (%.2f, %.2f, %.2f), yaw=%.1fdeg.",
                          exit_candidate_hits_, exit_confirmation_count_, goal.x(), goal.y(),
                          goal.z(), goal_yaw * 180.0 / M_PI);
        return true;
      }
      // 2026-07-23: 中途5/5墙端尚未达到最终锁门里程时不能因一次frontier耗尽抢占任务；
      // 保持可撤销，继续沿通道推进寻找真正末端。
      if (exit_candidate_confirmed_ && !exit_portal_locked_) {
        ROS_WARN_THROTTLE(
            1.0,
            "[exit_mission] stable wall-end path %.2fm is below final lock %.2fm; "
            "keep corridor search and allow it to be passed/revoked.",
            exit_geodesic_distance_, exit_lock_min_geodesic_distance_);
      }
      return false;
    }
    // 2026-07-22: 不使用固定9m里程猜最终出口；必须由累计地图证明预测点位于门框之后的外部FREE分支。
    if (!exit_portal_outside_verified_) {
      ROS_WARN_THROTTLE(1.0,
                        "[exit_mission] portal exists but door-outward FREE space is not verified; "
                        "keep corridor search.");
      return false;
    }

    // 2026-07-21: 删除“旧出口则在当前位置直降”的旁路；确认对象是门框，只能先去门内观察点。
    mission_stage_ = EXIT_APPROACH_INSIDE;
    mission_stage_start_ = ros::Time::now();
    active_goal_valid_ = false;
    ROS_ERROR("[exit_mission] stage SEARCH_CORRIDOR -> EXIT_APPROACH_INSIDE, reason=%s.",
              targets_complete ? "all required stage-2 targets found"
                               : "search coverage exhausted");
  }

  if (mission_stage_ == EXIT_APPROACH_INSIDE) {
    if ((cur_pos.head<2>() - exit_candidate_.head<2>()).norm() <= exit_arrive_distance_) {
      mission_stage_ = CROSS_EXIT;
      mission_stage_start_ = ros::Time::now();
      active_goal_valid_ = false;
      ROS_ERROR("[exit_mission] stage EXIT_APPROACH_INSIDE -> CROSS_EXIT, portal=(%.2f, %.2f).",
                exit_portal_center_.x(), exit_portal_center_.y());
    } else {
      goal = exit_candidate_;
      if (!mapPointSafe(goal) || goalTemporarilyBlocked(goal)) {
        ROS_WARN_THROTTLE(1.0, "[exit_mission] verified portal inside approach temporarily blocked.");
        return false;
      }
      goal_yaw = std::atan2(exit_outward_direction_.y(), exit_outward_direction_.x());
      return true;
    }
  }

  if (mission_stage_ == CROSS_EXIT) {
    const double signed_side =
        (cur_pos.head<2>() - exit_portal_center_.head<2>()).dot(exit_outward_direction_);
    if (signed_side >= exit_cross_confirm_distance_) {
      mission_stage_ = SEARCH_OUTSIDE_LANDING;
      mission_stage_start_ = ros::Time::now();
      active_goal_valid_ = false;
      outside_search_anchor_ = cur_pos;
      outside_search_anchor_.z() = cruise_height_;
      search_exhausted_since_ = ros::Time(0);
      publishSearchState();
      ROS_ERROR("[exit_mission] stage CROSS_EXIT -> SEARCH_OUTSIDE_LANDING, "
                "signed_side=%.2fm.",
                signed_side);
    } else {
      if (!buildSafeExitCrossingGoal(cur_pos, goal)) {
        ROS_WARN_THROTTLE(1.0,
                          "[exit_mission] CROSS_EXIT has no OCCUPIED-free footprint/axis step.");
        return false;
      }
      goal_yaw = std::atan2(exit_outward_direction_.y(), exit_outward_direction_.x());
      return true;
    }
  }

  if (mission_stage_ == SEARCH_OUTSIDE_LANDING) {
    // 出口只是FUEL与Diff的职责分界。启用外部链路后，FUEL不再选择任何门外
    // 搜索/接近目标，也不再发布降落请求；其残余轨迹输出由规划器仲裁器隔离。
    if (external_landing_planner_) return false;
    if (final_landing_marker_.found) {
      mission_stage_ = APPROACH_LANDING;
      mission_stage_start_ = ros::Time::now();
      active_goal_valid_ = false;
      publishSearchState();
      ROS_ERROR("[exit_mission] stage SEARCH_OUTSIDE_LANDING -> APPROACH_LANDING.");
    } else {
      // 门外仍有 frontier 时继续覆盖；耗尽后围绕出口锚点做局部平台搜索。
      if (!search_exhausted) return false;
      const int phase = static_cast<int>(
          std::floor((ros::Time::now() - mission_stage_start_).toSec() /
                     std::max(0.3, scan_dwell_time_))) % 8;
      for (int attempt = 0; attempt < 8; ++attempt) {
        const double angle = (phase + attempt) * M_PI_4;
        Eigen::Vector3d scan_goal = outside_search_anchor_;
        scan_goal.x() += scan_radius_ * std::cos(angle);
        scan_goal.y() += scan_radius_ * std::sin(angle);
        if (!mapPointSafe(scan_goal) || goalTemporarilyBlocked(scan_goal)) continue;
        goal = scan_goal;
        goal_yaw = wrapYaw(cur_yaw + M_PI_2);
        return true;
      }
      goal = outside_search_anchor_;
      goal_yaw = wrapYaw(cur_yaw + M_PI_2);
      return mapPointSafe(goal);
    }
  }

  if (mission_stage_ == APPROACH_LANDING) {
    const Eigen::Vector3d marker(final_landing_marker_.pose.pose.position.x,
                                 final_landing_marker_.pose.pose.position.y,
                                 final_landing_marker_.pose.pose.position.z);
    goal = Eigen::Vector3d(marker.x(), marker.y(),
                           marker.z() + landing_approach_height_);
    if (!landingColumnSafe(marker, goal) || goalTemporarilyBlocked(goal)) {
      ROS_WARN_THROTTLE(1.0,
                        "[exit_mission] landing safety cylinder is not clear; "
                        "continue mapping without handoff.");
      return false;
    }
    goal_yaw = std::atan2(goal.y() - cur_pos.y(), goal.x() - cur_pos.x());
    geometry_msgs::PoseStamped landing_target = final_landing_marker_.pose;
    landing_target.header.stamp = ros::Time::now();
    landing_target.header.frame_id = world_frame_;
    landing_target.pose.position.z = goal.z();
    landing_target_pub_.publish(landing_target);
    return true;
  }

  return false;
}

void TaskSearchManager::reportSearchCoverageAvailable() {
  if (!search_exhausted_since_.isZero()) {
    // 2026-07-16: 任意可覆盖viewpoint恢复即证明地图仍可继续探索，取消临时终点切换倒计时。
    ROS_WARN("[exit_mission] coverable search target recovered; cancel exhaustion confirmation.");
    search_exhausted_since_ = ros::Time(0);
  }
}

void TaskSearchManager::publishLandingRequest(bool active) {
  std_msgs::Bool request;
  request.data = active;
  landing_request_pub_.publish(request);
  landing_requested_ = active;
}

void TaskSearchManager::publishSearchState() {
  visualization_msgs::Marker visited;
  visited.header.stamp = ros::Time::now();
  visited.header.frame_id = world_frame_;
  visited.ns = "task_search_coverage";
  visited.id = 0;
  visited.type = visualization_msgs::Marker::SPHERE_LIST;
  visited.action = visualization_msgs::Marker::ADD;
  visited.scale.x = visited.scale.y = visited.scale.z = 0.12;
  visited.color.r = 0.10;
  visited.color.g = 0.85;
  visited.color.b = 0.25;
  visited.color.a = 0.85;
  for (const auto& pos : visited_positions_) {
    geometry_msgs::Point point;
    point.x = pos.x();
    point.y = pos.y();
    point.z = pos.z();
    visited.points.push_back(point);
  }
  marker_pub_.publish(visited);

  const float colors[3][3] = {{1.0f, 0.85f, 0.1f}, {0.1f, 0.4f, 1.0f}, {1.0f, 0.1f, 0.1f}};
  for (int i = 0; i < 3; ++i) {
    if (!targets_[i].found) continue;
    visualization_msgs::Marker marker;
    marker.header = targets_[i].pose.header;
    if (marker.header.frame_id.empty()) marker.header.frame_id = world_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "task_targets";
    marker.id = 10 + i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = targets_[i].pose.pose;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.28;
    marker.color.r = colors[i][0];
    marker.color.g = colors[i][1];
    marker.color.b = colors[i][2];
    marker.color.a = 1.0;
    marker_pub_.publish(marker);
  }

  // 2026-07-23: 单通道工程模式不再在RViz显示紫色拓扑预测点；它只保留为内部找远端的
  // 辅助量，不能被误认为门、终点或门外区域。持续发送DELETE清理旧锁存Marker。
  visualization_msgs::Marker delete_endpoint;
  delete_endpoint.header.stamp = ros::Time::now();
  delete_endpoint.header.frame_id = world_frame_;
  delete_endpoint.ns = "exit_mission";
  delete_endpoint.action = visualization_msgs::Marker::DELETE;
  delete_endpoint.id = 33;
  marker_pub_.publish(delete_endpoint);
  delete_endpoint.id = 34;
  marker_pub_.publish(delete_endpoint);

  // 2026-07-23: RViz始终把已有证据画成真正的矩形门框，而不是只给一个不直观的圆点。
  // 橙色表示仍在累计证据，绿色表示已确认；文字明确区分门框、紫色拓扑预测点和飞行目标。
  const bool has_exit_portal_visual = exit_candidate_confirmed_ || exit_candidate_hits_ > 0;
  if (has_exit_portal_visual) {
    const Eigen::Vector3d& visual_portal =
        exit_candidate_confirmed_ ? exit_portal_center_ : pending_exit_portal_center_;
    Eigen::Vector2d visual_outward = exit_candidate_confirmed_
        ? exit_outward_direction_ : pending_exit_outward_direction_;
    if (visual_outward.norm() < 1e-3) visual_outward = Eigen::Vector2d::UnitX();
    visual_outward.normalize();
    const Eigen::Vector2d visual_lateral(-visual_outward.y(), visual_outward.x());
    const bool confirmed = exit_candidate_confirmed_;

    visualization_msgs::Marker exit_marker;
    exit_marker.header.stamp = ros::Time::now();
    exit_marker.header.frame_id = world_frame_;
    exit_marker.ns = "exit_mission";
    exit_marker.id = 30;
    exit_marker.type = visualization_msgs::Marker::CYLINDER;
    exit_marker.action = visualization_msgs::Marker::ADD;
    exit_marker.pose.position.x = visual_portal.x();
    exit_marker.pose.position.y = visual_portal.y();
    exit_marker.pose.position.z = visual_portal.z();
    exit_marker.pose.orientation.w = 1.0;
    exit_marker.scale.x = exit_marker.scale.y = 0.45;
    exit_marker.scale.z = 0.10;
    exit_marker.color.r = confirmed ? 0.05 : 1.0;
    exit_marker.color.g = confirmed ? 1.0 : 0.48;
    exit_marker.color.b = confirmed ? 0.20 : 0.05;
    exit_marker.color.a = 0.95;
    marker_pub_.publish(exit_marker);

    visualization_msgs::Marker exit_text = exit_marker;
    exit_text.id = 31;
    exit_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    exit_text.pose.position.z += 0.45;
    exit_text.scale.z = 0.20;
    exit_text.color.r = confirmed ? 0.10 : 1.0;
    exit_text.color.g = 1.0;
    exit_text.color.b = confirmed ? 0.25 : 0.10;
    std::ostringstream exit_stream;
    if (confirmed && mission_stage_ != SEARCH_CORRIDOR) {
      static const char* stage_names[] = {
          "SEARCH", "APPROACH_INSIDE", "CROSS_EXIT",
          "SEARCH_OUTSIDE_LANDING", "APPROACH_LANDING", "LANDING"};
      const int stage_index = std::max(
          0, std::min(static_cast<int>(mission_stage_),
                      static_cast<int>(sizeof(stage_names) / sizeof(stage_names[0])) - 1));
      exit_stream << "EXIT DOOR CONFIRMED / " << stage_names[stage_index]
                  << " path=" << std::fixed << std::setprecision(1)
                  << exit_geodesic_distance_ << "m";
    } else if (confirmed && exit_portal_locked_) {
      // 2026-07-23: 明确区分可撤销墙端与已冻结最终门，方便RViz确认检测已经停止。
      exit_stream << "EXIT DOOR LOCKED / WAIT STAGE path="
                  << std::fixed << std::setprecision(1)
                  << exit_geodesic_distance_ << "m";
    } else if (confirmed) {
      exit_stream << "WALL-END READY / STILL INSIDE";
    } else {
      exit_stream << "WALL-END CANDIDATE / INSIDE "
                  << exit_candidate_hits_ << "/" << exit_confirmation_count_;
    }
    exit_text.text = exit_stream.str();
    marker_pub_.publish(exit_text);

    visualization_msgs::Marker exit_direction = exit_marker;
    exit_direction.id = 32;
    exit_direction.type = visualization_msgs::Marker::ARROW;
    exit_direction.points.clear();
    geometry_msgs::Point arrow_start, arrow_end;
    arrow_start = exit_marker.pose.position;
    arrow_end = arrow_start;
    arrow_end.x += 0.85 * visual_outward.x();
    arrow_end.y += 0.85 * visual_outward.y();
    exit_direction.points.push_back(arrow_start);
    exit_direction.points.push_back(arrow_end);
    exit_direction.scale.x = 0.08;
    exit_direction.scale.y = 0.16;
    exit_direction.scale.z = 0.16;
    exit_direction.color.r = confirmed ? 0.10 : 1.00;
    exit_direction.color.g = confirmed ? 0.90 : 0.55;
    exit_direction.color.b = confirmed ? 1.00 : 0.05;
    marker_pub_.publish(exit_direction);

    visualization_msgs::Marker door_frame = exit_marker;
    door_frame.id = 35;
    door_frame.type = visualization_msgs::Marker::LINE_LIST;
    // 2026-07-23: LINE_LIST顶点使用world绝对坐标，Marker位姿必须归零，避免门框被中心坐标二次平移。
    door_frame.pose.position.x = 0.0;
    door_frame.pose.position.y = 0.0;
    door_frame.pose.position.z = 0.0;
    door_frame.pose.orientation.x = 0.0;
    door_frame.pose.orientation.y = 0.0;
    door_frame.pose.orientation.z = 0.0;
    door_frame.pose.orientation.w = 1.0;
    door_frame.points.clear();
    door_frame.scale.x = 0.10;
    door_frame.color.r = confirmed ? 0.05 : 1.0;
    door_frame.color.g = confirmed ? 1.0 : 0.48;
    door_frame.color.b = confirmed ? 0.20 : 0.05;
    door_frame.color.a = 1.0;
    const double half_width = std::max(0.55, exit_portal_min_half_width_);
    const double bottom_z = std::max(0.05, visual_portal.z() - 0.55);
    const double top_z = visual_portal.z() + 0.75;
    const Eigen::Vector2d left_xy =
        visual_portal.head<2>() + half_width * visual_lateral;
    const Eigen::Vector2d right_xy =
        visual_portal.head<2>() - half_width * visual_lateral;
    auto frame_point = [](const Eigen::Vector2d& xy, double z) {
      geometry_msgs::Point point;
      point.x = xy.x();
      point.y = xy.y();
      point.z = z;
      return point;
    };
    const geometry_msgs::Point left_bottom = frame_point(left_xy, bottom_z);
    const geometry_msgs::Point left_top = frame_point(left_xy, top_z);
    const geometry_msgs::Point right_bottom = frame_point(right_xy, bottom_z);
    const geometry_msgs::Point right_top = frame_point(right_xy, top_z);
    door_frame.points = {
        left_bottom, left_top, right_bottom, right_top,
        left_top, right_top, left_bottom, right_bottom};
    marker_pub_.publish(door_frame);

    // 2026-07-23: 只有任务状态机正式升级为最终出口后才显示IN/OUT；普通墙端候选
    // 全部仍属于通道内，禁止局部几何提前制造“门外”语义。
    if (confirmed && mission_stage_ != SEARCH_CORRIDOR) {
      visualization_msgs::Marker side_text = exit_text;
      side_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      side_text.scale.z = 0.18;
      side_text.pose.position.z = visual_portal.z() + 0.18;
      side_text.id = 36;
      side_text.pose.position.x = visual_portal.x() - 0.65 * visual_outward.x();
      side_text.pose.position.y = visual_portal.y() - 0.65 * visual_outward.y();
      side_text.color.r = 0.20;
      side_text.color.g = 0.65;
      side_text.color.b = 1.00;
      side_text.text = "IN / CORRIDOR";
      marker_pub_.publish(side_text);
      side_text.id = 37;
      side_text.pose.position.x = visual_portal.x() + 0.65 * visual_outward.x();
      side_text.pose.position.y = visual_portal.y() + 0.65 * visual_outward.y();
      side_text.color.r = 0.20;
      side_text.color.g = 1.00;
      side_text.color.b = 0.30;
      side_text.text = "OUT / ARUCO-LANDING";
      marker_pub_.publish(side_text);
    } else {
      visualization_msgs::Marker delete_side = exit_marker;
      delete_side.action = visualization_msgs::Marker::DELETE;
      for (int marker_id : {36, 37}) {
        delete_side.id = marker_id;
        marker_pub_.publish(delete_side);
      }
    }
  } else {
    // 2026-07-23: 候选消失时清理锁存的中心、文字、箭头和门框，避免RViz继续显示旧出口。
    visualization_msgs::Marker delete_exit;
    delete_exit.header.stamp = ros::Time::now();
    delete_exit.header.frame_id = world_frame_;
    delete_exit.ns = "exit_mission";
    delete_exit.action = visualization_msgs::Marker::DELETE;
    for (int marker_id : {30, 31, 32, 35, 36, 37}) {
      delete_exit.id = marker_id;
      marker_pub_.publish(delete_exit);
    }
  }

  // 稳定 ArUco 世界位置确认后单独显示降落平台；未确认时禁止接近和降落。
  if (final_landing_marker_.found) {
    visualization_msgs::Marker landing_marker;
    landing_marker.header = final_landing_marker_.pose.header;
    landing_marker.header.stamp = ros::Time::now();
    if (landing_marker.header.frame_id.empty()) landing_marker.header.frame_id = world_frame_;
    landing_marker.ns = "exit_mission";
    landing_marker.id = 40;
    landing_marker.type = visualization_msgs::Marker::CUBE;
    landing_marker.action = visualization_msgs::Marker::ADD;
    landing_marker.pose = final_landing_marker_.pose.pose;
    landing_marker.scale.x = landing_marker.scale.y = 0.60;
    landing_marker.scale.z = 0.08;
    landing_marker.color.r = 0.05;
    landing_marker.color.g = 0.95;
    landing_marker.color.b = 0.95;
    landing_marker.color.a = 0.95;
    marker_pub_.publish(landing_marker);
  }

  std_msgs::String status;
  std::ostringstream stream;
  // 2026-07-13: 状态话题输出完整任务阶段，供后续第二架无人机和比赛任务面板直接订阅。
  // 状态话题明确区分门内接近、穿门和门外降落平台搜索。
  const char* stage_names[] = {"SEARCH_CORRIDOR", "EXIT_APPROACH_INSIDE", "CROSS_EXIT",
                               "SEARCH_OUTSIDE_LANDING", "APPROACH_LANDING", "LANDING"};
  stream << stage_names[static_cast<int>(mission_stage_)]
         << " color=" << targets_[0].found << " qrcode=" << targets_[1].found
         << " thermal=" << targets_[2].found << " exit=" << exit_candidate_confirmed_
         << " endpoint_hint=" << (exit_candidate_hits_ > 0)
         << " endpoint_outside=" << exit_endpoint_outside_verified_
         << " landing_aruco=" << final_landing_marker_.found
         << " landing=" << landing_requested_
         << " visited=" << visited_positions_.size();
  status.data = stream.str();
  status_pub_.publish(status);
}

}  // namespace fast_planner
