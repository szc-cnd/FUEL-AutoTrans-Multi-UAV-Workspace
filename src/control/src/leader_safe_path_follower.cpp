// 2026-07-16: 后机改为任务接力模式：门前原位等待，发布门点、滚动内部点和终点；
// 2026-07-16: 稀疏任务点只负责通信和阶段同步，点间运动沿前机实飞稠密折线执行，禁止直连跨墙。
// 2026-07-16: 取消固定5点上限；内部点可持续释放，只有真实降落请求才结束内部点生成并追加终点。

#include <algorithm>
#include <array>
#include <bspline/Bspline.h>
#include <bspline/non_uniform_bspline.h>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <sstream>  // 2026-07-28: 解析Diff返回的实际安全目标坐标。
#include <stdexcept>
#include <string>
#include <vector>  // 2026-07-16: 保存按任务进度持续生成并顺序执行的离散接力点。

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ldop/DynamicObjectArray.h>  // LDOP稳定目标状态；跟随器按自身规划时域进行短期预测。
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

namespace {

struct RoutePoint {
  geometry_msgs::Point position;
  double yaw{0.0};
  // 2026-07-15: 记录前机从起飞后的累计路程，用于“离开候选点1m后再释放”的离散接力逻辑。
  double progress{0.0};
};

struct DynamicObstacleSample {
  uint32_t id{0U};
  geometry_msgs::Point position;
  geometry_msgs::Vector3 size;
  std::vector<geometry_msgs::Point> predicted_positions;
};

struct BlockedRouteCandidate {
  geometry_msgs::Point position;
  double progress{0.0};
  ros::Time retry_after;
};

double distance3d(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::Point interpolate(const geometry_msgs::Point& a,
                                 const geometry_msgs::Point& b, double ratio) {
  geometry_msgs::Point result;
  result.x = a.x + ratio * (b.x - a.x);
  result.y = a.y + ratio * (b.y - a.y);
  result.z = a.z + ratio * (b.z - a.z);
  return result;
}

}  // namespace

class LeaderSafePathFollower {
 public:
  LeaderSafePathFollower() : nh_(), pnh_("~") {
    // 2026-07-27: 所有双机默认话题前缀统一为 UAV0/UAV1；公共坐标偏移和安全阈值仍可由 launch 标定。
    pnh_.param<std::string>("leader_odom_topic", leader_odom_topic_,
                            "/UAV0/fast_lio/Odom_high_freq");
    pnh_.param<std::string>("leader_history_path_topic",
                            leader_history_path_topic_,
                            "/UAV0/fast_lio/path");
    pnh_.param<std::string>("follower_odom_topic", follower_odom_topic_,
                            "/UAV1/fast_lio/Odometry");
    pnh_.param<std::string>("follower_cloud_topic", follower_cloud_topic_,
                            "/UAV1/fast_lio/cloud_registered");
    pnh_.param<std::string>("leader_trajectory_topic", leader_trajectory_topic_,
                            "/UAV0/planning/bspline");
    pnh_.param("enable_search_landing", enable_search_landing_, false);
    // 2026-07-29: 起飞就绪由各自 FAST-LIO 高度锁存，不再依赖 start_after_hover Bool。
    pnh_.param("leader_start_height", leader_start_height_, 0.3);
    pnh_.param("follower_start_height", follower_start_height_, 0.3);
    pnh_.param<std::string>("command_topic", command_topic_,
                            "/UAV1/planning/pos_cmd");
    // 2026-07-28: 新模式只向UAV1独立Diff发布离散目标；旧PositionCommand直控保留为可回退开关。
    pnh_.param("use_diff_planner", use_diff_planner_, true);
    // 简化接力模式把前机每段实际停稳终点作为FIFO目标直接交给Diff。跟随器不再
    // 生成短子目标、拉黑候选或退让坐标，规划失败时只重试当前队首。
    pnh_.param("simple_segment_endpoint_following",
               simple_segment_endpoint_following_, false);
    pnh_.param("simple_diff_retry_delay", simple_diff_retry_delay_, 0.20);
    pnh_.param<std::string>("diff_goal_topic", diff_goal_topic_,
                            "/UAV1/planning/goal");
    pnh_.param<std::string>("traj_started_topic", traj_started_topic_,
                            "/UAV1/planning/traj_started");
    // 2026-07-28: Diff若因瞬时占据把目标改到当前位置，周期重发原接力点，不能一次发送后永久锁死。
    pnh_.param("diff_goal_retry_period", diff_goal_retry_period_, 1.0);
    // 单次目标从首次发布起的总应答上限。与周期重发计时分离，规划器假死时
    // 必须把当前候选判失败并换点，不能每次重发都把超时起点向后推。
    pnh_.param("diff_goal_response_timeout", diff_goal_response_timeout_, 2.0);
    // 2026-07-28: 订阅UAV1 Diff明确规划状态；失败后沿前机实飞路线短步恢复。
    pnh_.param<std::string>("diff_status_topic", diff_status_topic_,
                            "/drone_1_planning/status");
    pnh_.param("diff_recovery_arrive_radius", diff_recovery_arrive_radius_, 0.18);
    // 前进候选来自UAV0稠密历史轨迹；失败点短时拉黑并换相邻点。只有一组前向
    // 候选均失败时，才沿UAV1自己已经执行的轨迹后退，不再猜UAV0路线上的退让点。
    pnh_.param("diff_failure_retreat_distance", diff_failure_retreat_distance_, 0.40);
    pnh_.param("diff_failure_retreat_arrive_radius",
               diff_failure_retreat_arrive_radius_, 0.10);
    pnh_.param("diff_candidate_blacklist_duration",
               diff_candidate_blacklist_duration_, 4.0);
    pnh_.param("diff_candidate_blacklist_radius",
               diff_candidate_blacklist_radius_, 0.08);
    pnh_.param("diff_forward_failures_before_retreat",
               diff_forward_failures_before_retreat_, 3);
    pnh_.param("diff_clipped_retry_limit", diff_clipped_retry_limit_, 2);
    pnh_.param("follower_history_sample_spacing",
               follower_history_sample_spacing_, 0.08);
    pnh_.param("follower_history_max_length", follower_history_max_length_, 30.0);
    // 狭窄弯道需要让Diff看到障碍另一侧；优先选择前机真实停稳的轨迹终点，
    // 但任一单次规划目标沿前机历史路线不得超过1.5m。
    pnh_.param("diff_history_target_max_distance",
               diff_history_target_max_distance_, 1.50);
    pnh_.param("leader_segment_endpoint_radius",
               leader_segment_endpoint_radius_, 0.20);
    pnh_.param("leader_segment_endpoint_max_speed",
               leader_segment_endpoint_max_speed_, 0.12);
    pnh_.param("leader_segment_endpoint_dwell",
               leader_segment_endpoint_dwell_, 0.25);
    if (!std::isfinite(diff_failure_retreat_distance_) ||
        !std::isfinite(diff_failure_retreat_arrive_radius_) ||
        diff_failure_retreat_distance_ <= diff_failure_retreat_arrive_radius_ ||
        diff_failure_retreat_arrive_radius_ <= 0.0 ||
        !std::isfinite(diff_goal_response_timeout_) ||
        diff_goal_response_timeout_ <= 0.0 ||
        !std::isfinite(diff_candidate_blacklist_duration_) ||
        diff_candidate_blacklist_duration_ <= 0.0 ||
        !std::isfinite(diff_candidate_blacklist_radius_) ||
        diff_candidate_blacklist_radius_ <= 0.0 ||
        diff_forward_failures_before_retreat_ < 1 || diff_clipped_retry_limit_ < 1 ||
        !std::isfinite(follower_history_sample_spacing_) ||
        follower_history_sample_spacing_ <= 0.0 ||
        !std::isfinite(follower_history_max_length_) ||
        follower_history_max_length_ <= diff_failure_retreat_distance_ ||
        !std::isfinite(diff_history_target_max_distance_) ||
        diff_history_target_max_distance_ <= 0.0 ||
        !std::isfinite(leader_segment_endpoint_radius_) ||
        leader_segment_endpoint_radius_ <= 0.0 ||
        !std::isfinite(leader_segment_endpoint_max_speed_) ||
        leader_segment_endpoint_max_speed_ <= 0.0 ||
        !std::isfinite(leader_segment_endpoint_dwell_) ||
        leader_segment_endpoint_dwell_ < 0.0 ||
        !std::isfinite(simple_diff_retry_delay_) || simple_diff_retry_delay_ < 0.0) {
      throw std::runtime_error(
          "leader_safe_path_follower: invalid flexible Diff recovery parameters");
    }
    pnh_.param<std::string>("leader_landing_target_topic", leader_landing_target_topic_,
                            "/UAV0/mission/landing_target");
    pnh_.param<std::string>("leader_landing_request_topic", leader_landing_request_topic_,
                            "/UAV0/mission/landing_request");
    pnh_.param<std::string>("follower_landing_target_topic", follower_landing_target_topic_,
                            "/UAV1/mission/landing_target");
    pnh_.param<std::string>("follower_landing_request_topic", follower_landing_request_topic_,
                            "/UAV1/mission/landing_request");
    pnh_.param<std::string>("follower_landing_trigger_topic", follower_landing_trigger_topic_,
                            "/UAV1/need_to_land");
    pnh_.param<std::string>("follower_assigned_target_topic", follower_assigned_target_topic_,
                            "/UAV1/landing/assigned_target");
    pnh_.param<std::string>("release_uav1_topic", release_uav1_topic_,
                            "/dual_uav_landing/release_uav1");
    // 2026-07-15: 只接收找门模块已经确认并锁存的门平面，禁止把临时候选门发给后机。
    pnh_.param<std::string>("door_pose_topic", door_pose_topic_,
                            "/UAV0/corridor_search/workspace_lock");
    // 2026-07-28: 前机最终锁门后独立发布门心；中途exit_candidate绝不进入后机队列。
    pnh_.param<std::string>("final_exit_pose_topic", final_exit_pose_topic_,
                            "/UAV0/mission/final_exit");
    pnh_.param<std::string>("landing_search_state_topic", landing_search_state_topic_,
                            "/landing_diff_search_manager/state");
    pnh_.param<std::string>("front_scan_anchor_topic", front_scan_anchor_topic_,
                            "/UAV0/landing/front_scan_anchor");
    // 2026-07-16: 接力点使用前机任务命名空间公开，后续实机可直接将该Path桥接给第二架无人机。
    pnh_.param<std::string>("relay_path_topic", relay_path_topic_,
                            "/UAV0/mission/relay_waypoints");
    const std::string alignment_prefix = "/dual_uav_frame_alignment";
    if (!nh_.getParam(alignment_prefix + "/mission_frame", world_frame_) ||
        !nh_.getParam(alignment_prefix + "/follower/x", follower_alignment_x_) ||
        !nh_.getParam(alignment_prefix + "/follower/y", follower_alignment_y_) ||
        !nh_.getParam(alignment_prefix + "/follower/z", follower_alignment_z_) ||
        !nh_.getParam(alignment_prefix + "/follower/yaw_rad", follower_alignment_yaw_)) {
      throw std::runtime_error(
          "leader_safe_path_follower: incomplete /dual_uav_frame_alignment parameters");
    }
    if (world_frame_.empty() || !std::isfinite(follower_alignment_x_) ||
        !std::isfinite(follower_alignment_y_) ||
        !std::isfinite(follower_alignment_z_) ||
        !std::isfinite(follower_alignment_yaw_)) {
      throw std::runtime_error("leader_safe_path_follower: invalid frame alignment");
    }
    follower_alignment_cos_ = std::cos(follower_alignment_yaw_);
    follower_alignment_sin_ = std::sin(follower_alignment_yaw_);
    // 双机普通接力只共享XY路线；后机名义高度与两机起飞悬停高度统一为0.60m。
    pnh_.param("follow_distance", follow_distance_, 1.50);
    pnh_.param("release_path_length", release_path_length_, 0.70);
    pnh_.param("min_separation", min_separation_, 0.70);
    // 航点路径进度只证明前机走过该段；发布前还必须用两机对齐后的实时XY验证水平净距。
    pnh_.param("waypoint_release_min_separation",
               waypoint_release_min_separation_, 0.70);
    pnh_.param("fixed_follow_height", fixed_follow_height_, 0.60);
    pnh_.param("follow_height_min", follow_height_min_, 0.60);
    pnh_.param("follow_height_max", follow_height_max_, 0.70);
    // 前视不足两码后，必须等UAV0真实升高并形成垂直分层，才允许UAV1前往前视锚点等待。
    pnh_.param("down_search_release_height", down_search_release_height_, 1.80);
    pnh_.param("down_search_min_vertical_separation",
               down_search_min_vertical_separation_, 1.00);
    pnh_.param("continuous_follow_before_exit", continuous_follow_before_exit_, true);
    pnh_.param("continuous_follow_speed", continuous_follow_speed_, 0.42);
    // Diff离散轨迹仍需持续检查双机水平间距；该距离只计算XY，不计入高度差。
    pnh_.param("enable_diff_separation_safety", enable_diff_separation_safety_, true);
    pnh_.param<std::string>("leader_task_status_topic", leader_task_status_topic_,
                            "/mission/task_status");
    // 保留后机检测阶段状态发布，便于LDOP运行状态监控。
    pnh_.param<std::string>("follower_detection_enable_topic",
                            follower_detection_enable_topic_,
                            "/UAV1/corridor_search/dynamic_detection_enable");
    // 前机反向时：先在min_separation锁点，继续压缩到recovery阈值后退让，恢复到release阈值再重规划。
    pnh_.param("separation_recovery_distance", separation_recovery_distance_, 0.60);
    pnh_.param("separation_release_distance", separation_release_distance_, 0.80);
    pnh_.param("emergency_retreat_step", emergency_retreat_step_, 0.35);
    pnh_.param("emergency_retreat_speed", emergency_retreat_speed_, 0.30);
    if (!std::isfinite(min_separation_) || !std::isfinite(separation_recovery_distance_) ||
        !std::isfinite(separation_release_distance_) || min_separation_ <= 0.0 ||
        separation_recovery_distance_ <= 0.0 ||
        separation_recovery_distance_ > min_separation_ ||
        separation_release_distance_ < min_separation_) {
      throw std::runtime_error(
          "leader_safe_path_follower: invalid separation safety thresholds");
    }
    // 2026-07-16: 前机锁定终点后，后机在其实飞路线末端后方约0.5m生成独立落点，避免同点降落。
    pnh_.param("terminal_landing_spacing", terminal_landing_spacing_, 0.50);
    pnh_.param("terminal_approach_height", terminal_approach_height_, 0.60);
    pnh_.param("terminal_arrive_radius", terminal_arrive_radius_, 0.25);
    pnh_.param("terminal_arrive_z_tolerance", terminal_arrive_z_tolerance_, 0.15);
    pnh_.param("terminal_arrive_dwell", terminal_arrive_dwell_, 1.0);
    pnh_.param("terminal_approach_speed", terminal_approach_speed_, 0.25);
    pnh_.param("path_sample_spacing", path_sample_spacing_, 0.08);
    pnh_.param("outside_door_distance", outside_door_distance_, 0.50);
    if (!std::isfinite(outside_door_distance_) || outside_door_distance_ <= 0.0) {
      throw std::runtime_error(
          "leader_safe_path_follower: invalid outside-door parameters");
    }
    pnh_.param("max_route_length", max_route_length_, 60.0);
    // 2026-07-21: 雷达里程计yaw可能不跟随实际转弯，接力判向改用最近实飞路线的局部切线；
    // 沿新走廊转弯可继续发点，沿路线倒退或重新进入旧路线仍暂停记录。
    pnh_.param("forward_projection_ratio", forward_projection_ratio_, -0.20);
    pnh_.param("route_direction_window", route_direction_window_, 0.45);
    pnh_.param("backtrack_pause_distance", backtrack_pause_distance_, 0.60);
    pnh_.param("route_revisit_radius", route_revisit_radius_, 0.45);
    pnh_.param("route_revisit_progress_gap", route_revisit_progress_gap_, 1.00);
    // 2026-07-20: 恢复半径覆盖0.1m里程计栅格和转弯横向误差；只有真实持续后退才会进入该状态。
    pnh_.param("route_resume_radius", route_resume_radius_, 0.45);
    // 2026-07-21: 反向投影先缓存；缓存轨迹离开全部旧路线后按新转弯分支接回，避免U形换路永久锁死接力发布。
    pnh_.param("turn_branch_confirm_distance", turn_branch_confirm_distance_, 0.50);
    pnh_.param("max_target_step", max_target_step_, 0.55);
    // 2026-07-15: 后机按前机历史折线逐段前视，不能从当前位置直连远端滞后点切过弯道墙体。
    pnh_.param("route_tracking_lookahead", route_tracking_lookahead_, 0.30);
    if (diff_history_target_max_distance_ <=
        std::max(0.18, route_tracking_lookahead_)) {
      throw std::runtime_error(
          "leader_safe_path_follower: diff history target maximum must exceed lookahead");
    }
    pnh_.param("cruise_speed", cruise_speed_, 0.35);
    pnh_.param("max_vertical_speed", max_vertical_speed_, 0.20);
    pnh_.param("odom_timeout", odom_timeout_, 0.50);
    // 跨机高频里程计发生网络积压时禁止用历史位置释放后机。
    pnh_.param("leader_odom_max_transport_age",
               leader_odom_max_transport_age_, 0.50);
    pnh_.param("cloud_timeout", cloud_timeout_, 0.60);
    // 不得高于leader_start_height，否则双机已达到开始门槛后仍无法形成可接力历史轨迹。
    pnh_.param("min_record_height", min_record_height_, 0.30);
    pnh_.param("obstacle_check_enabled", obstacle_check_enabled_, true);
    pnh_.param("require_fresh_cloud", require_fresh_cloud_, true);
    pnh_.param("obstacle_radius", obstacle_radius_, 0.28);
    pnh_.param("obstacle_z_margin", obstacle_z_margin_, 0.20);
    pnh_.param("obstacle_ignore_near", obstacle_ignore_near_, 0.18);
    pnh_.param("obstacle_min_points", obstacle_min_points_, 3);
    // 点云稀疏时用结构化LDOP状态补充后机避障；摆球短暂漏检仍有限保留。
    pnh_.param<std::string>("dynamic_obstacle_topic", dynamic_obstacle_topic_,
                            "/UAV1/ldop/dynamic_objects");
    pnh_.param("dynamic_obstacle_retention", dynamic_obstacle_retention_, 0.80);
    pnh_.param("dynamic_obstacle_safety_radius", dynamic_obstacle_safety_radius_, 0.35);
    pnh_.param("dynamic_obstacle_z_margin", dynamic_obstacle_z_margin_, 0.18);
    pnh_.param("enable_dynamic_obstacle_detection",
               enable_dynamic_obstacle_detection_, false);
    // 2026-07-28: 比赛动态物只在通道中部往复；墙边框不进入跨帧保留，预测越出中心安全带也不参与阻挡。
    pnh_.param("dynamic_retention_route_half_width", dynamic_retention_route_half_width_, 0.70);
    // 2026-07-28: 0.5m/s指令下FAST-LIO若出现数m/s跳变，立即请求控制器用MAVROS坐标锁点，禁止错误目标继续外推。
    pnh_.param("follower_odom_jump_speed", follower_odom_jump_speed_, 2.0);
    pnh_.param("follower_odom_jump_vertical_speed", follower_odom_jump_vertical_speed_, 1.2);
    pnh_.param("follower_odom_jump_confirm_samples",
                follower_odom_jump_confirm_samples_, 3);
    if (follower_odom_jump_confirm_samples_ < 1) {
      throw std::runtime_error(
          "leader_safe_path_follower: follower_odom_jump_confirm_samples must be >= 1");
    }
    // 2026-07-27: 持续有运动指令但机体1.5s内位移不足6cm时进入点云选向脱困，防止贴墙后永久推杆。
    pnh_.param("stuck_detection_timeout", stuck_detection_timeout_, 1.50);
    pnh_.param("stuck_min_progress", stuck_min_progress_, 0.06);
    pnh_.param("recovery_step", recovery_step_, 0.35);
    pnh_.param("recovery_speed", recovery_speed_, 0.20);
    pnh_.param("recovery_attempt_timeout", recovery_attempt_timeout_, 1.50);
    pnh_.param("recovery_success_distance", recovery_success_distance_, 0.12);
    pnh_.param("recovery_near_ignore", recovery_near_ignore_, 0.06);
    // 点云阻挡HOLD也必须能脱困，但当前缓存点未到达前禁止跳到下一点。
    pnh_.param("blocked_recovery_timeout", blocked_recovery_timeout_, 1.00);
    // 2026-07-16: 后机采用门点+滚动内部点+真实终点的任务接力；0表示内部点数量不限。
    pnh_.param("relay_release_distance", relay_release_distance_, 0.70);
    // 门点和内部点都等前机沿路线清空0.70m，再叠加实时双机0.70m净距门槛。
    pnh_.param("door_release_inside_distance", door_release_inside_distance_, 0.70);
    pnh_.param("relay_waypoint_spacing", relay_waypoint_spacing_, 2.50);
    pnh_.param("relay_arrive_radius", relay_arrive_radius_, 0.25);
    pnh_.param("relay_arrive_z_tolerance", relay_arrive_z_tolerance_, 0.20);
    // 最终降落点保留停驻净空；普通接力点另用小体素检查，不能让侧墙否决狭窄通道。
    pnh_.param("relay_endpoint_clearance_radius", relay_endpoint_clearance_radius_, 0.38);
    pnh_.param("relay_endpoint_z_margin", relay_endpoint_z_margin_, 0.28);
    pnh_.param("relay_endpoint_min_points", relay_endpoint_min_points_, 2);
    // 普通接力点只检查落点本身的小体素；侧墙进入大净空圆柱不能否决整段跟随。
    pnh_.param("relay_point_occupied_radius", relay_point_occupied_radius_, 0.12);
    pnh_.param("relay_point_occupied_z_margin", relay_point_occupied_z_margin_, 0.18);
    pnh_.param("relay_point_occupied_min_points", relay_point_occupied_min_points_, 2);
    pnh_.param("relay_point_check_distance", relay_point_check_distance_, 0.45);
    pnh_.param("relay_occupied_attachment_radius", relay_occupied_attachment_radius_, 0.35);
    // 可选的Diff目标点云净空预检查；关闭时目标直接交由Diff Planner进行轨迹碰撞检查。
    pnh_.param("enable_relay_goal_clearance", enable_relay_goal_clearance_, false);
    pnh_.param("relay_goal_clearance_radius", relay_goal_clearance_radius_, 0.20);
    pnh_.param("relay_goal_clearance_z_margin", relay_goal_clearance_z_margin_, 0.30);
    pnh_.param("relay_goal_clearance_min_points", relay_goal_clearance_min_points_, 2);
    pnh_.param("relay_goal_backtrack_max_distance",
                relay_goal_backtrack_max_distance_, 1.00);
    pnh_.param("relay_slowdown_radius", relay_slowdown_radius_, 0.55);
    pnh_.param("relay_approach_speed", relay_approach_speed_, 0.20);
    pnh_.param("relay_arrive_max_horizontal_speed", relay_arrive_max_horizontal_speed_, 0.10);
    pnh_.param("relay_arrive_max_vertical_speed", relay_arrive_max_vertical_speed_, 0.08);
    pnh_.param("relay_arrive_dwell", relay_arrive_dwell_, 0.50);
    // 2026-07-28: Diff末端速度差分可能尚未降到严格门槛；小半径内先锁点，再用停驻时间完成接力点。
    pnh_.param("diff_endpoint_capture_radius", diff_endpoint_capture_radius_, 0.15);
    pnh_.param("diff_endpoint_capture_dwell", diff_endpoint_capture_dwell_, 0.45);
    // Diff可能因局部占据裁短轨迹终点；裁短量超过该值时只能视作中间步，不能消费接力点。
    pnh_.param("diff_accepted_goal_tolerance", diff_accepted_goal_tolerance_, 0.20);
    if (!std::isfinite(diff_accepted_goal_tolerance_) ||
        diff_accepted_goal_tolerance_ < diff_endpoint_capture_radius_) {
      throw std::runtime_error(
          "leader_safe_path_follower: diff_accepted_goal_tolerance must be >= diff_endpoint_capture_radius");
    }
    // 2026-07-28: 直接监测UAV1 PositionCommand输出；轨迹结束但未到点时允许重新下发当前目标。
    pnh_.param("diff_command_stale_timeout", diff_command_stale_timeout_, 0.80);
    pnh_.param("max_internal_relay_points", max_internal_relay_points_, 0);

    // 安全门槛只需要最新位置。队列保留一帧并关闭 Nagle，避免网络恢复后依次回放旧坐标。
    leader_odom_sub_ = nh_.subscribe(
        leader_odom_topic_, 1, &LeaderSafePathFollower::leaderOdomCallback, this,
        ros::TransportHints().tcpNoDelay());
    leader_history_path_sub_ = nh_.subscribe(
        leader_history_path_topic_, 1,
        &LeaderSafePathFollower::leaderHistoryPathCallback, this,
        ros::TransportHints().tcpNoDelay());
    leader_trajectory_sub_ = nh_.subscribe(
        leader_trajectory_topic_, 2,
        &LeaderSafePathFollower::leaderTrajectoryCallback, this,
        ros::TransportHints().tcpNoDelay());
    follower_odom_sub_ = nh_.subscribe(follower_odom_topic_, 20,
                                       &LeaderSafePathFollower::followerOdomCallback, this);
    follower_cloud_sub_ = nh_.subscribe(follower_cloud_topic_, 1,
                                        &LeaderSafePathFollower::cloudCallback, this);
    if (enable_dynamic_obstacle_detection_) {
      dynamic_obstacle_sub_ = nh_.subscribe(
          dynamic_obstacle_topic_, 5,
          &LeaderSafePathFollower::dynamicObstacleCallback, this);
    }
    leader_landing_target_sub_ = nh_.subscribe(
        leader_landing_target_topic_, 2,
        &LeaderSafePathFollower::leaderLandingTargetCallback, this);
    leader_landing_request_sub_ = nh_.subscribe(
        leader_landing_request_topic_, 2,
        &LeaderSafePathFollower::leaderLandingRequestCallback, this);
    release_uav1_sub_ = nh_.subscribe(
        release_uav1_topic_, 1, &LeaderSafePathFollower::releaseUav1Callback, this);
    follower_assigned_target_sub_ = nh_.subscribe(
        follower_assigned_target_topic_, 1,
        &LeaderSafePathFollower::followerAssignedTargetCallback, this);
    follower_landing_trigger_sub_ = nh_.subscribe(
        follower_landing_trigger_topic_, 2,
        &LeaderSafePathFollower::followerLandingTriggerCallback, this);
    door_pose_sub_ = nh_.subscribe(door_pose_topic_, 1,
                                   &LeaderSafePathFollower::doorPoseCallback, this);
    final_exit_pose_sub_ = nh_.subscribe(
        final_exit_pose_topic_, 1,
        &LeaderSafePathFollower::finalExitPoseCallback, this);
    landing_search_state_sub_ = nh_.subscribe(
        landing_search_state_topic_, 2,
        &LeaderSafePathFollower::landingSearchStateCallback, this);
    front_scan_anchor_sub_ = nh_.subscribe(
        front_scan_anchor_topic_, 1,
        &LeaderSafePathFollower::frontScanAnchorCallback, this);
    leader_task_status_sub_ = nh_.subscribe(
        leader_task_status_topic_, 2,
        &LeaderSafePathFollower::leaderTaskStatusCallback, this);
    // 2026-07-28: 目标publish不等于轨迹生成；使用Diff反馈触发已验证路线子目标。
    diff_status_sub_ = nh_.subscribe(diff_status_topic_, 10,
                                     &LeaderSafePathFollower::diffStatusCallback, this);
    // 2026-07-28: 只订阅不发布Diff输出，用于区分“曾规划成功”和“当前轨迹仍然存活”。
    if (use_diff_planner_)
      diff_command_sub_ = nh_.subscribe(command_topic_, 20,
                                        &LeaderSafePathFollower::diffCommandCallback, this);
    // 2026-07-28: Diff模式禁止跟随器成为/UAV1/planning/pos_cmd的第二发布者；目标使用锁存发布避免启动时序丢包。
    if (!use_diff_planner_) {
      command_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(command_topic_, 10);
      traj_started_pub_ = nh_.advertise<std_msgs::Empty>(traj_started_topic_, 1, true);
    }
    diff_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(diff_goal_topic_, 1, true);
    follower_landing_target_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(follower_landing_target_topic_, 1, true);
    follower_landing_request_pub_ =
        nh_.advertise<std_msgs::Bool>(follower_landing_request_topic_, 1, true);
    follower_detection_enable_pub_ =
        nh_.advertise<std_msgs::Bool>(follower_detection_enable_topic_, 2, true);
    follower_safety_hold_pub_ =
        nh_.advertise<std_msgs::Bool>("/UAV1/planning/safety_hold", 1, true);
    route_pub_ = pnh_.advertise<nav_msgs::Path>("leader_safe_route", 1, true);
    relay_path_pub_ = nh_.advertise<nav_msgs::Path>(relay_path_topic_, 1, true);
    target_pub_ = pnh_.advertise<visualization_msgs::Marker>("target_marker", 1);
    state_pub_ = pnh_.advertise<visualization_msgs::Marker>("state_marker", 1);
    timer_ = nh_.createTimer(ros::Duration(0.05), &LeaderSafePathFollower::timerCallback, this);
    setFollowerDetectionEnable(false, "initialization", true);

    ROS_INFO("[safe_follower] relay ready: executor=%s continuous_before_exit=%d follow=%.2fm, "
             "door + rolling internal + terminal, "
             "internal_limit=%d (0=unlimited), "
             "release=%.2fm actual_separation_gate=%.2fm spacing=%.2fm "
             "diff_separation_safety=%d hold=%.2fm retreat=%.2fm recover=%.2fm "
             "alignment=(%.2f,%.2f,%.2f, yaw=%.3f)",
             use_diff_planner_ ? "UAV1_DIFF" : "LEGACY_POSITION_COMMAND",
             static_cast<int>(continuous_follow_before_exit_), follow_distance_,
             max_internal_relay_points_, relay_release_distance_,
             waypoint_release_min_separation_, relay_waypoint_spacing_,
             static_cast<int>(enable_diff_separation_safety_), min_separation_,
             separation_recovery_distance_, separation_release_distance_,
             follower_alignment_x_, follower_alignment_y_, follower_alignment_z_,
             follower_alignment_yaw_);
  }

 private:
  // UAV1检测阶段状态锁存发布，供运行监控和后续LDOP门控扩展。
  void setFollowerDetectionEnable(bool active, const char* reason, bool force = false) {
    if (!force && follower_detection_enabled_ == active) return;
    follower_detection_enabled_ = active;
    std_msgs::Bool msg;
    msg.data = active;
    follower_detection_enable_pub_.publish(msg);
    ROS_WARN("[safe_follower] UAV1 dynamic detection %s reason=%s.",
             active ? "ENABLED" : "DISABLED", reason);
  }

  geometry_msgs::Point leaderToWorld(const geometry_msgs::Point& local) const {
    return local;
  }

  geometry_msgs::Point followerToWorld(const geometry_msgs::Point& local) const {
    geometry_msgs::Point world;
    world.x = follower_alignment_cos_ * local.x - follower_alignment_sin_ * local.y +
              follower_alignment_x_;
    world.y = follower_alignment_sin_ * local.x + follower_alignment_cos_ * local.y +
              follower_alignment_y_;
    world.z = local.z + follower_alignment_z_;
    return world;
  }

  geometry_msgs::Point worldToFollower(const geometry_msgs::Point& world) const {
    const double dx = world.x - follower_alignment_x_;
    const double dy = world.y - follower_alignment_y_;
    geometry_msgs::Point local;
    local.x = follower_alignment_cos_ * dx + follower_alignment_sin_ * dy;
    local.y = -follower_alignment_sin_ * dx + follower_alignment_cos_ * dy;
    local.z = world.z - follower_alignment_z_;
    return local;
  }

  double worldYawToFollower(double world_yaw) const {
    return std::atan2(std::sin(world_yaw - follower_alignment_yaw_),
                      std::cos(world_yaw - follower_alignment_yaw_));
  }

  bool relayWaypointSeparationReady(const char* label) const {
    if (!have_leader_odom_ || !have_follower_odom_) {
      ROS_WARN_THROTTLE(1.0,
                        "[safe_follower] HOLD %s waypoint release: dual odometry unavailable.",
                        label);
      return false;
    }
    if (!leaderOdomFresh(ros::Time::now())) {
      ROS_WARN_THROTTLE(1.0,
                        "[safe_follower] HOLD %s waypoint release: leader odometry stale.",
                        label);
      return false;
    }
    const geometry_msgs::Point leader_world =
        leaderToWorld(leader_odom_.pose.pose.position);
    const geometry_msgs::Point follower_world =
        followerToWorld(follower_odom_.pose.pose.position);
    const double separation = std::hypot(leader_world.x - follower_world.x,
                                         leader_world.y - follower_world.y);
    if (separation + 1e-6 < waypoint_release_min_separation_) {
      ROS_WARN_THROTTLE(
          0.5,
          "[safe_follower] HOLD %s waypoint release: actual UAV0-UAV1 XY separation "
          "%.2fm < %.2fm.",
          label, separation, waypoint_release_min_separation_);
      return false;
    }
    return true;
  }

  // 2026-07-24: 前机里程计z不再通过接力路线传给后机；普通任务点只复用XY，
  // 后机在自身局部坐标系使用固定巡航高度，终点下降仍由专用terminal高度控制。
  geometry_msgs::Point useFollowerCruiseHeight(const geometry_msgs::Point& local) const {
    geometry_msgs::Point adjusted = local;
    adjusted.z = std::max(follow_height_min_,
                          std::min(follow_height_max_, fixed_follow_height_));
    return adjusted;
  }

  geometry_msgs::Point followerCruisePointToWorld(
      const geometry_msgs::Point& world_xy) const {
    geometry_msgs::Point local = worldToFollower(world_xy);
    local = useFollowerCruiseHeight(local);
    return followerToWorld(local);
  }

  bool leaderOdomFresh(const ros::Time& now) const {
    return have_leader_odom_ && !leader_odom_stamp_.isZero() &&
           (now - leader_odom_stamp_).toSec() <= odom_timeout_;
  }

  void doorPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (have_confirmed_door_) return;
    // 2026-07-15: workspace_lock 的朝向由起飞区指向作业区；保存一次后不允许后续重复消息拖动门点。
    confirmed_door_.position = followerCruisePointToWorld(msg->pose.position);
    confirmed_door_.yaw = yawFromQuaternion(msg->pose.orientation);
    have_confirmed_door_ = true;
    ROS_ERROR("[safe_follower] confirmed DOOR received at (%.2f, %.2f, %.2f), yaw=%.1fdeg; "
              "follower remains parked until leader is %.2fm inside.",
              confirmed_door_.position.x, confirmed_door_.position.y,
              confirmed_door_.position.z, confirmed_door_.yaw * 180.0 / M_PI,
              relay_release_distance_);
  }

  bool tryReleaseDoorWaypointFromLeaderPosition(
      const geometry_msgs::Point& leader_world) {
    if (!have_confirmed_door_ || door_waypoint_released_ ||
        terminal_mode_active_)
      return false;
    const double inside_progress =
        std::cos(confirmed_door_.yaw) *
            (leader_world.x - confirmed_door_.position.x) +
        std::sin(confirmed_door_.yaw) *
            (leader_world.y - confirmed_door_.position.y);
    if (inside_progress < door_release_inside_distance_) return false;
    if (!relayWaypointSeparationReady("DOOR")) return false;

    // 门的放行只依赖前机真实越过门心；历史路线的反向/断点保护仍只约束普通接力点。
    confirmed_door_.progress = nearestRouteProgress(confirmed_door_.position);
    appendRelayWaypoint(confirmed_door_, "DOOR");
    door_waypoint_released_ = true;
    setFollowerDetectionEnable(true, "door waypoint released");
    last_relay_selection_progress_ = confirmed_door_.progress;
    return true;
  }

  // 2026-07-28: 复用后文已有的nearestRouteProgress确定出口在前机实飞折线上的顺序；
  // 像入口一样在前机清空门后再释放出口门心，此前内部点保持原顺序，之后停止新增内部点。
  void tryReleaseFinalExitWaypoint(const char* reason) {
    if (!have_final_exit_ || !leader_outside_exit_ || exit_waypoint_released_ ||
        terminal_mode_active_)
      return;
    if (!leaderOdomFresh(ros::Time::now())) return;
    if (!relayWaypointSeparationReady("EXIT")) return;
    pending_relay_valid_ = false;
    confirmed_exit_.progress = nearestRouteProgress(confirmed_exit_.position);
    exit_waypoint_index_ = relay_waypoints_.size();
    appendRelayWaypoint(confirmed_exit_, "EXIT");

    RoutePoint outside_wait = confirmed_exit_;
    outside_wait.position.x += outside_door_distance_ * std::cos(confirmed_exit_.yaw);
    outside_wait.position.y += outside_door_distance_ * std::sin(confirmed_exit_.yaw);
    outside_wait.position = followerCruisePointToWorld(outside_wait.position);
    outside_wait.progress = confirmed_exit_.progress + outside_door_distance_;
    outside_wait_waypoint_index_ = relay_waypoints_.size();
    appendRelayWaypoint(outside_wait, "OUTSIDE_WAIT");
    exit_waypoint_released_ = true;
    outside_wait_waypoint_released_ = true;
    ROS_ERROR("[safe_follower] FINAL EXIT and OUTSIDE_WAIT queued as relays %zu/%zu; "
              "outside target is %.2fm along exit yaw, reason=%s.",
              exit_waypoint_index_ + 1, outside_wait_waypoint_index_ + 1,
              outside_door_distance_, reason);
  }

  void finalExitPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!enable_search_landing_) return;
    if (have_final_exit_) return;
    confirmed_exit_.position = followerCruisePointToWorld(msg->pose.position);
    confirmed_exit_.yaw = yawFromQuaternion(msg->pose.orientation);
    have_final_exit_ = true;
    ROS_ERROR("[safe_follower] FINAL EXIT received center=(%.2f, %.2f, %.2f) "
              "yaw=%.1fdeg; append it to the normal history route, then hover %.2fm outside.",
              confirmed_exit_.position.x, confirmed_exit_.position.y,
              confirmed_exit_.position.z, confirmed_exit_.yaw * 180.0 / M_PI,
              outside_door_distance_);
    tryReleaseFinalExitWaypoint("final exit received after leader outside");
  }

  void landingSearchStateCallback(const std_msgs::String::ConstPtr& msg) {
    if (!enable_search_landing_ || down_search_wait_requested_) return;
    const std::string& state = msg->data;
    const bool front_scan_fell_back_to_down =
        state == "FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP" ||
        state == "FRONT_ARUCO_HINT_DIFF_APPROACH" ||
        state == "FRONT_ARUCO_HINT_RETURN_COMPLETE_APPROACH" ||
        state == "FRONT_HINT_TIMEOUT_FALLBACK_DOWN_SWEEP";
    if (!front_scan_fell_back_to_down) return;
    down_search_wait_requested_ = true;
    ROS_ERROR("[safe_follower] UAV0 front search did not complete two-code assignment; "
              "wait for measured climb before releasing UAV1 to the front-search anchor. "
              "state=%s",
              state.c_str());
  }

  void frontScanAnchorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!enable_search_landing_) return;
    front_scan_anchor_.position = followerCruisePointToWorld(msg->pose.position);
    front_scan_anchor_.yaw = yawFromQuaternion(msg->pose.orientation);
    have_front_scan_anchor_ = true;
    ROS_ERROR("[safe_follower] UAV0 front-scan anchor received at "
              "(%.2f, %.2f, %.2f), yaw=%.1fdeg.",
              front_scan_anchor_.position.x, front_scan_anchor_.position.y,
              front_scan_anchor_.position.z,
              front_scan_anchor_.yaw * 180.0 / M_PI);
  }

  void tryReleaseFrontSearchWaitWaypoint() {
    if (!down_search_wait_requested_ || outside_wait_waypoint_released_ ||
        release_uav1_ || terminal_mode_active_ || !leader_outside_exit_) {
      return;
    }
    if (!have_front_scan_anchor_ || !have_leader_odom_ || !have_follower_odom_) {
      ROS_WARN_THROTTLE(1.0,
                        "[safe_follower] HOLD outside wait release: front-scan "
                        "anchor/odometry unavailable.");
      return;
    }
    const geometry_msgs::Point leader_world =
        leaderToWorld(leader_odom_.pose.pose.position);
    const double vertical_separation = leader_world.z - fixed_follow_height_;
    if (leader_world.z + 1e-6 < down_search_release_height_ ||
        vertical_separation + 1e-6 < down_search_min_vertical_separation_) {
      ROS_WARN_THROTTLE(
          0.5,
          "[safe_follower] HOLD outside wait release: UAV0 z=%.2fm, required %.2fm; "
          "vertical separation=%.2fm, required %.2fm.",
          leader_world.z, down_search_release_height_, vertical_separation,
          down_search_min_vertical_separation_);
      return;
    }
    if (!relayWaypointSeparationReady("OUTSIDE_WAIT")) return;

    pending_relay_valid_ = false;
    front_scan_anchor_.progress = nearestRouteProgress(front_scan_anchor_.position);
    exit_waypoint_index_ = relay_waypoints_.size();
    appendRelayWaypoint(front_scan_anchor_, "OUTSIDE_WAIT");
    exit_waypoint_released_ = true;
    outside_wait_waypoint_released_ = true;
    diff_goal_published_ = false;
    ROS_ERROR("[safe_follower] RELEASE OUTSIDE_WAIT after UAV0 entered down search: "
              "target=(%.2f, %.2f, %.2f), UAV0_z=%.2f, vertical_separation=%.2f.",
              front_scan_anchor_.position.x, front_scan_anchor_.position.y,
              fixed_follow_height_, leader_world.z, vertical_separation);
  }

  void publishRelayPath() {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = world_frame_;
    for (const RoutePoint& point : relay_waypoints_) {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = point.position;
      pose.pose.orientation.w = std::cos(point.yaw * 0.5);
      pose.pose.orientation.z = std::sin(point.yaw * 0.5);
      path.poses.push_back(pose);
    }
    relay_path_pub_.publish(path);
  }

  void appendRelayWaypoint(const RoutePoint& point, const char* label) {
    RoutePoint follower_point = point;
    // 2026-07-24: 对外Path中的普通门点/接力点也明确写成后机0.65m高度，
    // 不再把前机可能退化的z伪装成需要后机执行的三维坐标。
    if (std::string(label) != "TERMINAL")
      follower_point.position = followerCruisePointToWorld(point.position);
    relay_waypoints_.push_back(follower_point);
    publishRelayPath();
    // 2026-07-28: RELEASE同时输出消费索引和Diff门控状态，现场可直接判断新点为何尚未SEND。
    const double active_distance =
        have_follower_odom_ && active_relay_index_ < relay_waypoints_.size()
            ? distance3d(followerToWorld(follower_odom_.pose.pose.position),
                         relay_waypoints_[active_relay_index_].position)
            : -1.0;
    ROS_ERROR("[safe_follower] RELEASE %s waypoint sequence=%zu at (%.2f, %.2f, %.2f); "
              "active=%zu diff_goal=%zu published=%d response=%d command_live=%d "
              "active_distance=%.2fm speed=%.2f/%.2f.",
              label, relay_waypoints_.size(), follower_point.position.x,
              follower_point.position.y, follower_point.position.z,
              active_relay_index_ + 1,
              diff_goal_index_ == std::numeric_limits<std::size_t>::max()
                  ? 0 : diff_goal_index_ + 1,
              static_cast<int>(diff_goal_published_),
              static_cast<int>(diff_plan_response_received_),
              static_cast<int>(diff_command_seen_for_goal_ &&
                  (ros::Time::now() - diff_command_stamp_).toSec() <=
                      diff_command_stale_timeout_),
              active_distance, follower_horizontal_speed_, follower_vertical_speed_);
  }

  void leaderLandingTargetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 2026-07-14: 保存前机终点作为诊断参考；后机实际落点仍从已飞安全轨迹后退取点。
    leader_landing_target_ = *msg;
    have_leader_landing_target_ = true;
  }

  bool getRouteTargetBehindEnd(double distance, RoutePoint* target) const {
    if (route_.empty()) return false;
    double remaining = std::max(0.0, distance);
    for (std::size_t i = route_.size() - 1; i > 0; --i) {
      const RoutePoint& newer = route_[i];
      const RoutePoint& older = route_[i - 1];
      const double segment = distance3d(newer.position, older.position);
      if (segment >= remaining && segment > 1e-6) {
        const double ratio = remaining / segment;
        target->position = interpolate(newer.position, older.position, ratio);
        target->yaw = newer.yaw;
        // 2026-07-16: 终点接力也必须携带历史轨迹进度，否则点间跟踪会把终点误判为进度0。
        target->progress = newer.progress + ratio * (older.progress - newer.progress);
        return true;
      }
      remaining -= segment;
    }
    *target = route_.front();
    return true;
  }

  void leaderLandingRequestCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!enable_search_landing_) return;
    if (!msg->data) return;
    pending_leader_landing_request_ = true;
    tryQueueFollowerTerminalTarget();
  }

  void tryQueueFollowerTerminalTarget() {
    if (terminal_mode_active_) return;
    if (!release_uav1_ || !outside_wait_arrived_ ||
        !have_assigned_follower_target_) {
      ROS_INFO_THROTTLE(
          1.0,
          "[safe_follower] terminal gate: outside_arrived=%d release=%d assigned=%d.",
          static_cast<int>(outside_wait_arrived_), static_cast<int>(release_uav1_),
          static_cast<int>(have_assigned_follower_target_));
      return;
    }
    pending_leader_landing_request_ = false;
    queueFollowerTerminalTarget();
  }

  void queueFollowerTerminalTarget() {
    if (!have_assigned_follower_target_) return;
    RoutePoint nearby_target = assigned_follower_target_;

    // 2026-07-14: 终点接管时锁定一次落点，后续前机下降产生的里程计变化不能拖动后机目标。
    terminal_target_world_ = nearby_target;
    terminal_target_world_.position.z = terminal_approach_height_;
    terminal_mode_active_ = true;
    terminal_arrival_stamp_ = ros::Time(0);

    // 2026-07-16: 终点只由真实降落请求触发，并排在全部已发布接力点之后，不能越过队列直冲终点。
    pending_relay_valid_ = false;
    terminal_waypoint_index_ = relay_waypoints_.size();
    appendRelayWaypoint(terminal_target_world_, "TERMINAL");

    geometry_msgs::PoseStamped target_msg;
    target_msg.header.stamp = ros::Time::now();
    target_msg.header.frame_id = world_frame_;
    target_msg.pose.position = terminal_target_world_.position;
    target_msg.pose.orientation.w = 1.0;
    follower_landing_target_pub_.publish(target_msg);
    publishTarget(terminal_target_world_.position);
    ROS_ERROR("[safe_follower] TERMINAL queued: follower landing target=(%.2f, %.2f, %.2f), "
              "execute after earlier relay points; leader spacing check disabled.",
              terminal_target_world_.position.x, terminal_target_world_.position.y,
              terminal_target_world_.position.z);
  }

  void followerAssignedTargetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!enable_search_landing_) return;
    assigned_follower_target_.position = msg->pose.position;
    assigned_follower_target_.yaw = yawFromQuaternion(msg->pose.orientation);
    have_assigned_follower_target_ = true;
    ROS_INFO("[safe_follower] assigned UAV1 landing target received (%.2f, %.2f, %.2f).",
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    tryQueueFollowerTerminalTarget();
  }

  void followerLandingTriggerCallback(const std_msgs::Bool::ConstPtr& msg) {
    // 精降控制权一旦接管就永久锁存到本次任务结束。停止继续重发终端 Diff
    // 航点；实际 MAVROS 输出同时由 landing_setpoint_arbiter 切给精降节点。
    if (!msg->data || follower_precision_landing_active_) return;
    follower_precision_landing_active_ = true;
    diff_goal_published_ = false;
    diff_plan_response_received_ = false;
    setFollowerDetectionEnable(false, "precision landing takeover");
    ROS_ERROR("[safe_follower] UAV1 precision landing takeover active; "
              "stop issuing relay/Diff goals.");
  }

  void releaseUav1Callback(const std_msgs::Bool::ConstPtr& msg) {
    if (!enable_search_landing_) return;
    if (!msg->data || release_uav1_) return;
    release_uav1_ = true;
    ROS_ERROR("[safe_follower] UAV0 reached its landing target; latch UAV1 release, "
              "but keep waiting until UAV1 is stable outside the door.");
    tryQueueFollowerTerminalTarget();
  }

  // 2026-07-21: 从最近一段已认可实飞轨迹估计局部前进方向，避免使用与雷达机体框不一致的yaw。
  bool getRouteForwardDirection(double* direction_x, double* direction_y) const {
    if (route_.size() < 2) return false;
    const geometry_msgs::Point& end = route_.back().position;
    double accumulated = 0.0;
    std::size_t start_index = route_.size() - 2;
    for (std::size_t i = route_.size() - 1; i > 0; --i) {
      accumulated += distance3d(route_[i].position, route_[i - 1].position);
      start_index = i - 1;
      if (accumulated >= route_direction_window_) break;
    }
    const double dx = end.x - route_[start_index].position.x;
    const double dy = end.y - route_[start_index].position.y;
    const double norm = std::hypot(dx, dy);
    if (norm < 1e-3) return false;
    *direction_x = dx / norm;
    *direction_y = dy / norm;
    return true;
  }

  // 2026-07-21: 用全部已认可路线判定候选是否仍在走回头路；新分支必须真正离开旧路线安全带。
  double distanceToAcceptedRoute(const geometry_msgs::Point& position) const {
    double nearest = std::numeric_limits<double>::infinity();
    for (const RoutePoint& old_point : route_) {
      nearest = std::min(nearest,
                         std::hypot(position.x - old_point.position.x,
                                    position.y - old_point.position.y));
    }
    return nearest;
  }

  // 2026-07-21: 所有正常采样和确认后的转弯缓存统一从这里追加，保证progress、长度和裁剪同步。
  bool appendAcceptedRoutePoint(RoutePoint* point) {
    if (!route_.empty()) {
      const double segment = distance3d(route_.back().position, point->position);
      if (segment < path_sample_spacing_) return false;
      route_length_ += segment;
      leader_route_progress_ += segment;
    }
    point->progress = leader_route_progress_;
    route_.push_back(*point);
    while (route_.size() > 2 && route_length_ > max_route_length_) {
      route_length_ -= distance3d(route_[0].position, route_[1].position);
      route_.pop_front();
      if (follower_route_index_ > 0) --follower_route_index_;
    }
    while (!leader_segment_endpoints_.empty() &&
           leader_segment_endpoints_.front().progress + path_sample_spacing_ <
               route_.front().progress) {
      leader_segment_endpoints_.pop_front();
    }
    return true;
  }

  void leaderHistoryPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    if (msg->poses.empty()) return;

    std::vector<RoutePoint> samples;
    samples.reserve(msg->poses.size());
    for (const geometry_msgs::PoseStamped& pose : msg->poses) {
      const geometry_msgs::Point& local = pose.pose.position;
      if (!std::isfinite(local.x) || !std::isfinite(local.y) ||
          !std::isfinite(local.z) || local.z < min_record_height_) {
        continue;
      }
      RoutePoint sample;
      sample.position = followerCruisePointToWorld(leaderToWorld(local));
      sample.yaw = yawFromQuaternion(pose.pose.orientation);
      samples.push_back(sample);
    }
    if (samples.empty()) return;

    std::size_t start_index = 0;
    if (!route_.empty()) {
      std::size_t anchor_index = 0;
      double anchor_distance = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < samples.size(); ++i) {
        const double distance = distance3d(route_.back().position,
                                           samples[i].position);
        if (distance < anchor_distance) {
          anchor_distance = distance;
          anchor_index = i;
        }
      }
      start_index = anchor_index + 1;
    } else {
      leader_started_ = true;
    }

    std::size_t appended = 0;
    for (std::size_t i = start_index; i < samples.size(); ++i) {
      RoutePoint sample = samples[i];
      if (appendAcceptedRoutePoint(&sample)) ++appended;
    }

    if (appended == 0 || route_.empty()) return;
    last_leader_sample_ = route_.back();
    have_last_leader_sample_ = true;
    relay_route_paused_ = false;
    turn_candidate_route_.clear();
    turn_candidate_length_ = 0.0;
    publishRoute(msg->header.stamp.isZero() ? ros::Time::now()
                                             : msg->header.stamp);
    ROS_WARN("[safe_follower] recovered %zu UAV0 history samples; "
             "cached route remains %.2fm long.",
             appended, route_length_);
  }

  void leaderTrajectoryCallback(const bspline::Bspline::ConstPtr& msg) {
    if (msg->traj_id == last_leader_trajectory_id_ &&
        msg->start_time == last_leader_trajectory_start_time_) {
      return;
    }
    last_leader_trajectory_id_ = msg->traj_id;
    last_leader_trajectory_start_time_ = msg->start_time;

    const std::size_t control_point_count = msg->pos_pts.size();
    if (msg->order < 1) {
      ROS_WARN("[safe_follower] ignore invalid UAV0 B-spline id=%ld order=%d.",
               static_cast<long>(msg->traj_id), msg->order);
      pending_leader_segment_endpoint_valid_ = false;
      leader_segment_endpoint_dwell_start_ = ros::Time(0);
      return;
    }
    const std::size_t expected_knot_count =
        control_point_count + static_cast<std::size_t>(msg->order) + 1U;
    if (control_point_count <= static_cast<std::size_t>(msg->order) ||
        msg->knots.size() != expected_knot_count) {
      ROS_WARN("[safe_follower] ignore invalid UAV0 B-spline id=%ld order=%d "
               "control_points=%zu knots=%zu expected_knots=%zu.",
               static_cast<long>(msg->traj_id), msg->order, control_point_count,
               msg->knots.size(), expected_knot_count);
      pending_leader_segment_endpoint_valid_ = false;
      leader_segment_endpoint_dwell_start_ = ros::Time(0);
      return;
    }

    Eigen::MatrixXd control_points(control_point_count, 3);
    for (std::size_t i = 0; i < control_point_count; ++i) {
      if (!std::isfinite(msg->pos_pts[i].x) ||
          !std::isfinite(msg->pos_pts[i].y) ||
          !std::isfinite(msg->pos_pts[i].z)) {
        ROS_WARN("[safe_follower] ignore UAV0 B-spline id=%ld with non-finite control point.",
                 static_cast<long>(msg->traj_id));
        pending_leader_segment_endpoint_valid_ = false;
        leader_segment_endpoint_dwell_start_ = ros::Time(0);
        return;
      }
      control_points(static_cast<Eigen::Index>(i), 0) = msg->pos_pts[i].x;
      control_points(static_cast<Eigen::Index>(i), 1) = msg->pos_pts[i].y;
      control_points(static_cast<Eigen::Index>(i), 2) = msg->pos_pts[i].z;
    }
    Eigen::VectorXd knots(msg->knots.size());
    for (std::size_t i = 0; i < msg->knots.size(); ++i) {
      if (!std::isfinite(msg->knots[i]) ||
          (i > 0U && msg->knots[i] + 1.0e-9 < msg->knots[i - 1U])) {
        ROS_WARN("[safe_follower] ignore UAV0 B-spline id=%ld with invalid knots.",
                 static_cast<long>(msg->traj_id));
        pending_leader_segment_endpoint_valid_ = false;
        leader_segment_endpoint_dwell_start_ = ros::Time(0);
        return;
      }
      knots(static_cast<Eigen::Index>(i)) = msg->knots[i];
    }

    fast_planner::NonUniformBspline trajectory(control_points, msg->order, 1.0);
    trajectory.setKnot(knots);
    double trajectory_start = 0.0;
    double trajectory_end = 0.0;
    trajectory.getTimeSpan(trajectory_start, trajectory_end);
    if (!std::isfinite(trajectory_start) || !std::isfinite(trajectory_end) ||
        trajectory_end <= trajectory_start) {
      ROS_WARN("[safe_follower] ignore UAV0 B-spline id=%ld with invalid time span.",
               static_cast<long>(msg->traj_id));
      pending_leader_segment_endpoint_valid_ = false;
      leader_segment_endpoint_dwell_start_ = ros::Time(0);
      return;
    }
    const Eigen::VectorXd endpoint = trajectory.evaluateDeBoor(trajectory_end);
    if (endpoint.size() < 3 || !endpoint.head(3).allFinite()) {
      ROS_WARN("[safe_follower] ignore UAV0 B-spline id=%ld with invalid endpoint.",
               static_cast<long>(msg->traj_id));
      pending_leader_segment_endpoint_valid_ = false;
      leader_segment_endpoint_dwell_start_ = ros::Time(0);
      return;
    }

    geometry_msgs::Point endpoint_local;
    endpoint_local.x = endpoint(0);
    endpoint_local.y = endpoint(1);
    endpoint_local.z = endpoint(2);
    pending_leader_segment_endpoint_.position =
        followerCruisePointToWorld(leaderToWorld(endpoint_local));
    pending_leader_segment_endpoint_.yaw = 0.0;
    pending_leader_segment_endpoint_.progress = leader_route_progress_;
    pending_leader_segment_endpoint_valid_ = true;
    pending_leader_segment_trajectory_id_ = msg->traj_id;
    leader_segment_endpoint_dwell_start_ = ros::Time(0);
    ROS_INFO("[safe_follower] track UAV0 B-spline endpoint id=%ld world=(%.2f,%.2f,%.2f); "
             "cache only after actual stop.",
             static_cast<long>(msg->traj_id),
             pending_leader_segment_endpoint_.position.x,
             pending_leader_segment_endpoint_.position.y,
             pending_leader_segment_endpoint_.position.z);
  }

  void confirmPendingLeaderSegmentEndpoint(const RoutePoint& actual_point,
                                           const geometry_msgs::Twist& actual_twist,
                                           const ros::Time& now) {
    if (!pending_leader_segment_endpoint_valid_) return;
    const double endpoint_error = std::hypot(
        actual_point.position.x - pending_leader_segment_endpoint_.position.x,
        actual_point.position.y - pending_leader_segment_endpoint_.position.y);
    const double speed = std::sqrt(
        actual_twist.linear.x * actual_twist.linear.x +
        actual_twist.linear.y * actual_twist.linear.y +
        actual_twist.linear.z * actual_twist.linear.z);
    if (endpoint_error > leader_segment_endpoint_radius_ ||
        speed > leader_segment_endpoint_max_speed_) {
      leader_segment_endpoint_dwell_start_ = ros::Time(0);
      return;
    }
    if (leader_segment_endpoint_dwell_start_.isZero()) {
      leader_segment_endpoint_dwell_start_ = now;
      return;
    }
    if ((now - leader_segment_endpoint_dwell_start_).toSec() + 1.0e-6 <
        leader_segment_endpoint_dwell_) {
      return;
    }

    RoutePoint confirmed = actual_point;
    // 正常段末端与route_末点最多相差一个采样间距；转弯缓存尚未正式接入时，
    // 先用其累计长度作进度提示，选用前还会再次核对实飞折线位置。
    confirmed.progress = leader_route_progress_ + turn_candidate_length_;
    bool duplicate = false;
    for (const RoutePoint& endpoint : leader_segment_endpoints_) {
      if (std::fabs(endpoint.progress - confirmed.progress) <= path_sample_spacing_ &&
          std::hypot(endpoint.position.x - confirmed.position.x,
                     endpoint.position.y - confirmed.position.y) <=
              leader_segment_endpoint_radius_) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      leader_segment_endpoints_.push_back(confirmed);
      while (leader_segment_endpoints_.size() > 200U) {
        leader_segment_endpoints_.pop_front();
      }
      ROS_WARN("[safe_follower] CONFIRM UAV0 segment endpoint id=%ld progress=%.2f "
               "world=(%.2f,%.2f,%.2f), actual stop %.2fs.",
               static_cast<long>(pending_leader_segment_trajectory_id_),
               confirmed.progress, confirmed.position.x, confirmed.position.y,
               confirmed.position.z, leader_segment_endpoint_dwell_);
      if (simple_segment_endpoint_following_ && door_waypoint_released_ &&
          !exit_waypoint_released_ && !terminal_mode_active_) {
        const bool already_queued = !relay_waypoints_.empty() &&
            distance3d(relay_waypoints_.back().position, confirmed.position) <=
                leader_segment_endpoint_radius_;
        if (!already_queued) {
          appendRelayWaypoint(confirmed, "SEGMENT_ENDPOINT");
          ROS_ERROR("[safe_follower] FIFO cache UAV0 segment endpoint id=%ld; "
                    "UAV1 will consume it only after arrival.",
                    static_cast<long>(pending_leader_segment_trajectory_id_));
        }
      }
    }
    pending_leader_segment_endpoint_valid_ = false;
    leader_segment_endpoint_dwell_start_ = ros::Time(0);
  }

  void leaderOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    const ros::Time now = ros::Time::now();
    if (!msg->header.stamp.isZero()) {
      const double transport_age = (now - msg->header.stamp).toSec();
      if (transport_age > leader_odom_max_transport_age_) {
        ROS_WARN_THROTTLE(
            1.0,
            "[safe_follower] reject delayed leader odometry age=%.3fs > %.3fs; "
            "keep UAV1 parked.",
            transport_age, leader_odom_max_transport_age_);
        return;
      }
    }
    leader_odom_ = *msg;
    leader_odom_stamp_ = now;
    have_leader_odom_ = true;
    if (!leader_started_ && msg->pose.pose.position.z > leader_start_height_) {
      leader_started_ = true;
      ROS_WARN("[safe_follower] UAV0 height %.2fm > %.2fm; leader route recording enabled.",
               msg->pose.pose.position.z, leader_start_height_);
    }
    if (!leader_started_) return;

    const geometry_msgs::Point leader_world =
        leaderToWorld(msg->pose.pose.position);
    const bool door_released_from_position =
        tryReleaseDoorWaypointFromLeaderPosition(leader_world);
    if (msg->pose.pose.position.z < min_record_height_) return;

    RoutePoint point;
    point.position = leader_world;
    // 2026-07-24: 安全路线的几何进度只取前机XY，z统一为后机自己的巡航高度。
    point.position = followerCruisePointToWorld(point.position);
    point.yaw = yawFromQuaternion(msg->pose.pose.orientation);
    confirmPendingLeaderSegmentEndpoint(point, msg->twist.twist, now);
    bool current_point_already_appended = false;
    // 2026-07-20: 独立保存最近一次判向采样点，不能拿route_.back()判向；route_.back()在回头期间
    // 会故意保持不动，否则回到旧路后的一大段位移仍会被错误追加为新安全路线。
    if (!have_last_leader_sample_) {
      last_leader_sample_ = point;
      have_last_leader_sample_ = true;
    } else {
      const double dx = point.position.x - last_leader_sample_.position.x;
      const double dy = point.position.y - last_leader_sample_.position.y;
      const double horizontal_step = std::hypot(dx, dy);
      if (horizontal_step < path_sample_spacing_) return;
      last_leader_sample_ = point;

      // 2026-07-21: 首段尚无轨迹切线时先接受；之后以局部路线切线判断继续前进或真实倒退。
      double route_direction_x = 0.0;
      double route_direction_y = 0.0;
      const bool have_route_direction =
          getRouteForwardDirection(&route_direction_x, &route_direction_y);
      const double forward_projection =
          have_route_direction ? route_direction_x * dx + route_direction_y * dy
                               : horizontal_step;
      const double projection_ratio = forward_projection / std::max(1e-6, horizontal_step);
      if (projection_ratio < forward_projection_ratio_) {
        // 2026-07-21: 不能立即丢弃负投影点。U形通道转弯开始时相对旧切线必然短暂为负，
        // 先保存实飞折线；只有它始终贴着旧路线才是回头，离开旧路线后则确认成新的安全分支。
        turn_candidate_length_ += horizontal_step;
        turn_candidate_route_.push_back(point);
        const double distance_to_old_route = distanceToAcceptedRoute(point.position);
        const bool confirms_new_turn_branch =
            turn_candidate_length_ >= turn_branch_confirm_distance_ &&
            distance_to_old_route > route_revisit_radius_;
        if (confirms_new_turn_branch) {
          for (RoutePoint& candidate : turn_candidate_route_) {
            if (appendAcceptedRoutePoint(&candidate)) point.progress = candidate.progress;
          }
          current_point_already_appended = true;
          turn_candidate_route_.clear();
          turn_candidate_length_ = 0.0;
          consecutive_backward_distance_ = 0.0;
          relay_route_paused_ = false;
          ROS_ERROR("[safe_follower] ACCEPT curved/new route branch after %.2fm clearance from "
                    "old route; relay recording resumed without publishing a straight shortcut.",
                    distance_to_old_route);
        } else {
          // 2026-07-21: 与局部路线切线明确反向的采样不累计progress；持续倒退达到阈值后
          // 才清除pending并锁住路线，正常转弯的横向分量不会抹掉已形成的候选接力点。
          if (forward_projection < 0.0)
            consecutive_backward_distance_ += horizontal_step;
          else
            consecutive_backward_distance_ = 0.0;
          if (consecutive_backward_distance_ >= backtrack_pause_distance_) {
            relay_route_paused_ = true;
            pending_relay_valid_ = false;
          }
          ROS_WARN_THROTTLE(
              1.0,
              "[safe_follower] IGNORE route-reverse leader motion step=%.2fm projection=%.2f "
              "backward_sum=%.2fm paused=%d; no relay progress.",
              horizontal_step, projection_ratio, consecutive_backward_distance_,
              static_cast<int>(relay_route_paused_));
          return;
        }
      }
      if (!current_point_already_appended) {
        consecutive_backward_distance_ = 0.0;
        // 2026-07-21: 负投影后重新回到旧端点属于短暂抖动，缓存不能在之后误接成一段回头路线。
        turn_candidate_route_.clear();
        turn_candidate_length_ = 0.0;
      }

      if (relay_route_paused_ && !current_point_already_appended) {
        // 2026-07-20: 只比较水平距离，避免高度跟踪误差导致前机已经回到旧路线端点却无法解锁。
        const double distance_to_route_end =
            route_.empty() ? 0.0
                           : std::hypot(point.position.x - route_.back().position.x,
                                        point.position.y - route_.back().position.y);
        if (!route_.empty() && distance_to_route_end > route_resume_radius_) {
          pending_relay_valid_ = false;
          ROS_WARN_THROTTLE(
              1.0,
              "[safe_follower] IGNORE leader backtrack/rejoin distance_to_route_end=%.2fm; "
              "wait return to previous forward endpoint.",
              distance_to_route_end);
          return;
        }
        // 2026-07-20: 只有回到最后一个已认可的前进端点附近且再次机头向前，才恢复路线累计。
        relay_route_paused_ = false;
        ROS_WARN("[safe_follower] leader returned to forward route endpoint; relay recording resumed.");
      }

      bool revisits_old_route = false;
      if (!route_.empty() && !current_point_already_appended) {
        const double newest_progress = route_.back().progress;
        for (const RoutePoint& old_point : route_) {
          if (newest_progress - old_point.progress < route_revisit_progress_gap_) continue;
          if (std::hypot(point.position.x - old_point.position.x,
                         point.position.y - old_point.position.y) <= route_revisit_radius_) {
            revisits_old_route = true;
            break;
          }
        }
      }
      if (revisits_old_route) {
        // 2026-07-20: 即使机头已经转过来“正着飞”，只要正在重走历史路线仍属于回头路；
        // 暂停接力记录，避免后机再次执行前机走过的旧分支。
        relay_route_paused_ = true;
        pending_relay_valid_ = false;
        ROS_ERROR_THROTTLE(
            1.0,
            "[safe_follower] IGNORE revisited leader route near old path; no relay waypoint released.");
        return;
      }
    }
    if (!current_point_already_appended && !appendAcceptedRoutePoint(&point)) return;
    publishRoute(msg->header.stamp);

    if (!have_confirmed_door_ || terminal_mode_active_) return;
    if (door_released_from_position) return;
    if (!door_waypoint_released_) return;

    // 2026-07-28: 最终出口门心已排队后不再生成门外内部点；稠密实飞路线仍继续记录供终点接力使用。
    if (exit_waypoint_released_) return;
    // 简化模式仅消费前机真实停稳的分段终点；禁止按里程再插入内部候选点。
    if (simple_segment_endpoint_following_) return;

    // 2026-07-16: 0或负数表示任务全程滚动发点；正数仅保留为调试时的可选安全上限。
    if (max_internal_relay_points_ > 0 &&
        internal_relay_count_ >= max_internal_relay_points_) {
      return;
    }
    if (!pending_relay_valid_ &&
        point.progress - last_relay_selection_progress_ >= relay_waypoint_spacing_) {
      // 2026-07-15: 先锁定候选A，但此时不发布；继续观察前机真实走过1m后才确认这段可通行。
      pending_relay_ = point;
      pending_relay_valid_ = true;
      ROS_WARN("[safe_follower] pending internal waypoint at progress %.2fm; wait leader clear %.2fm.",
               pending_relay_.progress, relay_release_distance_);
    }
    if (pending_relay_valid_ &&
        point.progress - pending_relay_.progress >= relay_release_distance_) {
      if (!relayWaypointSeparationReady("INTERNAL")) return;
      appendRelayWaypoint(pending_relay_, "INTERNAL");
      last_relay_selection_progress_ = pending_relay_.progress;
      pending_relay_valid_ = false;
      ++internal_relay_count_;
    }
  }

  void appendFollowerExecutedPoint(const nav_msgs::Odometry& odom) {
    RoutePoint point;
    point.position = followerToWorld(odom.pose.pose.position);
    point.yaw = yawFromQuaternion(odom.pose.pose.orientation);
    if (follower_executed_route_.empty()) {
      point.progress = 0.0;
      follower_executed_route_.push_back(point);
      return;
    }
    const double segment = distance3d(follower_executed_route_.back().position,
                                      point.position);
    if (segment < follower_history_sample_spacing_) return;
    follower_history_length_ += segment;
    point.progress = follower_executed_route_.back().progress + segment;
    follower_executed_route_.push_back(point);
    while (follower_executed_route_.size() > 2 &&
           follower_history_length_ > follower_history_max_length_) {
      follower_history_length_ -= distance3d(follower_executed_route_[0].position,
                                             follower_executed_route_[1].position);
      follower_executed_route_.pop_front();
      if (diff_follower_retreat_target_index_ !=
              std::numeric_limits<std::size_t>::max() &&
          diff_follower_retreat_target_index_ > 0) {
        --diff_follower_retreat_target_index_;
      }
    }
  }

  void truncateFollowerExecutedRoute(std::size_t target_index,
                                      const geometry_msgs::Point& reached_local) {
    if (follower_executed_route_.empty()) return;
    target_index = std::min(target_index, follower_executed_route_.size() - 1);
    while (follower_executed_route_.size() > target_index + 1) {
      const std::size_t last = follower_executed_route_.size() - 1;
      follower_history_length_ -= distance3d(follower_executed_route_[last].position,
                                             follower_executed_route_[last - 1].position);
      follower_executed_route_.pop_back();
    }
    RoutePoint reached;
    reached.position = followerToWorld(reached_local);
    reached.yaw = yawFromQuaternion(follower_odom_.pose.pose.orientation);
    const double tail = distance3d(follower_executed_route_.back().position,
                                   reached.position);
    if (tail >= follower_history_sample_spacing_) {
      follower_history_length_ += tail;
      reached.progress = follower_executed_route_.back().progress + tail;
      follower_executed_route_.push_back(reached);
    }
  }

  void followerOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 2026-07-22: 用实际里程计位移估计到达速度，不能用规划指令速度代替真实制动状态。
    const ros::Time now = ros::Time::now();
    // 高频里程计会经网络成批到达。速度差分必须使用传感器时间戳，不能使用回调接收时间；
    // 否则毫米级点云校正除以亚毫秒级回调间隔会被放大成数m/s并误触发安全锁点。
    const ros::Time sample_stamp = msg->header.stamp;
    if (sample_stamp.isZero()) {
      ROS_WARN_THROTTLE(1.0,
                        "[safe_follower] reject zero-stamp follower odometry sample.");
      return;
    }
    if (have_follower_odom_) {
      const double dt = (sample_stamp - follower_odom_sample_stamp_).toSec();
      if (dt <= 0.0) {
        ROS_WARN_THROTTLE(
            1.0,
            "[safe_follower] reject non-monotonic follower odometry stamp dt=%.6fs.",
            dt);
        return;
      }
      if (dt < 1e-3) {
        ROS_WARN_THROTTLE(
            1.0,
            "[safe_follower] reject duplicate/burst follower odometry stamp dt=%.6fs.",
            dt);
        return;
      }
      if (dt < 0.25) {
        const double raw_horizontal_speed =
            std::hypot(msg->pose.pose.position.x - follower_odom_.pose.pose.position.x,
                       msg->pose.pose.position.y - follower_odom_.pose.pose.position.y) / dt;
        const double raw_vertical_speed =
            std::fabs(msg->pose.pose.position.z - follower_odom_.pose.pose.position.z) / dt;
        // 2026-07-28: 7月27日末次实验在111s后FAST-LIO连续发散到数百米；首次不可能速度即锁存故障并丢弃坏里程计。
        // 2026-07-28: 接管前允许FAST-LIO完成初始对齐；只有后机控制已交接后才把不可能速度判为飞行故障。
        if (follower_started_ &&
            (raw_horizontal_speed > follower_odom_jump_speed_ ||
             raw_vertical_speed > follower_odom_jump_vertical_speed_)) {
          ++follower_odom_jump_consecutive_samples_;
          ROS_WARN_THROTTLE(
              0.5,
              "[safe_follower] suspicious ODOM sample %d/%d: horizontal=%.2fm/s "
              "vertical=%.2fm/s dt=%.4fs; discard pending confirmation.",
              follower_odom_jump_consecutive_samples_,
              follower_odom_jump_confirm_samples_, raw_horizontal_speed,
              raw_vertical_speed, dt);
          if (!follower_odom_fault_latched_ &&
              follower_odom_jump_consecutive_samples_ >=
                  follower_odom_jump_confirm_samples_) {
            follower_odom_fault_latched_ = true;
            follower_odom_recovery_good_samples_ = 0;
            std_msgs::Bool hold_msg;
            hold_msg.data = true;
            follower_safety_hold_pub_.publish(hold_msg);
            ROS_ERROR("[safe_follower] ODOM FAULT latched after %d consecutive samples: "
                      "horizontal=%.2fm/s vertical=%.2fm/s; reject bad FAST-LIO sample "
                      "and request MAVROS-frame safety hold.",
                      follower_odom_jump_consecutive_samples_, raw_horizontal_speed,
                      raw_vertical_speed);
          }
          return;
        }
        follower_odom_jump_consecutive_samples_ = 0;
        if (follower_odom_fault_latched_) {
          ++follower_odom_recovery_good_samples_;
          if (follower_odom_recovery_good_samples_ >= 10) {
            follower_odom_fault_latched_ = false;
            follower_odom_recovery_good_samples_ = 0;
            if (use_diff_planner_) {
              // 故障期间可能积累了旧规划；恢复后继续保持锁点，直到新目标确实生成新轨迹。
              setDiffWaitPositionHold(true,
                                      "odometry recovered; wait fresh Diff trajectory");
            } else if (!diff_dynamic_hold_active_ &&
                       !diff_separation_hold_active_) {
              std_msgs::Bool hold_msg;
              hold_msg.data = false;
              follower_safety_hold_pub_.publish(hold_msg);
            }
            diff_goal_published_ = false;
            diff_plan_response_received_ = false;
            ROS_WARN("[safe_follower] follower odometry recovered after 10 trusted samples; "
                     "release fault hold and request a fresh Diff trajectory.");
          }
        }
        const double alpha = 0.35;
        follower_horizontal_speed_ =
            alpha * raw_horizontal_speed + (1.0 - alpha) * follower_horizontal_speed_;
        follower_vertical_speed_ =
            alpha * raw_vertical_speed + (1.0 - alpha) * follower_vertical_speed_;
      } else {
        // 长时间通信间断后的首帧不参与速度差分，也不能继承此前的疑似计数。
        follower_odom_jump_consecutive_samples_ = 0;
        follower_odom_recovery_good_samples_ = 0;
      }
    }
    follower_odom_ = *msg;
    follower_odom_stamp_ = now;
    follower_odom_sample_stamp_ = sample_stamp;
    have_follower_odom_ = true;
    if (!follower_started_ && msg->pose.pose.position.z > follower_start_height_) {
      follower_started_ = true;
      ROS_WARN("[safe_follower] UAV1 height %.2fm > %.2fm; follower is ready for a real Diff goal.",
               msg->pose.pose.position.z, follower_start_height_);
      if (!traj_started_sent_ && !use_diff_planner_) {
        traj_started_pub_.publish(std_msgs::Empty());
        traj_started_sent_ = true;
      }
    }
    if (follower_started_ && !follower_odom_fault_latched_ &&
        !diff_recovery_retreat_requested_ && !diff_recovery_goal_is_retreat_) {
      appendFollowerExecutedPoint(*msg);
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    follower_cloud_ = msg;
    cloud_stamp_ = ros::Time::now();
  }

  // 2026-07-28: traj_server实际输出比一次性的TRAJECTORY_PUBLISHED更能说明当前轨迹是否仍在驱动控制器。
  void diffCommandCallback(const quadrotor_msgs::PositionCommand::ConstPtr&) {
    diff_command_stamp_ = ros::Time::now();
    diff_command_seen_for_goal_ = true;
  }

  // LDOP输出模型状态；当前比赛小球默认CV3D=[x,y,z,vx,vy,vz]。
  // 跟随器自行生成1秒匀速预测，使避障时域仍由控制侧决定。
  void dynamicObstacleCallback(
      const ldop::DynamicObjectArray::ConstPtr& msg) {
    if (!enable_dynamic_obstacle_detection_) return;
    dynamic_obstacle_receive_stamp_ = ros::Time::now();
    std::vector<DynamicObstacleSample> corridor_obstacles;
    for (const auto& object : msg->objects) {
      if (object.model_state.size() < 3U ||
          !std::isfinite(object.model_state[0]) ||
          !std::isfinite(object.model_state[1]) ||
          !std::isfinite(object.model_state[2])) continue;
      DynamicObstacleSample obstacle;
      obstacle.id = object.id;
      obstacle.position.x = object.model_state[0];
      obstacle.position.y = object.model_state[1];
      obstacle.position.z = object.model_state[2];
      obstacle.size = object.size;
      const geometry_msgs::Point obstacle_world = followerToWorld(obstacle.position);
      const double route_distance = distanceToAcceptedRoute(obstacle_world);
      if (!route_.empty() && route_distance > dynamic_retention_route_half_width_) {
        ROS_WARN_THROTTLE(1.0,
                          "[safe_follower] DROP dynamic id=%u from retention: "
                          "route_distance=%.2fm > %.2fm corridor band.",
                          obstacle.id, route_distance, dynamic_retention_route_half_width_);
        continue;
      }
      geometry_msgs::Vector3 velocity;
      bool have_velocity = false;
      if (object.motion_model_type == ldop::DynamicObject::MOTION_MODEL_CA2D &&
          object.model_state.size() >= 5U) {
        velocity.x = object.model_state[3];
        velocity.y = object.model_state[4];
        velocity.z = 0.0;
        have_velocity = true;
      } else if ((object.motion_model_type == ldop::DynamicObject::MOTION_MODEL_CA3D ||
                  object.motion_model_type == ldop::DynamicObject::MOTION_MODEL_CV3D) &&
                 object.model_state.size() >= 6U) {
        velocity.x = object.model_state[3];
        velocity.y = object.model_state[4];
        velocity.z = object.model_state[5];
        have_velocity = true;
      } else if (object.motion_model_type == ldop::DynamicObject::MOTION_MODEL_CTRA &&
                 object.model_state.size() >= 6U) {
        const double speed = object.model_state[3];
        const double yaw = object.model_state[5];
        velocity.x = speed * std::cos(yaw);
        velocity.y = speed * std::sin(yaw);
        velocity.z = 0.0;
        have_velocity = true;
      }
      if (have_velocity && std::isfinite(velocity.x) &&
          std::isfinite(velocity.y) && std::isfinite(velocity.z)) {
        for (double horizon = 0.2; horizon <= 1.0 + 1e-6; horizon += 0.2) {
          geometry_msgs::Point predicted = obstacle.position;
          predicted.x += horizon * velocity.x;
          predicted.y += horizon * velocity.y;
          predicted.z += horizon * velocity.z;
          obstacle.predicted_positions.push_back(predicted);
        }
      }
      corridor_obstacles.push_back(obstacle);
    }
    // 2026-07-28: 合格非空帧刷新缓存；摆球端点和短时遮挡期间沿用上一帧，但只保留有限时长。
    if (!corridor_obstacles.empty()) {
      retained_dynamic_obstacles_ = corridor_obstacles;
      retained_dynamic_obstacle_stamp_ = dynamic_obstacle_receive_stamp_;
    }
  }

  // 2026-07-28: Diff无轨迹等待不能只发零速度；通过控制器安全话题锁存MAVROS本地位置，直到新轨迹真正发布。
  void setDiffWaitPositionHold(bool active, const std::string& reason) {
    if (diff_wait_hold_active_ == active) return;
    diff_wait_hold_active_ = active;
    if (!active && diff_dynamic_hold_active_) return;
    std_msgs::Bool hold_msg;
    hold_msg.data = active;
    follower_safety_hold_pub_.publish(hold_msg);
    ROS_ERROR("[safe_follower] UAV1 Diff wait-position HOLD %s reason=%s.",
              active ? "ACTIVE" : "RELEASED", reason.c_str());
  }

  void stampDiffGoalId(geometry_msgs::PoseStamped* goal) {
    // roscpp 会在发布时重写 Header.seq，不能用它关联规划回执。时间戳由消息
    // 原样传到规划器，同时纳秒值足以过滤锁存状态和上一进程的延迟回执。
    uint64_t goal_stamp_ns = goal->header.stamp.toNSec();
    if (goal_stamp_ns == 0ULL)
      goal_stamp_ns = ros::WallTime::now().toNSec();
    if (goal_stamp_ns <= diff_active_goal_stamp_ns_)
      goal_stamp_ns = diff_active_goal_stamp_ns_ + 1ULL;
    goal->header.stamp.fromNSec(goal_stamp_ns);
    diff_active_goal_stamp_ns_ = goal_stamp_ns;
  }

  void handleDiffPlanningFailure(const std::string& status,
                                 const ros::Time& now) {
    // 普通规划失败只锁点到下一条轨迹产生。当前候选立即拉黑并换相邻点；
    // 连续多个前向候选失败后，沿UAV1自己的实飞轨迹退一步，不进入人工恢复终态。
    setDiffWaitPositionHold(true, status);
    ++diff_planning_failure_events_;
    const bool failed_retreat =
        diff_recovery_goal_valid_ && diff_recovery_goal_is_retreat_;
    const bool failed_forward_candidate =
        diff_route_subgoal_valid_ && !diff_recovery_goal_valid_;
    if (failed_retreat) {
      const double retreat_key = -1000.0 - static_cast<double>(
          diff_follower_retreat_target_index_ ==
                  std::numeric_limits<std::size_t>::max()
              ? 0
              : diff_follower_retreat_target_index_);
      blacklistRouteCandidate(diff_recovery_goal_local_, retreat_key, now,
                              "UAV1-history retreat planning failed");
      ++diff_failure_retreat_attempts_;
      diff_recovery_requested_ = true;
      diff_recovery_retreat_requested_ = true;
    } else if (failed_forward_candidate) {
      blacklistRouteCandidate(diff_route_subgoal_local_,
                              diff_route_subgoal_progress_, now, status);
      ++diff_forward_candidate_failures_;
      diff_route_subgoal_valid_ = false;
      diff_route_subgoal_completes_relay_ = false;
      diff_clipped_retry_count_ = 0;
      const bool need_retreat =
          diff_forward_candidate_failures_ >=
          diff_forward_failures_before_retreat_;
      diff_recovery_requested_ = need_retreat;
      diff_recovery_retreat_requested_ = need_retreat;
    } else {
      // 终端点或没有活动短点时保留任务目标，仅触发下一周期重新规划。
      diff_recovery_requested_ = false;
      diff_recovery_retreat_requested_ = false;
    }
    diff_recovery_goal_valid_ = false;
    diff_recovery_goal_is_retreat_ = false;
    diff_goal_published_ = false;
    diff_accepted_goal_valid_ = false;
    diff_command_seen_for_goal_ = false;
    diff_goal_first_publish_stamp_ = ros::Time(0);
    ROS_ERROR_THROTTLE(
        0.5,
        "[safe_follower] UAV1 Diff status=%s; next action=%s, forward failures=%d.",
        status.c_str(), diff_recovery_retreat_requested_
                            ? "retreat on UAV1 executed history"
                            : "select another UAV0-history candidate",
        diff_forward_candidate_failures_);
  }

  void leaderTaskStatusCallback(const std_msgs::String::ConstPtr& msg) {
    if (!enable_search_landing_) {
      leader_outside_exit_ = false;
      return;
    }
    // task_status首字段为阶段名。前机真正越过出口后停止动态跟距，恢复离散任务点/终点执行。
    leader_outside_exit_ = msg->data.find("SEARCH_OUTSIDE_LANDING") == 0 ||
                           msg->data.find("SEARCH_OUTSIDE_QR") == 0 ||
                           msg->data.find("APPROACH_LANDING") == 0 ||
                           msg->data.find("LANDING") == 0;
    // 门只是通道终点。UAV0出门后，UAV1仍按已缓存历史轨迹依次到门心和门外等待点。
  }

  void diffStatusCallback(const std_msgs::String::ConstPtr& msg) {
    std::istringstream stream(msg->data);
    std::string status;
    std::string sequence_token;
    stream >> status >> sequence_token;
    uint64_t response_goal_stamp_ns = 0ULL;
    bool response_goal_stamp_valid = false;
    const std::string stamp_prefix = "goal_stamp_ns=";
    if (sequence_token.compare(0, stamp_prefix.size(), stamp_prefix) == 0) {
      std::istringstream sequence_stream(sequence_token.substr(stamp_prefix.size()));
      unsigned long long parsed_stamp_ns = 0ULL;
      char trailing = '\0';
      if ((sequence_stream >> parsed_stamp_ns) && !(sequence_stream >> trailing)) {
        response_goal_stamp_ns = static_cast<uint64_t>(parsed_stamp_ns);
        response_goal_stamp_valid = true;
      }
    }

    geometry_msgs::Point accepted;
    const bool accepted_valid =
        status == "TRAJECTORY_PUBLISHED" &&
        static_cast<bool>(stream >> accepted.x >> accepted.y >> accepted.z);

    // planning/status为锁存话题且规划线程可能延迟完成。目标时间戳、接力索引和逻辑
    // 候选必须仍然有效；已经因失败拉黑的候选即使晚到成功，也不能重新放行旧轨迹。
    const bool response_matches_active_goal =
        response_goal_stamp_valid && diff_goal_index_ == active_relay_index_ &&
        response_goal_stamp_ns == diff_active_goal_stamp_ns_;
    const bool late_success_for_active_goal =
        response_matches_active_goal && !diff_goal_published_ &&
        status == "TRAJECTORY_PUBLISHED" && accepted_valid &&
        !diff_separation_hold_active_ && !diff_recovery_requested_ &&
        (diff_route_subgoal_valid_ || diff_recovery_goal_valid_);
    if (!response_matches_active_goal ||
        (!diff_goal_published_ && !late_success_for_active_goal)) {
      const std::string response_stamp_label =
          response_goal_stamp_valid ? std::to_string(response_goal_stamp_ns) : "invalid";
      ROS_WARN("[safe_follower] ignore stale UAV1 Diff status=%s goal_stamp_ns=%s; "
               "active=%d relay=%zu/%zu active_stamp_ns=%llu.",
               status.c_str(), response_stamp_label.c_str(),
               static_cast<int>(diff_goal_published_), diff_goal_index_ + 1,
               active_relay_index_ + 1,
               static_cast<unsigned long long>(diff_active_goal_stamp_ns_));
      return;
    }
    if (late_success_for_active_goal) {
      diff_goal_published_ = true;
      diff_recovery_requested_ = false;
      diff_recovery_retreat_requested_ = false;
      diff_recovery_goal_valid_ = false;
      diff_recovery_goal_is_retreat_ = false;
      ROS_WARN("[safe_follower] accept delayed success for still-active UAV1 Diff goal "
               "stamp_ns=%llu.",
               static_cast<unsigned long long>(response_goal_stamp_ns));
    }

    // 间距保护期间只允许当前退让目标解除锁点；旧接力轨迹的延迟状态不能让后机再次前冲。
    if (diff_separation_hold_active_) {
      if (status == "PLANNING_FAILED" || status == "GOAL_REJECTED_OUTSIDE_MAP") {
        setDiffWaitPositionHold(true, "separation retreat planning failed");
        separation_recovery_goal_valid_ = false;
        diff_goal_published_ = false;
        diff_plan_response_received_ = true;
        diff_accepted_goal_valid_ = false;
        ROS_ERROR("[safe_follower] UAV1 Diff separation retreat status=%s; keep HOLD and "
                  "select another retreat step.", status.c_str());
        return;
      }
      if (status == "TRAJECTORY_PUBLISHED") {
        const double accepted_error = accepted_valid && separation_recovery_goal_valid_
            ? distance3d(accepted, separation_recovery_goal_local_)
            : std::numeric_limits<double>::infinity();
        const geometry_msgs::Point leader_world =
            leaderToWorld(leader_odom_.pose.pose.position);
        const geometry_msgs::Point follower_world =
            followerToWorld(follower_odom_.pose.pose.position);
        const geometry_msgs::Point accepted_world = followerToWorld(accepted);
        const double current_separation =
            std::hypot(leader_world.x - follower_world.x,
                       leader_world.y - follower_world.y);
        const double accepted_separation = accepted_valid
            ? std::hypot(leader_world.x - accepted_world.x,
                         leader_world.y - accepted_world.y)
            : -std::numeric_limits<double>::infinity();
        const bool accepted_improves_separation =
            accepted_separation >= current_separation + 0.05;
        if (!separation_recovery_active_ || !separation_recovery_goal_valid_ ||
            accepted_error > emergency_retreat_step_ + 0.10 ||
            !accepted_improves_separation) {
          setDiffWaitPositionHold(true, "ignore stale trajectory during separation hold");
          separation_recovery_goal_valid_ = false;
          diff_goal_published_ = false;
          diff_plan_response_received_ = true;
          diff_accepted_goal_valid_ = false;
          ROS_WARN("[safe_follower] ignore stale UAV1 Diff trajectory during separation "
                   "HOLD (accepted_valid=%d error=%.2fm separation=%.2f->%.2fm).",
                   static_cast<int>(accepted_valid), accepted_error,
                   current_separation, accepted_separation);
          return;
        }
        diff_plan_response_received_ = true;
        diff_accepted_goal_local_ = accepted;
        diff_accepted_goal_valid_ = true;
        setDiffWaitPositionHold(false, "separation retreat trajectory published");
        ROS_ERROR("[safe_follower] UAV1 Diff separation retreat accepted "
                  "actual_goal=(%.2f,%.2f,%.2f).", accepted.x, accepted.y, accepted.z);
        return;
      }
    }
    // A non-stale status is the planner's response to the current goal.
    diff_plan_response_received_ = true;
    diff_goal_first_publish_stamp_ = ros::Time(0);
    if (simple_segment_endpoint_following_) {
      if (status == "PLANNING_FAILED" || status == "GOAL_REJECTED_OUTSIDE_MAP") {
        setDiffWaitPositionHold(true, "retry same FIFO endpoint after Diff failure");
        ++diff_planning_failure_events_;
        diff_goal_published_ = false;
        diff_accepted_goal_valid_ = false;
        diff_command_seen_for_goal_ = false;
        simple_diff_retry_not_before_ =
            ros::Time::now() + ros::Duration(simple_diff_retry_delay_);
        ROS_ERROR("[safe_follower] UAV1 Diff status=%s; keep FIFO endpoint %zu and "
                  "retry the same coordinates (no alternate/recovery point).",
                  status.c_str(), active_relay_index_ + 1);
      } else if (status == "TRAJECTORY_PUBLISHED") {
        setDiffWaitPositionHold(false, "current FIFO endpoint trajectory published");
        diff_planning_failure_events_ = 0;
        if (accepted_valid) {
          diff_accepted_goal_local_ = accepted;
          diff_accepted_goal_valid_ = true;
        }
      } else if (status == "OCCUPIED_RECOVERY_VERTICAL" ||
                 status == "OCCUPIED_RECOVERY_HISTORY" ||
                 status == "OCCUPIED_RECOVERY_LATERAL") {
        // Diff正在沿自身安全历史执行脱障；期间禁止FIFO超时逻辑重发旧目标。
        simple_occupied_recovery_active_ = true;
        setDiffWaitPositionHold(false, "Diff internal occupied recovery trajectory");
      } else if (status == "OCCUPIED_RECOVERY_ABORTED") {
        // Diff会在地图刷新后继续从历史中寻找后退点；不要再次向占据位置前方规划。
        simple_occupied_recovery_active_ = true;
        setDiffWaitPositionHold(true, "wait for Diff occupied recovery retry");
        diff_goal_published_ = false;
        diff_accepted_goal_valid_ = false;
        diff_command_seen_for_goal_ = false;
      } else if (status == "OCCUPIED_RECOVERY_SUCCEEDED") {
        simple_occupied_recovery_active_ = false;
        setDiffWaitPositionHold(true, "occupied recovery finished; skip unsafe FIFO endpoint");
        const bool terminal_relay = terminal_mode_active_ &&
            active_relay_index_ == terminal_waypoint_index_;
        if (!terminal_relay && active_relay_index_ < relay_waypoints_.size()) {
          const geometry_msgs::Point skipped =
              relay_waypoints_[active_relay_index_].position;
          consumeSimpleRelayFront();
          ROS_ERROR("[safe_follower] UAV1 retreated along history; discard unsafe FIFO "
                    "endpoint (%.2f,%.2f,%.2f) and wait for UAV0 to clear the next "
                    "cached endpoint by %.2fm.",
                    skipped.x, skipped.y, skipped.z,
                    waypoint_release_min_separation_);
        }
        diff_goal_published_ = false;
        diff_plan_response_received_ = false;
        diff_accepted_goal_valid_ = false;
        diff_command_seen_for_goal_ = false;
        diff_goal_first_publish_stamp_ = ros::Time(0);
        diff_goal_index_ = std::numeric_limits<std::size_t>::max();
        simple_waypoint_clearance_hold_active_ = false;
        simple_diff_retry_not_before_ = ros::Time::now();
      }
      return;
    }
    // A true planning failure backs away first. An outside-map goal still uses the
    // existing short forward point because retreating cannot bring that goal into the map.
    if (status == "PLANNING_FAILED" || status == "GOAL_REJECTED_OUTSIDE_MAP") {
      handleDiffPlanningFailure(status, ros::Time::now());
    } else if (status == "TRAJECTORY_PUBLISHED") {
      // 2026-07-28: 只有Diff确认新轨迹已发布才解除等待锁点，避免“先解锁、后规划”空窗。
      setDiffWaitPositionHold(false, "new Diff trajectory published");
      diff_planning_failure_events_ = 0;
      if (accepted_valid) {
        diff_accepted_goal_local_ = accepted;
        diff_accepted_goal_valid_ = true;
        ROS_WARN("[safe_follower] UAV1 Diff trajectory accepted actual_goal=(%.2f,%.2f,%.2f).",
                 accepted.x, accepted.y, accepted.z);
      }
    } else if (status == "OCCUPIED_RECOVERY_VERTICAL" ||
               status == "OCCUPIED_RECOVERY_HISTORY" ||
               status == "OCCUPIED_RECOVERY_LATERAL") {
      // Diff已发布独立低速脱障轨迹；若仍保持控制器锁点，该轨迹永远无法执行。
      setDiffWaitPositionHold(false, "Diff occupied recovery trajectory published");
    } else if (status == "OCCUPIED_RECOVERY_ABORTED" ||
               status == "OCCUPIED_RECOVERY_SUCCEEDED") {
      // 失败时继续锁点；成功后也先等正常目标的新轨迹发布再解除。
      setDiffWaitPositionHold(true, status.c_str());
    }
  }

  void consumeSimpleRelayFront() {
    if (active_relay_index_ >= relay_waypoints_.size()) return;
    const std::size_t consumed = active_relay_index_;
    relay_waypoints_.erase(relay_waypoints_.begin() + consumed);
    const std::size_t invalid = std::numeric_limits<std::size_t>::max();
    auto adjust_index = [consumed, invalid](std::size_t* index) {
      if (*index == invalid) return;
      if (*index == consumed)
        *index = invalid;
      else if (*index > consumed)
        --(*index);
    };
    adjust_index(&exit_waypoint_index_);
    adjust_index(&outside_wait_waypoint_index_);
    adjust_index(&terminal_waypoint_index_);
    publishRelayPath();
  }

  bool handleSimpleDiffPlannerExecution(const ros::Time& now) {
    if (simple_occupied_recovery_active_) {
      publishState("DIFF_FIFO_OCCUPIED_HISTORY_RETREAT", 1.0, 0.25, 0.0);
      return true;
    }

    if (active_relay_index_ >= relay_waypoints_.size()) {
      setDiffWaitPositionHold(true, "waiting for next FIFO segment endpoint");
      publishState(have_confirmed_door_ ? "DIFF_WAIT_NEXT_SEGMENT_ENDPOINT"
                                        : "DIFF_WAIT_CONFIRMED_DOOR",
                   1.0, 0.65, 0.0);
      return true;
    }

    const RoutePoint desired_world = relay_waypoints_[active_relay_index_];
    const bool terminal_relay = terminal_mode_active_ &&
        active_relay_index_ == terminal_waypoint_index_;
    const bool outside_wait_relay = outside_wait_waypoint_released_ &&
        active_relay_index_ == outside_wait_waypoint_index_;
    const geometry_msgs::Point desired_local = terminal_relay
        ? worldToFollower(desired_world.position)
        : useFollowerCruiseHeight(worldToFollower(desired_world.position));
    const geometry_msgs::Point& current_local = follower_odom_.pose.pose.position;
    const geometry_msgs::Point current_world = followerToWorld(current_local);
    const double horizontal_error =
        std::hypot(desired_local.x - current_local.x,
                   desired_local.y - current_local.y);
    const double vertical_error = std::fabs(desired_local.z - current_local.z);
    const double pass_projection =
        (current_world.x - desired_world.position.x) * std::cos(desired_world.yaw) +
        (current_world.y - desired_world.position.y) * std::sin(desired_world.yaw);
    const bool waypoint_arrival = !terminal_relay &&
        horizontal_error <= relay_arrive_radius_ &&
        vertical_error <= relay_arrive_z_tolerance_;
    const bool waypoint_passed = !terminal_relay &&
        std::isfinite(desired_world.yaw) && pass_projection >= 0.0;

    // 已到达或越过当前普通点时直接消费，不能再被旧点的间距保护拉回。
    if (!terminal_relay && !waypoint_arrival && !waypoint_passed &&
        enable_diff_separation_safety_) {
      if (!leaderOdomFresh(now)) {
        simple_waypoint_clearance_hold_active_ = true;
        setDiffWaitPositionHold(true, "waiting for fresh UAV0 odometry before FIFO release");
        diff_goal_published_ = false;
        publishState("DIFF_FIFO_WAIT_LEADER_CLEAR_NEXT_POINT", 1.0, 0.3, 0.0);
        return true;
      }
      const geometry_msgs::Point leader_world =
          leaderToWorld(leader_odom_.pose.pose.position);
      const double leader_to_waypoint =
          std::hypot(leader_world.x - desired_world.position.x,
                     leader_world.y - desired_world.position.y);
      if (leader_to_waypoint + 1.0e-6 < waypoint_release_min_separation_) {
        if (!simple_waypoint_clearance_hold_active_) {
          simple_waypoint_clearance_hold_active_ = true;
          setDiffWaitPositionHold(true, "UAV0 has not cleared next FIFO endpoint");
          diff_goal_published_ = false;
          diff_accepted_goal_valid_ = false;
          diff_command_seen_for_goal_ = false;
          ROS_WARN("[safe_follower] FIFO endpoint HOLD: UAV0 is only %.2fm beyond/from "
                   "the cached point, require %.2fm before UAV1 plans to it.",
                   leader_to_waypoint, waypoint_release_min_separation_);
        }
        publishState("DIFF_FIFO_WAIT_LEADER_CLEAR_NEXT_POINT", 1.0, 0.3, 0.0);
        return true;
      }
      if (simple_waypoint_clearance_hold_active_) {
        simple_waypoint_clearance_hold_active_ = false;
        diff_goal_published_ = false;
        simple_diff_retry_not_before_ = now;
        ROS_WARN("[safe_follower] UAV0 is %.2fm from the cached FIFO point; release UAV1 "
                 "planning after %.2fm clearance.",
                 leader_to_waypoint, waypoint_release_min_separation_);
      }

      // 同时保留两机实时XY间距保护。
      const double separation = std::hypot(leader_world.x - current_world.x,
                                           leader_world.y - current_world.y);
      if (separation + 1.0e-6 < min_separation_) {
        if (!simple_separation_hold_active_) {
          simple_separation_hold_active_ = true;
          setDiffWaitPositionHold(true, "FIFO spacing hold");
          diff_goal_published_ = false;
          diff_accepted_goal_valid_ = false;
          diff_command_seen_for_goal_ = false;
          ROS_WARN("[safe_follower] FIFO spacing HOLD %.2fm < %.2fm; keep the same endpoint.",
                   separation, min_separation_);
        }
        publishState("DIFF_FIFO_SPACING_HOLD", 1.0, 0.3, 0.0);
        return true;
      }
      if (simple_separation_hold_active_) {
        simple_separation_hold_active_ = false;
        diff_goal_published_ = false;
        simple_diff_retry_not_before_ = now;
        ROS_WARN("[safe_follower] FIFO spacing restored to %.2fm; retry same endpoint.",
                 separation);
      }
    }

    // 普通FIFO航点与run_swarm的逐点切换一致：进入到达邻域即可消费，
    // 或沿该点航向越过目标平面；最终降落点仍保留严格停稳判定。
    const bool terminal_arrival = terminal_relay &&
        horizontal_error <= terminal_arrive_radius_ &&
        vertical_error <= relay_arrive_z_tolerance_ &&
        follower_horizontal_speed_ <= relay_arrive_max_horizontal_speed_ &&
        follower_vertical_speed_ <= relay_arrive_max_vertical_speed_;
    if (waypoint_arrival || waypoint_passed || terminal_arrival) {
      if (terminal_relay) {
        if (relay_arrival_stamp_.isZero()) relay_arrival_stamp_ = now;
        if ((now - relay_arrival_stamp_).toSec() < relay_arrive_dwell_) return true;
        if (!follower_landing_requested_) {
          std_msgs::Bool request;
          request.data = true;
          follower_landing_request_pub_.publish(request);
          follower_landing_requested_ = true;
          setFollowerDetectionEnable(false, "follower landing requested");
          ROS_ERROR("[safe_follower] UAV1 Diff terminal reached; published AUTO.LAND request.");
        }
        return true;
      }

      ROS_ERROR("[safe_follower] UAV1 FIFO %s and consume endpoint 1/%zu "
                "at (%.2f,%.2f,%.2f).",
                waypoint_passed && !waypoint_arrival ? "PASSED" : "ARRIVED",
                relay_waypoints_.size(), desired_local.x, desired_local.y,
                desired_local.z);
      if (outside_wait_relay) {
        outside_wait_arrived_ = true;
        setFollowerDetectionEnable(false, "follower reached outside-door hover point");
        tryQueueFollowerTerminalTarget();
      }
      setDiffWaitPositionHold(true, "FIFO endpoint arrived");
      consumeSimpleRelayFront();
      relay_arrival_stamp_ = ros::Time(0);
      diff_goal_published_ = false;
      diff_plan_response_received_ = false;
      diff_accepted_goal_valid_ = false;
      diff_command_seen_for_goal_ = false;
      diff_goal_first_publish_stamp_ = ros::Time(0);
      diff_goal_index_ = std::numeric_limits<std::size_t>::max();
      simple_diff_retry_not_before_ = now;
      return true;
    }
    relay_arrival_stamp_ = ros::Time(0);

    const bool response_timeout = diff_goal_published_ &&
        !diff_plan_response_received_ && !diff_goal_first_publish_stamp_.isZero() &&
        (now - diff_goal_first_publish_stamp_).toSec() >= diff_goal_response_timeout_;
    const bool command_stale = diff_goal_published_ && diff_plan_response_received_ &&
        ((diff_command_seen_for_goal_ && !diff_command_stamp_.isZero() &&
          (now - diff_command_stamp_).toSec() >= diff_command_stale_timeout_) ||
         (!diff_command_seen_for_goal_ && !diff_goal_publish_stamp_.isZero() &&
          (now - diff_goal_publish_stamp_).toSec() >= diff_command_stale_timeout_));
    if (response_timeout || command_stale) {
      setDiffWaitPositionHold(true, response_timeout ? "FIFO Diff response timeout"
                                                    : "FIFO Diff command stale");
      diff_goal_published_ = false;
      diff_plan_response_received_ = false;
      diff_accepted_goal_valid_ = false;
      diff_command_seen_for_goal_ = false;
      diff_goal_first_publish_stamp_ = ros::Time(0);
      simple_diff_retry_not_before_ = now + ros::Duration(simple_diff_retry_delay_);
      ROS_ERROR("[safe_follower] UAV1 Diff %s; retry same FIFO endpoint after %.2fs.",
                response_timeout ? "RESPONSE TIMEOUT" : "COMMAND STALE",
                simple_diff_retry_delay_);
      return true;
    }

    if (!diff_goal_published_ && now >= simple_diff_retry_not_before_) {
      if (diff_goal_pub_.getNumSubscribers() == 0) {
        setDiffWaitPositionHold(true, "waiting for Diff subscriber");
        publishState("WAIT_UAV1_DIFF_SUBSCRIBER", 1.0, 0.4, 0.0);
        return true;
      }
      geometry_msgs::PoseStamped goal;
      goal.header.stamp = now;
      goal.header.frame_id = follower_odom_.header.frame_id.empty()
                                 ? world_frame_ : follower_odom_.header.frame_id;
      goal.pose.position = desired_local;
      const double local_yaw = worldYawToFollower(desired_world.yaw);
      goal.pose.orientation.w = std::cos(local_yaw * 0.5);
      goal.pose.orientation.z = std::sin(local_yaw * 0.5);
      stampDiffGoalId(&goal);
      diff_goal_pub_.publish(goal);
      diff_goal_index_ = active_relay_index_;
      diff_goal_published_ = true;
      diff_plan_response_received_ = false;
      diff_accepted_goal_valid_ = false;
      diff_command_seen_for_goal_ = false;
      diff_goal_publish_stamp_ = now;
      diff_goal_first_publish_stamp_ = now;
      publishTarget(desired_world.position);
      ROS_ERROR("[safe_follower] SEND UAV1 DIFF FIFO endpoint local=(%.2f,%.2f,%.2f), "
                "queued=%zu.", desired_local.x, desired_local.y, desired_local.z,
                relay_waypoints_.size());
    }
    publishState(terminal_relay ? "UAV1_DIFF_GO_TERMINAL"
                                : "UAV1_DIFF_GO_FIFO_ENDPOINT",
                 0.1, 0.8, 1.0);
    return true;
  }

  bool getLaggedTarget(RoutePoint* target, std::size_t* route_index = nullptr) const {
    if (route_.size() < 2 || route_length_ < release_path_length_) return false;
    double remaining = follow_distance_;
    for (std::size_t i = route_.size() - 1; i > 0; --i) {
      const RoutePoint& newer = route_[i];
      const RoutePoint& older = route_[i - 1];
      const double segment = distance3d(newer.position, older.position);
      if (segment >= remaining && segment > 1e-6) {
        const double ratio_from_newer = remaining / segment;
        target->position = interpolate(newer.position, older.position, ratio_from_newer);
        target->yaw = std::atan2(newer.position.y - older.position.y,
                                 newer.position.x - older.position.x);
        target->progress = newer.progress +
                           ratio_from_newer * (older.progress - newer.progress);
        if (route_index) *route_index = i;
        return true;
      }
      remaining -= segment;
    }
    *target = route_.front();
    if (route_index) *route_index = 0;
    return true;
  }

  double nearestRouteProgress(const geometry_msgs::Point& position) const {
    double progress = 0.0;
    double nearest = std::numeric_limits<double>::infinity();
    for (const RoutePoint& point : route_) {
      const double distance = std::hypot(position.x - point.position.x,
                                         position.y - point.position.y);
      if (distance < nearest) {
        nearest = distance;
        progress = point.progress;
      }
    }
    return progress;
  }

  bool getRouteTrackingTarget(const geometry_msgs::Point& follower_world,
                              const RoutePoint& lagged_target, std::size_t lagged_index,
                              RoutePoint* target) {
    if (route_.empty()) return false;
    lagged_index = std::min(lagged_index, route_.size() - 1);
    follower_route_index_ = std::min(follower_route_index_, lagged_index);

    // 2026-07-15: 只在尚未超过滞后终点的历史段内找最近点，并保持索引单调前进，
    // 防止U形通道两段空间相近时跳到错误分支或重新走旧路。
    std::size_t nearest_index = follower_route_index_;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = follower_route_index_; i <= lagged_index; ++i) {
      const double distance = std::hypot(route_[i].position.x - follower_world.x,
                                         route_[i].position.y - follower_world.y);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = i;
      }
    }
    follower_route_index_ = nearest_index;

    double remaining = std::max(0.08, route_tracking_lookahead_);
    geometry_msgs::Point cursor = follower_world;
    for (std::size_t i = nearest_index; i < lagged_index; ++i) {
      const geometry_msgs::Point& waypoint = route_[i].position;
      const double segment = distance3d(cursor, waypoint);
      if (segment >= remaining && segment > 1e-6) {
        target->position = interpolate(cursor, waypoint, remaining / segment);
        target->yaw = route_[i].yaw;
        return true;
      }
      remaining -= segment;
      cursor = waypoint;
    }

    const double final_segment = distance3d(cursor, lagged_target.position);
    if (final_segment > remaining && final_segment > 1e-6) {
      target->position = interpolate(cursor, lagged_target.position, remaining / final_segment);
      target->yaw = lagged_target.yaw;
    } else {
      *target = lagged_target;
    }
    return true;
  }

  bool getRelayRouteTarget(const geometry_msgs::Point& follower_world,
                           const RoutePoint& relay_target, RoutePoint* target) {
    if (route_.empty()) return false;

    // 2026-07-16: 接力点之间必须沿前机已经实飞的稠密折线前进。对外任务点可持续滚动增加，
    // 内部控制仍不允许把相邻任务点直接连线，否则U形/直角通道会从墙体中间切过去。
    std::size_t route_end_index = 0;
    std::size_t route_start_index = 0;
    const double segment_start_progress =
        active_relay_index_ > 0 ? relay_waypoints_[active_relay_index_ - 1].progress : 0.0;
    for (std::size_t i = 0; i < route_.size(); ++i) {
      if (route_[i].progress + path_sample_spacing_ < segment_start_progress) {
        route_start_index = i;
      }
      if (route_[i].progress > relay_target.progress + path_sample_spacing_) break;
      route_end_index = i;
    }
    // 2026-07-16: 每段只允许在“上一个接力点到当前接力点”的实飞轨迹中找最近点，
    // 防止找门阶段的旧轨迹与通道空间接近时跳回旧分支。
    follower_route_index_ = std::max(follower_route_index_, route_start_index);
    return getRouteTrackingTarget(follower_world, relay_target, route_end_index, target);
  }

  void pruneBlockedRouteCandidates(const ros::Time& now) {
    blocked_route_candidates_.erase(
        std::remove_if(blocked_route_candidates_.begin(),
                       blocked_route_candidates_.end(),
                       [&now](const BlockedRouteCandidate& candidate) {
                         return candidate.retry_after <= now;
                       }),
        blocked_route_candidates_.end());
  }

  bool routeCandidateBlocked(const geometry_msgs::Point& candidate_local,
                             double progress, const ros::Time& now) const {
    for (const BlockedRouteCandidate& blocked : blocked_route_candidates_) {
      if (blocked.retry_after <= now) continue;
      const bool same_progress = std::fabs(blocked.progress - progress) <=
                                 diff_candidate_blacklist_radius_;
      const bool same_position = distance3d(blocked.position, candidate_local) <=
                                 diff_candidate_blacklist_radius_;
      if (same_progress || same_position) return true;
    }
    return false;
  }

  void blacklistRouteCandidate(const geometry_msgs::Point& candidate_local,
                               double progress, const ros::Time& now,
                               const std::string& reason) {
    pruneBlockedRouteCandidates(now);
    for (BlockedRouteCandidate& blocked : blocked_route_candidates_) {
      if (std::fabs(blocked.progress - progress) <= diff_candidate_blacklist_radius_ ||
          distance3d(blocked.position, candidate_local) <=
              diff_candidate_blacklist_radius_) {
        blocked.retry_after = now + ros::Duration(diff_candidate_blacklist_duration_);
        return;
      }
    }
    BlockedRouteCandidate blocked;
    blocked.position = candidate_local;
    blocked.progress = progress;
    blocked.retry_after = now + ros::Duration(diff_candidate_blacklist_duration_);
    blocked_route_candidates_.push_back(blocked);
    ROS_WARN("[safe_follower] temporarily blacklist route candidate progress=%.2f "
             "local=(%.2f,%.2f,%.2f) for %.1fs reason=%s.",
             progress, candidate_local.x, candidate_local.y, candidate_local.z,
             diff_candidate_blacklist_duration_, reason.c_str());
  }

  bool routePointAtProgress(double progress, RoutePoint* target) const {
    if (route_.empty()) return false;
    progress = std::max(route_.front().progress,
                        std::min(progress, route_.back().progress));
    if (progress <= route_.front().progress + 1.0e-6) {
      *target = route_.front();
      return true;
    }
    for (std::size_t i = 1; i < route_.size(); ++i) {
      if (route_[i].progress + 1.0e-6 < progress) continue;
      const double span = route_[i].progress - route_[i - 1].progress;
      const double ratio = span > 1.0e-6
                               ? std::max(0.0, std::min(1.0,
                                     (progress - route_[i - 1].progress) / span))
                               : 0.0;
      target->position = interpolate(route_[i - 1].position,
                                     route_[i].position, ratio);
      target->yaw = route_[i].yaw;
      target->progress = progress;
      return true;
    }
    *target = route_.back();
    return true;
  }

  bool selectForwardRouteCandidate(const geometry_msgs::Point& follower_world,
                                   const RoutePoint& relay_target,
                                   const ros::Time& now,
                                   RoutePoint* target,
                                   bool allow_beyond_relay = true) {
    if (route_.empty()) return false;
    pruneBlockedRouteCandidates(now);
    const double segment_start_progress = active_relay_index_ > 0
        ? relay_waypoints_[active_relay_index_ - 1].progress
        : route_.front().progress;
    const double relay_progress = std::min(relay_target.progress,
                                            route_.back().progress);
    std::size_t route_start_index = 0;
    while (route_start_index + 1 < route_.size() &&
           route_[route_start_index].progress + path_sample_spacing_ <
               segment_start_progress) {
      ++route_start_index;
    }
    const std::size_t search_start_index = diff_allow_route_backtrack_attachment_
        ? route_start_index
        : std::max(route_start_index,
                   std::min(follower_route_index_, route_.size() - 1));
    const double attachment_distance = std::hypot(
        route_[search_start_index].position.x - follower_world.x,
        route_[search_start_index].position.y - follower_world.y);
    // 接力点只表示已经完成到哪一段，不能把候选窗口截死在接力坐标上。
    // 当前接力点被占据时，允许在同一个1.5m窗口内选它后方的安全历史点。
    const double plausible_progress_upper = std::min(
        route_.back().progress + path_sample_spacing_,
        route_[search_start_index].progress + attachment_distance +
            diff_history_target_max_distance_ + 2.0 * path_sample_spacing_);
    std::size_t nearest_index = search_start_index;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = search_start_index; i < route_.size(); ++i) {
      if (route_[i].progress > plausible_progress_upper) break;
      const double distance = std::hypot(route_[i].position.x - follower_world.x,
                                         route_[i].position.y - follower_world.y);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = i;
      }
    }
    if (!std::isfinite(nearest_distance)) return false;
    follower_route_index_ = nearest_index;
    diff_allow_route_backtrack_attachment_ = false;
    const double current_progress = std::max(segment_start_progress,
                                             route_[nearest_index].progress);
    const double candidate_progress_limit = allow_beyond_relay
        ? route_.back().progress : relay_progress;
    const double maximum_candidate_progress = std::min(
        candidate_progress_limit, current_progress + diff_history_target_max_distance_);

    auto accept_candidate = [&](const RoutePoint& candidate,
                                const char* source) -> bool {
      const geometry_msgs::Point candidate_local =
          useFollowerCruiseHeight(worldToFollower(candidate.position));
      if (routeCandidateBlocked(candidate_local, candidate.progress, now)) return false;
      if (distance3d(candidate_local, follower_odom_.pose.pose.position) <=
          diff_endpoint_capture_radius_) {
        return false;
      }
      int occupied_hits = 0;
      if (relayPointOccupied(candidate_local, &occupied_hits)) {
        blacklistRouteCandidate(candidate_local, candidate.progress, now,
                                "point occupied in follower cloud");
        return false;
      }
      *target = candidate;
      ROS_WARN("[safe_follower] select %s UAV0-history target progress %.2f->%.2f "
               "route_distance=%.2fm milestone=%.2f world=(%.2f,%.2f,%.2f).",
               source, current_progress, candidate.progress,
               candidate.progress - current_progress, relay_progress,
               candidate.position.x, candidate.position.y, candidate.position.z);
      return true;
    };

    // UAV0确实到达并停稳的B-spline段终点优先。倒序搜索可在1.5m窗口内
    // 尽量跨过弯角；同时用对应progress处的实飞折线复核，防止误用未执行规划点。
    for (auto endpoint = leader_segment_endpoints_.rbegin();
         endpoint != leader_segment_endpoints_.rend(); ++endpoint) {
      if (endpoint->progress <= current_progress + 0.05 ||
          endpoint->progress > maximum_candidate_progress + 1.0e-6) {
        continue;
      }
      RoutePoint route_match;
      if (!routePointAtProgress(endpoint->progress, &route_match)) continue;
      const double route_match_error = std::hypot(
          route_match.position.x - endpoint->position.x,
          route_match.position.y - endpoint->position.y);
      if (route_match_error >
          leader_segment_endpoint_radius_ + 2.0 * path_sample_spacing_) {
        continue;
      }
      RoutePoint candidate = *endpoint;
      candidate.position = followerCruisePointToWorld(candidate.position);
      if (accept_candidate(candidate, "stopped-segment-endpoint")) return true;
    }

    // 没有可用段终点时，仍只沿UAV0实飞折线取点；由远到近逐级尝试，
    // 让Diff有足够路径长度完成弯道A*，同时保证单次目标不超过1.5m。
    const std::array<double, 7> step_ratios =
        {{1.00, 0.83, 0.67, 0.50, 0.33, 0.20, 0.12}};
    std::vector<double> tried_progress;
    for (double ratio : step_ratios) {
      const double step = std::max(0.18,
          diff_history_target_max_distance_ * ratio);
      const double candidate_progress = std::min(maximum_candidate_progress,
                                                  current_progress + step);
      bool duplicate = false;
      for (double tried : tried_progress) {
        if (std::fabs(tried - candidate_progress) < 0.03) duplicate = true;
      }
      if (duplicate || candidate_progress <= current_progress + 0.05) continue;
      tried_progress.push_back(candidate_progress);
      RoutePoint candidate;
      if (!routePointAtProgress(candidate_progress, &candidate)) continue;
      if (accept_candidate(candidate, "fallback")) return true;
    }
    return false;
  }

  bool getFollowerHistoryRetreatTarget(const geometry_msgs::Point& follower_local,
                                       const ros::Time& now, RoutePoint* target) {
    if (follower_executed_route_.size() < 2) return false;
    pruneBlockedRouteCandidates(now);
    const geometry_msgs::Point follower_world = followerToWorld(follower_local);
    std::size_t nearest_index = follower_executed_route_.size() - 1;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t offset = 0; offset < follower_executed_route_.size(); ++offset) {
      const std::size_t index = follower_executed_route_.size() - 1 - offset;
      const double distance = std::hypot(
          follower_executed_route_[index].position.x - follower_world.x,
          follower_executed_route_[index].position.y - follower_world.y);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = index;
      }
      if (offset > 30 && nearest_distance <= relay_occupied_attachment_radius_) break;
    }
    if (nearest_distance > std::max(0.50, relay_occupied_attachment_radius_)) return false;

    const std::array<double, 5> retreat_distances = {{
        diff_failure_retreat_distance_, 0.25, 0.55, 0.18, 0.70}};
    for (double desired_retreat : retreat_distances) {
      double accumulated = 0.0;
      for (std::size_t i = nearest_index; i > 0; --i) {
        accumulated += distance3d(follower_executed_route_[i].position,
                                  follower_executed_route_[i - 1].position);
        if (accumulated + 1.0e-6 < desired_retreat) continue;
        RoutePoint candidate = follower_executed_route_[i - 1];
        const geometry_msgs::Point candidate_local =
            worldToFollower(candidate.position);
        if (distance3d(candidate_local, follower_local) <=
            diff_failure_retreat_arrive_radius_ + 0.05)
          break;
        if (routeCandidateBlocked(candidate_local, -candidate.progress, now)) break;
        *target = candidate;
        diff_follower_retreat_target_index_ = i - 1;
        return true;
      }
    }
    return false;
  }

  geometry_msgs::Point limitTargetStep(const geometry_msgs::Point& current,
                                        const geometry_msgs::Point& target) const {
    const double distance = distance3d(current, target);
    if (distance <= max_target_step_ || distance < 1e-6) return target;
    return interpolate(current, target, max_target_step_ / distance);
  }

  bool getSeparationRecoveryTarget(const geometry_msgs::Point& leader_world,
                                   const geometry_msgs::Point& follower_world,
                                   RoutePoint* target) const {
    const double current_separation =
        std::hypot(leader_world.x - follower_world.x, leader_world.y - follower_world.y);
    double best_score = -std::numeric_limits<double>::infinity();
    bool found = false;

    // 2026-07-14: 优先退到前机已经飞过的邻近历史点；这比直接沿几何反方向横穿窄道更安全。
    for (const RoutePoint& candidate : route_) {
      const double move = std::hypot(candidate.position.x - follower_world.x,
                                     candidate.position.y - follower_world.y);
      const double candidate_separation =
          std::hypot(candidate.position.x - leader_world.x,
                     candidate.position.y - leader_world.y);
      if (move < 0.06 || move > emergency_retreat_step_ + 1e-3 ||
          candidate_separation < current_separation + 0.10)
        continue;
      const double score = candidate_separation - 0.30 * move;
      if (score > best_score) {
        *target = candidate;
        target->position.z = follower_world.z;
        best_score = score;
        found = true;
      }
    }
    if (found) return true;

    // 2026-07-14: 历史轨迹没有近邻退让点时只生成一个短的远离前机目标，后续仍需通过后机点云走廊检查。
    if (current_separation < 1e-3) return false;
    target->position = follower_world;
    target->position.x += emergency_retreat_step_ *
                          (follower_world.x - leader_world.x) / current_separation;
    target->position.y += emergency_retreat_step_ *
                          (follower_world.y - leader_world.y) / current_separation;
    target->yaw = yawFromQuaternion(follower_odom_.pose.pose.orientation);
    return true;
  }

  bool segmentBlocked(const geometry_msgs::Point& current_local,
                      const geometry_msgs::Point& target_local, int* hit_count) const {
    *hit_count = 0;
    if (!obstacle_check_enabled_ || !follower_cloud_) return false;
    const double dx = target_local.x - current_local.x;
    const double dy = target_local.y - current_local.y;
    const double length_sq = dx * dx + dy * dy;
    if (length_sq < 1e-6) return false;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*follower_cloud_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*follower_cloud_, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*follower_cloud_, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) continue;
        const double from_current = std::hypot(*iter_x - current_local.x, *iter_y - current_local.y);
        if (from_current < obstacle_ignore_near_) continue;
        double projection = ((*iter_x - current_local.x) * dx +
                             (*iter_y - current_local.y) * dy) / length_sq;
        if (projection < 0.0 || projection > 1.0) continue;
        const double closest_x = current_local.x + projection * dx;
        const double closest_y = current_local.y + projection * dy;
        if (std::hypot(*iter_x - closest_x, *iter_y - closest_y) > obstacle_radius_) continue;
        const double path_z = current_local.z + projection * (target_local.z - current_local.z);
        if (std::fabs(*iter_z - path_z) > obstacle_z_margin_) continue;
        if (++(*hit_count) >= obstacle_min_points_) return true;
      }
    } catch (const std::runtime_error& error) {
      ROS_ERROR_THROTTLE(1.0, "[safe_follower] invalid PointCloud2 fields: %s", error.what());
      return true;
    }
    return false;
  }

  // 检查后机短目标线段与LDOP当前/预测位置的扫掠冲突。
  bool dynamicSegmentBlocked(const geometry_msgs::Point& current_local,
                             const geometry_msgs::Point& target_local,
                             uint32_t* obstacle_id) const {
    if (!enable_dynamic_obstacle_detection_ ||
        retained_dynamic_obstacles_.empty() ||
        retained_dynamic_obstacle_stamp_.isZero() ||
        (ros::Time::now() - retained_dynamic_obstacle_stamp_).toSec() >
            dynamic_obstacle_retention_) {
      return false;
    }
    const double dx = target_local.x - current_local.x;
    const double dy = target_local.y - current_local.y;
    const double length_sq = dx * dx + dy * dy;
    if (length_sq < 1e-6) return false;

    for (const auto& obstacle : retained_dynamic_obstacles_) {
      const double object_radius =
          0.5 * std::max(obstacle.size.x, obstacle.size.y) +
          dynamic_obstacle_safety_radius_;
      const double object_z_half =
          0.5 * obstacle.size.z + dynamic_obstacle_z_margin_;
      const auto conflicts = [&](const geometry_msgs::Point& point) {
        const double projection = std::max(
            0.0, std::min(1.0, ((point.x - current_local.x) * dx +
                                (point.y - current_local.y) * dy) / length_sq));
        const double closest_x = current_local.x + projection * dx;
        const double closest_y = current_local.y + projection * dy;
        const double path_z = current_local.z +
                              projection * (target_local.z - current_local.z);
        return std::hypot(point.x - closest_x, point.y - closest_y) <= object_radius &&
               std::fabs(point.z - path_z) <= object_z_half;
      };
      if (conflicts(obstacle.position)) {
        if (obstacle_id) *obstacle_id = obstacle.id;
        return true;
      }
      for (const geometry_msgs::Point& predicted : obstacle.predicted_positions) {
        // 2026-07-28: 摆球预测只能在通道内部有效；匀速/加速度外推越出中心带时截断，不让虚假穿墙预测长期停车。
        if (!route_.empty() &&
            distanceToAcceptedRoute(followerToWorld(predicted)) >
                dynamic_retention_route_half_width_)
          continue;
        if (conflicts(predicted)) {
          if (obstacle_id) *obstacle_id = obstacle.id;
          return true;
        }
      }
    }
    return false;
  }

  // 2026-07-27: 脱困选向不能沿用“命中3点立即返回”，需要统计各候选短段的完整障碍点数进行比较。
  int countSegmentHits(const geometry_msgs::Point& current_local,
                       const geometry_msgs::Point& target_local,
                       double near_ignore) const {
    if (!follower_cloud_) return std::numeric_limits<int>::max() / 4;
    const double dx = target_local.x - current_local.x;
    const double dy = target_local.y - current_local.y;
    const double length_sq = dx * dx + dy * dy;
    if (length_sq < 1e-6) return std::numeric_limits<int>::max() / 4;
    int hits = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*follower_cloud_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*follower_cloud_, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*follower_cloud_, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) continue;
        if (std::hypot(*iter_x - current_local.x, *iter_y - current_local.y) < near_ignore) continue;
        const double projection = ((*iter_x - current_local.x) * dx +
                                   (*iter_y - current_local.y) * dy) / length_sq;
        if (projection < 0.0 || projection > 1.0) continue;
        const double closest_x = current_local.x + projection * dx;
        const double closest_y = current_local.y + projection * dy;
        if (std::hypot(*iter_x - closest_x, *iter_y - closest_y) > obstacle_radius_) continue;
        const double path_z = current_local.z + projection * (target_local.z - current_local.z);
        if (std::fabs(*iter_z - path_z) <= obstacle_z_margin_) ++hits;
      }
    } catch (const std::runtime_error& error) {
      ROS_ERROR_THROTTLE(1.0, "[safe_follower] recovery cloud fields invalid: %s", error.what());
      return std::numeric_limits<int>::max() / 4;
    }
    return hits;
  }

  // 2026-07-27: 从原指令左右、前后及斜向八个短目标中选点云命中最少者；每次失败旋转优先级避免重复顶墙。
  void selectRecoveryTarget(const ros::Time& now) {
    const geometry_msgs::Point current = follower_odom_.pose.pose.position;
    double base_angle = std::atan2(last_command_dy_, last_command_dx_);
    if (std::hypot(last_command_dx_, last_command_dy_) < 1e-3)
      base_angle = yawFromQuaternion(follower_odom_.pose.pose.orientation);
    const double offsets[] = {M_PI_2, -M_PI_2, 0.0, M_PI, 3.0 * M_PI_4,
                              -3.0 * M_PI_4, M_PI_4, -M_PI_4};
    int best_hits = std::numeric_limits<int>::max();
    geometry_msgs::Point best = current;
    const double retry_rotation = recovery_attempt_count_ * M_PI_4;
    for (double offset : offsets) {
      const double angle = base_angle + offset + retry_rotation;
      geometry_msgs::Point candidate = current;
      candidate.x += recovery_step_ * std::cos(angle);
      candidate.y += recovery_step_ * std::sin(angle);
      candidate.z = std::max(follow_height_min_, std::min(follow_height_max_, current.z));
      const int hits = countSegmentHits(current, candidate, recovery_near_ignore_);
      if (hits < best_hits) {
        best_hits = hits;
        best = candidate;
      }
    }
    recovery_target_local_ = best;
    recovery_attempt_start_ = now;
    ++recovery_attempt_count_;
    ROS_ERROR("[safe_follower] STUCK recovery attempt=%d target=(%.2f,%.2f,%.2f) cloud_hits=%d.",
              recovery_attempt_count_, best.x, best.y, best.z, best_hits);
  }

  // 2026-07-27: 运动卡死和点云阻挡共用同一恢复入口，避免HOLD状态因没有速度指令永远进不了脱困。
  void startRecovery(const ros::Time& now, const char* reason) {
    if (recovery_active_) return;
    recovery_active_ = true;
    recovery_origin_local_ = follower_odom_.pose.pose.position;
    recovery_yaw_ = yawFromQuaternion(follower_odom_.pose.pose.orientation);
    recovery_attempt_count_ = 0;
    selectRecoveryTarget(now);
    ROS_ERROR("[safe_follower] STUCK recovery entered reason=%s.", reason);
  }

  // 2026-07-27: 用“有运动指令但实际无位移”识别物理卡死；恢复期间独占控制，移动成功后重新接入前机路线。
  bool handleStuckRecovery(const ros::Time& now) {
    const geometry_msgs::Point current = follower_odom_.pose.pose.position;
    if (recovery_active_) {
      if (std::hypot(current.x - recovery_origin_local_.x,
                     current.y - recovery_origin_local_.y) >= recovery_success_distance_) {
        ROS_WARN("[safe_follower] STUCK recovery succeeded after %d attempt(s).",
                 recovery_attempt_count_);
        recovery_active_ = false;
        motion_monitor_active_ = false;
        last_command_moving_ = false;
        return false;
      }
      if ((now - recovery_attempt_start_).toSec() >= recovery_attempt_timeout_)
        selectRecoveryTarget(now);
      publishCommand(recovery_target_local_, recovery_yaw_, true, recovery_speed_);
      publishState("STUCK_RECOVERY", 1.0, 0.2, 0.0);
      return true;
    }

    if (!last_command_moving_) {
      motion_monitor_active_ = false;
      return false;
    }
    if (!motion_monitor_active_) {
      motion_monitor_origin_local_ = current;
      motion_monitor_start_ = now;
      motion_monitor_active_ = true;
      return false;
    }
    const double progress = std::hypot(current.x - motion_monitor_origin_local_.x,
                                       current.y - motion_monitor_origin_local_.y);
    if (progress >= stuck_min_progress_) {
      motion_monitor_origin_local_ = current;
      motion_monitor_start_ = now;
      return false;
    }
    if ((now - motion_monitor_start_).toSec() < stuck_detection_timeout_) return false;

    startRecovery(now, "commanded motion without odometry progress");
    publishCommand(recovery_target_local_, recovery_yaw_, true, recovery_speed_);
    publishState("STUCK_DETECTED", 1.0, 0.0, 0.0);
    ROS_ERROR("[safe_follower] STUCK detected: command active for %.2fs but progress=%.3fm.",
              stuck_detection_timeout_, progress);
    return true;
  }

  // 2026-07-24: 后机收到的普通路线只有XY语义。默认在0.65m飞；若该高度的短线段
  // 被自身点云占据，才在允许的0.60--0.70m范围内试探上下层，避免照搬前机退化z。
  bool chooseClearFollowerHeight(const geometry_msgs::Point& current_local,
                                 geometry_msgs::Point* target_local,
                                 int* hit_count) const {
    if (!segmentBlocked(current_local, *target_local, hit_count)) return true;

    const double candidates[] = {follow_height_max_, follow_height_min_};
    for (const double candidate_height : candidates) {
      if (std::fabs(candidate_height - target_local->z) < 1e-3) continue;
      geometry_msgs::Point adjusted = *target_local;
      adjusted.z = candidate_height;
      int candidate_hits = 0;
      if (!segmentBlocked(current_local, adjusted, &candidate_hits)) {
        ROS_WARN_THROTTLE(
            1.0,
            "[safe_follower] XY route default z=%.2f blocked hits=%d; use local avoid z=%.2f.",
            fixed_follow_height_, *hit_count, candidate_height);
        *target_local = adjusted;
        *hit_count = 0;
        return true;
      }
    }
    return false;
  }

  // 2026-07-22: 线段可通行不代表末端可安全悬停；单独检查接力点周围圆柱净空，动态障碍离开后可自动恢复。
  bool endpointOccupied(const geometry_msgs::Point& target_local, int* hit_count) const {
    *hit_count = 0;
    if (!obstacle_check_enabled_ || !follower_cloud_) return false;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*follower_cloud_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*follower_cloud_, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*follower_cloud_, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) continue;
        if (std::hypot(*iter_x - target_local.x, *iter_y - target_local.y) >
            relay_endpoint_clearance_radius_)
          continue;
        if (std::fabs(*iter_z - target_local.z) > relay_endpoint_z_margin_) continue;
        if (++(*hit_count) >= relay_endpoint_min_points_) return true;
      }
    } catch (const std::runtime_error& error) {
      ROS_ERROR_THROTTLE(1.0, "[safe_follower] invalid endpoint PointCloud2 fields: %s",
                         error.what());
      return true;
    }
    return false;
  }

  // 普通接力点不是悬停/降落平台。这里只回答“点本身是否落入点云占据体素”，
  // 不要求狭窄通道在目标周围提供一整圈净空。真正运动安全仍由 segmentBlocked() 保证。
  bool relayPointOccupied(const geometry_msgs::Point& target_local, int* hit_count) const {
    *hit_count = 0;
    if (!obstacle_check_enabled_ || !follower_cloud_) return false;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*follower_cloud_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*follower_cloud_, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*follower_cloud_, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) continue;
        if (std::hypot(*iter_x - target_local.x, *iter_y - target_local.y) >
            relay_point_occupied_radius_)
          continue;
        if (std::fabs(*iter_z - target_local.z) > relay_point_occupied_z_margin_) continue;
        if (++(*hit_count) >= relay_point_occupied_min_points_) return true;
      }
    } catch (const std::runtime_error& error) {
      ROS_ERROR_THROTTLE(1.0, "[safe_follower] invalid relay-point cloud fields: %s",
                         error.what());
      return true;
    }
    return false;
  }

  bool relayGoalHasClearance(const geometry_msgs::Point& target_local,
                             int* hit_count) const {
    *hit_count = 0;
    if (!obstacle_check_enabled_ || !follower_cloud_) return true;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*follower_cloud_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*follower_cloud_, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*follower_cloud_, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) ||
            !std::isfinite(*iter_z))
          continue;
        if (std::hypot(*iter_x - target_local.x, *iter_y - target_local.y) >=
            relay_goal_clearance_radius_)
          continue;
        if (std::fabs(*iter_z - target_local.z) > relay_goal_clearance_z_margin_)
          continue;
        if (++(*hit_count) >= relay_goal_clearance_min_points_) return false;
      }
    } catch (const std::runtime_error& error) {
      ROS_ERROR_THROTTLE(1.0, "[safe_follower] invalid relay-goal cloud fields: %s",
                         error.what());
      return false;
    }
    return true;
  }

  bool chooseClearRelayGoal(const RoutePoint& requested_world,
                            RoutePoint* selected_world,
                            int* requested_hit_count) const {
    *selected_world = requested_world;
    const geometry_msgs::Point requested_local =
        useFollowerCruiseHeight(worldToFollower(requested_world.position));
    if (relayGoalHasClearance(requested_local, requested_hit_count)) return true;

    const double previous_progress =
        active_relay_index_ > 0 ? relay_waypoints_[active_relay_index_ - 1].progress
                                : -std::numeric_limits<double>::infinity();
    const geometry_msgs::Point& current_local = follower_odom_.pose.pose.position;
    for (auto iter = route_.rbegin(); iter != route_.rend(); ++iter) {
      const double backtrack = requested_world.progress - iter->progress;
      if (backtrack < path_sample_spacing_) continue;
      if (backtrack > relay_goal_backtrack_max_distance_ + 1e-6) break;
      // 不允许退到上一个已完成点，否则会把当前点瞬间判为到达并跳到下一点。
      if (iter->progress <= previous_progress + path_sample_spacing_) continue;

      RoutePoint candidate = *iter;
      candidate.position = followerCruisePointToWorld(iter->position);
      const geometry_msgs::Point candidate_local =
          useFollowerCruiseHeight(worldToFollower(candidate.position));
      if (std::hypot(candidate_local.x - current_local.x,
                     candidate_local.y - current_local.y) <= relay_arrive_radius_)
        continue;
      int candidate_hits = 0;
      if (!relayGoalHasClearance(candidate_local, &candidate_hits)) continue;
      *selected_world = candidate;
      return true;
    }
    return false;
  }

  void publishCommand(const geometry_msgs::Point& target_local, double target_yaw,
                      bool moving, double speed_limit = -1.0) {
    // 2026-07-28: Diff模式的唯一控制指令发布者必须是UAV1 traj_server；防止遗留分支意外形成双发布者。
    if (use_diff_planner_) {
      ROS_ERROR_THROTTLE(1.0, "[safe_follower] blocked legacy PositionCommand publish in Diff mode.");
      return;
    }
    // 2026-07-22: 一旦重新进入安全运动状态，允许下一次异常重新锁存新的最后安全悬停点。
    if (moving) hold_target_latched_ = false;
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = follower_odom_.header.frame_id;
    cmd.position = target_local;
    if (moving) {
      const geometry_msgs::Point& current = follower_odom_.pose.pose.position;
      const double dx = target_local.x - current.x;
      const double dy = target_local.y - current.y;
      const double dz = target_local.z - current.z;
      // 2026-07-14: 速度仅指向本周期限幅后的短目标，避免远端历史点造成速度突跳。
      const double horizontal = std::hypot(dx, dy);
      if (horizontal > 1e-4) {
        // 2026-07-14: 接近滞后点时按误差降速，不能以固定巡航速度穿过目标造成窄道过冲。
        const double configured_speed = speed_limit > 0.0 ? speed_limit : cruise_speed_;
        const double desired_speed = std::min(configured_speed, 0.8 * horizontal);
        cmd.velocity.x = desired_speed * dx / horizontal;
        cmd.velocity.y = desired_speed * dy / horizontal;
      }
      cmd.velocity.z = std::max(-max_vertical_speed_,
                                std::min(max_vertical_speed_, 0.8 * dz));
    }
    // 2026-07-27: 保存真实下发的水平运动方向和状态，供下一控制周期进行卡死判定与脱困选向。
    last_command_moving_ = moving && std::hypot(cmd.velocity.x, cmd.velocity.y) > 0.05;
    last_command_dx_ = cmd.velocity.x;
    last_command_dy_ = cmd.velocity.y;
    cmd.yaw = target_yaw;
    cmd.trajectory_id = ++trajectory_id_;
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    command_pub_.publish(cmd);
  }

  void hold(const std::string& reason) {
    if (!have_follower_odom_) return;
    // 2026-07-28: Diff模式等待目标时请求控制器锁存MAVROS本地位置；零速度心跳不足以抵抗惯性与估计偏移。
    if (use_diff_planner_) {
      setDiffWaitPositionHold(true, reason);
      publishState(reason, 1.0, 0.65, 0.0);
      ROS_WARN_THROTTLE(1.0, "[safe_follower] DIFF WAIT: %s", reason.c_str());
      return;
    }
    // 2026-07-22: HOLD首次触发时锁住最后安全位置，后续里程计漂移或碰撞掉高不能拖着悬停目标一起跑。
    if (!hold_target_latched_) {
      hold_target_local_ = follower_odom_.pose.pose.position;
      hold_target_yaw_ = yawFromQuaternion(follower_odom_.pose.pose.orientation);
      hold_target_latched_ = true;
      ROS_ERROR("[safe_follower] latched safe HOLD target=(%.2f, %.2f, %.2f).",
                hold_target_local_.x, hold_target_local_.y, hold_target_local_.z);
    }
    publishCommand(hold_target_local_, hold_target_yaw_, false);
    publishState(reason, 1.0, 0.65, 0.0);
    ROS_WARN_THROTTLE(1.0, "[safe_follower] HOLD: %s", reason.c_str());
  }

  void handleTerminalApproach(const ros::Time& now) {
    geometry_msgs::Point target_local = worldToFollower(terminal_target_world_.position);
    const geometry_msgs::Point& current_local = follower_odom_.pose.pose.position;
    const double horizontal_error =
        std::hypot(target_local.x - current_local.x, target_local.y - current_local.y);
    const double vertical_error = std::fabs(target_local.z - current_local.z);

    // 2026-07-22: 终点同样先检查停驻区域；二维码/动态障碍占据落点时原位等待而不是继续下发目标。
    if (obstacle_check_enabled_) {
      if (require_fresh_cloud_ &&
          (!follower_cloud_ || (now - cloud_stamp_).toSec() > cloud_timeout_)) {
        terminal_arrival_stamp_ = ros::Time(0);
        hold("terminal endpoint cloud stale");
        return;
      }
      int endpoint_hits = 0;
      if (endpointOccupied(target_local, &endpoint_hits)) {
        terminal_arrival_stamp_ = ros::Time(0);
        hold("terminal endpoint occupied by follower cloud");
        ROS_ERROR_THROTTLE(1.0,
                           "[safe_follower] terminal endpoint occupied hits=%d; wait until clear.",
                           endpoint_hits);
        return;
      }
    }

    if (horizontal_error <= terminal_arrive_radius_ &&
        vertical_error <= terminal_arrive_z_tolerance_ &&
        follower_horizontal_speed_ <= relay_arrive_max_horizontal_speed_ &&
        follower_vertical_speed_ <= relay_arrive_max_vertical_speed_) {
      if (terminal_arrival_stamp_.isZero()) terminal_arrival_stamp_ = now;
      publishCommand(target_local, worldYawToFollower(terminal_target_world_.yaw), false);
      publishState("TERMINAL_DWELL", 0.2, 1.0, 0.2);
      if (!follower_landing_requested_ &&
          (now - terminal_arrival_stamp_).toSec() >= terminal_arrive_dwell_) {
        // 2026-07-14: 后机只在独立落点稳定到达后请求自身控制器切换 AUTO.LAND。
        std_msgs::Bool request;
        request.data = true;
        follower_landing_request_pub_.publish(request);
        follower_landing_requested_ = true;
        ROS_ERROR("[safe_follower] follower terminal target reached; published AUTO.LAND request.");
      }
      return;
    }
    terminal_arrival_stamp_ = ros::Time(0);

    geometry_msgs::Point short_target = limitTargetStep(current_local, target_local);
    if (obstacle_check_enabled_) {
      if (require_fresh_cloud_ &&
          (!follower_cloud_ || (now - cloud_stamp_).toSec() > cloud_timeout_)) {
        hold("terminal approach cloud stale");
        return;
      }
      int obstacle_hits = 0;
      if (segmentBlocked(current_local, short_target, &obstacle_hits)) {
        hold("terminal approach blocked by follower cloud");
        return;
      }
    }
    publishCommand(short_target, worldYawToFollower(terminal_target_world_.yaw), true,
                   terminal_approach_speed_);
    publishTarget(terminal_target_world_.position);
    publishState("TERMINAL_APPROACH_NO_SEPARATION", 0.2, 1.0, 0.2);
    ROS_WARN_THROTTLE(1.0,
                      "[safe_follower] TERMINAL approach error_xy=%.2fm error_z=%.2fm; "
                      "leader spacing check disabled.",
                      horizontal_error, vertical_error);
  }

  bool handleContinuousFollowBeforeExit(const ros::Time& now) {
    if (!continuous_follow_before_exit_ || leader_outside_exit_ || terminal_mode_active_ ||
        !door_waypoint_released_ || !leaderOdomFresh(now))
      return false;

    RoutePoint lagged_world;
    std::size_t lagged_index = 0;
    if (!getLaggedTarget(&lagged_world, &lagged_index)) return false;

    const geometry_msgs::Point follower_world =
        followerToWorld(follower_odom_.pose.pose.position);
    // 接力队列严格按到达条件消费。即使切回连续跟踪，也不能按路线进度批量跳过缓存点。

    const geometry_msgs::Point leader_world =
        leaderToWorld(leader_odom_.pose.pose.position);
    const double separation = std::hypot(leader_world.x - follower_world.x,
                                         leader_world.y - follower_world.y);
    if (separation <= min_separation_) {
      hold("continuous follow minimum separation");
      return true;
    }

    RoutePoint tracking_world;
    if (!getRouteTrackingTarget(follower_world, lagged_world, lagged_index,
                                &tracking_world)) {
      hold("continuous leader route unavailable");
      return true;
    }
    geometry_msgs::Point target_local =
        useFollowerCruiseHeight(worldToFollower(tracking_world.position));
    target_local = limitTargetStep(follower_odom_.pose.pose.position, target_local);
    if (obstacle_check_enabled_) {
      if (require_fresh_cloud_ &&
          (!follower_cloud_ || (now - cloud_stamp_).toSec() > cloud_timeout_)) {
        hold("continuous follow cloud stale");
        return true;
      }
      // 2026-07-28: 动态摆球采用当前框+1秒预测让行，不能等稀疏注册点云恰好命中3点后才刹车。
      uint32_t dynamic_id = 0;
      if (dynamicSegmentBlocked(follower_odom_.pose.pose.position, target_local,
                                &dynamic_id)) {
        continuous_blocked_since_ = ros::Time(0);
        hold("predicted dynamic obstacle crossing continuous path");
        ROS_WARN_THROTTLE(0.5,
                          "[safe_follower] DYNAMIC HOLD id=%u intersects current/predicted segment.",
                          dynamic_id);
        return true;
      }
      int obstacle_hits = 0;
      if (!chooseClearFollowerHeight(follower_odom_.pose.pose.position, &target_local,
                                     &obstacle_hits)) {
        // 2026-07-27: 连续跟随没有可跳过的固定检查点；局部段持续阻挡后直接进入点云选向恢复，
        // 恢复成功后下一周期会从前机稠密路线重新计算新的滚动目标。
        if (continuous_blocked_since_.isZero()) continuous_blocked_since_ = now;
        if ((now - continuous_blocked_since_).toSec() >= blocked_recovery_timeout_ &&
            !recovery_active_)
          startRecovery(now, "continuous follow path blocked while holding");
        hold("continuous follow local path blocked");
        return true;
      }
    }
    continuous_blocked_since_ = ros::Time(0);
    publishCommand(target_local, worldYawToFollower(tracking_world.yaw), true,
                   continuous_follow_speed_);
    publishTarget(lagged_world.position);
    publishState("CONTINUOUS_FOLLOW_0P7M_XY", 0.1, 0.85, 1.0);
    ROS_INFO_THROTTLE(1.0,
                      "[safe_follower] CONTINUOUS separation=%.2fm lag_error=%.2fm "
                      "target_local=(%.2f,%.2f,%.2f).",
                      separation,
                      std::hypot(lagged_world.position.x - follower_world.x,
                                 lagged_world.position.y - follower_world.y),
                      target_local.x, target_local.y, target_local.z);
    return true;
  }

  // Diff按离散目标独立飞行时也必须持续看两机实时XY。0.70m内先丢弃旧轨迹锁点；
  // 若前机继续后退压到0.50m内，则沿前机已飞路线（无候选时沿几何反方向）逐步退让。
  bool handleDiffSeparationSafety(const ros::Time& now) {
    const bool terminal_relay_active = terminal_mode_active_ &&
        active_relay_index_ == terminal_waypoint_index_;
    const bool leader_fresh = leaderOdomFresh(now);
    if (!enable_diff_separation_safety_ || terminal_relay_active || !leader_fresh) {
      if (diff_separation_hold_active_) {
        setDiffWaitPositionHold(
            true, leader_fresh ? "leave separation safety for terminal relay"
                               : "leader odometry stale; continue cached history only");
        diff_separation_hold_active_ = false;
        separation_recovery_active_ = false;
        separation_recovery_goal_valid_ = false;
        diff_goal_published_ = false;
        diff_plan_response_received_ = false;
        diff_accepted_goal_valid_ = false;
        diff_command_seen_for_goal_ = false;
      }
      return false;
    }

    const geometry_msgs::Point leader_world =
        leaderToWorld(leader_odom_.pose.pose.position);
    const geometry_msgs::Point follower_world =
        followerToWorld(follower_odom_.pose.pose.position);
    const double separation = std::hypot(leader_world.x - follower_world.x,
                                         leader_world.y - follower_world.y);

    if (!diff_separation_hold_active_ && separation >= min_separation_) return false;

    if (!diff_separation_hold_active_) {
      diff_separation_hold_active_ = true;
      separation_recovery_active_ = separation <= separation_recovery_distance_ && relay_route_paused_;
      separation_recovery_goal_valid_ = false;
      setDiffWaitPositionHold(true, "UAV0-UAV1 separation safety");
      // safety_hold在控制器侧立即丢弃旧轨迹；后续必须等新退让/原航点轨迹确认后才能解锁。
      diff_goal_published_ = false;
      diff_plan_response_received_ = false;
      diff_accepted_goal_valid_ = false;
      diff_command_seen_for_goal_ = false;
      ROS_ERROR("[safe_follower] DIFF SEPARATION HOLD active: XY=%.2fm <= %.2fm; "
                "retreat trigger=%.2fm release=%.2fm.", separation, min_separation_,
                separation_recovery_distance_, separation_release_distance_);
    }

    if (separation >= separation_release_distance_) {
      const bool retreated = separation_recovery_active_;
      diff_separation_hold_active_ = false;
      separation_recovery_active_ = false;
      separation_recovery_goal_valid_ = false;
      diff_goal_published_ = false;
      diff_plan_response_received_ = false;
      diff_accepted_goal_valid_ = false;
      diff_command_seen_for_goal_ = false;
      // 保持锁点，正常Diff分支将在本周期重新发布原航点，并只在新轨迹成功后解锁。
      setDiffWaitPositionHold(true, "separation restored; wait fresh Diff trajectory");
      ROS_WARN("[safe_follower] DIFF SEPARATION %s complete: XY=%.2fm >= %.2fm; "
               "replan active relay.", retreated ? "RETREAT" : "HOLD", separation,
               separation_release_distance_);
      return false;
    }

    if (!separation_recovery_active_ && relay_route_paused_ &&
        separation <= separation_recovery_distance_) {
      separation_recovery_active_ = true;
      separation_recovery_goal_valid_ = false;
      diff_goal_published_ = false;
      diff_plan_response_received_ = false;
      diff_accepted_goal_valid_ = false;
      diff_command_seen_for_goal_ = false;
      setDiffWaitPositionHold(true, "separation below retreat trigger");
      ROS_ERROR("[safe_follower] DIFF SEPARATION RETREAT triggered: XY=%.2fm <= %.2fm.",
                separation, separation_recovery_distance_);
    }

    if (!separation_recovery_active_) {
      setDiffWaitPositionHold(true, "separation below hold distance");
      publishState("DIFF_SEPARATION_HOLD_0P7M", 1.0, 0.35, 0.0);
      ROS_WARN_THROTTLE(0.5,
                        "[safe_follower] DIFF SEPARATION HOLD XY=%.2fm; wait %.2fm.",
                        separation, separation_release_distance_);
      return true;
    }

    const geometry_msgs::Point& current_local = follower_odom_.pose.pose.position;
    if (separation_recovery_goal_valid_) {
      const geometry_msgs::Point recovery_arrival_goal =
          diff_accepted_goal_valid_ ? diff_accepted_goal_local_
                                    : separation_recovery_goal_local_;
      const double recovery_error =
          std::hypot(recovery_arrival_goal.x - current_local.x,
                     recovery_arrival_goal.y - current_local.y);
      const bool retry_due = diff_goal_published_ && !diff_plan_response_received_ &&
          !diff_goal_publish_stamp_.isZero() &&
          (now - diff_goal_publish_stamp_).toSec() >= diff_goal_retry_period_;
      const bool command_stale = diff_goal_published_ && diff_plan_response_received_ &&
          diff_command_seen_for_goal_ && !diff_command_stamp_.isZero() &&
          (now - diff_command_stamp_).toSec() >= diff_command_stale_timeout_;
      if (recovery_error <= diff_failure_retreat_arrive_radius_) {
        setDiffWaitPositionHold(true, "separation retreat step reached");
        separation_recovery_goal_valid_ = false;
        diff_goal_published_ = false;
        diff_plan_response_received_ = false;
        diff_accepted_goal_valid_ = false;
        diff_command_seen_for_goal_ = false;
        ROS_WARN("[safe_follower] DIFF SEPARATION RETREAT step reached; XY=%.2fm, "
                 "select next step toward %.2fm.", separation,
                 separation_release_distance_);
      } else if (retry_due || command_stale) {
        setDiffWaitPositionHold(true, retry_due ? "separation retreat planning timeout"
                                                : "separation retreat command stale");
        separation_recovery_goal_valid_ = false;
        diff_goal_published_ = false;
        diff_plan_response_received_ = false;
        diff_accepted_goal_valid_ = false;
        diff_command_seen_for_goal_ = false;
        ROS_ERROR("[safe_follower] UAV1 Diff separation retreat %s before recovery "
                  "(error=%.2fm); select a fresh retreat step.",
                  retry_due ? "response timeout" : "command stale", recovery_error);
      } else {
        publishState("DIFF_SEPARATION_RETREAT", 1.0, 0.15, 0.0);
        return true;
      }
    }

    RoutePoint retreat_world;
    if (!getSeparationRecoveryTarget(leader_world, follower_world, &retreat_world)) {
      setDiffWaitPositionHold(true, "no valid separation retreat target");
      publishState("DIFF_SEPARATION_RETREAT_NO_TARGET", 1.0, 0.0, 0.0);
      ROS_ERROR_THROTTLE(0.5,
                         "[safe_follower] cannot select separation retreat target at "
                         "XY=%.2fm; keep HOLD.", separation);
      return true;
    }

    separation_recovery_goal_local_ =
        useFollowerCruiseHeight(worldToFollower(retreat_world.position));
    separation_recovery_goal_valid_ = true;
    const geometry_msgs::Point retreat_probe =
        limitTargetStep(current_local, separation_recovery_goal_local_);
    uint32_t dynamic_id = 0;
    if (dynamicSegmentBlocked(current_local, retreat_probe, &dynamic_id)) {
      diff_dynamic_hold_active_ = true;
      setDiffWaitPositionHold(true, "dynamic obstacle blocks separation retreat");
      separation_recovery_goal_valid_ = false;
      publishState("DIFF_SEPARATION_RETREAT_DYNAMIC_HOLD", 1.0, 0.0, 0.0);
      ROS_ERROR_THROTTLE(
          0.5, "[safe_follower] separation retreat blocked by dynamic id=%u; keep HOLD.",
          dynamic_id);
      return true;
    }
    // 旧前进方向上的动态HOLD不能永久阻止已经验证为清空的反向退让；等待锁点仍保持到新轨迹成功。
    diff_dynamic_hold_active_ = false;
    if (diff_goal_pub_.getNumSubscribers() == 0) {
      setDiffWaitPositionHold(true, "waiting for Diff subscriber for separation retreat");
      publishState("DIFF_SEPARATION_RETREAT_WAIT_SUBSCRIBER", 1.0, 0.2, 0.0);
      separation_recovery_goal_valid_ = false;
      return true;
    }

    geometry_msgs::PoseStamped goal;
    goal.header.stamp = now;
    goal.header.frame_id = follower_odom_.header.frame_id.empty()
                               ? world_frame_ : follower_odom_.header.frame_id;
    goal.pose.position = separation_recovery_goal_local_;
    const double local_yaw = worldYawToFollower(retreat_world.yaw);
    goal.pose.orientation.w = std::cos(local_yaw * 0.5);
    goal.pose.orientation.z = std::sin(local_yaw * 0.5);
    stampDiffGoalId(&goal);
    diff_goal_pub_.publish(goal);
    diff_goal_index_ = active_relay_index_;
    diff_goal_published_ = true;
    diff_plan_response_received_ = false;
    diff_accepted_goal_valid_ = false;
    diff_command_seen_for_goal_ = false;
    diff_goal_publish_stamp_ = now;
    publishTarget(retreat_world.position);
    publishState("DIFF_SEPARATION_RETREAT", 1.0, 0.15, 0.0);
    ROS_ERROR("[safe_follower] SEND UAV1 DIFF SEPARATION RETREAT local=(%.2f,%.2f,%.2f) "
              "XY=%.2fm target_release=%.2fm.", separation_recovery_goal_local_.x,
              separation_recovery_goal_local_.y, separation_recovery_goal_local_.z,
              separation, separation_release_distance_);
    return true;
  }

  // 2026-07-28: 将前机释放的离散接力点交给UAV1自身Diff；这里只管理顺序、到达和动态紧停，不生成飞行轨迹。
  bool handleDiffPlannerExecution(const ros::Time& now) {
    if (!use_diff_planner_) return false;
    if (simple_segment_endpoint_following_)
      return handleSimpleDiffPlannerExecution(now);
    if (handleDiffSeparationSafety(now)) return true;
    if (active_relay_index_ >= relay_waypoints_.size()) {
      // 2026-07-28: 上一点消费后到下一点释放前保持同一个物理锁点，不能让等待位置随里程计漂移重置。
      setDiffWaitPositionHold(true, "waiting for next relay waypoint");
      const std::string wait_state =
          outside_wait_arrived_ &&
                  (!release_uav1_ || !have_assigned_follower_target_)
              ? "DIFF_WAIT_RELEASE_OUTSIDE_DOOR"
              : (have_confirmed_door_ ? "DIFF_WAIT_NEXT_RELAY"
                                      : "DIFF_WAIT_CONFIRMED_DOOR");
      publishState(wait_state, 1.0, 0.65, 0.0);
      return true;
    }

    RoutePoint desired_world = relay_waypoints_[active_relay_index_];
    const bool terminal_relay = terminal_mode_active_ &&
                                active_relay_index_ == terminal_waypoint_index_;
    const bool outside_wait_relay = outside_wait_waypoint_released_ &&
        active_relay_index_ == outside_wait_waypoint_index_;
    const bool history_relay = !terminal_relay && !outside_wait_relay;
    if (enable_relay_goal_clearance_ && terminal_relay && !diff_goal_published_ &&
        !diff_recovery_goal_valid_) {
      if (require_fresh_cloud_ &&
          (!follower_cloud_ || (now - cloud_stamp_).toSec() > cloud_timeout_)) {
        setDiffWaitPositionHold(true, "relay goal clearance cloud stale");
        publishState("DIFF_WAIT_RELAY_CLEARANCE_CLOUD", 1.0, 0.4, 0.0);
        return true;
      }
      RoutePoint selected_world;
      int requested_hits = 0;
      if (!chooseClearRelayGoal(desired_world, &selected_world, &requested_hits)) {
        setDiffWaitPositionHold(true, "no clear relay goal on cached route");
        publishState("DIFF_WAIT_CLEAR_RELAY_GOAL", 1.0, 0.2, 0.0);
        ROS_ERROR_THROTTLE(
            1.0,
            "[safe_follower] HOLD relay %zu/%zu: requested goal has %d cloud hits "
            "inside %.2fm and no clear cached point within %.2fm behind it.",
            active_relay_index_ + 1, relay_waypoints_.size(), requested_hits,
            relay_goal_clearance_radius_, relay_goal_backtrack_max_distance_);
        return true;
      }
      const double backtrack = desired_world.progress - selected_world.progress;
      if (backtrack >= path_sample_spacing_) {
        relay_waypoints_[active_relay_index_] = selected_world;
        desired_world = selected_world;
        publishRelayPath();
        ROS_WARN("[safe_follower] relay %zu/%zu too close to obstacle (%d hits < %.2fm); "
                 "cache a point %.2fm backward on leader route at (%.2f,%.2f,%.2f).",
                 active_relay_index_ + 1, relay_waypoints_.size(), requested_hits,
                 relay_goal_clearance_radius_, backtrack, desired_world.position.x,
                 desired_world.position.y, desired_world.position.z);
      }
    }
    const geometry_msgs::Point desired_local =
        terminal_relay ? worldToFollower(desired_world.position)
                       : useFollowerCruiseHeight(worldToFollower(desired_world.position));
    const geometry_msgs::Point& current_local = follower_odom_.pose.pose.position;

    // 所有非终端接力都只是路线进度标记。实际下发点必须从UAV0稠密实飞轨迹的多个
    // 短步候选中选出；选不到时绝不回退为远端接力点直连。
    if (history_relay &&
        !diff_recovery_requested_ && !diff_recovery_retreat_requested_ &&
        !diff_recovery_goal_valid_ && !diff_route_subgoal_valid_ &&
        !diff_goal_published_) {
      RoutePoint route_step_world;
      if (selectForwardRouteCandidate(followerToWorld(current_local), desired_world,
                                      now, &route_step_world,
                                      active_relay_index_ != exit_waypoint_index_)) {
        const geometry_msgs::Point candidate =
            useFollowerCruiseHeight(worldToFollower(route_step_world.position));
        const double remaining_relay_progress = std::max(
            0.0, desired_world.progress - route_step_world.progress);
        diff_route_subgoal_local_ = candidate;
        diff_route_subgoal_progress_ = route_step_world.progress;
        // 接力点是进度里程碑，不是必须命中的坐标。候选沿UAV0实飞折线到达
        // 或越过该progress即可消费里程碑，从而能用接力点后方的安全停靠点替代占据点。
        diff_route_subgoal_completes_relay_ =
            remaining_relay_progress <= path_sample_spacing_;
        diff_route_subgoal_valid_ = true;
        diff_clipped_retry_count_ = 0;
        diff_command_stale_retry_count_ = 0;
        ROS_WARN("[safe_follower] relay %zu/%zu use UAV0-history candidate "
                 "progress=%.2f local=(%.2f,%.2f,%.2f), remaining=%.2fm%s.",
                 active_relay_index_ + 1, relay_waypoints_.size(),
                 route_step_world.progress, candidate.x, candidate.y, candidate.z,
                 remaining_relay_progress,
                 diff_route_subgoal_completes_relay_ ? " (milestone capture)" : "");
      } else {
        setDiffWaitPositionHold(true, "search alternate UAV0-history candidate");
        diff_recovery_requested_ = follower_executed_route_.size() >= 2;
        diff_recovery_retreat_requested_ = diff_recovery_requested_;
        publishState(diff_recovery_requested_ ? "DIFF_SELECT_UAV1_HISTORY_RETREAT"
                                              : "DIFF_RETRY_UAV0_HISTORY_CANDIDATES",
                     1.0, 0.35, 0.0);
        ROS_WARN_THROTTLE(
            0.5,
            "[safe_follower] no clear UAV0-history candidate for relay %zu; %s.",
            active_relay_index_ + 1,
            diff_recovery_requested_ ? "retreat on UAV1 executed history"
                                     : "retry candidates after map/blacklist refresh");
        return true;
      }
    }

    // 原接力点规划失败后先沿前机已验证的稠密路线后退，再从开阔位置重新选择
    // 前向历史轨迹子点，禁止在同一控制周期直接重发刚失败的原目标。
    // Diff可能把占据的退让目标投影到附近安全点，因此以实际接受点作为到达判据。
    const geometry_msgs::Point recovery_arrival_goal =
        diff_accepted_goal_valid_ ? diff_accepted_goal_local_ : diff_recovery_goal_local_;
    const double recovery_error = distance3d(current_local, recovery_arrival_goal);
    const bool retreat_reached = diff_recovery_goal_is_retreat_ &&
        recovery_error <= diff_failure_retreat_arrive_radius_ &&
        follower_horizontal_speed_ <= relay_arrive_max_horizontal_speed_ &&
        follower_vertical_speed_ <= relay_arrive_max_vertical_speed_;
    const bool forward_recovery_reached = !diff_recovery_goal_is_retreat_ &&
        recovery_error <= diff_recovery_arrive_radius_;
    if (diff_recovery_goal_valid_ &&
        (retreat_reached || forward_recovery_reached)) {
      const bool completed_retreat = diff_recovery_goal_is_retreat_;
      ROS_WARN("[safe_follower] UAV1 Diff reached recovery subgoal (%.2f,%.2f,%.2f).",
               diff_recovery_goal_local_.x, diff_recovery_goal_local_.y,
               diff_recovery_goal_local_.z);
      diff_recovery_goal_valid_ = false;
      if (completed_retreat && diff_follower_retreat_target_index_ !=
                                   std::numeric_limits<std::size_t>::max()) {
        truncateFollowerExecutedRoute(diff_follower_retreat_target_index_, current_local);
        diff_follower_retreat_target_index_ =
            std::numeric_limits<std::size_t>::max();
        diff_allow_route_backtrack_attachment_ = true;
        diff_forward_candidate_failures_ = 0;
      }
      diff_recovery_requested_ = !completed_retreat;
      diff_recovery_retreat_requested_ = false;
      diff_recovery_goal_is_retreat_ = false;
      diff_goal_published_ = false;
      diff_accepted_goal_valid_ = false;
      setDiffWaitPositionHold(
          true, completed_retreat ? "failure retreat reached; select new route subgoal"
                                  : "outside-map forward subgoal reached");
      // 返回后让下一控制周期先经过上方的verified-route选点分支。此前这里继续向下
      // 执行，会立刻把远端原目标再次发给Diff，历史轨迹子点逻辑没有运行机会。
      if (completed_retreat) return true;
    }
    if (diff_recovery_requested_ && history_relay) {
      RoutePoint recovery_world;
      const bool recovery_target_found =
          diff_recovery_retreat_requested_
              ? getFollowerHistoryRetreatTarget(current_local, now, &recovery_world)
              : getRelayRouteTarget(followerToWorld(current_local), desired_world,
                                    &recovery_world);
      if (recovery_target_found) {
        const geometry_msgs::Point candidate =
            diff_recovery_retreat_requested_
                ? worldToFollower(recovery_world.position)
                : useFollowerCruiseHeight(worldToFollower(recovery_world.position));
        const double required_distance = diff_recovery_retreat_requested_
                                             ? diff_failure_retreat_arrive_radius_
                                             : diff_recovery_arrive_radius_;
        if (distance3d(current_local, candidate) > required_distance) {
          diff_recovery_goal_local_ = candidate;
          diff_recovery_goal_valid_ = true;
          diff_recovery_goal_is_retreat_ = diff_recovery_retreat_requested_;
          diff_command_stale_retry_count_ = 0;
          if (diff_recovery_goal_is_retreat_) ++diff_failure_retreat_attempts_;
          diff_recovery_requested_ = false;
          diff_recovery_retreat_requested_ = false;
          if (diff_recovery_goal_is_retreat_) {
            ROS_ERROR("[safe_follower] UAV1 Diff flexible retreat %d on own executed history "
                      "local=(%.2f,%.2f,%.2f), then reattach relay %zu/%zu.",
                      diff_failure_retreat_attempts_,
                      candidate.x, candidate.y, candidate.z,
                      active_relay_index_ + 1, relay_waypoints_.size());
          } else {
            ROS_ERROR("[safe_follower] UAV1 Diff outside-map forward subgoal local=(%.2f,%.2f,%.2f) "
                      "toward relay %zu/%zu.", candidate.x, candidate.y, candidate.z,
                      active_relay_index_ + 1, relay_waypoints_.size());
          }
        }
      }
      if (!diff_recovery_goal_valid_) {
        const bool retreat_unavailable = diff_recovery_retreat_requested_;
        setDiffWaitPositionHold(
            true, retreat_unavailable ? "no verified retreat point after planning failure"
                                      : "no verified forward point for outside-map goal");
        publishState(retreat_unavailable ? "DIFF_RESELECT_WITHOUT_RETREAT"
                                         : "DIFF_RETRY_FORWARD_HISTORY",
                     1.0, 0.2, 0.0);
        ROS_ERROR_THROTTLE(
            0.5,
            "[safe_follower] no %s history candidate now; return to rolling selection, "
            "never enter operator-only HOLD.",
            retreat_unavailable ? "UAV1 retreat" : "UAV0 forward");
        diff_recovery_requested_ = false;
        diff_recovery_retreat_requested_ = false;
        return true;
      }
    }
    if (history_relay && !diff_recovery_goal_valid_ &&
        !diff_route_subgoal_valid_) {
      setDiffWaitPositionHold(true, "nonterminal relay requires verified history candidate");
      publishState("DIFF_RESELECT_HISTORY_CANDIDATE", 1.0, 0.3, 0.0);
      return true;
    }
    const geometry_msgs::Point command_local = diff_recovery_goal_valid_
        ? diff_recovery_goal_local_
        : (diff_route_subgoal_valid_ ? diff_route_subgoal_local_ : desired_local);
    // 恢复点只负责把后机带离规划失败位置，绝不能消费正常接力点。恢复状态完全
    // 清空后才允许进入接力点到达判定。
    const bool normal_relay_arrival_enabled =
        !diff_recovery_requested_ && !diff_recovery_retreat_requested_ &&
        !diff_recovery_goal_valid_;
    const bool accepted_matches_command = diff_accepted_goal_valid_ &&
        distance3d(diff_accepted_goal_local_, command_local) <=
            diff_accepted_goal_tolerance_;
    // Diff实际终点用于判断本段轨迹是否结束，但只有它仍接近当前命令点时才有资格完成
    // 子目标或接力点；被大幅裁短的终点到达后只重发当前命令。
    const geometry_msgs::Point arrival_goal =
        (!terminal_relay && normal_relay_arrival_enabled && diff_accepted_goal_valid_)
            ? diff_accepted_goal_local_
            : desired_local;
    const double horizontal_error =
        std::hypot(arrival_goal.x - current_local.x, arrival_goal.y - current_local.y);
    const double vertical_error = std::fabs(arrival_goal.z - current_local.z);

    // 2026-07-28: 动态摆球只检查机前短段并临时锁点；清空后重新下发同一目标，让Diff基于最新地图生成新轨迹。
    geometry_msgs::Point short_probe = limitTargetStep(current_local, command_local);
    uint32_t dynamic_id = 0;
    const bool dynamic_blocked = dynamicSegmentBlocked(current_local, short_probe, &dynamic_id);
    if (dynamic_blocked) {
      if (!diff_dynamic_hold_active_) {
        std_msgs::Bool hold_msg;
        hold_msg.data = true;
        follower_safety_hold_pub_.publish(hold_msg);
        diff_dynamic_hold_active_ = true;
        diff_goal_published_ = false;
        ROS_ERROR("[safe_follower] UAV1 Diff dynamic HOLD id=%u; discard current trajectory.",
                  dynamic_id);
      }
      if (!terminal_relay && diff_route_subgoal_valid_) {
        blacklistRouteCandidate(diff_route_subgoal_local_,
                                diff_route_subgoal_progress_, now,
                                "dynamic obstacle blocks current candidate");
        ++diff_forward_candidate_failures_;
        diff_route_subgoal_valid_ = false;
        diff_route_subgoal_completes_relay_ = false;
        diff_clipped_retry_count_ = 0;
      } else if (diff_recovery_goal_valid_ && diff_recovery_goal_is_retreat_) {
        const double retreat_key = -1000.0 - static_cast<double>(
            diff_follower_retreat_target_index_ ==
                    std::numeric_limits<std::size_t>::max()
                ? 0
                : diff_follower_retreat_target_index_);
        blacklistRouteCandidate(diff_recovery_goal_local_, retreat_key, now,
                                "dynamic obstacle blocks UAV1-history retreat");
        diff_recovery_goal_valid_ = false;
        diff_recovery_requested_ = true;
        diff_recovery_retreat_requested_ = true;
      }
      publishState("DIFF_DYNAMIC_HOLD", 1.0, 0.2, 0.0);
      return true;
    }
    if (diff_dynamic_hold_active_) {
      // 2026-07-28: 若此时仍处于接力点等待锁定，动态框清空不能误解除位置锁点。
      if (!diff_wait_hold_active_) {
        std_msgs::Bool hold_msg;
        hold_msg.data = false;
        follower_safety_hold_pub_.publish(hold_msg);
      }
      diff_dynamic_hold_active_ = false;
      diff_goal_published_ = false;
      ROS_WARN("[safe_follower] UAV1 Diff dynamic path clear; request a fresh trajectory.");
    }

    const bool position_reached = horizontal_error <= relay_arrive_radius_ &&
                                  vertical_error <= relay_arrive_z_tolerance_;
    const bool strict_arrival =
        normal_relay_arrival_enabled && position_reached &&
        follower_horizontal_speed_ <= relay_arrive_max_horizontal_speed_ &&
        follower_vertical_speed_ <= relay_arrive_max_vertical_speed_;
    // 2026-07-28: 普通接力点进入更小的安全半径后立即捕获当前位置；避免轨迹先结束、
    // FAST-LIO差分速度后收敛而形成“到点但永远不ARRIVED”的循环死锁。
    const bool endpoint_capture =
        normal_relay_arrival_enabled && !terminal_relay && diff_accepted_goal_valid_ &&
        horizontal_error <= diff_endpoint_capture_radius_ &&
        vertical_error <= relay_arrive_z_tolerance_;
    if (endpoint_capture && diff_endpoint_capture_stamp_.isZero()) {
      diff_endpoint_capture_stamp_ = now;
      setDiffWaitPositionHold(true, "captured inside Diff endpoint radius");
      ROS_ERROR("[safe_follower] UAV1 Diff ENDPOINT CAPTURE relay=%zu/%zu error=%.3fm "
                "speed=%.2f/%.2f; hold %.2fs before ARRIVED.",
                active_relay_index_ + 1, relay_waypoints_.size(), horizontal_error,
                follower_horizontal_speed_, follower_vertical_speed_,
                diff_endpoint_capture_dwell_);
    }
    if (!endpoint_capture) diff_endpoint_capture_stamp_ = ros::Time(0);
    const bool captured_dwell_complete =
        endpoint_capture && !diff_endpoint_capture_stamp_.isZero() &&
        (now - diff_endpoint_capture_stamp_).toSec() >= diff_endpoint_capture_dwell_;
    if (strict_arrival || captured_dwell_complete) {
      if (relay_arrival_stamp_.isZero()) relay_arrival_stamp_ = now;
      publishState(terminal_relay ? "DIFF_TERMINAL_DWELL" : "DIFF_RELAY_DWELL",
                   0.2, 1.0, 0.2);
      const double required_dwell = captured_dwell_complete ? 0.0 : relay_arrive_dwell_;
      if ((now - relay_arrival_stamp_).toSec() < required_dwell) return true;

      const bool clipped_endpoint_reached = diff_accepted_goal_valid_ &&
                                            !accepted_matches_command;
      if ((diff_route_subgoal_valid_ &&
           !diff_route_subgoal_completes_relay_) ||
          clipped_endpoint_reached) {
        const bool route_subgoal_completed =
            diff_route_subgoal_valid_ && !clipped_endpoint_reached;
        ROS_WARN("[safe_follower] UAV1 Diff reached %s endpoint (%.2f,%.2f,%.2f); "
                 "keep relay %zu/%zu active and continue along verified route.",
                 route_subgoal_completed ? "verified-route subgoal" : "clipped",
                 arrival_goal.x, arrival_goal.y, arrival_goal.z,
                 active_relay_index_ + 1, relay_waypoints_.size());
        setDiffWaitPositionHold(true,
            route_subgoal_completed ? "verified-route subgoal reached"
                                    : "clipped endpoint reached; choose flexible retry");
        if (route_subgoal_completed) {
          diff_route_subgoal_valid_ = false;
          diff_route_subgoal_completes_relay_ = false;
          diff_forward_candidate_failures_ = 0;
          diff_clipped_retry_count_ = 0;
          diff_command_stale_retry_count_ = 0;
          diff_failure_retreat_attempts_ = 0;
        } else {
          ++diff_clipped_retry_count_;
          if (!terminal_relay && diff_route_subgoal_valid_ &&
              diff_clipped_retry_count_ >= diff_clipped_retry_limit_) {
            blacklistRouteCandidate(diff_route_subgoal_local_,
                                    diff_route_subgoal_progress_, now,
                                    "Diff repeatedly clipped endpoint");
            ++diff_forward_candidate_failures_;
            diff_route_subgoal_valid_ = false;
            diff_route_subgoal_completes_relay_ = false;
            diff_clipped_retry_count_ = 0;
            ROS_WARN("[safe_follower] clipped endpoint retry limit reached; "
                     "switch to another UAV0-history candidate.");
          }
        }
        relay_arrival_stamp_ = ros::Time(0);
        diff_endpoint_capture_stamp_ = ros::Time(0);
        diff_goal_published_ = false;
        diff_plan_response_received_ = false;
        diff_accepted_goal_valid_ = false;
        diff_command_seen_for_goal_ = false;
        return true;
      }

      if (terminal_relay) {
        if (!follower_landing_requested_) {
          std_msgs::Bool request;
          request.data = true;
          follower_landing_request_pub_.publish(request);
          follower_landing_requested_ = true;
          setFollowerDetectionEnable(false, "follower landing requested");
          ROS_ERROR("[safe_follower] UAV1 Diff terminal reached; published AUTO.LAND request.");
        }
        return true;
      }

      ROS_ERROR("[safe_follower] UAV1 Diff ARRIVED relay waypoint %zu/%zu.",
                active_relay_index_ + 1, relay_waypoints_.size());
      if (active_relay_index_ == outside_wait_waypoint_index_) {
        outside_wait_arrived_ = true;
        setFollowerDetectionEnable(false, "follower reached outside-door hover point");
        tryQueueFollowerTerminalTarget();
      }
      // 2026-07-28: 在清除当前目标之前先锁存到达位置，杜绝旧轨迹超时前的残余指令继续拉动后机。
      setDiffWaitPositionHold(true, "relay waypoint arrived");
      ++active_relay_index_;
      relay_arrival_stamp_ = ros::Time(0);
      diff_endpoint_capture_stamp_ = ros::Time(0);
      diff_goal_published_ = false;
      // 2026-07-28: 原接力点完成后清除上一段Diff短子目标恢复状态。
      diff_recovery_requested_ = false;
      diff_recovery_retreat_requested_ = false;
      diff_recovery_goal_valid_ = false;
      diff_recovery_goal_is_retreat_ = false;
      diff_route_subgoal_valid_ = false;
      diff_route_subgoal_completes_relay_ = false;
      diff_accepted_goal_valid_ = false;
      diff_forward_candidate_failures_ = 0;
      diff_clipped_retry_count_ = 0;
      diff_command_stale_retry_count_ = 0;
      diff_failure_retreat_attempts_ = 0;
      return true;
    }
    relay_arrival_stamp_ = ros::Time(0);

    // 规划器若连状态都不返回，总应答计时必须把当前候选判失败，而不是靠周期重发
    // 永久刷新超时。独立看门狗会重启假死进程；此处保证候选状态机立即继续。
    const bool response_timeout = diff_goal_published_ &&
        !diff_plan_response_received_ && !diff_goal_first_publish_stamp_.isZero() &&
        (now - diff_goal_first_publish_stamp_).toSec() >=
            diff_goal_response_timeout_;
    if (response_timeout) {
      const double wait_time = (now - diff_goal_first_publish_stamp_).toSec();
      ROS_ERROR("[safe_follower] UAV1 Diff TOTAL RESPONSE TIMEOUT %.2fs at "
                "relay=%zu/%zu; blacklist current candidate and continue.",
                wait_time, active_relay_index_ + 1, relay_waypoints_.size());
      diff_plan_response_received_ = true;
      handleDiffPlanningFailure("PLANNER_RESPONSE_TIMEOUT", now);
      return true;
    }

    // 2026-07-28: 已收到规划成功但PositionCommand已经失活，说明控制器正在零速度等待；
    // 未进入近目标捕获区时必须重发当前点，不能让旧accepted状态永久占住活动索引。
    const bool command_stale = diff_goal_published_ && diff_plan_response_received_ &&
        diff_command_seen_for_goal_ && !diff_command_stamp_.isZero() &&
        (now - diff_command_stamp_).toSec() >= diff_command_stale_timeout_;
    if (command_stale) {
      bool reselect_after_stale = false;
      setDiffWaitPositionHold(true, "Diff PositionCommand stale before arrival");
      ROS_ERROR("[safe_follower] UAV1 Diff COMMAND STALE %.2fs at relay=%zu/%zu "
                "error=%.2fm; reissue current goal instead of waiting forever.",
                (now - diff_command_stamp_).toSec(), active_relay_index_ + 1,
                relay_waypoints_.size(), horizontal_error);
      ++diff_command_stale_retry_count_;
      if (!terminal_relay && diff_route_subgoal_valid_ &&
          diff_command_stale_retry_count_ >= diff_clipped_retry_limit_) {
        blacklistRouteCandidate(diff_route_subgoal_local_,
                                diff_route_subgoal_progress_, now,
                                "Diff command repeatedly stale before arrival");
        ++diff_forward_candidate_failures_;
        diff_route_subgoal_valid_ = false;
        diff_route_subgoal_completes_relay_ = false;
        diff_command_stale_retry_count_ = 0;
        reselect_after_stale = true;
        ROS_WARN("[safe_follower] stale-command retry limit reached; "
                 "switch to another UAV0-history candidate.");
      } else if (diff_recovery_goal_valid_ && diff_recovery_goal_is_retreat_ &&
                 diff_command_stale_retry_count_ >= diff_clipped_retry_limit_) {
        const double retreat_key = -1000.0 - static_cast<double>(
            diff_follower_retreat_target_index_ ==
                    std::numeric_limits<std::size_t>::max()
                ? 0
                : diff_follower_retreat_target_index_);
        blacklistRouteCandidate(diff_recovery_goal_local_, retreat_key, now,
                                "UAV1-history retreat command repeatedly stale");
        diff_recovery_goal_valid_ = false;
        diff_recovery_requested_ = true;
        diff_recovery_retreat_requested_ = true;
        diff_command_stale_retry_count_ = 0;
        reselect_after_stale = true;
        ROS_WARN("[safe_follower] retreat command stale repeatedly; "
                 "switch to another UAV1-history retreat point.");
      }
      diff_goal_published_ = false;
      diff_plan_response_received_ = false;
      diff_accepted_goal_valid_ = false;
      diff_command_seen_for_goal_ = false;
      if (reselect_after_stale) return true;
    }

    // 2026-07-28: 只有连Diff状态都没收到才超时重发；已生成轨迹后禁止1Hz重置规划器。
    const bool retry_due = !diff_plan_response_received_ && !diff_goal_publish_stamp_.isZero() &&
        (now - diff_goal_publish_stamp_).toSec() >= diff_goal_retry_period_;
    if (!diff_goal_published_ || diff_goal_index_ != active_relay_index_ || retry_due) {
      if (diff_goal_pub_.getNumSubscribers() == 0) {
        publishState("WAIT_UAV1_DIFF_SUBSCRIBER", 1.0, 0.4, 0.0);
        ROS_WARN_THROTTLE(1.0, "[safe_follower] waiting for UAV1 Diff goal subscriber on %s.",
                          diff_goal_topic_.c_str());
        return true;
      }
      geometry_msgs::PoseStamped goal;
      goal.header.stamp = now;
      goal.header.frame_id = follower_odom_.header.frame_id.empty()
                                 ? world_frame_ : follower_odom_.header.frame_id;
      // 恢复期间只给Diff一个位于前机已验证折线后的退让目标，避免在障碍边原地重算。
      goal.pose.position = command_local;
      const double local_yaw = worldYawToFollower(desired_world.yaw);
      goal.pose.orientation.w = std::cos(local_yaw * 0.5);
      goal.pose.orientation.z = std::sin(local_yaw * 0.5);
      const bool starts_new_response_window =
          !diff_goal_published_ || diff_goal_index_ != active_relay_index_ ||
          diff_goal_first_publish_stamp_.isZero();
      stampDiffGoalId(&goal);
      diff_goal_pub_.publish(goal);
      diff_goal_index_ = active_relay_index_;
      diff_goal_published_ = true;
      diff_plan_response_received_ = false;
      diff_accepted_goal_valid_ = false;
      diff_command_seen_for_goal_ = false;
      diff_goal_publish_stamp_ = now;
      if (starts_new_response_window) diff_goal_first_publish_stamp_ = now;
      publishTarget(desired_world.position);
      ROS_ERROR("[safe_follower] SEND UAV1 DIFF goal %zu/%zu local=(%.2f,%.2f,%.2f).",
                active_relay_index_ + 1, relay_waypoints_.size(), command_local.x,
                command_local.y, command_local.z);
    }
    publishState(terminal_relay ? "UAV1_DIFF_GO_TERMINAL"
                                : (outside_wait_relay
                                       ? "UAV1_DIFF_GO_OUTSIDE_DOOR"
                                       : "UAV1_DIFF_GO_RELAY"),
                 0.1, 0.8, 1.0);
    return true;
  }

  void timerCallback(const ros::TimerEvent&) {
    if (follower_precision_landing_active_) {
      publishState("UAV1_PRECISION_LANDING_OWNS_CONTROL", 0.2, 1.0, 0.2);
      return;
    }
    if (!follower_started_) return;
    // FAST-LIO异常期间不生成目标；连续可信样本恢复后回调会自动解除并请求新轨迹。
    if (follower_odom_fault_latched_) {
      publishState("ODOM_FAULT_WAIT_AUTOMATIC_RECOVERY", 1.0, 0.0, 0.0);
      return;
    }
    if (!have_follower_odom_) {
      hold("waiting for follower odometry");
      return;
    }
    const ros::Time now = ros::Time::now();
    if ((now - follower_odom_stamp_).toSec() > odom_timeout_) return;
    if (!have_leader_odom_ && route_.empty()) {
      hold("waiting for leader history");
      return;
    }
    if (!leaderOdomFresh(now)) {
      ROS_WARN_THROTTLE(
          1.0,
          "[safe_follower] leader odometry stale; continue only along cached connected history.");
    }
    // 2026-07-27: 卡死恢复优先于正常连续/离散跟随，避免正常目标每50ms覆盖脱困指令。
    if (!simple_segment_endpoint_following_ && handleStuckRecovery(now)) return;
    if (!leader_started_) {
      hold("leader mission not started");
      return;
    }

    // 出口阶段消息可能只到达一次；若当时不足1m，在定时器中持续按实时间距重试。
    tryReleaseFinalExitWaypoint("periodic separation recheck");

    // 2026-07-28: Diff执行分支在旧连续追踪/直控逻辑之前截断，确保后机只由自身规划器输出轨迹。
    if (handleDiffPlannerExecution(now)) return;

    if (handleContinuousFollowBeforeExit(now)) return;

    // 出口前由1.00m连续滞后模式执行；出口外及终点阶段恢复离散任务点顺序控制。
    if (active_relay_index_ >= relay_waypoints_.size()) {
      hold(have_confirmed_door_ ? "waiting for leader to release next relay waypoint"
                                : "waiting for confirmed door");
      return;
    }

    const RoutePoint& desired_world = relay_waypoints_[active_relay_index_];
    const bool terminal_relay = terminal_mode_active_ &&
                                active_relay_index_ == terminal_waypoint_index_;
    // 2026-07-24: 普通门点/接力点执行时再次强制XY-only高度；最终降落点保留0.60m专用值。
    const geometry_msgs::Point desired_local =
        terminal_relay
            ? worldToFollower(desired_world.position)
            : useFollowerCruiseHeight(worldToFollower(desired_world.position));
    RoutePoint tracking_world = desired_world;
    // 2026-07-16: 第一个门点保持直接进门；进入作业区后的接力点沿前机实飞轨迹逐段跟踪，
    // 解决离散点跨越转角后，后机把墙后的目标当成直线目标持续怼墙的问题。
    if (active_relay_index_ > 0 &&
        !getRelayRouteTarget(followerToWorld(follower_odom_.pose.pose.position),
                             desired_world, &tracking_world)) {
      hold("leader route unavailable for relay segment");
      return;
    }
    geometry_msgs::Point target_local =
        terminal_relay
            ? worldToFollower(tracking_world.position)
            : useFollowerCruiseHeight(worldToFollower(tracking_world.position));
    const geometry_msgs::Point& current_local = follower_odom_.pose.pose.position;
    // 2026-07-16: 到达判定看最终接力点，控制发布看0.3m稠密前视点；两者不能混用而提前跳点。
    const double horizontal_error =
        std::hypot(desired_local.x - current_local.x, desired_local.y - current_local.y);
    const double vertical_error = std::fabs(desired_local.z - current_local.z);

    // 普通接力点在远处时不做“大圆柱净空”硬否决。靠近后只检查落点自身小体素；
    // 若精确点确实占据，允许停在前机实飞折线上的安全附件并消费下一点。
    bool relay_point_occupied = false;
    int relay_point_hits = 0;
    if (obstacle_check_enabled_) {
      if (require_fresh_cloud_ &&
          (!follower_cloud_ || (now - cloud_stamp_).toSec() > cloud_timeout_)) {
        relay_arrival_stamp_ = ros::Time(0);
        hold("relay endpoint cloud stale");
        return;
      }
      if (terminal_relay) {
        int endpoint_hits = 0;
        if (endpointOccupied(desired_local, &endpoint_hits)) {
          relay_arrival_stamp_ = ros::Time(0);
          hold("terminal relay endpoint occupied by follower cloud");
          ROS_ERROR_THROTTLE(1.0,
                             "[safe_follower] terminal relay endpoint occupied hits=%d; "
                             "wait until landing area clears.",
                             endpoint_hits);
          return;
        }
      } else if (horizontal_error <= relay_point_check_distance_) {
        relay_point_occupied = relayPointOccupied(desired_local, &relay_point_hits);
      }
    }

    const bool occupied_attachment = relay_point_occupied &&
                                     horizontal_error <= relay_occupied_attachment_radius_;
    const bool position_reached =
        (horizontal_error <= relay_arrive_radius_ &&
         vertical_error <= relay_arrive_z_tolerance_) ||
        (occupied_attachment &&
         vertical_error <= std::max(relay_arrive_z_tolerance_,
                                    relay_point_occupied_z_margin_));
    if (position_reached &&
        follower_horizontal_speed_ <= relay_arrive_max_horizontal_speed_ &&
        follower_vertical_speed_ <= relay_arrive_max_vertical_speed_) {
      // 2026-07-22: 距离和速度同时满足后持续稳定一段时间，消除带速穿过阈值导致的假到达。
      if (relay_arrival_stamp_.isZero()) relay_arrival_stamp_ = now;
      if ((now - relay_arrival_stamp_).toSec() < relay_arrive_dwell_) {
        hold_target_latched_ = false;
        publishCommand(occupied_attachment ? current_local : desired_local,
                       worldYawToFollower(desired_world.yaw), false);
        publishState(occupied_attachment ? "RELAY_OCCUPIED_ATTACHMENT_DWELL"
                                         : "RELAY_ARRIVAL_DWELL",
                     0.2, 1.0, 0.2);
        if (occupied_attachment)
          ROS_WARN_THROTTLE(1.0,
                            "[safe_follower] relay point %zu exact voxel occupied hits=%d; "
                            "accept safe attachment %.2fm away.",
                            active_relay_index_ + 1, relay_point_hits, horizontal_error);
        return;
      }
      const bool terminal_reached = terminal_relay;
      if (!terminal_reached) {
        ROS_ERROR("[safe_follower] ARRIVED relay waypoint %zu/%zu; wait/execute next released point.",
                  active_relay_index_ + 1, relay_waypoints_.size());
        // 2026-07-22: 正常抵达后把已验证接力点锁为等待目标；即使受扰离开，也会主动拉回而不是接受漂移后的位置。
        hold_target_local_ = occupied_attachment ? current_local : desired_local;
        hold_target_yaw_ = worldYawToFollower(desired_world.yaw);
        hold_target_latched_ = true;
        if (active_relay_index_ == outside_wait_waypoint_index_) {
          outside_wait_arrived_ = true;
          setFollowerDetectionEnable(false,
                                     "follower reached outside-door hover point");
          tryQueueFollowerTerminalTarget();
        }
        ++active_relay_index_;
        relay_arrival_stamp_ = ros::Time(0);
        terminal_arrival_stamp_ = ros::Time(0);
        hold(active_relay_index_ < relay_waypoints_.size() ? "switching to next relay waypoint"
                                                           : "waiting for next relay waypoint");
        return;
      }

      if (terminal_arrival_stamp_.isZero()) terminal_arrival_stamp_ = now;
      publishCommand(target_local, worldYawToFollower(desired_world.yaw), false);
      publishState("TERMINAL_DWELL", 0.2, 1.0, 0.2);
      if (!follower_landing_requested_ &&
          (now - terminal_arrival_stamp_).toSec() >= terminal_arrive_dwell_) {
        // 2026-07-15: 只有按顺序完成门点和内部接力点后，才允许后机在独立终点请求降落。
        std_msgs::Bool request;
        request.data = true;
        follower_landing_request_pub_.publish(request);
        follower_landing_requested_ = true;
        // 2026-07-27: 后机完成全部接力并请求降落后关闭检测，清空下游动态目标残影。
        setFollowerDetectionEnable(false, "follower landing requested");
        ROS_ERROR("[safe_follower] follower completed all relay points; published AUTO.LAND request.");
      }
      return;
    }
    relay_arrival_stamp_ = ros::Time(0);
    terminal_arrival_stamp_ = ros::Time(0);

    target_local = limitTargetStep(follower_odom_.pose.pose.position, target_local);
    if (obstacle_check_enabled_) {
      if (require_fresh_cloud_ &&
          (!follower_cloud_ || (now - cloud_stamp_).toSec() > cloud_timeout_)) {
        hold("follower cloud stale");
        return;
      }
      // 2026-07-28: 离散接力同样先处理动态预测；动态阻挡只等待，不累计不可达时间、跳点或触发随机脱困。
      uint32_t dynamic_id = 0;
      if (dynamicSegmentBlocked(follower_odom_.pose.pose.position, target_local,
                                &dynamic_id)) {
        blocked_since_ = ros::Time(0);
        blocked_waypoint_index_ = std::numeric_limits<std::size_t>::max();
        hold("predicted dynamic obstacle crossing relay path");
        ROS_WARN_THROTTLE(0.5,
                          "[safe_follower] DYNAMIC HOLD id=%u blocks relay segment.",
                          dynamic_id);
        return;
      }
      int obstacle_hits = 0;
      const bool clear_path =
          terminal_relay
              ? !segmentBlocked(follower_odom_.pose.pose.position, target_local,
                                &obstacle_hits)
              : chooseClearFollowerHeight(follower_odom_.pose.pose.position,
                                          &target_local, &obstacle_hits);
      if (!clear_path) {
        // 当前点持续阻挡时保持该点并尝试局部脱困；禁止递增索引跳过缓存点。
        if (blocked_waypoint_index_ != active_relay_index_) {
          blocked_waypoint_index_ = active_relay_index_;
          blocked_since_ = now;
        }
        const double blocked_duration = (now - blocked_since_).toSec();
        if (blocked_duration >= blocked_recovery_timeout_ && !recovery_active_)
          startRecovery(now, "local path blocked while holding");
        hold("local path blocked by follower cloud");
        return;
      }
      blocked_since_ = ros::Time(0);
      blocked_waypoint_index_ = std::numeric_limits<std::size_t>::max();
    }

    // 2026-07-22: 接近任何接力点都提前降速，给速度闭环留出制动距离；终点仍使用更低的专用速度。
    double command_speed = terminal_mode_active_ &&
                                   active_relay_index_ == terminal_waypoint_index_
                               ? terminal_approach_speed_
                               : cruise_speed_;
    if (horizontal_error <= relay_slowdown_radius_)
      command_speed = std::min(command_speed, relay_approach_speed_);
    publishCommand(target_local, worldYawToFollower(tracking_world.yaw), true,
                   command_speed);
    publishTarget(desired_world.position);
    publishState(terminal_mode_active_ && active_relay_index_ == terminal_waypoint_index_
                     ? "GO_TERMINAL_RELAY"
                     : "GO_DISCRETE_RELAY",
                 0.1, 0.8, 1.0);
    ROS_INFO_THROTTLE(
        1.0,
        "[safe_follower] RELAY target=%zu/%zu error_xy=%.2fm target_local=(%.2f,%.2f,%.2f)",
        active_relay_index_ + 1, relay_waypoints_.size(), horizontal_error,
        target_local.x, target_local.y, target_local.z);
  }

  void publishRoute(const ros::Time& stamp) {
    nav_msgs::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = world_frame_;
    for (const RoutePoint& point : route_) {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = point.position;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    route_pub_.publish(path);
  }

  void publishTarget(const geometry_msgs::Point& target_world) {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = world_frame_;
    marker.ns = "safe_follower";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = target_world;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.20;
    marker.color.a = 1.0;
    marker.color.r = 0.1;
    marker.color.g = 0.75;
    marker.color.b = 1.0;
    target_pub_.publish(marker);
  }

  void publishState(const std::string& text, double red, double green, double blue) {
    if (!have_follower_odom_) return;
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = follower_odom_.header.frame_id;
    marker.ns = "safe_follower_state";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = follower_odom_.pose.pose.position;
    marker.pose.position.z += 0.45;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.18;
    marker.color.a = 1.0;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.text = text;
    state_pub_.publish(marker);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber leader_odom_sub_, leader_history_path_sub_, leader_trajectory_sub_,
      follower_odom_sub_, follower_cloud_sub_;
  ros::Subscriber diff_command_sub_;  // 2026-07-28: UAV1实际轨迹输出存活监测，不参与发布。
  ros::Subscriber dynamic_obstacle_sub_;  // UAV1 LDOP结构化目标状态。
  ros::Subscriber leader_landing_target_sub_, leader_landing_request_sub_, door_pose_sub_;
  ros::Subscriber release_uav1_sub_;
  ros::Subscriber follower_assigned_target_sub_;
  ros::Subscriber follower_landing_trigger_sub_;
  ros::Subscriber final_exit_pose_sub_;  // 2026-07-28: 前机永久锁存的最终出口门心。
  ros::Subscriber leader_task_status_sub_, landing_search_state_sub_,
      front_scan_anchor_sub_;
  ros::Subscriber diff_status_sub_;  // 2026-07-28: UAV1 Diff轨迹成功/失败反馈。
  ros::Publisher command_pub_, traj_started_pub_, diff_goal_pub_, route_pub_, relay_path_pub_, target_pub_, state_pub_;
  ros::Publisher follower_landing_target_pub_, follower_landing_request_pub_;
  ros::Publisher follower_detection_enable_pub_;  // UAV1独立LDOP阶段状态。
  ros::Publisher follower_safety_hold_pub_;  // 2026-07-28: LIO跳变时请求控制器按MAVROS坐标锁点。
  ros::Timer timer_;
  nav_msgs::Odometry leader_odom_, follower_odom_;
  geometry_msgs::PoseStamped leader_landing_target_;
  RoutePoint terminal_target_world_, confirmed_door_, confirmed_exit_,
      front_scan_anchor_, pending_relay_;
  RoutePoint assigned_follower_target_;
  sensor_msgs::PointCloud2::ConstPtr follower_cloud_;
  std::vector<DynamicObstacleSample> retained_dynamic_obstacles_;
  std::deque<RoutePoint> route_;
  // UAV0每段B-spline只有在真实到达且停稳后才进入该表；progress用于确认它属于实飞折线。
  std::deque<RoutePoint> leader_segment_endpoints_;
  // UAV1实际执行轨迹作为可逆安全栈：正常前进时追加，退让时沿栈向后取点并在到达后截断。
  std::deque<RoutePoint> follower_executed_route_;
  std::deque<BlockedRouteCandidate> blocked_route_candidates_;
  // 2026-07-21: 保存尚未判定为“真实回头”或“U形新分支”的负投影实飞折线，确认后原样接入而非直连。
  std::deque<RoutePoint> turn_candidate_route_;
  // 2026-07-16: relay_waypoints_ 是实际下发给后机的门点、滚动内部点和真实终点。
  std::vector<RoutePoint> relay_waypoints_;
  ros::Time leader_odom_stamp_, follower_odom_stamp_, follower_odom_sample_stamp_, cloud_stamp_;
  ros::Time dynamic_obstacle_receive_stamp_, retained_dynamic_obstacle_stamp_;
  std::string leader_odom_topic_, leader_history_path_topic_, leader_trajectory_topic_,
      follower_odom_topic_, follower_cloud_topic_;
  std::string command_topic_, diff_goal_topic_;
  std::string diff_status_topic_;  // 2026-07-28: 默认/drone_1_planning/status。
  std::string traj_started_topic_, world_frame_;
  std::string leader_landing_target_topic_, leader_landing_request_topic_;
  std::string release_uav1_topic_;
  std::string follower_assigned_target_topic_;
  std::string follower_landing_target_topic_, follower_landing_request_topic_;
  std::string follower_landing_trigger_topic_, door_pose_topic_;
  std::string final_exit_pose_topic_;  // 2026-07-28: 默认/UAV0/mission/final_exit。
  std::string landing_search_state_topic_, front_scan_anchor_topic_;
  std::string relay_path_topic_, leader_task_status_topic_, follower_detection_enable_topic_;
  std::string dynamic_obstacle_topic_;  // 默认/UAV1/ldop/dynamic_objects。
  bool have_leader_odom_{false}, have_follower_odom_{false};
  bool use_diff_planner_{true};  // 2026-07-28: 默认启用UAV1独立Diff规划，旧直控仅作显式回退。
  bool simple_segment_endpoint_following_{false};
  bool simple_occupied_recovery_active_{false};
  bool simple_waypoint_clearance_hold_active_{false};
  bool leader_started_{false}, follower_started_{false}, traj_started_sent_{false};
  bool have_leader_landing_target_{false}, terminal_mode_active_{false};
  bool release_uav1_{false};
  bool pending_leader_landing_request_{false};
  bool have_assigned_follower_target_{false};
  bool follower_landing_requested_{false};
  bool follower_precision_landing_active_{false};
  bool follower_detection_enabled_{false};  // 2026-07-27: 锁存的UAV1检测会话状态。
  bool diff_goal_published_{false}, diff_dynamic_hold_active_{false}; // 2026-07-28: UAV1 Diff目标与动态紧停状态。
  uint64_t diff_active_goal_stamp_ns_{0ULL};
  bool enable_dynamic_obstacle_detection_{false};
  bool diff_wait_hold_active_{false};  // 2026-07-28: 接力点之间使用MAVROS位置闭环锁点，区别于动态临时HOLD。
  bool diff_plan_response_received_{false}, diff_accepted_goal_valid_{false}; // 2026-07-28: Diff应答与实际落点。
  bool diff_recovery_requested_{false}, diff_recovery_goal_valid_{false}; // 已验证路线短子目标恢复。
  bool diff_recovery_retreat_requested_{false}, diff_recovery_goal_is_retreat_{false};
  bool diff_command_seen_for_goal_{false};  // 2026-07-28: 当前Diff目标是否真正产生过PositionCommand。
  bool follower_odom_fault_latched_{false};  // 异常期间锁点，连续可信样本恢复后自动解除。
  bool have_confirmed_door_{false}, door_waypoint_released_{false};
  bool have_final_exit_{false}, exit_waypoint_released_{false};  // 2026-07-28: 最终出口接收/排队锁存。
  bool have_front_scan_anchor_{false};
  bool down_search_wait_requested_{false};
  bool outside_wait_waypoint_released_{false}, outside_wait_arrived_{false};
  bool pending_relay_valid_{false};
  bool continuous_follow_before_exit_{true}, leader_outside_exit_{false};
  bool enable_diff_separation_safety_{true};
  bool enable_search_landing_{false};
  // 2026-07-20: 接力路线判向、回头暂停和恢复状态独立于前机原始Odometry保存。
  bool have_last_leader_sample_{false}, relay_route_paused_{false};
  bool pending_leader_segment_endpoint_valid_{false};
  bool obstacle_check_enabled_{true}, require_fresh_cloud_{true};
  bool diff_separation_hold_active_{false}, separation_recovery_active_{false};
  bool separation_recovery_goal_valid_{false};
  bool simple_separation_hold_active_{false};
  // 2026-07-27: 后机物理卡死监测与点云选向脱困状态，独立于两机间距恢复逻辑。
  bool motion_monitor_active_{false}, last_command_moving_{false}, recovery_active_{false};
  // 2026-07-22: HOLD目标只在进入等待/故障的首周期锁存，避免随后位置漂移不断改写恢复目标。
  bool hold_target_latched_{false};
  geometry_msgs::Point hold_target_local_;
  double hold_target_yaw_{0.0};
  double follower_alignment_x_{-1.20}, follower_alignment_y_{0.0};
  double follower_alignment_z_{0.0}, follower_alignment_yaw_{0.0};
  double follower_alignment_cos_{1.0}, follower_alignment_sin_{0.0};
  double leader_start_height_{0.3}, follower_start_height_{0.3};
  // 默认0.70m缓存路径间隔；双机XY间距0.70m内锁点，0.60m内触发退让。
  double follow_distance_{1.50}, release_path_length_{0.70}, min_separation_{0.70};
  double waypoint_release_min_separation_{0.70};
  double fixed_follow_height_{0.60}, follow_height_min_{0.60}, follow_height_max_{0.70};
  double down_search_release_height_{1.80};
  double down_search_min_vertical_separation_{1.00};
  double continuous_follow_speed_{0.42};
  double separation_recovery_distance_{0.60}, separation_release_distance_{0.80};
  double emergency_retreat_step_{0.35}, emergency_retreat_speed_{0.30};
  // 2026-07-16: 无launch覆盖时也保持后机在前机终点路线后方约0.5m的独立落点。
  double terminal_landing_spacing_{0.50}, terminal_approach_height_{0.60};
  double terminal_arrive_radius_{0.25}, terminal_arrive_z_tolerance_{0.15};
  double terminal_arrive_dwell_{1.0}, terminal_approach_speed_{0.25};
  double path_sample_spacing_{0.08};
  double outside_door_distance_{0.50};
  double max_route_length_{60.0}, route_length_{0.0};
  // 2026-07-21: 用最近0.45m实飞路线切线判向，不再依赖可能与实际转弯不一致的雷达里程计yaw。
  double forward_projection_ratio_{-0.20}, route_direction_window_{0.45};
  double backtrack_pause_distance_{0.20};
  double route_revisit_radius_{0.45}, route_revisit_progress_gap_{1.00};
  double route_resume_radius_{0.25}, consecutive_backward_distance_{0.0};
  // 2026-07-21: 新分支需累计足够长度且离开全部旧路线后才可恢复接力，抑制里程计短时横跳误判。
  double turn_branch_confirm_distance_{0.50}, turn_candidate_length_{0.0};
  double max_target_step_{0.55}, route_tracking_lookahead_{0.30};
  double cruise_speed_{0.35}, max_vertical_speed_{0.20};
  double odom_timeout_{0.50}, leader_odom_max_transport_age_{0.50};
  double cloud_timeout_{0.60}, min_record_height_{0.30};
  double obstacle_radius_{0.28}, obstacle_z_margin_{0.20}, obstacle_ignore_near_{0.18};
  double dynamic_obstacle_retention_{0.80}, dynamic_obstacle_safety_radius_{0.35};
  double dynamic_obstacle_z_margin_{0.18};
  double dynamic_retention_route_half_width_{0.70};  // 2026-07-28: 动态保留只覆盖通道中心带，墙边框立即淘汰。
  double diff_goal_retry_period_{1.0};  // 2026-07-28: 仅Diff无任何状态应答时使用的超时重试周期。
  double diff_goal_response_timeout_{2.0};  // 首次发布到任意规划状态的总上限。
  double simple_diff_retry_delay_{0.20};
  double diff_recovery_arrive_radius_{0.18};  // 2026-07-28: 短子目标切换半径。
  double diff_failure_retreat_distance_{0.40};  // 规划失败后沿已验证路线后退的距离。
  double diff_failure_retreat_arrive_radius_{0.10};
  double diff_candidate_blacklist_duration_{4.0};
  double diff_candidate_blacklist_radius_{0.08};
  double diff_history_target_max_distance_{1.50};
  double leader_segment_endpoint_radius_{0.20};
  double leader_segment_endpoint_max_speed_{0.12};
  double leader_segment_endpoint_dwell_{0.25};
  double follower_history_sample_spacing_{0.08};
  double follower_history_max_length_{30.0};
  double follower_history_length_{0.0};
  double diff_endpoint_capture_radius_{0.15}, diff_endpoint_capture_dwell_{0.45};
  double diff_accepted_goal_tolerance_{0.20};
  double diff_command_stale_timeout_{0.80};  // 2026-07-28: 与控制器0.60s轨迹超时错开0.20s。
  double follower_odom_jump_speed_{2.0}, follower_odom_jump_vertical_speed_{1.2};
  int follower_odom_jump_confirm_samples_{3};
  int follower_odom_jump_consecutive_samples_{0};
  int follower_odom_recovery_good_samples_{0};
  double stuck_detection_timeout_{1.50}, stuck_min_progress_{0.06};
  double recovery_step_{0.35}, recovery_speed_{0.20}, recovery_attempt_timeout_{1.50};
  double recovery_success_distance_{0.12}, recovery_near_ignore_{0.06};
  double blocked_recovery_timeout_{1.00};
  double last_command_dx_{0.0}, last_command_dy_{0.0}, recovery_yaw_{0.0};
  int obstacle_min_points_{3};
  double relay_release_distance_{0.70}, door_release_inside_distance_{0.70};
  double relay_waypoint_spacing_{2.50};
  double relay_arrive_radius_{0.25}, relay_arrive_z_tolerance_{0.20};
  // 终点仍使用较大停驻净空；普通接力点只查落点小体素并允许安全附件到达。
  double relay_endpoint_clearance_radius_{0.38}, relay_endpoint_z_margin_{0.28};
  double relay_point_occupied_radius_{0.12}, relay_point_occupied_z_margin_{0.18};
  double relay_point_check_distance_{0.45}, relay_occupied_attachment_radius_{0.35};
  double relay_goal_clearance_radius_{0.20}, relay_goal_clearance_z_margin_{0.30};
  double relay_goal_backtrack_max_distance_{1.00};
  double relay_slowdown_radius_{0.55}, relay_approach_speed_{0.20};
  double relay_arrive_max_horizontal_speed_{0.10}, relay_arrive_max_vertical_speed_{0.08};
  double relay_arrive_dwell_{0.50};
  double follower_horizontal_speed_{0.0}, follower_vertical_speed_{0.0};
  bool enable_relay_goal_clearance_{false};
  int relay_endpoint_min_points_{2};
  int relay_point_occupied_min_points_{2};
  int relay_goal_clearance_min_points_{2};
  int diff_forward_failures_before_retreat_{3};
  int diff_clipped_retry_limit_{2};
  double leader_route_progress_{0.0}, last_relay_selection_progress_{0.0};
  int max_internal_relay_points_{0}, internal_relay_count_{0};
  std::size_t active_relay_index_{0}, terminal_waypoint_index_{std::numeric_limits<std::size_t>::max()};
  std::size_t exit_waypoint_index_{std::numeric_limits<std::size_t>::max()};  // 2026-07-28: 到达后关闭通道动态检测。
  std::size_t outside_wait_waypoint_index_{std::numeric_limits<std::size_t>::max()};
  std::size_t diff_goal_index_{std::numeric_limits<std::size_t>::max()}; // 2026-07-28: 每个离散点仅触发一次规划。
  std::size_t follower_route_index_{0};
  RoutePoint last_leader_sample_;  // 2026-07-20: 最近一次达到采样间距的前机点，仅用于运动方向判断。
  RoutePoint pending_leader_segment_endpoint_;
  int64_t last_leader_trajectory_id_{std::numeric_limits<int64_t>::min()};
  int64_t pending_leader_segment_trajectory_id_{0};
  ros::Time last_leader_trajectory_start_time_;
  ros::Time leader_segment_endpoint_dwell_start_;
  ros::Time terminal_arrival_stamp_, relay_arrival_stamp_;
  ros::Time diff_goal_publish_stamp_;  // 2026-07-28: 防止瞬时假占据导致一次性目标永久失效。
  ros::Time diff_goal_first_publish_stamp_;  // 不被同一点周期重发刷新的总应答计时。
  ros::Time simple_diff_retry_not_before_;
  ros::Time diff_command_stamp_, diff_endpoint_capture_stamp_;  // 2026-07-28: 轨迹存活与近目标捕获计时。
  ros::Time motion_monitor_start_, recovery_attempt_start_;
  ros::Time blocked_since_;  // 2026-07-27: 当前串行检查点连续被自身点云阻挡的起始时间。
  ros::Time continuous_blocked_since_;  // 2026-07-27: 出口前滚动目标的连续点云阻挡计时。
  std::size_t blocked_waypoint_index_{std::numeric_limits<std::size_t>::max()};
  geometry_msgs::Point motion_monitor_origin_local_, recovery_origin_local_, recovery_target_local_;
  geometry_msgs::Point diff_recovery_goal_local_;  // 2026-07-28: 当前已验证路线短子目标。
  geometry_msgs::Point separation_recovery_goal_local_;  // 双机过近时的逐步退让目标。
  geometry_msgs::Point diff_accepted_goal_local_;  // 2026-07-28: Diff对占据原目标修正后的真正落点。
  geometry_msgs::Point diff_route_subgoal_local_;  // UAV0已验证折线上的当前短步目标。
  bool diff_route_subgoal_valid_{false};
  bool diff_route_subgoal_completes_relay_{false};
  bool diff_allow_route_backtrack_attachment_{false};
  double diff_route_subgoal_progress_{0.0};
  int diff_forward_candidate_failures_{0};
  int diff_clipped_retry_count_{0};
  int diff_command_stale_retry_count_{0};
  std::size_t diff_follower_retreat_target_index_{std::numeric_limits<std::size_t>::max()};
  int recovery_attempt_count_{0};
  int diff_planning_failure_events_{0};  // 2026-07-28: Diff连续失败诊断计数。
  int diff_failure_retreat_attempts_{0};
  uint32_t trajectory_id_{0};
};

int main(int argc, char** argv) {
  // 2026-07-14: 后机跟随节点与两架控制器分离，便于单独停用并回退到 hold_only。
  ros::init(argc, argv, "leader_safe_path_follower");
  LeaderSafePathFollower follower;
  ros::spin();
  return 0;
}
