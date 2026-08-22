// 2026-07-13: 新增比赛第二阶段任务搜索模块，使用搜索覆盖和目标状态主导选点，frontier 仅作为候选来源。
#ifndef TASK_SEARCH_MANAGER_H_
#define TASK_SEARCH_MANAGER_H_

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/RCIn.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <exploration_manager/motion_direction_rules.h>
#include <quadrotor_msgs/ExplorationGoal.h>

#include <deque>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace fast_planner {

class SDFMap;

class TaskSearchManager {
public:
  void initialize(ros::NodeHandle& nh);
  // 2026-07-13: 第三阶段直接读取 FUEL 累计占据地图，不依赖缺陷较大的单帧雷达点云。
  void setMap(const std::shared_ptr<SDFMap>& map);
  void setCorridorFrame(const Eigen::Vector3d& origin, const Eigen::Vector3d& inside_dir);
  void updateRobotPose(const Eigen::Vector3d& pos, double yaw);
  int selectSearchCandidate(const std::vector<Eigen::Vector3d>& points,
                            const std::vector<double>& yaws,
                            const std::vector<std::vector<Eigen::Vector3d>>& frontiers,
                            const Eigen::Vector3d& cur_pos, double cur_yaw);
  std::vector<Eigen::Vector3d> recoveryDirections(double cur_yaw);
  // 2026-07-28: 仅供局部脱困判断真实最近航向和短回撤方向；普通frontier仍执行全局禁回头。
  Eigen::Vector3d recoveryForwardDirection(double cur_yaw);
  // 向A*提供当前通道的单调前进轴；地图确认转弯后自动切换到新通道方向。
  bool astarNoReturnDirection(Eigen::Vector3d& direction) const;
  // 查询并锁存累计占据地图确认的真实转弯，直到机头对准新通道，防止位置恢复先消费结果。
  bool mappedCorridorDirection(double cur_yaw, Eigen::Vector3d& direction);
  // 已建立分段后，机头偏离实际通道轴线时只校正yaw，不重复建立分段或禁回门。
  bool corridorYawCorrectionDirection(double cur_yaw,
                                      Eigen::Vector3d& direction) const;
  bool turnYawAlignmentPending() const { return turn_yaw_follow_latch_.active(); }
  bool isRecoveryDirectionBackward(const Eigen::Vector3d& direction,
                                   double cur_yaw);
  bool isRecoveryCandidateUseful(const Eigen::Vector3d& candidate) const;
  bool isTaskMotionAllowed(const Eigen::Vector3d& candidate) const;
  bool isMissionBoundaryMotionAllowed(const Eigen::Vector3d& candidate) const;
  bool isTaskPathAllowed(const std::vector<Eigen::Vector3d>& path) const;
  // 2026-07-28: 短回撤只豁免路径开头方向，门平面、旧通道和逐点障碍约束仍全部保留。
  bool isRecoveryPathAllowed(const std::vector<Eigen::Vector3d>& path,
                             bool allow_initial_reverse) const;
  void freezeExplorationInitialYaw(double yaw);
  bool explorationInitialYawFrozen() const { return exploration_initial_yaw_frozen_; }
  void fillExplorationConstraint(quadrotor_msgs::ExplorationMotionConstraint& msg) const;
  void recordExplorationReached(const Eigen::Vector3d& goal);
  double clampSearchHeight(double z) const;
  double preferredSearchHeight() const;
  // 2026-07-13: 窄通道任务点优先保持水平飞行，仅以有限步长向巡航高度收敛。
  double projectSearchHeight(double candidate_z, double current_z) const;
  void recordSelectedGoal(const Eigen::Vector3d& goal);
  void reportGoalFailure(const Eigen::Vector3d& goal);
  void clearActiveGoal() { active_goal_valid_ = false; }
  // 2026-07-13: search_exhausted 只负责允许切换阶段，出口位置始终由地图拓扑决定。
  bool buildStage3Goal(const Eigen::Vector3d& cur_pos, double cur_yaw, bool search_exhausted,
                       Eigen::Vector3d& goal, double& goal_yaw);
  // 2026-07-16: 仍有frontier/viewpoint时明确撤销“搜索耗尽”计时，失败冷却不能被误当成终点条件。
  void reportSearchCoverageAvailable();
  bool landingRequested() const { return landing_requested_; }
  // CH9 已完成低->高确认时，允许 FUEL 在没有外部航点的情况下启动一次规划。
  bool consumeRcSearchLandingStartRequest();
  bool stage3Active() const { return mission_stage_ != SEARCH_CORRIDOR; }
  // 普通FUEL搜索期间入口始终是单向门，防止开阔的起飞区frontier把飞机吸回门外；
  // 只有最终出口状态机生成的专用穿门轨迹可以临时绕过入口半平面。
  bool entryWorkspaceLockActive() const { return !exitTransitActive(); }
  // 2026-07-23: 只有“到门内点/正在穿门”允许跨越最近历史航迹；一旦进入门外搜索，
  // 出口立即成为单向门，后续目标和路径必须继续留在外侧。
  bool exitTransitActive() const {
    return mission_stage_ == EXIT_APPROACH_INSIDE || mission_stage_ == CROSS_EXIT;
  }
  bool enabled() const { return enabled_; }
  // 2026-07-23: 稳定且里程足够远的最终门锁存后先冻结普通frontier，等待耗尽确认切入
  // 出口状态；中途短里程墙端仍允许被越过/撤销，不能抢占普通通道搜索。
  bool finalExitFrontierGuardActive() const {
    return exit_portal_locked_ ||
           mission_stage_ == EXIT_APPROACH_INSIDE || mission_stage_ == CROSS_EXIT;
  }
  // 2026-07-24: 出口候选从第一票开始就属于连续雷达验证窗口；前机相机摆头必须让路，
  // 否则门框会被当作新障碍触发环扫，并把下一帧双墙终止切面打乱。
  bool exitVerificationActive() const {
    return mission_stage_ == SEARCH_CORRIDOR &&
           (exit_candidate_hits_ > 0 ||
            (!exit_verification_hold_until_.isZero() &&
             ros::Time::now() < exit_verification_hold_until_));
  }

private:
  struct TargetRecord {
    bool found{false};
    geometry_msgs::PoseStamped pose;
    ros::Time stamp;
  };

  enum MissionStage {
    SEARCH_CORRIDOR = 0,
    EXIT_APPROACH_INSIDE = 1,
    CROSS_EXIT = 2,
    SEARCH_OUTSIDE_LANDING = 3,
    APPROACH_LANDING = 4,
    LANDING = 5
  };

  void colorDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void qrcodeDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void thermalDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void finalLandingMarkerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void rcSearchLandingCallback(const mavros_msgs::RCInConstPtr& msg);
  void activateRcSearchLanding(const ros::Time& stamp);
  // 2026-07-23: 用短时机体系 Mid360 点云检测通道双墙共同终止，出口判断不再依赖会随 z 漂移失真的绝对高度切片。
  void bodyCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void registerDetection(int type, const geometry_msgs::PoseStamped& msg);
  void updateExitCandidate(const Eigen::Vector3d& cur_pos);
  struct RadarExitResult {
    Eigen::Vector3d portal_center{0.0, 0.0, 0.0};
    Eigen::Vector2d outward_direction{1.0, 0.0};
    double confidence{0.0};
    int inside_left_support{0};
    int inside_right_support{0};
    int bilateral_changed{0};
    int clear_ray_frames{0};
  };
  // 2026-07-23: 独立局部雷达确认器只使用短时水平射线、XY航迹切向和墙体连续性；
  // 长期占据地图给出的远端只作任务拓扑候选，不能单独确认出口。
  bool detectRadarExit(const Eigen::Vector3d& cur_pos,
                       const Eigen::Vector2d& forward_hint,
                       RadarExitResult& result);
  // 2026-07-23: 单通道出口主判据改为当前机附近累计占据地图的XY双墙终止；
  // 使用相对当前里程计高度带，避免绝对z退化，也避免body点云拼接方向误差否决清晰断面。
  bool detectLocalMapExit(const Eigen::Vector3d& cur_pos,
                          const Eigen::Vector2d& forward_hint,
                          RadarExitResult& result) const;
  bool mapRelativeColumnOccupied(const Eigen::Vector3d& point,
                                 double reference_z) const;
  // 2026-07-22: 拓扑远端只输出预测锚点；沿路径另找最靠远端的“连续双墙共同终止”出口截面。
  bool inferExitFromMap(const Eigen::Vector3d& cur_pos, Eigen::Vector3d& candidate,
                        Eigen::Vector3d& portal_center, Eigen::Vector2d& outward_direction,
                        Eigen::Vector3d& endpoint_prediction,
                        bool& endpoint_outside_verified,
                        double& geodesic_distance, double& confidence);
  // 2026-07-21: 用累计占据地图验证左右边界、中间净空和门外非占据延伸，排除普通拐角/死胡同。
  bool validateExitPortal(const Eigen::Vector3d& inside_hint,
                          const Eigen::Vector3d& portal_center,
                          const Eigen::Vector2d& outward_direction) const;
  // 2026-07-21: 穿门只发布已成为FREE的短滚动目标，UNKNOWN只用于证明开口延伸，不能直接飞入。
  bool buildSafeExitCrossingGoal(const Eigen::Vector3d& cur_pos, Eigen::Vector3d& goal) const;
  // 2026-07-22: 出口候选接近确认且普通frontier耗尽时，发布门内局部补扫点，避免停在门口无法补齐地图证据。
  bool buildPendingExitObservationGoal(const Eigen::Vector3d& cur_pos,
                                       Eigen::Vector3d& goal, double& goal_yaw) const;
  bool mapPointSafe(const Eigen::Vector3d& point) const;
  bool landingColumnSafe(const Eigen::Vector3d& marker,
                         const Eigen::Vector3d& approach) const;
  bool mapColumnOccupied(const Eigen::Vector3d& point) const;
  // 2026-07-22: 用累计占据柱沿指定方向的连续支撑判断墙面，避免把孤立箱体/柱子当作通道墙或门框。
  bool mapWallRaySupported(const Eigen::Vector3d& start,
                           const Eigen::Vector2d& direction,
                           double length, double min_ratio) const;
  // 2026-07-28: 从累计占据地图的前墙和至少一侧连续墙轮廓推断正常拐弯轴线，
  // 供恢复器在90度弯道替换已经失效的历史直行切线。
  bool inferOccupancyTurnDirection(const Eigen::Vector3d& travel_direction,
                                   Eigen::Vector3d& turn_direction,
                                   double& forward_free_length,
                                   double& turn_free_length) const;
  void commitCorridorTurn(const Eigen::Vector3d& turn_direction);
  bool confirmCorridorTurnEvidence(const Eigen::Vector3d& turn_direction);
  void clearPendingTurnEvidence();
  bool allStage2TargetsFound() const;
  bool goalTemporarilyBlocked(const Eigen::Vector3d& goal) const;
  void publishLandingRequest(bool active);
  void publishSearchState();
  double minDistance2D(const Eigen::Vector3d& point,
                       const std::deque<Eigen::Vector3d>& history) const;
  double knownHorizontalClearance(const Eigen::Vector3d& point) const;
  // 独立于瞬时速度维护任务推进方向：正常转弯可逐段更新，短回撤不能把前后语义翻转。
  Eigen::Vector2d stableProgressDirection() const;
  double wrapYaw(double yaw) const;

  ros::Subscriber color_detection_sub_;
  ros::Subscriber qrcode_detection_sub_;
  ros::Subscriber thermal_detection_sub_;
  ros::Subscriber final_landing_marker_sub_;
  ros::Subscriber body_cloud_sub_;
  ros::Subscriber rc_search_landing_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher status_pub_;
  ros::Publisher exit_pose_pub_;
  ros::Publisher final_exit_pose_pub_;  // 2026-07-28: 只发布达到锁门里程的最终出口，供后机接力。
  ros::Publisher landing_target_pub_;
  ros::Publisher landing_request_pub_;

  std::shared_ptr<SDFMap> sdf_map_;

  bool enabled_{true};
  // Hybrid route history must not alter native FUEL exploration behavior.
  bool hybrid_constraints_enabled_{false};
  // 2026-07-16: 颜色、普通二维码、温度识别接口始终保留；无摄像头时可关闭其任务完成门槛。
  bool require_stage2_detections_{true};
  bool corridor_frame_received_{false};
  std::string world_frame_{"world"};
  Eigen::Vector3d corridor_origin_{0.0, 0.0, 0.0};
  Eigen::Vector3d corridor_dir_{1.0, 0.0, 0.0};
  Eigen::Vector2d stable_progress_direction_{1.0, 0.0};
  bool stable_progress_direction_valid_{false};
  task_search::TurnYawFollowLatch turn_yaw_follow_latch_;
  Eigen::Vector2d pending_turn_direction_{1.0, 0.0};
  Eigen::Vector3d pending_turn_probe_origin_{0.0, 0.0, 0.0};
  bool pending_turn_probe_origin_valid_{false};
  ros::Time pending_turn_last_evidence_;
  int pending_turn_confirmations_{0};
  std::uint64_t pending_turn_pose_sequence_{0};
  Eigen::Vector2d latest_turn_anchor_{0.0, 0.0};
  Eigen::Vector2d latest_turn_incoming_direction_{1.0, 0.0};
  bool latest_turn_anchor_valid_{false};
  struct CompletedGate {
    std::uint32_t sequence{0};
    Eigen::Vector2d center{0.0, 0.0};
    Eigen::Vector2d normal{1.0, 0.0};
    double left_extent{1.45};
    double right_extent{1.45};
    double thickness{0.30};
  };
  std::vector<CompletedGate> completed_gates_;
  std::uint32_t next_gate_sequence_{1};
  bool exploration_initial_yaw_frozen_{false};
  double exploration_initial_yaw_{0.0};
  double entry_inside_yaw_{0.0};
  Eigen::Vector2d segment_origin_{0.0, 0.0};
  Eigen::Vector2d segment_direction_{1.0, 0.0};
  double segment_high_water_{0.0};
  bool transition_active_{false};
  Eigen::Vector2d transition_anchor_{0.0, 0.0};
  Eigen::Vector2d transition_incoming_direction_{1.0, 0.0};
  Eigen::Vector2d transition_outgoing_direction_{1.0, 0.0};
  double transition_old_high_water_{0.0};
  std::deque<Eigen::Vector3d> visited_positions_;
  std::deque<Eigen::Vector3d> selected_goals_;
  // 2026-07-23: 每帧保留雷达原点和近水平回波的世界XY；不保存/比较世界z，从源头隔离高度退化。
  struct BodyCloudFrame {
    ros::Time stamp;
    Eigen::Vector2d sensor_xy{0.0, 0.0};
    std::vector<Eigen::Vector2d> endpoints_xy;
  };
  std::deque<BodyCloudFrame> body_cloud_frames_;
  mutable std::mutex body_cloud_mutex_;
  Eigen::Vector3d latest_robot_pos_{0.0, 0.0, 0.0};
  double latest_robot_yaw_{0.0};
  bool latest_robot_pose_valid_{false};
  std::uint64_t latest_robot_pose_sequence_{0};
  // 2026-07-23: 任务区域只允许“起点外 -> 入口 -> 通道内 -> 最终出口 -> 终点外”
  // 一次性状态推进；入口穿越一旦锁存，任何局部墙端/拐角都不能把状态重新解释成门外。
  bool corridor_entry_crossed_{false};
  TargetRecord targets_[3];
  TargetRecord final_landing_marker_;
  MissionStage mission_stage_{SEARCH_CORRIDOR};

  // CH9 仅由 UAV0 读取。必须先见低位再持续高位，避免上电时拨杆已在高位而误触发。
  bool rc_search_landing_enabled_{false};
  std::string rc_search_landing_topic_{"/UAV0/mavros/rc/in"};
  int rc_search_landing_channel_{8};
  int rc_search_landing_low_pwm_{1300};
  int rc_search_landing_high_pwm_{1800};
  double rc_search_landing_hold_sec_{0.5};
  bool rc_search_landing_armed_{false};
  bool rc_search_landing_triggered_{false};
  bool rc_search_landing_start_requested_{false};
  ros::Time rc_search_landing_high_since_;

  // 2026-07-13: 出口候选需要跨多次地图更新稳定，不能由单帧噪声直接触发第三阶段。
  bool exit_candidate_confirmed_{false};
  // 2026-07-23: 与短时5/5结构确认分离；累计通道里程足够远后才永久冻结最终门中心和法向。
  bool exit_portal_locked_{false};
  Eigen::Vector3d exit_candidate_{0.0, 0.0, 0.75};
  Eigen::Vector3d pending_exit_candidate_{0.0, 0.0, 0.75};
  // 2026-07-21: 区分门内接近点、门框中心和穿出方向；拓扑远端不再与降落点共用一个变量。
  Eigen::Vector3d exit_portal_center_{0.0, 0.0, 0.75};
  Eigen::Vector3d pending_exit_portal_center_{0.0, 0.0, 0.75};
  Eigen::Vector2d exit_outward_direction_{1.0, 0.0};
  Eigen::Vector2d pending_exit_outward_direction_{1.0, 0.0};
  // 2026-07-22: 终点预测点只用于给出口检测提供拓扑路径，不发布为飞行/降落目标。
  Eigen::Vector3d exit_endpoint_prediction_{0.0, 0.0, 0.75};
  Eigen::Vector3d pending_exit_endpoint_prediction_{0.0, 0.0, 0.75};
  bool exit_endpoint_outside_verified_{false};
  bool pending_exit_endpoint_outside_verified_{false};
  // 2026-07-23: 区分“门外开放空间已验证”和“紫色拓扑预测点位于门外”；
  // 前者控制穿门，后者只解释预测点，避免把门内预测点硬标成outside。
  bool exit_portal_outside_verified_{false};
  bool pending_exit_portal_outside_verified_{false};
  Eigen::Vector3d outside_search_anchor_{0.0, 0.0, 0.75};
  double exit_geodesic_distance_{0.0};
  double exit_confidence_{0.0};
  int exit_candidate_hits_{0};
  // 2026-07-24: 单帧门证据短暂丢失后仍保持前视一小段时间，避免立刻重启环扫形成1->0循环。
  ros::Time exit_verification_hold_until_;
  double exit_verification_yaw_hold_time_{2.5};
  int final_landing_marker_hits_{0};
  bool landing_requested_{false};
  ros::Time last_exit_inference_;
  ros::Time mission_stage_start_;
  ros::Time search_exhausted_since_;
  ros::Time active_goal_stamp_;
  Eigen::Vector3d active_goal_{0.0, 0.0, 0.0};
  bool active_goal_valid_{false};
  std::deque<std::pair<Eigen::Vector3d, ros::Time>> failed_goals_;

  double visit_spacing_{0.35};
  double revisit_radius_{0.65};
  double repeat_goal_radius_{0.45};
  int max_goal_repeats_{2};
  int max_history_size_{120};
  double cruise_height_{0.75};
  double min_search_height_{0.55};
  double max_search_height_{1.00};
  // 2026-07-13: 限制单个搜索目标的升降量，避免 frontier 高度把无人机持续拉向天花板。
  double max_goal_climb_{0.06};
  double max_goal_descent_{0.10};
  double novelty_weight_{2.5};
  double travel_weight_{0.65};
  double yaw_weight_{0.20};
  double height_weight_{2.0};
  double repeat_penalty_{6.0};
  double frontier_gain_weight_{0.10};
  bool clearance_reward_enabled_{true};
  double clearance_reward_start_{0.30};
  double clearance_reward_full_{0.60};
  double clearance_reward_max_{5.0};
  double entry_forward_distance_{2.0};
  double entry_forward_weight_{1.5};
  double forward_viewpoint_bonus_{5.0};
  double backward_viewpoint_penalty_{5.0};
  // 2026-07-21: 普通单通道搜索默认禁止把“暂无前向候选”解释成掉头；显式故障回撤才允许后向恢复。
  bool prefer_motion_forward_{true};
  bool allow_search_backtrack_{false};
  bool global_no_return_{true};
  double backward_cos_threshold_{-0.15};
  double min_goal_hold_time_{1.2};
  // 2026-07-14: 对尚未到达的活动目标增加空间连续性代价，避免相邻重规划周期跨数米反向换点。
  double goal_switch_weight_{1.2};
  double failed_goal_radius_{0.55};
  double failed_goal_cooldown_{2.0};
  double inside_return_margin_{0.10};
  double entry_path_direction_grace_distance_{1.50};
  // 2026-07-28: 恢复器仅在旧前向受阻、侧向已知FREE明显更长且左右墙连续时采用地图拐弯方向。
  bool recovery_occupancy_turn_enabled_{true};
  double recovery_turn_probe_length_{1.50};
  double recovery_turn_probe_step_{0.10};
  double recovery_turn_min_free_length_{0.50};
  double recovery_turn_min_free_gain_{0.15};
  double recovery_turn_min_angle_deg_{30.0};
  double recovery_turn_max_angle_deg_{120.0};
  double recovery_turn_wall_min_half_width_{0.35};
  double recovery_turn_wall_max_half_width_{1.05};
  int recovery_turn_min_wall_support_{2};
  int recovery_turn_confirmation_count_{1};
  double recovery_turn_confirmation_min_interval_{0.15};
  double recovery_turn_confirmation_angle_deg_{15.0};
  double recovery_turn_confirmation_accumulation_window_{8.0};
  double recovery_turn_long_view_length_{4.50};
  double recovery_turn_long_view_step_{0.25};
  double recovery_turn_long_view_min_depth_{2.00};
  int recovery_turn_long_view_min_free_sections_{5};
  int recovery_turn_long_view_min_wall_sections_{3};
  int recovery_turn_long_view_min_paired_wall_sections_{4};
  double recovery_turn_long_view_width_tolerance_{0.30};
  double recovery_turn_long_view_center_tolerance_{0.25};
  double recovery_turn_no_return_margin_{0.20};
  double recovery_turn_yaw_release_angle_deg_{15.0};


  // 2026-07-13: 第三阶段地图拓扑、出口确认、平台扫描和降落触发参数。
  // 关闭后不运行地图/雷达出口检测，也不进入出口穿越和降落状态机。
  bool exit_detection_enabled_{true};
  double exit_grid_resolution_{0.20};
  double exit_inference_period_{1.0};
  double exit_min_geodesic_distance_{4.0};
  double exit_lock_min_geodesic_distance_{8.0};
  double exit_candidate_stability_radius_{0.80};
  int exit_confirmation_count_{3};
  // 2026-07-22: 出口按任务推进方向只允许被更靠拓扑远端的稳定双墙终止截面修正。
  int exit_supersede_confirmation_count_{2};
  double exit_supersede_min_path_increase_{0.60};
  // 2026-07-16: 无可覆盖frontier必须持续稳定一段时间，避免一次瞬时过滤为空就锁死错误出口。
  double search_exhausted_confirm_time_{3.0};
  double exit_standoff_{0.45};
  double exit_arrive_distance_{0.50};
  // 2026-07-21: 门框验证和短步穿越阈值，均基于累计占据地图而非原始单帧点云。
  double exit_portal_min_half_width_{0.42};
  double exit_portal_max_half_width_{1.10};
  double exit_portal_side_sample_step_{0.10};
  // 2026-07-22: 门洞净空与门框搜索保留独立边界，避免采样邻域在0.42m处互相重叠。
  double exit_portal_core_margin_{0.10};
  // 2026-07-22: 出口门框只接受工作高度带内具有多层连续支撑的柱状占据；地板和单个漂移点不能充当门柱。
  double exit_column_lower_margin_{0.15};
  double exit_column_upper_margin_{0.15};
  double exit_column_sample_step_{0.10};
  double exit_column_min_z_{0.18};
  double exit_column_max_z_{1.55};
  int exit_column_min_occupied_layers_{2};
  double exit_column_min_vertical_span_{0.18};
  double exit_transition_inside_probe_{0.45};
  double exit_transition_outside_probe_{0.65};
  // 2026-07-22: 出口结构先排除中心线正常转弯，再要求左右连续通道墙在相近截面同步终止；
  // 横向门框只作加分证据，不能让通道内的柱子/箱体单独触发出口。
  double exit_max_centerline_turn_deg_{35.0};
  double exit_turn_window_distance_{0.60};
  double exit_wall_support_length_{0.90};
  double exit_wall_support_min_ratio_{0.55};
  double exit_frame_outward_support_length_{0.30};
  double exit_min_bilateral_wall_shift_{0.18};
  // 2026-07-22: 用多个门内/门外截面验证墙轨迹生命周期，区分连续墙、局部障碍物和点云空洞。
  double exit_wall_track_width_tolerance_{0.25};
  double exit_wall_end_probe_step_{0.25};
  int exit_wall_end_probe_count_{4};
  int exit_wall_end_min_changed_count_{3};
  double exit_outside_min_free_ratio_{0.75};
  // 2026-07-22: 单通道任务中，已经被前机越过的历史截面永久失效，禁止锁门后再要求回头。
  double exit_candidate_max_behind_distance_{0.35};
  // 单段接近反向的运动视为短回撤，不更新任务推进方向；门候选也不得相对稳定方向突变。
  double exit_progress_reverse_reject_deg_{120.0};
  double exit_max_direction_jump_deg_{100.0};
  // 2026-07-28: 出口只能在飞机接近门平面后确认，避免远距离稀疏点云提前锁门。
  double exit_confirmation_max_distance_{1.00};
  // 2026-07-28: 深入通道后全局FREE拓扑断链时，允许近场直线双墙终止结构降级接管。
  double exit_near_fallback_min_travel_{10.0};
  double exit_near_fallback_max_distance_{1.0};
  double exit_near_fallback_max_turn_deg_{25.0};
  // 2026-07-23: 保存拓扑父链判定出的正常拐弯截面，禁止局部雷达把同一弯角重新包装成出口。
  double exit_normal_turn_reject_radius_{0.90};
  std::vector<Eigen::Vector3d> last_topology_turn_centers_;
  // 2026-07-23: 短时机体系出口检测参数；水平仰角过滤地面/顶棚，双墙终止和穿门自由射线共同确认。
  bool radar_exit_detection_enabled_{true};
  // 2026-07-27: 出口雷达复核默认话题前缀统一为 UAV0。
  std::string radar_exit_body_cloud_topic_{"/UAV0/fast_lio/cloud_registered_body"};
  double radar_exit_accumulation_time_{1.50};
  double radar_exit_max_elevation_deg_{12.0};
  double radar_exit_min_range_{0.25};
  double radar_exit_max_range_{12.0};
  int radar_exit_max_points_per_frame_{1800};
  double radar_exit_search_min_forward_{-0.15};
  double radar_exit_search_max_forward_{1.25};
  double radar_exit_search_step_{0.10};
  double radar_exit_wall_bin_step_{0.12};
  double radar_exit_inside_length_{0.90};
  int radar_exit_min_inside_bins_{5};
  double radar_exit_wall_track_tolerance_{0.22};
  double radar_exit_min_wall_shift_{0.18};
  int radar_exit_outside_probe_count_{4};
  int radar_exit_min_changed_probes_{3};
  double radar_exit_clear_half_width_{0.28};
  double radar_exit_clear_depth_{0.80};
  int radar_exit_min_clear_ray_frames_{2};
  // 2026-07-22: 预测点必须在门框之后保有足够长的已知FREE拓扑分支，才判为位于通道外。
  double exit_endpoint_outside_route_min_length_{1.00};
  // 2026-07-22: 单次门洞复核失败只衰减稳定计数；高稳定候选可触发门内补扫，不再瞬时归零后原地卡死。
  int exit_candidate_reject_decay_{1};
  int exit_pending_observation_min_hits_{3};
  double exit_pending_observation_lateral_offset_{0.25};
  double exit_pending_observation_min_move_{0.22};
  double exit_outside_probe_distance_{0.90};
  int exit_outside_min_nonoccupied_samples_{3};
  double exit_cross_step_{0.35};
  double exit_cross_target_distance_{0.90};
  double exit_cross_confirm_distance_{0.45};
  double exit_footprint_extra_radius_{0.22};
  int exit_footprint_samples_{8};
  double scan_radius_{0.35};
  double scan_dwell_time_{1.2};
  // true时FUEL只发布出口切换阶段，门外搜索、平台接近和降落请求全部交给外部Diff链路。
  bool external_landing_planner_{false};
  bool require_final_landing_marker_{true};
  // 2026-07-20: 最终出口确认后仍允许出口附近绕障微调，远处普通frontier只能有限增加
  // 到出口的距离，防止重新发布整段回头长路径。
  double exit_frontier_local_adjust_radius_{1.20};
  double exit_frontier_max_distance_increase_{0.45};
  int final_landing_marker_confirmation_count_{3};
  double final_landing_marker_consistency_radius_{0.40};
  double landing_approach_height_{0.70};
  double landing_trigger_distance_{0.35};
  double landing_column_bottom_clearance_{0.20};
  double landing_column_step_{0.10};
};

}  // namespace fast_planner

#endif
