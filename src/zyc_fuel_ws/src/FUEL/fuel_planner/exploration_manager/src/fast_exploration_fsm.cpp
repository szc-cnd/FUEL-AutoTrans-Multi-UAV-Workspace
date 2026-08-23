
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/replan_execution_policy.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

#include <limits>

using Eigen::Vector4d;

namespace fast_planner {
void FastExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  // 2026-07-13: 最新日志出现 0.35~0.44m 跟踪误差，超过机体安全余量时不允许继续执行旧轨迹。
  nh.param("fsm/max_tracking_error_xy", fp_->max_tracking_error_xy_, 0.30);
  nh.param("fsm/max_tracking_error_z", fp_->max_tracking_error_z_, 0.25);
  // 2026-07-13: 新轨迹交接和短时误差先平滑重规划，只有持续或严重偏差才触发急停。
  nh.param("fsm/tracking_error_confirm_time", fp_->tracking_error_confirm_time_, 0.25);
  nh.param("fsm/tracking_error_grace_time", fp_->tracking_error_grace_time_, 0.35);
  nh.param("fsm/hard_tracking_error_xy", fp_->hard_tracking_error_xy_, 0.65);
  nh.param("fsm/hard_tracking_error_z", fp_->hard_tracking_error_z_, 0.50);
  // 2026-07-22: FAIL后保持悬停并等待地图更新，禁止100Hz重复规划同一目标。
  nh.param("fsm/plan_failure_retry_interval", fp_->plan_failure_retry_interval_, 0.50);
  nh.param("fsm/safety_hold_enabled", safety_hold_enabled_, true);
  nh.param("fsm/hold_on_plan_failure", hold_on_plan_failure_, true);
  nh.param("fsm/endpoint_hold_enabled", endpoint_hold_enabled_, true);
  nh.param("fsm/endpoint_hold_lead_time", endpoint_hold_lead_time_, 0.05);
  endpoint_hold_lead_time_ = std::max(0.0, endpoint_hold_lead_time_);
  nh.param("fsm/periodic_replan_enabled", periodic_replan_enabled_, true);
  nh.param("fsm/use_diff_for_fuel_exploration", use_diff_for_fuel_exploration_, false);
  nh.param("fsm/fuel_diff_goal_topic", external_goal_topic_, external_goal_topic_);
  nh.param("fsm/fuel_diff_status_topic", external_status_topic_, external_status_topic_);
  nh.param("fsm/fuel_diff_cancel_topic", external_cancel_topic_, external_cancel_topic_);
  nh.param("fsm/fuel_diff_trigger_topic", external_trigger_topic_, external_trigger_topic_);
  // 2026-07-28: 连续复核覆盖至少两次20Hz地图/安全周期；起点误差过大则从真实里程计重规划。
  nh.param("fsm/trajectory_release_confirm_time", fp_->trajectory_release_confirm_time_, 0.12);
  nh.param("fsm/trajectory_release_check_interval", fp_->trajectory_release_check_interval_, 0.04);
  nh.param("fsm/trajectory_release_max_start_error",
           fp_->trajectory_release_max_start_error_, 0.25);
  nh.param("fsm/emergency_brake_horizon", fp_->emergency_brake_horizon_, 0.12);

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  planner_manager_ = expl_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  next_plan_retry_time_ = ros::Time(0);
  pending_traj_safe_since_ = ros::Time(0);
  next_pending_traj_check_ = ros::Time(0);
  active_traj_valid_ = false;
  active_traj_braked_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

  trigger_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &FastExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);
  // 任务状态决定门内/门外，并对外发布统一的动态检测阶段状态。
  std::string mission_status_topic;
  std::string dynamic_detection_enable_topic;
  nh.param("fsm/mission_status_topic", mission_status_topic,
           std::string("/mission/task_status"));
  nh.param("fsm/dynamic_detection_enable_topic", dynamic_detection_enable_topic,
           std::string("/UAV0/corridor_search/dynamic_detection_enable"));
  mission_status_sub_ = nh.subscribe(mission_status_topic, 2,
                                     &FastExplorationFSM::missionStatusCallback, this);
  if (use_diff_for_fuel_exploration_) {
    external_status_sub_ = nh.subscribe(
        external_status_topic_, 10, &FastExplorationFSM::externalStatusCallback, this);
    external_goal_pub_ = nh.advertise<quadrotor_msgs::ExplorationGoal>(
        external_goal_topic_, 2, false);
    external_cancel_pub_ = nh.advertise<quadrotor_msgs::ExplorationCancel>(
        external_cancel_topic_, 2, false);
    external_trigger_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        external_trigger_topic_, 2, true);
    external_session_id_ = static_cast<std::uint64_t>(ros::Time::now().toNSec());
  }

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
  emergency_brake_pub_ = nh.advertise<std_msgs::Int32>("/planning/emergency_brake", 2);
  // 2026-07-13: latch 保证后启动的控制器也能收到当前安全门控状态。
  safety_hold_pub_ = nh.advertise<std_msgs::Bool>("/planning/safety_hold", 2, true);
  endpoint_hold_pub_ = nh.advertise<std_msgs::Bool>("/planning/endpoint_hold", 2, true);
  dynamic_detection_enable_pub_ =
      nh.advertise<std_msgs::Bool>(dynamic_detection_enable_topic, 2, true);
  setSafetyHold(false, "initialization");
  setEndpointHold(false, "initialization");
  // 锁存初始false，后启动的监控节点也能获得当前阶段。
  setDynamicDetectionEnable(false, "initialization", true);
}

void FastExplorationFSM::setSafetyHold(bool active, const string& reason) {
  if (active && !safety_hold_enabled_) {
    ROS_WARN_THROTTLE(1.0, "[safety_hold] ignored reason=%s; FUEL hold disabled.",
                      reason.c_str());
    return;
  }
  if (safety_hold_active_ == active && reason != "initialization") return;
  safety_hold_active_ = active;
  std_msgs::Bool msg;
  msg.data = active;
  safety_hold_pub_.publish(msg);
  ROS_WARN("[safety_hold] %s reason=%s.", active ? "ACTIVE" : "RELEASED", reason.c_str());
}

void FastExplorationFSM::setEndpointHold(bool active, const string& reason) {
  if (!endpoint_hold_enabled_ && active) return;
  if (endpoint_hold_active_ == active && reason != "initialization") return;
  endpoint_hold_active_ = active;
  std_msgs::Bool msg;
  msg.data = active;
  endpoint_hold_pub_.publish(msg);
  ROS_WARN("[endpoint_hold] %s reason=%s.", active ? "ACTIVE" : "RELEASED",
           reason.c_str());
}

void FastExplorationFSM::requestActiveTrajectoryBrake(const string& reason) {
  if (!active_traj_valid_ || active_traj_braked_) return;

  std_msgs::Int32 brake_msg;
  brake_msg.data = active_traj_.traj_id_;
  emergency_brake_pub_.publish(brake_msg);
  const double elapsed = std::max(0.0, (ros::Time::now() - active_traj_.start_time_).toSec());
  active_traj_.duration_ =
      std::min(active_traj_.duration_, elapsed + std::max(0.02, fp_->emergency_brake_horizon_));
  active_traj_braked_ = true;
  fd_->static_state_ = true;
  ROS_ERROR("[trajectory_brake] requested reason=%s horizon=%.2fs.", reason.c_str(),
            fp_->emergency_brake_horizon_);
}

// 统一发布规划器判定后的门内动态检测阶段，重复状态不重复刷屏。
void FastExplorationFSM::setDynamicDetectionEnable(bool active, const string& reason,
                                                   bool force) {
  if (!force && dynamic_detection_enabled_ == active) return;
  dynamic_detection_enabled_ = active;
  std_msgs::Bool msg;
  msg.data = active;
  dynamic_detection_enable_pub_.publish(msg);
  ROS_WARN("[dynamic_detection_gate] %s reason=%s.", active ? "ENABLED" : "DISABLED",
           reason.c_str());
}

// 2026-07-27: 通道内、出口门内接近及穿门阶段保持检测；确认门外搜索后立即关闭。
void FastExplorationFSM::missionStatusCallback(const std_msgs::StringConstPtr &msg) {
  const std::string &state = msg->data;
  narrow_corridor_stage_ =
      state.find("SEARCH_CORRIDOR") == 0 ||
      state.find("EXIT_APPROACH_INSIDE") == 0 ||
      state.find("CROSS_EXIT") == 0;
  mission_allows_dynamic_detection_ = narrow_corridor_stage_;
  setDynamicDetectionEnable(first_corridor_traj_published_ &&
                                mission_allows_dynamic_detection_,
                            mission_allows_dynamic_detection_
                                ? "inside corridor with published trajectory"
                                : "outside corridor mission stage");
  const bool landing_stage = state.find("SEARCH_OUTSIDE_LANDING") == 0 ||
      state.find("SEARCH_OUTSIDE_QR") == 0 || state.find("APPROACH_LANDING") == 0 ||
      state.find("LANDING") == 0;
  if (use_diff_for_fuel_exploration_ && landing_stage) {
    if (external_goal_pending_) {
      quadrotor_msgs::ExplorationCancel cancel;
      cancel.header.stamp = ros::Time::now();
      cancel.session_id = external_session_id_;
      cancel.goal_id = external_goal_id_;
      cancel.reason = "landing handoff";
      external_cancel_pub_.publish(cancel);
    }
    external_exploration_active_ = false;
    external_goal_pending_ = false;
    setSafetyHold(true, "landing handoff");
  }
}

void FastExplorationFSM::activateExternalExploration(
    const geometry_msgs::PoseStamped& entry_trigger) {
  external_exploration_active_ = true;
  external_goal_pending_ = false;
  external_next_select_at_ = ros::Time::now();
  if (fd_->have_odom_)
    expl_manager_->freezeExplorationInitialYaw(fd_->odom_yaw_);

  geometry_msgs::PoseStamped trigger = entry_trigger;
  trigger.header.stamp = ros::Time::now();
  if (trigger.header.frame_id.empty()) trigger.header.frame_id = "world";
  external_trigger_pub_.publish(trigger);
  setSafetyHold(true, "DIFF owns fuel exploration execution");
  ROS_WARN("[fuel_diff] external DIFF execution activated by entry trigger.");
}

bool FastExplorationFSM::publishExternalViewpoint() {
  if (!use_diff_for_fuel_exploration_ || !external_exploration_active_ ||
      !fd_->have_odom_) return false;
  const ros::Time now = ros::Time::now();
  expl_manager_->freezeExplorationInitialYaw(fd_->odom_yaw_);
  if (external_goal_pending_) {
    if ((now - external_goal_sent_at_).toSec() >= 1.0) {
      external_last_goal_.header.stamp = now;
      external_goal_pub_.publish(external_last_goal_);
      external_goal_sent_at_ = now;
    }
    return true;
  }
  if (!external_next_select_at_.isZero() && now < external_next_select_at_) return true;

  Vector3d next_pos;
  double next_yaw = fd_->odom_yaw_;
  if (!expl_manager_->selectExplorationViewpoint(
          fd_->odom_pos_, fd_->odom_vel_, fd_->start_acc_, fd_->start_yaw_,
          next_pos, next_yaw)) {
    external_next_select_at_ = now + ros::Duration(0.50);
    setSafetyHold(true, "FUEL has no executable viewpoint");
    return false;
  }
  external_last_pose_ = geometry_msgs::Pose();
  external_last_pose_.position.x = next_pos.x();
  external_last_pose_.position.y = next_pos.y();
  external_last_pose_.position.z = next_pos.z();
  external_last_pose_.orientation.w = std::cos(0.5 * next_yaw);
  external_last_pose_.orientation.z = std::sin(0.5 * next_yaw);
  quadrotor_msgs::ExplorationGoal goal;
  goal.header.stamp = now;
  goal.header.frame_id = "world";
  goal.session_id = external_session_id_;
  goal.goal_id = ++external_goal_id_;
  goal.target_type = quadrotor_msgs::ExplorationGoal::TARGET_FRONTIER;
  goal.motion_type = quadrotor_msgs::ExplorationGoal::MOTION_MOVE;
  goal.target_pose = external_last_pose_;
  goal.enforce_yaw = false;
  expl_manager_->fillExplorationConstraint(goal.motion_constraint);
  external_last_goal_ = goal;
  external_goal_pending_ = true;
  external_goal_sent_at_ = now;
  external_goal_pub_.publish(goal);
  setSafetyHold(true, "waiting for DIFF trajectory");
  ROS_INFO("[fuel_diff] publish goal session=%llu id=%llu view=(%.2f %.2f %.2f).",
           static_cast<unsigned long long>(goal.session_id),
           static_cast<unsigned long long>(goal.goal_id),
           next_pos.x(), next_pos.y(), next_pos.z());
  return true;
}

void FastExplorationFSM::externalStatusCallback(
    const quadrotor_msgs::ExplorationGoalStatusConstPtr& msg) {
  if (!use_diff_for_fuel_exploration_ || msg->session_id != external_session_id_ ||
      msg->goal_id != external_goal_id_ || !external_goal_pending_)
    return;
  if (msg->state == quadrotor_msgs::ExplorationGoalStatus::STATE_ACCEPTED ||
      msg->state == quadrotor_msgs::ExplorationGoalStatus::STATE_EXECUTING) {
    setSafetyHold(true, "DIFF accepted/executing viewpoint");
    return;
  }
  if (msg->state == quadrotor_msgs::ExplorationGoalStatus::STATE_REACHED) {
    expl_manager_->recordExplorationReached(
        Vector3d(external_last_pose_.position.x, external_last_pose_.position.y,
                 external_last_pose_.position.z));
    external_goal_pending_ = false;
    external_next_select_at_ = ros::Time::now() + ros::Duration(0.05);
    setSafetyHold(true, "viewpoint reached; selecting next FUEL viewpoint");
    return;
  }
  if (msg->state == quadrotor_msgs::ExplorationGoalStatus::STATE_PREEMPTED &&
      msg->reason == quadrotor_msgs::ExplorationGoalStatus::REASON_LANDING_HANDOFF) {
    external_goal_pending_ = false;
    external_exploration_active_ = false;
    return;
  }
  if (msg->state == quadrotor_msgs::ExplorationGoalStatus::STATE_REJECTED ||
      msg->state == quadrotor_msgs::ExplorationGoalStatus::STATE_PLANNING_FAILED ||
      msg->state == quadrotor_msgs::ExplorationGoalStatus::STATE_ABORTED ||
      msg->state == quadrotor_msgs::ExplorationGoalStatus::STATE_CANCELED) {
    expl_manager_->reportTrajectoryCollision();
    external_goal_pending_ = false;
    external_next_select_at_ = ros::Time::now() + ros::Duration(0.05);
    setSafetyHold(true, "DIFF failed viewpoint; FUEL selecting replacement");
  }
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

  // 已开始准备下一条轨迹但旧轨迹先到终点时，明确结束旧轨迹并让控制器锁住实时XY。
  // 仅在PLAN/PUB等待阶段触发；正常EXEC阶段仍提前1秒开始规划，不会提前刹停。
  if ((state_ == PLAN_TRAJ || state_ == PUB_TRAJ) && active_traj_valid_) {
    const double elapsed =
        std::max(0.0, (ros::Time::now() - active_traj_.start_time_).toSec());
    const double time_to_end = active_traj_.duration_ - elapsed;
    if (exploration_policy::shouldHoldAtTrajectoryEnd(
            true, false, active_traj_braked_, time_to_end,
            endpoint_hold_lead_time_)) {
      requestActiveTrajectoryBrake("old trajectory ended before replacement publish");
      setEndpointHold(true, "wait for validated replacement at old trajectory endpoint");
    }
  }

  if (use_diff_for_fuel_exploration_ && external_exploration_active_ &&
      state_ != INIT && state_ != FINISH) {
    fd_->static_state_ = true;
    fd_->start_acc_.setZero();
    setSafetyHold(true, "DIFF external execution owner");
    publishExternalViewpoint();
    return;
  }

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      // CH9 是搜索降落的完整人工入口：无需先发送普通航点。
      if (expl_manager_->consumeRcSearchLandingStartRequest()) {
        fd_->trigger_ = true;
        fd_->static_state_ = true;
        next_plan_retry_time_ = ros::Time(0);
        transitState(PLAN_TRAJ, "CH9 search-landing trigger");
        ROS_ERROR("[exit_mission] CH9 started search landing directly from WAIT_TRIGGER.");
        break;
      }
      // 普通通道搜索仍等待航点触发。
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH: {
      // 2026-07-13: exploration 结束或降落接管前不再沿用最后一条轨迹。
      setSafetyHold(true, "exploration finished");
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case PLAN_TRAJ: {
      // 2026-07-22: 失败冷却期间不发布replan、不重新选点也不刷新规划可视化。
      if (!next_plan_retry_time_.isZero() && ros::Time::now() < next_plan_retry_time_) return;
      // exploration_node 是单线程 spinner，规划函数运行期间安全定时器不能插入执行。
      // 因此每次开始一次可能耗时的重规划前，先同步复核 traj_server 正在执行的旧轨迹。
      if (active_traj_valid_ && !active_traj_braked_) {
        double active_collision_distance = 0.0;
        const bool raw_escape_safe =
            !inflation_escape_active_ ||
            planner_manager_->isRawPositionSafe(fd_->odom_pos_) ||
            planner_manager_->isControlledEscapePosition(fd_->odom_pos_);
        const bool active_safe = raw_escape_safe && planner_manager_->checkTrajCollision(
                                                        active_traj_, active_collision_distance,
                                                        inflation_escape_active_);
        if (!active_safe) {
          requestActiveTrajectoryBrake("unsafe old trajectory before replanning");
          expl_manager_->reportTrajectoryCollision();
          ROS_WARN("[trajectory_brake] old trajectory rejected before planner call, "
                   "path_dist=%.2fm.",
                   active_collision_distance);
        }
      }
      // 膨胀层脱困必须从真实里程计起点生成。这里只切换规划起点，不发布safety_hold，
      // 控制器仍会继续接收当前控制状态，直到替代轨迹通过复核并发布。
      if (!fd_->static_state_ &&
          expl_manager_->shouldStartInflationHistoryEscape(fd_->odom_pos_)) {
        fd_->static_state_ = true;
        ROS_WARN("[inflation_history_escape] switch replanning start from 0.2s prediction "
                 "to live odometry; controller hold remains disabled.");
      }
      // local_data_ 会被本轮候选规划改写；动态重规划起点必须来自上一条已经发布的轨迹。
      if (!fd_->static_state_) {
        const double active_replan_time =
            active_traj_valid_
                ? (ros::Time::now() - active_traj_.start_time_).toSec() + fp_->replan_time_
                : std::numeric_limits<double>::infinity();
        if (!active_traj_valid_ || active_traj_braked_ ||
            active_replan_time >= active_traj_.duration_) {
          fd_->static_state_ = true;
        }
      }
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();

        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &active_traj_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }

      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        // 候选轨迹还要经过 PUB_TRAJ 的连续地图复核；复核通过前不能截断旧轨迹。
        next_plan_retry_time_ = ros::Time(0);
        pending_traj_safe_since_ = ros::Time(0);
        next_pending_traj_check_ = ros::Time(0);
        transitState(PUB_TRAJ, "FSM");
      } else if (res == NO_FRONTIER) {
        transitState(FINISH, "FSM");
        fd_->static_state_ = true;
        clearVisMarker();
      } else if (res == FAIL) {
        // Still in PLAN_TRAJ state, keep replanning
        // 2026-07-13: 新轨迹未生成时立即悬停，旧 bspline 不允许继续把机体带向障碍物。
        if (hold_on_plan_failure_) setSafetyHold(true, "planning failed");
        // 2026-07-14: FSM 定时器为 100 Hz，连续不可达时限制重复日志，保留悬停和后续重规划行为。
        ROS_WARN_THROTTLE(1.0, "plan fail");
        fd_->static_state_ = true;
        next_plan_retry_time_ =
            ros::Time::now() + ros::Duration(std::max(0.05, fp_->plan_failure_retry_interval_));
      }
      break;
    }

    case PUB_TRAJ: {
      const ros::Time now = ros::Time::now();
      // 2026-07-28: 即便规划阶段复核过，发布前仍等待刷新地图上的连续安全证据；
      // 轨迹起点若已与真实机体分离，禁止把远参考点直接交给控制器。
      if (next_pending_traj_check_.isZero() || now >= next_pending_traj_check_) {
        next_pending_traj_check_ =
            now + ros::Duration(std::max(0.02, fp_->trajectory_release_check_interval_));
        auto& trajectory = planner_manager_->local_data_.position_traj_;
        const Eigen::Vector3d trajectory_start = trajectory.evaluateDeBoorT(0.0);
        const double start_error = (trajectory_start - fd_->odom_pos_).norm();
        const bool controlled_escape_start =
            planner_manager_->isControlledEscapePosition(fd_->odom_pos_);
        const bool starts_in_inflation = planner_manager_->isPositionInflated(fd_->odom_pos_);
        const bool raw_start_safe = planner_manager_->isRawPositionSafe(fd_->odom_pos_) ||
                                    controlled_escape_start;
        const bool clears_inflation =
            !controlled_escape_start ||
            planner_manager_->trajectoryClearsInflation(
                0.80, 0.02, controlled_escape_start);
        const bool release_safe = raw_start_safe && clears_inflation &&
                                  start_error <= fp_->trajectory_release_max_start_error_ &&
                                  planner_manager_->isTrajectorySafe(
                                      0.03, controlled_escape_start);
        if (!release_safe) {
          ROS_ERROR("[trajectory_release] reject before publish: start_error=%.2fm raw_safe=%d "
                    "inflated=%d clears=%d.",
                    start_error, static_cast<int>(raw_start_safe),
                    static_cast<int>(starts_in_inflation), static_cast<int>(clears_inflation));
          pending_traj_safe_since_ = ros::Time(0);
          fd_->static_state_ = true;
          next_plan_retry_time_ =
              now + ros::Duration(std::max(0.05, fp_->plan_failure_retry_interval_));
          transitState(PLAN_TRAJ, "trajectory-release-validation");
          break;
        }
        if (pending_traj_safe_since_.isZero()) pending_traj_safe_since_ = now;
      }
      if (!pending_traj_safe_since_.isZero() &&
          (now - pending_traj_safe_since_).toSec() >=
              std::max(0.0, fp_->trajectory_release_confirm_time_)) {
        // 2026-07-28: 确认等待后把发布时刻重置为执行起点，既避免轨迹已经过去0.1~0.3s而跳点，
        // 也不给执行期安全定时器留下对负轨迹时间采样的窗口。
        const ros::Time release_start = now;
        planner_manager_->local_data_.start_time_ = release_start;
        fd_->newest_traj_.start_time = release_start;
        // bspline 本身会原子替换 traj_server 的旧轨迹；这里不再跨话题先发 replan，
        // 避免消息乱序后 replan 反而把刚收到的新轨迹截短。
        bspline_pub_.publish(fd_->newest_traj_);
        active_traj_ = planner_manager_->local_data_;
        active_traj_valid_ = true;
        active_traj_braked_ = false;
        active_turn_in_place_ = pending_turn_in_place_;
        if (active_turn_in_place_) turn_alignment_since_ = ros::Time(0);
        // 2026-07-27: 发布顺序固定为“轨迹先、检测使能后”，满足入口目标下发后才开始识别。
        if (!first_corridor_traj_published_) {
          first_corridor_traj_published_ = true;
          setDynamicDetectionEnable(mission_allows_dynamic_detection_,
                                    "first corridor trajectory published");
        }
        // 2026-07-13: 只有新轨迹已经发布后才解除安全悬停，避免规划成功与轨迹服务器接收之间的空窗。
        inflation_escape_active_ =
            planner_manager_->isControlledEscapePosition(fd_->odom_pos_);
        inflation_escape_clear_since_ = ros::Time(0);
        setSafetyHold(false, inflation_escape_active_
                                 ? "validated inflation escape trajectory published"
                                 : "validated new trajectory published");
        setEndpointHold(false, "validated new trajectory published");
        fd_->static_state_ = false;
        transitState(EXEC_TRAJ, "FSM");

        // 可视化必须在单线程 ROS 回调内读取当前轨迹和 frontier。分离线程会与紧随其后的
        // 安全重规划并发改写同一批容器，造成悬空访问并以 SIGSEGV 退出。
        visualize();
      }
      break;
    }

    case EXEC_TRAJ: {
      LocalTrajData* info = active_traj_valid_ ? &active_traj_ : &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();

      // Replan if traj is almost fully executed
      double time_to_end = info->duration_ - t_cur;
      if (active_turn_in_place_) {
        // 原地转向要执行到yaw终点，不能按普通平移轨迹在“剩余1秒”时提前打断。
        if (time_to_end <= 0.05) {
          const double yaw_error =
              expl_manager_->turnInPlaceFinalYawError(fd_->odom_yaw_);
          const double tolerance =
              expl_manager_->turnInPlaceCompletionTolerance();
          if (std::fabs(yaw_error) > tolerance) {
            turn_alignment_since_ = ros::Time(0);
            active_turn_in_place_ = false;
            fd_->static_state_ = true;
            transitState(PLAN_TRAJ, "turn-in-place-next-segment");
            ROS_WARN("[turn_in_place] segment ended with actual final-yaw error "
                     "%.1fdeg; continue next segment.",
                     yaw_error * 180.0 / M_PI);
            return;
          }
          const ros::Time now = ros::Time::now();
          if (turn_alignment_since_.isZero()) turn_alignment_since_ = now;
          if ((now - turn_alignment_since_).toSec() <
              expl_manager_->turnInPlaceCompletionConfirmTime())
            return;
          expl_manager_->completeTurnInPlace();
          active_turn_in_place_ = false;
          turn_alignment_since_ = ros::Time(0);
          fd_->static_state_ = true;
          transitState(PLAN_TRAJ, "turn-in-place-complete");
          ROS_ERROR("[turn_in_place] actual yaw stayed within %.1fdeg; "
                    "replan translation from live odometry.",
                    tolerance * 180.0 / M_PI);
        }
        return;
      }
      if (exploration_policy::shouldReplanNearTrajectoryEnd(
              time_to_end, fp_->replan_thresh1_)) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: traj fully executed=================================");
        return;
      }
      // Replan if next frontier to be visited is covered
      if (exploration_policy::shouldReplanForCoveredFrontier(
              expl_manager_->frontier_finder_->isFrontierCovered(),
              narrow_corridor_stage_, t_cur, fp_->replan_thresh2_)) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: cluster covered=====================================");
        return;
      }
      // Replan after some time
      if (!classic_ && exploration_policy::shouldPeriodicReplan(
                           t_cur, fp_->replan_thresh3_, periodic_replan_enabled_)) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: periodic call=======================================");
      }
      break;
    }
  }
}

int FastExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  int res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                             fd_->start_yaw_);
  pending_turn_in_place_ =
      res == SUCCEED && expl_manager_->currentPlanIsTurnInPlace();
  classic_ = false;

  // int res = expl_manager_->classicFrontier(fd_->start_pt_, fd_->start_yaw_[0]);
  // classic_ = true;

  // int res = expl_manager_->rapidFrontier(fd_->start_pt_, fd_->start_vel_, fd_->start_yaw_[0],
  // classic_);

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
  }
  return res;
}

void FastExplorationFSM::visualize() {
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  // Draw updated box
  // Vector3d bmin, bmax;
  // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
  // 4);

  // Draw frontier
  static int last_ftr_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                              visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
                              "frontier", i, 4);
    // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
    //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  last_ftr_num = ed_ptr->frontiers_.size();
  // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
  //   visualization_->drawCubes(ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier",
  //                             i, 4);
  // for (int i = ed_ptr->dead_frontiers_.size(); i < 5; ++i)
  //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);

  // Draw global top viewpoints info
  // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
  // "point-average", 0, 6);

  // Draw local refined viewpoints info
  // visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
  //                           Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0,
  // 1),
  //                           "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1,
  // 0, 1),
  //                           "refine_pair", 0, 6);
  // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
  //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
  //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
  //                               ed_ptr->frontiers_.size()),
  //                               "n_points", i, 6);
  // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
  //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

  // Draw trajectory
  // visualization_->drawSpheres({ ed_ptr->next_goal_ }, 0.3, Vector4d(0, 1, 1, 1), "next_goal", 0, 6);
  visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
                              Vector4d(1, 1, 0, 1));
  // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0, 0);
  // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1, 6);
}

void FastExplorationFSM::clearVisMarker() {
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  static int delay = 0;
  if (++delay < 5) return;

  // 2026-07-16: FINISH 后任务与降落链路已经接管，禁止后台继续改动 frontier 代价矩阵；
  // 旧逻辑会在落地阶段重复删除 frontier，并最终触发 invalid pointer。
  if (state_ == WAIT_TRIGGER) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();
    ft->updateFrontierCostMatrix();

    ft->getFrontiers(ed->frontiers_);
    ft->getFrontierBoxes(ed->frontier_boxes_);

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
                                visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
                                "frontier", i, 4);
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      // Vector4d(0.5, 0, 1, 0.3),
      //                         "frontier_boxes", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
  }

  // if (!fd_->static_state_)
  // {
  //   static double astar_time = 0.0;
  //   static int astar_num = 0;
  //   auto t1 = ros::Time::now();

  //   planner_manager_->path_finder_->reset();
  //   planner_manager_->path_finder_->setResolution(0.4);
  //   if (planner_manager_->path_finder_->search(fd_->odom_pos_, Vector3d(-5, 0, 1)))
  //   {
  //     auto path = planner_manager_->path_finder_->getPath();
  //     visualization_->drawLines(path, 0.05, Vector4d(1, 0, 0, 1), "astar", 0, 6);
  //     auto visit = planner_manager_->path_finder_->getVisited();
  //     visualization_->drawCubes(visit, 0.3, Vector4d(0, 0, 1, 0.4), "astar-visit", 0, 6);
  //   }
  //   astar_num += 1;
  //   astar_time = (ros::Time::now() - t1).toSec();
  //   ROS_WARN("Average astar time: %lf", astar_time);
  // }
}

void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses.empty()) {
    ROS_ERROR("[fuel_diff] ignore empty entry trigger path.");
    return;
  }
  if (msg->poses[0].pose.position.z < -0.1) return;
  if (state_ != WAIT_TRIGGER) return;
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  if (use_diff_for_fuel_exploration_) {
    if (exploration_policy::shouldActivateExternalExploration(
            true, true, external_exploration_active_)) {
      activateExternalExploration(msg->poses[0]);
    }
    return;
  }
  transitState(PLAN_TRAJ, "triggerCallback");
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  const bool replacement_pending =
      state_ == EXPL_STATE::PLAN_TRAJ || state_ == EXPL_STATE::PUB_TRAJ;
  const bool execution_stage = replacement_pending || state_ == EXPL_STATE::EXEC_TRAJ;
  if (!execution_stage || !exploration_policy::shouldMonitorPublishedTrajectory(
                              active_traj_valid_, replacement_pending,
                              active_traj_braked_))
    return;

  // 正常平移执行期间同步累计地图转弯证据。配置的确认次数满足后立刻短制动并切换到
  // 原地yaw轨迹，不再等当前平移轨迹接近终点才发现拐角。
  if (state_ == EXPL_STATE::EXEC_TRAJ && !active_turn_in_place_ &&
      !inflation_escape_active_) {
    Eigen::Vector3d turn_direction;
    if (expl_manager_->detectMappedTurnDuringExecution(
            fd_->odom_yaw_, turn_direction)) {
      requestActiveTrajectoryBrake("mapped turn confirmed; stop before yaw alignment");
      fd_->static_state_ = true;
      transitState(PLAN_TRAJ, "safetyCallback-turn-in-place");
      ROS_ERROR("[turn_in_place] mapped turn confirmed during execution; brake translation "
                "and rotate toward %.1fdeg.",
                std::atan2(turn_direction.y(), turn_direction.x()) * 180.0 / M_PI);
      return;
    }
  }

  // 候选规划会改写 planner_manager_->local_data_，这里只检查 traj_server 真正执行的快照。
  auto& trajectory = active_traj_.position_traj_;
  const double traj_time = std::max(
      0.0, std::min((ros::Time::now() - active_traj_.start_time_).toSec(),
                    active_traj_.duration_));
  const Eigen::Vector3d desired = trajectory.evaluateDeBoorT(traj_time);
  const Eigen::Vector3d tracking_error = fd_->odom_pos_ - desired;
  const double error_xy = tracking_error.head<2>().norm();
  const double error_z = std::fabs(tracking_error.z());
  const bool in_handover_grace = traj_time < fp_->tracking_error_grace_time_;
  const bool tracking_exceeded = error_xy > fp_->max_tracking_error_xy_ ||
                                 error_z > fp_->max_tracking_error_z_;
  if (in_handover_grace || !tracking_exceeded) {
    tracking_error_since_ = ros::Time(0);
  } else {
    if (tracking_error_since_.isZero()) tracking_error_since_ = ros::Time::now();
    const double exceeded_time = (ros::Time::now() - tracking_error_since_).toSec();
    const bool hard_error = error_xy > fp_->hard_tracking_error_xy_ ||
                            error_z > fp_->hard_tracking_error_z_;
    if (hard_error || exceeded_time >= fp_->tracking_error_confirm_time_) {
      if (hard_error) requestActiveTrajectoryBrake("hard tracking error");
      fd_->static_state_ = true;
      ROS_ERROR("[tracking_safety] %s replan: error_xy=%.2fm error_z=%.2fm duration=%.2fs.",
                hard_error ? "short-brake" : "odometry", error_xy, error_z, exceeded_time);
      tracking_error_since_ = ros::Time(0);
      if (state_ != EXPL_STATE::PLAN_TRAJ)
        transitState(PLAN_TRAJ, "safetyCallback-tracking");
    } else {
      ROS_WARN_THROTTLE(
          0.5, "[tracking_safety] confirming error_xy=%.2fm error_z=%.2fm for %.2f/%.2fs.",
          error_xy, error_z, exceeded_time, fp_->tracking_error_confirm_time_);
    }
  }

  // 已通过发布门控的膨胀层逃逸，在真正离开膨胀层前使用单调净空判据。
  if (inflation_escape_active_) {
    const bool still_inflated = planner_manager_->isPositionInflated(fd_->odom_pos_);
    const bool raw_safe = planner_manager_->isRawPositionSafe(fd_->odom_pos_);
    const bool still_needs_escape = still_inflated || !raw_safe;
    if (still_needs_escape &&
        !planner_manager_->isControlledEscapePosition(fd_->odom_pos_)) {
      requestActiveTrajectoryBrake("footprint contact worsened during inflation escape");
      fd_->static_state_ = true;
      inflation_escape_active_ = false;
      if (state_ != EXPL_STATE::PLAN_TRAJ)
        transitState(PLAN_TRAJ, "safetyCallback-raw-escape");
      return;
    }
    if (!still_needs_escape) {
      if (inflation_escape_clear_since_.isZero())
        inflation_escape_clear_since_ = ros::Time::now();
      if ((ros::Time::now() - inflation_escape_clear_since_).toSec() >= 0.20) {
        inflation_escape_active_ = false;
        ROS_WARN("[footprint_safety] inflation escape completed after 0.20s continuous clearance.");
      }
    } else {
      inflation_escape_clear_since_ = ros::Time(0);
    }
  }

  double dist;
  const bool safe =
      planner_manager_->checkTrajCollision(active_traj_, dist, inflation_escape_active_);
  if (exploration_policy::shouldBrakePublishedTrajectory(active_traj_valid_, !safe,
                                                          active_traj_braked_)) {
    requestActiveTrajectoryBrake("future footprint collision");
    expl_manager_->reportTrajectoryCollision();
    ROS_WARN("Replan: collision detected, old trajectory short-braked at path_dist=%.2fm", dist);
    if (state_ != EXPL_STATE::PLAN_TRAJ)
      transitState(PLAN_TRAJ, "safetyCallback");
  }
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  // 2026-07-22: 按真实里程计连续建立任务航迹，规划失败/低频重规划时旧路过滤仍立即有效。
  if (expl_manager_) expl_manager_->updateMissionOdometry(fd_->odom_pos_, fd_->odom_yaw_);

  fd_->have_odom_ = true;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
       << endl;
}
}  // namespace fast_planner
