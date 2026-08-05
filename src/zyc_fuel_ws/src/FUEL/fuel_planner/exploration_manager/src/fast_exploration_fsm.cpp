
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

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
  // 2026-07-28: 连续复核覆盖至少两次20Hz地图/安全周期；起点误差过大则从真实里程计重规划。
  nh.param("fsm/trajectory_release_confirm_time", fp_->trajectory_release_confirm_time_, 0.12);
  nh.param("fsm/trajectory_release_check_interval", fp_->trajectory_release_check_interval_, 0.04);
  nh.param("fsm/trajectory_release_max_start_error",
           fp_->trajectory_release_max_start_error_, 0.25);

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

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

  trigger_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &FastExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);
  // 2026-07-27: 任务状态决定门内/门外，LDOT 不自行解析门平面或目标坐标。
  std::string mission_status_topic;
  std::string dynamic_detection_enable_topic;
  nh.param("fsm/mission_status_topic", mission_status_topic,
           std::string("/mission/task_status"));
  nh.param("fsm/dynamic_detection_enable_topic", dynamic_detection_enable_topic,
           std::string("/UAV0/corridor_search/dynamic_detection_enable"));
  mission_status_sub_ = nh.subscribe(mission_status_topic, 2,
                                     &FastExplorationFSM::missionStatusCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
  // 2026-07-13: latch 保证后启动的控制器也能收到当前安全门控状态。
  safety_hold_pub_ = nh.advertise<std_msgs::Bool>("/planning/safety_hold", 2, true);
  dynamic_detection_enable_pub_ =
      nh.advertise<std_msgs::Bool>(dynamic_detection_enable_topic, 2, true);
  setSafetyHold(false, "initialization");
  // 2026-07-27: 锁存 false，后启动的 LDOT 在首条通道内轨迹前也必须保持冻结。
  setDynamicDetectionEnable(false, "initialization", true);
}

void FastExplorationFSM::setSafetyHold(bool active, const string& reason) {
  if (safety_hold_active_ == active && reason != "initialization") return;
  safety_hold_active_ = active;
  std_msgs::Bool msg;
  msg.data = active;
  safety_hold_pub_.publish(msg);
  ROS_WARN("[safety_hold] %s reason=%s.", active ? "ACTIVE" : "RELEASED", reason.c_str());
}

// 2026-07-27: 统一发布规划器判定后的门内检测授权，重复状态不打断 LDOT 跟踪会话。
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
  mission_allows_dynamic_detection_ =
      state.find("SEARCH_CORRIDOR") == 0 ||
      state.find("EXIT_APPROACH_INSIDE") == 0 ||
      state.find("CROSS_EXIT") == 0;
  setDynamicDetectionEnable(first_corridor_traj_published_ &&
                                mission_allows_dynamic_detection_,
                            mission_allows_dynamic_detection_
                                ? "inside corridor with published trajectory"
                                : "outside corridor mission stage");
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

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
      // Do nothing but wait for trigger
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
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();

        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }

      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty());
      int res = callExplorationPlanner();
      if (res == SUCCEED) {
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
        setSafetyHold(true, "planning failed");
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
        const bool raw_start_safe = planner_manager_->isRawPositionSafe(fd_->odom_pos_);
        const bool starts_in_inflation = planner_manager_->isPositionInflated(fd_->odom_pos_);
        const bool clears_inflation =
            !starts_in_inflation || planner_manager_->trajectoryClearsInflation();
        const bool release_safe = raw_start_safe && clears_inflation &&
                                  start_error <= fp_->trajectory_release_max_start_error_ &&
                                  planner_manager_->isTrajectorySafe();
        if (!release_safe) {
          ROS_ERROR("[trajectory_release] reject before publish: start_error=%.2fm raw_safe=%d "
                    "inflated=%d clears=%d.",
                    start_error, static_cast<int>(raw_start_safe),
                    static_cast<int>(starts_in_inflation), static_cast<int>(clears_inflation));
          pending_traj_safe_since_ = ros::Time(0);
          setSafetyHold(true, "trajectory release validation failed");
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
        bspline_pub_.publish(fd_->newest_traj_);
        // 2026-07-27: 发布顺序固定为“轨迹先、检测使能后”，满足入口目标下发后才开始识别。
        if (!first_corridor_traj_published_) {
          first_corridor_traj_published_ = true;
          setDynamicDetectionEnable(mission_allows_dynamic_detection_,
                                    "first corridor trajectory published");
        }
        // 2026-07-13: 只有新轨迹已经发布后才解除安全悬停，避免规划成功与轨迹服务器接收之间的空窗。
        inflation_escape_active_ = planner_manager_->isPositionInflated(fd_->odom_pos_);
        inflation_escape_clear_since_ = ros::Time(0);
        setSafetyHold(false, inflation_escape_active_
                                 ? "validated inflation escape trajectory published"
                                 : "validated new trajectory published");
        fd_->static_state_ = false;
        transitState(EXEC_TRAJ, "FSM");

        thread vis_thread(&FastExplorationFSM::visualize, this);
        vis_thread.detach();
      }
      break;
    }

    case EXEC_TRAJ: {
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();

      // Replan if traj is almost fully executed
      double time_to_end = info->duration_ - t_cur;
      if (time_to_end < fp_->replan_thresh1_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: traj fully executed=================================");
        return;
      }
      // Replan if next frontier to be visited is covered
      if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: cluster covered=====================================");
        return;
      }
      // Replan after some time
      if (t_cur > fp_->replan_thresh3_ && !classic_) {
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
  if (msg->poses[0].pose.position.z < -0.1) return;
  if (state_ != WAIT_TRIGGER) return;
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  transitState(PLAN_TRAJ, "triggerCallback");
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // 2026-07-13: 碰撞检查不能只看理想轨迹，还要检查真实 /Odometry 与当前样条的偏差。
    // 2026-07-13: 该仓库的 evaluateDeBoorT 接口不是 const，使用可变引用仅用于读取当前期望位置。
    auto& trajectory = planner_manager_->local_data_.position_traj_;
    const double traj_time = std::max(
        0.0, std::min((ros::Time::now() - planner_manager_->local_data_.start_time_).toSec(),
                      planner_manager_->local_data_.duration_));
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
        // 2026-07-13: 普通持续偏差从真实里程计柔和重规划，不把目标误判为碰撞；严重偏差才清空轨迹急停。
        if (hard_error) setSafetyHold(true, "hard tracking error");
        fd_->static_state_ = true;
        ROS_ERROR("[tracking_safety] %s replan: error_xy=%.2fm error_z=%.2fm duration=%.2fs.",
                  hard_error ? "hard-stop" : "odometry", error_xy, error_z, exceeded_time);
        tracking_error_since_ = ros::Time(0);
        transitState(PLAN_TRAJ, "safetyCallback-tracking");
        return;
      }
      ROS_WARN_THROTTLE(0.5,
                        "[tracking_safety] confirming error_xy=%.2fm error_z=%.2fm for %.2f/%.2fs.",
                        error_xy, error_z, exceeded_time, fp_->tracking_error_confirm_time_);
    }
    // Check safety and trigger replan if necessary
    double dist;
    // 2026-07-28: 已通过发布门控的膨胀层逃逸，在真正离开膨胀层前使用单调净空判据，
    // 防止普通检查在path_dist=0处立即否决同一条轨迹。
    if (inflation_escape_active_) {
      if (!planner_manager_->isRawPositionSafe(fd_->odom_pos_)) {
        setSafetyHold(true, "raw footprint collision during inflation escape");
        fd_->static_state_ = true;
        inflation_escape_active_ = false;
        transitState(PLAN_TRAJ, "safetyCallback-raw-escape");
        return;
      }
      if (!planner_manager_->isPositionInflated(fd_->odom_pos_)) {
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
    bool safe = planner_manager_->checkTrajCollision(dist, inflation_escape_active_);
    if (!safe) {
      // 2026-07-13: 碰撞预测先刹停再重规划，修复仅切 FSM 状态但控制器仍执行旧轨迹的问题。
      setSafetyHold(true, "future footprint collision");
      // 2026-07-13: 同步通知任务选点层冷却碰撞目标，防止安全重规划反复选择同一柱边点。
      expl_manager_->reportTrajectoryCollision();
      ROS_WARN("Replan: collision detected==================================");
      transitState(PLAN_TRAJ, "safetyCallback");
    }
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
