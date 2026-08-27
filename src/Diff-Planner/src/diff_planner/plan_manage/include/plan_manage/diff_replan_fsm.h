#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <cstdint>
#include <deque>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <sstream>  // 2026-07-28: 规划成功状态附带Diff修正后的实际目标坐标。
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <ldop/DynamicObjectArray.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>
#include <plan_manage/swing_obstacle_guard.h>

using std::vector;

namespace diff_planner
{

  class DiffReplanFSM
  {
  public:
    DiffReplanFSM() {}
    ~DiffReplanFSM() {}

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START,
      OCCUPIED_RECOVERY
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3,
      SEARCH_TARGET = 4
    };
    /* Anomaly Detection Parameters */
    Eigen::Vector3d last_local_target_pos_;
    double last_target_change_time_;
    // 2026-07-07: 为首飞阶段和 RViz 2D 触发后的前几秒提供 stuck detect 豁免窗口，避免首段轨迹刚下发就被误判卡死。
    double stuck_detect_grace_time_;
    double stuck_detect_ignore_until_;
    int replan_fail_count_;
    static constexpr double TARGET_STUCK_THRESH = 0.3;  // Threshold for target movement below which it's considered "stuck"
    double TARGET_STUCK_TIME;                           // Default time threshold (seconds) for being considered stuck before reinitialization
    static constexpr int MAX_REPLAN_FAIL_COUNT = 10;    // Threshold for maximum optimization failure count
    /* planning utils */
    DiffPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    bool enable_periodic_replan_;
    double waypoints_[50][3];
    int waypoint_num_, wpt_id_;
    double planning_horizen_;
    // Maximum allowed 3-D separation between the previous trajectory's
    // predicted state and the measured odometry before replanning from odom.
    double max_tracking_error_;
    double emergency_time_;
    bool enable_occupied_recovery_;
    double escape_max_distance_;
    double escape_history_time_;
    double escape_speed_;
    double escape_reach_tolerance_;
    double escape_stop_speed_;
    double escape_min_clearance_;
    double escape_clearance_search_radius_;
    double escape_max_occupied_prefix_;
    double escape_recovery_height_;
    int escape_free_cycles_;
    int escape_max_attempts_;
    bool history_only_occupied_recovery_;
    bool wait_new_target_after_occupied_recovery_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool enable_ground_height_measurement_;
    bool flag_escape_emergency_;
    bool need_hover_stop_;
    bool depth_timeout_emergency_{false};
    bool mondify_final_goal_;
    bool preserve_modified_goal_height_;
    bool enable_stuck_detect_; // Whether to enable stuck detection
    // 2026-07-28: FUEL->Diff接力模式可关闭原生Diff编队的“等待所有前序无人机轨迹”门槛。
    bool require_pre_agent_trajectory_;
    // 2026-07-28: 异构接力可禁用已确认会阻塞FSM的随机多项式初始化，失败改由上层安全子目标恢复。
    bool enable_random_global_init_;
    bool enable_random_local_init_;
    std::string search_subgoal_topic_;
    std::string manual_goal_topic_; // 2026-07-28: UAV1接力规划使用独立目标话题，避免与前机全局/goal串线。
    bool enable_swing_obstacle_guard_;
    std::string swing_obstacle_topic_;
    double swing_prediction_horizon_;
    double swing_prediction_dt_;
    double swing_release_clear_time_;
    double swing_release_speed_;

    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    // AutoTrans 恢复成功后置位；在基于最新里程计发布新轨迹前禁止继续使用旧局部轨迹。
    bool controller_restart_pending_;
    bool swing_wait_active_;
    double swing_clear_since_;
    uint32_t swing_wait_obstacle_id_;
    // 当前外部目标时间戳随每条规划状态回传，供接力管理器过滤锁存/延迟回执。
    uint64_t active_external_goal_stamp_ns_{0ULL};
    // 地图更新使目标落入障碍后，成功重规划时需把替换后的自由目标回传上层。
    bool external_goal_modified_{false};
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d final_goal_;                             // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
    std::vector<Eigen::Vector3d> wps_;

    struct FreeOdomSample
    {
      double stamp;
      double x;
      double y;
      double z;
    };
    std::deque<FreeOdomSample> free_odom_history_;
    Eigen::Vector3d occupied_recovery_target_;
    double occupied_recovery_deadline_;
    double last_free_history_record_time_;
    double last_escape_path_check_time_;
    double last_escape_target_search_time_;
    int occupied_recovery_free_count_;
    int occupied_recovery_attempt_count_;
    bool occupied_recovery_active_;
    bool occupied_recovery_episode_;
    bool occupied_recovery_from_history_;
    bool occupied_recovery_vertical_;
    bool occupied_recovery_failure_reported_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, subgoal_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_, planning_restart_sub_, swing_obstacle_sub_;
    ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, ground_height_pub_;
    ros::Publisher planning_status_pub_;  // 2026-07-28: 向接力管理器反馈Diff规划成功/连续失败。
    SwingObstacleGuard swing_obstacle_guard_;

    /* state machine functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void printFSMExecState();
    std::pair<int, DiffReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /* safety */
    void checkCollisionCallback(const ros::TimerEvent &e);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);
    void dynamicObjectsCallback(const ldop::DynamicObjectArrayConstPtr &msg);
    std::vector<SwingTrajectorySample> sampleCurrentTrajectory(double now) const;
    std::vector<SwingTrajectorySample> sampleReleaseTrajectory() const;
    bool swingTrajectoryBlocked(const std::vector<SwingTrajectorySample> &samples,
                                double now,
                                SwingCollisionResult *result) const;
    void startSwingWait(const SwingCollisionResult &collision);
    bool callOccupiedRecovery(const Eigen::Vector3d &target);
    void updateFreeOdomHistory(double now);
    bool startOccupiedRecovery(double now);
    bool selectOccupiedRecoveryTarget(Eigen::Vector3d &target,
                                      bool &from_history,
                                      double &clearance);
    bool selectHistoryRecoveryTarget(Eigen::Vector3d &target,
                                     double &clearance);
    bool selectVerticalRecoveryTarget(Eigen::Vector3d &target,
                                      double &clearance);
    bool selectLateralRecoveryTarget(Eigen::Vector3d &target,
                                     double &clearance);
    bool validateRecoverySegment(const Eigen::Vector3d &start,
                                 const Eigen::Vector3d &end,
                                 bool allow_initial_occupied,
                                 double *occupied_prefix = nullptr);
    double estimateInflatedClearance(const Eigen::Vector3d &pos);
    void abortOccupiedRecovery(const char *reason);
    void planningRestartCallback(const std_msgs::Empty &msg);

    /* local planning */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /* global trajectory */
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void readGivenWpsAndPlan();
    bool planNextWaypoint(const Eigen::Vector3d next_wp, bool flag_2replan);
    bool mondifyInCollisionFinalGoal();
    void finishProcess();

    /* input-output */
    void mandatoryStopCallback(const std_msgs::Empty &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void searchSubgoalCallback(const geometry_msgs::PoseStampedPtr &msg);
    void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);
    void publishPlanningStatus(const std::string &status);  // 2026-07-28: 结构化规划状态输出。

    /* ground height measurement */
    bool measureGroundHeight(double &height);
    Eigen::Vector3d projectPointToLineSegment(const Eigen::Vector3d& a,
                                              const Eigen::Vector3d& b,
                                              const Eigen::Vector3d& p);
  };

} // namespace diff_planner

#endif
