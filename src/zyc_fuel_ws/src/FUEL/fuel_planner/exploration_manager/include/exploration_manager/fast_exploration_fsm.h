#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>  // 将任务门内/门外阶段转换为动态检测状态。
#include <quadrotor_msgs/ExplorationGoal.h>
#include <quadrotor_msgs/ExplorationCancel.h>
#include <quadrotor_msgs/ExplorationGoalStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <plan_manage/plan_container.hpp>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH };

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, mission_status_sub_;
  ros::Subscriber external_status_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, emergency_brake_pub_, safety_hold_pub_,
      endpoint_hold_pub_, dynamic_detection_enable_pub_;
  ros::Publisher external_goal_pub_, external_cancel_pub_, external_trigger_pub_;
  bool use_diff_for_fuel_exploration_{false};
  bool external_exploration_active_{false};
  bool external_goal_pending_{false};
  std::string external_goal_topic_{"/UAV0/fuel_diff/goal"};
  std::string external_status_topic_{"/UAV0/fuel_diff/status"};
  std::string external_cancel_topic_{"/UAV0/fuel_diff/cancel"};
  std::string external_trigger_topic_{"/UAV0/fuel_diff/trigger"};
  std::uint64_t external_session_id_{0};
  std::uint64_t external_goal_id_{0};
  ros::Time external_goal_sent_at_;
  ros::Time external_next_select_at_;
  geometry_msgs::Pose external_last_pose_;
  quadrotor_msgs::ExplorationGoal external_last_goal_;
  bool safety_hold_active_{false};
  bool safety_hold_enabled_{true};
  bool hold_on_plan_failure_{true};
  bool endpoint_hold_active_{false};
  bool endpoint_hold_enabled_{true};
  double endpoint_hold_lead_time_{0.05};
  bool periodic_replan_enabled_{true};
  // 2026-07-13: 记录连续跟踪误差起点，区分短时控制滞后与真实失控。
  ros::Time tracking_error_since_;
  // 2026-07-22: 规划失败后低频重试，禁止PLAN_TRAJ在100Hz下重复生成同一批轨迹和RViz线段。
  ros::Time next_plan_retry_time_;
  // 2026-07-28: 发布前在多次地图刷新间连续复核，阻断“发布即解锁、20ms后自判碰撞”的振荡。
  ros::Time pending_traj_safe_since_;
  ros::Time next_pending_traj_check_;
  bool inflation_escape_active_{false};
  ros::Time inflation_escape_clear_since_;
  // 规划器生成下一条候选轨迹时会改写 local_data_。单独保存 traj_server 当前正在执行的
  // 已发布轨迹，保证 PLAN_TRAJ/PUB_TRAJ 阶段仍能检查旧轨迹安全性。
  LocalTrajData active_traj_;
  bool active_traj_valid_{false};
  bool active_traj_braked_{false};
  bool pending_turn_in_place_{false};
  bool active_turn_in_place_{false};
  ros::Time turn_alignment_since_;
  // 首条通道内轨迹发布后记录动态检测阶段；LDOP当前持续运行，该状态供监控保留。
  bool first_corridor_traj_published_{false};
  bool mission_allows_dynamic_detection_{false};
  bool narrow_corridor_stage_{false};
  bool dynamic_detection_enabled_{false};

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);
  // 2026-07-13: 规划碰撞或失败时显式通知控制器刹停，禁止继续消费上一条轨迹。
  void setSafetyHold(bool active, const string& reason);
  void setEndpointHold(bool active, const string& reason);
  void requestActiveTrajectoryBrake(const string& reason);
  void setDynamicDetectionEnable(bool active, const string& reason, bool force = false);
  void activateExternalExploration(const geometry_msgs::PoseStamped& entry_trigger);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void missionStatusCallback(const std_msgs::StringConstPtr& msg);
  void externalStatusCallback(const quadrotor_msgs::ExplorationGoalStatusConstPtr& msg);
  bool publishExternalViewpoint();
  void visualize();
  void clearVisMarker();

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
