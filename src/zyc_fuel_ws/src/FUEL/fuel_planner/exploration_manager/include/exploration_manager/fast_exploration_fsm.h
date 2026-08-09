#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>  // 2026-07-27: 将任务门内/门外阶段转换为 LDOT 明确使能。
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
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
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_, heartbeat_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, mission_status_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, safety_hold_pub_,
      dynamic_detection_enable_pub_, planning_heartbeat_pub_;
  bool safety_hold_active_{false};
  // 2026-07-13: 记录连续跟踪误差起点，区分短时控制滞后与真实失控。
  ros::Time tracking_error_since_;
  // 2026-07-22: 规划失败后低频重试，禁止PLAN_TRAJ在100Hz下重复生成同一批轨迹和RViz线段。
  ros::Time next_plan_retry_time_;
  // 2026-07-28: 发布前在多次地图刷新间连续复核，阻断“发布即解锁、20ms后自判碰撞”的振荡。
  ros::Time pending_traj_safe_since_;
  ros::Time next_pending_traj_check_;
  bool inflation_escape_active_{false};
  ros::Time inflation_escape_clear_since_;
  // 2026-07-27: 必须先发布首条通道内轨迹，且任务仍在门内/穿出口阶段，才允许 LDOT 工作。
  bool first_corridor_traj_published_{false};
  bool mission_allows_dynamic_detection_{false};
  bool dynamic_detection_enabled_{false};

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);
  // 2026-07-13: 规划碰撞或失败时显式通知控制器刹停，禁止继续消费上一条轨迹。
  void setSafetyHold(bool active, const string& reason);
  void setDynamicDetectionEnable(bool active, const string& reason, bool force = false);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void heartbeatCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void missionStatusCallback(const std_msgs::StringConstPtr& msg);
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
