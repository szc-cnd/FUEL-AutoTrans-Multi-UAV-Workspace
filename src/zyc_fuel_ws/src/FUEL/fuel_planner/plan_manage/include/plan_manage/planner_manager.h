#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline_opt/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

#include <path_searching/astar2.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/topo_prm.h>

#include <plan_env/edt_environment.h>

#include <active_perception/frontier_finder.h>
#include <active_perception/heading_planner.h>

#include <plan_manage/plan_container.hpp>

#include <ros/ros.h>

namespace fast_planner {
// Fast Planner Manager
// Key algorithms of mapping and planning are called

class FastPlannerManager {
  // SECTION stable
public:
  FastPlannerManager();
  ~FastPlannerManager();

  /* main planning interface */
  bool kinodynamicReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel,
                         const Eigen::Vector3d& start_acc, const Eigen::Vector3d& end_pt,
                         const Eigen::Vector3d& end_vel, const double& time_lb = -1);
  // 2026-07-14: 返回轨迹生成是否成功，拒绝少于三点或含零长度分段的非法 waypoint 输入。
  bool planExploreTraj(const vector<Eigen::Vector3d>& tour, const Eigen::Vector3d& cur_vel,
                       const Eigen::Vector3d& cur_acc, const double& time_lb = -1);
  // 地图确认真实转弯后，位置保持不动，只执行yaw对准。
  bool planStationaryTraj(const Eigen::Vector3d& position, double duration);
  bool planGlobalTraj(const Eigen::Vector3d& start_pos);
  bool topoReplan(bool collide);

  void planYaw(const Eigen::Vector3d& start_yaw);
  // 2026-07-24: continuous_scan为前置相机提供随位置轨迹连续单方向环扫，不改变位置规划链路。
  void planYawExplore(const Eigen::Vector3d& start_yaw, const double& end_yaw, bool lookfwd,
                      const double& relax_time, bool continuous_scan = false,
                      double scan_yaw_rate = 0.0);

  void initPlanModules(ros::NodeHandle& nh);
  void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);

  // 2026-07-28: 膨胀层脱困轨迹在执行期沿用“原始足迹安全且净空单调增加”的判据，
  // 避免发布前允许逃逸、发布后20ms又被普通膨胀层检查立即否决。
  bool checkTrajCollision(double& distance, bool allow_inflation_escape = false);
  bool checkTrajCollision(LocalTrajData& trajectory, double& distance,
                          bool allow_inflation_escape = false);
  // 2026-07-13: 使用独立机体足迹检查轨迹/路径，避免 0.15m 建图膨胀小于 Iris 实际旋翼半径。
  bool isPositionSafe(const Eigen::Vector3d& position) const;
  // 2026-07-28: FSM发布门控需要区分原始占据碰撞与仅接触膨胀层，决定能否执行受控逃逸。
  bool isRawPositionSafe(const Eigen::Vector3d& position) const;
  // 局部脱困专用：允许少量受支撑占据采样，但只能沿采样数和ESDF净空持续改善的方向退出。
  int rawFootprintCollisionCount(const Eigen::Vector3d& position) const;
  bool isControlledEscapePosition(const Eigen::Vector3d& position) const;
  bool isPositionInflated(const Eigen::Vector3d& position) const;
  bool isPathSafe(const vector<Eigen::Vector3d>& path,
                  bool allow_contact_escape = false) const;
  // 2026-07-14: B-spline 发布前复核完整 Iris 足迹，避免优化曲线偏离安全 A* 折线后才被执行期急停。
  bool isTrajectorySafe(double sample_dt = 0.03,
                        bool allow_contact_escape = false);
  // 2026-07-28: 起点已在膨胀层时，新轨迹必须在限定路程内真正回到非膨胀区，
  // 不能仅靠净空不下降生成一条始终贴墙的“伪恢复”轨迹。
  bool trajectoryClearsInflation(double max_path_distance = 0.80,
                                 double sample_dt = 0.02,
                                 bool allow_contact_escape = false);
  void calcNextYaw(const double& last_yaw, double& yaw);

  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;
  unique_ptr<Astar> path_finder_;
  unique_ptr<TopologyPRM> topo_prm_;

private:
  /* main planning algorithms & modules */
  shared_ptr<SDFMap> sdf_map_;

  // 2026-07-23: A*保留膨胀图引导，最终路径安全以原始占据上的真实圆盘足迹为准，
  // 避免把膨胀层与机体半径重复作为两个硬约束。
  double footprint_check_radius_{0.18};
  int footprint_check_samples_{12};
  int footprint_min_occupied_support_{2};
  double footprint_support_radius_{0.10};
  bool supported_occupancy_hard_reject_enabled_{true};
  int escape_max_initial_occupied_samples_{6};
  double turn_slowdown_angle_deg_{30.0};
  double turn_time_scale_{1.5};
  // 2026-07-23: 真实圆盘足迹检查供路径/轨迹最终复核；膨胀图仅用于A*引导，
  // 避免当前位置被膨胀层擦到后所有前向脱困路径因共享同一起点而全部失败。
  bool isRawFootprintSafe(const Eigen::Vector3d& position) const;
  bool isSupportedOccupied(const Eigen::Vector3d& position) const;

  unique_ptr<KinodynamicAstar> kino_path_finder_;
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;

  void updateTrajInfo();

  // topology guided optimization

  void findCollisionRange(vector<Eigen::Vector3d>& colli_start, vector<Eigen::Vector3d>& colli_end,
                          vector<Eigen::Vector3d>& start_pts, vector<Eigen::Vector3d>& end_pts);

  void optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path,
                           int traj_id);
  Eigen::MatrixXd paramLocalTraj(double start_t, double& dt, double& duration);
  Eigen::MatrixXd reparamLocalTraj(const double& start_t, const double& duration, const double& dt);

  void selectBestTraj(NonUniformBspline& traj);
  void refineTraj(NonUniformBspline& best_traj);
  void reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt,
                      double& time_inc);

  // Heading planning

  // !SECTION stable

  // SECTION developing

public:
  typedef shared_ptr<FastPlannerManager> Ptr;

  void planYawActMap(const Eigen::Vector3d& start_yaw);
  void test();
  void searchFrontier(const Eigen::Vector3d& p);

private:
  unique_ptr<FrontierFinder> frontier_finder_;
  unique_ptr<HeadingPlanner> heading_planner_;
  unique_ptr<VisibilityUtil> visib_util_;

  // Benchmark method, local exploration
public:
  bool localExplore(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                    Eigen::Vector3d end_pt);

  // !SECTION
};
}  // namespace fast_planner

#endif
