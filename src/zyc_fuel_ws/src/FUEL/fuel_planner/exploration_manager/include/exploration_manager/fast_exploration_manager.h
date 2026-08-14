#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <deque>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <exploration_manager/vertical_detour_policy.h>
using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment;
class SDFMap;
class FastPlannerManager;
class FrontierFinder;
class TaskSearchManager;
struct ExplorationParam;
struct ExplorationData;

enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED };

class FastExplorationManager {
public:
  FastExplorationManager();
  ~FastExplorationManager();

  void initialize(ros::NodeHandle& nh);

  int planExploreMotion(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc,
                        const Vector3d& yaw);
  // 2026-07-13: 执行轨迹预测碰撞时把当前任务目标加入失败冷却，防止下一周期再次选择同一危险点。
  void reportTrajectoryCollision();
  // 2026-07-22: 由FSM里程计回调连续更新任务航迹，旧路判断不再只依赖稀疏重规划时刻。
  void updateMissionOdometry(const Vector3d& pos, double yaw);
  bool shouldStartInflationHistoryEscape(const Vector3d& odom_pos) const;
  bool detectMappedTurnDuringExecution(double yaw, Vector3d& direction);
  bool currentPlanIsTurnInPlace() const { return turn_in_place_plan_; }

  // Benchmark method, classic frontier and rapid frontier
  int classicFrontier(const Vector3d& pos, const double& yaw);
  int rapidFrontier(const Vector3d& pos, const Vector3d& vel, const double& yaw, bool& classic);

  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FrontierFinder> frontier_finder_;
  // unique_ptr<ViewFinder> view_finder_;
  ros::Publisher endpose_pub_;
  ros::Publisher cmu_pose_pub_;
  ros::Subscriber workspace_lock_sub_;
  geometry_msgs::Point end_pose;
  geometry_msgs::PointStamped cmu_pose;
  bool cmu_exploration;
private:
  shared_ptr<EDTEnvironment> edt_environment_;
  shared_ptr<SDFMap> sdf_map_;
  // 2026-07-13: 门后第二阶段由任务搜索模块维护覆盖、重复站点和三类作业目标，FUEL 降级为地图与轨迹后端。
  shared_ptr<TaskSearchManager> task_search_manager_;
  bool mission_entered_search_region_{false};
  bool mission_workspace_lock_received_{false};
  // 2026-07-16: 预留相机观测航向接口；Mid360比赛模式默认按运动方向飞行，不执行原地转头观测。
  bool use_camera_viewpoint_yaw_{false};
  // 2026-07-24: 前机通道搜索期间控制机体航向，为前置相机提供规律扫描视场。
  bool camera_head_sweep_enabled_{true};
  // 2026-07-24: 全程连续360度只保留为调试回退项；比赛默认由独立障碍物触发一次快速环扫。
  bool camera_head_full_rotation_{false};
  double camera_head_sweep_amplitude_rad_{0.5235987756};
  double camera_head_sweep_period_{10.0};
  double camera_head_scan_direction_{1.0};
  ros::Time camera_head_sweep_start_;
  // 2026-07-24: 障碍物导向扫描状态；墙由Mid360累计图负责，前置相机只重点扫描非连续墙占据簇。
  bool camera_obstacle_scan_enabled_{true};
  double camera_clear_sweep_amplitude_rad_{0.6108652382};
  double camera_clear_sweep_period_{3.5};
  double camera_obstacle_scan_period_{6.0};
  double camera_turn_threshold_rad_{0.5235987756};
  double camera_turn_hold_time_{1.2};
  double camera_obstacle_min_forward_{0.55};
  double camera_obstacle_max_forward_{1.60};
  double camera_obstacle_lateral_range_{0.90};
  double camera_obstacle_dedup_radius_{0.55};
  double camera_wall_support_length_{0.90};
  double camera_wall_support_ratio_{0.68};
  bool camera_route_heading_valid_{false};
  double camera_last_route_heading_{0.0};
  ros::Time camera_turning_until_;
  bool camera_obstacle_scan_active_{false};
  Vector3d camera_active_obstacle_{0.0, 0.0, 0.0};
  ros::Time camera_obstacle_scan_start_;
  vector<Vector3d> camera_scanned_obstacles_;
  Vector3d mission_workspace_origin_{0.0, 0.0, 0.0};
  Vector3d mission_workspace_dir_{1.0, 0.0, 0.0};
  // 2026-07-14: 保存任务层请求的最终目标；执行期碰撞不能只冷却 5m 截断后的中间点。
  Vector3d last_requested_goal_{0.0, 0.0, 0.0};
  bool has_last_requested_goal_{false};
  // 正前方不可行后短暂锁定同一侧绕行，推进0.2m后重新允许选择。
  bool recovery_side_latched_{false};
  Vector3d recovery_side_origin_{0.0, 0.0, 0.0};
  Vector3d recovery_side_dir_{1.0, 0.0, 0.0};
  double recovery_side_release_distance_{0.20};
  // 绕障路径转回主方向前，若地图允许则继续沿当前直线多走一段，给机体后部留出通过距离。
  bool straight_run_extension_enabled_{true};
  double straight_run_extension_distance_{0.30};
  double straight_run_extension_min_turn_deg_{35.0};
  // 前方竖直障碍把通道横向分开时，优先横移到占据地图中净宽更大的一侧。
  bool wide_side_bypass_enabled_{true};
  double wide_side_bypass_min_lookahead_{0.20};
  double wide_side_bypass_max_lookahead_{0.90};
  double wide_side_bypass_lateral_range_{0.90};
  double wide_side_bypass_min_lane_width_{0.25};
  double wide_side_bypass_max_lateral_step_{0.45};
  double wide_side_bypass_forward_step_{0.20};
  double wide_side_bypass_low_support_height_{0.20};
  int wide_side_bypass_min_vertical_support_layers_{2};
  // 可选的一次性短回撤；比赛窄通道默认关闭。
  bool short_backtrack_enabled_{false};
  bool short_backtrack_latched_{false};
  Vector3d short_backtrack_release_origin_{0.0, 0.0, 0.0};
  Vector3d short_backtrack_forward_dir_{1.0, 0.0, 0.0};
  Vector3d short_backtrack_last_dir_{0.0, 0.0, 0.0};
  ros::Time short_backtrack_last_time_;
  int short_backtrack_chain_count_{0};
  bool pending_short_backtrack_{false};
  Vector3d pending_short_backtrack_target_{0.0, 0.0, 0.0};
  Vector3d pending_short_backtrack_dir_{0.0, 0.0, 0.0};
  Vector3d pending_short_backtrack_forward_dir_{1.0, 0.0, 0.0};
  double short_backtrack_min_distance_{0.10};
  double short_backtrack_max_distance_{0.20};
  double short_backtrack_forward_release_{0.60};
  double short_backtrack_retry_cooldown_{1.5};
  int short_backtrack_max_chain_{1};
  double short_backtrack_min_direction_change_deg_{35.0};
  // 仅在当前位置落入膨胀层时，沿高密度真实航迹切线前后脱困；不改变普通任务禁回头规则。
  bool inflation_history_escape_enabled_{true};
  double inflation_history_escape_max_distance_{0.45};
  double inflation_history_escape_sample_step_{0.05};
  double inflation_history_escape_clearance_tolerance_{0.02};
  bool inflation_wall_pull_enabled_{true};
  double inflation_escape_clear_margin_{0.10};
  double recovery_history_spacing_{0.03};
  int recovery_history_max_size_{200};
  std::deque<Vector3d> recovery_odom_history_;
  bool turn_in_place_enabled_{true};
  bool turn_in_place_plan_{false};
  double turn_in_place_yaw_rate_deg_{40.0};
  double turn_in_place_min_duration_{1.0};
  double turn_in_place_max_duration_{4.0};
  // 水平绕障全部失败后的三维恢复。下绕必须先原地下降并连续确认，不能生成斜向俯冲轨迹。
  bool vertical_detour_enabled_{true};
  double vertical_detour_low_height_{0.10};
  double vertical_detour_forward_check_distance_{0.20};
  double vertical_detour_upper_step_{0.20};
  double vertical_detour_upper_max_rise_{0.60};
  double vertical_detour_upper_forward_max_distance_{0.60};
  double vertical_detour_upper_max_side_angle_deg_{45.0};
  double vertical_detour_down_first_height_{0.75};
  double vertical_detour_height_tolerance_{0.06};
  double vertical_detour_xy_tolerance_{0.08};
  double vertical_detour_verification_timeout_{2.0};
  double vertical_detour_ascent_timeout_{4.0};
  double vertical_detour_footprint_radius_{0.17};
  int vertical_detour_required_confirmations_{1};
  int vertical_detour_release_confirmations_{2};
  vertical_detour::LowProbePhase low_probe_phase_{
      vertical_detour::LowProbePhase::IDLE};
  int low_probe_confirmations_{0};
  int low_probe_release_confirmations_{0};
  double low_probe_advanced_distance_{0.0};
  Vector3d low_probe_origin_{0.0, 0.0, 0.0};
  Vector3d low_probe_direction_{1.0, 0.0, 0.0};
  Vector3d low_probe_target_{0.0, 0.0, 0.0};
  double low_probe_return_height_{0.60};
  double low_probe_yaw_{0.0};
  ros::Time low_probe_verify_start_;
  ros::Time low_probe_ascent_start_;
  Vector3d low_probe_ascent_origin_{0.0, 0.0, 0.0};

  // Find optimal tour for coarse viewpoints of all frontiers
  void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
                      vector<int>& indices);

  // Refine local tour for next few frontiers, using more diverse viewpoints
  void refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
                       const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
                       vector<Vector3d>& refined_pts, vector<double>& refined_yaws);

  void shortenPath(vector<Vector3d>& path);
  void extendSafeStraightRuns(vector<Vector3d>& path);

  // 2026-07-08 19:26: 为比赛任务层提供最小约束工具，把“先过门进入搜索区、起飞区禁搜、未完成任务不结束”
  // 直接接进 exploration_manager，避免继续只按 frontier 覆盖率行事。
  bool pointInBox(const Vector3d& pt, const Vector3d& box_min, const Vector3d& box_max) const;
  void updateMissionRegionState(const Vector3d& pos);
  bool shouldUseMissionEntryTransit(const Vector3d& pos) const;
  void applyMissionFrontierFilter();
  // 2026-07-10: 门内锁约束工具，FUEL 第二阶段不能越过入口半平面回到起飞区。
  void workspaceLockCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  bool pointInsideWorkspaceLock(const Vector3d& pt) const;
  bool pathInsideWorkspaceLock(const vector<Vector3d>& path) const;
  // 2026-07-13: 动态门后阶段改为多方向任务搜索恢复，不再永久沿入口朝向直飞。
  bool buildMissionForwardFallback(const Vector3d& pos, double cur_yaw, Vector3d& next_pos,
                                   double& next_yaw);
  bool buildWideSideBypass(const Vector3d& pos, double cur_yaw,
                           const Vector3d& forward, Vector3d& next_pos,
                           double& next_yaw, bool& split_obstacle_detected);
  bool occupiedNearHeight(const Vector3d& point, double height) const;
  bool hasLowVerticalSupport(const Vector3d& point,
                             double current_height) const;
  bool planInflationHistoryEscape(const Vector3d& pos, const Vector3d& vel,
                                  const Vector3d& acc, const Vector3d& yaw);
  bool buildTurnInPlacePlan(const Vector3d& pos, const Vector3d& yaw,
                            const Vector3d& turn_direction);
  bool buildVerticalDetourFallback(const Vector3d& pos, double cur_yaw,
                                   const Vector3d& forward, Vector3d& next_pos,
                                   double& next_yaw,
                                   bool require_map_confirmed_underpass = false);
  bool handleActiveLowProbe(const Vector3d& pos, Vector3d& next_pos, double& next_yaw,
                            bool& wait_for_confirmation);
  void cancelActiveLowProbe(const char* reason);
  bool isKnownSafeHorizontalCorridor(const Vector3d& start, const Vector3d& direction,
                                     double distance) const;
  bool isLowProbeCorridorSafe(const Vector3d& start, const Vector3d& direction,
                              double distance) const;
  bool isKnownSafeVerticalPath(const Vector3d& start, double target_z,
                               bool allow_unknown = true) const;
  // 2026-07-24: 以当前高度附近的XY占据柱检测物体，并用沿通道方向的连续支撑剔除左右墙。
  bool cameraOccupancyColumn(const Vector3d& point, double reference_z) const;
  bool cameraWallSupported(const Vector3d& point, const Eigen::Vector2d& travel_dir) const;
  bool detectCameraObstacle(const Vector3d& pos, const Eigen::Vector2d& travel_dir,
                            Vector3d& obstacle_center) const;
  bool cameraObstacleAlreadyScanned(const Vector3d& obstacle_center) const;

public:
  typedef shared_ptr<FastExplorationManager> Ptr;
};

}  // namespace fast_planner

#endif
