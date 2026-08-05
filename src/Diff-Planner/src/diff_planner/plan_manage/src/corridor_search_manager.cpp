// 2026-07-08: 为 indoor1/未知走廊验证新增结构感知搜索管理器。
// 作用：不再用 frontier 最大化当顶层目标，而是根据局部雷达结构给 Diff-Planner 下发短程子目标，
// 避免在起飞区或开阔区域盲目追逐未知地图边界。
// 2026-07-08 19:26: 升级为 FUEL 前置通道进入管理器。
// 起飞后先发布通道入口坐标与可视化标记，再直接给控制器发布 /planning/pos_cmd，
// 等无人机进入搜索区后才触发 exploration_manager，避免 RViz 点击后直接开始刷起飞区 frontier。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

namespace {

struct CandidateScore {
  double yaw = 0.0;
  double score = -std::numeric_limits<double>::infinity();
  double target_dist = 0.0;
  int support_count = 0;
  double forward_extent = 0.0;
  double ref_alignment = -1.0;
  double ref_progress = 0.0;
  int axis_support_count = 0;
  double lateral_span = std::numeric_limits<double>::infinity();
  bool has_goal_point = false;
  bool door_detected = false;
  double door_width = 0.0;
  double door_depth = 0.0;
  geometry_msgs::Point goal_point;
  geometry_msgs::Point door_point;
  geometry_msgs::Point left_post;
  geometry_msgs::Point right_post;
};

struct AccumulatedVoxel {
  int hits = 0;
  int ix = 0;
  int iy = 0;
  int iz = 0;
  pcl::PointXYZ center;
};

double wrapAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double pointDistance2D(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

class CorridorSearchManager {
public:
  CorridorSearchManager() = default;

  void init(ros::NodeHandle& nh) {
    nh_ = nh;

    // 2026-07-08: 参数全部独立收在 search_manager 命名空间，便于后续按场地调节而不改主规划参数。
    nh_.param("search_manager/goal_topic", goal_topic_, std::string("/goal"));
    nh_.param("search_manager/odom_topic", odom_topic_, std::string("/Odometry"));
    nh_.param("search_manager/cloud_topic", cloud_topic_, std::string("/cloud_registered"));
    // 2026-07-09: 前置找门正式切到地图侧，优先使用 FUEL 已融合好的局部占据/未知地图，
    // 不再以瞬时雷达点云作为主判据。
    // 2026-07-09: match 当前这版 FUEL 的 /sdf_map/occupancy_local_inflate 发布链路是空的。
    // 2026-07-09: 门检测改为默认订阅累计占据图 /sdf_map/occupancy_all，再在机体前方局部窗口内裁剪判断；
    // 这样判据更接近 RViz 上看到的累计地图，而不是当前一帧雷达点云或空的 inflate 图。
    nh_.param("search_manager/map_occ_topic", map_occ_topic_, std::string("/sdf_map/occupancy_all"));
    nh_.param("search_manager/map_unknown_topic", map_unknown_topic_, std::string("/sdf_map/unknown"));
    nh_.param("search_manager/trigger_topic", trigger_topic_, std::string("/goal"));
    nh_.param("search_manager/world_frame", world_frame_, std::string("world"));
    // 2026-07-08 19:26: 这些参数把“比赛前置任务阶段”从纯 frontier 搜索里拆出来。
    nh_.param("search_manager/ready_topic", ready_topic_, std::string("/start_after_hover"));
    nh_.param("search_manager/entry_pose_topic", entry_pose_topic_, std::string("/corridor_search/entry_pose"));
    nh_.param("search_manager/exploration_trigger_topic", exploration_trigger_topic_,
              std::string("/corridor_search/exploration_trigger"));
    nh_.param("search_manager/position_cmd_topic", position_cmd_topic_, std::string("/planning/pos_cmd"));
    // 2026-07-10: 给 FUEL 第二阶段发布门平面锁，穿过入口后只允许在门内/作业区侧选择 frontier 和 viewpoint。
    nh_.param("search_manager/workspace_lock_topic", workspace_lock_topic_,
              std::string("/corridor_search/workspace_lock"));
    nh_.param("search_manager/wait_for_hover_ready", wait_for_hover_ready_, true);
    nh_.param("search_manager/use_fixed_entry_goal", use_fixed_entry_goal_, true);
    nh_.param("search_manager/entry_goal_x", entry_goal_.x, -0.20);
    nh_.param("search_manager/entry_goal_y", entry_goal_.y, 1.15);
    nh_.param("search_manager/entry_goal_z", entry_goal_.z, 0.80);
    nh_.param("search_manager/entry_arrive_dist", entry_arrive_dist_, 0.65);
    nh_.param("search_manager/entry_reach_by_search_region", entry_reach_by_search_region_, true);
    nh_.param("search_manager/publish_entry_marker", publish_entry_marker_, true);
    nh_.param("search_manager/publish_pre_entry_pos_cmd", publish_pre_entry_pos_cmd_, true);
    nh_.param("search_manager/search_region_min_x", search_region_min_.x, -1.20);
    nh_.param("search_manager/search_region_min_y", search_region_min_.y, 0.70);
    nh_.param("search_manager/search_region_min_z", search_region_min_.z, 0.20);
    nh_.param("search_manager/search_region_max_x", search_region_max_.x, 6.20);
    nh_.param("search_manager/search_region_max_y", search_region_max_.y, 6.80);
    nh_.param("search_manager/search_region_max_z", search_region_max_.z, 1.80);
    nh_.param("search_manager/min_start_height", min_start_height_, 0.35);
    nh_.param("search_manager/cruise_height", cruise_height_, 0.8);
    nh_.param("search_manager/goal_height_mode", goal_height_mode_, std::string("keep_current"));
    nh_.param("search_manager/min_goal_z", min_goal_z_, 0.5);
    nh_.param("search_manager/max_goal_z", max_goal_z_, 1.2);
    nh_.param("search_manager/local_radius", local_radius_, 4.0);
    nh_.param("search_manager/height_window", height_window_, 0.9);
    nh_.param("search_manager/corridor_half_width", corridor_half_width_, 1.4);
    nh_.param("search_manager/lookahead_dist", lookahead_dist_, 1.8);
    nh_.param("search_manager/probe_dist", probe_dist_, 0.9);
    nh_.param("search_manager/arrive_dist", arrive_dist_, 0.5);
    nh_.param("search_manager/reissue_timeout", reissue_timeout_, 4.0);
    nh_.param("search_manager/stale_cloud_timeout", stale_cloud_timeout_, 1.0);
    nh_.param("search_manager/min_support_points", min_support_points_, 25);
    nh_.param("search_manager/repeat_penalty_radius", repeat_penalty_radius_, 1.2);
    nh_.param("search_manager/max_history_size", max_history_size_, 30);
    nh_.param("search_manager/search_period", search_period_, 0.6);
    nh_.param("search_manager/prefer_heading_weight", prefer_heading_weight_, 1.0);
    nh_.param("search_manager/structure_weight", structure_weight_, 0.05);
    nh_.param("search_manager/repeat_penalty_weight", repeat_penalty_weight_, 1.5);
    nh_.param("search_manager/open_space_penalty", open_space_penalty_, 1.0);
    nh_.param("search_manager/fallback_sweep_step_deg", fallback_sweep_step_deg_, 35.0);
    // 2026-07-08 19:26: 新增“门框搜索”硬阶段参数，前置阶段只在前向扇区里找门，不允许回头扫背面。
    nh_.param("search_manager/front_search_half_fov_deg", front_search_half_fov_deg_, 55.0);
    nh_.param("search_manager/front_search_step_deg", front_search_step_deg_, 10.0);
    nh_.param("search_manager/min_entry_support_points", min_entry_support_points_, 18);
    nh_.param("search_manager/min_entry_progress", min_entry_progress_, 0.60);
    nh_.param("search_manager/entry_hold_dist", entry_hold_dist_, 0.35);
    nh_.param("search_manager/prefer_progress_weight", prefer_progress_weight_, 1.4);
    nh_.param("search_manager/axis_half_width", axis_half_width_, 0.45);
    nh_.param("search_manager/lateral_span_penalty_weight", lateral_span_penalty_weight_, 1.2);
    nh_.param("search_manager/axis_support_weight", axis_support_weight_, 0.08);
    nh_.param("search_manager/forward_extent_weight", forward_extent_weight_, 1.1);
    nh_.param("search_manager/door_projection_ratio", door_projection_ratio_, 0.85);
    // 2026-07-09: 显式门框检测参数，按“墙-开口-墙 + 过门后交接”的流程替代旧的纯方向打分。
    nh_.param("search_manager/door_bin_step_deg", door_bin_step_deg_, 5.0);
    nh_.param("search_manager/min_door_depth", min_door_depth_, 1.4);
    nh_.param("search_manager/max_door_side_range", max_door_side_range_, 2.2);
    nh_.param("search_manager/door_width_min", door_width_min_, 0.4);
    nh_.param("search_manager/door_width_max", door_width_max_, 1.5);
    nh_.param("search_manager/door_prefer_width", door_prefer_width_, 1.2);
    nh_.param("search_manager/door_pass_dist", door_pass_dist_, 0.8);
    nh_.param("search_manager/door_cross_trigger_dist", door_cross_trigger_dist_, 0.25);
    nh_.param("search_manager/door_mid_free_margin", door_mid_free_margin_, 0.55);
    nh_.param("search_manager/door_mid_block_margin", door_mid_block_margin_, 0.20);
    nh_.param("search_manager/max_door_center_lateral", max_door_center_lateral_, 1.4);
    nh_.param("search_manager/min_door_alignment", min_door_alignment_, 0.6);
    nh_.param("search_manager/min_door_gap_free_ratio", min_door_gap_free_ratio_, 0.60);
    nh_.param("search_manager/min_door_side_points", min_door_side_points_, 3);
    nh_.param("search_manager/min_door_post_height", min_door_post_height_, 0.35);
    nh_.param("search_manager/max_door_mid_blocked_bins", max_door_mid_blocked_bins_, 1);
    nh_.param("search_manager/max_door_mid_near_bins", max_door_mid_near_bins_, 0);
    // 2026-07-09: 将“起飞区出口/窄道入口”与“通道内部标准门”拆开处理。
    // 入口门允许更短的成型深度、更少的双侧支撑行数，避免把明显入口误判为非门。
    nh_.param("search_manager/entry_door_depth_min", entry_door_depth_min_, 0.55);
    nh_.param("search_manager/entry_door_support_rows_min", entry_door_support_rows_min_, 2);
    nh_.param("search_manager/entry_door_min_unknown_hits", entry_door_min_unknown_hits_, 3);
    // 2026-07-09: 新增“地图框洞”规则。
    // 只要两侧点云墙形成的间隙在地图上至少接近 1m x 1m，就优先当作比赛入口候选。
    nh_.param("search_manager/frame_opening_width_min", frame_opening_width_min_, 1.0);
    nh_.param("search_manager/frame_opening_height_min", frame_opening_height_min_, 1.0);
    nh_.param("search_manager/frame_opening_depth_min", frame_opening_depth_min_, 0.45);
    nh_.param("search_manager/frame_unknown_hits_min", frame_unknown_hits_min_, 2);
    // 2026-07-10: 比赛门可能不是墙体开洞，而是两根竖直柱形成的 portal。
    // 这种门两侧没有连续墙，必须单独按“竖向柱对 + 中间可通 + 后方可通”识别。
    nh_.param("search_manager/portal_min_post_height", portal_min_post_height_, 0.70);
    nh_.param("search_manager/portal_max_depth_mismatch", portal_max_depth_mismatch_, 0.25);
    nh_.param("search_manager/portal_max_gap_occupied", portal_max_gap_occupied_, 1);
    // 2026-07-10: portal 不能只靠两根柱子成立，门后必须继续出现通道/侧墙结构；
    // 否则天花板投影或近处柱状碎片会被误认为入口。
    nh_.param("search_manager/portal_exit_probe_depth", portal_exit_probe_depth_, 1.10);
    nh_.param("search_manager/portal_min_exit_support_rows", portal_min_exit_support_rows_, 4);
    // 2026-07-09: 起飞后的前几帧地图往往还没稳定，孤立噪声点也容易临时拼出“假门”。
    // 这里增加地图预热和门候选连续确认，避免刚起飞就被一帧坏地图带跑。
    nh_.param("search_manager/map_ready_hold_sec", map_ready_hold_sec_, 1.8);
    nh_.param("search_manager/map_ready_min_points", map_ready_min_points_, 120);
    nh_.param("search_manager/door_confirm_cycles", door_confirm_cycles_, 3);
    nh_.param("search_manager/door_confirm_pos_tol", door_confirm_pos_tol_, 0.35);
    nh_.param("search_manager/door_confirm_width_tol", door_confirm_width_tol_, 0.30);
    // 2026-07-10: 前置阶段要找“第一道入口”，不是找通道里结构分最高/最深的截面；
    // 因此合法门候选先按前向进深从近到远选，只有进深接近时才比较原始结构分。
    nh_.param("search_manager/prefer_first_entry", prefer_first_entry_, true);
    nh_.param("search_manager/first_entry_progress_tolerance", first_entry_progress_tolerance_, 0.35);
    // 2026-07-09: 仿真雷达点云存在明显稀疏/断裂，门检测不能只看当前一帧或当前一次地图消息。
    // 这里在 corridor_search_manager 内部再维护一份体素累计地图，门识别只使用稳定累计后的占据结构。
    nh_.param("search_manager/use_internal_accumulated_map", use_internal_accumulated_map_, true);
    nh_.param("search_manager/map_accum_voxel_res", map_accum_voxel_res_, 0.10);
    nh_.param("search_manager/map_accum_min_hits", map_accum_min_hits_, 2);
    nh_.param("search_manager/map_accum_max_voxels", map_accum_max_voxels_, 60000);
    // 2026-07-10: 仿真雷达会把少量重复散点累计成“稳定点”，这些脏点会切断门洞自由区域。
    // 稳定占据点现在必须有三维邻居或同 XY 竖向连续支撑，孤立重复点不参与找门。
    nh_.param("search_manager/map_accum_min_neighbors", map_accum_min_neighbors_, 2);
    nh_.param("search_manager/map_accum_min_vertical_bins", map_accum_min_vertical_bins_, 2);
    // 2026-07-10: RViz 和找门输入过滤天花板层，只显示/使用比赛目标高度附近的墙和柱。
    nh_.param("search_manager/map_accum_max_use_z", map_accum_max_use_z_, 1.25);
    nh_.param("search_manager/door_path_clearance", door_path_clearance_, 0.25);
    // 2026-07-09: 门检测调试输出。候选门洞被拒绝时，在 RViz 上标注原因并在日志里汇总数量，
    // 用来确认“肉眼看是门”的位置到底卡在宽度、侧墙、高度、纵深还是 unknown 条件。
    nh_.param("search_manager/debug_door_rejects", debug_door_rejects_, true);
    nh_.param("search_manager/max_debug_reject_markers", max_debug_reject_markers_, 16);

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &CorridorSearchManager::odomCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &CorridorSearchManager::cloudCallback, this);
    map_occ_sub_ = nh_.subscribe(map_occ_topic_, 1, &CorridorSearchManager::mapOccCallback, this);
    map_unknown_sub_ = nh_.subscribe(map_unknown_topic_, 1, &CorridorSearchManager::mapUnknownCallback, this);
    trigger_sub_ = nh_.subscribe(trigger_topic_, 1, &CorridorSearchManager::triggerCallback, this);
    ready_sub_ = nh_.subscribe(ready_topic_, 1, &CorridorSearchManager::readyCallback, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1, true);
    entry_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(entry_pose_topic_, 1, true);
    workspace_lock_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(workspace_lock_topic_, 1, true);
    exploration_trigger_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(exploration_trigger_topic_, 1, true);
    pos_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(position_cmd_topic_, 5);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/corridor_search/debug_goal", 1, true);
    // 2026-07-09: 发布内部累计地图，方便 RViz 直接对照“找门实际看的地图”，避免把 FUEL 原始显示和内部判据混淆。
    accumulated_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corridor_search/accumulated_occ_map", 1, true);
    timer_ = nh_.createTimer(ros::Duration(search_period_), &CorridorSearchManager::timerCallback, this);
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    odom_ = *msg;
    have_odom_ = true;

    const auto& q = msg->pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::fromROSMsg(*msg, local_cloud_);
    cloud_stamp_ = msg->header.stamp;
    have_cloud_ = true;
  }

  void mapOccCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> incoming_cloud;
    pcl::fromROSMsg(*msg, incoming_cloud);
    if (!use_internal_accumulated_map_) {
      local_occ_map_cloud_ = incoming_cloud;
    } else {
      accumulateOccupancyMap(incoming_cloud);
    }
    // 2026-07-09: FUEL 某些地图点云历史版本没填 header.stamp，直接用会让 map_age 永远等于仿真总时长。
    // 这里对零时间戳做接收端兜底，至少保证“有图到达”时不会永远卡在 waiting for local occupancy map。
    map_occ_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_occ_map_ = true;
  }

  std::int64_t makeAccumVoxelKey(int ix, int iy, int iz) const {
    constexpr std::int64_t kMask = (1LL << 21) - 1LL;
    constexpr std::int64_t kOffset = 1LL << 20;
    const std::int64_t x = (static_cast<std::int64_t>(ix) + kOffset) & kMask;
    const std::int64_t y = (static_cast<std::int64_t>(iy) + kOffset) & kMask;
    const std::int64_t z = (static_cast<std::int64_t>(iz) + kOffset) & kMask;
    return (x << 42) | (y << 21) | z;
  }

  std::int64_t makeAccumXYKey(int ix, int iy) const {
    constexpr std::int64_t kMask = (1LL << 21) - 1LL;
    constexpr std::int64_t kOffset = 1LL << 20;
    const std::int64_t x = (static_cast<std::int64_t>(ix) + kOffset) & kMask;
    const std::int64_t y = (static_cast<std::int64_t>(iy) + kOffset) & kMask;
    return (x << 21) | y;
  }

  void accumulateOccupancyMap(const pcl::PointCloud<pcl::PointXYZ>& incoming_cloud) {
    if (incoming_cloud.points.empty()) return;

    for (const auto& pt : incoming_cloud.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      // 2026-07-09: 累计地图只保留比赛飞行高度附近的结构，避免地面/天花板体素把门洞误封死。
      if (pt.z < search_region_min_.z || pt.z > search_region_max_.z) continue;
      const int ix = static_cast<int>(std::floor(pt.x / map_accum_voxel_res_));
      const int iy = static_cast<int>(std::floor(pt.y / map_accum_voxel_res_));
      const int iz = static_cast<int>(std::floor(pt.z / map_accum_voxel_res_));
      const std::int64_t key = makeAccumVoxelKey(ix, iy, iz);
      auto& voxel = accumulated_occ_voxels_[key];
      voxel.hits = std::min(voxel.hits + 1, 1000000);
      voxel.ix = ix;
      voxel.iy = iy;
      voxel.iz = iz;
      voxel.center.x = (static_cast<double>(ix) + 0.5) * map_accum_voxel_res_;
      voxel.center.y = (static_cast<double>(iy) + 0.5) * map_accum_voxel_res_;
      voxel.center.z = (static_cast<double>(iz) + 0.5) * map_accum_voxel_res_;
    }

    if (static_cast<int>(accumulated_occ_voxels_.size()) > map_accum_max_voxels_) {
      // 2026-07-09: 防止长时间运行时累计表无限增长。比赛场景很小，超过上限时优先丢掉低命中噪声体素。
      for (auto it = accumulated_occ_voxels_.begin(); it != accumulated_occ_voxels_.end();) {
        if (it->second.hits < map_accum_min_hits_) {
          it = accumulated_occ_voxels_.erase(it);
        } else {
          ++it;
        }
      }
    }

    std::unordered_map<std::int64_t, int> stable_xy_bins;
    stable_xy_bins.reserve(accumulated_occ_voxels_.size());
    for (const auto& item : accumulated_occ_voxels_) {
      if (item.second.hits < map_accum_min_hits_) continue;
      // 2026-07-10: stable XY 统计也排除天花板，避免顶面体素增加“竖向连续”假象。
      if (item.second.center.z > map_accum_max_use_z_) continue;
      ++stable_xy_bins[makeAccumXYKey(item.second.ix, item.second.iy)];
    }

    local_occ_map_cloud_.clear();
    local_occ_map_cloud_.points.reserve(accumulated_occ_voxels_.size());
    int stable_before_noise_filter = 0;
    int noise_filtered = 0;
    int ceiling_filtered = 0;
    for (const auto& item : accumulated_occ_voxels_) {
      const AccumulatedVoxel& voxel = item.second;
      if (voxel.hits < map_accum_min_hits_) continue;
      if (voxel.center.z > map_accum_max_use_z_) {
        ++ceiling_filtered;
        continue;
      }
      ++stable_before_noise_filter;

      int neighbor_count = 0;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            const auto neighbor_it =
                accumulated_occ_voxels_.find(makeAccumVoxelKey(voxel.ix + dx, voxel.iy + dy, voxel.iz + dz));
            if (neighbor_it == accumulated_occ_voxels_.end()) continue;
            if (neighbor_it->second.hits < map_accum_min_hits_) continue;
            if (neighbor_it->second.center.z > map_accum_max_use_z_) continue;
            ++neighbor_count;
          }
        }
      }

      const auto xy_it = stable_xy_bins.find(makeAccumXYKey(voxel.ix, voxel.iy));
      const int vertical_bins = xy_it == stable_xy_bins.end() ? 0 : xy_it->second;
      // 2026-07-10: 墙面/门框应当在累计体素里形成邻接块或竖向柱；单个孤立体素即使命中两次也视作噪声。
      if (neighbor_count < map_accum_min_neighbors_ && vertical_bins < map_accum_min_vertical_bins_) {
        ++noise_filtered;
        continue;
      }

      local_occ_map_cloud_.points.push_back(voxel.center);
    }
    local_occ_map_cloud_.width = static_cast<uint32_t>(local_occ_map_cloud_.points.size());
    local_occ_map_cloud_.height = 1;
    local_occ_map_cloud_.is_dense = false;

    sensor_msgs::PointCloud2 accumulated_msg;
    pcl::toROSMsg(local_occ_map_cloud_, accumulated_msg);
    accumulated_msg.header.stamp = ros::Time::now();
    accumulated_msg.header.frame_id = world_frame_;
    accumulated_map_pub_.publish(accumulated_msg);

    ROS_INFO_THROTTLE(1.0,
                      "[corridor_search_manager] accumulated map voxels=%zu stable_raw=%d stable_filtered=%zu"
                      " noise_removed=%d ceiling_removed=%d min_hits=%d neigh=%d vertical=%d max_z=%.2f.",
                      accumulated_occ_voxels_.size(), stable_before_noise_filter,
                      local_occ_map_cloud_.points.size(), noise_filtered, ceiling_filtered,
                      map_accum_min_hits_, map_accum_min_neighbors_, map_accum_min_vertical_bins_,
                      map_accum_max_use_z_);
  }

  void mapUnknownCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::fromROSMsg(*msg, local_unknown_map_cloud_);
    map_unknown_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_unknown_map_ = true;
  }

  void readyCallback(const std_msgs::BoolConstPtr& msg) {
    hover_ready_ = msg->data;
  }

  bool entrySearchReady(const ros::Time& now) {
    const double map_age =
        have_occ_map_ ? (now - map_occ_stamp_).toSec() : std::numeric_limits<double>::infinity();
    const bool map_ready = have_occ_map_ && map_age <= stale_cloud_timeout_;
    if (!map_ready) {
      map_ready_start_time_ = ros::Time(0);
      ROS_WARN_THROTTLE(1.0,
                        "[corridor_search_manager] waiting for local occupancy map. have_occ_map=%d age=%.2f",
                        static_cast<int>(have_occ_map_), map_age);
      return false;
    }

    if (map_ready_start_time_.isZero()) {
      map_ready_start_time_ = now;
      ROS_INFO("[corridor_search_manager] local occupancy map became available, start warmup.");
      return false;
    }

    const double warmup_sec = (now - map_ready_start_time_).toSec();
    if (warmup_sec < map_ready_hold_sec_) {
      ROS_WARN_THROTTLE(1.0,
                        "[corridor_search_manager] warming up local map %.2f/%.2fs before door search.",
                        warmup_sec, map_ready_hold_sec_);
      return false;
    }

    const int occ_points = static_cast<int>(local_occ_map_cloud_.points.size());
    if (occ_points < map_ready_min_points_) {
      ROS_WARN_THROTTLE(1.0,
                        "[corridor_search_manager] waiting denser local map, occ_points=%d < %d.",
                        occ_points, map_ready_min_points_);
      return false;
    }
    return true;
  }

  bool confirmDoorCandidate(const CandidateScore& candidate) {
    if (!candidate.door_detected) {
      have_pending_door_ = false;
      pending_door_confirm_count_ = 0;
      return false;
    }

    if (!have_pending_door_) {
      pending_door_candidate_ = candidate;
      pending_door_confirm_count_ = 1;
      have_pending_door_ = true;
      return door_confirm_cycles_ <= 1;
    }

    const double dx = candidate.door_point.x - pending_door_candidate_.door_point.x;
    const double dy = candidate.door_point.y - pending_door_candidate_.door_point.y;
    const double pos_dist = std::sqrt(dx * dx + dy * dy);
    const double width_diff = std::fabs(candidate.door_width - pending_door_candidate_.door_width);
    if (pos_dist <= door_confirm_pos_tol_ && width_diff <= door_confirm_width_tol_) {
      pending_door_candidate_ = candidate;
      ++pending_door_confirm_count_;
    } else {
      pending_door_candidate_ = candidate;
      pending_door_confirm_count_ = 1;
    }
    return pending_door_confirm_count_ >= door_confirm_cycles_;
  }

  bool shouldReplaceDoorCandidate(double score, double progress, const CandidateScore& best) const {
    if (!best.door_detected) return true;

    // 2026-07-10: 入口识别不能按“越往里越高分”选，否则会跳过起飞区边界的第一道入口，
    // 锁到通道后方的第二个截面。前置阶段优先选择第一个稳定可通过的门口。
    if (prefer_first_entry_) {
      if (progress + first_entry_progress_tolerance_ < best.ref_progress) return true;
      if (progress > best.ref_progress + first_entry_progress_tolerance_) return false;
    }

    return score > best.score;
  }

  void timerCallback(const ros::TimerEvent&) {
    // 2026-07-21: 按用户要求回退15:37/15:46改动，恢复触发后再执行原入口搜索流程。
    if (!have_odom_) {
      ROS_WARN_THROTTLE(1.0, "[corridor_search_manager] waiting for odometry.");
      return;
    }

    if (!search_enabled_) {
      ROS_WARN_THROTTLE(1.0, "[corridor_search_manager] waiting for RViz trigger.");
      return;
    }

    const ros::Time now = ros::Time::now();
    publishEntryReference();

    if (wait_for_hover_ready_ && !hover_ready_) {
      ROS_WARN_THROTTLE(1.0, "[corridor_search_manager] waiting for /start_after_hover.");
      return;
    }

    if (odom_.pose.pose.position.z < min_start_height_) {
      // 2026-07-08: 起飞前或刚离地时 /Odometry 高度仍接近地面，Diff 的 virtual_ground 会把 start_pt 判成越界/障碍。
      // 这里先禁止发布搜索目标，等高度稳定高于起飞门槛后再启动走廊搜索，避免反复出现 start_occ=-1。
      ROS_WARN_THROTTLE(1.0,
                        "[corridor_search_manager] waiting for takeoff, current z=%.3f < min_start_height=%.3f.",
                        odom_.pose.pose.position.z, min_start_height_);
      return;
    }

    if (exploration_triggered_) {
      ROS_INFO_THROTTLE(2.0, "[corridor_search_manager] exploration already triggered.");
      return;
    }

    if (mission_stage_ == SEARCH_ENTRY && !entry_locked_) {
      if (!entrySearchReady(now)) {
        // 2026-07-09: 地图未预热完成前不再发布前向位移点。
        // 最新日志显示 0.25m warmup 点会被连续跟踪并把飞机慢慢送到墙上，预热阶段只允许原地保持。
        CandidateScore warmup_candidate = makeHoldCandidate(entry_search_ref_yaw_);
        publishGoal(warmup_candidate, now, "map_warmup");
        if (publish_pre_entry_pos_cmd_) publishPositionCommand(active_goal_, now);
        return;
      }

      CandidateScore candidate = detectEntryCandidate();
      const bool support_ok = candidate.support_count >= min_entry_support_points_;
      const bool progress_ok = candidate.ref_progress >= min_entry_progress_;
      const bool door_confirmed = confirmDoorCandidate(candidate);
      // 未经连续确认的入口候选只用于检测和调试，不能直接发给飞控。
      // 否则累计地图每次更新造成的门中心/航向小幅跳变，会通过 PositionCommand
      // 直接表现为无人机左右摆动。
      const bool entry_ready = candidate.door_detected && support_ok && progress_ok && door_confirmed;
      if (entry_ready) {
        publishGoal(candidate, now, "confirmed_entry");
      } else {
        CandidateScore hold_candidate = makeHoldCandidate(entry_search_ref_yaw_);
        publishGoal(hold_candidate, now, "entry_wait_hold");
      }
      if (publish_pre_entry_pos_cmd_) publishPositionCommand(active_goal_, now);

      // 2026-07-09: 没有显式门框检测成功时，绝不允许直接锁远目标。
      // 上一版就是在这里把右侧墙边的高分方向误锁成了“门”。
      if (candidate.door_detected && !door_confirmed) {
        ROS_WARN_THROTTLE(0.8,
                          "[corridor_search_manager] door candidate seen but waiting confirmation %d/%d."
                          " center=(%.2f, %.2f) width=%.2f support=%d progress=%.2f",
                          pending_door_confirm_count_, door_confirm_cycles_, candidate.door_point.x,
                          candidate.door_point.y, candidate.door_width, candidate.support_count,
                          candidate.ref_progress);
      }
      if (entry_ready) {
        // 2026-07-08 19:26: 一旦在前向扇区里找到足够像门框的候选，就锁定入口，后续只准压着这个门过。
        locked_entry_goal_ = active_goal_;
        entry_locked_ = true;
        locked_door_detected_ = candidate.door_detected;
        locked_door_point_ = candidate.door_point;
        locked_entry_yaw_ = candidate.yaw;
        locked_door_width_ = candidate.door_width;
        mission_stage_ = APPROACH_ENTRY;
        ROS_WARN("[corridor_search_manager] entry locked at (%.2f, %.2f, %.2f), support=%d, progress=%.2f, door=%.2f/%.2f.",
                 locked_entry_goal_.pose.position.x, locked_entry_goal_.pose.position.y,
                 locked_entry_goal_.pose.position.z, candidate.support_count, candidate.ref_progress,
                 candidate.door_point.x, candidate.door_point.y);
      } else if (!candidate.door_detected) {
        ROS_WARN_THROTTLE(1.0,
                          "[corridor_search_manager] no explicit door detected yet, keep probing. goal=(%.2f, %.2f, %.2f)",
                          active_goal_.pose.position.x, active_goal_.pose.position.y, active_goal_.pose.position.z);
      }
      return;
    }

    if (mission_stage_ == APPROACH_ENTRY) {
      active_goal_ = locked_entry_goal_;
      active_goal_.header.stamp = now;
      have_active_goal_ = true;
      goal_pub_.publish(active_goal_);
      publishMarker(active_goal_, "locked_entry", 0.95, 0.55, 0.05);
      if (publish_pre_entry_pos_cmd_) publishPositionCommand(active_goal_, now);
      if (hasReachedEntry()) {
        triggerExploration(now);
        return;
      }
      return;
    }

    if (hasReachedEntry()) {
      triggerExploration(now);
      return;
    }

    CandidateScore candidate = use_fixed_entry_goal_ ? makeEntryCandidate() : detectEntryCandidate();
    publishGoal(candidate, now, "search_entry");
    if (publish_pre_entry_pos_cmd_) publishPositionCommand(active_goal_, now);
  }

  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    if (search_enabled_) return;
    search_enabled_ = true;
    entry_search_ref_yaw_ = current_yaw_;
    mission_stage_ = SEARCH_ENTRY;
    ROS_INFO("[corridor_search_manager] search enabled by trigger at (%.2f, %.2f, %.2f).",
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }

  CandidateScore makeEntryCandidate() const {
    CandidateScore candidate;
    const double dx = entry_goal_.x - odom_.pose.pose.position.x;
    const double dy = entry_goal_.y - odom_.pose.pose.position.y;
    candidate.yaw = std::atan2(dy, dx);
    candidate.score = 1.0;
    candidate.target_dist = std::sqrt(dx * dx + dy * dy);
    candidate.forward_extent = candidate.target_dist;
    candidate.has_goal_point = true;
    candidate.goal_point = entry_goal_;
    candidate.door_point = entry_goal_;
    return candidate;
  }

  CandidateScore makeHoldCandidate(double yaw) const {
    CandidateScore candidate;
    candidate.yaw = yaw;
    candidate.score = 0.0;
    candidate.target_dist = 0.0;
    candidate.ref_alignment = std::cos(wrapAngle(yaw - entry_search_ref_yaw_));
    candidate.ref_progress = 0.0;
    candidate.support_count = 0;
    candidate.axis_support_count = 0;
    candidate.forward_extent = 0.0;
    candidate.lateral_span = std::numeric_limits<double>::infinity();
    candidate.goal_point = odom_.pose.pose.position;
    candidate.goal_point.z = resolveGoalHeightFor(odom_.pose.pose.position.z);
    candidate.door_point = candidate.goal_point;
    candidate.has_goal_point = true;
    return candidate;
  }

  CandidateScore chooseCandidate() {
    CandidateScore best;

    const double cloud_age = have_cloud_ ? (ros::Time::now() - cloud_stamp_).toSec() : std::numeric_limits<double>::infinity();
    const bool cloud_ready = have_cloud_ && cloud_age <= stale_cloud_timeout_;
    const std::vector<double> offsets_deg = {0.0, 35.0, -35.0, 70.0, -70.0, 110.0, -110.0, 180.0};

    const double fallback_bias = fallback_sweep_step_deg_ * static_cast<double>(fallback_index_);
    double sparse_best_score = -std::numeric_limits<double>::infinity();
    double sparse_best_yaw = current_yaw_;

    for (double offset_deg : offsets_deg) {
      const double yaw = wrapAngle(current_yaw_ + (offset_deg + fallback_bias) * M_PI / 180.0);
      CandidateScore score = evaluateDirection(yaw, cloud_ready);
      if (score.score > best.score) best = score;
      if (!cloud_ready && score.score > sparse_best_score) {
        sparse_best_score = score.score;
        sparse_best_yaw = yaw;
      }
    }

    if (!cloud_ready || best.support_count < min_support_points_) {
      // 2026-07-08: 点云稀疏或附近没有明显结构时，不再长距离冲向未知，而是沿偏航扇区做短距离探测。
      best.yaw = sparse_best_yaw;
      best.score = sparse_best_score;
      best.target_dist = probe_dist_;
      best.support_count = 0;
      best.forward_extent = 0.0;
      fallback_index_ = (fallback_index_ + 1) % 6;
    } else {
      fallback_index_ = 0;
    }

    return best;
  }

  CandidateScore detectEntryCandidate() {
    if (use_fixed_entry_goal_) {
      return makeEntryCandidate();
    }

    CandidateScore best;
    const double map_age =
        have_occ_map_ ? (ros::Time::now() - map_occ_stamp_).toSec() : std::numeric_limits<double>::infinity();
    const bool map_ready = have_occ_map_ && map_age <= stale_cloud_timeout_;
    if (!map_ready) {
      // 2026-07-09: 当前置地图还没准备好时，不再退回到原始雷达点云方向打分；
      // 这里只保留一个严格受限的前向小步扫描，等待局部占据地图建立起来。
      CandidateScore fallback;
      fallback.yaw = entry_search_ref_yaw_;
      fallback.target_dist = probe_dist_;
      fallback.ref_alignment = 1.0;
      fallback.ref_progress = probe_dist_;
      fallback.score = 0.0;
      fallback.goal_point = makePoint(probe_dist_, entry_search_ref_yaw_);
      fallback.door_point = fallback.goal_point;
      fallback.has_goal_point = true;
      return fallback;
    }

    // 2026-07-09: 显式找“左右墙柱 + 中间空隙”的门框几何。
    // 上一版把“开口”误写成“前方有足够远的点云”，这在雷达场景里会把真门口漏掉，
    // 因为门中间往往正是空的。这里改成直接找两侧近墙点和中间空隙。
    CandidateScore door_candidate = detectDoorOpening();
    if (door_candidate.door_detected) {
      return door_candidate;
    }

    // 2026-07-09: 用户要求去掉“当前雷达点云高分方向”这一套 fallback。
    // 2026-07-09: 最新日志确认找不到门时的 0.35m 左右摆扫会持续把飞机送进墙里。
    // Mid360 已提供 360 度视野，未检测到门时固定触发瞬间的参考航向原地等待。
    // 不再快速交替发布正负 yaw 扫描，避免控制器带动机体左右摆动。
    return makeHoldCandidate(entry_search_ref_yaw_);
  }

  CandidateScore detectDoorOpening() const {
    // 2026-07-09: 门检测改成“局部二维栅格 + 连通域”。
    // 不再依赖角度 bin 的“墙-空-墙”瞬时几何，而是在前向局部地图里直接找
    // 被两侧占据夹出的可通行自由矩形，并验证其后方是否连到更深的自由区域。
    struct GridCell {
      int count = 0;
      double min_z = std::numeric_limits<double>::infinity();
      double max_z = -std::numeric_limits<double>::infinity();
      bool occupied = false;
    };

    CandidateScore best;
    struct RejectStats {
      int runs = 0;
      int width = 0;
      int lateral = 0;
      int side_wall = 0;
      int component = 0;
      int progress = 0;
      int free_ratio = 0;
      int alignment = 0;
      int semantic = 0;
      int path = 0;
      int portal = 0;
      int portal_exit = 0;
      int accepted = 0;
      int occ_window = 0;
      int unknown_window = 0;
    } stats;
    int debug_marker_id = 100;
    const double grid_res = 0.10;
    const double lateral_limit = std::max(corridor_half_width_, max_door_center_lateral_ + door_width_max_);
    const int rows = std::max(12, static_cast<int>(std::ceil(local_radius_ / grid_res)));
    const int cols = std::max(12, static_cast<int>(std::ceil((2.0 * lateral_limit) / grid_res)));
    // 2026-07-09: 这里输入是累计占据点云 /sdf_map/occupancy_all，不额外做栅格膨胀；
    // 门宽判定直接基于 RViz 上能看到的占据体素。
    const int inflate_cells = 0;
    const int min_component_cells = 10;
    const int min_row = std::max(2, static_cast<int>(std::floor(std::max(0.45, min_entry_progress_) / grid_res)));
    // 2026-07-09: 之前这里把门搜索前向深度错误地截在 max_door_side_range_ 附近，
    // 结果只能看离机体很近的几排栅格，远处已经成形的主通道门口根本不会进入候选。
    // 这里改成搜索完整前向局部窗口，允许“近处空、远处连续墙体和通道已经成形”的门被检测到。
    const int max_row = rows - 2;
    const int min_depth_cells = std::max(3, static_cast<int>(std::ceil(min_door_depth_ / grid_res)));
    // 2026-07-09: 之前只在开口边缘外各探 0.3m 左右找侧墙，膨胀后很容易探不到真正的门柱/侧墙，
    // 尤其是“中间空得很明显、两侧墙稍远一点”的走廊入口。这里放宽侧向探测范围。
    const int side_probe_cells =
        std::max(4, static_cast<int>(std::ceil((0.5 * door_width_max_ + 0.35) / grid_res)));
    std::vector<GridCell> grid(rows * cols);
    std::vector<int> unknown_counts(rows * cols, 0);

    const double dir_x = std::cos(entry_search_ref_yaw_);
    const double dir_y = std::sin(entry_search_ref_yaw_);
    const auto cellIndex = [cols](int r, int c) { return r * cols + c; };
    const auto inBounds = [rows, cols](int r, int c) { return r >= 0 && r < rows && c >= 0 && c < cols; };
    const auto worldToGrid = [&](double world_x, double world_y, int& r, int& c) {
      const double rel_x = world_x - odom_.pose.pose.position.x;
      const double rel_y = world_y - odom_.pose.pose.position.y;
      const double forward = rel_x * dir_x + rel_y * dir_y;
      const double lateral = rel_x * (-dir_y) + rel_y * dir_x;
      if (forward < 0.0 || forward > local_radius_) return false;
      if (std::fabs(lateral) > lateral_limit) return false;
      r = std::max(0, std::min(rows - 1, static_cast<int>(std::floor(forward / grid_res))));
      c = std::max(0, std::min(cols - 1, static_cast<int>(std::floor((lateral + lateral_limit) / grid_res))));
      return true;
    };
    const auto gridToWorld = [&](int r, int c) {
      geometry_msgs::Point pt;
      const double forward = (static_cast<double>(r) + 0.5) * grid_res;
      const double lateral = -lateral_limit + (static_cast<double>(c) + 0.5) * grid_res;
      pt.x = odom_.pose.pose.position.x + forward * dir_x + lateral * (-dir_y);
      pt.y = odom_.pose.pose.position.y + forward * dir_y + lateral * dir_x;
      pt.z = resolveGoalHeightFor(odom_.pose.pose.position.z);
      return pt;
    };
    const auto publishRejectMarker = [&](int row, int col, const std::string& reason) {
      if (!debug_door_rejects_ || debug_marker_id >= 100 + max_debug_reject_markers_) return;
      visualization_msgs::Marker marker;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = world_frame_;
      marker.ns = "door_reject";
      marker.id = debug_marker_id++;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.pose.position = gridToWorld(row, col);
      marker.pose.position.z = std::min(max_goal_z_, marker.pose.position.z + 0.55);
      marker.scale.z = 0.18;
      marker.color.a = 0.95;
      marker.color.r = 1.0;
      marker.color.g = 0.10;
      marker.color.b = 0.05;
      marker.lifetime = ros::Duration(std::max(0.5, search_period_ * 5.0));
      marker.text = reason;
      marker_pub_.publish(marker);
    };

    // 2026-07-09: 这里开始使用 /sdf_map/occupancy_all，而不是 /cloud_registered。
    // 先从累计地图里裁剪机体前方窗口，再在这个窗口中找门框/通道入口。
    for (const auto& pt : local_occ_map_cloud_.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      const double rel_z = pt.z - odom_.pose.pose.position.z;
      if (std::fabs(rel_z) > height_window_) continue;

      int r = 0;
      int c = 0;
      if (!worldToGrid(pt.x, pt.y, r, c)) continue;
      ++stats.occ_window;
      GridCell& cell = grid[cellIndex(r, c)];
      ++cell.count;
      cell.min_z = std::min(cell.min_z, static_cast<double>(pt.z));
      cell.max_z = std::max(cell.max_z, static_cast<double>(pt.z));
    }

    // 2026-07-09: unknown 地图只作为“这个开口后面是不是还通向未知作业区”的附加偏置，
    // 不把它直接当障碍物用。
    for (const auto& pt : local_unknown_map_cloud_.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      const double rel_z = pt.z - odom_.pose.pose.position.z;
      if (std::fabs(rel_z) > height_window_) continue;

      int r = 0;
      int c = 0;
      if (!worldToGrid(pt.x, pt.y, r, c)) continue;
      ++stats.unknown_window;
      ++unknown_counts[cellIndex(r, c)];
    }

    for (auto& cell : grid) {
      cell.occupied = cell.count > 0;
    }

    std::vector<bool> inflated_occ(rows * cols, false);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        if (!grid[cellIndex(r, c)].occupied) continue;
        for (int dr = -inflate_cells; dr <= inflate_cells; ++dr) {
          for (int dc = -inflate_cells; dc <= inflate_cells; ++dc) {
            const int nr = r + dr;
            const int nc = c + dc;
            if (!inBounds(nr, nc)) continue;
            inflated_occ[cellIndex(nr, nc)] = true;
          }
        }
      }
    }

    std::vector<bool> wall_occ(rows * cols, false);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        if (!grid[cellIndex(r, c)].occupied) continue;
        // 2026-07-09: 雷达累计占据图里的墙经常是断裂体素，直接按单行 free-run 会把真门洞切碎。
        // 这里仅对“墙体判据”做一格闭合式膨胀，用于找连续门框；中间通行自由度仍用原始/未膨胀占据校验。
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            const int nr = r + dr;
            const int nc = c + dc;
            if (!inBounds(nr, nc)) continue;
            wall_occ[cellIndex(nr, nc)] = true;
          }
        }
      }
    }

    auto sideWallInfo = [&](int row, int col, int direction, geometry_msgs::Point& post_pt,
                            double& vertical_span, int& support_count) {
      int best_col = -1;
      int best_count = 0;
      double best_forward_bias = std::numeric_limits<double>::infinity();
      for (int step = 1; step <= side_probe_cells + 2; ++step) {
        const int nc = col + direction * step;
        if (!inBounds(row, nc)) break;
        int total_count = 0;
        double min_z = std::numeric_limits<double>::infinity();
        double max_z = -std::numeric_limits<double>::infinity();
        // 2026-07-09: 占据地图是体素点云，单个 XY 格子里的 z 跨度经常不够。
        // 门柱/侧墙高度改为在附近几行同列聚合，避免真实竖直墙面被单格高度过滤掉。
        for (int rr = std::max(0, row - 1); rr <= std::min(rows - 1, row + 1); ++rr) {
          const GridCell& cell = grid[cellIndex(rr, nc)];
          if (!cell.occupied) continue;
          total_count += cell.count;
          min_z = std::min(min_z, cell.min_z);
          max_z = std::max(max_z, cell.max_z);
        }
        const double span = max_z - min_z;
        if (total_count >= min_door_side_points_ && span >= min_door_post_height_) {
          const double bias = static_cast<double>(step);
          if (total_count > best_count || (total_count == best_count && bias < best_forward_bias)) {
            best_col = nc;
            best_count = total_count;
            best_forward_bias = bias;
            vertical_span = span;
          }
        }
      }
      if (best_col < 0) return false;
      post_pt = gridToWorld(row, best_col);
      post_pt.z = resolveGoalHeightFor(odom_.pose.pose.position.z);
      support_count = best_count;
      return true;
    };

    auto aggregatePostInfo = [&](int row, int col, geometry_msgs::Point& post_pt,
                                 double& vertical_span, int& support_count) {
      int total_count = 0;
      double min_z = std::numeric_limits<double>::infinity();
      double max_z = -std::numeric_limits<double>::infinity();
      const int row_margin = 2;
      const int col_margin = 1;
      for (int rr = std::max(0, row - row_margin); rr <= std::min(rows - 1, row + row_margin); ++rr) {
        for (int cc = std::max(0, col - col_margin); cc <= std::min(cols - 1, col + col_margin); ++cc) {
          const GridCell& cell = grid[cellIndex(rr, cc)];
          if (!cell.occupied) continue;
          total_count += cell.count;
          min_z = std::min(min_z, cell.min_z);
          max_z = std::max(max_z, cell.max_z);
        }
      }
      if (total_count < min_door_side_points_) return false;
      vertical_span = max_z - min_z;
      if (vertical_span < min_door_post_height_) return false;
      post_pt = gridToWorld(row, col);
      post_pt.z = resolveGoalHeightFor(odom_.pose.pose.position.z);
      support_count = total_count;
      return true;
    };

    auto hasWallNear = [&](int row, int col, int row_margin, int col_margin) {
      for (int rr = std::max(0, row - row_margin); rr <= std::min(rows - 1, row + row_margin); ++rr) {
        for (int cc = std::max(0, col - col_margin); cc <= std::min(cols - 1, col + col_margin); ++cc) {
          if (wall_occ[cellIndex(rr, cc)]) return true;
        }
      }
      return false;
    };

    auto countRawOccupiedInGap = [&](int row, int left_col, int right_col) {
      int blocked = 0;
      for (int cc = std::max(0, left_col + 2); cc <= std::min(cols - 1, right_col - 2); ++cc) {
        if (grid[cellIndex(row, cc)].occupied) ++blocked;
      }
      return blocked;
    };

    auto pathToThroughClear = [&](int door_row, int through_row, int center_col) {
      const int half_clear_cells = std::max(2, static_cast<int>(std::ceil(door_path_clearance_ / grid_res)));
      int blocked = 0;
      for (int rr = 1; rr <= std::min(rows - 1, through_row); ++rr) {
        // 2026-07-09: 门框所在行允许两侧门柱贴近，但从飞机到门后点的中心走廊必须基本无占据。
        const int local_half = std::abs(rr - door_row) <= 1 ? std::max(1, half_clear_cells - 1) : half_clear_cells;
        for (int cc = std::max(0, center_col - local_half);
             cc <= std::min(cols - 1, center_col + local_half); ++cc) {
          if (!grid[cellIndex(rr, cc)].occupied) continue;
          ++blocked;
          if (blocked > max_door_mid_blocked_bins_) return false;
        }
      }
      return true;
    };

    auto gapClearNearDoor = [&](int row, int left_col, int right_col) {
      int blocked = 0;
      for (int rr = std::max(0, row - 1); rr <= std::min(rows - 1, row + 1); ++rr) {
        for (int cc = std::max(0, left_col + 2); cc <= std::min(cols - 1, right_col - 2); ++cc) {
          if (!grid[cellIndex(rr, cc)].occupied) continue;
          ++blocked;
          if (blocked > portal_max_gap_occupied_) return false;
        }
      }
      return true;
    };

    auto portalExitSupported = [&](int door_row, int through_row, int left_col, int right_col, int center_col,
                                   int& support_rows_out) {
      support_rows_out = 0;
      const int exit_end_row =
          std::min(rows - 1, door_row + static_cast<int>(std::ceil(portal_exit_probe_depth_ / grid_res)));
      const int half_clear_cells = std::max(1, static_cast<int>(std::ceil(door_path_clearance_ / grid_res)));
      for (int rr = through_row; rr <= exit_end_row; ++rr) {
        int center_blocked = 0;
        for (int cc = std::max(0, center_col - half_clear_cells);
             cc <= std::min(cols - 1, center_col + half_clear_cells); ++cc) {
          if (grid[cellIndex(rr, cc)].occupied) ++center_blocked;
        }
        if (center_blocked > max_door_mid_blocked_bins_) continue;

        bool side_support = false;
        for (int cc = std::max(0, left_col - 3); cc <= std::min(cols - 1, left_col + 1); ++cc) {
          if (hasWallNear(rr, cc, 1, 1)) {
            side_support = true;
            break;
          }
        }
        if (!side_support) {
          for (int cc = std::max(0, right_col - 1); cc <= std::min(cols - 1, right_col + 3); ++cc) {
            if (hasWallNear(rr, cc, 1, 1)) {
              side_support = true;
              break;
            }
          }
        }
        if (side_support) ++support_rows_out;
      }
      return support_rows_out >= portal_min_exit_support_rows_;
    };

    // 2026-07-10: 专门识别图片里的“柱门/门架”。
    // 这类入口在累计地图中表现为两根竖直柱，中间没有连续墙体；旧的墙洞检测会因为缺少侧墙延伸而漏检。
    const int min_portal_width_cells =
        std::max(3, static_cast<int>(std::ceil(door_width_min_ / grid_res)));
    const int max_portal_width_cells =
        std::max(min_portal_width_cells + 1, static_cast<int>(std::ceil(door_width_max_ / grid_res)));
    const int max_depth_mismatch_cells =
        std::max(1, static_cast<int>(std::ceil(portal_max_depth_mismatch_ / grid_res)));
    for (int left_row = min_row; left_row <= max_row; ++left_row) {
      for (int left_col = 1; left_col < cols - min_portal_width_cells - 1; ++left_col) {
        if (!grid[cellIndex(left_row, left_col)].occupied) continue;

        geometry_msgs::Point left_post;
        double left_span = 0.0;
        int left_support = 0;
        if (!aggregatePostInfo(left_row, left_col, left_post, left_span, left_support) ||
            left_span < portal_min_post_height_) {
          continue;
        }

        for (int gap_cells = min_portal_width_cells; gap_cells <= max_portal_width_cells; ++gap_cells) {
          const int right_col = left_col + gap_cells;
          if (right_col >= cols - 1) break;

          for (int right_row = std::max(min_row, left_row - max_depth_mismatch_cells);
               right_row <= std::min(max_row, left_row + max_depth_mismatch_cells); ++right_row) {
            if (!grid[cellIndex(right_row, right_col)].occupied) continue;

            geometry_msgs::Point right_post;
            double right_span = 0.0;
            int right_support = 0;
            if (!aggregatePostInfo(right_row, right_col, right_post, right_span, right_support) ||
                right_span < portal_min_post_height_) {
              continue;
            }

            const int door_row = (left_row + right_row) / 2;
            const int center_col = (left_col + right_col) / 2;
            const double center_lateral = -lateral_limit + (static_cast<double>(center_col) + 0.5) * grid_res;
            if (std::fabs(center_lateral) > max_door_center_lateral_) continue;
            if (!gapClearNearDoor(door_row, left_col, right_col)) continue;

            const int through_row =
                std::min(rows - 1, door_row + static_cast<int>(std::ceil(door_pass_dist_ / grid_res)));
            if (!pathToThroughClear(door_row, through_row, center_col)) {
              ++stats.path;
              publishRejectMarker(door_row, center_col, "portal_path");
              continue;
            }
            int portal_exit_support_rows = 0;
            if (!portalExitSupported(door_row, through_row, left_col, right_col, center_col,
                                     portal_exit_support_rows)) {
              ++stats.portal_exit;
              publishRejectMarker(door_row, center_col, "portal_exit");
              continue;
            }

            geometry_msgs::Point center_pt = gridToWorld(door_row, center_col);
            const double alignment = std::cos(wrapAngle(std::atan2(center_pt.y - odom_.pose.pose.position.y,
                                                                   center_pt.x - odom_.pose.pose.position.x) -
                                                        entry_search_ref_yaw_));
            if (alignment < min_door_alignment_) continue;

            const double progress = (static_cast<double>(door_row) + 0.5) * grid_res;
            if (progress < min_entry_progress_) continue;

            ++stats.portal;
            const double width = static_cast<double>(gap_cells) * grid_res;
            const double width_score =
                1.0 - std::min(1.0, std::fabs(width - door_prefer_width_) / std::max(0.2, door_prefer_width_));
            const double height_score =
                std::min(left_span, right_span) / std::max(0.1, portal_min_post_height_);
            const double score = 6.0 * progress + 2.8 * width_score +
                                 1.6 * std::min(1.8, height_score) +
                                 0.25 * static_cast<double>(portal_exit_support_rows) +
                                 0.08 * static_cast<double>(left_support + right_support) -
                                 1.2 * std::fabs(center_lateral);
            if (!shouldReplaceDoorCandidate(score, progress, best)) continue;

            best.score = score;
            best.yaw = entry_search_ref_yaw_;
            best.target_dist = std::max(probe_dist_, progress + door_pass_dist_);
            best.support_count = left_support + right_support;
            best.forward_extent = door_pass_dist_;
            best.ref_alignment = alignment;
            best.ref_progress = progress;
            best.axis_support_count = 2;
            best.lateral_span = width;
            best.has_goal_point = true;
            best.door_detected = true;
            best.door_width = width;
            best.door_depth = door_pass_dist_;
            best.left_post = left_post;
            best.right_post = right_post;
            best.door_point = center_pt;
            best.goal_point = gridToWorld(through_row, center_col);
          }
        }
      }
    }

    // 2026-07-09: 比赛场景入口不一定能在雷达累计图里形成完整双边门框。
    // 若一侧已经形成连续墙，另一侧是足够大的空缺，且门后中心走廊可通行，则按“单边墙推断入口”处理。
    const int one_sided_support_rows_min =
        std::max(5, static_cast<int>(std::ceil(std::max(0.7, entry_door_depth_min_) / grid_res)));
    const int one_sided_gap_min_cells =
        std::max(5, static_cast<int>(std::ceil(std::max(0.55, door_width_min_) / grid_res)));
    const int one_sided_gap_max_cells =
        std::max(one_sided_gap_min_cells + 2,
                 static_cast<int>(std::ceil(std::max(1.8, door_width_max_ + 0.3) / grid_res)));
    for (int row = min_row; row <= max_row; ++row) {
      for (int wall_col = 1; wall_col < cols - 1; ++wall_col) {
        if (!wall_occ[cellIndex(row, wall_col)]) continue;

        for (int gap_dir : {-1, 1}) {
          int support_rows = 0;
          int free_rows = 0;
          const int row_window =
              std::min(rows - 1, row + std::max(one_sided_support_rows_min, min_depth_cells / 2));
          int min_gap_cells_seen = one_sided_gap_max_cells;
          for (int rr = row; rr <= row_window; ++rr) {
            if (hasWallNear(rr, wall_col, 1, 1)) ++support_rows;

            int gap_cells = 0;
            for (int step = 1; step <= one_sided_gap_max_cells; ++step) {
              const int cc = wall_col + gap_dir * step;
              if (!inBounds(rr, cc)) break;
              if (grid[cellIndex(rr, cc)].occupied) break;
              ++gap_cells;
            }
            min_gap_cells_seen = std::min(min_gap_cells_seen, gap_cells);
            if (gap_cells >= one_sided_gap_min_cells) ++free_rows;
          }

          const int checked_rows = std::max(1, row_window - row + 1);
          const double free_ratio = static_cast<double>(free_rows) / static_cast<double>(checked_rows);
          if (support_rows < one_sided_support_rows_min || free_ratio < 0.65) continue;

          const int gap_cells = std::min(one_sided_gap_max_cells, std::max(one_sided_gap_min_cells, min_gap_cells_seen));
          const int center_col = wall_col + gap_dir * std::max(2, gap_cells / 2);
          if (!inBounds(row, center_col)) continue;

          const double center_lateral = -lateral_limit + (static_cast<double>(center_col) + 0.5) * grid_res;
          if (std::fabs(center_lateral) > max_door_center_lateral_) continue;

          const int through_row =
              std::min(rows - 1, row + static_cast<int>(std::ceil(door_pass_dist_ / grid_res)));
          if (!pathToThroughClear(row, through_row, center_col)) {
            ++stats.path;
            publishRejectMarker(row, center_col, "path");
            continue;
          }

          geometry_msgs::Point wall_post;
          double wall_span = 0.0;
          int wall_support = 0;
          if (!aggregatePostInfo(row, wall_col, wall_post, wall_span, wall_support)) continue;

          geometry_msgs::Point center_pt = gridToWorld(row, center_col);
          const double alignment = std::cos(wrapAngle(std::atan2(center_pt.y - odom_.pose.pose.position.y,
                                                                 center_pt.x - odom_.pose.pose.position.x) -
                                                      entry_search_ref_yaw_));
          if (alignment < min_door_alignment_) continue;

          const double progress = (static_cast<double>(row) + 0.5) * grid_res;
          if (progress < min_entry_progress_) continue;

          const double width = static_cast<double>(gap_cells) * grid_res;
          const double open_depth = static_cast<double>(row_window - row + 1) * grid_res;
          const double score = 4.2 * progress + 2.4 * open_depth + 1.8 * free_ratio +
                               0.08 * static_cast<double>(wall_support) -
                               1.0 * std::fabs(center_lateral);
          if (!shouldReplaceDoorCandidate(score, progress, best)) continue;

          geometry_msgs::Point inferred_post = gridToWorld(row, wall_col + gap_dir * gap_cells);
          best.score = score;
          best.yaw = entry_search_ref_yaw_;
          best.target_dist = std::max(probe_dist_, progress + door_pass_dist_);
          best.support_count = wall_support;
          best.forward_extent = open_depth;
          best.ref_alignment = alignment;
          best.ref_progress = progress;
          best.axis_support_count = support_rows;
          best.lateral_span = width;
          best.has_goal_point = true;
          best.door_detected = true;
          best.door_width = width;
          best.door_depth = open_depth;
          if (gap_dir > 0) {
            best.left_post = wall_post;
            best.right_post = inferred_post;
          } else {
            best.left_post = inferred_post;
            best.right_post = wall_post;
          }
          best.door_point = center_pt;
          best.goal_point = gridToWorld(through_row, center_col);
        }
      }
    }

    // 2026-07-09: 新增全局地图“多行框洞”检测。
    // 它不再要求某一行必须形成完美的 墙-空-墙 free-run，而是跨多行统计两侧墙段和中间可通行矩形，
    // 适配 RViz 中那种由稀疏雷达体素组成、肉眼明显但单行很破碎的门/通道入口。
    const int max_frame_width_cells =
        std::max(4, static_cast<int>(std::ceil(std::max(door_width_max_, frame_opening_width_min_ + 0.8) / grid_res)));
    const int min_frame_width_cells =
        std::max(2, static_cast<int>(std::floor(std::min(door_width_min_, frame_opening_width_min_) / grid_res)));
    for (int row = min_row; row <= max_row; ++row) {
      for (int seed_col = 1; seed_col < cols - 1; seed_col += 2) {
        const double center_lateral = -lateral_limit + (static_cast<double>(seed_col) + 0.5) * grid_res;
        if (std::fabs(center_lateral) > max_door_center_lateral_) continue;

        int left_col = -1;
        int right_col = -1;
        for (int step = 1; step <= max_frame_width_cells + side_probe_cells; ++step) {
          const int cc = seed_col - step;
          if (cc < 0) break;
          if (wall_occ[cellIndex(row, cc)]) {
            left_col = cc;
            break;
          }
        }
        for (int step = 1; step <= max_frame_width_cells + side_probe_cells; ++step) {
          const int cc = seed_col + step;
          if (cc >= cols) break;
          if (wall_occ[cellIndex(row, cc)]) {
            right_col = cc;
            break;
          }
        }
        if (left_col < 0 || right_col < 0) continue;

        const int gap_cells = right_col - left_col - 1;
        if (gap_cells < min_frame_width_cells || gap_cells > max_frame_width_cells) continue;

        const double width = static_cast<double>(gap_cells) * grid_res;
        const int center_col = (left_col + right_col) / 2;
        const double real_center_lateral = -lateral_limit + (static_cast<double>(center_col) + 0.5) * grid_res;
        if (std::fabs(real_center_lateral) > max_door_center_lateral_) continue;

        const int depth_window = std::max(4, static_cast<int>(std::ceil(std::max(entry_door_depth_min_, frame_opening_depth_min_) / grid_res)));
        const int row_window = std::min(rows - 1, row + std::max(depth_window, min_depth_cells / 2));
        int support_rows = 0;
        int free_like_rows = 0;
        int unknown_hits = 0;
        for (int rr = row; rr <= row_window; ++rr) {
          const bool left_support = hasWallNear(rr, left_col, 1, 2);
          const bool right_support = hasWallNear(rr, right_col, 1, 2);
          if (left_support && right_support) ++support_rows;

          const int blocked_bins = countRawOccupiedInGap(rr, left_col, right_col);
          if (blocked_bins <= max_door_mid_blocked_bins_) ++free_like_rows;
          for (int cc = std::max(0, left_col + 1); cc <= std::min(cols - 1, right_col - 1); ++cc) {
            unknown_hits += unknown_counts[cellIndex(rr, cc)];
          }
        }

        const int checked_rows = std::max(1, row_window - row + 1);
        const double free_ratio = static_cast<double>(free_like_rows) / static_cast<double>(checked_rows);
        if (support_rows < entry_door_support_rows_min_ || free_ratio < min_door_gap_free_ratio_) continue;

        geometry_msgs::Point left_post;
        geometry_msgs::Point right_post;
        double left_vertical_span = 0.0;
        double right_vertical_span = 0.0;
        int left_support = 0;
        int right_support = 0;
        if (!aggregatePostInfo(row, left_col, left_post, left_vertical_span, left_support) ||
            !aggregatePostInfo(row, right_col, right_post, right_vertical_span, right_support)) {
          continue;
        }

        geometry_msgs::Point center_pt = gridToWorld(row, center_col);
        const double alignment = std::cos(wrapAngle(std::atan2(center_pt.y - odom_.pose.pose.position.y,
                                                               center_pt.x - odom_.pose.pose.position.x) -
                                                    entry_search_ref_yaw_));
        if (alignment < min_door_alignment_) continue;

        const double progress = (static_cast<double>(row) + 0.5) * grid_res;
        if (progress < min_entry_progress_) continue;

        const double open_depth = static_cast<double>(row_window - row + 1) * grid_res;
        const double frame_height = std::min(left_vertical_span, right_vertical_span);
        // 2026-07-09: 当前 FUEL 的 /sdf_map/unknown 经常为空；门口是否成立不能被 unknown=0 直接否决。
        // unknown 只在存在有效 unknown 图时作为加分/确认条件，否则由占据墙段和中间可通行框洞决定。
        const bool unknown_ok =
            stats.unknown_window == 0 || frame_unknown_hits_min_ <= 0 || unknown_hits >= frame_unknown_hits_min_;
        const bool is_frame_opening =
            width >= frame_opening_width_min_ &&
            frame_height >= min_door_post_height_ &&
            open_depth >= frame_opening_depth_min_ &&
            unknown_ok;
        const bool is_entry_door =
            open_depth >= entry_door_depth_min_ &&
            support_rows >= entry_door_support_rows_min_ &&
            unknown_ok;
        if (!is_frame_opening && !is_entry_door) continue;

        const double width_score =
            1.0 - std::min(1.0, std::fabs(width - door_prefer_width_) / std::max(0.2, door_prefer_width_));
        const double vertical_score =
            std::min(left_vertical_span, right_vertical_span) / std::max(0.1, min_door_post_height_);
        const double score = 4.0 * progress + 2.2 * open_depth + 1.6 * free_ratio + 1.2 * width_score +
                             0.6 * std::min(1.8, vertical_score) +
                             0.12 * static_cast<double>(support_rows) +
                             (is_frame_opening ? 2.0 : 0.0) -
                             1.4 * std::fabs(real_center_lateral);
        if (!shouldReplaceDoorCandidate(score, progress, best)) continue;

        const int through_row =
            std::min(rows - 1, row + static_cast<int>(std::ceil(door_pass_dist_ / grid_res)));
        if (!pathToThroughClear(row, through_row, center_col)) {
          ++stats.path;
          publishRejectMarker(row, center_col, "path");
          continue;
        }
        best.score = score;
        best.yaw = entry_search_ref_yaw_;
        best.target_dist = std::max(probe_dist_, progress + door_pass_dist_);
        best.support_count = left_support + right_support;
        best.forward_extent = open_depth;
        best.ref_alignment = alignment;
        best.ref_progress = progress;
        best.axis_support_count = support_rows;
        best.lateral_span = width;
        best.has_goal_point = true;
        best.door_detected = true;
        best.door_width = width;
        best.door_depth = open_depth;
        best.left_post = left_post;
        best.right_post = right_post;
        best.door_point = center_pt;
        best.goal_point = gridToWorld(through_row, center_col);
      }
    }

    std::vector<int> component(rows * cols, -1);
    int next_component = 0;
    std::vector<int> queue;
    queue.reserve(rows * cols);

    for (int row = min_row; row <= max_row; ++row) {
      int col = 0;
      while (col < cols) {
        while (col < cols && inflated_occ[cellIndex(row, col)]) ++col;
        if (col >= cols) break;
        const int run_start = col;
        while (col < cols && !inflated_occ[cellIndex(row, col)]) ++col;
        const int run_end = col - 1;
        ++stats.runs;
        const double width = static_cast<double>(run_end - run_start + 1) * grid_res;
        const int seed_col = (run_start + run_end) / 2;
        if (width < door_width_min_ || width > door_width_max_) {
          ++stats.width;
          if (width >= 0.30 && width <= door_width_max_ + 1.00) publishRejectMarker(row, seed_col, "width");
          continue;
        }

        const double center_lateral =
            -lateral_limit + (0.5 * static_cast<double>(run_start + run_end + 1)) * grid_res;
        if (std::fabs(center_lateral) > max_door_center_lateral_) {
          ++stats.lateral;
          publishRejectMarker(row, seed_col, "lateral");
          continue;
        }

        geometry_msgs::Point left_post;
        geometry_msgs::Point right_post;
        double left_vertical_span = 0.0;
        double right_vertical_span = 0.0;
        int left_support = 0;
        int right_support = 0;
        if (!sideWallInfo(row, run_start, -1, left_post, left_vertical_span, left_support) ||
            !sideWallInfo(row, run_end, +1, right_post, right_vertical_span, right_support)) {
          ++stats.side_wall;
          publishRejectMarker(row, seed_col, "side");
          continue;
        }

        const int seed_idx = cellIndex(row, seed_col);
        if (inflated_occ[seed_idx]) continue;

        if (component[seed_idx] < 0) {
          component[seed_idx] = next_component++;
          queue.clear();
          queue.push_back(seed_idx);
          for (size_t qi = 0; qi < queue.size(); ++qi) {
            const int idx = queue[qi];
            const int cr = idx / cols;
            const int cc = idx % cols;
            const int drs[4] = {1, -1, 0, 0};
            const int dcs[4] = {0, 0, 1, -1};
            for (int k = 0; k < 4; ++k) {
              const int nr = cr + drs[k];
              const int nc = cc + dcs[k];
              if (!inBounds(nr, nc)) continue;
              const int nidx = cellIndex(nr, nc);
              if (inflated_occ[nidx] || component[nidx] >= 0) continue;
              component[nidx] = component[idx];
              queue.push_back(nidx);
            }
          }
        }

        const int comp_id = component[seed_idx];
        int min_comp_row = rows;
        int max_comp_row = -1;
        int comp_cells = 0;
        for (int idx = 0; idx < rows * cols; ++idx) {
          if (component[idx] != comp_id) continue;
          ++comp_cells;
          const int cr = idx / cols;
          min_comp_row = std::min(min_comp_row, cr);
          max_comp_row = std::max(max_comp_row, cr);
        }
        if (comp_cells < min_component_cells) {
          ++stats.component;
          publishRejectMarker(row, seed_col, "component");
          continue;
        }

        const double progress = (static_cast<double>(row) + 0.5) * grid_res;
        const double open_depth = static_cast<double>(max_comp_row - row + 1) * grid_res;
        if (progress < min_entry_progress_) {
          ++stats.progress;
          publishRejectMarker(row, seed_col, "progress");
          continue;
        }

        int free_like_rows = 0;
        int support_rows = 0;
        const int row_window = std::min(rows - 1, row + min_depth_cells);
        for (int rr = row; rr <= row_window; ++rr) {
          bool all_free = true;
          for (int cc = run_start; cc <= run_end; ++cc) {
            if (inflated_occ[cellIndex(rr, cc)]) {
              all_free = false;
              break;
            }
          }
          if (all_free) ++free_like_rows;

          bool has_left_support = false;
          bool has_right_support = false;
          for (int sc = std::max(0, run_start - side_probe_cells); sc < run_start; ++sc) {
            const GridCell& cell = grid[cellIndex(rr, sc)];
            // 2026-07-09: support_rows 用于判断两侧是否连续有墙，不再要求单格子自己具备完整高度。
            if (cell.occupied) {
              has_left_support = true;
              break;
            }
          }
          for (int sc = run_end + 1; sc <= std::min(cols - 1, run_end + side_probe_cells); ++sc) {
            const GridCell& cell = grid[cellIndex(rr, sc)];
            if (cell.occupied) {
              has_right_support = true;
              break;
            }
          }
          if (has_left_support && has_right_support) ++support_rows;
        }
        const double free_ratio =
            static_cast<double>(free_like_rows) / static_cast<double>(std::max(1, row_window - row + 1));
        if (free_ratio < min_door_gap_free_ratio_) {
          ++stats.free_ratio;
          publishRejectMarker(row, seed_col, "free");
          continue;
        }

        geometry_msgs::Point center_pt = gridToWorld(row, seed_col);
        geometry_msgs::Point through_pt = gridToWorld(
            std::min(rows - 1, row + static_cast<int>(std::ceil(door_pass_dist_ / grid_res))), seed_col);
        const int through_row =
            std::min(rows - 1, row + static_cast<int>(std::ceil(door_pass_dist_ / grid_res)));
        if (!pathToThroughClear(row, through_row, seed_col)) {
          ++stats.path;
          publishRejectMarker(row, seed_col, "path");
          continue;
        }

        const double alignment = std::cos(wrapAngle(std::atan2(center_pt.y - odom_.pose.pose.position.y,
                                                               center_pt.x - odom_.pose.pose.position.x) -
                                                    entry_search_ref_yaw_));
        if (alignment < min_door_alignment_) {
          ++stats.alignment;
          publishRejectMarker(row, seed_col, "align");
          continue;
        }

        const double width_score =
            1.0 - std::min(1.0, std::fabs(width - door_prefer_width_) / std::max(0.2, door_prefer_width_));
        const double vertical_score =
            std::min(left_vertical_span, right_vertical_span) / std::max(0.1, min_door_post_height_);
        const double center_penalty = std::fabs(center_lateral);
        int unknown_hits = 0;
        for (int rr = row; rr <= std::min(rows - 1, row + min_depth_cells + 3); ++rr) {
          for (int cc = run_start; cc <= run_end; ++cc) {
            unknown_hits += unknown_counts[cellIndex(rr, cc)];
          }
        }
        const double unknown_bias = std::min(1.5, 0.05 * static_cast<double>(unknown_hits));
        const double frame_height = std::min(left_vertical_span, right_vertical_span);
        const bool is_frame_opening =
            width >= frame_opening_width_min_ &&
            frame_height >= frame_opening_height_min_ &&
            open_depth >= frame_opening_depth_min_ &&
            unknown_hits >= frame_unknown_hits_min_;
        const bool is_corridor_door =
            open_depth >= min_door_depth_ && support_rows >= std::max(2, min_depth_cells - 1);
        const bool is_entry_door =
            open_depth >= entry_door_depth_min_ &&
            support_rows >= entry_door_support_rows_min_ &&
            unknown_hits >= entry_door_min_unknown_hits_;
        // 2026-07-09: 门分成三类：
        // 1) frame_opening: 地图上已经形成接近 1m x 1m 的框形开口，优先级最高；
        // 2) corridor_door: 通道内部的标准门，要求纵深长、双侧支撑持续；
        // 3) entry_door: 起飞区出口/窄道入口，只要求近处成形且后方通向未知。
        if (!is_frame_opening && !is_corridor_door && !is_entry_door) {
          ++stats.semantic;
          publishRejectMarker(row, seed_col, "type");
          continue;
        }

        ++stats.accepted;
        const double score = 3.5 * progress + 1.8 * open_depth + 1.2 * free_ratio + 0.8 * width_score +
                             0.5 * std::min(1.5, vertical_score) +
                             0.10 * static_cast<double>(support_rows) +
                             (is_frame_opening ? 1.6 : 0.0) +
                             (is_entry_door && !is_corridor_door ? 0.9 : 0.0) +
                             unknown_bias +
                             0.02 * static_cast<double>(left_support + right_support + comp_cells) -
                             1.2 * center_penalty;
        if (!shouldReplaceDoorCandidate(score, progress, best)) continue;

        best.score = score;
        best.yaw = entry_search_ref_yaw_;
        best.target_dist = std::max(probe_dist_, progress + door_pass_dist_);
        best.support_count = left_support + right_support;
        best.forward_extent = open_depth;
        best.ref_alignment = alignment;
        best.ref_progress = progress;
        best.axis_support_count = comp_cells;
        best.lateral_span = width;
        best.has_goal_point = true;
        best.door_detected = true;
        best.door_width = width;
        best.door_depth = open_depth;
        best.left_post = left_post;
        best.right_post = right_post;
        best.door_point = center_pt;
        best.goal_point = through_pt;
      }
    }

    ROS_WARN_THROTTLE(1.0,
                      "[corridor_search_manager][door_debug] occ=%d unknown=%d runs=%d accept=%d reject:"
                      " width=%d lateral=%d side=%d comp=%d progress=%d free=%d align=%d type=%d path=%d"
                      " portal=%d portal_exit=%d best=%d",
                      stats.occ_window, stats.unknown_window, stats.runs, stats.accepted, stats.width,
                      stats.lateral, stats.side_wall, stats.component, stats.progress, stats.free_ratio,
                      stats.alignment, stats.semantic, stats.path, stats.portal, stats.portal_exit,
                      static_cast<int>(best.door_detected));
    return best;
  }

  CandidateScore evaluateDirection(double yaw, bool cloud_ready) const {
    CandidateScore result;
    result.yaw = yaw;

    const double heading_alignment = std::cos(wrapAngle(yaw - current_yaw_));
    const double ref_alignment = std::cos(wrapAngle(yaw - entry_search_ref_yaw_));
    const double dir_x = std::cos(yaw);
    const double dir_y = std::sin(yaw);

    int support_count = 0;
    double forward_extent = 0.0;
    int axis_support_count = 0;
    double min_lateral = std::numeric_limits<double>::infinity();
    double max_lateral = -std::numeric_limits<double>::infinity();
    double door_forward_sum = 0.0;
    double door_lateral_sum = 0.0;
    int door_vote_count = 0;

    if (cloud_ready) {
      for (const auto& pt : local_cloud_.points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        const double rel_x = pt.x - odom_.pose.pose.position.x;
        const double rel_y = pt.y - odom_.pose.pose.position.y;
        const double rel_z = pt.z - odom_.pose.pose.position.z;
        const double planar_dist = std::sqrt(rel_x * rel_x + rel_y * rel_y);
        if (planar_dist < 0.3 || planar_dist > local_radius_) continue;
        if (std::fabs(rel_z) > height_window_) continue;

        const double forward = rel_x * dir_x + rel_y * dir_y;
        if (forward <= 0.0) continue;

        const double lateral = std::fabs((-dir_y) * rel_x + dir_x * rel_y);
        if (lateral > corridor_half_width_) continue;

        ++support_count;
        forward_extent = std::max(forward_extent, forward);
        min_lateral = std::min(min_lateral, lateral);
        max_lateral = std::max(max_lateral, lateral);
        if (lateral < axis_half_width_) {
          ++axis_support_count;
        }
        if (forward > lookahead_dist_ * 0.8) {
          door_forward_sum += forward;
          door_lateral_sum += ((-dir_y) * rel_x + dir_x * rel_y);
          ++door_vote_count;
        }
      }
    }

    double target_dist = probe_dist_;
    if (support_count >= min_support_points_) {
      target_dist = std::max(probe_dist_, std::min(lookahead_dist_, 0.65 * forward_extent));
    }

    const geometry_msgs::Point candidate_pt = makePoint(target_dist, yaw);
    const double repeat_penalty = computeRepeatPenalty(candidate_pt);
    const double sparse_penalty = support_count >= min_support_points_ ? 0.0 : open_space_penalty_;

    result.support_count = support_count;
    result.forward_extent = forward_extent;
    result.target_dist = target_dist;
    result.ref_alignment = ref_alignment;
    result.ref_progress = target_dist * ref_alignment;
    result.axis_support_count = axis_support_count;
    result.lateral_span =
        support_count > 0 ? std::max(0.0, max_lateral - min_lateral) : std::numeric_limits<double>::infinity();
    result.score = prefer_heading_weight_ * heading_alignment +
                   prefer_progress_weight_ * result.ref_progress +
                   forward_extent_weight_ * forward_extent +
                   axis_support_weight_ * static_cast<double>(axis_support_count) +
                   structure_weight_ * static_cast<double>(support_count) -
                   lateral_span_penalty_weight_ * std::min(result.lateral_span, corridor_half_width_ * 2.0) -
                   repeat_penalty_weight_ * repeat_penalty - sparse_penalty;

    geometry_msgs::Point target_pt = makePoint(target_dist, yaw);
    result.goal_point = target_pt;
    result.has_goal_point = true;

    if (door_vote_count > 0) {
      const double mean_forward = door_forward_sum / door_vote_count;
      const double mean_lateral = door_lateral_sum / door_vote_count;
      geometry_msgs::Point door_pt;
      door_pt.x = odom_.pose.pose.position.x + mean_forward * dir_x + mean_lateral * (-dir_y);
      door_pt.y = odom_.pose.pose.position.y + mean_forward * dir_y + mean_lateral * dir_x;
      door_pt.z = resolveGoalHeightFor(odom_.pose.pose.position.z);
      result.door_point = door_pt;
      result.goal_point.x = odom_.pose.pose.position.x + std::min(mean_forward * door_projection_ratio_, lookahead_dist_) * dir_x;
      result.goal_point.y = odom_.pose.pose.position.y + std::min(mean_forward * door_projection_ratio_, lookahead_dist_) * dir_y;
      result.goal_point.z = door_pt.z;
    } else {
      result.door_point = target_pt;
    }
    return result;
  }

  geometry_msgs::Point makePoint(double target_dist, double yaw) const {
    if (use_fixed_entry_goal_) {
      geometry_msgs::Point pt = entry_goal_;
      pt.z = resolveGoalHeightFor(entry_goal_.z);
      return pt;
    }

    geometry_msgs::Point pt;
    pt.x = odom_.pose.pose.position.x + target_dist * std::cos(yaw);
    pt.y = odom_.pose.pose.position.y + target_dist * std::sin(yaw);
    pt.z = resolveGoalHeightFor(odom_.pose.pose.position.z);
    return pt;
  }

  double resolveGoalHeightFor(double z_hint) const {
    double z = z_hint;
    if (goal_height_mode_ == "fixed") {
      z = cruise_height_;
    }
    z = std::max(min_goal_z_, std::min(max_goal_z_, z));
    return z;
  }

  bool pointInBox(const geometry_msgs::Point& pt, const geometry_msgs::Point& bmin,
                  const geometry_msgs::Point& bmax) const {
    return pt.x >= bmin.x && pt.x <= bmax.x && pt.y >= bmin.y && pt.y <= bmax.y && pt.z >= bmin.z &&
           pt.z <= bmax.z;
  }

  bool hasReachedEntry() const {
    if (entry_locked_ && locked_door_detected_) {
      const double dir_x = std::cos(locked_entry_yaw_);
      const double dir_y = std::sin(locked_entry_yaw_);
      const double rel_x = odom_.pose.pose.position.x - locked_door_point_.x;
      const double rel_y = odom_.pose.pose.position.y - locked_door_point_.y;
      const double cross_progress = rel_x * dir_x + rel_y * dir_y;
      const double lateral = std::fabs((-dir_y) * rel_x + dir_x * rel_y);
      if (cross_progress >= door_cross_trigger_dist_ &&
          lateral <= std::max(1.2, 0.5 * locked_door_width_ + 0.6)) {
        return true;
      }
    }
    if (!entry_locked_ && entry_reach_by_search_region_ &&
        pointInBox(odom_.pose.pose.position, search_region_min_, search_region_max_)) {
      return true;
    }
    // 2026-07-08 19:26: 前置通道阶段不再用宽松的“距离入口点差不多”就放权，必须真的进到搜索区。
    return false;
  }

  void triggerExploration(const ros::Time& now) {
    geometry_msgs::PoseStamped trigger_msg;
    trigger_msg.header.stamp = now;
    trigger_msg.header.frame_id = world_frame_;
    trigger_msg.pose = odom_.pose.pose;
    if (trigger_msg.pose.position.z < min_goal_z_) {
      trigger_msg.pose.position.z = min_goal_z_;
    }
    exploration_trigger_pub_.publish(trigger_msg);
    exploration_triggered_ = true;
    mission_stage_ = HANDOFF_TO_EXPLORATION;
    ROS_WARN("[corridor_search_manager] entry reached, exploration triggered at (%.2f, %.2f, %.2f).",
             trigger_msg.pose.position.x, trigger_msg.pose.position.y, trigger_msg.pose.position.z);
  }

  void publishPositionCommand(const geometry_msgs::PoseStamped& goal, const ros::Time& now) {
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = now;
    cmd.header.frame_id = world_frame_;
    cmd.position = goal.pose.position;
    cmd.velocity = geometry_msgs::Vector3();
    cmd.acceleration = geometry_msgs::Vector3();
    cmd.jerk = geometry_msgs::Vector3();

    const double dx = goal.pose.position.x - odom_.pose.pose.position.x;
    const double dy = goal.pose.position.y - odom_.pose.pose.position.y;
    if (std::hypot(dx, dy) > 0.05) {
      cmd.yaw = std::atan2(dy, dx);
    } else {
      // 2026-07-09: 原地保持目标仍需要带 yaw，用于未识别到门时只转向扫图、不产生 XY 位移。
      const auto& q = goal.pose.orientation;
      const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      cmd.yaw = std::atan2(siny_cosp, cosy_cosp);
    }
    cmd.yaw_dot = 0.0;
    cmd.trajectory_id = static_cast<uint32_t>(trajectory_id_++);
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    pos_cmd_pub_.publish(cmd);
  }

  double computeRepeatPenalty(const geometry_msgs::Point& pt) const {
    double penalty = 0.0;
    for (const auto& prev : goal_history_) {
      const double dist = pointDistance2D(prev.pose.position, pt);
      if (dist < repeat_penalty_radius_) {
        penalty += (repeat_penalty_radius_ - dist) / std::max(0.1, repeat_penalty_radius_);
      }
    }
    return penalty;
  }

  void publishGoal(const CandidateScore& candidate, const ros::Time& now, const std::string& marker_ns) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = now;
    goal.header.frame_id = world_frame_;
    goal.pose.position = candidate.has_goal_point ? candidate.goal_point : makePoint(candidate.target_dist, candidate.yaw);
    goal.pose.orientation.w = std::cos(candidate.yaw * 0.5);
    goal.pose.orientation.z = std::sin(candidate.yaw * 0.5);

    goal_pub_.publish(goal);
    publishMarker(goal, marker_ns, 0.10, 0.90, 0.20);
    publishDoorMarker(candidate, now);

    active_goal_ = goal;
    have_active_goal_ = true;
    goal_history_.push_back(goal);
    while (static_cast<int>(goal_history_.size()) > max_history_size_) {
      goal_history_.pop_front();
    }

    ROS_INFO("[corridor_search_manager] publish goal=(%.2f, %.2f, %.2f) yaw=%.1fdeg support=%d axis=%d extent=%.2f span=%.2f dist=%.2f ref_prog=%.2f stage=%d",
             goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
             candidate.yaw * 180.0 / M_PI, candidate.support_count, candidate.axis_support_count,
             candidate.forward_extent, candidate.lateral_span, candidate.target_dist, candidate.ref_progress,
             int(mission_stage_));
    if (candidate.door_detected) {
      ROS_WARN("[corridor_search_manager] door detected center=(%.2f, %.2f, %.2f) width=%.2f depth=%.2f through=(%.2f, %.2f, %.2f)",
               candidate.door_point.x, candidate.door_point.y, candidate.door_point.z,
               candidate.door_width, candidate.door_depth,
               candidate.goal_point.x, candidate.goal_point.y, candidate.goal_point.z);
    }
  }

  void publishEntryReference() {
    geometry_msgs::PoseStamped entry_pose;
    entry_pose.header.stamp = ros::Time::now();
    entry_pose.header.frame_id = world_frame_;
    // 2026-07-10: entry_pose 是给 RViz/第二架无人机看的入口坐标，应该落在入口边界本身；
    // locked_entry_goal_ 是门后的 through-point，只用于控制当前飞机穿过入口，不能拿来当入口标记。
    if (entry_locked_ && locked_door_detected_) {
      entry_pose.pose.position = locked_door_point_;
    } else if (entry_locked_) {
      entry_pose.pose.position = locked_entry_goal_.pose.position;
    } else {
      entry_pose.pose.position = makePoint(0.0, entry_search_ref_yaw_);
    }

    const double dx = entry_pose.pose.position.x - odom_.pose.pose.position.x;
    const double dy = entry_pose.pose.position.y - odom_.pose.pose.position.y;
    const double yaw = std::atan2(dy, dx);
    entry_pose.pose.orientation.w = std::cos(yaw * 0.5);
    entry_pose.pose.orientation.z = std::sin(yaw * 0.5);

    entry_pose_pub_.publish(entry_pose);
    if (publish_entry_marker_) publishMarker(entry_pose, "entry_reference", 0.15, 0.70, 1.00);

    // 2026-07-10: workspace_lock 的位置是门口边界，朝向是从起飞区指向窄道/作业区的 inside_dir；
    // FUEL 接管后用这个半平面过滤门外 frontier，避免又回起飞区补图。
    if (entry_locked_ && locked_door_detected_) {
      geometry_msgs::PoseStamped lock_pose;
      lock_pose.header = entry_pose.header;
      lock_pose.pose.position = locked_door_point_;
      lock_pose.pose.orientation.w = std::cos(locked_entry_yaw_ * 0.5);
      lock_pose.pose.orientation.z = std::sin(locked_entry_yaw_ * 0.5);
      workspace_lock_pub_.publish(lock_pose);
    }
  }

  void publishDoorMarker(const CandidateScore& candidate, const ros::Time& now) {
    if (!candidate.door_detected) return;

    // 2026-07-09: 用门框线和 through-point 明确标出“门在哪里、门后要穿到哪里”，避免 RViz 里只看到一个模糊色块。
    visualization_msgs::Marker frame;
    frame.header.stamp = now;
    frame.header.frame_id = world_frame_;
    frame.ns = "door_frame";
    frame.id = 10;
    frame.type = visualization_msgs::Marker::LINE_LIST;
    frame.action = visualization_msgs::Marker::ADD;
    frame.scale.x = 0.08;
    frame.color.a = 1.0;
    frame.color.r = 1.0;
    frame.color.g = 0.45;
    frame.color.b = 0.05;

    geometry_msgs::Point left_bottom = candidate.left_post;
    geometry_msgs::Point left_top = candidate.left_post;
    geometry_msgs::Point right_bottom = candidate.right_post;
    geometry_msgs::Point right_top = candidate.right_post;
    left_bottom.z = min_goal_z_;
    right_bottom.z = min_goal_z_;
    left_top.z = std::min(max_goal_z_, candidate.door_point.z + 0.45);
    right_top.z = left_top.z;

    frame.points.push_back(left_bottom);
    frame.points.push_back(left_top);
    frame.points.push_back(right_bottom);
    frame.points.push_back(right_top);
    frame.points.push_back(left_top);
    frame.points.push_back(right_top);
    frame.points.push_back(left_bottom);
    frame.points.push_back(right_bottom);
    marker_pub_.publish(frame);

    visualization_msgs::Marker center = frame;
    center.ns = "door_center";
    center.id = 11;
    center.type = visualization_msgs::Marker::SPHERE;
    center.pose.position = candidate.door_point;
    center.pose.orientation.w = 1.0;
    center.scale.x = 0.20;
    center.scale.y = 0.20;
    center.scale.z = 0.20;
    center.color.r = 1.0;
    center.color.g = 0.9;
    center.color.b = 0.1;
    center.points.clear();
    marker_pub_.publish(center);

    visualization_msgs::Marker through = center;
    through.ns = "door_through";
    through.id = 12;
    through.pose.position = candidate.goal_point;
    through.scale.x = 0.18;
    through.scale.y = 0.18;
    through.scale.z = 0.18;
    through.color.r = 0.10;
    through.color.g = 0.95;
    through.color.b = 0.20;
    marker_pub_.publish(through);

    visualization_msgs::Marker text = center;
    text.ns = "door_label";
    text.id = 13;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.pose.position = candidate.door_point;
    text.pose.position.z = left_top.z + 0.25;
    text.scale.z = 0.28;
    text.color.a = 1.0;
    text.color.r = 1.0;
    text.color.g = 0.95;
    text.color.b = 0.20;
    text.text = "door";
    marker_pub_.publish(text);
  }

  void publishMarker(const geometry_msgs::PoseStamped& goal, const std::string& ns,
                     double r, double g, double b) {
    visualization_msgs::Marker marker;
    marker.header = goal.header;
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = goal.pose;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker_pub_.publish(marker);

    visualization_msgs::Marker text = marker;
    text.id = 1;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.pose.position.z += 0.35;
    text.scale.z = 0.20;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.text = ns + ": (" + std::to_string(goal.pose.position.x) + ", " +
                std::to_string(goal.pose.position.y) + ", " +
                std::to_string(goal.pose.position.z) + ")";
    marker_pub_.publish(text);
  }

private:
  enum MissionStage {
    WAIT_TRIGGER = 0,
    SEARCH_ENTRY = 1,
    APPROACH_ENTRY = 2,
    HANDOFF_TO_EXPLORATION = 3
  };

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber map_occ_sub_;
  ros::Subscriber map_unknown_sub_;
  ros::Subscriber trigger_sub_;
  ros::Subscriber ready_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher entry_pose_pub_;
	  ros::Publisher workspace_lock_pub_;
	  ros::Publisher exploration_trigger_pub_;
	  ros::Publisher pos_cmd_pub_;
	  ros::Publisher marker_pub_;
	  ros::Publisher accumulated_map_pub_;
  ros::Timer timer_;

  nav_msgs::Odometry odom_;
	  pcl::PointCloud<pcl::PointXYZ> local_cloud_;
	  pcl::PointCloud<pcl::PointXYZ> local_occ_map_cloud_;
	  pcl::PointCloud<pcl::PointXYZ> local_unknown_map_cloud_;
	  std::unordered_map<std::int64_t, AccumulatedVoxel> accumulated_occ_voxels_;
  ros::Time cloud_stamp_;
  ros::Time map_occ_stamp_;
  ros::Time map_unknown_stamp_;
  ros::Time map_ready_start_time_;
  geometry_msgs::PoseStamped active_goal_;
  geometry_msgs::PoseStamped locked_entry_goal_;
  std::deque<geometry_msgs::PoseStamped> goal_history_;

  bool have_odom_{false};
  bool have_cloud_{false};
  bool have_occ_map_{false};
  bool have_unknown_map_{false};
  bool have_active_goal_{false};
  bool search_enabled_{false};
  bool hover_ready_{false};
  bool exploration_triggered_{false};
  bool entry_locked_{false};
  bool locked_door_detected_{false};
  bool have_pending_door_{false};
  int fallback_index_{0};
  int pending_door_confirm_count_{0};
  int trajectory_id_{1};
  double current_yaw_{0.0};
  double entry_search_ref_yaw_{0.0};
  double locked_entry_yaw_{0.0};
  double locked_door_width_{0.0};
  MissionStage mission_stage_{WAIT_TRIGGER};

  std::string goal_topic_;
  std::string odom_topic_;
  std::string cloud_topic_;
  std::string map_occ_topic_;
  std::string map_unknown_topic_;
  std::string trigger_topic_;
  std::string world_frame_;
  std::string ready_topic_;
  std::string entry_pose_topic_;
  std::string workspace_lock_topic_;
  std::string exploration_trigger_topic_;
  std::string position_cmd_topic_;
  std::string goal_height_mode_;
  CandidateScore pending_door_candidate_;
  geometry_msgs::Point entry_goal_;
  geometry_msgs::Point locked_door_point_;
  geometry_msgs::Point search_region_min_;
  geometry_msgs::Point search_region_max_;
  double cruise_height_{0.8};
  double min_start_height_{0.35};
  double min_goal_z_{0.5};
  double max_goal_z_{1.2};
  double entry_arrive_dist_{0.65};
  double local_radius_{4.0};
  double height_window_{0.9};
  double corridor_half_width_{1.4};
  double lookahead_dist_{1.8};
  double probe_dist_{0.9};
  double arrive_dist_{0.5};
  double reissue_timeout_{4.0};
  double stale_cloud_timeout_{1.0};
  int min_support_points_{25};
  double repeat_penalty_radius_{1.2};
  int max_history_size_{30};
  double search_period_{0.6};
  double prefer_heading_weight_{1.0};
  double structure_weight_{0.05};
  double repeat_penalty_weight_{1.5};
  double open_space_penalty_{1.0};
  double fallback_sweep_step_deg_{35.0};
  double front_search_half_fov_deg_{55.0};
  double front_search_step_deg_{10.0};
  double min_entry_progress_{0.60};
  double entry_hold_dist_{0.35};
  double prefer_progress_weight_{1.4};
  double axis_half_width_{0.45};
  double lateral_span_penalty_weight_{1.2};
  double axis_support_weight_{0.08};
  double forward_extent_weight_{1.1};
  double door_projection_ratio_{0.85};
  double door_bin_step_deg_{5.0};
  double min_door_depth_{1.4};
  double max_door_side_range_{2.2};
  double door_width_min_{0.7};
  double door_width_max_{2.2};
  double door_prefer_width_{1.2};
  double door_pass_dist_{0.8};
  double door_cross_trigger_dist_{0.25};
  double door_mid_free_margin_{0.55};
  double door_mid_block_margin_{0.20};
  double max_door_center_lateral_{1.4};
  double min_door_alignment_{0.60};
  double min_door_gap_free_ratio_{0.60};
  double min_door_post_height_{0.35};
  double entry_door_depth_min_{0.55};
  double frame_opening_width_min_{1.0};
  double frame_opening_height_min_{1.0};
  double frame_opening_depth_min_{0.45};
  double portal_min_post_height_{0.70};
  double portal_max_depth_mismatch_{0.25};
  double portal_exit_probe_depth_{1.10};
	  double map_ready_hold_sec_{1.8};
	  double door_confirm_pos_tol_{0.35};
	  double door_confirm_width_tol_{0.30};
	  double first_entry_progress_tolerance_{0.35};
	  double map_accum_voxel_res_{0.10};
	  double map_accum_max_use_z_{1.25};
	  double door_path_clearance_{0.25};
  int min_entry_support_points_{18};
  int map_ready_min_points_{120};
  int min_door_side_points_{3};
  int entry_door_support_rows_min_{2};
  int entry_door_min_unknown_hits_{3};
  int frame_unknown_hits_min_{2};
  int portal_max_gap_occupied_{1};
  int portal_min_exit_support_rows_{4};
	  int door_confirm_cycles_{3};
	  int max_door_mid_blocked_bins_{1};
	  int max_door_mid_near_bins_{0};
	  int max_debug_reject_markers_{16};
	  int map_accum_min_hits_{2};
	  int map_accum_max_voxels_{60000};
	  int map_accum_min_neighbors_{2};
	  int map_accum_min_vertical_bins_{2};
  bool wait_for_hover_ready_{true};
  bool use_fixed_entry_goal_{true};
  bool entry_reach_by_search_region_{true};
	  bool publish_entry_marker_{true};
	  bool publish_pre_entry_pos_cmd_{true};
	  bool debug_door_rejects_{true};
	  bool use_internal_accumulated_map_{true};
	  bool prefer_first_entry_{true};
	};

int main(int argc, char** argv) {
  ros::init(argc, argv, "corridor_search_manager");
  ros::NodeHandle nh("~");

  CorridorSearchManager manager;
  manager.init(nh);

  ros::spin();
  return 0;
}
