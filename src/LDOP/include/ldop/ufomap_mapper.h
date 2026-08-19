#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>
#include <ufo/geometry/point.hpp>
#include <ufo/geometry/sphere.hpp>
#include <ufo/map/types.hpp>

namespace ufo {
// 这里只需要在私有类型别名里声明 CloudElement；完整定义放在 .cpp 中引入。
// 直接 include ufo/map/point_cloud.hpp 会触发 UFOMap 安装头的包含顺序问题。
template <class... Types>
struct CloudElement;
}  // namespace ufo

namespace ldopcore {

// UFOMap 模块内部使用的运行时配置：构造阶段由 ROS 参数快照转换而来。
struct UfomapMapperConfig {
  double resolution{0.2};
  int depth_levels{16};
  double min_range{0.5};
  double max_range{10.0};
  double input_max_z{-1.0};
  bool ground_filter_enabled{true};
  int ground_estimation_frames{30};
  double ground_estimation_radius{4.0};
  double ground_estimation_bin_size{0.02};
  int ground_estimation_min_points{100};
  double ground_estimation_candidate_band{0.25};
  double ground_estimation_inlier_threshold{0.04};
  double ground_estimation_max_slope{0.35};
  int warmup_frames{30};
  bool temporal_motion_enabled{true};
  double temporal_match_distance{0.10};
  double temporal_search_radius{0.60};
  double temporal_cluster_radius{0.35};
  int temporal_min_cluster_points{4};
  // 通道动态候选层。它独立于 UFOMap 的静态占据分辨率和历史证据。
  bool corridor_dynamic_enabled{false};
  double corridor_width{1.5};
  double corridor_wall_clearance{0.1};
  // 相对传感器的通道纵向 ROI；只在该有限窗口内生成/接管候选。
  double corridor_roi_min_forward{-0.5};
  double corridor_roi_max_forward{3.0};
  double corridor_roi_min_z{0.15};
  double corridor_roi_max_z{1.5};
  double corridor_detection_voxel{0.1};
  int corridor_history_frames{5};
  int corridor_min_confirm_hits{3};
  double corridor_min_lateral_speed{0.1};
  double corridor_min_lateral_span{0.15};
  double corridor_max_forward_speed{0.8};
  double corridor_max_vertical_speed{0.8};
  double corridor_association_gate{0.45};
  double corridor_track_timeout{0.4};
  int corridor_min_cluster_points{4};
  double corridor_max_cluster_extent{0.6};
  int corridor_max_candidates{1};
  int corridor_reject_candidate_count{3};
  int corridor_min_wall_points{12};
  double corridor_wall_search_forward{2.5};
  double corridor_wall_filter_alpha{0.2};
  double corridor_reactivation_displacement{0.04};
  double corridor_turn_reset_yaw{0.35};
  int insert_hit_depth{0};
  int insert_miss_depth{0};
  int ray_casting_depth{0};
  int num_threads{0};
  bool only_valid{false};
  int inflate_unknown{1};
  bool inflate_unknown_compensation{true};
  bool ray_passthrough_hits{false};
  double inflate_hits_dist{0.4};
  std::string down_sampling_method{"center"};
  bool simple_ray_casting{false};
  double simple_ray_casting_factor{1.0};
  int sliding_window_size{0};
  bool parallel{true};
  bool propagate{true};
  double static_map_visualization_max_z{3.0};
};

struct UfomapMapperParams {
  double input_max_z{-1.0};
  bool ground_filter_enabled{true};
  int ground_estimation_frames{30};
  double ground_estimation_radius{4.0};
  double ground_estimation_bin_size{0.02};
  int ground_estimation_min_points{100};
  double ground_estimation_candidate_band{0.25};
  double ground_estimation_inlier_threshold{0.04};
  double ground_estimation_max_slope{0.35};
  int warmup_frames{30};
  bool temporal_motion_enabled{true};
  double temporal_match_distance{0.10};
  double temporal_search_radius{0.60};
  double temporal_cluster_radius{0.35};
  int temporal_min_cluster_points{4};
  bool corridor_dynamic_enabled{false};
  double corridor_width{1.5};
  double corridor_wall_clearance{0.1};
  double corridor_roi_min_forward{-0.5};
  double corridor_roi_max_forward{3.0};
  double corridor_roi_min_z{0.15};
  double corridor_roi_max_z{1.5};
  double corridor_detection_voxel{0.1};
  int corridor_history_frames{5};
  int corridor_min_confirm_hits{3};
  double corridor_min_lateral_speed{0.1};
  double corridor_min_lateral_span{0.15};
  double corridor_max_forward_speed{0.8};
  double corridor_max_vertical_speed{0.8};
  double corridor_association_gate{0.45};
  double corridor_track_timeout{0.4};
  int corridor_min_cluster_points{4};
  double corridor_max_cluster_extent{0.6};
  int corridor_max_candidates{1};
  int corridor_reject_candidate_count{3};
  int corridor_min_wall_points{12};
  double corridor_wall_search_forward{2.5};
  double corridor_wall_filter_alpha{0.2};
  double corridor_reactivation_displacement{0.04};
  double corridor_turn_reset_yaw{0.35};
  double resolution{0.2};             // 大于0.0，叶子体素尺寸
  int depth_levels{16};               // [2, 20]，由UFOMAP自身限制范围，八叉树层级规模    
  double min_range{0.5};              // 大于等于0.0，积分和查询的最小范围 
  double max_range{10.0};             // 非负时大于min_range，-1时不限制范围，积分和查询的最大范围   
  int insert_hit_depth{0};            // [0, depth_levels]，命中点积分深度，0 表示叶子节点     
  int insert_miss_depth{0};           // [0, depth_levels]，空闲点积分深度，0 表示叶子节点  
  int ray_casting_depth{0};           // [0, depth_levels]，射线穿透查询深度，0 表示叶子节点
  int num_threads{0};                 // 大于等于0，积分线程数，0 表示交给 UFOMap 使用默认线程策略
  bool only_valid{false};             // 是否仅积分有效点（即满足 min_range <= range <= max_range 的点）
  int inflate_unknown{1};             // 大于等于0，按邻域膨胀d_p未知区域以容忍定位误差，单位为一个体素
  bool inflate_unknown_compensation{true};          // 是否补偿未知区域膨胀导致的过度占用风险
  bool ray_passthrough_hits{false};                 // 射线是否穿过命中点；如果为 true，射线将继续穿过命中点并可能在更远处产生额外的空闲更新
  double inflate_hits_dist{0.4};                    // 大于等于0，建议为体素的倍数，沿射线扩展d_s命中区域以容忍测距噪声，天花板/墙顶这种大平面在仿真雷达里容易被斜向射线、体素离散、测距噪声打出“边缘擦过”的情况。
  std::string down_sampling_method{"center"};     // 下采样方法，支持 "none"（不下采样）、"center"（保留体素中心点）、"centroid"（保留体素质心点）和 "uniform"（在体素内随机均匀采样一个点）
  bool simple_ray_casting{false};                   // 是否使用简单射线穿透方法；false为启用简单投射，射线穿透查询将直接将目标深度的体素视为命中，不考虑实际占用概率，会加速积分但降低分类准确率
  double simple_ray_casting_factor{1.0};            // 大于0，投射节点的间隔的系数，以层级为投射基础？以体素尺寸为投射基础？
  int sliding_window_size{0};                       // 大于等于0，滑动窗口大小，0 表示不使用滑动窗口；启用，UFOMap 将仅保留最近 N 帧的积分结果？还是保留最近 N 帧再/一起积分
  bool parallel{true};                              // 是否启用并行处理；如果为 true，UFOMap 将在积分过程中使用多线程，这可能会加速处理但也可能引入线程安全问题或增加资源竞争
  bool propagate{true};                             // 积分后是否马上更新父节点状态（树的状态）；如果为 false，父节点状态将仅在查询时根据子节点动态计算，这可能会加速积分但增加查询时的计算负担
  double static_map_visualization_max_z{3.0};       // 大于0，静态地图的可视化高度
};

struct UfomapRuntimeStats {
  // 已完成 UFOMap 积分的帧数，以及当前地图节点规模。
  std::uint64_t update_count{0};
  std::size_t node_count{0};
  std::uint64_t processed_frame_count{0};
  bool ground_plane_ready{false};
  double ground_plane_z{0.0};
  double ground_plane_slope_x{0.0};
  double ground_plane_slope_y{0.0};
  bool warmup_ready{false};
  std::size_t temporal_motion_point_count{0};
  std::size_t corridor_candidate_point_count{0};
  std::size_t corridor_confirmed_point_count{0};
  bool corridor_wall_valid{false};
};

struct UfomapTimingStats {
  // 单帧 processInputCloud() 整体耗时，单位为毫秒。
  double process_input_cloud_ms{0.0};
  // 点云提取、地图更新与当前帧分类的分阶段耗时，单位为毫秒。
  double extract_finite_points_ms{0.0};
  double update_ufomap_ms{0.0};
  double classify_current_frame_ms{0.0};
};

struct UfomapVoxelCode {
  // raw_code 保留 UFOMap Code::raw()，可用于调试或后续直接复用节点编码。
  ufo::code_t raw_code{0};
  // key_* / depth 用于后续按 UFOMap key 空间生成 6/18/26 邻域。
  ufo::key_t key_x{0};
  ufo::key_t key_y{0};
  ufo::key_t key_z{0};
  ufo::depth_t depth{0};
};

struct UfomapDynamicClusterPoint {
  // 原始动态点用于聚类后计算 AABB，避免体素中心放大目标框。
  ufo::Point point{};
  UfomapVoxelCode voxel_code;
  // 通道候选使用独立 0.1m 检测网格；普通 LDOP 点仍使用 UFOMap key。
  UfomapVoxelCode detector_voxel_code;
  bool use_detector_voxel{false};
};

struct UfomapClassificationResult {
  // static_cloud_msg / dynamic_cloud_msg 的语义由调用方定义，
  // 当前在 LDOP 主流程中分别用于“当前帧静态点云 / 当前帧动态点云”。
  sensor_msgs::PointCloud2 static_cloud_msg;
  sensor_msgs::PointCloud2 dynamic_cloud_msg;
  // Current-frame static points for transient voxel visualization; not persistent UFOMap data.
  std::vector<ufo::Point> static_points;
  // Current-frame dynamic points after map and temporal motion candidates are merged.
  std::vector<ufo::Point> dynamic_points;
  // 当前帧动态点的结构化聚类输入；不改变 dynamic_cloud_msg 的 ROS 输出语义。
  std::vector<UfomapDynamicClusterPoint> dynamic_cluster_points;
  // Indices refer to the filtered input cloud passed to classifyPoints().
  std::vector<std::size_t> dynamic_indices;
  std::size_t static_point_count{0};
  std::size_t dynamic_point_count{0};
  std::size_t input_point_count{0};
};

struct UfomapFrameResult {
  // 当前阶段 static_map_cloud_msg / dynamic_cloud_msg 分别用于发布当前帧静态/动态点云。
  sensor_msgs::PointCloud2 static_map_cloud_msg;
  sensor_msgs::PointCloud2 dynamic_cloud_msg;
  UfomapClassificationResult classification;
  // 按深度分组的静态体素 CUBE_LIST，可直接给 RViz MarkerArray 显示。
  visualization_msgs::MarkerArray static_map_markers;
  UfomapRuntimeStats runtime_stats;
  UfomapTimingStats timing;
};

enum class UfomapOccupancyState : std::uint8_t {
  Unknown = 0,
  Free = 1,
  Occupied = 2,
  OutOfMap = 3,
};

struct UfomapNodeQueryResult {
  // state 是 planner-facing 的主语义；下面的 bool 字段保留给现有调用点做低成本迁移。
  UfomapOccupancyState state{UfomapOccupancyState::Unknown};
  bool exists{false};
  bool occupied{false};
  bool free{false};
  bool unknown{true};
  bool out_of_map{false};
  bool seen_free{false};
  ufo::Point center{};
  ufo::depth_t depth{0};
  double voxel_size{0.0};
};

struct UfomapCandidateQuery {
  // depth=0 保持 UFOMap Leaf(0) 的默认语义。
  ufo::depth_t depth{0};
  std::uint32_t min_hits{1};
  std::uint32_t label{0};
  bool require_seen_free{true};
  std::size_t max_results{0};
  std::vector<ufo::Sphere> intersects;
};

struct UfomapCandidateNode {
  ufo::Point center{};
  ufo::depth_t depth{0};
  std::uint32_t hits{0};
  std::uint32_t label{0};
  bool seen_free{false};
};

struct UfomapLocalOccupiedQuery {
  // depth=0 保持 UFOMap 叶子查询语义；P3 只读取静态占据证据，不混入动态 label 语义。
  ufo::depth_t depth{0};
  std::uint32_t min_hits{1};
  std::size_t max_results{0};
  std::vector<ufo::Sphere> intersects;
};

struct UfomapLocalOccupiedNode {
  ufo::Point center{};
  ufo::depth_t depth{0};
  double voxel_size{0.0};
  std::uint32_t hits{0};
};

class UfomapMapper {
 public:
  // 独立负责处理输入点云、更新 UFOMap，并输出动静态点云与静态地图云。
  UfomapMapper(ros::NodeHandle& nh,
               ros::NodeHandle& pnh,
               bool verbose);
  ~UfomapMapper();

  void reserveMap(std::size_t node_capacity);
  void clearMap();
  void propagateModified();
  bool seenFree(const ufo::Point& point) const;
  UfomapNodeQueryResult queryNode(const ufo::Point& point, ufo::depth_t depth = 0) const;
  void setNodeLabel(const ufo::Point& point, std::uint32_t label, bool propagate = true);
  std::uint32_t nodeLabel(const ufo::Point& point) const;
  void clearAllLabels();

  // 暴露 UFOMap 中 seenFree / label 等基础查询能力，供上层构建更高层逻辑。
  std::vector<UfomapCandidateNode> queryDynamicCandidates(
      const UfomapCandidateQuery& query) const;
  // 中性静态占据邻域查询：P3 用它查询预测点膨胀半径内的 occupied/near obstacle。
  std::vector<UfomapLocalOccupiedNode> queryLocalOccupied(
      const UfomapLocalOccupiedQuery& query) const;
  // 最近 occupied 查询只返回静态占据证据；动态风险仍由预测层单独计算，避免污染静态地图语义。
  std::optional<UfomapLocalOccupiedNode> queryNearestOccupied(
      const ufo::Point& center,
      double radius,
      ufo::depth_t depth = 0,
      std::uint32_t min_hits = 1U) const;
  UfomapFrameResult processInputCloud(const sensor_msgs::PointCloud2& cloud_msg,
                                      const nav_msgs::Odometry& odom_msg);

 private:
  struct UfomapRuntime;
  using UfomapPointCloud = std::vector<ufo::CloudElement<ufo::Point>>;

  struct GroundPlaneModel {
    double slope_x{0.0};
    double slope_y{0.0};
    double intercept{0.0};

    double heightAt(const double x, const double y) const {
      return slope_x * x + slope_y * y + intercept;
    }
  };

  void loadParameters();
  void configureUfomap();
  // 调用方需先持有地图读锁或写锁；当前实现中 runtime_ 在对象存活期内应始终有效。
  UfomapRuntime& runtimeLocked();
  const UfomapRuntime& runtimeLocked() const;
  std::optional<GroundPlaneModel> estimateGroundPlane(
      const UfomapPointCloud& points,
      const ufo::Point& sensor_origin) const;
  void updateGroundPlane(const UfomapPointCloud& points, const ufo::Point& sensor_origin);
  UfomapPointCloud filterGroundPoints(
      const UfomapPointCloud& points,
      std::vector<std::size_t>* retained_source_indices = nullptr) const;
  std::vector<std::size_t> detectTemporalMotion(
      const UfomapPointCloud& points,
      const std::vector<bool>* corridor_handled_indices = nullptr);
  struct CorridorCandidateResult {
    std::vector<bool> handled_indices;
    std::vector<bool> dynamic_indices;
    // 候选过多时不发布为动态，但也不应立即写入静态地图。
    std::vector<bool> holdout_indices;
    std::size_t candidate_point_count{0U};
    std::size_t confirmed_point_count{0U};
    bool wall_valid{false};
  };

  struct CorridorTrackSample {
    ros::Time stamp;
    ufo::Point center{};
  };

  struct CorridorTrack {
    std::uint32_t id{0U};
    ufo::Point center{};
    ufo::Point velocity{};
    ufo::Point size{0.2F, 0.2F, 0.2F};
    std::deque<CorridorTrackSample> history;
    std::size_t hits{0U};
    std::size_t missed_frames{0U};
    // 最近窗口内的方向证据；0 表示该帧速度低于阈值。
    std::deque<int> lateral_direction_history;
    bool confirmed{false};
    bool released_static{false};
    ros::Time last_seen;
  };

  CorridorCandidateResult detectCorridorCandidates(
      const UfomapPointCloud& points,
      const ufo::Point& sensor_origin,
      const nav_msgs::Odometry& odom_msg,
      const ros::Time& stamp);
  void resetCorridorTracks();
  UfomapVoxelCode makeDetectorVoxelCode(const ufo::Point& point) const;
  void updatePreviousFrameSnapshot(const UfomapPointCloud& points);
  UfomapClassificationResult classifyPoints(const std_msgs::Header& header,
                                            const UfomapPointCloud& points) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  bool verbose_;
  UfomapMapperParams params_;
  UfomapMapperConfig config_;

  std::atomic<std::uint64_t> ufomap_update_count_;
  // 地图读多写少，使用读写锁避免多个查询路径彼此串行化。
  mutable std::shared_mutex map_mutex_;

  std::unique_ptr<UfomapRuntime> runtime_;
  std::uint64_t processed_frame_count_{0U};
  std::optional<GroundPlaneModel> ground_plane_;
  std::vector<GroundPlaneModel> ground_plane_samples_;
  bool ground_plane_locked_{false};
  std::vector<ufo::Point> previous_frame_points_;
  std::vector<CorridorTrack> corridor_tracks_;
  std::uint32_t next_corridor_track_id_{1U};
  bool corridor_wall_valid_{false};
  bool corridor_wall_measured_{false};
  double corridor_left_wall_{-0.75};
  double corridor_right_wall_{0.75};
  double corridor_forward_x_{1.0};
  double corridor_forward_y_{0.0};
  bool corridor_last_yaw_valid_{false};
  double corridor_last_yaw_{0.0};
};

}  // namespace ldopcore
