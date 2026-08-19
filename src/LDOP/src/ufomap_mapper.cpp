#include <ldop/ufomap_mapper.h>
#include <ldop/utils.h>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <ufo/map/integration/integration.hpp>
#include <ufo/map/integration/integration_parameters.hpp>
#include <ufo/map/key.hpp>
#include <ufo/map/node.hpp>
#include <ufo/map/point.hpp>
#include <ufo/map/point_cloud.hpp>
#include <ufo/map/types.hpp>
#include <ufo/map/ufomap.hpp>

namespace ldopcore {

struct UfomapMapper::UfomapRuntime {
  using MapType =
      ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION | ufo::MapType::LABEL>;

  explicit UfomapRuntime(const float resolution, const ufo::depth_t depth_levels)
      : map(resolution, depth_levels) {}

  ufo::IntegrationParams integration_params;
  MapType map;
};

namespace {

using DufomapMapType =
    ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION | ufo::MapType::LABEL>;

struct ExtractFinitePointsResult {
  ufo::PointCloud points;
  bool has_invalid_points{false};
};

struct StaticMapVisualizationSnapshot {
  // 只保留 RViz CUBE_LIST 需要的最小数据，缩短 UFOMap 读锁持有时间。
  std::vector<std::vector<geometry_msgs::Point>> points_by_depth;
  std::vector<double> voxel_sizes_by_depth;
};

struct TemporalGridKey {
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const TemporalGridKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct TemporalGridKeyHash {
  std::size_t operator()(const TemporalGridKey& key) const {
    std::size_t hash = std::hash<int>{}(key.x);
    hash ^= std::hash<int>{}(key.y) + static_cast<std::size_t>(0x9e3779b9U) +
            (hash << 6U) + (hash >> 2U);
    hash ^= std::hash<int>{}(key.z) + static_cast<std::size_t>(0x9e3779b9U) +
            (hash << 6U) + (hash >> 2U);
    return hash;
  }
};

TemporalGridKey makeTemporalGridKey(const ufo::Point& point, const double cell_size) {
  return TemporalGridKey{
      static_cast<int>(std::floor(static_cast<double>(point.x) / cell_size)),
      static_cast<int>(std::floor(static_cast<double>(point.y) / cell_size)),
      static_cast<int>(std::floor(static_cast<double>(point.z) / cell_size))};
}

double medianValue(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2U == 1U) {
    return *middle;
  }

  const auto lower_middle = std::max_element(values.begin(), middle);
  return (*lower_middle + *middle) * 0.5;
}

double dotHorizontal(const ufo::Point& point, const double axis_x, const double axis_y) {
  return static_cast<double>(point.x) * axis_x + static_cast<double>(point.y) * axis_y;
}

double wrappedAngleDifference(const double lhs, const double rhs) {
  return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

sensor_msgs::PointCloud2 buildCloudFromPoints(const std_msgs::Header& header,
                                              const ufo::PointCloud& points) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header = header;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const auto& point : points) {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  cloud.is_dense = true;
  return cloud;
}

StaticMapVisualizationSnapshot captureStaticMapVisualizationSnapshot(
    const DufomapMapType& map,
    const std::vector<ufo::Point>& static_points,
    const double max_visualization_z) {
  const int max_depth = static_cast<int>(map.rootDepth());
  StaticMapVisualizationSnapshot snapshot;
  snapshot.points_by_depth.resize(static_cast<std::size_t>(max_depth) + 1U);
  snapshot.voxel_sizes_by_depth.resize(static_cast<std::size_t>(max_depth) + 1U);
  for (int depth = 0; depth <= max_depth; ++depth) {
    snapshot.voxel_sizes_by_depth[static_cast<std::size_t>(depth)] =
        map.size(static_cast<ufo::depth_t>(depth));
  }

  std::vector<std::unordered_set<ufo::code_t>> seen_codes_by_depth(
      snapshot.points_by_depth.size());
  for (const auto& point : static_points) {
    if (max_visualization_z > 0.0 && point.z > max_visualization_z) {
      continue;
    }

    const auto code = map.toCodeChecked(point, 0);
    if (!code.has_value()) {
      continue;
    }

    const ufo::Key key = *code;
    const int depth = static_cast<int>(key.depth());
    if (depth < 0 || depth > max_depth) {
      continue;
    }

    if (!seen_codes_by_depth[static_cast<std::size_t>(depth)].insert(code->raw()).second) {
      continue;
    }

    geometry_msgs::Point center;
    const auto node_center = map.toCoord(*code);
    if (max_visualization_z > 0.0 && node_center.z > max_visualization_z) {
      continue;
    }
    center.x = node_center.x;
    center.y = node_center.y;
    center.z = node_center.z;
    snapshot.points_by_depth[static_cast<std::size_t>(depth)].push_back(center);
  }

  return snapshot;
}

visualization_msgs::MarkerArray buildStaticMapVisualization(
    const std_msgs::Header& header,
    StaticMapVisualizationSnapshot snapshot) {
  visualization_msgs::MarkerArray static_map_markers;
  visualization_msgs::Marker clear_marker;
  clear_marker.header = header;
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  static_map_markers.markers.push_back(std::move(clear_marker));

  std::vector<visualization_msgs::Marker> markers_by_depth(snapshot.points_by_depth.size());
  for (std::size_t depth = 0; depth < markers_by_depth.size(); ++depth) {
    auto& marker = markers_by_depth[depth];
    marker.header = header;
    marker.ns = "static_map_occupied";
    marker.id = static_cast<int>(depth);
    marker.action = visualization_msgs::Marker::DELETE;

    if (snapshot.points_by_depth[depth].empty()) {
      continue;
    }

    // ns/id 按 depth 固定，RViz 会用本帧内容覆盖上一帧同一 marker。
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = snapshot.voxel_sizes_by_depth[depth];
    marker.scale.y = marker.scale.x;
    marker.scale.z = marker.scale.x;
    marker.color.r = 0.10f;
    marker.color.g = 0.85f;
    marker.color.b = 0.25f;
    marker.color.a = 1.0f;
    marker.points = std::move(snapshot.points_by_depth[depth]);
  }

  static_map_markers.markers.reserve(markers_by_depth.size() + 1U);
  for (auto& marker : markers_by_depth) {
    static_map_markers.markers.push_back(std::move(marker));
  }

  return static_map_markers;
}

ufo::DownSamplingMethod toUfoDownSamplingMethod(const std::string& method) {
  if (method == "none") {
    return ufo::DownSamplingMethod::NONE;
  }
  if (method == "centroid") {
    return ufo::DownSamplingMethod::CENTROID;
  }
  if (method == "uniform") {
    return ufo::DownSamplingMethod::UNIFORM;
  }
  return ufo::DownSamplingMethod::CENTER;
}

ufo::depth_t clampQueryDepth(const DufomapMapType& map, const ufo::depth_t depth) {
  return std::min(depth, map.rootDepth());
}

UfomapOccupancyState classifyPlannerOccupancyState(const DufomapMapType& map,
                                                   const ufo::Node node) {
  // 当前 DUFOMap 运行时没有使用概率占据层；静态占据来自 reflection hits，
  // planner-facing free 只能由 SEEN_FREE 的射线穿越证据表达，且必须让 hit 优先生效。
  if (map.hits(node) > 0) {
    return UfomapOccupancyState::Occupied;
  }
  if (map.seenFree(node)) {
    return UfomapOccupancyState::Free;
  }
  return UfomapOccupancyState::Unknown;
}

void applyOccupancyStateFlags(const UfomapOccupancyState state,
                              UfomapNodeQueryResult& result) {
  result.state = state;
  result.occupied = state == UfomapOccupancyState::Occupied;
  result.free = state == UfomapOccupancyState::Free;
  result.unknown = state == UfomapOccupancyState::Unknown;
  result.out_of_map = state == UfomapOccupancyState::OutOfMap;
}

UfomapMapperConfig buildUfomapConfig(const UfomapMapperParams& params) {
  const UfomapMapperParams defaults;
  UfomapMapperConfig config;

  config.resolution = params.resolution > 0.0 ? params.resolution : defaults.resolution;
  config.depth_levels = params.depth_levels;
  // UFOMap 的 depth 必须落在 MapType 支持范围内，插入/射线 depth 也不能超过实际树深度。
  const int min_depth_levels = static_cast<int>(DufomapMapType::minDepthLevels());
  const int max_depth_levels = static_cast<int>(DufomapMapType::maxDepthLevels());
  config.depth_levels = std::clamp(config.depth_levels, min_depth_levels, max_depth_levels);
  const int root_depth = config.depth_levels - 1;
  config.insert_hit_depth = std::clamp(params.insert_hit_depth, 0, root_depth);
  config.insert_miss_depth = std::clamp(params.insert_miss_depth, 0, root_depth);
  config.ray_casting_depth = std::clamp(params.ray_casting_depth, 0, root_depth);
  config.min_range = std::max(0.0, params.min_range);
  if (params.max_range == -1.0) {
    config.max_range = params.max_range;
  } else if (params.max_range >= 0.0 && params.max_range > config.min_range) {
    config.max_range = params.max_range;
  } else {
    // max_range 依赖已收束的 min_range；默认最大距离仍越界时退回 -1 的无上限语义。
    config.max_range = defaults.max_range > config.min_range ? defaults.max_range : -1.0;
  }
  config.input_max_z = params.input_max_z > 0.0 ? params.input_max_z : defaults.input_max_z;
  config.ground_filter_enabled = params.ground_filter_enabled;
  config.ground_estimation_frames = std::max(1, params.ground_estimation_frames);
  config.ground_estimation_radius = params.ground_estimation_radius > 0.0
      ? params.ground_estimation_radius : defaults.ground_estimation_radius;
  config.ground_estimation_bin_size = params.ground_estimation_bin_size > 0.0
      ? params.ground_estimation_bin_size : defaults.ground_estimation_bin_size;
  config.ground_estimation_min_points = std::max(1, params.ground_estimation_min_points);
  config.ground_estimation_candidate_band = params.ground_estimation_candidate_band > 0.0
      ? params.ground_estimation_candidate_band : defaults.ground_estimation_candidate_band;
  config.ground_estimation_inlier_threshold = params.ground_estimation_inlier_threshold > 0.0
      ? params.ground_estimation_inlier_threshold : defaults.ground_estimation_inlier_threshold;
  config.ground_estimation_max_slope = params.ground_estimation_max_slope > 0.0
      ? params.ground_estimation_max_slope : defaults.ground_estimation_max_slope;
  config.warmup_frames = std::max(0, params.warmup_frames);
  config.temporal_motion_enabled = params.temporal_motion_enabled;
  config.temporal_match_distance = params.temporal_match_distance > 0.0
      ? params.temporal_match_distance : defaults.temporal_match_distance;
  const double minimum_search_radius = config.temporal_match_distance * 1.5;
  config.temporal_search_radius = params.temporal_search_radius > minimum_search_radius
      ? params.temporal_search_radius
      : std::max(defaults.temporal_search_radius, minimum_search_radius);
  config.temporal_cluster_radius = params.temporal_cluster_radius > 0.0
      ? params.temporal_cluster_radius : defaults.temporal_cluster_radius;
  config.temporal_min_cluster_points = std::max(1, params.temporal_min_cluster_points);
  config.corridor_dynamic_enabled = params.corridor_dynamic_enabled;
  config.corridor_width = params.corridor_width > 0.0
      ? params.corridor_width : defaults.corridor_width;
  config.corridor_wall_clearance = std::clamp(
      params.corridor_wall_clearance, 0.0, 0.45 * config.corridor_width);
  config.corridor_roi_min_forward = std::isfinite(params.corridor_roi_min_forward)
      ? params.corridor_roi_min_forward : defaults.corridor_roi_min_forward;
  config.corridor_roi_max_forward =
      std::isfinite(params.corridor_roi_max_forward) &&
              params.corridor_roi_max_forward > config.corridor_roi_min_forward
          ? params.corridor_roi_max_forward
          : defaults.corridor_roi_max_forward;
  if (config.corridor_roi_max_forward <= config.corridor_roi_min_forward) {
    config.corridor_roi_min_forward = defaults.corridor_roi_min_forward;
    config.corridor_roi_max_forward = defaults.corridor_roi_max_forward;
  }
  config.corridor_roi_min_z = params.corridor_roi_min_z;
  config.corridor_roi_max_z = params.corridor_roi_max_z > params.corridor_roi_min_z
      ? params.corridor_roi_max_z : defaults.corridor_roi_max_z;
  config.corridor_detection_voxel = params.corridor_detection_voxel > 0.0
      ? params.corridor_detection_voxel : defaults.corridor_detection_voxel;
  config.corridor_history_frames = std::max(2, params.corridor_history_frames);
  config.corridor_min_confirm_hits = std::clamp(
      params.corridor_min_confirm_hits, 2, config.corridor_history_frames);
  config.corridor_min_publish_hits = std::clamp(
      params.corridor_min_publish_hits, 1, config.corridor_history_frames);
  config.corridor_min_lateral_speed = std::max(0.0, params.corridor_min_lateral_speed);
  config.corridor_min_lateral_span = std::max(0.0, params.corridor_min_lateral_span);
  config.corridor_publish_unknown_as_dynamic = params.corridor_publish_unknown_as_dynamic;
  config.corridor_static_confirm_frames = std::clamp(
      params.corridor_static_confirm_frames, 2, config.corridor_history_frames);
  config.corridor_confirmed_static_confirm_frames = std::max(
      config.corridor_static_confirm_frames,
      params.corridor_confirmed_static_confirm_frames);
  config.corridor_static_lateral_speed = std::max(
      0.0, params.corridor_static_lateral_speed);
  config.corridor_forward_alignment_cos = std::clamp(
      params.corridor_forward_alignment_cos, 0.0, 1.0);
  config.corridor_max_missed_frames = std::max(0, params.corridor_max_missed_frames);
  config.corridor_internal_max_missed_frames = std::max(
      config.corridor_max_missed_frames, params.corridor_internal_max_missed_frames);
  config.corridor_internal_track_timeout = params.corridor_internal_track_timeout > 0.0
      ? params.corridor_internal_track_timeout : defaults.corridor_internal_track_timeout;
  config.corridor_max_forward_speed = std::max(0.0, params.corridor_max_forward_speed);
  config.corridor_max_vertical_speed = std::max(0.0, params.corridor_max_vertical_speed);
  config.corridor_association_gate = params.corridor_association_gate > 0.0
      ? params.corridor_association_gate : defaults.corridor_association_gate;
  config.corridor_track_timeout = params.corridor_track_timeout > 0.0
      ? params.corridor_track_timeout : defaults.corridor_track_timeout;
  config.corridor_min_cluster_points = std::max(2, params.corridor_min_cluster_points);
  config.corridor_min_confirm_extent = params.corridor_min_confirm_extent > 0.0
      ? params.corridor_min_confirm_extent : defaults.corridor_min_confirm_extent;
  config.corridor_min_confirm_second_extent =
      params.corridor_min_confirm_second_extent > 0.0
          ? params.corridor_min_confirm_second_extent
          : defaults.corridor_min_confirm_second_extent;
  config.corridor_min_confirm_extent_frames = std::clamp(
      params.corridor_min_confirm_extent_frames, 2, config.corridor_history_frames);
  config.corridor_max_cluster_extent = params.corridor_max_cluster_extent > 0.0
      ? params.corridor_max_cluster_extent : defaults.corridor_max_cluster_extent;
  config.corridor_oversized_split_enabled = params.corridor_oversized_split_enabled;
  config.corridor_split_min_cluster_points = std::max(
      2, params.corridor_split_min_cluster_points);
  config.corridor_split_max_subclusters = std::max(1, params.corridor_split_max_subclusters);
  config.corridor_max_candidates = std::max(1, params.corridor_max_candidates);
  config.corridor_reject_candidate_count = std::max(
      config.corridor_max_candidates, params.corridor_reject_candidate_count);
  config.corridor_min_wall_points = std::max(2, params.corridor_min_wall_points);
  config.corridor_wall_search_forward = params.corridor_wall_search_forward > 0.0
      ? params.corridor_wall_search_forward : defaults.corridor_wall_search_forward;
  config.corridor_wall_filter_alpha = std::clamp(
      params.corridor_wall_filter_alpha, 0.01, 1.0);
  config.corridor_reactivation_displacement = std::max(
      0.0, params.corridor_reactivation_displacement);
  config.corridor_turn_reset_yaw = std::max(0.0, params.corridor_turn_reset_yaw);
  config.num_threads = std::max(0, params.num_threads);
  config.only_valid = params.only_valid;
  config.inflate_unknown = std::max(0, params.inflate_unknown);
  config.inflate_unknown_compensation = params.inflate_unknown_compensation;
  config.ray_passthrough_hits = params.ray_passthrough_hits;
  config.inflate_hits_dist = std::max(0.0, params.inflate_hits_dist);
  if (params.down_sampling_method == "none" || params.down_sampling_method == "center" ||
      params.down_sampling_method == "centroid" || params.down_sampling_method == "uniform") {
    config.down_sampling_method = params.down_sampling_method;
  } else {
    config.down_sampling_method = defaults.down_sampling_method;
  }
  config.simple_ray_casting = params.simple_ray_casting;
  config.simple_ray_casting_factor = params.simple_ray_casting_factor > 0.0
      ? params.simple_ray_casting_factor : defaults.simple_ray_casting_factor;
  config.sliding_window_size = std::max(0, params.sliding_window_size);
  config.parallel = params.parallel;
  config.propagate = params.propagate;
  config.static_map_visualization_max_z = params.static_map_visualization_max_z > 0.0
      ? params.static_map_visualization_max_z : defaults.static_map_visualization_max_z;
  return config;
}

template <class Predicate>
void appendCandidateNodes(const DufomapMapType& map,
                          Predicate&& predicate,
                          const std::size_t max_results,
                          std::vector<UfomapCandidateNode>& candidates) {
  for (const auto node : map.query(std::forward<Predicate>(predicate))) {
    UfomapCandidateNode candidate;
    candidate.center = map.center(node);
    candidate.depth = node.depth();
    candidate.hits = static_cast<std::uint32_t>(map.hits(node));
    candidate.label = static_cast<std::uint32_t>(map.label(node.index()));
    candidate.seen_free = map.seenFree(node);
    candidates.push_back(candidate);
    if (max_results > 0U && candidates.size() >= max_results) {
      return;
    }
  }
}

template <class Predicate>
void appendLocalOccupiedNodes(const DufomapMapType& map,
                              Predicate&& predicate,
                              const std::size_t max_results,
                              std::vector<UfomapLocalOccupiedNode>& nodes) {
  for (const auto node : map.query(std::forward<Predicate>(predicate))) {
    UfomapLocalOccupiedNode occupied_node;
    occupied_node.center = map.center(node);
    occupied_node.depth = node.depth();
    occupied_node.voxel_size = map.size(node.depth());
    occupied_node.hits = static_cast<std::uint32_t>(map.hits(node));
    nodes.push_back(occupied_node);
    if (max_results > 0U && nodes.size() >= max_results) {
      return;
    }
  }
}

ExtractFinitePointsResult extractFinitePoints(const sensor_msgs::PointCloud2& cloud_msg) {
  ExtractFinitePointsResult result;
  result.points.reserve(static_cast<std::size_t>(cloud_msg.width) *
                        static_cast<std::size_t>(cloud_msg.height));
  if (cloud_msg.data.empty()) {
    return result;
  }

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float point_x = *iter_x;
      const float point_y = *iter_y;
      const float point_z = *iter_z;

      if (!std::isfinite(point_x) || !std::isfinite(point_y) || !std::isfinite(point_z)) {
        result.has_invalid_points = true;
        continue;
      }

      result.points.emplace_back(ufo::Point(point_x, point_y, point_z));
    }
  } catch (const std::runtime_error& error) {
    ROS_WARN_STREAM_THROTTLE(1.0,
                             "failed to access xyz fields from cloud: " << error.what());
    return ExtractFinitePointsResult{};
  }

  return result;
}
}  // namespace

UfomapMapper::UfomapMapper(ros::NodeHandle& nh,
                           ros::NodeHandle& pnh,
                           const bool verbose)
  : nh_(nh),
      pnh_(pnh),
      verbose_(verbose),
      params_(),
      config_(),
      ufomap_update_count_(0U),
      runtime_(nullptr) {
  loadParameters();
  configureUfomap();

  ROS_INFO_STREAM("UFOMap module is ready. resolution: " << config_.resolution
                  << ", depth levels: " << config_.depth_levels
                  << ", min range: " << config_.min_range
                  << ", max range: " << config_.max_range
                  << ", input max z: " << config_.input_max_z
                  << ", ground filter: " << (config_.ground_filter_enabled ? "on" : "off")
                  << " (keep points above estimated ground plane)"
                  << ", warmup frames: " << config_.warmup_frames
                  << ", temporal motion: "
                  << (config_.temporal_motion_enabled ? "on" : "off"));
}

UfomapMapper::~UfomapMapper() = default;

void UfomapMapper::loadParameters() {
  // 默认值只从参数快照结构体取，避免读参处和结构体初值各维护一份数字。
  const UfomapMapperParams defaults;
  pnh_.param("ufomap_resolution", params_.resolution, defaults.resolution);
  pnh_.param("ufomap_depth_levels", params_.depth_levels, defaults.depth_levels);
  pnh_.param("ufomap_min_range", params_.min_range, defaults.min_range);
  pnh_.param("ufomap_max_range", params_.max_range, defaults.max_range);
  pnh_.param("ufomap_input_max_z", params_.input_max_z, defaults.input_max_z);
  pnh_.param("ufomap_ground_filter_enabled", params_.ground_filter_enabled,
             defaults.ground_filter_enabled);
  pnh_.param("ufomap_ground_estimation_frames", params_.ground_estimation_frames,
             defaults.ground_estimation_frames);
  pnh_.param("ufomap_ground_estimation_radius", params_.ground_estimation_radius,
             defaults.ground_estimation_radius);
  pnh_.param("ufomap_ground_estimation_bin_size", params_.ground_estimation_bin_size,
             defaults.ground_estimation_bin_size);
  pnh_.param("ufomap_ground_estimation_min_points", params_.ground_estimation_min_points,
             defaults.ground_estimation_min_points);
  pnh_.param("ufomap_ground_estimation_candidate_band", params_.ground_estimation_candidate_band,
             defaults.ground_estimation_candidate_band);
  pnh_.param("ufomap_ground_estimation_inlier_threshold", params_.ground_estimation_inlier_threshold,
             defaults.ground_estimation_inlier_threshold);
  pnh_.param("ufomap_ground_estimation_max_slope", params_.ground_estimation_max_slope,
             defaults.ground_estimation_max_slope);
  pnh_.param("ufomap_warmup_frames", params_.warmup_frames, defaults.warmup_frames);
  pnh_.param("ufomap_temporal_motion_enabled", params_.temporal_motion_enabled,
             defaults.temporal_motion_enabled);
  pnh_.param("ufomap_temporal_match_distance", params_.temporal_match_distance,
             defaults.temporal_match_distance);
  pnh_.param("ufomap_temporal_search_radius", params_.temporal_search_radius,
             defaults.temporal_search_radius);
  pnh_.param("ufomap_temporal_cluster_radius", params_.temporal_cluster_radius,
             defaults.temporal_cluster_radius);
  pnh_.param("ufomap_temporal_min_cluster_points", params_.temporal_min_cluster_points,
             defaults.temporal_min_cluster_points);
  pnh_.param("corridor_dynamic_enabled", params_.corridor_dynamic_enabled,
             defaults.corridor_dynamic_enabled);
  pnh_.param("corridor_width", params_.corridor_width, defaults.corridor_width);
  pnh_.param("corridor_wall_clearance", params_.corridor_wall_clearance,
             defaults.corridor_wall_clearance);
  pnh_.param("corridor_roi_min_forward", params_.corridor_roi_min_forward,
             defaults.corridor_roi_min_forward);
  pnh_.param("corridor_roi_max_forward", params_.corridor_roi_max_forward,
             defaults.corridor_roi_max_forward);
  pnh_.param("corridor_roi_min_z", params_.corridor_roi_min_z,
             defaults.corridor_roi_min_z);
  pnh_.param("corridor_roi_max_z", params_.corridor_roi_max_z,
             defaults.corridor_roi_max_z);
  pnh_.param("corridor_detection_voxel", params_.corridor_detection_voxel,
             defaults.corridor_detection_voxel);
  pnh_.param("corridor_history_frames", params_.corridor_history_frames,
             defaults.corridor_history_frames);
  pnh_.param("corridor_min_confirm_hits", params_.corridor_min_confirm_hits,
             defaults.corridor_min_confirm_hits);
  pnh_.param("corridor_min_publish_hits", params_.corridor_min_publish_hits,
             defaults.corridor_min_publish_hits);
  pnh_.param("corridor_min_lateral_speed", params_.corridor_min_lateral_speed,
             defaults.corridor_min_lateral_speed);
  pnh_.param("corridor_min_lateral_span", params_.corridor_min_lateral_span,
             defaults.corridor_min_lateral_span);
  pnh_.param("corridor_publish_unknown_as_dynamic",
             params_.corridor_publish_unknown_as_dynamic,
             defaults.corridor_publish_unknown_as_dynamic);
  pnh_.param("corridor_static_confirm_frames", params_.corridor_static_confirm_frames,
             defaults.corridor_static_confirm_frames);
  pnh_.param("corridor_confirmed_static_confirm_frames",
             params_.corridor_confirmed_static_confirm_frames,
             defaults.corridor_confirmed_static_confirm_frames);
  pnh_.param("corridor_static_lateral_speed", params_.corridor_static_lateral_speed,
             defaults.corridor_static_lateral_speed);
  pnh_.param("corridor_forward_alignment_cos", params_.corridor_forward_alignment_cos,
             defaults.corridor_forward_alignment_cos);
  pnh_.param("corridor_max_missed_frames", params_.corridor_max_missed_frames,
             defaults.corridor_max_missed_frames);
  pnh_.param("corridor_internal_max_missed_frames",
             params_.corridor_internal_max_missed_frames,
             defaults.corridor_internal_max_missed_frames);
  pnh_.param("corridor_internal_track_timeout",
             params_.corridor_internal_track_timeout,
             defaults.corridor_internal_track_timeout);
  pnh_.param("corridor_max_forward_speed", params_.corridor_max_forward_speed,
             defaults.corridor_max_forward_speed);
  pnh_.param("corridor_max_vertical_speed", params_.corridor_max_vertical_speed,
             defaults.corridor_max_vertical_speed);
  pnh_.param("corridor_association_gate", params_.corridor_association_gate,
             defaults.corridor_association_gate);
  pnh_.param("corridor_track_timeout", params_.corridor_track_timeout,
             defaults.corridor_track_timeout);
  pnh_.param("corridor_min_cluster_points", params_.corridor_min_cluster_points,
             defaults.corridor_min_cluster_points);
  pnh_.param("corridor_min_confirm_extent", params_.corridor_min_confirm_extent,
             defaults.corridor_min_confirm_extent);
  pnh_.param("corridor_min_confirm_second_extent",
             params_.corridor_min_confirm_second_extent,
             defaults.corridor_min_confirm_second_extent);
  pnh_.param("corridor_min_confirm_extent_frames",
             params_.corridor_min_confirm_extent_frames,
             defaults.corridor_min_confirm_extent_frames);
  pnh_.param("corridor_max_cluster_extent", params_.corridor_max_cluster_extent,
             defaults.corridor_max_cluster_extent);
  pnh_.param("corridor_oversized_split_enabled", params_.corridor_oversized_split_enabled,
             defaults.corridor_oversized_split_enabled);
  pnh_.param("corridor_split_min_cluster_points", params_.corridor_split_min_cluster_points,
             defaults.corridor_split_min_cluster_points);
  pnh_.param("corridor_split_max_subclusters", params_.corridor_split_max_subclusters,
             defaults.corridor_split_max_subclusters);
  pnh_.param("corridor_max_candidates", params_.corridor_max_candidates,
             defaults.corridor_max_candidates);
  pnh_.param("corridor_reject_candidate_count", params_.corridor_reject_candidate_count,
             defaults.corridor_reject_candidate_count);
  pnh_.param("corridor_min_wall_points", params_.corridor_min_wall_points,
             defaults.corridor_min_wall_points);
  pnh_.param("corridor_wall_search_forward", params_.corridor_wall_search_forward,
             defaults.corridor_wall_search_forward);
  pnh_.param("corridor_wall_filter_alpha", params_.corridor_wall_filter_alpha,
             defaults.corridor_wall_filter_alpha);
  pnh_.param("corridor_reactivation_displacement", params_.corridor_reactivation_displacement,
             defaults.corridor_reactivation_displacement);
  pnh_.param("corridor_turn_reset_yaw", params_.corridor_turn_reset_yaw,
             defaults.corridor_turn_reset_yaw);
  pnh_.param("ufomap_insert_hit_depth", params_.insert_hit_depth, defaults.insert_hit_depth);
  pnh_.param("ufomap_insert_miss_depth", params_.insert_miss_depth, defaults.insert_miss_depth);
  pnh_.param("ufomap_ray_casting_depth", params_.ray_casting_depth, defaults.ray_casting_depth);
  pnh_.param("ufomap_num_threads", params_.num_threads, defaults.num_threads);
  pnh_.param("ufomap_insert_only_valid", params_.only_valid, defaults.only_valid);
  pnh_.param("ufomap_inflate_unknown", params_.inflate_unknown, defaults.inflate_unknown);
  pnh_.param("ufomap_inflate_unknown_compensation", params_.inflate_unknown_compensation, defaults.inflate_unknown_compensation);
  pnh_.param("ufomap_ray_passthrough_hits", params_.ray_passthrough_hits, defaults.ray_passthrough_hits);
  pnh_.param("ufomap_inflate_hits_dist", params_.inflate_hits_dist, defaults.inflate_hits_dist);
  pnh_.param("ufomap_simple_ray_casting", params_.simple_ray_casting, defaults.simple_ray_casting);
  pnh_.param("ufomap_simple_ray_casting_factor", params_.simple_ray_casting_factor, defaults.simple_ray_casting_factor);
  pnh_.param("ufomap_sliding_window_size", params_.sliding_window_size, defaults.sliding_window_size);
  pnh_.param("ufomap_parallel", params_.parallel, defaults.parallel);
  pnh_.param("ufomap_propagate", params_.propagate, defaults.propagate);
  pnh_.param("static_map_visualization_max_z", params_.static_map_visualization_max_z, defaults.static_map_visualization_max_z);
  pnh_.param("ufomap_down_sampling_method", params_.down_sampling_method, defaults.down_sampling_method);
  config_ = buildUfomapConfig(params_);
}

void UfomapMapper::configureUfomap() {
  // 该函数会在 clearMap() 后重建 runtime。
  ufomap_update_count_.store(0U);
  processed_frame_count_ = 0U;
  ground_plane_.reset();
  ground_plane_samples_.clear();
  ground_plane_locked_ = false;
  previous_frame_points_.clear();
  resetCorridorTracks();
  runtime_ = std::make_unique<UfomapRuntime>(static_cast<ufo::node_size_t>(config_.resolution),
                                             static_cast<ufo::depth_t>(config_.depth_levels));

  runtime_->integration_params = {};
  runtime_->integration_params.down_sampling_method =
      toUfoDownSamplingMethod(config_.down_sampling_method);
  runtime_->integration_params.hit_depth = static_cast<ufo::depth_t>(config_.insert_hit_depth);
  runtime_->integration_params.miss_depth = static_cast<ufo::depth_t>(config_.insert_miss_depth);
  runtime_->integration_params.ray_casting_depth =
      static_cast<ufo::depth_t>(config_.ray_casting_depth);
  runtime_->integration_params.min_range = static_cast<float>(config_.min_range);
  runtime_->integration_params.max_range = static_cast<float>(config_.max_range);
  runtime_->integration_params.only_valid = config_.only_valid;
  runtime_->integration_params.inflate_unknown = static_cast<std::size_t>(config_.inflate_unknown);
  runtime_->integration_params.inflate_unknown_compensation =
      config_.inflate_unknown_compensation;
  runtime_->integration_params.ray_passthrough_hits = config_.ray_passthrough_hits;
  runtime_->integration_params.inflate_hits_dist = static_cast<float>(config_.inflate_hits_dist);
  runtime_->integration_params.ray_casting_method =
      config_.simple_ray_casting ? ufo::RayCastingMethod::SIMPLE : ufo::RayCastingMethod::PROPER;
  runtime_->integration_params.simple_ray_casting_factor =
      static_cast<float>(config_.simple_ray_casting_factor);
  runtime_->integration_params.sliding_window_size = config_.sliding_window_size;
  runtime_->integration_params.num_threads =
      static_cast<std::size_t>(config_.num_threads);
  runtime_->integration_params.parallel = config_.parallel;
  runtime_->integration_params.propagate = config_.propagate;
}

UfomapMapper::UfomapRuntime& UfomapMapper::runtimeLocked() {
  assert(runtime_ != nullptr && "runtime_ must exist while UfomapMapper is alive");
  return *runtime_;
}

const UfomapMapper::UfomapRuntime& UfomapMapper::runtimeLocked() const {
  assert(runtime_ != nullptr && "runtime_ must exist while UfomapMapper is alive");
  return *runtime_;
}

std::optional<UfomapMapper::GroundPlaneModel> UfomapMapper::estimateGroundPlane(
    const UfomapPointCloud& points,
    const ufo::Point& sensor_origin) const {
  if (!config_.ground_filter_enabled || points.empty()) {
    return std::nullopt;
  }

  const double radius_sq = config_.ground_estimation_radius *
                           config_.ground_estimation_radius;
  const double candidate_min_z = static_cast<double>(sensor_origin.z) - 3.0;
  const double candidate_max_z = static_cast<double>(sensor_origin.z) + 0.25;
  const double bin_size = config_.ground_estimation_bin_size;
  std::unordered_map<int, std::size_t> histogram;
  histogram.reserve(points.size());

  for (const auto& point : points) {
    const double dx = static_cast<double>(point.x) - sensor_origin.x;
    const double dy = static_cast<double>(point.y) - sensor_origin.y;
    const double point_z = static_cast<double>(point.z);
    if (dx * dx + dy * dy > radius_sq || point_z < candidate_min_z ||
        point_z > candidate_max_z) {
      continue;
    }

    const int bin = static_cast<int>(std::floor(point_z / bin_size));
    ++histogram[bin];
  }

  int best_bin = 0;
  std::size_t best_count = 0U;
  for (const auto& entry : histogram) {
    if (entry.second > best_count) {
      best_bin = entry.first;
      best_count = entry.second;
    }
  }
  if (best_count < static_cast<std::size_t>(config_.ground_estimation_min_points)) {
    return std::nullopt;
  }

  const double best_bin_center = (static_cast<double>(best_bin) + 0.5) * bin_size;
  struct GroundFitPoint {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  std::vector<GroundFitPoint> candidates;
  candidates.reserve(points.size());
  const double candidate_half_width = config_.ground_estimation_candidate_band;
  for (const auto& point : points) {
    const double dx = static_cast<double>(point.x) - sensor_origin.x;
    const double dy = static_cast<double>(point.y) - sensor_origin.y;
    const double point_z = static_cast<double>(point.z);
    if (dx * dx + dy * dy > radius_sq || point_z < candidate_min_z ||
        point_z > candidate_max_z ||
        std::abs(point_z - best_bin_center) > candidate_half_width) {
      continue;
    }
    candidates.push_back(GroundFitPoint{dx, dy, point_z});
  }

  if (candidates.size() < static_cast<std::size_t>(config_.ground_estimation_min_points)) {
    return std::nullopt;
  }

  const double inlier_threshold = config_.ground_estimation_inlier_threshold;
  const double max_slope = config_.ground_estimation_max_slope;

  auto fitFromIndices = [&](const std::vector<std::size_t>& indices)
      -> std::optional<GroundPlaneModel> {
    if (indices.size() < 3U) {
      return std::nullopt;
    }

    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (const std::size_t index : indices) {
      const GroundFitPoint& point = candidates[index];
      const Eigen::Vector3d row(point.x, point.y, 1.0);
      normal.noalias() += row * row.transpose();
      rhs.noalias() += row * point.z;
    }
    const auto decomposition = normal.fullPivLu();
    if (decomposition.rank() < 3) {
      return std::nullopt;
    }

    const Eigen::Vector3d coefficients = decomposition.solve(rhs);
    if (!coefficients.allFinite() ||
        std::hypot(coefficients.x(), coefficients.y()) > max_slope) {
      return std::nullopt;
    }

    GroundPlaneModel model;
    model.slope_x = coefficients.x();
    model.slope_y = coefficients.y();
    model.intercept = coefficients.z() - model.slope_x * sensor_origin.x -
                      model.slope_y * sensor_origin.y;
    return model;
  };

  auto heightAtLocal = [&](const GroundPlaneModel& model,
                           const GroundFitPoint& point) {
    return model.heightAt(sensor_origin.x + point.x, sensor_origin.y + point.y);
  };

  std::mt19937 generator(static_cast<std::uint32_t>(candidates.size() * 2654435761U));
  std::uniform_int_distribution<std::size_t> distribution(0U, candidates.size() - 1U);
  constexpr std::size_t kRansacIterations = 160U;
  std::vector<std::size_t> best_inliers;
  double best_squared_error = std::numeric_limits<double>::infinity();

  for (std::size_t iteration = 0U; iteration < kRansacIterations; ++iteration) {
    const std::size_t first = distribution(generator);
    std::size_t second = distribution(generator);
    std::size_t third = distribution(generator);
    if (first == second || first == third || second == third) {
      continue;
    }

    const std::vector<std::size_t> sample_indices{first, second, third};
    const std::optional<GroundPlaneModel> model = fitFromIndices(sample_indices);
    if (!model.has_value()) {
      continue;
    }

    std::vector<std::size_t> inliers;
    inliers.reserve(candidates.size());
    double squared_error = 0.0;
    for (std::size_t index = 0U; index < candidates.size(); ++index) {
      const double residual = candidates[index].z - heightAtLocal(*model, candidates[index]);
      if (std::abs(residual) <= inlier_threshold) {
        inliers.push_back(index);
        squared_error += residual * residual;
      }
    }

    if (inliers.size() > best_inliers.size() ||
        (inliers.size() == best_inliers.size() && squared_error < best_squared_error)) {
      best_inliers = std::move(inliers);
      best_squared_error = squared_error;
    }
  }

  if (best_inliers.size() < static_cast<std::size_t>(config_.ground_estimation_min_points)) {
    return std::nullopt;
  }

  std::optional<GroundPlaneModel> refined_model = fitFromIndices(best_inliers);
  if (!refined_model.has_value()) {
    return std::nullopt;
  }

  for (int refinement = 0; refinement < 2; ++refinement) {
    std::vector<std::size_t> refined_inliers;
    refined_inliers.reserve(best_inliers.size());
    for (std::size_t index = 0U; index < candidates.size(); ++index) {
      const double residual = candidates[index].z -
                              heightAtLocal(*refined_model, candidates[index]);
      if (std::abs(residual) <= inlier_threshold) {
        refined_inliers.push_back(index);
      }
    }
    if (refined_inliers.size() <
        static_cast<std::size_t>(config_.ground_estimation_min_points)) {
      return std::nullopt;
    }
    refined_model = fitFromIndices(refined_inliers);
    if (!refined_model.has_value()) {
      return std::nullopt;
    }
  }

  return refined_model;
}

void UfomapMapper::updateGroundPlane(const UfomapPointCloud& points,
                                     const ufo::Point& sensor_origin) {
  if (!config_.ground_filter_enabled || ground_plane_locked_) {
    return;
  }

  const std::optional<GroundPlaneModel> sample = estimateGroundPlane(points, sensor_origin);
  if (!sample.has_value()) {
    return;
  }

  if (ground_plane_.has_value() &&
      std::abs(sample->heightAt(sensor_origin.x, sensor_origin.y) -
               ground_plane_->heightAt(sensor_origin.x, sensor_origin.y)) > 0.25) {
    return;
  }

  ground_plane_samples_.push_back(*sample);
  if (ground_plane_samples_.size() >
      static_cast<std::size_t>(config_.ground_estimation_frames)) {
    ground_plane_samples_.erase(ground_plane_samples_.begin());
  }
  ground_plane_ = GroundPlaneModel{
      medianValue([&]() {
        std::vector<double> values;
        values.reserve(ground_plane_samples_.size());
        for (const auto& plane : ground_plane_samples_) {
          values.push_back(plane.slope_x);
        }
        return values;
      }()),
      medianValue([&]() {
        std::vector<double> values;
        values.reserve(ground_plane_samples_.size());
        for (const auto& plane : ground_plane_samples_) {
          values.push_back(plane.slope_y);
        }
        return values;
      }()),
      medianValue([&]() {
        std::vector<double> values;
        values.reserve(ground_plane_samples_.size());
        for (const auto& plane : ground_plane_samples_) {
          values.push_back(plane.intercept);
        }
        return values;
      }())};
  if (ground_plane_samples_.size() >=
      static_cast<std::size_t>(config_.ground_estimation_frames)) {
    ground_plane_locked_ = true;
    ROS_INFO_STREAM("Ground plane locked: z=" << ground_plane_->slope_x << "*x + "
                    << ground_plane_->slope_y << "*y + " << ground_plane_->intercept
                    << " using " << ground_plane_samples_.size() << " frames");
  }
}

UfomapMapper::UfomapPointCloud UfomapMapper::filterGroundPoints(
    const UfomapPointCloud& points,
    std::vector<std::size_t>* retained_source_indices) const {
  if (retained_source_indices != nullptr) {
    retained_source_indices->clear();
    retained_source_indices->reserve(points.size());
  }
  if (!config_.ground_filter_enabled || !ground_plane_.has_value()) {
    if (retained_source_indices != nullptr) {
      for (std::size_t index = 0U; index < points.size(); ++index) {
        retained_source_indices->push_back(index);
      }
    }
    return points;
  }

  UfomapPointCloud filtered_points;
  filtered_points.reserve(points.size());
  for (std::size_t index = 0U; index < points.size(); ++index) {
    const auto& point = points[index];
    // 只删除拟合地面及以下的点；地面以上的点全部保留，不使用对称高度带。
    const double ground_height = ground_plane_->heightAt(point.x, point.y);
    if (static_cast<double>(point.z) > ground_height) {
      filtered_points.push_back(point);
      if (retained_source_indices != nullptr) {
        retained_source_indices->push_back(index);
      }
    }
  }
  return filtered_points;
}

std::vector<std::size_t> UfomapMapper::detectTemporalMotion(
    const UfomapPointCloud& points,
    const std::vector<bool>* corridor_handled_indices) {
  std::vector<std::size_t> motion_indices;
  if (!config_.temporal_motion_enabled || points.empty() ||
      previous_frame_points_.empty()) {
    return motion_indices;
  }

  const double match_distance = config_.temporal_match_distance;
  const double search_radius = config_.temporal_search_radius;
  const double cell_size = std::max(match_distance, search_radius / 4.0);
  const double match_distance_sq = match_distance * match_distance;
  const double search_radius_sq = search_radius * search_radius;

  using PreviousPointGrid =
      std::unordered_map<TemporalGridKey, std::vector<std::size_t>, TemporalGridKeyHash>;
  PreviousPointGrid previous_grid;
  previous_grid.reserve(previous_frame_points_.size());
  for (std::size_t index = 0U; index < previous_frame_points_.size(); ++index) {
    previous_grid[makeTemporalGridKey(previous_frame_points_[index], cell_size)].push_back(index);
  }

  const auto hasNearbyPreviousPoint = [&](const ufo::Point& point,
                                          const double radius_sq) {
    const double radius = std::sqrt(radius_sq);
    const TemporalGridKey min_key = makeTemporalGridKey(
        ufo::Point(static_cast<float>(static_cast<double>(point.x) - radius),
                   static_cast<float>(static_cast<double>(point.y) - radius),
                   static_cast<float>(static_cast<double>(point.z) - radius)),
        cell_size);
    const TemporalGridKey max_key = makeTemporalGridKey(
        ufo::Point(static_cast<float>(static_cast<double>(point.x) + radius),
                   static_cast<float>(static_cast<double>(point.y) + radius),
                   static_cast<float>(static_cast<double>(point.z) + radius)),
        cell_size);

    for (int x = min_key.x; x <= max_key.x; ++x) {
      for (int y = min_key.y; y <= max_key.y; ++y) {
        for (int z = min_key.z; z <= max_key.z; ++z) {
          const auto found = previous_grid.find(TemporalGridKey{x, y, z});
          if (found == previous_grid.end()) {
            continue;
          }
          for (const std::size_t previous_index : found->second) {
            const auto& previous_point = previous_frame_points_[previous_index];
            const double dx = static_cast<double>(point.x) - previous_point.x;
            const double dy = static_cast<double>(point.y) - previous_point.y;
            const double dz = static_cast<double>(point.z) - previous_point.z;
            if (dx * dx + dy * dy + dz * dz <= radius_sq) {
              return true;
            }
          }
        }
      }
    }
    return false;
  };

  std::vector<std::size_t> raw_motion_indices;
  raw_motion_indices.reserve(points.size() / 10U + 1U);
  std::shared_lock<std::shared_mutex> map_lock(map_mutex_);
  const auto& runtime = runtimeLocked();
  for (std::size_t index = 0U; index < points.size(); ++index) {
    // 通道候选层已经负责 ROI 内的跨帧关联；不能再让通用 seenFree/hits
    // 复检把同一批点按另一套规则重新分类。
    if (corridor_handled_indices != nullptr &&
        index < corridor_handled_indices->size() &&
        (*corridor_handled_indices)[index]) {
      continue;
    }
    const auto& point = points[index];
    const ufo::Point current_point(point.x, point.y, point.z);
    if (hasNearbyPreviousPoint(current_point, match_distance_sq)) {
      continue;
    }
    const auto code = runtime.map.toCodeChecked(current_point);
    if (code.has_value()) {
      if (runtime.map.seenFree(current_point)) {
        continue;
      }
      if (runtime.map.exists(*code)) {
        const auto node = runtime.map(*code);
        if (runtime.map.hits(node) > 0) {
          continue;
        }
      }
    }
    if (hasNearbyPreviousPoint(current_point, search_radius_sq)) {
      raw_motion_indices.push_back(index);
    }
  }

  if (raw_motion_indices.empty()) {
    return motion_indices;
  }

  const double cluster_radius = config_.temporal_cluster_radius;
  const double cluster_radius_sq = cluster_radius * cluster_radius;
  const double cluster_cell_size = std::max(cluster_radius / 2.0, match_distance);
  PreviousPointGrid candidate_grid;
  candidate_grid.reserve(raw_motion_indices.size());
  for (const std::size_t index : raw_motion_indices) {
    candidate_grid[makeTemporalGridKey(points[index], cluster_cell_size)].push_back(index);
  }

  const auto nearbyCandidateCount = [&](const std::size_t candidate_index) {
    const auto& point = points[candidate_index];
    const TemporalGridKey min_key = makeTemporalGridKey(
        ufo::Point(static_cast<float>(static_cast<double>(point.x) - cluster_radius),
                   static_cast<float>(static_cast<double>(point.y) - cluster_radius),
                   static_cast<float>(static_cast<double>(point.z) - cluster_radius)),
        cluster_cell_size);
    const TemporalGridKey max_key = makeTemporalGridKey(
        ufo::Point(static_cast<float>(static_cast<double>(point.x) + cluster_radius),
                   static_cast<float>(static_cast<double>(point.y) + cluster_radius),
                   static_cast<float>(static_cast<double>(point.z) + cluster_radius)),
        cluster_cell_size);

    std::size_t count = 0U;
    for (int x = min_key.x; x <= max_key.x; ++x) {
      for (int y = min_key.y; y <= max_key.y; ++y) {
        for (int z = min_key.z; z <= max_key.z; ++z) {
          const auto found = candidate_grid.find(TemporalGridKey{x, y, z});
          if (found == candidate_grid.end()) {
            continue;
          }
          for (const std::size_t neighbor_index : found->second) {
            const auto& neighbor = points[neighbor_index];
            const double dx = static_cast<double>(point.x) - neighbor.x;
            const double dy = static_cast<double>(point.y) - neighbor.y;
            const double dz = static_cast<double>(point.z) - neighbor.z;
            if (dx * dx + dy * dy + dz * dz <= cluster_radius_sq) {
              ++count;
              if (count >= static_cast<std::size_t>(config_.temporal_min_cluster_points)) {
                return count;
              }
            }
          }
        }
      }
    }
    return count;
  };

  motion_indices.reserve(raw_motion_indices.size());
  for (const std::size_t index : raw_motion_indices) {
    if (nearbyCandidateCount(index) >=
        static_cast<std::size_t>(config_.temporal_min_cluster_points)) {
      motion_indices.push_back(index);
    }
  }
  ROS_INFO_STREAM_THROTTLE(
      1.0, "Ufomap temporal candidates: raw=" << raw_motion_indices.size()
                                               << ", clustered=" << motion_indices.size()
                                               << ", radius=" << cluster_radius
                                               << ", min_points="
                                               << config_.temporal_min_cluster_points);
  return motion_indices;
}

void UfomapMapper::resetCorridorTracks() {
  corridor_tracks_.clear();
  next_corridor_track_id_ = 1U;
  // 传感器刚转入通道时可能先看到球、随后才看到两侧墙。启用通道检测时先用
  // 配置宽度建立名义边界，后续双侧墙观测只负责校正它。
  corridor_wall_valid_ = config_.corridor_dynamic_enabled;
  corridor_wall_measured_ = false;
  corridor_left_wall_ = -0.5 * config_.corridor_width;
  corridor_right_wall_ = 0.5 * config_.corridor_width;
  corridor_forward_x_ = 1.0;
  corridor_forward_y_ = 0.0;
  corridor_last_yaw_valid_ = false;
  corridor_last_yaw_ = 0.0;
}

UfomapVoxelCode UfomapMapper::makeDetectorVoxelCode(const ufo::Point& point) const {
  // 动态聚类需要一个与 UFOMap 分辨率无关的 key。加偏置是为了让负世界坐标
  // 在 UFOMap 使用的无符号 key_t 中仍保持相邻关系。
  constexpr std::int64_t kKeyOffset = (static_cast<std::int64_t>(1) << 29);
  const double resolution = std::max(0.02, config_.corridor_detection_voxel);
  const auto toKey = [resolution](const double value) {
    const std::int64_t key = static_cast<std::int64_t>(std::floor(value / resolution)) +
                             kKeyOffset;
    return static_cast<ufo::key_t>(std::max<std::int64_t>(0, key));
  };

  UfomapVoxelCode code;
  code.key_x = toKey(point.x);
  code.key_y = toKey(point.y);
  code.key_z = toKey(point.z);
  code.depth = 0;
  code.raw_code = 0;
  return code;
}

UfomapMapper::CorridorCandidateResult UfomapMapper::detectCorridorCandidates(
    const UfomapPointCloud& points,
    const ufo::Point& sensor_origin,
    const nav_msgs::Odometry& odom_msg,
    const ros::Time& stamp) {
  CorridorCandidateResult result;
  result.handled_indices.assign(points.size(), false);
  result.dynamic_indices.assign(points.size(), false);
  result.provisional_indices.assign(points.size(), false);
  result.source_track_ids.assign(points.size(), 0U);
  result.measured_velocities.assign(points.size(), ufo::Point(0.0F, 0.0F, 0.0F));
  result.holdout_indices.assign(points.size(), false);
  result.released_static_indices.assign(points.size(), false);
  if (!config_.corridor_dynamic_enabled || points.empty()) {
    return result;
  }

  const auto& orientation = odom_msg.pose.pose.orientation;
  const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
  double yaw = std::atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                          1.0 - 2.0 * (orientation.y * orientation.y +
                                       orientation.z * orientation.z));
  if (!std::isfinite(yaw) || quaternion_norm < 1e-6) {
    yaw = std::atan2(corridor_forward_y_, corridor_forward_x_);
  }

  if (corridor_last_yaw_valid_ && config_.corridor_turn_reset_yaw > 0.0 &&
      std::abs(wrappedAngleDifference(yaw, corridor_last_yaw_)) >=
          config_.corridor_turn_reset_yaw) {
    // 转弯后只清除依赖旧横向轴的速度/方向证据，保留世界坐标中心和
    // released_static 状态。这样墙面采样不会因转弯重新生成一串 provisional ID，
    // 同时真正新出现的球仍会从当前帧建立新轨迹。
    for (auto& track : corridor_tracks_) {
      track.velocity = ufo::Point(0.0F, 0.0F, 0.0F);
      track.history.clear();
      track.lateral_direction_history.clear();
      track.static_evidence_frames = 0U;
      track.lateral_evidence_frames = 0U;
      track.reactivation_evidence_frames = 0U;
      track.motion_qualified = false;
      track.last_motion_qualified_stamp = ros::Time();
      track.missed_frames = 0U;
      if (!track.confirmed && !track.released_static) {
        // 未确认轨迹的 hits 属于旧通道轴下的证据 epoch。保留 source ID 和
        // 世界坐标中心用于重关联，但从转弯帧重新累计确认/超时证据。
        track.hits = 0U;
        track.last_seen = stamp;
      }
    }
    corridor_left_wall_ = -0.5 * config_.corridor_width;
    corridor_right_wall_ = 0.5 * config_.corridor_width;
    corridor_wall_valid_ = true;
    corridor_wall_measured_ = false;
  }
  corridor_last_yaw_ = yaw;
  corridor_last_yaw_valid_ = true;
  corridor_forward_x_ = std::cos(yaw);
  corridor_forward_y_ = std::sin(yaw);
  const double lateral_x = -corridor_forward_y_;
  const double lateral_y = corridor_forward_x_;

  std::vector<double> left_wall_samples;
  std::vector<double> right_wall_samples;
  left_wall_samples.reserve(points.size() / 20U + 1U);
  right_wall_samples.reserve(points.size() / 20U + 1U);
  const double wall_search_limit = std::max(config_.corridor_width * 1.5, 1.0);
  for (const auto& point : points) {
    const double dx = static_cast<double>(point.x) - sensor_origin.x;
    const double dy = static_cast<double>(point.y) - sensor_origin.y;
    const double longitudinal = dx * corridor_forward_x_ + dy * corridor_forward_y_;
    const double lateral = dx * lateral_x + dy * lateral_y;
    if (std::abs(longitudinal) > config_.corridor_wall_search_forward ||
        point.z < config_.corridor_roi_min_z - 0.4 ||
        point.z > config_.corridor_roi_max_z + 0.4) {
      continue;
    }
    if (lateral < -0.2 && lateral > -wall_search_limit) {
      left_wall_samples.push_back(lateral);
    } else if (lateral > 0.2 && lateral < wall_search_limit) {
      right_wall_samples.push_back(lateral);
    }
  }

  if (left_wall_samples.size() >= static_cast<std::size_t>(config_.corridor_min_wall_points) &&
      right_wall_samples.size() >= static_cast<std::size_t>(config_.corridor_min_wall_points)) {
    const double measured_left = medianValue(std::move(left_wall_samples));
    const double measured_right = medianValue(std::move(right_wall_samples));
    const double measured_width = measured_right - measured_left;
    if (std::isfinite(measured_left) && std::isfinite(measured_right) &&
        // 已知通道宽度约为1.5m；过宽的“左右边界”通常是背景点/人或球
        // 被误当成一侧墙。拒绝这类测量并继续使用上一帧/名义墙模型，避免
        // 把真正的内部球排到 ROI 外。
        measured_width >= 0.85 * config_.corridor_width &&
        measured_width <= 1.15 * config_.corridor_width) {
      const double alpha = corridor_wall_measured_ ? config_.corridor_wall_filter_alpha : 1.0;
      corridor_left_wall_ = alpha * measured_left + (1.0 - alpha) * corridor_left_wall_;
      corridor_right_wall_ = alpha * measured_right + (1.0 - alpha) * corridor_right_wall_;
      corridor_wall_valid_ = true;
      corridor_wall_measured_ = true;
    }
  }

  // 单帧看不全两侧墙时继续沿用上次墙模型；首次进入通道则使用名义宽度。
  result.wall_valid = corridor_wall_valid_;
  if (!corridor_wall_valid_) {
    for (auto& track : corridor_tracks_) {
      ++track.missed_frames;
    }
    corridor_tracks_.erase(
        std::remove_if(corridor_tracks_.begin(), corridor_tracks_.end(),
                       [&](const CorridorTrack& track) {
                         return track.missed_frames > static_cast<std::size_t>(
                                    config_.corridor_internal_max_missed_frames) ||
                                (stamp - track.last_seen).toSec() >
                                    config_.corridor_internal_track_timeout;
                       }),
        corridor_tracks_.end());
    return result;
  }

  // 先标记通道内部和墙边点为“由通道层接管”。这样通道墙面的 seenFree
  // 不会绕过墙面过滤重新进入通用动态分类。
  std::unordered_map<TemporalGridKey, std::vector<std::size_t>, TemporalGridKeyHash>
      candidate_buckets;
  const double detection_voxel = std::max(0.02, config_.corridor_detection_voxel);
  for (std::size_t index = 0U; index < points.size(); ++index) {
    const auto& point = points[index];
    const double dx = static_cast<double>(point.x) - sensor_origin.x;
    const double dy = static_cast<double>(point.y) - sensor_origin.y;
    const double longitudinal = dx * corridor_forward_x_ + dy * corridor_forward_y_;
    const double lateral = dx * lateral_x + dy * lateral_y;
    if (longitudinal < config_.corridor_roi_min_forward ||
        longitudinal > config_.corridor_roi_max_forward ||
        point.z < config_.corridor_roi_min_z || point.z > config_.corridor_roi_max_z ||
        lateral < corridor_left_wall_ - config_.corridor_wall_clearance ||
        lateral > corridor_right_wall_ + config_.corridor_wall_clearance) {
      continue;
    }
    result.handled_indices[index] = true;
    if (lateral <= corridor_left_wall_ + config_.corridor_wall_clearance ||
        lateral >= corridor_right_wall_ - config_.corridor_wall_clearance) {
      continue;
    }
    // 即使本帧只有少量回波、尚未达到聚类点数，也先隔离出静态积分。
    // 下一帧点数恢复后仍可建立同一候选，避免首见稀疏球点先污染历史占据。
    result.holdout_indices[index] = true;
    candidate_buckets[makeTemporalGridKey(point, detection_voxel)].push_back(index);
  }

  std::size_t published_track_count = 0U;

  struct CorridorCluster {
    std::vector<std::size_t> indices;
    std::vector<TemporalGridKey> keys;
    ufo::Point center{};
    ufo::Point size{};
    double raw_extent{0.0};
    double raw_second_extent{0.0};
  };
  std::vector<CorridorCluster> clusters;
  std::unordered_set<TemporalGridKey, TemporalGridKeyHash> visited;
  const std::array<std::array<int, 3>, 26> offsets = [] {
    std::array<std::array<int, 3>, 26> values{};
    std::size_t cursor = 0U;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          values[cursor++] = {dx, dy, dz};
        }
      }
    }
    return values;
  }();
  const std::array<std::array<int, 3>, 6> split_offsets = {{{-1, 0, 0}, {1, 0, 0},
                                                              {0, -1, 0}, {0, 1, 0},
                                                              {0, 0, -1}, {0, 0, 1}}};

  const auto makeCluster = [&](std::vector<std::size_t> indices,
                               std::vector<TemporalGridKey> keys,
                               const bool reject_oversized,
                               CorridorCluster* output) {
    if (output == nullptr || indices.size() < 2U) {
      return false;
    }
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    for (const std::size_t index : indices) {
      const auto& point = points[index];
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      min_z = std::min(min_z, static_cast<double>(point.z));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
      max_z = std::max(max_z, static_cast<double>(point.z));
      sum_x += point.x;
      sum_y += point.y;
      sum_z += point.z;
    }
    const double extent = std::hypot(std::hypot(max_x - min_x, max_y - min_y), max_z - min_z);
    if (reject_oversized && config_.corridor_max_cluster_extent > 0.0 &&
        extent > config_.corridor_max_cluster_extent) {
      return false;
    }
    const double count = static_cast<double>(indices.size());
    output->indices = std::move(indices);
    output->keys = std::move(keys);
    output->center = ufo::Point(static_cast<float>(sum_x / count),
                                static_cast<float>(sum_y / count),
                                static_cast<float>(sum_z / count));
    output->size = ufo::Point(static_cast<float>(std::max(max_x - min_x, detection_voxel)),
                              static_cast<float>(std::max(max_y - min_y, detection_voxel)),
                              static_cast<float>(std::max(max_z - min_z, detection_voxel)));
    output->raw_extent = extent;
    std::array<double, 3> raw_axis_extents{max_x - min_x, max_y - min_y, max_z - min_z};
    std::sort(raw_axis_extents.begin(), raw_axis_extents.end());
    output->raw_second_extent = raw_axis_extents[1];
    return true;
  };

  for (const auto& bucket : candidate_buckets) {
    if (visited.count(bucket.first) != 0U) {
      continue;
    }
    std::queue<TemporalGridKey> queue;
    queue.push(bucket.first);
    visited.insert(bucket.first);
    CorridorCluster cluster;
    while (!queue.empty()) {
      const TemporalGridKey key = queue.front();
      queue.pop();
      const auto found = candidate_buckets.find(key);
      if (found == candidate_buckets.end()) {
        continue;
      }
      cluster.keys.push_back(key);
      cluster.indices.insert(cluster.indices.end(), found->second.begin(), found->second.end());
      for (const auto& offset : offsets) {
        const TemporalGridKey neighbor{key.x + offset[0], key.y + offset[1], key.z + offset[2]};
        if (candidate_buckets.count(neighbor) != 0U && visited.insert(neighbor).second) {
          queue.push(neighbor);
        }
      }
    }

    CorridorCluster parent;
    if (!makeCluster(std::move(cluster.indices), std::move(cluster.keys), false, &parent)) {
      continue;
    }
    const bool oversized = config_.corridor_max_cluster_extent > 0.0 &&
                           parent.raw_extent > config_.corridor_max_cluster_extent;
    if (!oversized || !config_.corridor_oversized_split_enabled) {
      if (!oversized) {
        clusters.push_back(std::move(parent));
      }
      continue;
    }

    // 大簇通常是球与墙边通过一两个对角体素桥接。只对这个父簇
    // 重跑 6 邻域，避免全局改成 6 邻域后把稀疏球切碎。
    std::unordered_set<TemporalGridKey, TemporalGridKeyHash> parent_keys(
        parent.keys.begin(), parent.keys.end());
    std::unordered_set<TemporalGridKey, TemporalGridKeyHash> split_visited;
    std::size_t accepted_subclusters = 0U;
    for (const auto& start_key : parent.keys) {
      if (!split_visited.insert(start_key).second) {
        continue;
      }
      std::queue<TemporalGridKey> split_queue;
      split_queue.push(start_key);
      std::vector<TemporalGridKey> child_keys;
      std::vector<std::size_t> child_indices;
      while (!split_queue.empty()) {
        const TemporalGridKey key = split_queue.front();
        split_queue.pop();
        const auto found = candidate_buckets.find(key);
        if (found == candidate_buckets.end()) {
          continue;
        }
        child_keys.push_back(key);
        child_indices.insert(child_indices.end(), found->second.begin(), found->second.end());
        for (const auto& offset : split_offsets) {
          const TemporalGridKey neighbor{key.x + offset[0], key.y + offset[1], key.z + offset[2]};
          if (parent_keys.count(neighbor) != 0U && split_visited.insert(neighbor).second) {
            split_queue.push(neighbor);
          }
        }
      }

      CorridorCluster child;
      if (!makeCluster(std::move(child_indices), std::move(child_keys), true, &child) ||
          child.indices.size() < static_cast<std::size_t>(config_.corridor_split_min_cluster_points) ||
          child.raw_second_extent < config_.corridor_min_confirm_second_extent) {
        continue;
      }
      clusters.push_back(std::move(child));
      if (++accepted_subclusters >=
          static_cast<std::size_t>(config_.corridor_split_max_subclusters)) {
        break;
      }
    }
    if (accepted_subclusters == 0U) {
      ROS_DEBUG_STREAM_THROTTLE(1.0, "corridor oversized cluster unresolved, extent="
                                      << parent.raw_extent << " points=" << parent.indices.size());
    }
  }

  // 摆球的主要位移发生在通道横向。漏帧后允许横向门逐步放宽，
  // 前向和高度仍保持较紧，避免把同一截面外的静态碎片接进来。
  const auto associationCost = [&](const CorridorTrack& track,
                                   const CorridorCluster& cluster) {
    const double missed = static_cast<double>(std::min<std::size_t>(
        track.missed_frames,
        static_cast<std::size_t>(std::min(
            config_.corridor_internal_max_missed_frames, 5))));
    const double relaxed_gate = config_.corridor_association_gate * (1.0 + 0.20 * missed);
    const double lateral_gate = track.confirmed
        ? std::max(config_.corridor_association_gate, relaxed_gate) : relaxed_gate;
    const double forward_gate = track.confirmed
        ? std::max(0.35, relaxed_gate) : relaxed_gate;
    const double vertical_gate = track.confirmed
        ? std::max(0.35, relaxed_gate) : relaxed_gate;
    const ufo::Point delta(cluster.center.x - track.center.x,
                           cluster.center.y - track.center.y,
                           cluster.center.z - track.center.z);
    const double lateral_delta = std::abs(dotHorizontal(delta, lateral_x, lateral_y));
    const double forward_delta = std::abs(dotHorizontal(
        delta, corridor_forward_x_, corridor_forward_y_));
    const double vertical_delta = std::abs(static_cast<double>(delta.z));
    if (lateral_delta > lateral_gate || forward_delta > forward_gate ||
        vertical_delta > vertical_gate) {
      return std::numeric_limits<double>::infinity();
    }
    const double size_dx = static_cast<double>(cluster.size.x) - track.size.x;
    const double size_dy = static_cast<double>(cluster.size.y) - track.size.y;
    const double size_dz = static_cast<double>(cluster.size.z) - track.size.z;
    const double size_delta = std::sqrt(size_dx * size_dx + size_dy * size_dy +
                                        size_dz * size_dz);
    if (track.confirmed) {
      const auto sizeRatio = [](const double lhs, const double rhs) {
        if (!(std::isfinite(lhs) && std::isfinite(rhs)) || lhs <= 1e-6 || rhs <= 1e-6) {
          return 0.0;
        }
        return std::min(lhs, rhs) / std::max(lhs, rhs);
      };
      const double track_area = static_cast<double>(track.size.x) * track.size.y;
      const double cluster_area = static_cast<double>(cluster.size.x) * cluster.size.y;
      const double horizontal_area_ratio = sizeRatio(track_area, cluster_area);
      const double minimum_axis_ratio = std::min(
          sizeRatio(track.size.x, cluster.size.x),
          sizeRatio(track.size.y, cluster.size.y));
      if (size_delta > 0.35 || horizontal_area_ratio < 0.25 ||
          minimum_axis_ratio < 0.35) {
        return std::numeric_limits<double>::infinity();
      }
    }
    const double normalized_cost =
        lateral_delta / std::max(lateral_gate, 1e-6) +
        forward_delta / std::max(forward_gate, 1e-6) +
        vertical_delta / std::max(vertical_gate, 1e-6) +
        0.25 * size_delta / std::max(config_.corridor_max_cluster_extent, 1e-6);
    // 已确认轨迹只获得有限偏置，不再无条件抢走门内任意碎片。
    return normalized_cost - (track.confirmed ? 0.25 : 0.0);
  };
  const auto trackPriority = [](const CorridorTrack& track) {
    return track.released_static ? 3 : 0;
  };
  const auto bestAssociation = [&](const CorridorCluster& cluster) {
    std::pair<int, double> best_active{2, std::numeric_limits<double>::infinity()};
    std::pair<int, double> best_released{3, std::numeric_limits<double>::infinity()};
    for (const auto& track : corridor_tracks_) {
      const double cost = associationCost(track, cluster);
      if (!std::isfinite(cost)) {
        continue;
      }
      const int priority = trackPriority(track);
      const std::pair<int, double> candidate{priority, cost};
      if (priority == 3) {
        if (candidate < best_released) {
          best_released = candidate;
        }
      } else if (candidate < best_active) {
        best_active = candidate;
      }
    }
    if (best_active.first < 2) {
      return best_active;
    }
    // 2 表示没有活动轨迹可接；它排在 released-static 碎片之前，确保首次出现
    // 的球不会被大量历史静态候选挤出每帧处理上限。
    if (std::isfinite(best_released.second)) {
      return best_released;
    }
    return std::pair<int, double>{2, std::numeric_limits<double>::infinity()};
  };
  std::stable_sort(clusters.begin(), clusters.end(), [&](const CorridorCluster& lhs,
                                                         const CorridorCluster& rhs) {
    const auto lhs_match = bestAssociation(lhs);
    const auto rhs_match = bestAssociation(rhs);
    if (lhs_match.first != rhs_match.first) {
      return lhs_match.first < rhs_match.first;
    }
    if (std::isfinite(lhs_match.second) != std::isfinite(rhs_match.second)) {
      return std::isfinite(lhs_match.second);
    }
    if (std::isfinite(lhs_match.second) && lhs_match.second != rhs_match.second) {
      return lhs_match.second < rhs_match.second;
    }
    return lhs.indices.size() > rhs.indices.size();
  });

  const auto ageTracks = [&]() {
    for (auto& track : corridor_tracks_) {
      ++track.missed_frames;
    }
    corridor_tracks_.erase(
        std::remove_if(corridor_tracks_.begin(), corridor_tracks_.end(),
                       [&](const CorridorTrack& track) {
                         return track.missed_frames > static_cast<std::size_t>(
                                    config_.corridor_internal_max_missed_frames) ||
                                (stamp - track.last_seen).toSec() >
                                    config_.corridor_internal_track_timeout;
                       }),
        corridor_tracks_.end());
  };

  // 所有紧凑未知簇先从静态积分中隔离。候选很多时不再“全部拒绝”，而是
  // 优先处理能续上已有轨迹的簇，再处理点数较多的簇；其余簇保持 holdout。
  for (const auto& cluster : clusters) {
    for (const std::size_t index : cluster.indices) {
      result.holdout_indices[index] = true;
    }
  }
  if (clusters.empty()) {
    ageTracks();
    return result;
  }
  const std::size_t process_limit = static_cast<std::size_t>(
      std::max(config_.corridor_max_candidates,
               config_.corridor_reject_candidate_count));
  if (clusters.size() > process_limit) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "Corridor dynamic candidates limited: count=" << clusters.size()
                                                             << ", processed="
                                                             << process_limit);
    clusters.resize(process_limit);
  }
  std::vector<bool> matched_tracks(corridor_tracks_.size(), false);
  for (const auto& cluster : clusters) {
    // 每个紧凑候选在完成运动确认前都暂缓写入 UFOMap。只有 released_static
    // 的稳定内部簇会在下面清除 holdout；这样候选排序/截断不会把真正的球
    // 静默地写死成静态占据。
    int best_track = -1;
    int best_priority = 4;
    double best_cost = std::numeric_limits<double>::infinity();
    for (std::size_t track_index = 0U; track_index < corridor_tracks_.size(); ++track_index) {
      if (matched_tracks[track_index]) {
        continue;
      }
      const auto& track = corridor_tracks_[track_index];
      const double cost = associationCost(track, cluster);
      const int priority = trackPriority(track);
      // 确认状态只作为有限的身份偏置，最终仍以几何/尺寸代价选择；
      // 这样既能续上摆球，也不会让旧 ID 无条件抢走附近静态碎片。
      if (std::isfinite(cost) &&
          (priority < best_priority || (priority == best_priority && cost < best_cost))) {
        best_priority = priority;
        best_cost = cost;
        best_track = static_cast<int>(track_index);
      }
    }

    CorridorTrack* track = nullptr;
    if (best_track < 0) {
      CorridorTrack new_track;
      new_track.id = next_corridor_track_id_++;
      new_track.center = cluster.center;
      new_track.size = cluster.size;
      new_track.hits = 1U;
      new_track.static_evidence_frames = 0U;
      new_track.last_seen = stamp;
      new_track.history.push_back(
          {stamp, cluster.center, cluster.raw_extent, cluster.raw_second_extent});
      corridor_tracks_.push_back(std::move(new_track));
      matched_tracks.push_back(true);
      track = &corridor_tracks_.back();
    } else {
      matched_tracks[static_cast<std::size_t>(best_track)] = true;
      track = &corridor_tracks_[static_cast<std::size_t>(best_track)];
      const ufo::Point previous_center = track->center;
      const double dt = (stamp - track->last_seen).toSec();
      track->size = cluster.size;
      track->missed_frames = 0U;
      track->last_seen = stamp;
      const ufo::Point center_delta(cluster.center.x - previous_center.x,
                                    cluster.center.y - previous_center.y,
                                    cluster.center.z - previous_center.z);
      const double lateral_displacement = std::abs(
          dotHorizontal(center_delta, lateral_x, lateral_y));
      bool reactivating = false;
      if (track->released_static) {
        if (lateral_displacement >= config_.corridor_reactivation_displacement) {
          ++track->reactivation_evidence_frames;
        } else {
          track->reactivation_evidence_frames = 0U;
        }
        // 单帧静态点云抖动不能重新激活轨迹；至少需要两次连续的
        // 横向位移证据，才重新交给 planner。
        reactivating = track->reactivation_evidence_frames >= 2U;
      }
      const bool stale_unconfirmed = !track->confirmed && !track->released_static &&
          (dt > 0.25 || track->history.empty());
      if (stale_unconfirmed) {
        // 未确认候选跨越较长空帧后重新出现，不能把两段无关噪声拼成一条
        // 运动轨迹；从当前观测重新开始命中和方向计数。
        track->hits = 0U;
        track->history.clear();
        track->lateral_direction_history.clear();
        track->velocity = ufo::Point(0.0F, 0.0F, 0.0F);
        track->static_evidence_frames = 0U;
        track->lateral_evidence_frames = 0U;
        track->reactivation_evidence_frames = 0U;
        track->motion_qualified = false;
        track->last_motion_qualified_stamp = ros::Time();
      }
      if (reactivating) {
        // 静态释放后的再次运动必须从当前观测重新积累证据，不能让旧历史速度
        // 直接把目标重新确认成动态。
        track->center = cluster.center;
        track->velocity = ufo::Point(0.0F, 0.0F, 0.0F);
        track->hits = 1U;
        track->confirmed = false;
        track->released_static = false;
        track->history.clear();
        track->history.push_back(
            {stamp, cluster.center, cluster.raw_extent, cluster.raw_second_extent});
        track->lateral_direction_history.clear();
        track->static_evidence_frames = 0U;
        track->lateral_evidence_frames = 0U;
        track->reactivation_evidence_frames = 0U;
        track->motion_qualified = false;
        track->last_motion_qualified_stamp = ros::Time();
      } else if (track->released_static) {
        // 保留横向静态锚点，同时跟随前向/z 位置更新关联中心；否则前向
        // 运动目标超过 gate 后会反复新建 provisional 轨迹。
        const double anchor_lateral = dotHorizontal(track->center, lateral_x, lateral_y);
        const double observed_lateral = dotHorizontal(cluster.center, lateral_x, lateral_y);
        track->center = cluster.center;
        const double lateral_correction = anchor_lateral - observed_lateral;
        track->center.x += static_cast<float>(lateral_correction * lateral_x);
        track->center.y += static_cast<float>(lateral_correction * lateral_y);
        ++track->hits;
      } else {
        track->center = cluster.center;
        ++track->hits;
      }

      if (!reactivating && !track->released_static &&
          (stale_unconfirmed || (dt > 1e-3 && dt < 1.0))) {
        track->history.push_back(
            {stamp, cluster.center, cluster.raw_extent, cluster.raw_second_extent});
        while (track->history.size() >
               static_cast<std::size_t>(config_.corridor_history_frames)) {
          track->history.pop_front();
        }
        std::vector<double> vx;
        std::vector<double> vy;
        std::vector<double> vz;
        for (std::size_t sample_index = 1U; sample_index < track->history.size(); ++sample_index) {
          const auto& previous = track->history[sample_index - 1U];
          const auto& current = track->history[sample_index];
          const double sample_dt = (current.stamp - previous.stamp).toSec();
          if (sample_dt <= 1e-3 || sample_dt > 1.0) {
            continue;
          }
          vx.push_back((static_cast<double>(current.center.x) - previous.center.x) / sample_dt);
          vy.push_back((static_cast<double>(current.center.y) - previous.center.y) / sample_dt);
          vz.push_back((static_cast<double>(current.center.z) - previous.center.z) / sample_dt);
        }
        if (!vx.empty()) {
          track->velocity = ufo::Point(static_cast<float>(medianValue(std::move(vx))),
                                       static_cast<float>(medianValue(std::move(vy))),
                                       static_cast<float>(medianValue(std::move(vz))));
        }

        const ufo::Point frame_velocity(
            static_cast<float>((static_cast<double>(cluster.center.x) - previous_center.x) / dt),
            static_cast<float>((static_cast<double>(cluster.center.y) - previous_center.y) / dt),
            static_cast<float>((static_cast<double>(cluster.center.z) - previous_center.z) / dt));
        const double frame_lateral_velocity = dotHorizontal(
            frame_velocity, lateral_x, lateral_y);
        const double frame_forward_velocity = dotHorizontal(
            frame_velocity, corridor_forward_x_, corridor_forward_y_);
        const double frame_horizontal_speed = std::hypot(
            frame_lateral_velocity, frame_forward_velocity);
        const bool frame_forward_aligned = frame_horizontal_speed >=
                config_.corridor_static_lateral_speed &&
            std::abs(frame_forward_velocity) / frame_horizontal_speed >=
                config_.corridor_forward_alignment_cos;
        int direction = 0;
        if (!frame_forward_aligned &&
            frame_lateral_velocity >= config_.corridor_min_lateral_speed) {
          direction = 1;
        } else if (!frame_forward_aligned &&
                   frame_lateral_velocity <= -config_.corridor_min_lateral_speed) {
          direction = -1;
        }
        track->lateral_direction_history.push_back(direction);
        while (track->lateral_direction_history.size() >
               static_cast<std::size_t>(config_.corridor_history_frames)) {
          track->lateral_direction_history.pop_front();
        }
      }
    }

    const double lateral_velocity = dotHorizontal(track->velocity, lateral_x, lateral_y);
    const double forward_velocity = dotHorizontal(track->velocity,
                                                  corridor_forward_x_, corridor_forward_y_);
    const double vertical_velocity = static_cast<double>(track->velocity.z);
    const double horizontal_speed = std::hypot(lateral_velocity, forward_velocity);
    const bool forward_aligned = horizontal_speed >=
            config_.corridor_static_lateral_speed &&
        std::abs(forward_velocity) / horizontal_speed >=
            config_.corridor_forward_alignment_cos;
    const bool lateral_dominant =
        std::abs(lateral_velocity) >= config_.corridor_min_lateral_speed &&
        !forward_aligned &&
        std::abs(vertical_velocity) <= config_.corridor_max_vertical_speed;
    const bool stationary_or_forward =
        std::abs(lateral_velocity) < config_.corridor_static_lateral_speed ||
        forward_aligned;
    std::size_t consecutive_direction = 0U;
    const int latest_direction = track->lateral_direction_history.empty()
        ? 0 : track->lateral_direction_history.back();
    if (latest_direction != 0) {
      for (auto it = track->lateral_direction_history.rbegin();
           it != track->lateral_direction_history.rend() && *it == latest_direction; ++it) {
        ++consecutive_direction;
      }
    }
    // 单次横向跳变不能作为真实摆动证据。只有最近两次方向连续时才累计
    // lateral evidence；交替抖动/方向不连续则按静止证据处理，允许已误确认
    // 的轨迹回退为静态。
    const bool continuous_lateral_motion = lateral_dominant &&
        consecutive_direction >= 2U;
    const bool provisional_lateral_motion = lateral_dominant &&
        consecutive_direction >= 3U;
    if (!track->released_static) {
      if (continuous_lateral_motion) {
        ++track->lateral_evidence_frames;
        track->static_evidence_frames = 0U;
      } else if (stationary_or_forward || lateral_dominant) {
        ++track->static_evidence_frames;
        track->lateral_evidence_frames = 0U;
      } else {
        track->static_evidence_frames = 0U;
        track->lateral_evidence_frames = 0U;
      }
    }
    const std::size_t positive_evidence = static_cast<std::size_t>(std::count(
        track->lateral_direction_history.begin(), track->lateral_direction_history.end(), 1));
    const std::size_t negative_evidence = static_cast<std::size_t>(std::count(
        track->lateral_direction_history.begin(), track->lateral_direction_history.end(), -1));
    const std::size_t dominant_evidence = std::max(positive_evidence, negative_evidence);
    const std::size_t contrary_evidence = std::min(positive_evidence, negative_evidence);
    const bool recent_direction_consistent = consecutive_direction >= 4U &&
        dominant_evidence >= 4U && dominant_evidence > contrary_evidence;
    double lateral_span = 0.0;
    if (track->history.size() >= 2U) {
      double minimum_lateral = std::numeric_limits<double>::infinity();
      double maximum_lateral = -std::numeric_limits<double>::infinity();
      for (const auto& sample : track->history) {
        const double sample_lateral = dotHorizontal(sample.center, lateral_x, lateral_y);
        minimum_lateral = std::min(minimum_lateral, sample_lateral);
        maximum_lateral = std::max(maximum_lateral, sample_lateral);
      }
      lateral_span = maximum_lateral - minimum_lateral;
    }
    const bool strong_lateral_motion =
        track->hits >= static_cast<std::size_t>(config_.corridor_min_confirm_hits) &&
        track->lateral_evidence_frames >= 2U &&
        recent_direction_consistent &&
        lateral_span >= config_.corridor_min_lateral_span &&
        continuous_lateral_motion &&
        std::abs(forward_velocity) <= config_.corridor_max_forward_speed &&
        std::abs(vertical_velocity) <= config_.corridor_max_vertical_speed;
    if (!track->released_static && strong_lateral_motion) {
      track->motion_qualified = true;
      track->last_motion_qualified_stamp = stamp;
    }
    const std::size_t recent_extent_evidence = static_cast<std::size_t>(std::count_if(
        track->history.begin(), track->history.end(), [&](const CorridorTrackSample& sample) {
          return sample.raw_extent >= config_.corridor_min_confirm_extent &&
              sample.raw_second_extent >= config_.corridor_min_confirm_second_extent;
        }));
    const bool recent_motion_qualified = !track->history.empty() &&
        !track->last_motion_qualified_stamp.isZero() &&
        track->last_motion_qualified_stamp >= track->history.front().stamp;
    if (!track->confirmed && !track->released_static &&
        track->motion_qualified &&
        recent_motion_qualified &&
        recent_extent_evidence >=
            static_cast<std::size_t>(config_.corridor_min_confirm_extent_frames)) {
      track->confirmed = true;
      ROS_INFO_STREAM("Corridor dynamic track confirmed: id=" << track->id
                      << ", hits=" << track->hits
                      << ", lateral_speed=" << lateral_velocity
                      << ", lateral_span=" << lateral_span
                      << ", extent_shape_frames=" << recent_extent_evidence
                      << ", raw_second_extent=" << cluster.raw_second_extent
                      << ", center=(" << track->center.x << "," << track->center.y
                      << "," << track->center.z << ")"
                      << ", size=(" << track->size.x << "," << track->size.y
                      << "," << track->size.z << ")");
    }

    const bool unresolved_too_long =
        !track->motion_qualified &&
        track->hits >= 2U * static_cast<std::size_t>(config_.corridor_history_frames) &&
        !recent_direction_consistent;
    const std::size_t static_release_frames = static_cast<std::size_t>(
        (track->confirmed || track->motion_qualified)
            ? config_.corridor_confirmed_static_confirm_frames
            : config_.corridor_static_confirm_frames);
    if (!track->released_static &&
        (track->static_evidence_frames >= static_release_frames ||
         (!track->confirmed && unresolved_too_long))) {
      // 连续无横向速度，或速度近似沿无人机前后方向时，才把未知候选释放为
      // 静态。之后只有足够大的横向位移才能重新激活。
      track->released_static = true;
      track->confirmed = false;
      track->velocity = ufo::Point(0.0F, 0.0F, 0.0F);
      track->history.clear();
      track->history.push_back({stamp, track->center, 0.0, 0.0});
      track->lateral_direction_history.clear();
      track->static_evidence_frames = 0U;
      track->lateral_evidence_frames = 0U;
      track->reactivation_evidence_frames = 0U;
      track->motion_qualified = false;
      track->last_motion_qualified_stamp = ros::Time();
      ROS_INFO_STREAM("Corridor candidate released static: id=" << track->id
                      << ", hits=" << track->hits
                      << ", lateral_speed=" << lateral_velocity
                      << ", forward_speed=" << forward_velocity);
    }

    if (!track->released_static) {
      result.candidate_point_count += cluster.indices.size();
      const bool has_publish_history =
          track->hits >= static_cast<std::size_t>(config_.corridor_min_publish_hits);
      // 未确认点簇仅凭多帧命中仍可能是静态结构的采样抖动。至少观察到
      // 连续同向的横向运动后才发布 provisional；确认轨迹继续正常输出。
      const bool provisional_motion_ready = provisional_lateral_motion ||
          track->motion_qualified;
      const bool publish_current = cluster.indices.size() >=
              static_cast<std::size_t>(config_.corridor_min_cluster_points) &&
          (track->confirmed ||
           (config_.corridor_publish_unknown_as_dynamic && has_publish_history &&
            provisional_motion_ready));
      if (publish_current &&
          published_track_count < static_cast<std::size_t>(config_.corridor_max_candidates)) {
        for (const std::size_t index : cluster.indices) {
          result.dynamic_indices[index] = true;
          result.provisional_indices[index] = !track->confirmed;
          result.source_track_ids[index] = track->id;
          result.measured_velocities[index] = track->velocity;
          result.holdout_indices[index] = false;
        }
        if (track->confirmed) {
          result.confirmed_point_count += cluster.indices.size();
        }
        ++published_track_count;
      }
    } else {
      // 连续多帧稳定且没有运动证据的簇才重新允许写入静态图。
      for (const std::size_t index : cluster.indices) {
        result.holdout_indices[index] = false;
        result.released_static_indices[index] = true;
      }
    }
  }

  for (std::size_t track_index = 0U; track_index < matched_tracks.size(); ++track_index) {
    if (!matched_tracks[track_index]) {
      ++corridor_tracks_[track_index].missed_frames;
    }
  }
  corridor_tracks_.erase(
      std::remove_if(corridor_tracks_.begin(), corridor_tracks_.end(),
                     [&](const CorridorTrack& track) {
                       return track.missed_frames > static_cast<std::size_t>(
                                  config_.corridor_internal_max_missed_frames) ||
                              (stamp - track.last_seen).toSec() >
                                  config_.corridor_internal_track_timeout;
                     }),
      corridor_tracks_.end());

  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(
        1.0, "Corridor detector: wall=" << (result.wall_valid ? "valid" : "invalid")
                                         << ", clusters=" << clusters.size()
                                         << ", candidate_points=" << result.candidate_point_count
                                         << ", confirmed_points=" << result.confirmed_point_count
                                         << ", tracks=" << corridor_tracks_.size());
  }
  return result;
}

void UfomapMapper::updatePreviousFrameSnapshot(const UfomapPointCloud& points) {
  previous_frame_points_.clear();
  previous_frame_points_.reserve(points.size());
  for (const auto& point : points) {
    previous_frame_points_.emplace_back(point.x, point.y, point.z);
  }
}

UfomapFrameResult UfomapMapper::processInputCloud(const sensor_msgs::PointCloud2& cloud_msg,
                                                  const nav_msgs::Odometry& odom_msg) {
  const auto total_start = std::chrono::steady_clock::now();
  UfomapFrameResult result;
  ++processed_frame_count_;
  const auto extract_start = std::chrono::steady_clock::now();
  const ExtractFinitePointsResult extracted = extractFinitePoints(cloud_msg);
  const auto extract_end = std::chrono::steady_clock::now();
  result.timing.extract_finite_points_ms = elapsedMs(extract_start, extract_end);

  const ufo::Point sensor_origin(static_cast<float>(odom_msg.pose.pose.position.x),
                                static_cast<float>(odom_msg.pose.pose.position.y),
                                static_cast<float>(odom_msg.pose.pose.position.z));
  const auto classify_current_start = std::chrono::steady_clock::now();
  // 当前帧输出只关心配置工作半径内的点，避免对远处点做无意义的 seenFree 查询。
  const ufo::PointCloud* input_points = &extracted.points;
  ufo::PointCloud filtered_input_points;
  if (config_.input_max_z > 0.0) {
    filtered_input_points.reserve(extracted.points.size());
    const float input_max_z = static_cast<float>(config_.input_max_z);
    for (const auto& point : extracted.points) {
      if (point.z < input_max_z) {
        filtered_input_points.push_back(point);
      }
    }
    input_points = &filtered_input_points;
  }

  const ufo::PointCloud* range_filtered_points = input_points;
  ufo::PointCloud filtered_output_points;
  if (config_.max_range > 0.0) {
    const float max_range_sq = static_cast<float>(config_.max_range * config_.max_range);
    filtered_output_points.reserve(input_points->size());
    for (const auto& point : *input_points) {
      if (sensor_origin.squaredDistance(point) <= max_range_sq) {
        filtered_output_points.push_back(point);
      }
    }
    range_filtered_points = &filtered_output_points;
  }

  updateGroundPlane(*range_filtered_points, sensor_origin);
  std::vector<std::size_t> output_source_indices;
  const ufo::PointCloud ground_filtered_points = filterGroundPoints(
      *range_filtered_points, &output_source_indices);
  const ufo::PointCloud* output_points = &ground_filtered_points;
  // 通道层必须看到地面滤波之前的原始回波，否则地面估计偏高时会先把摆球
  // 删除。静态 UFOMap 接收 ground_filtered_points，以及状态机明确释放的 raw-only
  // 静态点；不会把全量地面重新写入地图。
  const CorridorCandidateResult corridor_candidates = detectCorridorCandidates(
      *range_filtered_points, sensor_origin, odom_msg, cloud_msg.header.stamp);

  // filterGroundPoints() 保序返回原始点子集，因此可以按原始索引精确复制掩码，
  // 不做空间邻域膨胀，避免把球附近的墙点或静态点误标为动态/暂存。
  std::vector<bool> corridor_handled_output(output_points->size(), false);
  std::vector<bool> corridor_dynamic_output(output_points->size(), false);
  std::vector<bool> corridor_provisional_output(output_points->size(), false);
  std::vector<std::uint32_t> corridor_source_track_ids_output(output_points->size(), 0U);
  std::vector<ufo::Point> corridor_measured_velocities_output(
      output_points->size(), ufo::Point(0.0F, 0.0F, 0.0F));
  std::vector<bool> corridor_holdout_output(output_points->size(), false);
  std::vector<bool> corridor_dynamic_raw_kept(range_filtered_points->size(), false);
  std::vector<bool> corridor_raw_kept(range_filtered_points->size(), false);
  for (std::size_t output_index = 0U;
       output_index < output_source_indices.size(); ++output_index) {
    const std::size_t source_index = output_source_indices[output_index];
    if (source_index >= corridor_candidates.handled_indices.size()) {
      continue;
    }
    corridor_handled_output[output_index] =
        corridor_candidates.handled_indices[source_index];
    corridor_dynamic_output[output_index] =
        corridor_candidates.dynamic_indices[source_index];
    corridor_provisional_output[output_index] =
        corridor_candidates.provisional_indices[source_index];
    corridor_source_track_ids_output[output_index] =
        corridor_candidates.source_track_ids[source_index];
    corridor_measured_velocities_output[output_index] =
        corridor_candidates.measured_velocities[source_index];
    corridor_holdout_output[output_index] =
        corridor_candidates.holdout_indices[source_index];
    corridor_dynamic_raw_kept[source_index] = corridor_dynamic_output[output_index];
    corridor_raw_kept[source_index] = true;
  }
  std::vector<std::size_t> corridor_released_static_raw_indices;
  corridor_released_static_raw_indices.reserve(range_filtered_points->size());
  for (std::size_t source_index = 0U;
       source_index < corridor_candidates.released_static_indices.size(); ++source_index) {
    if (!corridor_candidates.released_static_indices[source_index] ||
        corridor_candidates.dynamic_indices[source_index] ||
        corridor_candidates.holdout_indices[source_index] ||
        corridor_raw_kept[source_index]) {
      continue;
    }
    corridor_released_static_raw_indices.push_back(source_index);
  }
  const std::vector<std::size_t> temporal_motion_indices =
      detectTemporalMotion(*output_points, &corridor_handled_output);
  updatePreviousFrameSnapshot(*output_points);

  result.classification = classifyPoints(cloud_msg.header, *output_points);
  const bool warmup_ready =
      processed_frame_count_ > static_cast<std::uint64_t>(config_.warmup_frames);
  std::vector<bool> dynamic_mask(output_points->size(), false);
  if (warmup_ready) {
    for (const std::size_t index : result.classification.dynamic_indices) {
      if (index < dynamic_mask.size() &&
          (index >= corridor_handled_output.size() || !corridor_handled_output[index])) {
        dynamic_mask[index] = true;
      }
    }
    for (const std::size_t index : temporal_motion_indices) {
      if (index < dynamic_mask.size()) {
        dynamic_mask[index] = true;
      }
    }
  }
  for (std::size_t index = 0U; index < corridor_dynamic_output.size(); ++index) {
    if (corridor_dynamic_output[index]) {
      dynamic_mask[index] = true;
    }
  }

  auto rebuildClassificationOutputs = [&]() {
    auto& classification = result.classification;
    classification.static_points.clear();
    classification.dynamic_points.clear();
    classification.dynamic_indices.clear();
    classification.dynamic_cluster_points.clear();

    ufo::PointCloud static_cloud_points;
    ufo::PointCloud dynamic_cloud_points;
    static_cloud_points.reserve(output_points->size() +
                                corridor_released_static_raw_indices.size());
    dynamic_cloud_points.reserve(output_points->size());
    classification.static_points.reserve(output_points->size() +
                                         corridor_released_static_raw_indices.size());
    classification.dynamic_points.reserve(output_points->size());
    classification.dynamic_indices.reserve(output_points->size());
    classification.dynamic_cluster_points.reserve(output_points->size());

    {
      std::shared_lock<std::shared_mutex> lock(map_mutex_);
      const auto& runtime = runtimeLocked();
      for (std::size_t index = 0U; index < output_points->size(); ++index) {
        const auto& point = (*output_points)[index];
        const ufo::Point point_value(point.x, point.y, point.z);
        const bool corridor_holdout = index < corridor_holdout_output.size() &&
                                      corridor_holdout_output[index];
        if (!dynamic_mask[index]) {
          // 临时候选既不是当前帧已确认动态，也不是可写入/可显示的静态点。
          // 将它从静态输出中省略，避免 RViz 看起来像“已经静态化”。
          if (corridor_holdout) {
            continue;
          }
          static_cloud_points.push_back(point);
          classification.static_points.push_back(point_value);
          continue;
        }

        dynamic_cloud_points.push_back(point);
        classification.dynamic_points.push_back(point_value);
        classification.dynamic_indices.push_back(index);
        UfomapDynamicClusterPoint cluster_point;
        cluster_point.point = point;
        if (index < corridor_dynamic_output.size() && corridor_dynamic_output[index]) {
          cluster_point.use_detector_voxel = true;
          cluster_point.detector_voxel_code = makeDetectorVoxelCode(point);
          cluster_point.realtime_only = true;
          cluster_point.provisional = index < corridor_provisional_output.size() &&
                                      corridor_provisional_output[index];
          cluster_point.corridor_source_track_id =
              index < corridor_source_track_ids_output.size()
                  ? corridor_source_track_ids_output[index]
                  : 0U;
          cluster_point.measured_velocity =
              index < corridor_measured_velocities_output.size()
                  ? corridor_measured_velocities_output[index]
                  : ufo::Point(0.0F, 0.0F, 0.0F);
          cluster_point.measured_velocity_valid =
              cluster_point.corridor_source_track_id != 0U;
        } else {
          const auto code = runtime.map.toCodeChecked(point);
          if (!code.has_value()) {
            continue;
          }
          const ufo::Key key = *code;
          cluster_point.voxel_code.raw_code = code->raw();
          cluster_point.voxel_code.key_x = key.x();
          cluster_point.voxel_code.key_y = key.y();
          cluster_point.voxel_code.key_z = key.z();
          cluster_point.voxel_code.depth = key.depth();
        }
        classification.dynamic_cluster_points.push_back(cluster_point);
      }
    }

    // 地面滤波可能误删低于估计平面的真实障碍。只有状态机明确释放为静态、
    // 且没有出现在 ground-filtered 输出中的原始点才补回；普通地面、holdout
    // 和仍为动态的点不会进入该列表。
    for (const std::size_t source_index : corridor_released_static_raw_indices) {
      const auto& source_point = (*range_filtered_points)[source_index];
      static_cloud_points.push_back(source_point);
      classification.static_points.emplace_back(
          source_point.x, source_point.y, source_point.z);
    }

    // 地面滤波可能已经移除了摆球回波；确认后的 raw-only 点仍直接进入动态
    // 输出，但不回写 UFOMap。按原始索引去重，保留同一体素内的全部真实回波。
    for (std::size_t source_index = 0U;
         source_index < corridor_candidates.dynamic_indices.size(); ++source_index) {
      if (!corridor_candidates.dynamic_indices[source_index] ||
          corridor_dynamic_raw_kept[source_index]) {
        continue;
      }
      const auto& source_point = (*range_filtered_points)[source_index];
      const ufo::Point point(source_point.x, source_point.y, source_point.z);
      dynamic_cloud_points.push_back(source_point);
      classification.dynamic_points.push_back(point);
      UfomapDynamicClusterPoint cluster_point;
      cluster_point.point = point;
      cluster_point.use_detector_voxel = true;
      cluster_point.detector_voxel_code = makeDetectorVoxelCode(point);
      cluster_point.realtime_only = true;
      cluster_point.provisional = source_index < corridor_candidates.provisional_indices.size() &&
                                  corridor_candidates.provisional_indices[source_index];
      cluster_point.corridor_source_track_id =
          source_index < corridor_candidates.source_track_ids.size()
              ? corridor_candidates.source_track_ids[source_index]
              : 0U;
      cluster_point.measured_velocity =
          source_index < corridor_candidates.measured_velocities.size()
              ? corridor_candidates.measured_velocities[source_index]
              : ufo::Point(0.0F, 0.0F, 0.0F);
      cluster_point.measured_velocity_valid = cluster_point.corridor_source_track_id != 0U;
      classification.dynamic_cluster_points.push_back(cluster_point);
    }

    classification.input_point_count = output_points->size();
    classification.static_point_count = classification.static_points.size();
    classification.dynamic_point_count = classification.dynamic_points.size();
    classification.static_cloud_msg = buildCloudFromPoints(cloud_msg.header, static_cloud_points);
    classification.dynamic_cloud_msg =
        buildCloudFromPoints(cloud_msg.header, dynamic_cloud_points);
  };

  rebuildClassificationOutputs();
  const auto classify_current_end = std::chrono::steady_clock::now();
  result.timing.classify_current_frame_ms =
      elapsedMs(classify_current_start, classify_current_end);

  if (extracted.has_invalid_points) {
    result.classification.static_cloud_msg.is_dense = false;
    result.classification.dynamic_cloud_msg.is_dense = false;
  }
  // 当前阶段主流程输出当前帧静态/动态点云；静态体素 marker 在地图更新后按当前帧静态点生成。
  result.static_map_cloud_msg = result.classification.static_cloud_msg;
  result.dynamic_cloud_msg = result.classification.dynamic_cloud_msg;

  UfomapPointCloud integration_points;
  integration_points.reserve(output_points->size() +
                             corridor_released_static_raw_indices.size());
  for (std::size_t index = 0U; index < output_points->size(); ++index) {
    const bool corridor_holdout = index < corridor_holdout_output.size() &&
                                  corridor_holdout_output[index];
    if (!dynamic_mask[index] && !corridor_holdout) {
      integration_points.push_back((*output_points)[index]);
    }
  }
  for (const std::size_t source_index : corridor_released_static_raw_indices) {
    integration_points.push_back((*range_filtered_points)[source_index]);
  }

  const auto update_start = std::chrono::steady_clock::now();
  if (!integration_points.empty()) {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    auto& runtime = runtimeLocked();
    // 积分与统计更新在同一把锁内，保证 frame_result 与地图状态一致。
    ufo::insertPointCloud(runtime.map, integration_points, sensor_origin,
                          runtime.integration_params,
                          runtime.integration_params.propagate);

    const std::uint64_t update_count = ufomap_update_count_.fetch_add(1U) + 1U;
    result.runtime_stats.update_count = update_count;
  }
  const auto update_end = std::chrono::steady_clock::now();
  result.timing.update_ufomap_ms = elapsedMs(update_start, update_end);

  {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    result.runtime_stats.update_count = ufomap_update_count_.load();
    result.runtime_stats.node_count = runtimeLocked().map.numNodes();
  }
  result.runtime_stats.processed_frame_count = processed_frame_count_;
  result.runtime_stats.ground_plane_ready = ground_plane_.has_value();
  result.runtime_stats.ground_plane_z = ground_plane_.has_value()
                                            ? ground_plane_->intercept
                                            : 0.0;
  result.runtime_stats.ground_plane_slope_x = ground_plane_.has_value()
                                                 ? ground_plane_->slope_x
                                                 : 0.0;
  result.runtime_stats.ground_plane_slope_y = ground_plane_.has_value()
                                                 ? ground_plane_->slope_y
                                                 : 0.0;
  result.runtime_stats.warmup_ready = warmup_ready;
  result.runtime_stats.temporal_motion_point_count = temporal_motion_indices.size();
  result.runtime_stats.corridor_candidate_point_count =
      corridor_candidates.candidate_point_count;
  result.runtime_stats.corridor_confirmed_point_count =
      corridor_candidates.confirmed_point_count;
  result.runtime_stats.corridor_wall_valid = corridor_candidates.wall_valid;

  const double static_map_visualization_max_z = config_.static_map_visualization_max_z;
  StaticMapVisualizationSnapshot static_map_snapshot;
  {
    // 只读取当前帧静态点对应的体素编码，不遍历持久化地图中的历史占据体素。
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    static_map_snapshot = captureStaticMapVisualizationSnapshot(
        runtimeLocked().map,
        result.classification.static_points,
        static_map_visualization_max_z);
  }
  result.static_map_markers = buildStaticMapVisualization(
      cloud_msg.header, std::move(static_map_snapshot));

  result.timing.process_input_cloud_ms = elapsedMs(total_start, std::chrono::steady_clock::now());
  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Ufomap timing"
                                      << ": processInputCloud="
                                      << formatFloatMs(result.timing.process_input_cloud_ms)
                                      << "ms, extractFinitePoints="
                                      << formatFloatMs(result.timing.extract_finite_points_ms)
                                      << "ms, updateUfomap="
                                      << formatFloatMs(result.timing.update_ufomap_ms)
                                      << "ms, classifyCurrentFrame="
                                      << formatFloatMs(
                                              result.timing.classify_current_frame_ms)
                                      << "ms"
                                      << ", groundPlane="
                                      << (ground_plane_.has_value()
                                              ? (std::to_string(ground_plane_->slope_x) +
                                                 "*x+" +
                                                 std::to_string(ground_plane_->slope_y) +
                                                 "*y+" +
                                                 std::to_string(ground_plane_->intercept))
                                              : std::string("unknown"))
                                      << ", temporalMotionPoints="
                                      << temporal_motion_indices.size()
                                      << ", corridorCandidatePoints="
                                      << corridor_candidates.candidate_point_count
                                      << ", corridorConfirmedPoints="
                                      << corridor_candidates.confirmed_point_count
                                      << ", warmupReady=" << (warmup_ready ? "true" : "false"));
  }
  return result;
}

void UfomapMapper::reserveMap(const std::size_t node_capacity) {
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  runtimeLocked().map.reserve(node_capacity);
}

void UfomapMapper::clearMap() {
  std::unique_lock<std::shared_mutex> map_lock(map_mutex_);
  configureUfomap();
}

void UfomapMapper::propagateModified() {
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  runtimeLocked().map.propagateModified();
}

bool UfomapMapper::seenFree(const ufo::Point& point) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  return runtimeLocked().map.seenFree(point);
}

UfomapNodeQueryResult UfomapMapper::queryNode(const ufo::Point& point,
                                              const ufo::depth_t depth) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  const auto& runtime = runtimeLocked();
  const ufo::depth_t query_depth = clampQueryDepth(runtime.map, depth);

  UfomapNodeQueryResult result;
  result.depth = query_depth;
  result.voxel_size = runtime.map.size(query_depth);

  const auto code = runtime.map.toCodeChecked(point, query_depth);
  if (!code.has_value()) {
    result.center = point;
    applyOccupancyStateFlags(UfomapOccupancyState::OutOfMap, result);
    return result;
  }

  result.center = runtime.map.toCoord(*code);
  result.seen_free = runtime.map.seenFree(point, query_depth);
  if (!runtime.map.exists(*code)) {
    if (result.seen_free) {
      // 对规划层而言，射线穿越证据已经足以表达 FREE；
      // 即使 exact code 未物化为普通节点，也不应退化成 UNKNOWN。
      result.exists = true;
      applyOccupancyStateFlags(UfomapOccupancyState::Free, result);
    } else {
      applyOccupancyStateFlags(UfomapOccupancyState::Unknown, result);
    }
    return result;
  }

  const auto node = runtime.map(*code);

  result.exists = true;
  result.seen_free = runtime.map.seenFree(node);
  applyOccupancyStateFlags(classifyPlannerOccupancyState(runtime.map, node), result);
  result.depth = node.depth();
  result.voxel_size = runtime.map.size(node.depth());
  return result;
}

void UfomapMapper::setNodeLabel(const ufo::Point& point,
                                const std::uint32_t label,
                                const bool propagate) {
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  runtimeLocked().map.setLabel(point, static_cast<ufo::label_t>(label), propagate);
}

std::uint32_t UfomapMapper::nodeLabel(const ufo::Point& point) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  return static_cast<std::uint32_t>(runtimeLocked().map.label(point));
}

void UfomapMapper::clearAllLabels() {
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  auto& runtime = runtimeLocked();

  for (auto it = runtime.map.begin(); it != runtime.map.end(); ++it) {
    runtime.map.setLabel(*it, 0U);
  }
}

UfomapClassificationResult UfomapMapper::classifyPoints(
    const std_msgs::Header& header,
    const UfomapMapper::UfomapPointCloud& points) const {
  UfomapClassificationResult result;
  ufo::PointCloud static_points;
  ufo::PointCloud dynamic_points;
  static_points.reserve(points.size());
  dynamic_points.reserve(points.size());
  result.static_points.reserve(points.size());
  result.dynamic_points.reserve(points.size());
  result.dynamic_indices.reserve(points.size());
  result.dynamic_cluster_points.reserve(points.size());

  {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    result.input_point_count = points.size();
    const auto& runtime = runtimeLocked();
    std::size_t point_index = 0U;
    for (const auto& point : points) {
      if (runtime.map.seenFree(point)) {
        dynamic_points.push_back(point);
        result.dynamic_points.emplace_back(point.x, point.y, point.z);
        result.dynamic_indices.push_back(point_index);
        const auto code = runtime.map.toCodeChecked(point);
        if (code.has_value()) {
          const ufo::Key key = *code;
          UfomapDynamicClusterPoint cluster_point;
          cluster_point.point = point;
          cluster_point.voxel_code.raw_code = code->raw();
          cluster_point.voxel_code.key_x = key.x();
          cluster_point.voxel_code.key_y = key.y();
          cluster_point.voxel_code.key_z = key.z();
          cluster_point.voxel_code.depth = key.depth();
          result.dynamic_cluster_points.push_back(cluster_point);
        }
      } else {
        static_points.push_back(point);
        result.static_points.emplace_back(point.x, point.y, point.z);
      }
      ++point_index;
    }
  }

  result.static_point_count = static_points.size();
  result.dynamic_point_count = dynamic_points.size();
  result.static_cloud_msg = buildCloudFromPoints(header, static_points);
  result.dynamic_cloud_msg = buildCloudFromPoints(header, dynamic_points);
  return result;
}

std::vector<UfomapCandidateNode> UfomapMapper::queryDynamicCandidates(
    const UfomapCandidateQuery& query) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  std::vector<UfomapCandidateNode> candidates;
  const auto& runtime = runtimeLocked();

  const auto leaf = ufo::pred::Leaf(clampQueryDepth(runtime.map, query.depth));
  const auto hits = ufo::pred::HitsMin(static_cast<ufo::count_t>(query.min_hits));
  const auto label = ufo::pred::Label(static_cast<ufo::label_t>(query.label));
  // intersects 为空时走全局查询；非空时附加空间约束查询。
  if (query.intersects.empty()) {
    if (query.require_seen_free) {
      appendCandidateNodes(runtime.map, leaf && ufo::pred::SeenFree() && hits && label,
                           query.max_results, candidates);
    } else {
      appendCandidateNodes(runtime.map, leaf && hits && label, query.max_results, candidates);
    }
    return candidates;
  }

  if (query.require_seen_free) {
    appendCandidateNodes(
        runtime.map,
        leaf && ufo::pred::SeenFree() && hits && label && ufo::pred::Intersects(query.intersects),
        query.max_results, candidates);
  } else {
    appendCandidateNodes(runtime.map,
                         leaf && hits && label && ufo::pred::Intersects(query.intersects),
                         query.max_results, candidates);
  }
  return candidates;
}

std::vector<UfomapLocalOccupiedNode> UfomapMapper::queryLocalOccupied(
    const UfomapLocalOccupiedQuery& query) const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  std::vector<UfomapLocalOccupiedNode> occupied_nodes;
  const auto& runtime = runtimeLocked();

  const auto leaf = ufo::pred::Leaf(clampQueryDepth(runtime.map, query.depth));
  const auto hits = ufo::pred::HitsMin(static_cast<ufo::count_t>(query.min_hits));
  // P3 只需要静态占据证据；这里不叠加 label/SeenFree，避免把动态候选语义泄漏给规划侧。
  if (query.intersects.empty()) {
    appendLocalOccupiedNodes(runtime.map, leaf && hits, query.max_results, occupied_nodes);
    return occupied_nodes;
  }

  appendLocalOccupiedNodes(runtime.map,
                           leaf && hits && ufo::pred::Intersects(query.intersects),
                           query.max_results,
                           occupied_nodes);
  return occupied_nodes;
}

std::optional<UfomapLocalOccupiedNode> UfomapMapper::queryNearestOccupied(
    const ufo::Point& center,
    const double radius,
    const ufo::depth_t depth,
    const std::uint32_t min_hits) const {
  if (radius < 0.0) {
    return std::nullopt;
  }

  UfomapLocalOccupiedQuery query;
  query.depth = depth;
  query.min_hits = min_hits;
  query.intersects.emplace_back(center, static_cast<float>(radius));
  const auto occupied_nodes = queryLocalOccupied(query);

  std::optional<UfomapLocalOccupiedNode> nearest;
  double nearest_surface_distance = std::numeric_limits<double>::infinity();
  for (const auto& occupied : occupied_nodes) {
    const double surface_distance =
        static_cast<double>(center.distance(occupied.center)) -
        std::max(0.0, occupied.voxel_size) * 0.5;
    if (surface_distance > radius) {
      continue;
    }
    if (surface_distance < nearest_surface_distance) {
      nearest_surface_distance = surface_distance;
      nearest = occupied;
    }
  }
  return nearest;
}

}  // namespace ldopcore
