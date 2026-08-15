#include <ldop/ufomap_mapper.h>
#include <ldop/utils.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
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

sensor_msgs::PointCloud2 buildCloudFromPointVector(const std_msgs::Header& header,
                                                   const std::vector<ufo::Point>& points) {
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
    const UfomapPointCloud& points) const {
  if (!config_.ground_filter_enabled || !ground_plane_.has_value()) {
    return points;
  }

  UfomapPointCloud filtered_points;
  filtered_points.reserve(points.size());
  for (const auto& point : points) {
    // 只删除拟合地面及以下的点；地面以上的点全部保留，不使用对称高度带。
    const double ground_height = ground_plane_->heightAt(point.x, point.y);
    if (static_cast<double>(point.z) > ground_height) {
      filtered_points.push_back(point);
    }
  }
  return filtered_points;
}

std::vector<std::size_t> UfomapMapper::detectTemporalMotion(
    const UfomapPointCloud& points) {
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
  const ufo::PointCloud ground_filtered_points = filterGroundPoints(*range_filtered_points);
  const ufo::PointCloud* output_points = &ground_filtered_points;
  const std::vector<std::size_t> temporal_motion_indices =
      detectTemporalMotion(*output_points);
  updatePreviousFrameSnapshot(*output_points);

  result.classification = classifyPoints(cloud_msg.header, *output_points);
  std::vector<bool> dynamic_mask(output_points->size(), false);
  for (const std::size_t index : result.classification.dynamic_indices) {
    if (index < dynamic_mask.size()) {
      dynamic_mask[index] = true;
    }
  }
  for (const std::size_t index : temporal_motion_indices) {
    if (index < dynamic_mask.size()) {
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
    static_cloud_points.reserve(output_points->size());
    dynamic_cloud_points.reserve(output_points->size());
    classification.static_points.reserve(output_points->size());
    classification.dynamic_points.reserve(output_points->size());
    classification.dynamic_indices.reserve(output_points->size());
    classification.dynamic_cluster_points.reserve(output_points->size());

    {
      std::shared_lock<std::shared_mutex> lock(map_mutex_);
      const auto& runtime = runtimeLocked();
      for (std::size_t index = 0U; index < output_points->size(); ++index) {
        const auto& point = (*output_points)[index];
        const ufo::Point point_value(point.x, point.y, point.z);
        if (!dynamic_mask[index]) {
          static_cloud_points.push_back(point);
          classification.static_points.push_back(point_value);
          continue;
        }

        dynamic_cloud_points.push_back(point);
        classification.dynamic_points.push_back(point_value);
        classification.dynamic_indices.push_back(index);
        const auto code = runtime.map.toCodeChecked(point);
        if (!code.has_value()) {
          continue;
        }
        const ufo::Key key = *code;
        UfomapDynamicClusterPoint cluster_point;
        cluster_point.point = point;
        cluster_point.voxel_code.raw_code = code->raw();
        cluster_point.voxel_code.key_x = key.x();
        cluster_point.voxel_code.key_y = key.y();
        cluster_point.voxel_code.key_z = key.z();
        cluster_point.voxel_code.depth = key.depth();
        classification.dynamic_cluster_points.push_back(cluster_point);
      }
    }

    classification.input_point_count = output_points->size();
    classification.static_point_count = classification.static_points.size();
    classification.dynamic_point_count = classification.dynamic_points.size();
    classification.static_cloud_msg = buildCloudFromPoints(cloud_msg.header, static_cloud_points);
    classification.dynamic_cloud_msg =
        buildCloudFromPoints(cloud_msg.header, dynamic_cloud_points);
  };

  rebuildClassificationOutputs();
  const bool warmup_ready =
      processed_frame_count_ > static_cast<std::uint64_t>(config_.warmup_frames);
  if (!warmup_ready) {
    result.classification.dynamic_points.clear();
    result.classification.dynamic_indices.clear();
    result.classification.dynamic_cluster_points.clear();
    result.classification.dynamic_point_count = 0U;
    result.classification.dynamic_cloud_msg =
        buildCloudFromPointVector(cloud_msg.header, std::vector<ufo::Point>{});
  }
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
  integration_points.reserve(output_points->size());
  for (std::size_t index = 0U; index < output_points->size(); ++index) {
    if (!dynamic_mask[index]) {
      integration_points.push_back((*output_points)[index]);
    }
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
