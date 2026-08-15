#include <ldop/dynamic_object_clusterer.h>
#include <ldop/utils.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace ldopcore {

namespace {

struct ClusterKey {
  ufo::key_t x{0};
  ufo::key_t y{0};
  ufo::key_t z{0};
  ufo::depth_t depth{0};

  // 八叉树键相等需同时比较坐标和层级；同坐标不同 depth 代表不同体素。
  bool operator==(const ClusterKey& other) const {
    return x == other.x && y == other.y && z == other.z && depth == other.depth;
  }
};
 
struct ClusterKeyHash {
  // unordered_map 会调用该哈希器把 ClusterKey 映射到哈希桶。
  std::size_t operator()(const ClusterKey& key) const {
    std::size_t seed = static_cast<std::size_t>(key.x);
    seed ^= static_cast<std::size_t>(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= static_cast<std::size_t>(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= static_cast<std::size_t>(key.depth) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct VoxelBucket {
  ClusterKey key;
  std::vector<std::size_t> point_indices;
};

struct Fragment {
  // 当前帧一个初始 cluster 的原始统计量。min/max 表示真实 AABB 边界，
  // sum/count 用来在合并后继续保持“点云质心”作为 detection center。
  double min_x{std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double min_z{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};
  double max_z{-std::numeric_limits<double>::infinity()};
  double sum_x{0.0};
  double sum_y{0.0};
  double sum_z{0.0};
  std::size_t point_count{0U};
};

// 根据 6/18/26 连通规则生成邻域偏移，供后续 key 空间 BFS 使用。
std::vector<std::array<int, 3>> neighborOffsets(const int connectivity) {
  std::vector<std::array<int, 3>> offsets;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        const int non_zero = (dx != 0 ? 1 : 0) + (dy != 0 ? 1 : 0) + (dz != 0 ? 1 : 0);
        if (non_zero == 0) {
          continue;
        }
        if ((connectivity == 6 && non_zero == 1) ||
            (connectivity == 18 && non_zero <= 2) ||
            (connectivity == 26 && non_zero <= 3)) {
          offsets.push_back({dx, dy, dz});
        }
      }
    }
  }
  return offsets;
}

// 按给定偏移平移 key；负方向平移时做下溢保护。
bool shiftedKey(const ClusterKey& key,
                const std::array<int, 3>& offset,
                const ufo::key_t step,
                ClusterKey& shifted) {
  shifted = key;
  auto shift_axis = [step](const ufo::key_t value, const int delta, ufo::key_t& out) {
    if (delta < 0) {
      // key_t 为无符号，负向平移前先检查是否会下溢。
      if (value < step) {
        return false;
      }
      out = value - step;
      return true;
    }
    if (delta > 0) {
      out = value + step;
      return true;
    }
    out = value;
    return true;
  };

  return shift_axis(key.x, offset[0], shifted.x) &&
         shift_axis(key.y, offset[1], shifted.y) &&
         shift_axis(key.z, offset[2], shifted.z);
}

void appendBoundingBoxEdges(const BoundingBox3D& bbox, visualization_msgs::Marker& marker) {
  const double half_x = bbox.size.x * 0.5;
  const double half_y = bbox.size.y * 0.5;
  const double half_z = bbox.size.z * 0.5;
  const double min_x = bbox.center.x - half_x;
  const double max_x = bbox.center.x + half_x;
  const double min_y = bbox.center.y - half_y;
  const double max_y = bbox.center.y + half_y;
  const double min_z = bbox.center.z - half_z;
  const double max_z = bbox.center.z + half_z;

  // 8 个角点按“底面 4 点 + 顶面 4 点”的顺序组织，后续通过索引连边。
  const std::array<geometry_msgs::Point, 8> corners{
      makePoint(min_x, min_y, min_z), makePoint(max_x, min_y, min_z),
      makePoint(max_x, max_y, min_z), makePoint(min_x, max_y, min_z),
      makePoint(min_x, min_y, max_z), makePoint(max_x, min_y, max_z),
      makePoint(max_x, max_y, max_z), makePoint(min_x, max_y, max_z)};
  // 12 条边：底面 4 条、顶面 4 条、上下连接 4 条。
  constexpr std::array<std::array<std::size_t, 2>, 12> kEdges{{
      {{0U, 1U}}, {{1U, 2U}}, {{2U, 3U}}, {{3U, 0U}},
      {{4U, 5U}}, {{5U, 6U}}, {{6U, 7U}}, {{7U, 4U}},
      {{0U, 4U}}, {{1U, 5U}}, {{2U, 6U}}, {{3U, 7U}},
  }};

  marker.points.reserve(kEdges.size() * 2U);
  for (const auto& edge : kEdges) {
    marker.points.push_back(corners[edge[0]]);
    marker.points.push_back(corners[edge[1]]);
  }
}

void mergeInto(Fragment& target, const Fragment& source) {
  target.min_x = std::min(target.min_x, source.min_x);
  target.min_y = std::min(target.min_y, source.min_y);
  target.min_z = std::min(target.min_z, source.min_z);
  target.max_x = std::max(target.max_x, source.max_x);
  target.max_y = std::max(target.max_y, source.max_y);
  target.max_z = std::max(target.max_z, source.max_z);
  target.sum_x += source.sum_x;
  target.sum_y += source.sum_y;
  target.sum_z += source.sum_z;
  target.point_count += source.point_count;
}

DynamicObjectDetection fragmentToDetection(const Fragment& fragment,
                                           const std::uint32_t detection_id,
                                           const ros::Time& stamp) {
  DynamicObjectDetection detection;
  detection.detection_id = detection_id;
  detection.point_count = fragment.point_count;
  const double point_count = static_cast<double>(std::max<std::size_t>(fragment.point_count, 1U));
  detection.bbox.center.x = fragment.sum_x / point_count;
  detection.bbox.center.y = fragment.sum_y / point_count;
  detection.bbox.center.z = fragment.sum_z / point_count;
  detection.bbox.size.x = std::max(fragment.max_x - fragment.min_x, 0.0);
  detection.bbox.size.y = std::max(fragment.max_y - fragment.min_y, 0.0);
  detection.bbox.size.z = std::max(fragment.max_z - fragment.min_z, 0.0);
  detection.bbox.yaw = 0.0;
  detection.stamp = stamp;
  return detection;
}

bool shouldMergeVerticalFragments(const Fragment& lhs,
                                  const Fragment& rhs,
                                  const DynamicObjectClustererConfig& config) {
  constexpr double kAreaEpsilon = 1e-9;

  const double lhs_size_x = std::max(lhs.max_x - lhs.min_x, 0.0);
  const double lhs_size_y = std::max(lhs.max_y - lhs.min_y, 0.0);
  const double lhs_size_z = std::max(lhs.max_z - lhs.min_z, 0.0);
  const double rhs_size_x = std::max(rhs.max_x - rhs.min_x, 0.0);
  const double rhs_size_y = std::max(rhs.max_y - rhs.min_y, 0.0);
  const double rhs_size_z = std::max(rhs.max_z - rhs.min_z, 0.0);

  const double union_z_size = std::max(lhs.max_z, rhs.max_z) -
                              std::min(lhs.min_z, rhs.min_z);
  // 只修复“上下切开”的碎片；若两个框高度范围几乎一致，更像水平并排目标。
  if (union_z_size <= std::max(lhs_size_z, rhs_size_z) + 1e-6) {
    return false;
  }

  const double z_gap =
      std::max({lhs.min_z - rhs.max_z, rhs.min_z - lhs.max_z, 0.0});
  if (z_gap > config.vertical_merge_max_z_gap) {
    return false;
  }

  const double intersection_x =
      std::max(0.0, std::min(lhs.max_x, rhs.max_x) - std::max(lhs.min_x, rhs.min_x));
  const double intersection_y =
      std::max(0.0, std::min(lhs.max_y, rhs.max_y) - std::max(lhs.min_y, rhs.min_y));
  const double intersection_area = intersection_x * intersection_y;
  const double lhs_area = lhs_size_x * lhs_size_y;
  const double rhs_area = rhs_size_x * rhs_size_y;
  const double min_area = std::min(lhs_area, rhs_area);
  // 只解决垂直方向断裂：水平投影必须有足够重叠，不跨 xy 空隙合并。
  if (!(intersection_area > kAreaEpsilon && min_area > kAreaEpsilon &&
        intersection_area / min_area >= config.vertical_merge_min_xy_overlap_ratio)) {
    return false;
  }

  const double merged_size_x = std::max(lhs.max_x, rhs.max_x) - std::min(lhs.min_x, rhs.min_x);
  const double merged_size_y = std::max(lhs.max_y, rhs.max_y) - std::min(lhs.min_y, rhs.min_y);
  const double merged_size_z = std::max(lhs.max_z, rhs.max_z) - std::min(lhs.min_z, rhs.min_z);
  const double merged_extent =
      std::sqrt(merged_size_x * merged_size_x +
                merged_size_y * merged_size_y +
                merged_size_z * merged_size_z);
  if (config.max_extent > 0.0 && merged_extent > config.max_extent) {
    return false;
  }

  return true;
}

std::vector<Fragment> mergeFragmentedDetections(
    std::vector<Fragment> fragments,
    const DynamicObjectClustererConfig& config) {
  // 当前帧目标数通常很少，直接反复合并第一对满足条件的上下碎片。
  bool merged = true;
  while (merged) {
    merged = false;
    for (std::size_t lhs_index = 0; lhs_index < fragments.size() && !merged; ++lhs_index) {
      for (std::size_t rhs_index = lhs_index + 1U; rhs_index < fragments.size(); ++rhs_index) {
        if (!shouldMergeVerticalFragments(fragments[lhs_index], fragments[rhs_index], config)) {
          continue;
        }
        mergeInto(fragments[lhs_index], fragments[rhs_index]);
        fragments.erase(fragments.begin() + static_cast<std::ptrdiff_t>(rhs_index));
        merged = true;
        break;
      }
    }
  }

  return fragments;
}

}  // namespace

DynamicObjectClustererConfig buildClustererConfig(const DynamicObjectClustererParams& params) {
  const DynamicObjectClustererParams defaults;
  DynamicObjectClustererConfig config;

  // 这些参数来自 ROS/YAML，属于跨边界输入；这里只按 Params 注释里已经声明的
  // 正数契约做默认值兜底，避免把负点数等非法值带入后续无符号类型或几何阈值。
  const int min_points = params.min_points > 0 ? params.min_points : defaults.min_points;
  config.min_points = static_cast<std::size_t>(min_points);
  config.max_extent = params.max_extent > 0.0 ? params.max_extent : defaults.max_extent;
  config.vertical_merge_max_z_gap = params.vertical_merge_max_z_gap > 0.0
      ? params.vertical_merge_max_z_gap : defaults.vertical_merge_max_z_gap;
  config.vertical_merge_min_xy_overlap_ratio = params.vertical_merge_min_xy_overlap_ratio > 0.0
      ? params.vertical_merge_min_xy_overlap_ratio : defaults.vertical_merge_min_xy_overlap_ratio;

  // connectivity 是离散枚举型约束；非法值回到 Params 默认值，后续邻域生成才能
  // 始终落在 6/18/26 三种有定义的连通规则内。
  const bool valid_connectivity =
      params.connectivity == 6 || params.connectivity == 18 || params.connectivity == 26;
  config.connectivity = valid_connectivity ? params.connectivity : defaults.connectivity;
  return config;
}

DynamicObjectClusterer::DynamicObjectClusterer(ros::NodeHandle& pnh, const bool verbose)
    : pnh_(pnh), params_(), config_(), verbose_(verbose) {
  loadParameters();

  ROS_INFO_STREAM("Dynamic object clusterer is ready. connectivity: " << config_.connectivity
                  << ", min points: " << config_.min_points
                  << ", max extent: " << config_.max_extent
                  << ", vertical merge z gap: " << config_.vertical_merge_max_z_gap
                  << ", vertical merge overlap ratio: "
                  << config_.vertical_merge_min_xy_overlap_ratio);
}

void DynamicObjectClusterer::loadParameters() {
  // 默认值只从参数快照结构体取，避免读参处和结构体初值各维护一份数字。
  const DynamicObjectClustererParams defaults;
  pnh_.param("dynamic_cluster_min_points", params_.min_points, defaults.min_points);
  pnh_.param("dynamic_cluster_max_extent", params_.max_extent, defaults.max_extent);
  pnh_.param("dynamic_cluster_connectivity", params_.connectivity, defaults.connectivity);
  pnh_.param("dynamic_cluster_vertical_merge_max_z_gap", params_.vertical_merge_max_z_gap, defaults.vertical_merge_max_z_gap);
  pnh_.param("dynamic_cluster_vertical_merge_min_xy_overlap_ratio", params_.vertical_merge_min_xy_overlap_ratio, defaults.vertical_merge_min_xy_overlap_ratio);
  config_ = buildClustererConfig(params_);
}

DynamicObjectClustererFrameResult DynamicObjectClusterer::processDynamicObjects(
    const std_msgs::Header& header,
    const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const {
  const auto total_start = std::chrono::steady_clock::now();
  DynamicObjectClustererFrameResult result;

  const auto build_detections_start = std::chrono::steady_clock::now();
  result.detections = buildDetections(header.stamp, dynamic_points);
  result.timing.build_detections_ms =
      elapsedMs(build_detections_start, std::chrono::steady_clock::now());

  const auto build_markers_start = std::chrono::steady_clock::now();
  result.dynamic_object_markers_msg = buildObjectMarkers(header, result.detections);
  result.timing.build_object_markers_ms =
      elapsedMs(build_markers_start, std::chrono::steady_clock::now());

  result.timing.process_dynamic_objects_ms =
      elapsedMs(total_start, std::chrono::steady_clock::now());

  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Dynamic object clusterer timing"
                                      << ": processDynamicObjects="
                                      << formatFloatMs(result.timing.process_dynamic_objects_ms)
                                      << "ms, buildDetections="
                                      << formatFloatMs(result.timing.build_detections_ms)
                                      << "ms, buildObjectMarkers="
                                      << formatFloatMs(result.timing.build_object_markers_ms)
                                      << "ms, dynamicPoints=" << dynamic_points.size()
                                      << ", detections=" << result.detections.size());
  }

  return result;
}

std::vector<DynamicObjectDetection> DynamicObjectClusterer::buildDetections(
    const ros::Time& stamp,
    const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const {
  if (dynamic_points.empty()) {
    return {};
  }

  // 本函数按“体素桶 -> key 空间连通域 -> Fragment 统计量 -> 当前帧后处理 -> detection”
  // 的顺序组织。读代码时先跟着这条主线走，neighborOffsets()/shiftedKey() 只是 BFS
  // 找邻居的工具函数，mergeFragmentedDetections() 是 detection 生成前的局部修正。

  // 1) 先按体素 key 聚合点，后续在 key 空间做连通域，避免对原始点做半径搜索。
  std::vector<VoxelBucket> buckets;
  std::unordered_map<ClusterKey, std::size_t, ClusterKeyHash> bucket_by_key;
  buckets.reserve(dynamic_points.size());
  bucket_by_key.reserve(dynamic_points.size());

  for (std::size_t index = 0; index < dynamic_points.size(); ++index) {
    const auto& code = dynamic_points[index].voxel_code;
    const ClusterKey key{code.key_x, code.key_y, code.key_z, code.depth};
    const auto [it, inserted] = bucket_by_key.emplace(key, buckets.size());
    if (inserted) {
      VoxelBucket bucket;
      bucket.key = key;
      buckets.push_back(std::move(bucket));
    }
    buckets[it->second].point_indices.push_back(index);
  }

  // 2) 固定遍历顺序，降低 unordered_map 无序迭代带来的检测 ID 抖动。
  std::vector<std::size_t> sorted_bucket_indices(buckets.size());
  for (std::size_t i = 0; i < sorted_bucket_indices.size(); ++i) {
    sorted_bucket_indices[i] = i;
  }
  // std::sort 是 C++ 标准库里的“排序算法”，作用是把一段区间按你给的规则排好序。
  // unordered_map 本身遍历顺序不稳定，先把桶索引排好序，后续遍历就稳定了，检测结果顺序和 detection_id 也更不容易抖动。
  std::sort(sorted_bucket_indices.begin(), sorted_bucket_indices.end(),
            [&buckets](const std::size_t lhs, const std::size_t rhs) {
              const auto& a = buckets[lhs].key;
              const auto& b = buckets[rhs].key;
              // 字典序比较：先比 depth，若相等再比 x，再比 y，最后比 z。
              return std::tie(a.depth, a.x, a.y, a.z) < std::tie(b.depth, b.x, b.y, b.z);
            });

  // 3) 在 UFOMap key 空间做 BFS 连通域，每个连通块对应一个候选目标。
  const auto offsets = neighborOffsets(config_.connectivity);
  std::vector<bool> visited(buckets.size(), false);
  std::vector<Fragment> fragments;

  for (const std::size_t start_bucket : sorted_bucket_indices) {
    if (visited[start_bucket]) {
      continue;
    }

    std::vector<std::size_t> cluster_points;
    std::queue<std::size_t> queue;
    queue.push(start_bucket);
    visited[start_bucket] = true;

    while (!queue.empty()) {
      const std::size_t bucket_index = queue.front();
      queue.pop();

      const auto& bucket = buckets[bucket_index];
      // 将当前桶里的点索引并入当前连通块。
      cluster_points.insert(cluster_points.end(),
                            bucket.point_indices.begin(),
                            bucket.point_indices.end());

      // 同层 key 邻居步长由 depth 决定。
      const ufo::key_t step = ufo::key_t{1} << bucket.key.depth;
      for (const auto& offset : offsets) {
        ClusterKey neighbor;
        if (!shiftedKey(bucket.key, offset, step, neighbor)) {
          continue;
        }
        const auto found = bucket_by_key.find(neighbor);
        if (found == bucket_by_key.end() || visited[found->second]) {
          continue;
        }
        visited[found->second] = true;
        queue.push(found->second);
      }
    }

    // 点数不足的连通块视为噪声。
    if (cluster_points.size() < config_.min_points) {
      continue;
    }

    // 4) 统计当前连通块的 Fragment。min/max 形成真实 AABB，sum/count 保留点云质心；
    // 后面可能会合并上下碎片，所以这里先不直接生成 DynamicObjectDetection。
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    for (const std::size_t point_index : cluster_points) {
      const auto& point = dynamic_points[point_index].point;
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      min_z = std::min(min_z, static_cast<double>(point.z));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
      max_z = std::max(max_z, static_cast<double>(point.z));
      sum_x += static_cast<double>(point.x);
      sum_y += static_cast<double>(point.y);
      sum_z += static_cast<double>(point.z);
    }

    const double size_x = std::max(max_x - min_x, 0.0);
    const double size_y = std::max(max_y - min_y, 0.0);
    const double size_z = std::max(max_z - min_z, 0.0);
    // 5) 用包围盒对角线做尺寸过滤，剔除异常大目标。
    if (config_.max_extent > 0.0 &&
        std::sqrt(size_x * size_x + size_y * size_y + size_z * size_z) >
            config_.max_extent) {
      continue;
    }

    // 6) 先保留 fragment 统计量；合并完成后再统一生成 detection。
    Fragment fragment;
    fragment.min_x = min_x;
    fragment.min_y = min_y;
    fragment.min_z = min_z;
    fragment.max_x = max_x;
    fragment.max_y = max_y;
    fragment.max_z = max_z;
    fragment.sum_x = sum_x;
    fragment.sum_y = sum_y;
    fragment.sum_z = sum_z;
    fragment.point_count = cluster_points.size();
    fragments.push_back(fragment);
  }

  // 7) 当前帧检测级后处理：合并上下断裂的碎片，再生成帧内 detection_id。
  const auto merged_fragments = mergeFragmentedDetections(std::move(fragments), config_);
  std::vector<DynamicObjectDetection> detections;
  detections.reserve(merged_fragments.size());
  for (std::size_t index = 0; index < merged_fragments.size(); ++index) {
    detections.push_back(fragmentToDetection(merged_fragments[index],
                                             static_cast<std::uint32_t>(index),
                                             stamp));
  }
  return detections;
}

visualization_msgs::MarkerArray DynamicObjectClusterer::buildObjectMarkers(
    const std_msgs::Header& header,
    const std::vector<DynamicObjectDetection>& detections) const {
  visualization_msgs::MarkerArray marker_array;
  // 先清空历史 marker，避免旧框残留在 RViz。
  marker_array.markers.reserve(detections.size() + 1U);
  marker_array.markers.push_back(makeDeleteAllMarker(header));

  // 每个检测输出一个 LINE_LIST 包围盒。
  for (const auto& detection : detections) {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "dynamic_object_bbox";
    marker.id = static_cast<int>(detection.detection_id);
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.07;
    marker.color.r = 1.0F;
    marker.color.g = 0.90F;
    marker.color.b = 0.05F;
    marker.color.a = 1.0F;

    appendBoundingBoxEdges(detection.bbox, marker);
    marker_array.markers.push_back(std::move(marker));
  }

  return marker_array;
}

}  // namespace ldopcore
