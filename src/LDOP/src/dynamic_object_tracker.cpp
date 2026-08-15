#include <ldop/dynamic_object_tracker.h>
#include <ldop/utils.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Dense>
#include <boost/math/distributions/chi_squared.hpp>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace ldopcore {

namespace {

struct AssociationCandidate {
  // 记录一次 detection-track 候选匹配；raw_cost 保留 gate 内的原始马氏距离平方。
  std::size_t detection_index{0U};
  std::size_t track_index{0U};
  double raw_cost{0.0};
};

// 行对应 detection，列对应已有 track；invalid_cost 表示该组合被 gate 拒绝。
using CostMatrix = std::vector<std::vector<double>>;

struct ClassificationSizeUpdate {
  geometry_msgs::Vector3 stable_size;
  bool reliable_for_classification{true};
};

geometry_msgs::Point lastMatchedTrackCenter(const TrackState& track) {
  for (auto history_it = track.history.rbegin(); history_it != track.history.rend();
       ++history_it) {
    if (history_it->matched && history_it->model_state.size() >= 3) {
      return makePoint(history_it->model_state(0), history_it->model_state(1),
                       history_it->model_state(2));
    }
  }
  return track.bbox.center;
}

bool hasSimilarHorizontalBoundingBoxSize(const DynamicObjectDetection& detection,
                                         const TrackState& track) {
  const auto sizeRatio = [](const double lhs, const double rhs) {
    if (!(std::isfinite(lhs) && std::isfinite(rhs)) || lhs <= 1e-6 || rhs <= 1e-6) {
      return 0.0;
    }
    return std::min(lhs, rhs) / std::max(lhs, rhs);
  };

  // 同一目标在点云稀疏或被遮挡时中心和竖向尺寸可能跳动，水平占地通常更稳定；
  // 用水平尺寸相似度承接 IoU 不重叠的情况，避免把细长固定杆并入小球轨迹。
  constexpr double kMinimumSizeRatio = 0.18;
  return sizeRatio(detection.bbox.size.x, track.bbox.size.x) >= kMinimumSizeRatio &&
         sizeRatio(detection.bbox.size.y, track.bbox.size.y) >= kMinimumSizeRatio;
}

double horizontalBoundingBoxSizeRatio(const geometry_msgs::Vector3& lhs,
                                      const geometry_msgs::Vector3& rhs) {
  const auto sizeRatio = [](const double first, const double second) {
    if (!(std::isfinite(first) && std::isfinite(second)) || first <= 1e-6 || second <= 1e-6) {
      return 0.0;
    }
    return std::min(first, second) / std::max(first, second);
  };

  return std::min(sizeRatio(lhs.x, rhs.x), sizeRatio(lhs.y, rhs.y));
}

bool isPublishableTrack(const TrackState& track,
                        const DynamicObjectTrackerConfig& config) {
  return track.hits >= config.min_hits_to_publish &&
         track.motion_confirmed &&
         track.missed_frames <= config.max_publish_missed_frames;
}

std::size_t classScoreIndex(const ObjectClass object_class) {
  switch (object_class) {
    case ObjectClass::Human:
      return 0U;
    case ObjectClass::Vehicle:
      return 1U;
    case ObjectClass::Uav:
      return 2U;
    case ObjectClass::Other:
      return 3U;
    case ObjectClass::Unknown:
      break;
  }
  return 4U;
}

MotionModelType motionModelForClass(const ObjectClass object_class,
                                    const MotionModelType current_model) {
  switch (object_class) {
    case ObjectClass::Human:
      return MotionModelType::CA2D;
    case ObjectClass::Vehicle:
      return MotionModelType::CTRA;
    case ObjectClass::Uav:
      return MotionModelType::CA3D;
    case ObjectClass::Other:
      return MotionModelType::CV3D;
    case ObjectClass::Unknown:
      break;
  }
  return current_model;
}

const char* objectClassDisplayName(const ObjectClass object_class) {
  switch (object_class) {
    case ObjectClass::Human:
      return "Human";
    case ObjectClass::Vehicle:
      return "Vehicle";
    case ObjectClass::Uav:
      return "UAV";
    case ObjectClass::Other:
      return "Other";
    case ObjectClass::Unknown:
      break;
  }
  return "Unknown";
}

std::string makeTrackLabelText(const TrackState& track) {
  const double speed_mps = track.filter->velocity().norm();

  std::ostringstream stream;
  // RViz 文字面向现场观察者，因此显示速度模长，而不是三轴速度分量。
  stream << "ID: " << track.id << '\n'
      << "Class: " << objectClassDisplayName(track.object_class) << '\n'
      << "Speed: " << std::fixed << std::setprecision(2) << speed_mps << " m/s";
  return stream.str();
}

void appendTrackBoundingBoxEdges(const BoundingBox3D& bbox,
                                 visualization_msgs::Marker& marker) {
  const double half_x = bbox.size.x * 0.5;
  const double half_y = bbox.size.y * 0.5;
  const double half_z = bbox.size.z * 0.5;
  const double min_x = bbox.center.x - half_x;
  const double max_x = bbox.center.x + half_x;
  const double min_y = bbox.center.y - half_y;
  const double max_y = bbox.center.y + half_y;
  const double min_z = bbox.center.z - half_z;
  const double max_z = bbox.center.z + half_z;

  const std::array<geometry_msgs::Point, 8> corners{
      makePoint(min_x, min_y, min_z), makePoint(max_x, min_y, min_z),
      makePoint(max_x, max_y, min_z), makePoint(min_x, max_y, min_z),
      makePoint(min_x, min_y, max_z), makePoint(max_x, min_y, max_z),
      makePoint(max_x, max_y, max_z), makePoint(min_x, max_y, max_z)};
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

double chiSquare3DThresholdFromConfidence(const double confidence) {
  constexpr double kDegreesOfFreedom = 3.0;
  constexpr double kMinConfidence = 1e-6;
  constexpr double kMaxConfidence = 1.0 - 1e-6;

  // 使用卡方分布分位数连续计算 gate，避免离散查表导致门限突跳。
  const double bounded_confidence = std::clamp(confidence, kMinConfidence, kMaxConfidence);
  const boost::math::chi_squared chi_square_distribution(kDegreesOfFreedom);
  return boost::math::quantile(chi_square_distribution, bounded_confidence);
}

DynamicObjectTrackerConfig buildTrackerConfig(const DynamicObjectTrackerParams& params) {
  DynamicObjectTrackerConfig config;
  const DynamicObjectTrackerParams defaults;

  config.max_history_size =
      static_cast<std::size_t>(params.history_size > 0 ? params.history_size : defaults.history_size);
  config.max_coast_frames =
      static_cast<std::size_t>(params.max_missed_frames > 0 ? params.max_missed_frames : defaults.max_missed_frames);
  const int requested_publish_missed_frames =
      params.max_publish_missed_frames >= 0
          ? params.max_publish_missed_frames
          : defaults.max_publish_missed_frames;
  config.max_publish_missed_frames = std::min(
      static_cast<std::size_t>(requested_publish_missed_frames), config.max_coast_frames);
  const std::size_t requested_min_hits =
      static_cast<std::size_t>(params.min_hits_to_publish > 0 ? params.min_hits_to_publish : defaults.min_hits_to_publish);
  const std::size_t max_publish_hits = config.max_history_size > 1U ? config.max_history_size - 1U : 1U;
  config.min_hits_to_publish = std::min(requested_min_hits, max_publish_hits);
  config.motion_min_displacement =
      params.motion_min_displacement > 0.0
          ? params.motion_min_displacement
          : defaults.motion_min_displacement;
  const std::size_t requested_motion_evidence_frames =
      static_cast<std::size_t>(params.motion_min_evidence_frames > 0
                                   ? params.motion_min_evidence_frames
                                   : defaults.motion_min_evidence_frames);
  config.motion_min_evidence_frames =
      std::min(requested_motion_evidence_frames, max_publish_hits);
  config.motion_confirmation_speed =
      params.motion_confirmation_speed > 0.0
          ? params.motion_confirmation_speed
          : defaults.motion_confirmation_speed;
  // 外部参数是置信度，tracker 内部关联使用 3 自由度卡方分布的马氏距离平方门限。
  config.association_gate_threshold = chiSquare3DThresholdFromConfidence(params.association_gate_confidence);
  config.coasting_gate_relax_factor =
      params.coasting_gate_relax_factor >= 1.0 ? params.coasting_gate_relax_factor : defaults.coasting_gate_relax_factor;
  config.box_size_smoothing_alpha = std::clamp(params.box_size_smoothing_alpha, 0.0, 1.0);
  config.spawn_suppression_distance =
      params.spawn_suppression_distance > 0.0 ? params.spawn_suppression_distance : defaults.spawn_suppression_distance;
  config.spawn_suppression_iou_threshold = params.spawn_suppression_iou_threshold > 0.0
      ? std::min(params.spawn_suppression_iou_threshold, 1.0) : defaults.spawn_suppression_iou_threshold;
  config.duplicate_merge_distance = params.duplicate_merge_distance > 0.0
      ? params.duplicate_merge_distance : 0.0;
  config.max_dt = params.max_dt > 0.0 ? params.max_dt : defaults.max_dt;
  config.classification_enabled = params.classification_enabled;
  config.classification_start_frame = static_cast<std::size_t>(params.classification_start_frame > 0
      ? params.classification_start_frame : defaults.classification_start_frame);
  config.classification_score_decay =
      params.classification_score_decay > 0.0 && !(params.classification_score_decay >= 1.0)
          ? params.classification_score_decay : defaults.classification_score_decay;
  config.classification_score_increment =
      params.classification_score_increment > 0.0 ? params.classification_score_increment
                                                  : defaults.classification_score_increment;
  config.classification_confirm_score =
      params.classification_confirm_score > 0.0 ? params.classification_confirm_score
                                                : defaults.classification_confirm_score;
  config.classification_switch_margin =
      params.classification_switch_margin > 0.0 ? params.classification_switch_margin
                                                : defaults.classification_switch_margin;
  config.classification_size_change_ratio =
      std::clamp(params.classification_size_change_ratio, 0.0, 1.0);
  config.classification_point_count_change_ratio =
      std::clamp(params.classification_point_count_change_ratio, 0.0, 1.0);
  config.classification_size_change_confirm_frames =
      static_cast<std::size_t>(params.classification_size_change_confirm_frames > 0
          ? params.classification_size_change_confirm_frames : defaults.classification_size_change_confirm_frames);
  config.classify_human_threshold = params.classify_human_threshold;
  config.classify_vehicle_threshold = params.classify_vehicle_threshold;
  config.classify_uav_threshold = params.classify_uav_threshold;
  // 共享卡尔曼参数快照统一由 buildMultiModelKalmanFilterConfig 转换为运行时配置。
  config.filter_config = buildMultiModelKalmanFilterConfig(params.filter);
  return config;
}

double computeDeltaSeconds(const ros::Time& current_stamp,
                           const ros::Time& previous_stamp,
                           const double fallback_dt,
                           const double max_dt) {
  if (current_stamp.isZero() || previous_stamp.isZero()) {
    return fallback_dt;
  }

  const double dt = (current_stamp - previous_stamp).toSec();
  const double positive_dt =
      std::isfinite(dt) && dt > 0.0 ? dt : fallback_dt;
  const double sanitized_max_dt =
      std::isfinite(max_dt) && max_dt > 0.0 ? max_dt : positive_dt;
  return std::min(positive_dt, sanitized_max_dt);
}

double computeMahalanobisCost(const Eigen::Vector3d& detection_position,
                              const TrackState& track) {
  // 这里的 innovation 是 detection 位置观测与预测轨迹中心的偏差；当前观测来自点簇质心。
  const Eigen::Vector3d innovation = detection_position - track.filter->position();
  Eigen::Matrix3d position_covariance =
      track.filter->covariance().block<3, 3>(0, 0);
  position_covariance = 0.5 * (position_covariance + position_covariance.transpose());

  // 关联代价使用预测位置协方差；轻微正则化避免数值退化时误拒绝所有候选。
  position_covariance += 1e-6 * Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d inverse_covariance = position_covariance.inverse();
  if (!inverse_covariance.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }

  const double cost = innovation.dot(inverse_covariance * innovation);
  return std::isfinite(cost) && cost >= 0.0 ? cost : std::numeric_limits<double>::infinity();
}

double computeHorizontalIntersectionOverUnion(const BoundingBox3D& lhs,
                                              const BoundingBox3D& rhs) {
  // 只计算水平 AABB IoU；当前检测框 yaw 固定为 0，先不引入有向框几何。
  const double lhs_size_x = std::max(lhs.size.x, 0.0);
  const double lhs_size_y = std::max(lhs.size.y, 0.0);
  const double rhs_size_x = std::max(rhs.size.x, 0.0);
  const double rhs_size_y = std::max(rhs.size.y, 0.0);
  const double lhs_area = lhs_size_x * lhs_size_y;
  const double rhs_area = rhs_size_x * rhs_size_y;
  if (lhs_area <= 0.0 || rhs_area <= 0.0) {
    return 0.0;
  }

  const double lhs_min_x = lhs.center.x - lhs_size_x * 0.5;
  const double lhs_max_x = lhs.center.x + lhs_size_x * 0.5;
  const double lhs_min_y = lhs.center.y - lhs_size_y * 0.5;
  const double lhs_max_y = lhs.center.y + lhs_size_y * 0.5;
  const double rhs_min_x = rhs.center.x - rhs_size_x * 0.5;
  const double rhs_max_x = rhs.center.x + rhs_size_x * 0.5;
  const double rhs_min_y = rhs.center.y - rhs_size_y * 0.5;
  const double rhs_max_y = rhs.center.y + rhs_size_y * 0.5;

  const double intersection_x = std::max(0.0, std::min(lhs_max_x, rhs_max_x) -
                                                  std::max(lhs_min_x, rhs_min_x));
  const double intersection_y = std::max(0.0, std::min(lhs_max_y, rhs_max_y) -
                                                  std::max(lhs_min_y, rhs_min_y));
  const double intersection_area = intersection_x * intersection_y;
  const double union_area = lhs_area + rhs_area - intersection_area;
  return union_area > 0.0 ? intersection_area / union_area : 0.0;
}

ObjectClass classifyObservation(const BoundingBox3D& detection_bbox,
                                const geometry_msgs::Vector3& stable_size,
                                const DynamicObjectTrackerConfig& config) {
  const double x_width = std::max(stable_size.x, 0.0);
  const double y_width = std::max(stable_size.y, 0.0);
  const double z_width = std::max(stable_size.z, 0.0);
  const double max_xy_width = std::max(x_width, y_width);
  const double max_size = std::max({x_width, y_width, z_width});
  if (max_size <= 1e-6) {
    return ObjectClass::Other;
  }

  const double human_z_width_ratio = config.classify_human_threshold[0];
  const double human_centroid_z_ratio = config.classify_human_threshold[1];
  const double vehicle_xy_width_ratio = config.classify_vehicle_threshold[0];
  const double vehicle_centroid_z_ratio = config.classify_vehicle_threshold[1];
  const double uav_max_size = config.classify_uav_threshold[0];
  const double uav_centroid_z_ratio = config.classify_uav_threshold[1];

  // 这里沿用 LDOT 的尺寸/质心启发式：用稳定尺寸判断外形，用当前质心高度区分近地目标和空中目标。
  // LDOP 的动静态分割已经在 UFOMap 前级完成，因此这里不再做点云运动投票。
  if (z_width >= max_xy_width * human_z_width_ratio &&
      detection_bbox.center.z < z_width * human_centroid_z_ratio) {
    return ObjectClass::Human;
  }
  if (max_xy_width >= z_width * vehicle_xy_width_ratio &&
      detection_bbox.center.z < z_width * vehicle_centroid_z_ratio) {
    return ObjectClass::Vehicle;
  }
  if (x_width < uav_max_size && y_width < uav_max_size && z_width < uav_max_size &&
      detection_bbox.center.z > z_width * uav_centroid_z_ratio) {
    return ObjectClass::Uav;
  }
  return ObjectClass::Other;
}

std::optional<ObjectClass> confirmedClassFromScores(
    const std::array<double, 4U>& scores,
    const DynamicObjectTrackerConfig& config) {
  std::size_t best_index = 0U;
  std::size_t second_index = 1U;
  if (scores[second_index] > scores[best_index]) {
    std::swap(best_index, second_index);
  }
  for (std::size_t index = 2U; index < scores.size(); ++index) {
    if (scores[index] > scores[best_index]) {
      second_index = best_index;
      best_index = index;
    } else if (scores[index] > scores[second_index]) {
      second_index = index;
    }
  }

  const double best_score = scores[best_index];
  const double second_score = scores[second_index];
  if (best_score < config.classification_confirm_score ||
      best_score - second_score < config.classification_switch_margin) {
    return std::nullopt;
  }

  switch (best_index) {
    case 0U:
      return ObjectClass::Human;
    case 1U:
      return ObjectClass::Vehicle;
    case 2U:
      return ObjectClass::Uav;
    case 3U:
      return ObjectClass::Other;
    default:
      break;
  }
  return std::nullopt;
}

Eigen::VectorXd stateForModel(const MotionModelType model_type,
                              const Eigen::Vector3d& position,
                              const Eigen::Vector3d& velocity,
                              const double preserved_yaw,
                              const bool has_preserved_yaw) {
  constexpr double kYawSpeedThreshold = 1e-3;

  switch (model_type) {
    case MotionModelType::CA2D: {
      Eigen::VectorXd state = Eigen::VectorXd::Zero(7);
      state << position.x(), position.y(), position.z(), velocity.x(), velocity.y(), 0.0, 0.0;
      return state;
    }
    case MotionModelType::CA3D: {
      Eigen::VectorXd state = Eigen::VectorXd::Zero(9);
      state << position.x(), position.y(), position.z(), velocity.x(), velocity.y(), velocity.z(),
          0.0, 0.0, 0.0;
      return state;
    }
    case MotionModelType::CV3D: {
      Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
      state << position.x(), position.y(), position.z(), velocity.x(), velocity.y(), velocity.z();
      return state;
    }
    case MotionModelType::CTRA: {
      Eigen::VectorXd state = Eigen::VectorXd::Zero(7);
      const double speed = std::hypot(velocity.x(), velocity.y());
      const double yaw = speed > kYawSpeedThreshold
                             ? std::atan2(velocity.y(), velocity.x())
                             : (has_preserved_yaw ? preserved_yaw : 0.0);
      state << position.x(), position.y(), position.z(), speed, 0.0, yaw, 0.0;
      return state;
    }
  }

  return Eigen::VectorXd();
}

ClassificationSizeUpdate updateClassificationSizeState(
    TrackState& track,
    const DynamicObjectDetection& detection,
    const DynamicObjectTrackerConfig& config) {
  constexpr double kSizeEpsilon = 1e-6;

  if (maxComponent(track.max_observed_size) <= kSizeEpsilon) {
    track.max_observed_size = detection.bbox.size;
  }

  const double ratio_x =
      detection.bbox.size.x / std::max(track.max_observed_size.x, kSizeEpsilon);
  const double ratio_y =
      detection.bbox.size.y / std::max(track.max_observed_size.y, kSizeEpsilon);
  const double ratio_z =
      detection.bbox.size.z / std::max(track.max_observed_size.z, kSizeEpsilon);
  const double max_size_ratio = std::max({ratio_x, ratio_y, ratio_z});
  const double point_ratio =
      static_cast<double>(detection.point_count) /
      static_cast<double>(std::max<std::size_t>(track.last_point_count, 1U));

  const double merge_threshold = 1.0 + config.classification_size_change_ratio;
  const double separation_threshold = 1.0 - config.classification_size_change_ratio;
  const double point_merge_threshold = 1.0 + config.classification_point_count_change_ratio;
  const double point_separation_threshold = 1.0 - config.classification_point_count_change_ratio;

  const bool looks_merged =
      max_size_ratio > merge_threshold || point_ratio > point_merge_threshold;
  const bool looks_separated =
      max_size_ratio < separation_threshold || point_ratio < point_separation_threshold;

  ClassificationSizeUpdate update;
  update.stable_size = detection.bbox.size;

  if (looks_merged) {
    ++track.large_size_counter;
    track.small_size_counter = 0U;
    if (track.large_size_counter >= config.classification_size_change_confirm_frames) {
      track.max_observed_size = componentMax(track.max_observed_size, detection.bbox.size);
      track.last_point_count = detection.point_count;
      update.stable_size = track.max_observed_size;
    } else {
      // 短时粘连常把 bbox 和点数一起拉大，冻结类别证据能避免把两个人误判成车。
      // 点数参考也暂不刷新，否则第二帧会把突变比例洗掉，无法累计确认帧数。
      update.stable_size = track.max_observed_size;
      update.reliable_for_classification = false;
    }
  } else if (looks_separated) {
    ++track.small_size_counter;
    track.large_size_counter = 0U;
    if (track.small_size_counter > config.classification_size_change_confirm_frames) {
      track.max_observed_size = detection.bbox.size;
      track.last_point_count = detection.point_count;
      update.stable_size = track.max_observed_size;
    } else {
      // 短时遮挡或残片会让 bbox/点数突然变小；此时保留历史尺寸并冻结分类证据。
      // 同样冻结点数参考，使持续残片能被确认，短时残片则不会污染类别。
      update.stable_size = track.max_observed_size;
      update.reliable_for_classification = false;
    }
  } else {
    track.large_size_counter = 0U;
    track.small_size_counter = 0U;
    track.max_observed_size = componentMax(track.max_observed_size, detection.bbox.size);
    track.last_point_count = detection.point_count;
    update.stable_size = track.max_observed_size;
  }

  return update;
}

double computeFallbackAssociationCost(const DynamicObjectDetection& detection,
                                      const TrackState& track,
                                      const DynamicObjectTrackerConfig& config,
                                      const double rejected_gate_cost,
                                      const double invalid_cost) {
  const bool use_distance_fallback = config.spawn_suppression_distance > 0.0;
  const bool use_iou_fallback = config.spawn_suppression_iou_threshold > 0.0;
  if (!use_distance_fallback && !use_iou_fallback) {
    return invalid_cost;
  }

  double distance_penalty = 0.0;
  if (use_distance_fallback) {
    // 预测状态可能在漏检帧中漂移到错误位置；回退重关联应优先参考最近一次真实观测。
    const double center_distance =
        distance3D(detection.bbox.center, lastMatchedTrackCenter(track));
    if (!(std::isfinite(center_distance) &&
          center_distance <= config.spawn_suppression_distance)) {
      return invalid_cost;
    }
    // 回退关联先过距离门槛；只有这一层通过后，后面的 IoU 判据才有意义。
    // 距离越近，回退代价越低；但整体仍故意高于任何合法马氏距离匹配。
    distance_penalty =
        center_distance / std::max(config.spawn_suppression_distance, 1e-6);
  }

  double iou_penalty = 0.0;
  if (use_iou_fallback) {
    const double iou = computeHorizontalIntersectionOverUnion(detection.bbox, track.bbox);
    if (!(std::isfinite(iou) && iou >= config.spawn_suppression_iou_threshold)) {
      // 中心跳变会使水平 IoU 变为 0；成熟轨迹仍可在尺寸相似且距离合适时重关联。
      if (!track.motion_confirmed &&
          (track.hits < config.min_hits_to_publish ||
           !hasSimilarHorizontalBoundingBoxSize(detection, track))) {
        return invalid_cost;
      }
      iou_penalty = 1.0;
    } else {
      // IoU 越大，回退代价越低；同样只作为马氏距离失败后的次优候选。
      iou_penalty = 1.0 - std::clamp(iou, 0.0, 1.0);
    }
  }

  const double fallback_penalty =
      use_distance_fallback && use_iou_fallback
          ? 0.5 * (distance_penalty + iou_penalty)
          : (use_distance_fallback ? distance_penalty : iou_penalty);
  return rejected_gate_cost + 1.0 + fallback_penalty;
}

bool shouldSuppressTrackSpawn(const DynamicObjectDetection& detection,
                              const std::vector<TrackState>& tracks,
                              const DynamicObjectTrackerConfig& config) {
  if (config.spawn_suppression_distance <= 0.0 &&
      config.spawn_suppression_iou_threshold <= 0.0) {
    return false;
  }

  for (const auto& track : tracks) {
    // 未匹配 detection 若仍贴近已有轨迹，先作为 tentative detection 挂起，避免中心跳动时立刻裂出新 ID。
    if (config.spawn_suppression_iou_threshold > 0.0 &&
        computeHorizontalIntersectionOverUnion(detection.bbox, track.bbox) >=
            config.spawn_suppression_iou_threshold) {
      return true;
    }
    if (config.spawn_suppression_distance > 0.0 &&
        distance3D(detection.bbox.center, lastMatchedTrackCenter(track)) <=
            config.spawn_suppression_distance) {
      if (track.motion_confirmed ||
          config.spawn_suppression_iou_threshold <= 0.0 ||
          track.hits < config.min_hits_to_publish ||
          hasSimilarHorizontalBoundingBoxSize(detection, track)) {
        return true;
      }
    }
  }

  return false;
}

void updateBBoxCenterFromFilter(TrackState& track) {
  const Eigen::Vector3d position = track.filter->position();
  track.bbox.center = makePoint(position.x(), position.y(), position.z());
}

void appendHistorySample(TrackState& track, const ros::Time& stamp, const bool matched,
                         const std::size_t max_history_size) {
  TrackHistorySample sample;
  sample.stamp = stamp;
  sample.object_class = track.object_class;
  sample.motion_model_type = track.model_type;
  sample.model_state = track.filter->state();
  sample.model_covariance = track.filter->covariance();
  sample.matched = matched;
  track.history.push_back(std::move(sample));

  if (max_history_size == 0U) {
    track.history.clear();
    return;
  }

  if (track.history.size() > max_history_size) {
    track.history.erase(track.history.begin(),
                        track.history.begin() +
                            static_cast<std::ptrdiff_t>(track.history.size() - max_history_size));
  }
}

CostMatrix buildCostMatrix(const std::vector<DynamicObjectDetection>& detections,
                           const std::vector<TrackState>& tracks,
                           const DynamicObjectTrackerConfig& config,
                           const double gate_threshold,
                           const double coasting_gate_relax_factor,
                           const double invalid_cost) {
  CostMatrix matrix(detections.size(), std::vector<double>(tracks.size(), invalid_cost));

  for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
    const Eigen::Vector3d detection_position = pointToEigen(detections[detection_index].bbox.center);
    for (std::size_t track_index = 0; track_index < tracks.size(); ++track_index) {
      // cost 仍保留原始马氏距离平方；是否可匹配由对应轨迹的 gate 决定。
      const double cost = computeMahalanobisCost(detection_position, tracks[track_index]);
      const double relaxed_gate =
          tracks[track_index].missed_frames > 0U
              ? gate_threshold * coasting_gate_relax_factor
              : gate_threshold;
      if (std::isfinite(cost) && cost <= relaxed_gate) {
        matrix[detection_index][track_index] = cost;
        continue;
      }

      // 主关联仍优先使用马氏距离；只有它被 gate 拒绝时，才用距离/IoU 阈值给这对
      // detection-track 一个“次优回退候选”，专门补原地旋转或小半径旋转时的中心抖动。
      const double fallback_cost = computeFallbackAssociationCost(
          detections[detection_index], tracks[track_index], config, relaxed_gate, invalid_cost);
      if (fallback_cost < invalid_cost) {
        matrix[detection_index][track_index] = fallback_cost;
      }
    }
  }

  return matrix;
}

std::vector<AssociationCandidate> selectAssociations(const CostMatrix& original,
                                                     const double invalid_cost) {
  if (original.empty() || original.front().empty()) {
    return {};
  }

  const std::size_t detection_count = original.size();
  const std::size_t track_count = original.front().size();
  const std::size_t dimension = detection_count + track_count;
  const double unmatched_penalty = invalid_cost * 0.5;

  // 扩展成方阵后用 Hungarian 算法求全局最小匹配；dummy 行/列显式表示未匹配。
  CostMatrix cost(dimension, std::vector<double>(dimension, 0.0));
  for (std::size_t row = 0; row < dimension; ++row) {
    for (std::size_t column = 0; column < dimension; ++column) {
      if (row < detection_count && column < track_count) {
        cost[row][column] =
            original[row][column] < invalid_cost ? original[row][column] : invalid_cost;
      } else if ((row < detection_count && column >= track_count) ||
                 (row >= detection_count && column < track_count)) {
        // dummy 行/列表示漏检或未匹配 detection，代价高于任何合法 gate 内匹配。
        cost[row][column] = unmatched_penalty;
      }
    }
  }

  std::vector<double> row_potential(dimension + 1U, 0.0);
  std::vector<double> column_potential(dimension + 1U, 0.0);
  std::vector<std::size_t> assigned_row_for_column(dimension + 1U, 0U);
  std::vector<std::size_t> previous_column(dimension + 1U, 0U);

  for (std::size_t row = 1U; row <= dimension; ++row) {
    assigned_row_for_column[0] = row;
    std::size_t current_column = 0U;
    std::vector<double> min_value(dimension + 1U, std::numeric_limits<double>::infinity());
    std::vector<bool> used_column(dimension + 1U, false);

    do {
      // Hungarian 势能更新：逐步扩展交替树，直到找到可增广的未占用列。
      used_column[current_column] = true;
      const std::size_t current_row = assigned_row_for_column[current_column];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t next_column = 0U;

      for (std::size_t column = 1U; column <= dimension; ++column) {
        if (used_column[column]) {
          continue;
        }
        const double candidate =
            cost[current_row - 1U][column - 1U] - row_potential[current_row] -
            column_potential[column];
        if (candidate < min_value[column]) {
          min_value[column] = candidate;
          previous_column[column] = current_column;
        }
        if (min_value[column] < delta) {
          delta = min_value[column];
          next_column = column;
        }
      }

      for (std::size_t column = 0U; column <= dimension; ++column) {
        if (used_column[column]) {
          row_potential[assigned_row_for_column[column]] += delta;
          column_potential[column] -= delta;
        } else {
          min_value[column] -= delta;
        }
      }
      current_column = next_column;
    } while (assigned_row_for_column[current_column] != 0U);

    do {
      const std::size_t next_column = previous_column[current_column];
      assigned_row_for_column[current_column] = assigned_row_for_column[next_column];
      current_column = next_column;
    } while (current_column != 0U);
  }

  std::vector<std::size_t> assigned_column_for_row(dimension, dimension);
  for (std::size_t column = 1U; column <= dimension; ++column) {
    if (assigned_row_for_column[column] != 0U) {
      assigned_column_for_row[assigned_row_for_column[column] - 1U] = column - 1U;
    }
  }

  std::vector<AssociationCandidate> matches;
  matches.reserve(std::min(detection_count, track_count));
  for (std::size_t detection_index = 0; detection_index < detection_count; ++detection_index) {
    const std::size_t track_index = assigned_column_for_row[detection_index];
    if (track_index >= track_count) {
      continue;
    }
    const double raw_cost = original[detection_index][track_index];
    // 只把原始 gate 内的真实 detection-track 组合返回；dummy 匹配在这里被过滤掉。
    if (raw_cost < invalid_cost) {
      matches.push_back({detection_index, track_index, raw_cost});
    }
  }

  return matches;
}

}  // namespace

TrackState::TrackState(const std::uint32_t track_id,
                       const MotionModelType track_model_type,
                       const MultiModelKalmanFilterConfig& filter_config)
    : id(track_id),
      model_type(track_model_type),
      filter(createKalmanFilter(track_model_type, filter_config)) {
  if (filter == nullptr) {
    throw std::runtime_error("failed to construct track filter");
  }
}

TrackState::~TrackState() = default;

TrackState::TrackState(TrackState&&) noexcept = default;

TrackState& TrackState::operator=(TrackState&&) noexcept = default;

DynamicObjectTracker::DynamicObjectTracker(
    ros::NodeHandle& pnh,
    const bool verbose)
    : pnh_(pnh),
      config_(),
      params_(),
      tracks_(),
      next_track_id_(0U),
      verbose_(verbose) {
  loadParameters();

  if (verbose_) {
    ROS_INFO_STREAM("Dynamic object tracker is ready. gate threshold: "
                    << config_.association_gate_threshold
                    << ", history size: " << config_.max_history_size
                    << ", max missed frames: " << config_.max_coast_frames
                    << ", min hits to publish: " << config_.min_hits_to_publish);
  }
}

void DynamicObjectTracker::loadParameters() {
  // 默认值只从参数快照结构体取，避免读参处和结构体初值各维护一份数字。
  const DynamicObjectTrackerParams defaults;
  pnh_.param("tracking_history_size", params_.history_size, defaults.history_size);
  pnh_.param("tracking_max_missed_frames", params_.max_missed_frames, defaults.max_missed_frames);
  pnh_.param("tracking_max_publish_missed_frames", params_.max_publish_missed_frames,
             defaults.max_publish_missed_frames);
  pnh_.param("tracking_min_hits_to_publish", params_.min_hits_to_publish, defaults.min_hits_to_publish);
  pnh_.param("tracking_motion_min_displacement", params_.motion_min_displacement,
             defaults.motion_min_displacement);
  pnh_.param("tracking_motion_min_evidence_frames", params_.motion_min_evidence_frames,
             defaults.motion_min_evidence_frames);
  pnh_.param("tracking_motion_confirmation_speed", params_.motion_confirmation_speed,
             defaults.motion_confirmation_speed);
  pnh_.param("tracking_association_gate_confidence", params_.association_gate_confidence, defaults.association_gate_confidence);
  pnh_.param("tracking_coasting_gate_relax_factor", params_.coasting_gate_relax_factor, defaults.coasting_gate_relax_factor);
  pnh_.param("tracking_box_size_smoothing_alpha", params_.box_size_smoothing_alpha, defaults.box_size_smoothing_alpha);
  pnh_.param("tracking_spawn_suppression_distance", params_.spawn_suppression_distance,  defaults.spawn_suppression_distance);
  pnh_.param("tracking_spawn_suppression_iou_threshold", params_.spawn_suppression_iou_threshold, defaults.spawn_suppression_iou_threshold);
  pnh_.param("tracking_duplicate_merge_distance", params_.duplicate_merge_distance,
             defaults.duplicate_merge_distance);
  pnh_.param("tracking_default_dt", params_.filter.default_dt, defaults.filter.default_dt);
  pnh_.param("tracking_max_dt", params_.max_dt, defaults.max_dt);
  pnh_.param("tracking_classification_enabled", params_.classification_enabled,
             defaults.classification_enabled);
  pnh_.param("classification_start_frame", params_.classification_start_frame, defaults.classification_start_frame);
  pnh_.param("classification_score_decay", params_.classification_score_decay, defaults.classification_score_decay);
  pnh_.param("classification_score_increment", params_.classification_score_increment, defaults.classification_score_increment);
  pnh_.param("classification_confirm_score", params_.classification_confirm_score, defaults.classification_confirm_score);
  pnh_.param("classification_switch_margin", params_.classification_switch_margin, defaults.classification_switch_margin);
  pnh_.param("classification_size_change_ratio", params_.classification_size_change_ratio, defaults.classification_size_change_ratio);
  pnh_.param("classification_point_count_change_ratio", params_.classification_point_count_change_ratio, defaults.classification_point_count_change_ratio);
  pnh_.param("classification_size_change_confirm_frames", params_.classification_size_change_confirm_frames, defaults.classification_size_change_confirm_frames);
  pnh_.param("classify_human_threshold", params_.classify_human_threshold, defaults.classify_human_threshold);
  pnh_.param("classify_vehicle_threshold", params_.classify_vehicle_threshold, defaults.classify_vehicle_threshold);
  pnh_.param("classify_uav_threshold", params_.classify_uav_threshold, defaults.classify_uav_threshold);

  // Kalman 噪声和协方差只从 LDOT 风格分组读取，避免旧顶层参数与当前分组参数并存。
  pnh_.param("kalman_filter/adaptive_window_size", params_.filter.adaptive_window_size, defaults.filter.adaptive_window_size);
  pnh_.param("kalman_filter/adaptive_alpha", params_.filter.adaptive_alpha, defaults.filter.adaptive_alpha);
  pnh_.param("kalman_filter/adaptive_r_alpha", params_.filter.adaptive_r_alpha, defaults.filter.adaptive_r_alpha);
  pnh_.param("kalman_filter/adaptive_min_noise_ratio", params_.filter.adaptive_min_noise_ratio, defaults.filter.adaptive_min_noise_ratio);
  pnh_.param("kalman_filter/enable_cov_limit", params_.filter.enable_cov_limit, defaults.filter.enable_cov_limit);
  pnh_.param("kalman_filter/max_pos_cov", params_.filter.max_pos_cov, defaults.filter.max_pos_cov);
  pnh_.param("kalman_filter/max_vel_cov", params_.filter.max_vel_cov, defaults.filter.max_vel_cov);
  pnh_.param("kalman_filter/max_acc_cov", params_.filter.max_acc_cov, defaults.filter.max_acc_cov);
  pnh_.param("kalman_filter/ca_model/human/jerk_sigma", params_.filter.ca_human_jerk_sigma, defaults.filter.ca_human_jerk_sigma);
  pnh_.param("kalman_filter/ca_model/human/init_cov", params_.filter.ca_human_init_cov, defaults.filter.ca_human_init_cov);
  pnh_.param("kalman_filter/ca_model/human/meas_noise", params_.filter.ca_human_meas_noise, defaults.filter.ca_human_meas_noise);
  pnh_.param("kalman_filter/ca_model/human/z_process_noise", params_.filter.ca_human_z_process_noise, defaults.filter.ca_human_z_process_noise);
  pnh_.param("kalman_filter/ca_model/uav/jerk_sigma", params_.filter.ca_uav_jerk_sigma, defaults.filter.ca_uav_jerk_sigma);
  pnh_.param("kalman_filter/ca_model/uav/init_cov", params_.filter.ca_uav_init_cov, defaults.filter.ca_uav_init_cov);
  pnh_.param("kalman_filter/ca_model/uav/meas_noise", params_.filter.ca_uav_meas_noise, defaults.filter.ca_uav_meas_noise);
  pnh_.param("kalman_filter/cv_model/acc_sigma", params_.filter.cv_acc_sigma, defaults.filter.cv_acc_sigma);
  pnh_.param("kalman_filter/cv_model/init_cov", params_.filter.cv_init_cov, defaults.filter.cv_init_cov);
  pnh_.param("kalman_filter/cv_model/meas_noise", params_.filter.cv_meas_noise, defaults.filter.cv_meas_noise);
  pnh_.param("kalman_filter/ctra_model/init_cov", params_.filter.ctra_init_cov, defaults.filter.ctra_init_cov);
  pnh_.param("kalman_filter/ctra_model/process_noise", params_.filter.ctra_process_noise, defaults.filter.ctra_process_noise);
  pnh_.param("kalman_filter/ctra_model/meas_noise", params_.filter.ctra_meas_noise, defaults.filter.ctra_meas_noise);

  config_ = buildTrackerConfig(params_);
}

DynamicObjectTrackerFrameResult DynamicObjectTracker::processDynamicTracks(
    const std_msgs::Header& header,
    const std::vector<DynamicObjectDetection>& detections) {
  const auto total_start = std::chrono::steady_clock::now();
  DynamicObjectTrackerFrameResult result;

  // 单帧主流程按固定顺序推进：
  // 1) 所有旧轨迹先 predict 到当前时间；
  // 2) 当前 detections 与预测轨迹做全局关联；
  // 3) 已匹配轨迹 update，未匹配 detection 可能建轨，未匹配旧轨迹 coast；
  // 4) 删除过期轨迹，再构建对外消息和 RViz marker。
  // Kalman 数学只在单条轨迹内部执行，这里负责的是“这一帧每条轨迹走哪条生命周期分支”。

  // 先把所有已有轨迹预测到当前帧时间，再用当前 detection 做同一时刻的数据关联。
  predictTracks(header.stamp);

  const auto association_start = std::chrono::steady_clock::now();
  std::vector<bool> matched_detections(detections.size(), false);
  std::vector<bool> matched_tracks(tracks_.size(), false);
  const CostMatrix original_cost =
      buildCostMatrix(detections,
                      tracks_,
                      config_,
                      config_.association_gate_threshold,
                      config_.coasting_gate_relax_factor,
                      config_.invalid_cost);
  const auto matches = selectAssociations(original_cost, config_.invalid_cost);
  result.timing.association_ms = elapsedMs(association_start, std::chrono::steady_clock::now());

  const auto update_tracks_start = std::chrono::steady_clock::now();
  std::size_t suppressed_track_spawns = 0U;
  for (const auto& match : matches) {
    // 匹配成功的轨迹用 detection 更新滤波器，同时刷新 bbox 尺寸和历史样本。
    matched_detections[match.detection_index] = true;
    matched_tracks[match.track_index] = true;
    updateMatchedTrack(tracks_[match.track_index], detections[match.detection_index], header.stamp);
  }

  for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
    if (!matched_detections[detection_index]) {
      // 未匹配 detection 先经过建轨抑制，防止同一目标中心抖动时立即产生新 ID。
      if (shouldSuppressTrackSpawn(detections[detection_index], tracks_, config_)) {
        ++suppressed_track_spawns;
        continue;
      }
      createTrack(detections[detection_index], header.stamp);
    }
  }

  for (std::size_t track_index = 0; track_index < matched_tracks.size(); ++track_index) {
    if (!matched_tracks[track_index]) {
      // 未匹配轨迹进入 coast 状态，保留短期重关联机会，而不是立刻删除。
      coastTrack(tracks_[track_index], header.stamp);
    }
  }

  mergeDuplicateTracks();
  deleteExpiredTracks();
  result.timing.update_tracks_ms = elapsedMs(update_tracks_start, std::chrono::steady_clock::now());

  const auto build_output_start = std::chrono::steady_clock::now();
  result.dynamic_objects_msg = buildOutput(header);
  result.prediction_inputs = buildPredictionInputs(header);
  result.timing.build_output_ms = elapsedMs(build_output_start, std::chrono::steady_clock::now());

  const auto build_markers_start = std::chrono::steady_clock::now();
  result.dynamic_track_markers_msg = buildTrackMarkers(header);
  result.timing.build_track_markers_ms =
      elapsedMs(build_markers_start, std::chrono::steady_clock::now());

  result.timing.process_dynamic_tracks_ms = elapsedMs(total_start, std::chrono::steady_clock::now());

  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Dynamic object tracker timing"
                                      << ": processDynamicTracks="
                                      << formatFloatMs(result.timing.process_dynamic_tracks_ms)
                                      << "ms, association="
                                      << formatFloatMs(result.timing.association_ms)
                                      << "ms, updateTracks="
                                      << formatFloatMs(result.timing.update_tracks_ms)
                                      << "ms, buildOutput="
                                      << formatFloatMs(result.timing.build_output_ms)
                                      << "ms, buildTrackMarkers="
                                      << formatFloatMs(result.timing.build_track_markers_ms)
                                      << "ms, detections=" << detections.size()
                                      << ", tracks=" << tracks_.size()
                                      << ", suppressedSpawns=" << suppressed_track_spawns
                                      << ", publishedObjects=" << result.dynamic_objects_msg.objects.size()
                                      << ", predictionInputs=" << result.prediction_inputs.size());
  }

  return result;
}

visualization_msgs::MarkerArray DynamicObjectTracker::buildTrackMarkers(
    const std_msgs::Header& header) const {
  visualization_msgs::MarkerArray markers;
  markers.markers.reserve(1U + tracks_.size() * 4U);
  markers.markers.push_back(makeDeleteAllMarker(header, "dynamic_tracks"));

  for (const auto& track : tracks_) {
    if (!isPublishableTrack(track, config_)) {
      continue;
    }
    const bool coasting = track.missed_frames > 0U;
    std_msgs::ColorRGBA color;
    // 漏检轨迹保留同一颜色但降低透明度，方便 RViz 区分预测延续和本帧匹配。
    color.r = 0.10F;
    color.g = 0.80F;
    color.b = 0.95F;
    color.a = coasting ? 0.35F : 1.0F;

    visualization_msgs::Marker bbox_marker;
    bbox_marker.header = header;
    bbox_marker.ns = "dynamic_track_bbox";
    bbox_marker.id = static_cast<int>(track.id);
    bbox_marker.type = visualization_msgs::Marker::LINE_LIST;
    bbox_marker.action = visualization_msgs::Marker::ADD;
    bbox_marker.pose.orientation.w = 1.0;
    bbox_marker.scale.x = 0.08;
    bbox_marker.color = color;
    appendTrackBoundingBoxEdges(track.bbox, bbox_marker);
    markers.markers.push_back(std::move(bbox_marker));

    visualization_msgs::Marker history_marker;
    history_marker.header = header;
    history_marker.ns = "dynamic_track_history";
    history_marker.id = static_cast<int>(track.id * 2U);
    history_marker.type = visualization_msgs::Marker::LINE_STRIP;
    history_marker.action = visualization_msgs::Marker::ADD;
    history_marker.pose.orientation.w = 1.0;
    history_marker.scale.x = 0.04;
    history_marker.color = color;
    history_marker.points.reserve(track.history.size());
    for (const auto& sample : track.history) {
      // 当前所有运动模型前三维都固定为 x/y/z；历史线只需要位置，不重新拆出公共状态结构。
      history_marker.points.push_back(
          makePoint(sample.model_state(0), sample.model_state(1), sample.model_state(2)));
    }
    // RViz 要求 LINE_STRIP 至少有 2 个点；新轨迹首帧先显示头部和文字，避免 history marker 报错。
    if (history_marker.points.size() >= 2U) {
      markers.markers.push_back(std::move(history_marker));
    }

    visualization_msgs::Marker head_marker;
    head_marker.header = header;
    head_marker.ns = "dynamic_track_head";
    head_marker.id = static_cast<int>(track.id * 2U + 1U);
    head_marker.type = visualization_msgs::Marker::SPHERE;
    head_marker.action = visualization_msgs::Marker::ADD;
    head_marker.pose.position = track.bbox.center;
    head_marker.pose.orientation.w = 1.0;
    head_marker.scale.x = 0.18;
    head_marker.scale.y = 0.18;
    head_marker.scale.z = 0.18;
    head_marker.color = color;
    markers.markers.push_back(std::move(head_marker));

    if (coasting) {
      visualization_msgs::Marker coast_box_marker;
      coast_box_marker.header = header;
      coast_box_marker.ns = "dynamic_track_coast_bbox";
      coast_box_marker.id = static_cast<int>(track.id);
      coast_box_marker.type = visualization_msgs::Marker::LINE_LIST;
      coast_box_marker.action = visualization_msgs::Marker::ADD;
      coast_box_marker.pose.orientation.w = 1.0;
      coast_box_marker.scale.x = 0.07;
      coast_box_marker.color.r = 0.10F;
      coast_box_marker.color.g = 0.90F;
      coast_box_marker.color.b = 1.0F;
      coast_box_marker.color.a = 0.85F;
      appendTrackBoundingBoxEdges(track.bbox, coast_box_marker);
      markers.markers.push_back(std::move(coast_box_marker));
    }

    visualization_msgs::Marker label_marker;
    label_marker.header = header;
    label_marker.ns = "dynamic_track_label";
    label_marker.id = static_cast<int>(track.id);
    label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label_marker.action = visualization_msgs::Marker::ADD;
    label_marker.pose.position = track.bbox.center;
    // TEXT_VIEW_FACING 的锚点在文字中心；按目标高度上移，避免文字压住头部球体。
    label_marker.pose.position.z +=
        std::max(0.45, 0.5 * std::max(track.bbox.size.z, 0.0) + 0.30);
    label_marker.pose.orientation.w = 1.0;
    // TEXT_VIEW_FACING 只用 scale.z 表达文字高度；使用白色提高 ID 在点云上的可读性。
    label_marker.scale.z = 0.45;
    label_marker.color.r = 1.0F;
    label_marker.color.g = 1.0F;
    label_marker.color.b = 1.0F;
    label_marker.color.a = coasting ? 0.45F : 1.0F;
    label_marker.text = makeTrackLabelText(track);
    markers.markers.push_back(std::move(label_marker));
  }

  return markers;
}

void DynamicObjectTracker::predictTracks(const ros::Time& stamp) {
  // 如果当前是第一帧，`tracks_` 为空，这个循环什么都不做。
  for (auto& track : tracks_) {
    // 计算预测步长
    const double dt =
        computeDeltaSeconds(stamp, track.last_stamp, config_.filter_config.default_dt, config_.max_dt);
    // setDt() 这里只把本次预测要使用的 dt 交给运动模型；
    // F/Q 的读取发生在 predict()，adaptive Q/R 的估计发生在 update()。
    track.filter->setDt(dt);
    track.filter->predict();
    updateBBoxCenterFromFilter(track);
  }
}

void DynamicObjectTracker::updateMatchedTrack(TrackState& track,
                                             const DynamicObjectDetection& detection,
                                             const ros::Time& stamp) {
  const std::size_t matched_hits = track.hits + 1U;
  const double observed_displacement =
      distance3D(detection.bbox.center, lastMatchedTrackCenter(track));
  if (std::isfinite(observed_displacement) &&
      observed_displacement >= config_.motion_min_displacement) {
    ++track.motion_evidence_frames;
  }
  const geometry_msgs::Vector3 stable_size = config_.classification_enabled
      ? prepareMatchedTrackForUpdate(track, detection, matched_hits)
      : updateClassificationSizeState(track, detection, config_).stable_size;
  if (!config_.classification_enabled) {
    track.object_class = ObjectClass::Unknown;
  }

  // filter 只接收 detection 中心作为 3D 位置观测；bbox 尺寸另行维护，不进入状态向量。
  track.filter->update(pointToEigen(detection.bbox.center));
  if (track.motion_evidence_frames >= config_.motion_min_evidence_frames &&
      track.filter->velocity().norm() >= config_.motion_confirmation_speed) {
    track.motion_confirmed = true;
  }
  const auto previous_size = track.bbox.size;
  track.bbox = detection.bbox;
  // 尺寸变化通常比中心位置更受点云稀疏/视角影响，因此只对 size 做指数平滑。
  track.bbox.size.x = config_.box_size_smoothing_alpha * stable_size.x +
                      (1.0 - config_.box_size_smoothing_alpha) * previous_size.x;
  track.bbox.size.y = config_.box_size_smoothing_alpha * stable_size.y +
                      (1.0 - config_.box_size_smoothing_alpha) * previous_size.y;
  track.bbox.size.z = config_.box_size_smoothing_alpha * stable_size.z +
                      (1.0 - config_.box_size_smoothing_alpha) * previous_size.z;
  updateBBoxCenterFromFilter(track);
  track.age += 1U;
  track.hits += 1U;
  track.missed_frames = 0U;
  track.last_stamp = stamp;
  appendHistorySample(track, stamp, true, config_.max_history_size);
}

geometry_msgs::Vector3 DynamicObjectTracker::prepareMatchedTrackForUpdate(
    TrackState& track,
    const DynamicObjectDetection& detection,
    const std::size_t matched_hits) {
  const ClassificationSizeUpdate size_update =
      updateClassificationSizeState(track, detection, config_);

  if (matched_hits < config_.classification_start_frame ||
      !size_update.reliable_for_classification) {
    return size_update.stable_size;
  }

  const ObjectClass observed_class =
      classifyObservation(detection.bbox, size_update.stable_size, config_);
  const std::size_t score_index = classScoreIndex(observed_class);
  if (score_index >= track.classification_scores.size()) {
    return size_update.stable_size;
  }

  // 可靠观测才做 decay + increment；coasting、临时粘连和遮挡残片都冻结分数。
  for (double& score : track.classification_scores) {
    score *= config_.classification_score_decay;
  }
  track.classification_scores[score_index] += config_.classification_score_increment;

  const std::optional<ObjectClass> confirmed_class =
      confirmedClassFromScores(track.classification_scores, config_);
  if (!confirmed_class.has_value()) {
    return size_update.stable_size;
  }

  if (*confirmed_class != track.object_class) {
    switchTrackModel(track, *confirmed_class);
    track.object_class = *confirmed_class;
  }
  return size_update.stable_size;
}

void DynamicObjectTracker::switchTrackModel(TrackState& track, const ObjectClass target_class) {
  const MotionModelType target_model = motionModelForClass(target_class, track.model_type);
  if (target_model == track.model_type) {
    return;
  }

  double preserved_yaw = 0.0;
  bool has_preserved_yaw = false;
  if (track.model_type == MotionModelType::CTRA && track.filter->state().size() >= 6) {
    preserved_yaw = track.filter->state()(5);
    has_preserved_yaw = true;
  }

  const Eigen::Vector3d position = track.filter->position();
  const Eigen::Vector3d velocity = track.filter->velocity();
  auto new_filter = createKalmanFilter(target_model, config_.filter_config);
  if (new_filter == nullptr) {
    throw std::runtime_error("failed to construct switched track filter");
  }

  // 模型切换发生在当前帧 update 之前：先把预测后的旧状态迁移到新模型，
  // 再让新模型吸收本帧 detection，尽量贴近 LDOT 的 switchKalmanModel() 顺序。
  new_filter->initializeState(stateForModel(target_model,
                                            position,
                                            velocity,
                                            preserved_yaw,
                                            has_preserved_yaw));
  track.model_type = target_model;
  track.filter = std::move(new_filter);
  updateBBoxCenterFromFilter(track);
}

void DynamicObjectTracker::createTrack(const DynamicObjectDetection& detection, const ros::Time& stamp) {
  TrackState track(next_track_id_++, MotionModelType::CV3D, config_.filter_config);
  // 参考 LDOT：新轨迹直接以首帧 detection 中心初始化状态，避免首帧仍被 Kalman 增益
  // 拉向原点/旧参考点，导致蓝色轨迹头落在无人机与目标之间。
  track.filter->initialize(pointToEigen(detection.bbox.center));
  track.bbox = detection.bbox;
  track.object_class = ObjectClass::Unknown;
  track.max_observed_size = detection.bbox.size;
  track.last_point_count = detection.point_count;
  updateBBoxCenterFromFilter(track);
  track.age = 1U;
  track.hits = 1U;
  track.missed_frames = 0U;
  track.last_stamp = stamp;
  appendHistorySample(track, stamp, true, config_.max_history_size);
  tracks_.push_back(std::move(track));
}

void DynamicObjectTracker::coastTrack(TrackState& track, const ros::Time& stamp) {
  track.age += 1U;
  track.missed_frames += 1U;
  track.last_stamp = stamp;
  updateBBoxCenterFromFilter(track);
  // 漏检样本同样记录公共状态协方差；预测保活阶段的不确定性对后续调试和预测更关键。
  appendHistorySample(track, stamp, false, config_.max_history_size);
}

void DynamicObjectTracker::mergeDuplicateTracks() {
  if (config_.duplicate_merge_distance <= 0.0 || tracks_.size() < 2U) {
    return;
  }

  const auto is_preferred = [this](const TrackState& lhs, const TrackState& rhs) {
    const bool lhs_has_motion = lhs.motion_confirmed;
    const bool rhs_has_motion = rhs.motion_confirmed;
    if (lhs_has_motion != rhs_has_motion) {
      return lhs_has_motion;
    }
    const bool lhs_publishable = lhs.hits >= config_.min_hits_to_publish;
    const bool rhs_publishable = rhs.hits >= config_.min_hits_to_publish;
    if (lhs_publishable != rhs_publishable) {
      return lhs_publishable;
    }
    if (lhs.hits != rhs.hits) {
      return lhs.hits > rhs.hits;
    }
    if (lhs.missed_frames != rhs.missed_frames) {
      return lhs.missed_frames < rhs.missed_frames;
    }
    if (lhs.age != rhs.age) {
      return lhs.age > rhs.age;
    }
    return lhs.id < rhs.id;
  };

  bool merged = true;
  while (merged) {
    merged = false;
    for (std::size_t lhs_index = 0U;
         lhs_index < tracks_.size() && !merged;
         ++lhs_index) {
      for (std::size_t rhs_index = lhs_index + 1U;
           rhs_index < tracks_.size();
           ++rhs_index) {
        const double center_distance =
            distance3D(tracks_[lhs_index].bbox.center, tracks_[rhs_index].bbox.center);
        if (!(std::isfinite(center_distance) &&
              center_distance <= config_.duplicate_merge_distance)) {
          continue;
        }

        const bool one_track_is_unconfirmed =
            tracks_[lhs_index].motion_confirmed != tracks_[rhs_index].motion_confirmed;
        const bool one_track_is_coasting =
            tracks_[lhs_index].missed_frames > 0U || tracks_[rhs_index].missed_frames > 0U;
        const bool likely_partial_fragment =
            horizontalBoundingBoxSizeRatio(tracks_[lhs_index].bbox.size,
                                            tracks_[rhs_index].bbox.size) < 0.65;
        const bool can_merge_as_same_dynamic_target =
            (tracks_[lhs_index].motion_confirmed || tracks_[rhs_index].motion_confirmed) &&
            (one_track_is_unconfirmed || one_track_is_coasting || likely_partial_fragment);
        if (!can_merge_as_same_dynamic_target) {
          continue;
        }

        const std::size_t keep_index =
            is_preferred(tracks_[lhs_index], tracks_[rhs_index]) ? lhs_index : rhs_index;
        const std::size_t remove_index = keep_index == lhs_index ? rhs_index : lhs_index;
        tracks_.erase(tracks_.begin() + static_cast<std::ptrdiff_t>(remove_index));
        merged = true;
        break;
      }
    }
  }
}

void DynamicObjectTracker::deleteExpiredTracks() {
  // 只删除连续漏检超过阈值的轨迹；短期漏检由 coastTrack() 继续维护。
  tracks_.erase(std::remove_if(tracks_.begin(),
                               tracks_.end(),
                               [this](const TrackState& track) {
                                 return track.missed_frames > config_.max_coast_frames;
                               }),
                tracks_.end());
}

ldop::DynamicObjectArray DynamicObjectTracker::buildOutput(const std_msgs::Header& header) const {
  ldop::DynamicObjectArray object_array;
  object_array.header = header;

  std::vector<const TrackState*> ordered_tracks;
  ordered_tracks.reserve(tracks_.size());
  for (const auto& track : tracks_) {
    ordered_tracks.push_back(&track);
  }
  std::sort(ordered_tracks.begin(),
            ordered_tracks.end(),
            [](const TrackState* lhs, const TrackState* rhs) { return lhs->id < rhs->id; });

  object_array.objects.reserve(ordered_tracks.size());
  for (const TrackState* track : ordered_tracks) {
    // hits 阈值过滤短轨迹，漏检帧阈值限制对外预测保活时长。
    if (!isPublishableTrack(*track, config_)) {
      continue;
    }
    ldop::DynamicObject object;
    object.id = track->id;
    object.size = track->bbox.size;
    const Eigen::VectorXd& state = track->filter->state();
    object.model_state.reserve(static_cast<std::size_t>(state.size()));
    for (int index = 0; index < state.size(); ++index) {
      object.model_state.push_back(state(index));
    }
    const Eigen::MatrixXd& covariance = track->filter->covariance();
    object.model_covariance.reserve(
        static_cast<std::size_t>(covariance.rows() * covariance.cols()));
    for (int row = 0; row < covariance.rows(); ++row) {
      for (int col = 0; col < covariance.cols(); ++col) {
        object.model_covariance.push_back(covariance(row, col));
      }
    }
    object.object_class = toRosObjectClass(track->object_class);
    object.motion_model_type = toRosMotionModelType(track->model_type);
    object_array.objects.push_back(std::move(object));
  }

  return object_array;
}

std::vector<TrackPredictionInput> DynamicObjectTracker::buildPredictionInputs(
    const std_msgs::Header& header) const {
  std::vector<const TrackState*> ordered_tracks;
  ordered_tracks.reserve(tracks_.size());
  for (const auto& track : tracks_) {
    ordered_tracks.push_back(&track);
  }
  std::sort(ordered_tracks.begin(),
            ordered_tracks.end(),
            [](const TrackState* lhs, const TrackState* rhs) { return lhs->id < rhs->id; });

  std::vector<TrackPredictionInput> prediction_inputs;
  prediction_inputs.reserve(ordered_tracks.size());

  for (const TrackState* track : ordered_tracks) {
    // 与 DynamicObjectArray 保持同一稳定轨迹过滤策略，避免 predictor 以后把临时噪声当作真实目标。
    if (!isPublishableTrack(*track, config_)) {
      continue;
    }

    TrackPredictionInput input;
    input.stamp = header.stamp;
    input.id = track->id;
    input.object_class = track->object_class;
    input.motion_model_type = track->model_type;
    input.model_state = track->filter->state();
    input.model_covariance = track->filter->covariance();
    input.bbox = track->bbox;
    input.history = track->history;
    input.age = track->age;
    input.hits = track->hits;
    input.missed_frames = track->missed_frames;
    input.matched_in_current_frame = track->missed_frames == 0U;
    prediction_inputs.push_back(std::move(input));
  }

  return prediction_inputs;
}

}  // namespace ldopcore
