#include <ldop/dynamic_object_pose_fusion.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <ros/ros.h>

namespace ldopcore {

namespace {

struct AssociationCandidate {
  std::size_t source_index{0U};
  std::size_t detection_index{0U};
  double cost{0.0};
};

struct AssociationSearchResult {
  std::vector<int> detection_by_source;
  std::size_t matched_count{0U};
  double total_cost{0.0};
};

Eigen::Vector3d posePositionToEigen(const geometry_msgs::Pose& pose) {
  return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
}

geometry_msgs::Pose makePoseMsg(const Eigen::Vector3d& position,
                                const Eigen::Quaterniond& orientation) {
  geometry_msgs::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

std::optional<Eigen::Quaterniond> normalizedQuaternion(
    const geometry_msgs::Quaternion& quaternion_msg) {
  Eigen::Quaterniond quaternion(quaternion_msg.w,
                                quaternion_msg.x,
                                quaternion_msg.y,
                                quaternion_msg.z);
  const double norm = quaternion.norm();
  if (norm <= 1e-9) {
    return std::nullopt;
  }
  quaternion.normalize();
  return quaternion;
}

Eigen::Vector3d quaternionLog(Eigen::Quaterniond quaternion) {
  quaternion.normalize();
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  const Eigen::Vector3d vector_part(quaternion.x(), quaternion.y(), quaternion.z());
  const double sin_half_angle = vector_part.norm();
  if (sin_half_angle <= 1e-12) {
    return 2.0 * vector_part;
  }
  const double angle = 2.0 * std::atan2(sin_half_angle, quaternion.w());
  return angle * vector_part / sin_half_angle;
}

double ceilToGrid(const double stamp, const double step) {
  return std::ceil((stamp - 1e-9) / step) * step;
}

void searchBestAssociation(
    const std::size_t source_index,
    const std::size_t source_count,
    const std::vector<AssociationCandidate>& candidates,
    std::vector<bool>& used_detections,
    std::vector<int>& current_assignment,
    const std::size_t current_matched_count,
    const double current_cost,
    AssociationSearchResult& best_result) {
  if (source_index >= source_count) {
    if (current_matched_count > best_result.matched_count ||
        (current_matched_count == best_result.matched_count &&
         current_cost < best_result.total_cost)) {
      best_result.detection_by_source = current_assignment;
      best_result.matched_count = current_matched_count;
      best_result.total_cost = current_cost;
    }
    return;
  }

  // 当前目标数第一版很小，递归穷举比引入额外 Hungarian 依赖更透明；
  // 先允许 source 不匹配，保证所有通过 gate 的组合都能参与全局比较。
  current_assignment[source_index] = -1;
  searchBestAssociation(source_index + 1U,
                        source_count,
                        candidates,
                        used_detections,
                        current_assignment,
                        current_matched_count,
                        current_cost,
                        best_result);

  for (const AssociationCandidate& candidate : candidates) {
    if (candidate.source_index != source_index ||
        used_detections[candidate.detection_index]) {
      continue;
    }
    used_detections[candidate.detection_index] = true;
    current_assignment[source_index] = static_cast<int>(candidate.detection_index);
    searchBestAssociation(source_index + 1U,
                          source_count,
                          candidates,
                          used_detections,
                          current_assignment,
                          current_matched_count + 1U,
                          current_cost + candidate.cost,
                          best_result);
    used_detections[candidate.detection_index] = false;
    current_assignment[source_index] = -1;
  }
}

}  // namespace

DynamicObjectPoseFusionConfig buildPoseFusionConfig(
    const DynamicObjectPoseFusionParams& params) {
  const DynamicObjectPoseFusionParams defaults;
  DynamicObjectPoseFusionConfig config;

  config.mocap_topics = params.mocap_topics.empty() ? defaults.mocap_topics : params.mocap_topics;
  const std::vector<double>& translation =
      params.mocap_to_ldop_translation.size() == 3U
          ? params.mocap_to_ldop_translation
          : defaults.mocap_to_ldop_translation;
  config.mocap_to_ldop_translation =
      Eigen::Vector3d(translation[0], translation[1], translation[2]);

  config.mocap_sync_tolerance =
      params.mocap_sync_tolerance > 0.0 ? params.mocap_sync_tolerance
                                        : defaults.mocap_sync_tolerance;
  config.association_max_distance =
      params.association_max_distance > 0.0 ? params.association_max_distance
                                            : defaults.association_max_distance;
  config.offset_alpha = std::clamp(params.offset_alpha, 0.0, 1.0);
  config.offset_observation_max_norm =
      params.offset_observation_max_norm > 0.0 ? params.offset_observation_max_norm
                                               : defaults.offset_observation_max_norm;
  config.offset_update_max_delta =
      params.offset_update_max_delta > 0.0 ? params.offset_update_max_delta
                                           : defaults.offset_update_max_delta;
  config.lidar_anchor_timeout =
      params.lidar_anchor_timeout > 0.0 ? params.lidar_anchor_timeout
                                        : defaults.lidar_anchor_timeout;

  const int bounded_min_samples =
      params.bspline_min_samples >= 5 ? params.bspline_min_samples : 5;
  config.bspline_min_samples = static_cast<std::size_t>(bounded_min_samples);
  const int max_future_samples = std::max(bounded_min_samples - 1, 0);
  const int requested_future_samples =
      params.bspline_future_samples >= 0 ? params.bspline_future_samples
                                         : defaults.bspline_future_samples;
  config.bspline_future_samples =
      static_cast<std::size_t>(std::min(requested_future_samples, max_future_samples));

  config.bspline_reset_translation_delta =
      params.bspline_reset_translation_delta > 0.0 ? params.bspline_reset_translation_delta
                                                   : defaults.bspline_reset_translation_delta;
  config.mocap_nominal_rate =
      params.mocap_nominal_rate > 0.0 ? params.mocap_nominal_rate
                                      : defaults.mocap_nominal_rate;
  config.virtual_imu_rate_hz =
      params.virtual_imu_rate_hz > 0.0 ? params.virtual_imu_rate_hz
                                       : defaults.virtual_imu_rate_hz;
  config.mocap_dt_jitter_tolerance =
      params.mocap_dt_jitter_tolerance >= 0.0 ? params.mocap_dt_jitter_tolerance
                                              : defaults.mocap_dt_jitter_tolerance;
  config.gravity_z = params.gravity_z;
  config.fused_objects_topic = params.fused_objects_topic;
  config.virtual_imu_topic = params.virtual_imu_topic;
  config.marker_topic = params.marker_topic;
  return config;
}

DynamicObjectPoseFusion::DynamicObjectPoseFusion(ros::NodeHandle& pnh,
                                                 const bool verbose)
    : params_(), config_(), verbose_(verbose) {
  loadParameters(pnh);
  sources_.resize(config_.mocap_topics.size());
}

DynamicObjectPoseFusion::DynamicObjectPoseFusion(
    const DynamicObjectPoseFusionConfig& config,
    const bool verbose)
    : params_(), config_(config), sources_(config_.mocap_topics.size()), verbose_(verbose) {}

const std::vector<std::string>& DynamicObjectPoseFusion::mocapTopics() const {
  return config_.mocap_topics;
}

const std::string& DynamicObjectPoseFusion::fusedObjectsTopic() const {
  return config_.fused_objects_topic;
}

const std::string& DynamicObjectPoseFusion::virtualImuTopic() const {
  return config_.virtual_imu_topic;
}

const std::string& DynamicObjectPoseFusion::markerTopic() const {
  return config_.marker_topic;
}

DynamicObjectPoseFusionSnapshot DynamicObjectPoseFusion::processMocapPose(
    const std::size_t mocap_source_index,
    const geometry_msgs::PoseStamped& pose_msg) {
  const std::optional<Eigen::Quaterniond> orientation =
      normalizedQuaternion(pose_msg.pose.orientation);
  if (!orientation.has_value()) {
    return {};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (mocap_source_index >= sources_.size()) {
    return {};
  }
  SourceState& source = sources_[mocap_source_index];
  if (!source.mocap_history.empty() &&
      pose_msg.header.stamp <= source.mocap_history.back().stamp) {
    return {};
  }

  MocapPoseSample sample;
  sample.stamp = pose_msg.header.stamp;
  sample.position_ldop = posePositionToEigen(pose_msg.pose) + config_.mocap_to_ldop_translation;
  sample.orientation_ldop = *orientation;
  if (source.last_valid_quaternion.has_value() &&
      source.last_valid_quaternion->dot(sample.orientation_ldop) < 0.0) {
    // q 和 -q 表示同一个姿态；这里固定符号连续性，避免后续 SO(3) 差分/样条把符号翻转误判成大旋转。
    sample.orientation_ldop.coeffs() *= -1.0;
  }
  source.last_valid_quaternion = sample.orientation_ldop;

  source.mocap_history.push_back(sample);
  while (source.mocap_history.size() > 200U) {
    source.mocap_history.pop_front();
  }

  if (!source.has_anchor) {
    return {};
  }
  const double anchor_age = (sample.stamp - source.anchor_stamp).toSec();
  if (anchor_age > config_.lidar_anchor_timeout) {
    return {};
  }

  DynamicObjectPoseFusionSnapshot snapshot;
  ldop::FusedDynamicObjectArray array;
  array.header.frame_id = pose_msg.header.frame_id;
  array.header.stamp = sample.stamp;
  const Eigen::Vector3d fused_position =
      sample.position_ldop + sample.orientation_ldop * source.offset_body;
  MocapPoseSample fused_pose_sample;
  fused_pose_sample.stamp = sample.stamp;
  fused_pose_sample.position_ldop = fused_position;
  fused_pose_sample.orientation_ldop = sample.orientation_ldop;
  appendFusedPoseKnot(source, fused_pose_sample);

  ldop::FusedDynamicObject object;
  object.header.frame_id = array.header.frame_id;
  object.header.stamp = sample.stamp;
  object.mocap_source_index = static_cast<std::uint32_t>(mocap_source_index);
  object.fused_pose = makePoseMsg(fused_position, sample.orientation_ldop);
  object.bbox_size = source.latest_bbox_size;
  object.point_count = static_cast<std::uint32_t>(source.latest_point_count);
  object.offset_body = eigenToVector(source.offset_body);
  object.lidar_anchor_age = anchor_age;
  object.lidar_updated_in_current_frame = false;
  array.objects.push_back(object);
  snapshot.markers = buildMarkers(array);
  snapshot.fused_objects = std::move(array);
  snapshot.virtual_imu =
      buildVirtualImuArray(mocap_source_index, source, pose_msg.header.frame_id, sample.stamp);
  return snapshot;
}

DynamicObjectPoseFusionSnapshot DynamicObjectPoseFusion::updateLidarAnchors(
    const std_msgs::Header& header,
    const std::vector<DynamicObjectDetection>& detections) {
  std::lock_guard<std::mutex> lock(mutex_);
  DynamicObjectPoseFusionSnapshot snapshot;
  if (detections.empty()) {
    return snapshot;
  }

  struct SyncSourcePose {
    bool available{false};
    MocapPoseSample sample;
  };

  std::vector<SyncSourcePose> source_samples(sources_.size());
  for (std::size_t source_index = 0U; source_index < sources_.size(); ++source_index) {
    SourceState& source = sources_[source_index];
    double best_dt = config_.mocap_sync_tolerance;
    for (const MocapPoseSample& sample : source.mocap_history) {
      const double dt = std::abs((sample.stamp - header.stamp).toSec());
      if (dt <= best_dt) {
        source_samples[source_index].available = true;
        source_samples[source_index].sample = sample;
        best_dt = dt;
      }
    }
  }

  std::vector<AssociationCandidate> candidates;
  for (std::size_t source_index = 0U; source_index < sources_.size(); ++source_index) {
    if (!source_samples[source_index].available) {
      continue;
    }
    const SourceState& source = sources_[source_index];
    const MocapPoseSample& sample = source_samples[source_index].sample;
    for (std::size_t detection_index = 0U; detection_index < detections.size(); ++detection_index) {
      const DynamicObjectDetection& detection = detections[detection_index];
      const Eigen::Vector3d predicted_fused =
          source.has_anchor
              ? sample.position_ldop + sample.orientation_ldop * source.offset_body
              : sample.position_ldop;
      const double cost = (pointToEigen(detection.bbox.center) - predicted_fused).norm();
      if (cost <= config_.association_max_distance) {
        candidates.push_back({source_index, detection_index, cost});
      }
    }
  }

  AssociationSearchResult best_result;
  best_result.detection_by_source = std::vector<int>(sources_.size(), -1);
  best_result.total_cost = std::numeric_limits<double>::infinity();
  std::vector<int> current_assignment(sources_.size(), -1);
  std::vector<bool> used_detections(detections.size(), false);
  searchBestAssociation(0U,
                        sources_.size(),
                        candidates,
                        used_detections,
                        current_assignment,
                        0U,
                        0.0,
                        best_result);

  ldop::FusedDynamicObjectArray array;
  array.header.frame_id = header.frame_id;
  for (std::size_t source_index = 0U; source_index < sources_.size(); ++source_index) {
    if (best_result.detection_by_source[source_index] < 0) {
      continue;
    }
    SourceState& source = sources_[source_index];
    if (!source_samples[source_index].available) {
      continue;
    }
    const MocapPoseSample& sample = source_samples[source_index].sample;
    const DynamicObjectDetection& detection =
        detections[static_cast<std::size_t>(best_result.detection_by_source[source_index])];
    const Eigen::Vector3d lidar_center = pointToEigen(detection.bbox.center);

    const Eigen::Vector3d observed_offset =
        sample.orientation_ldop.inverse() * (lidar_center - sample.position_ldop);
    if (observed_offset.norm() > config_.offset_observation_max_norm) {
      if (verbose_) {
        ROS_WARN_STREAM("rejected fusion offset for source " << source_index
                        << " because observed norm is " << observed_offset.norm());
      }
      continue;
    }
    if (source.has_anchor &&
        (observed_offset - source.offset_body).norm() > config_.offset_update_max_delta) {
      if (verbose_) {
        ROS_WARN_STREAM("rejected fusion offset for source " << source_index
                        << " because delta is "
                        << (observed_offset - source.offset_body).norm());
      }
      continue;
    }

    const bool had_anchor = source.has_anchor;
    const Eigen::Vector3d previous_offset = source.offset_body;
    source.offset_body = source.has_anchor
        ? (1.0 - config_.offset_alpha) * source.offset_body +
              config_.offset_alpha * observed_offset
        : observed_offset;
    const double accepted_offset_correction =
        had_anchor ? (source.offset_body - previous_offset).norm() : 0.0;
    if (!had_anchor ||
        accepted_offset_correction > config_.bspline_reset_translation_delta) {
      // anchor 是融合坐标连续性的起点；offset 大修正会让融合中心出现位置跳变，虚拟 IMU 不跨段做差分。
      source.fused_pose_knots.clear();
      source.pose_grid_knots.clear();
      source.last_published_virtual_imu_eval_stamp.reset();
    }
    source.has_anchor = true;
    source.anchor_stamp = header.stamp;
    source.anchor_mocap_pose_ldop = sample;
    source.latest_bbox_size = detection.bbox.size;
    source.latest_point_count = detection.point_count;

    const Eigen::Vector3d fused_position =
        sample.position_ldop + sample.orientation_ldop * source.offset_body;
    source.anchor_fused_pose = makePoseMsg(fused_position, sample.orientation_ldop);
    MocapPoseSample fused_pose_sample;
    fused_pose_sample.stamp = sample.stamp;
    fused_pose_sample.position_ldop = fused_position;
    fused_pose_sample.orientation_ldop = sample.orientation_ldop;
    appendFusedPoseKnot(source, fused_pose_sample);

    ldop::FusedDynamicObject object;
    object.header.frame_id = header.frame_id;
    object.header.stamp = sample.stamp;
    object.mocap_source_index = static_cast<std::uint32_t>(source_index);
    object.fused_pose = source.anchor_fused_pose;
    object.bbox_size = source.latest_bbox_size;
    object.point_count = static_cast<std::uint32_t>(source.latest_point_count);
    object.offset_body = eigenToVector(source.offset_body);
    object.lidar_anchor_age = (sample.stamp - source.anchor_stamp).toSec();
    object.lidar_updated_in_current_frame = true;
    array.objects.push_back(object);
    array.header.stamp = sample.stamp;
  }

  if (!array.objects.empty()) {
    snapshot.markers = buildMarkers(array);
    snapshot.fused_objects = std::move(array);
  }
  return snapshot;
}

void DynamicObjectPoseFusion::appendFusedPoseKnot(SourceState& source,
                                                 const MocapPoseSample& fused_pose) {
  MocapPoseSample continuous_pose = fused_pose;
  if (!source.fused_pose_knots.empty()) {
    const double dt = (continuous_pose.stamp - source.fused_pose_knots.back().stamp).toSec();
    if (dt <= 1e-9) {
      return;
    }
    const double nominal_dt = 1.0 / config_.mocap_nominal_rate;
    if (std::abs(dt - nominal_dt) > config_.mocap_dt_jitter_tolerance) {
      // 样条窗口假设近似等间隔输入；遇到丢帧或跳时先断开窗口，避免跨不连续段产生虚假速度。
      source.fused_pose_knots.clear();
      source.pose_grid_knots.clear();
      source.last_published_virtual_imu_eval_stamp.reset();
    } else if (source.fused_pose_knots.back().orientation_ldop.dot(
                   continuous_pose.orientation_ldop) < 0.0) {
      continuous_pose.orientation_ldop.coeffs() *= -1.0;
    }
  }

  source.fused_pose_knots.push_back(continuous_pose);
  while (source.fused_pose_knots.size() > 200U) {
    source.fused_pose_knots.pop_front();
  }
  extendPoseGrid(source);
}

void DynamicObjectPoseFusion::extendPoseGrid(SourceState& source) {
  if (source.fused_pose_knots.empty()) {
    return;
  }

  const double nominal_dt = 1.0 / config_.mocap_nominal_rate;
  if (source.pose_grid_knots.empty()) {
    source.pose_grid_knots.push_back(source.fused_pose_knots.front());
  }

  double next_stamp = source.pose_grid_knots.back().stamp.toSec() + nominal_dt;
  const double newest_stamp = source.fused_pose_knots.back().stamp.toSec();
  while (next_stamp <= newest_stamp + 1e-9) {
    std::optional<MocapPoseSample> pose =
        interpolatePoseKnot(source.fused_pose_knots, ros::Time(next_stamp));
    if (!pose.has_value()) {
      break;
    }
    if (!source.pose_grid_knots.empty() &&
        source.pose_grid_knots.back().orientation_ldop.dot(pose->orientation_ldop) < 0.0) {
      pose->orientation_ldop.coeffs() *= -1.0;
    }
    source.pose_grid_knots.push_back(*pose);
    next_stamp += nominal_dt;
  }

  while (source.pose_grid_knots.size() > 200U) {
    source.pose_grid_knots.pop_front();
  }
}

std::optional<DynamicObjectPoseFusion::MocapPoseSample>
DynamicObjectPoseFusion::interpolatePoseKnot(
    const std::deque<MocapPoseSample>& knots,
    const ros::Time& stamp) const {
  if (knots.empty()) {
    return std::nullopt;
  }
  if (stamp < knots.front().stamp - ros::Duration(1e-9) ||
      stamp > knots.back().stamp + ros::Duration(1e-9)) {
    return std::nullopt;
  }
  if (stamp <= knots.front().stamp + ros::Duration(1e-9)) {
    return knots.front();
  }

  for (std::size_t index = 1U; index < knots.size(); ++index) {
    const MocapPoseSample& upper = knots[index];
    if (stamp > upper.stamp + ros::Duration(1e-9)) {
      continue;
    }
    if (std::abs((stamp - upper.stamp).toSec()) <= 1e-9) {
      return upper;
    }
    const MocapPoseSample& lower = knots[index - 1U];
    const double span = (upper.stamp - lower.stamp).toSec();
    if (span <= 1e-9) {
      return std::nullopt;
    }
    const double ratio = (stamp - lower.stamp).toSec() / span;
    Eigen::Quaterniond upper_orientation = upper.orientation_ldop;
    if (lower.orientation_ldop.dot(upper_orientation) < 0.0) {
      upper_orientation.coeffs() *= -1.0;
    }

    MocapPoseSample interpolated;
    interpolated.stamp = stamp;
    interpolated.position_ldop =
        (1.0 - ratio) * lower.position_ldop + ratio * upper.position_ldop;
    interpolated.orientation_ldop = lower.orientation_ldop.slerp(ratio, upper_orientation);
    interpolated.orientation_ldop.normalize();
    return interpolated;
  }

  return knots.back();
}

std::optional<ldop::FusedDynamicObjectVirtualImuArray>
DynamicObjectPoseFusion::buildVirtualImuArray(
    const std::size_t mocap_source_index,
    SourceState& source,
    const std::string& frame_id,
    const ros::Time& available_stamp) {
  if (source.pose_grid_knots.size() < config_.bspline_min_samples ||
      source.pose_grid_knots.size() < 3U) {
    return std::nullopt;
  }

  const double nominal_dt = 1.0 / config_.mocap_nominal_rate;
  const double eval_dt = 1.0 / config_.virtual_imu_rate_hz;
  const std::size_t derivative_delay =
      std::max<std::size_t>(config_.bspline_future_samples, 1U);
  const double earliest_eval =
      source.pose_grid_knots.front().stamp.toSec() + nominal_dt;
  const double latest_confirmed_eval =
      source.pose_grid_knots.back().stamp.toSec() -
      static_cast<double>(derivative_delay) * nominal_dt;
  if (latest_confirmed_eval + 1e-9 < earliest_eval) {
    return std::nullopt;
  }

  double next_eval = ceilToGrid(earliest_eval, eval_dt);
  if (source.last_published_virtual_imu_eval_stamp.has_value()) {
    next_eval = std::max(
        next_eval,
        source.last_published_virtual_imu_eval_stamp->toSec() + eval_dt);
  }

  ldop::FusedDynamicObjectVirtualImuArray array;
  array.header.frame_id = frame_id;
  array.header.stamp = available_stamp;

  while (next_eval <= latest_confirmed_eval + 1e-9) {
    const ros::Time eval_stamp(next_eval);
    const std::optional<MocapPoseSample> previous =
        interpolatePoseKnot(source.pose_grid_knots, ros::Time(next_eval - nominal_dt));
    const std::optional<MocapPoseSample> current =
        interpolatePoseKnot(source.pose_grid_knots, eval_stamp);
    const std::optional<MocapPoseSample> next =
        interpolatePoseKnot(source.pose_grid_knots, ros::Time(next_eval + nominal_dt));
    if (!previous.has_value() || !current.has_value() || !next.has_value()) {
      break;
    }

    Eigen::Quaterniond next_orientation = next->orientation_ldop;
    if (previous->orientation_ldop.dot(next_orientation) < 0.0) {
      next_orientation.coeffs() *= -1.0;
    }
    const Eigen::Quaterniond delta_orientation =
        previous->orientation_ldop.inverse() * next_orientation;
    const Eigen::Vector3d linear_velocity =
        (next->position_ldop - previous->position_ldop) / (2.0 * nominal_dt);
    const Eigen::Vector3d linear_acceleration =
        (next->position_ldop - 2.0 * current->position_ldop + previous->position_ldop) /
        (nominal_dt * nominal_dt);
    const Eigen::Vector3d angular_velocity =
        quaternionLog(delta_orientation) / (2.0 * nominal_dt);
    const Eigen::Vector3d gravity(0.0, 0.0, config_.gravity_z);
    const Eigen::Vector3d specific_force =
        current->orientation_ldop.inverse() * (linear_acceleration - gravity);

    ldop::FusedDynamicObjectVirtualImu sample;
    sample.header.frame_id = frame_id;
    sample.header.stamp = eval_stamp;
    sample.available_stamp = available_stamp;
    sample.mocap_source_index = static_cast<std::uint32_t>(mocap_source_index);
    sample.fused_pose_at_eval =
        makePoseMsg(current->position_ldop, current->orientation_ldop);
    sample.virtual_linear_velocity = eigenToVector(linear_velocity);
    sample.virtual_angular_velocity = eigenToVector(angular_velocity);
    sample.virtual_linear_acceleration = eigenToVector(linear_acceleration);
    sample.virtual_specific_force = eigenToVector(specific_force);
    array.samples.push_back(sample);
    source.last_published_virtual_imu_eval_stamp = eval_stamp;
    next_eval += eval_dt;
  }

  if (array.samples.empty()) {
    return std::nullopt;
  }
  return array;
}

visualization_msgs::MarkerArray DynamicObjectPoseFusion::buildMarkers(
    const ldop::FusedDynamicObjectArray& objects) const {
  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(makeDeleteAllMarker(objects.header, "fused_dynamic_objects"));

  for (const ldop::FusedDynamicObject& object : objects.objects) {
    visualization_msgs::Marker box_marker;
    box_marker.header = object.header;
    box_marker.ns = "fused_dynamic_object_box";
    box_marker.id = static_cast<int>(object.mocap_source_index);
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.action = visualization_msgs::Marker::ADD;
    box_marker.pose = object.fused_pose;
    box_marker.scale = object.bbox_size;
    box_marker.color.r = 0.1F;
    box_marker.color.g = 0.7F;
    box_marker.color.b = 1.0F;
    box_marker.color.a = 0.35F;
    box_marker.lifetime = ros::Duration(0.2);
    markers.markers.push_back(box_marker);

    if (!verbose_) {
      continue;
    }
    const std::optional<Eigen::Quaterniond> orientation =
        normalizedQuaternion(object.fused_pose.orientation);
    if (!orientation.has_value()) {
      continue;
    }
    const Eigen::Vector3d fused_position = posePositionToEigen(object.fused_pose);
    const Eigen::Vector3d offset(object.offset_body.x,
                                 object.offset_body.y,
                                 object.offset_body.z);
    const Eigen::Vector3d rigid_position = fused_position - (*orientation) * offset;

    visualization_msgs::Marker offset_marker;
    offset_marker.header = object.header;
    offset_marker.ns = "fused_dynamic_object_offset";
    offset_marker.id = static_cast<int>(object.mocap_source_index);
    offset_marker.type = visualization_msgs::Marker::LINE_LIST;
    offset_marker.action = visualization_msgs::Marker::ADD;
    offset_marker.scale.x = 0.03;
    offset_marker.color.r = 1.0F;
    offset_marker.color.g = 0.8F;
    offset_marker.color.b = 0.1F;
    offset_marker.color.a = 0.9F;
    offset_marker.points.push_back(
        makePoint(rigid_position.x(), rigid_position.y(), rigid_position.z()));
    offset_marker.points.push_back(
        makePoint(fused_position.x(), fused_position.y(), fused_position.z()));
    offset_marker.lifetime = ros::Duration(0.2);
    markers.markers.push_back(offset_marker);
  }

  return markers;
}

void DynamicObjectPoseFusion::loadParameters(ros::NodeHandle& pnh) {
  const DynamicObjectPoseFusionParams defaults;
  pnh.param("fusion_mocap_topics", params_.mocap_topics, defaults.mocap_topics);
  pnh.param("fusion_mocap_to_ldop_translation",
            params_.mocap_to_ldop_translation,
            defaults.mocap_to_ldop_translation);
  pnh.param("fusion_mocap_sync_tolerance",
            params_.mocap_sync_tolerance,
            defaults.mocap_sync_tolerance);
  pnh.param("fusion_association_max_distance",
            params_.association_max_distance,
            defaults.association_max_distance);
  pnh.param("fusion_offset_alpha", params_.offset_alpha, defaults.offset_alpha);
  pnh.param("fusion_offset_observation_max_norm",
            params_.offset_observation_max_norm,
            defaults.offset_observation_max_norm);
  pnh.param("fusion_offset_update_max_delta",
            params_.offset_update_max_delta,
            defaults.offset_update_max_delta);
  pnh.param("fusion_lidar_anchor_timeout",
            params_.lidar_anchor_timeout,
            defaults.lidar_anchor_timeout);
  pnh.param("fusion_bspline_min_samples",
            params_.bspline_min_samples,
            defaults.bspline_min_samples);
  pnh.param("fusion_bspline_future_samples",
            params_.bspline_future_samples,
            defaults.bspline_future_samples);
  pnh.param("fusion_bspline_reset_translation_delta",
            params_.bspline_reset_translation_delta,
            defaults.bspline_reset_translation_delta);
  pnh.param("fusion_mocap_nominal_rate",
            params_.mocap_nominal_rate,
            defaults.mocap_nominal_rate);
  pnh.param("fusion_virtual_imu_rate_hz",
            params_.virtual_imu_rate_hz,
            defaults.virtual_imu_rate_hz);
  pnh.param("fusion_mocap_dt_jitter_tolerance",
            params_.mocap_dt_jitter_tolerance,
            defaults.mocap_dt_jitter_tolerance);
  pnh.param("fusion_gravity_z", params_.gravity_z, defaults.gravity_z);
  pnh.param("fusion_objects_topic", params_.fused_objects_topic, defaults.fused_objects_topic);
  pnh.param("fusion_virtual_imu_topic", params_.virtual_imu_topic, defaults.virtual_imu_topic);
  pnh.param("fusion_marker_topic", params_.marker_topic, defaults.marker_topic);

  config_ = buildPoseFusionConfig(params_);
}

}  // namespace ldopcore
