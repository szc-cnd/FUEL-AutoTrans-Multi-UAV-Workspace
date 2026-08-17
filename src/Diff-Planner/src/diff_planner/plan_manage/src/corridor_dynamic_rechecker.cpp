#include "plan_manage/corridor_dynamic_rechecker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace diff_planner {

CorridorDynamicRechecker::CorridorDynamicRechecker(
    const CorridorDynamicRecheckerConfig& config) : config_(config) {}

void CorridorDynamicRechecker::reset() {
  previous_points_.clear();
  tracks_.clear();
  wall_valid_ = false;
  next_id_ = 1;
}

CorridorDynamicRechecker::Voxel CorridorDynamicRechecker::voxelOf(
    const Eigen::Vector3d& p) const {
  const double r = std::max(0.02, config_.voxel_resolution);
  return {static_cast<int>(std::floor(p.x() / r)),
          static_cast<int>(std::floor(p.y() / r)),
          static_cast<int>(std::floor(p.z() / r))};
}

Eigen::Vector3d CorridorDynamicRechecker::channelForward(
    const Eigen::Vector3d& odom_velocity) {
  Eigen::Vector3d horizontal(odom_velocity.x(), odom_velocity.y(), 0.0);
  if (horizontal.norm() > 0.05) forward_ = horizontal.normalized();
  lateral_ = Eigen::Vector3d(-forward_.y(), forward_.x(), 0.0).normalized();
  return forward_;
}

bool CorridorDynamicRechecker::estimateWalls(
    const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& origin,
    const Eigen::Vector3d& forward, double* left, double* right) const {
  std::vector<double> left_samples, right_samples;
  const double expected = std::max(0.5, config_.corridor_width * 0.5);
  for (const auto& p : points) {
    const Eigen::Vector3d q = p - origin;
    const double longitudinal = q.dot(forward);
    const double lateral = q.dot(lateral_);
    if (std::abs(longitudinal) > 2.5 || p.z() < config_.roi_min_z - 0.4 ||
        p.z() > config_.roi_max_z + 0.4) continue;
    if (lateral < -0.35 && lateral > -std::max(1.5, expected * 2.2))
      left_samples.push_back(lateral);
    if (lateral > 0.35 && lateral < std::max(1.5, expected * 2.2))
      right_samples.push_back(lateral);
  }
  if (left_samples.size() < static_cast<size_t>(config_.min_wall_points) ||
      right_samples.size() < static_cast<size_t>(config_.min_wall_points)) return false;
  auto median = [](std::vector<double> values) {
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
  };
  const double l = median(left_samples);
  const double r = median(right_samples);
  if (r - l < 0.7 || r - l > config_.corridor_width * 1.8) return false;
  *left = l;
  *right = r;
  return true;
}

bool CorridorDynamicRechecker::nearPreviousPoint(const Eigen::Vector3d& point) const {
  const double radius2 = config_.temporal_search_radius * config_.temporal_search_radius;
  for (const auto& previous : previous_points_)
    if ((point - previous).squaredNorm() <= radius2) return true;
  return false;
}

std::vector<CorridorDynamicObject> CorridorDynamicRechecker::publishable(
    const ros::Time& stamp) {
  std::vector<CorridorDynamicObject> output;
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    if (!it->confirmed || (stamp - it->last_seen).toSec() > config_.track_timeout) {
      if ((stamp - it->last_seen).toSec() > config_.track_timeout) it = tracks_.erase(it);
      else ++it;
      continue;
    }
    CorridorDynamicObject object;
    object.id = it->id;
    object.position = it->position + it->velocity * std::min(0.15, (stamp - it->last_seen).toSec());
    object.velocity = it->velocity;
    object.size = it->size;
    output.push_back(object);
    ++it;
  }
  return output;
}

std::vector<CorridorDynamicObject> CorridorDynamicRechecker::process(
    const std::vector<Eigen::Vector3d>& points, const ros::Time& stamp,
    const Eigen::Vector3d& sensor_position, const Eigen::Vector3d& odom_velocity) {
  const Eigen::Vector3d forward = channelForward(odom_velocity);
  double estimated_left = 0.0, estimated_right = 0.0;
  if (estimateWalls(points, sensor_position, forward, &estimated_left, &estimated_right)) {
    const double alpha = wall_valid_ ? 0.15 : 1.0;
    left_wall_ = alpha * estimated_left + (1.0 - alpha) * left_wall_;
    right_wall_ = alpha * estimated_right + (1.0 - alpha) * right_wall_;
    wall_valid_ = true;
  } else if (!wall_valid_) {
    previous_points_ = points;
    return publishable(stamp);
  }

  std::map<Voxel, std::vector<Eigen::Vector3d>> buckets;
  for (const auto& p : points) {
    const Eigen::Vector3d q = p - sensor_position;
    const double lateral = q.dot(lateral_);
    if (p.z() < config_.roi_min_z || p.z() > config_.roi_max_z ||
        lateral < left_wall_ + config_.wall_clearance ||
        lateral > right_wall_ - config_.wall_clearance || !nearPreviousPoint(p)) continue;
    buckets[voxelOf(p)].push_back(p);
  }

  struct Cluster { Eigen::Vector3d center; Eigen::Vector3d size; int count; };
  std::vector<Cluster> clusters;
  for (const auto& item : buckets) {
    if (static_cast<int>(item.second.size()) < config_.min_cluster_voxels) continue;
    Eigen::Vector3d minp = item.second.front(), maxp = minp, sum = Eigen::Vector3d::Zero();
    for (const auto& p : item.second) { sum += p; minp = minp.cwiseMin(p); maxp = maxp.cwiseMax(p); }
    clusters.push_back({sum / item.second.size(), (maxp - minp).cwiseMax(Eigen::Vector3d::Constant(0.12)),
                        static_cast<int>(item.second.size())});
  }

  std::vector<bool> matched(tracks_.size(), false);
  for (const auto& cluster : clusters) {
    int best = -1;
    double best_distance = config_.association_gate;
    for (size_t i = 0; i < tracks_.size(); ++i) {
      const double distance = (tracks_[i].position - cluster.center).norm();
      if (!matched[i] && distance < best_distance) { best = static_cast<int>(i); best_distance = distance; }
    }
    if (best < 0) {
      Track track; track.id = next_id_++; track.position = cluster.center; track.size = cluster.size;
      track.last_seen = stamp; track.hits = 1; track.history.push_back({stamp, cluster.center});
      tracks_.push_back(track); matched.push_back(true); continue;
    }
    Track& track = tracks_[best]; matched[best] = true;
    if (!track.history.empty()) {
      const double dt = (stamp - track.history.back().stamp).toSec();
      if (dt > 1e-3 && dt < 1.0) {
        const Eigen::Vector3d raw_velocity = (cluster.center - track.history.back().position) / dt;
        track.velocity = 0.5 * track.velocity + 0.5 * raw_velocity;
        const double lateral_velocity = track.velocity.dot(lateral_);
        if (lateral_velocity > config_.min_lateral_speed) ++track.positive_lateral;
        if (lateral_velocity < -config_.min_lateral_speed) ++track.negative_lateral;
      }
    }
    track.position = cluster.center; track.size = cluster.size; track.last_seen = stamp;
    track.history.push_back({stamp, cluster.center}); track.hits++; track.misses = 0;
    while (static_cast<int>(track.history.size()) > config_.history_frames) track.history.pop_front();
    const double forward_speed = std::abs(track.velocity.dot(forward));
    const double vertical_speed = std::abs(track.velocity.z());
    if (!track.confirmed && track.hits >= config_.min_confirm_hits &&
        std::max(track.positive_lateral, track.negative_lateral) >= 2 &&
        forward_speed <= config_.max_forward_speed && vertical_speed <= config_.max_vertical_speed)
      track.confirmed = true;
  }
  for (size_t i = 0; i < tracks_.size(); ++i) if (!matched[i]) tracks_[i].misses++;
  previous_points_ = points;
  return publishable(stamp);
}

}  // namespace diff_planner
