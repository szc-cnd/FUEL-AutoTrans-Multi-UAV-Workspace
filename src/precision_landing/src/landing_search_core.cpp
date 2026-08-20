#include "precision_landing/landing_search_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace precision_landing {
namespace {

bool finiteVector(const Eigen::Vector3d &value) {
  return value.array().isFinite().all();
}

double median(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2U != 0U) {
    return upper;
  }
  std::nth_element(values.begin(), values.begin() + middle - 1U,
                   values.begin() + middle);
  return 0.5 * (values[middle - 1U] + upper);
}

} // namespace

WorldTargetFilter::WorldTargetFilter(const WorldTargetFilterConfig &config)
    : config_(config) {
  if (config_.stable_samples == 0U || config_.max_sample_gap_sec <= 0.0 ||
      config_.max_position_jump_m <= 0.0 || config_.max_spread_m <= 0.0) {
    throw std::invalid_argument("invalid world-target filter configuration");
  }
}

bool WorldTargetFilter::add(int marker_id, const Eigen::Vector3d &point_world,
                            double stamp_sec) {
  if (marker_id < 0 || !finiteVector(point_world) ||
      !std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
    reset();
    return false;
  }

  if (!samples_.empty()) {
    const double gap = stamp_sec - samples_.back().stamp_sec;
    const double jump = (point_world - samples_.back().point).norm();
    if (marker_id != marker_id_ || gap <= 0.0 ||
        gap > config_.max_sample_gap_sec ||
        jump > config_.max_position_jump_m) {
      reset();
    }
  }

  marker_id_ = marker_id;
  samples_.push_back({point_world, stamp_sec});
  while (samples_.size() > config_.stable_samples) {
    samples_.pop_front();
  }
  recompute();
  return stable_;
}

void WorldTargetFilter::reset() {
  samples_.clear();
  marker_id_ = -1;
  stable_ = false;
  filtered_point_.setZero();
  spread_ = 0.0;
}

bool WorldTargetFilter::stable() const { return stable_; }

int WorldTargetFilter::markerId() const { return marker_id_; }

Eigen::Vector3d WorldTargetFilter::filteredPoint() const {
  return filtered_point_;
}

double WorldTargetFilter::spread() const { return spread_; }

void WorldTargetFilter::recompute() {
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  xs.reserve(samples_.size());
  ys.reserve(samples_.size());
  zs.reserve(samples_.size());
  for (const Sample &sample : samples_) {
    xs.push_back(sample.point.x());
    ys.push_back(sample.point.y());
    zs.push_back(sample.point.z());
  }
  filtered_point_ = Eigen::Vector3d(median(xs), median(ys), median(zs));
  spread_ = 0.0;
  for (const Sample &sample : samples_) {
    spread_ = std::max(spread_, (sample.point - filtered_point_).norm());
  }
  stable_ = samples_.size() == config_.stable_samples &&
            spread_ <= config_.max_spread_m;
}

Eigen::Vector3d
cameraPointToWorld(const Eigen::Vector3d &point_camera,
                   const Eigen::Vector3d &camera_translation_body,
                   const Eigen::Quaterniond &camera_orientation_body,
                   const Eigen::Vector3d &body_translation_world,
                   const Eigen::Quaterniond &body_orientation_world) {
  if (!finiteVector(point_camera) || !finiteVector(camera_translation_body) ||
      !finiteVector(body_translation_world) ||
      !camera_orientation_body.coeffs().array().isFinite().all() ||
      !body_orientation_world.coeffs().array().isFinite().all() ||
      camera_orientation_body.norm() < 1.0e-9 ||
      body_orientation_world.norm() < 1.0e-9) {
    throw std::invalid_argument("non-finite or degenerate landing transform");
  }
  const Eigen::Quaterniond normalized_camera =
      camera_orientation_body.normalized();
  const Eigen::Quaterniond normalized_body =
      body_orientation_world.normalized();
  const Eigen::Vector3d point_body =
      normalized_camera * point_camera + camera_translation_body;
  return normalized_body * point_body + body_translation_world;
}

} // namespace precision_landing
