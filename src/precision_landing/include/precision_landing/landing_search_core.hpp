#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <deque>

namespace precision_landing {

struct WorldTargetFilterConfig {
  std::size_t stable_samples{8U};
  double max_sample_gap_sec{0.25};
  double max_position_jump_m{0.35};
  double max_spread_m{0.12};
};

class WorldTargetFilter {
public:
  explicit WorldTargetFilter(const WorldTargetFilterConfig &config);

  bool add(int marker_id, const Eigen::Vector3d &point_world, double stamp_sec);
  void reset();
  bool stable() const;
  int markerId() const;
  Eigen::Vector3d filteredPoint() const;
  double spread() const;

private:
  struct Sample {
    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    double stamp_sec{0.0};
  };

  void recompute();

  WorldTargetFilterConfig config_;
  std::deque<Sample> samples_;
  int marker_id_{-1};
  bool stable_{false};
  Eigen::Vector3d filtered_point_{Eigen::Vector3d::Zero()};
  double spread_{0.0};
};

Eigen::Vector3d
cameraPointToWorld(const Eigen::Vector3d &point_camera,
                   const Eigen::Vector3d &camera_translation_body,
                   const Eigen::Quaterniond &camera_orientation_body,
                   const Eigen::Vector3d &body_translation_world,
                   const Eigen::Quaterniond &body_orientation_world);

} // namespace precision_landing
