#pragma once

#include <deque>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/Vector3.h>
#include <ldop/DynamicObjectArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>

namespace diff_planner {

struct CorridorDynamicRecheckerConfig {
  double corridor_width = 1.5;
  double wall_clearance = 0.20;
  double roi_min_z = 0.15;
  double roi_max_z = 1.50;
  double voxel_resolution = 0.08;
  double temporal_search_radius = 0.30;
  int history_frames = 5;
  int min_confirm_hits = 3;
  double min_lateral_speed = 0.10;
  double max_forward_speed = 0.80;
  double max_vertical_speed = 0.80;
  double association_gate = 0.45;
  double track_timeout = 0.40;
  int min_cluster_voxels = 3;
  int min_wall_points = 12;
};

struct CorridorDynamicObject {
  uint32_t id = 0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d size = Eigen::Vector3d::Constant(0.20);
};

class CorridorDynamicRechecker {
 public:
  explicit CorridorDynamicRechecker(const CorridorDynamicRecheckerConfig& config);

  void reset();

  // Processes one registered cloud in the odometry/world frame. The returned
  // objects are confirmed tracks only; tentative point clusters are withheld.
  std::vector<CorridorDynamicObject> process(
      const std::vector<Eigen::Vector3d>& points, const ros::Time& stamp,
      const Eigen::Vector3d& sensor_position, const Eigen::Vector3d& odom_velocity);

  bool lastWallEstimateValid() const { return wall_valid_; }

 private:
  struct Voxel {
    int x = 0, y = 0, z = 0;
    bool operator<(const Voxel& other) const {
      if (x != other.x) return x < other.x;
      if (y != other.y) return y < other.y;
      return z < other.z;
    }
  };
  struct TrackSample { ros::Time stamp; Eigen::Vector3d position; };
  struct Track {
    uint32_t id = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d size = Eigen::Vector3d::Constant(0.20);
    std::deque<TrackSample> history;
    int hits = 0;
    int misses = 0;
    int positive_lateral = 0;
    int negative_lateral = 0;
    bool confirmed = false;
    ros::Time last_seen;
  };

  Voxel voxelOf(const Eigen::Vector3d& p) const;
  Eigen::Vector3d channelForward(const Eigen::Vector3d& odom_velocity);
  bool estimateWalls(const std::vector<Eigen::Vector3d>& points,
                     const Eigen::Vector3d& origin,
                     const Eigen::Vector3d& forward,
                     double* left, double* right) const;
  bool nearPreviousPoint(const Eigen::Vector3d& point) const;
  std::vector<CorridorDynamicObject> publishable(const ros::Time& stamp);

  CorridorDynamicRecheckerConfig config_;
  std::vector<Eigen::Vector3d> previous_points_;
  std::vector<Track> tracks_;
  Eigen::Vector3d forward_ = Eigen::Vector3d::UnitX();
  Eigen::Vector3d lateral_ = Eigen::Vector3d::UnitY();
  double left_wall_ = 0.75;
  double right_wall_ = 0.75;
  bool wall_valid_ = false;
  uint32_t next_id_ = 1;
};

}  // namespace diff_planner
