#ifndef DIFF_PLANNER_SWING_OBSTACLE_GUARD_H_
#define DIFF_PLANNER_SWING_OBSTACLE_GUARD_H_

#include <Eigen/Eigen>

#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace diff_planner
{

struct SwingObstacleObservation
{
  uint32_t id{0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
};

struct SwingTrajectorySample
{
  double time_from_now{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

struct SwingCollisionResult
{
  uint32_t obstacle_id{0};
  double time_from_now{0.0};
  Eigen::Vector3d obstacle_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vehicle_position{Eigen::Vector3d::Zero()};
};

class SwingObstacleGuard
{
public:
  struct Config
  {
    double corridor_width{1.5};
    double corridor_boundary_margin{0.02};
    double vehicle_radius{0.25};
    double vehicle_half_height{0.15};
    double horizontal_margin{0.10};
    double vertical_margin{0.10};
    double observation_retention{0.80};
    double underpass_learning_time{3.0};
    double velocity_deadband{0.08};
    double minimum_swing_speed{0.25};
    bool enable_harmonic_prediction{false};
    int harmonic_min_samples{12};
    int harmonic_max_history_samples{80};
    double harmonic_min_motion_span{0.35};
    double harmonic_reversal_velocity_epsilon{0.04};
    double harmonic_min_half_period{0.3};
    double harmonic_max_half_period{4.0};
  };

  SwingObstacleGuard();
  explicit SwingObstacleGuard(const Config &config);

  void setConfig(const Config &config);
  const Config &config() const;
  void update(const std::vector<SwingObstacleObservation> &observations,
              double observation_time);
  void clear();

  bool findCollision(const std::vector<SwingTrajectorySample> &trajectory,
                     double query_time,
                     SwingCollisionResult *result = nullptr) const;

  static double reflectedCoordinate(double coordinate, double velocity,
                                    double time, double half_width);
  static double harmonicCoordinate(double coordinate, double velocity,
                                   double time, double center,
                                   double amplitude, double half_period);

private:
  struct Track
  {
    struct TimedPosition
    {
      double time{0.0};
      Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    };

    SwingObstacleObservation observation;
    Eigen::Vector3d motion_hint{Eigen::Vector3d::Zero()};
    std::deque<TimedPosition> position_history;
    double first_observation_time{0.0};
    double minimum_bottom_z{0.0};
    double observation_time{0.0};
  };

  Config config_;
  std::unordered_map<uint32_t, Track> tracks_;

  static Config sanitizeConfig(const Config &config);
};

} // namespace diff_planner

#endif
