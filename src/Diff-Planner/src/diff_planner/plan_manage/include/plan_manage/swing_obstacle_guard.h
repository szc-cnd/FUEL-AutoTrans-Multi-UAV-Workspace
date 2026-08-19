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
  enum class ObservationState
  {
    EMPTY,
    MEASURED,
    PREDICTION_ONLY,
    STALE
  };
  struct Config
  {
    double corridor_width{1.5};
    double corridor_boundary_margin{0.02};
    double vehicle_radius{0.25};
    double vehicle_half_height{0.15};
    double horizontal_margin{0.10};
    double vertical_margin{0.10};
    double observation_retention{0.80};
    double measurement_freshness{0.25};
    double prediction_only_timeout{1.50};
    double logical_track_reset_timeout{3.0};
    // 新 ID 在短时遮挡后可接管旧摆球历史；不满足空间/尺寸/运动门限时不继承。
    double identity_handoff_max_gap{2.0};
    double identity_handoff_position_gate{0.65};
    double identity_handoff_size_ratio{0.35};
    double underpass_learning_time{3.0};
    double velocity_deadband{0.08};
    double minimum_swing_speed{0.25};
    bool enable_harmonic_prediction{false};
    // 通道实时模式只把当前观测位置作为未来各采样时刻的占据位置，
    // 不使用反射模型或简谐模型推断障碍物未来横向运动。
    bool realtime_observation_only{false};
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

  ObservationState observationState(double now) const;
  bool hasRecentMeasurement(double now, double max_age = 0.25) const;
  bool hasLogicalTrack(double now) const;
  bool logicalTrackSnapshot(double now,
                            SwingObstacleObservation *observation,
                            double *minimum_bottom_z = nullptr,
                            double *first_observation_time = nullptr) const;

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
    ObservationState state{ObservationState::MEASURED};
  };

  Config config_;
  // 通道场景只允许一个主要摆球。map 的 key 固定为0，观测中的 LDOP ID
  // 仅用于日志，不参与轨迹身份判断。
  std::unordered_map<uint32_t, Track> tracks_;

  static Config sanitizeConfig(const Config &config);
  double associationCost(const Track &track,
                         const SwingObstacleObservation &observation,
                         double observation_time) const;
};

} // namespace diff_planner

#endif
