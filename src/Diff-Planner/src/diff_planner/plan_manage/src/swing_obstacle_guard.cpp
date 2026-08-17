#include <plan_manage/swing_obstacle_guard.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace diff_planner
{
namespace
{

bool finiteObservation(const SwingObstacleObservation &observation)
{
  return observation.position.allFinite() && observation.velocity.allFinite() &&
         observation.size.allFinite() && (observation.size.array() >= 0.0).all();
}

double positiveOr(double value, double fallback)
{
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

double nonnegativeOr(double value, double fallback)
{
  return std::isfinite(value) && value >= 0.0 ? value : fallback;
}

} // namespace

SwingObstacleGuard::SwingObstacleGuard()
  : SwingObstacleGuard(Config())
{
}

SwingObstacleGuard::SwingObstacleGuard(const Config &config)
  : config_(sanitizeConfig(config))
{
}

void SwingObstacleGuard::setConfig(const Config &config)
{
  config_ = sanitizeConfig(config);
}

const SwingObstacleGuard::Config &SwingObstacleGuard::config() const
{
  return config_;
}

SwingObstacleGuard::Config SwingObstacleGuard::sanitizeConfig(const Config &config)
{
  Config sanitized = config;
  sanitized.corridor_width = positiveOr(config.corridor_width, 1.5);
  sanitized.corridor_boundary_margin =
      nonnegativeOr(config.corridor_boundary_margin, 0.02);
  sanitized.vehicle_radius = nonnegativeOr(config.vehicle_radius, 0.25);
  sanitized.vehicle_half_height = nonnegativeOr(config.vehicle_half_height, 0.15);
  sanitized.horizontal_margin = nonnegativeOr(config.horizontal_margin, 0.10);
  sanitized.vertical_margin = nonnegativeOr(config.vertical_margin, 0.10);
  sanitized.observation_retention = positiveOr(config.observation_retention, 0.80);
  sanitized.underpass_learning_time =
      nonnegativeOr(config.underpass_learning_time, 3.0);
  sanitized.velocity_deadband = nonnegativeOr(config.velocity_deadband, 0.08);
  sanitized.minimum_swing_speed = positiveOr(config.minimum_swing_speed, 0.25);
  return sanitized;
}

void SwingObstacleGuard::update(
    const std::vector<SwingObstacleObservation> &observations,
    double observation_time)
{
  if (!std::isfinite(observation_time))
    return;

  for (const SwingObstacleObservation &observation : observations)
  {
    if (!finiteObservation(observation))
      continue;

    auto existing = tracks_.find(observation.id);
    Eigen::Vector3d motion_hint = observation.velocity;
    const double object_bottom =
        observation.position.z() - 0.5 * observation.size.z();
    double first_observation_time = observation_time;
    double minimum_bottom_z = object_bottom;
    if (existing != tracks_.end())
    {
      const double dt = observation_time - existing->second.observation_time;
      const bool continuous_track =
          dt >= 0.0 && dt <= config_.observation_retention;
      if (dt > 1.0e-3 && continuous_track)
      {
        const Eigen::Vector3d finite_difference =
            (observation.position - existing->second.observation.position) / dt;
        if (finite_difference.head<2>().norm() > config_.velocity_deadband)
          motion_hint = finite_difference;
      }

      if (continuous_track &&
          motion_hint.head<2>().norm() <= config_.velocity_deadband)
        motion_hint = existing->second.motion_hint;

      if (continuous_track)
      {
        first_observation_time = existing->second.first_observation_time;
        minimum_bottom_z =
            std::min(existing->second.minimum_bottom_z, object_bottom);
      }
    }

    Track track;
    track.observation = observation;
    track.motion_hint = motion_hint;
    track.first_observation_time = first_observation_time;
    track.minimum_bottom_z = minimum_bottom_z;
    track.observation_time = observation_time;
    tracks_[observation.id] = track;
  }

  for (auto it = tracks_.begin(); it != tracks_.end();)
  {
    if (observation_time - it->second.observation_time >
        config_.observation_retention)
      it = tracks_.erase(it);
    else
      ++it;
  }
}

void SwingObstacleGuard::clear()
{
  tracks_.clear();
}

double SwingObstacleGuard::reflectedCoordinate(double coordinate,
                                               double velocity,
                                               double time,
                                               double half_width)
{
  if (!std::isfinite(coordinate) || !std::isfinite(velocity) ||
      !std::isfinite(time) || !std::isfinite(half_width) || half_width <= 0.0)
    return coordinate;

  const double bounded_coordinate =
      std::max(-half_width, std::min(half_width, coordinate));
  const double period = 4.0 * half_width;
  double phase = std::fmod(bounded_coordinate + velocity * std::max(0.0, time) +
                               half_width,
                           period);
  if (phase < 0.0)
    phase += period;
  return phase <= 2.0 * half_width
             ? -half_width + phase
             : 3.0 * half_width - phase;
}

bool SwingObstacleGuard::findCollision(
    const std::vector<SwingTrajectorySample> &trajectory, double query_time,
    SwingCollisionResult *result) const
{
  if (trajectory.size() < 2 || !std::isfinite(query_time))
    return false;

  Eigen::Vector3d forward = trajectory.back().position - trajectory.front().position;
  forward.z() = 0.0;
  if (!forward.allFinite() || forward.head<2>().norm() < 1.0e-3)
    return false;
  forward.normalize();
  const Eigen::Vector3d lateral(-forward.y(), forward.x(), 0.0);
  const Eigen::Vector3d origin = trajectory.front().position;

  double earliest_collision = std::numeric_limits<double>::infinity();
  SwingCollisionResult earliest_result;

  for (const auto &entry : tracks_)
  {
    const Track &track = entry.second;
    const double age = std::max(0.0, query_time - track.observation_time);
    if (age > config_.observation_retention || !finiteObservation(track.observation))
      continue;

    const double object_radius =
        0.5 * std::max(track.observation.size.x(), track.observation.size.y());
    const double half_width = std::max(
        0.01, 0.5 * config_.corridor_width - object_radius -
                  config_.corridor_boundary_margin);
    const Eigen::Vector3d relative = track.observation.position - origin;
    const double longitudinal_coordinate = relative.dot(forward);
    const double lateral_coordinate = relative.dot(lateral);

    double lateral_velocity = track.observation.velocity.dot(lateral);
    if (std::abs(lateral_velocity) <= config_.velocity_deadband)
      lateral_velocity = track.motion_hint.dot(lateral);

    double direction = lateral_velocity < 0.0 ? -1.0 : 1.0;
    if (std::abs(lateral_velocity) <= config_.velocity_deadband)
      direction = lateral_coordinate > 0.0 ? -1.0 : 1.0;
    const double speed = std::max(std::abs(lateral_velocity),
                                  config_.minimum_swing_speed);
    lateral_velocity = direction * speed;
    if (lateral_coordinate <= -half_width + config_.corridor_boundary_margin &&
        lateral_velocity < 0.0)
      lateral_velocity = speed;
    else if (lateral_coordinate >= half_width - config_.corridor_boundary_margin &&
             lateral_velocity > 0.0)
      lateral_velocity = -speed;

    const double horizontal_clearance = object_radius + config_.vehicle_radius +
                                        config_.horizontal_margin;
    const bool underpass_height_learned =
        query_time - track.first_observation_time >=
        config_.underpass_learning_time;

    for (const SwingTrajectorySample &sample : trajectory)
    {
      if (!std::isfinite(sample.time_from_now) || sample.time_from_now < 0.0 ||
          !sample.position.allFinite())
        continue;

      // Do not infer underpass clearance from a side endpoint, where a pendulum
      // is highest. Learn the lowest observed bottom before enabling underpass.
      const double vehicle_top = sample.position.z() + config_.vehicle_half_height +
                                 config_.vertical_margin;
      if (underpass_height_learned && vehicle_top <= track.minimum_bottom_z)
        continue;

      const double predicted_lateral = reflectedCoordinate(
          lateral_coordinate, lateral_velocity, age + sample.time_from_now,
          half_width);
      const Eigen::Vector3d predicted_position =
          origin + forward * longitudinal_coordinate + lateral * predicted_lateral +
          Eigen::Vector3d(0.0, 0.0, track.observation.position.z() - origin.z());

      if ((predicted_position - sample.position).head<2>().norm() <=
              horizontal_clearance &&
          sample.time_from_now < earliest_collision)
      {
        earliest_collision = sample.time_from_now;
        earliest_result.obstacle_id = track.observation.id;
        earliest_result.time_from_now = sample.time_from_now;
        earliest_result.obstacle_position = predicted_position;
        earliest_result.vehicle_position = sample.position;
      }
    }
  }

  if (!std::isfinite(earliest_collision))
    return false;
  if (result != nullptr)
    *result = earliest_result;
  return true;
}

} // namespace diff_planner
