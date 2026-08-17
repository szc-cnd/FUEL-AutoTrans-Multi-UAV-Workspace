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

struct HarmonicEstimate
{
  double center{0.0};
  double amplitude{0.0};
  double half_period{0.0};
};

template <typename History>
bool estimateHarmonicMotion(
    const History &history,
    const Eigen::Vector3d &origin, const Eigen::Vector3d &lateral,
    const SwingObstacleGuard::Config &config, double half_width,
    HarmonicEstimate *estimate)
{
  if (estimate == nullptr ||
      history.size() < static_cast<std::size_t>(config.harmonic_min_samples))
    return false;

  std::vector<double> coordinates;
  coordinates.reserve(history.size());
  for (const auto &sample : history)
    coordinates.push_back((sample.position - origin).dot(lateral));

  const auto extrema = std::minmax_element(coordinates.begin(), coordinates.end());
  const double observed_minimum = *extrema.first;
  const double observed_maximum = *extrema.second;
  const double observed_span = observed_maximum - observed_minimum;
  if (observed_span < config.harmonic_min_motion_span)
    return false;

  std::vector<double> reversal_times;
  int previous_direction = 0;
  for (std::size_t index = 1; index < history.size(); ++index)
  {
    const double dt = history[index].time - history[index - 1].time;
    if (dt <= 1.0e-6)
      continue;
    const double speed = (coordinates[index] - coordinates[index - 1]) / dt;
    if (std::abs(speed) < config.harmonic_reversal_velocity_epsilon)
      continue;
    const int direction = speed > 0.0 ? 1 : -1;
    if (previous_direction != 0 && direction != previous_direction)
      reversal_times.push_back(history[index - 1].time);
    previous_direction = direction;
  }
  if (reversal_times.size() < 2)
    return false;

  std::vector<double> half_periods;
  for (std::size_t index = 1; index < reversal_times.size(); ++index)
  {
    const double half_period = reversal_times[index] - reversal_times[index - 1];
    if (half_period >= config.harmonic_min_half_period &&
        half_period <= config.harmonic_max_half_period)
      half_periods.push_back(half_period);
  }
  if (half_periods.empty())
    return false;
  std::sort(half_periods.begin(), half_periods.end());

  const double center = 0.5 * (observed_minimum + observed_maximum);
  const double corridor_limited_amplitude =
      std::min(center + half_width, half_width - center);
  const double amplitude =
      std::min(0.5 * observed_span, corridor_limited_amplitude);
  if (amplitude < 0.5 * config.harmonic_min_motion_span)
    return false;

  estimate->center = center;
  estimate->amplitude = amplitude;
  estimate->half_period = half_periods[half_periods.size() / 2];
  return true;
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
  sanitized.harmonic_min_samples = std::max(4, config.harmonic_min_samples);
  sanitized.harmonic_max_history_samples =
      std::max(sanitized.harmonic_min_samples, config.harmonic_max_history_samples);
  sanitized.harmonic_min_motion_span =
      positiveOr(config.harmonic_min_motion_span, 0.35);
  sanitized.harmonic_reversal_velocity_epsilon =
      nonnegativeOr(config.harmonic_reversal_velocity_epsilon, 0.04);
  sanitized.harmonic_min_half_period =
      positiveOr(config.harmonic_min_half_period, 0.3);
  sanitized.harmonic_max_half_period =
      positiveOr(config.harmonic_max_half_period, 4.0);
  if (sanitized.harmonic_max_half_period <= sanitized.harmonic_min_half_period)
    sanitized.harmonic_max_half_period =
        std::max(4.0, 2.0 * sanitized.harmonic_min_half_period);
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
    std::deque<Track::TimedPosition> position_history;
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
        position_history = existing->second.position_history;
      }
    }

    position_history.push_back({observation_time, observation.position});
    while (position_history.size() >
           static_cast<std::size_t>(config_.harmonic_max_history_samples))
      position_history.pop_front();

    Track track;
    track.observation = observation;
    track.motion_hint = motion_hint;
    track.position_history = std::move(position_history);
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

double SwingObstacleGuard::harmonicCoordinate(double coordinate,
                                              double velocity,
                                              double time,
                                              double center,
                                              double amplitude,
                                              double half_period)
{
  if (!std::isfinite(coordinate) || !std::isfinite(velocity) ||
      !std::isfinite(time) || !std::isfinite(center) ||
      !std::isfinite(amplitude) || !std::isfinite(half_period) ||
      amplitude <= 0.0 || half_period <= 0.0)
    return coordinate;

  const double angular_frequency = std::acos(-1.0) / half_period;
  const double sine = std::max(
      -1.0, std::min(1.0, (coordinate - center) / amplitude));
  const double cosine_magnitude =
      std::sqrt(std::max(0.0, 1.0 - sine * sine));
  const double cosine = velocity < 0.0 ? -cosine_magnitude : cosine_magnitude;
  const double phase = std::atan2(sine, cosine);
  return center + amplitude *
                      std::sin(phase + angular_frequency * std::max(0.0, time));
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

    HarmonicEstimate harmonic_estimate;
    const bool use_harmonic_prediction =
        config_.enable_harmonic_prediction &&
        estimateHarmonicMotion(track.position_history, origin, lateral, config_,
                               half_width, &harmonic_estimate);

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

      const double prediction_time = age + sample.time_from_now;
      const double predicted_lateral =
          use_harmonic_prediction
              ? harmonicCoordinate(lateral_coordinate, lateral_velocity,
                                   prediction_time, harmonic_estimate.center,
                                   harmonic_estimate.amplitude,
                                   harmonic_estimate.half_period)
              : reflectedCoordinate(lateral_coordinate, lateral_velocity,
                                    prediction_time, half_width);
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
