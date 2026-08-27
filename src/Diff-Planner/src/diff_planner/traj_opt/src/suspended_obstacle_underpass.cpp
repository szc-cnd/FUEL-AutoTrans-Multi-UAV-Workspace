#include <optimizer/suspended_obstacle_underpass.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace diff_planner
{

void SuspendedObstacleUnderpass::setConfig(const Config &config)
{
  config_ = config;
  if (!std::isfinite(config_.corridor_half_width) ||
      config_.corridor_half_width <= 0.0)
    config_.corridor_half_width = 0.12;
  if (!std::isfinite(config_.min_descent) || config_.min_descent <= 0.0)
    config_.min_descent = 0.10;
  if (!std::isfinite(config_.min_vertical_gap) ||
      config_.min_vertical_gap <= 0.0)
    config_.min_vertical_gap = 0.35;
  if (!std::isfinite(config_.search_timeout) || config_.search_timeout <= 0.0)
    config_.search_timeout = 0.05;
}

const SuspendedObstacleUnderpass::Config &
SuspendedObstacleUnderpass::config() const
{
  return config_;
}

bool SuspendedObstacleUnderpass::hasRequiredDescent(
    const std::vector<Eigen::Vector3d> &path,
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    const double min_descent)
{
  if (path.empty() || !std::isfinite(min_descent) || min_descent <= 0.0)
    return false;

  double minimum_z = std::numeric_limits<double>::infinity();
  for (const Eigen::Vector3d &point : path)
  {
    if (!point.allFinite())
      return false;
    minimum_z = std::min(minimum_z, point.z());
  }
  return minimum_z <= std::min(start.z(), end.z()) - min_descent + 1.0e-6;
}

bool SuspendedObstacleUnderpass::hasVerticalGap(
    const std::vector<int> &downward_occupancy, const double step_size,
    const double min_vertical_gap)
{
  if (downward_occupancy.empty() || !std::isfinite(step_size) ||
      step_size <= 0.0 || !std::isfinite(min_vertical_gap) ||
      min_vertical_gap <= 0.0)
    return false;

  double free_height = 0.0;
  bool below_obstacle = false;
  for (const int occupancy : downward_occupancy)
  {
    if (occupancy < 0)
      return false;
    if (!below_obstacle)
    {
      if (occupancy == 0)
      {
        below_obstacle = true;
        free_height = step_size;
      }
      continue;
    }
    if (occupancy != 0)
      break;
    free_height += step_size;
  }
  return free_height + 1.0e-6 >= min_vertical_gap;
}

bool SuspendedObstacleUnderpass::isSuspendedObstacle(
    const GridMap::Ptr &grid_map, const double step_size,
    const Eigen::Vector3d &start, const Eigen::Vector3d &end) const
{
  if (!grid_map)
    return false;

  const Eigen::Vector3d segment = end - start;
  const double length = segment.norm();
  const int sample_count =
      std::max(1, static_cast<int>(std::ceil(length / step_size)));
  const int downward_samples =
      std::max(1, static_cast<int>(std::ceil(2.0 / step_size)));
  bool found_occupied_column = false;

  for (int sample = 0; sample <= sample_count; ++sample)
  {
    const Eigen::Vector3d point =
        start + segment * (static_cast<double>(sample) / sample_count);
    if (grid_map->getInflateOccupancy(point) == 0)
      continue;

    found_occupied_column = true;
    std::vector<int> downward_occupancy;
    downward_occupancy.reserve(downward_samples);
    for (int index = 1; index <= downward_samples; ++index)
    {
      Eigen::Vector3d below = point;
      below.z() -= index * step_size;
      downward_occupancy.push_back(grid_map->getInflateOccupancy(below));
    }
    if (!hasVerticalGap(downward_occupancy, step_size,
                        config_.min_vertical_gap))
      return false;
  }
  return found_occupied_column;
}

bool SuspendedObstacleUnderpass::trySearch(
    AStar &a_star, const GridMap::Ptr &grid_map, const double step_size,
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    std::vector<Eigen::Vector3d> &path) const
{
  path.clear();
  if (!config_.enabled || !start.allFinite() || !end.allFinite() ||
      !std::isfinite(step_size) || step_size <= 0.0 ||
      (end - start).head<2>().norm() <= 1.0e-3)
    return false;
  if (!isSuspendedObstacle(grid_map, step_size, start, end))
    return false;

  AStarSearchRegion region;
  region.enabled = true;
  region.max_z = std::max(start.z(), end.z());
  region.corridor_half_width = config_.corridor_half_width;
  region.time_limit = config_.search_timeout;
  region.report_timeout = false;
  region.line_start = start;
  region.line_end = end;

  if (a_star.AstarSearch(step_size, start, end, &region) != ASTAR_RET::SUCCESS)
    return false;

  std::vector<Eigen::Vector3d> candidate = a_star.getPath();
  if (!hasRequiredDescent(candidate, start, end, config_.min_descent))
    return false;

  path = std::move(candidate);
  return true;
}

} // namespace diff_planner
