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

bool SuspendedObstacleUnderpass::trySearch(
    AStar &a_star, const double step_size,
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    std::vector<Eigen::Vector3d> &path) const
{
  path.clear();
  if (!config_.enabled || !start.allFinite() || !end.allFinite() ||
      !std::isfinite(step_size) || step_size <= 0.0 ||
      (end - start).head<2>().norm() <= 1.0e-3)
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
