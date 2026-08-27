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
        if (free_height + 1.0e-6 >= min_vertical_gap)
          return true;
      }
      continue;
    }
    if (occupancy != 0)
      break;
    free_height += step_size;
    if (free_height + 1.0e-6 >= min_vertical_gap)
      return true;
  }
  return false;
}

bool SuspendedObstacleUnderpass::choosePassageHeight(
    const double top_z, const double bottom_z, const double start_z,
    const double end_z, const double min_descent, double &target_z)
{
  if (!std::isfinite(top_z) || !std::isfinite(bottom_z) ||
      !std::isfinite(start_z) || !std::isfinite(end_z) ||
      !std::isfinite(min_descent) || top_z < bottom_z || min_descent <= 0.0)
    return false;
  target_z = std::min(0.5 * (top_z + bottom_z),
                      std::min(start_z, end_z) - min_descent);
  return target_z + 1.0e-6 >= bottom_z;
}

bool SuspendedObstacleUnderpass::findSuspendedPassage(
    const GridMap::Ptr &grid_map, const double step_size,
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    Passage &passage) const
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
  double common_top_z = std::numeric_limits<double>::infinity();
  double common_bottom_z = -std::numeric_limits<double>::infinity();
  std::vector<Eigen::Vector3d> occupied_points;

  for (int sample = 0; sample <= sample_count; ++sample)
  {
    const Eigen::Vector3d point =
        start + segment * (static_cast<double>(sample) / sample_count);
    if (grid_map->getInflateOccupancy(point) == 0)
      continue;

    found_occupied_column = true;
    occupied_points.push_back(point);
    std::vector<int> downward_occupancy;
    downward_occupancy.reserve(downward_samples);
    for (int index = 1; index <= downward_samples; ++index)
    {
      Eigen::Vector3d below = point;
      below.z() -= index * step_size;
      downward_occupancy.push_back(grid_map->getInflateOccupancy(below));
    }
    int first_free = -1;
    int last_free = -1;
    for (int index = 0; index < downward_samples; ++index)
    {
      if (downward_occupancy[index] == 0)
      {
        if (first_free < 0)
          first_free = index;
        last_free = index;
      }
      else if (first_free >= 0)
      {
        break;
      }
    }
    if (first_free < 0 ||
        (last_free - first_free + 1) * step_size + 1.0e-6 <
            config_.min_vertical_gap)
      return false;
    common_top_z = std::min(
        common_top_z, point.z() - (first_free + 1) * step_size);
    common_bottom_z = std::max(
        common_bottom_z, point.z() - (last_free + 1) * step_size);
  }
  if (!found_occupied_column ||
      common_top_z - common_bottom_z + step_size + 1.0e-6 <
          config_.min_vertical_gap)
    return false;

  passage.axis = Eigen::Vector3d(end.x() - start.x(), end.y() - start.y(), 0.0);
  passage.axis.normalize();
  passage.center.setZero();
  for (const Eigen::Vector3d &point : occupied_points)
    passage.center += point;
  passage.center /= occupied_points.size();
  passage.half_length = 0.0;
  for (const Eigen::Vector3d &point : occupied_points)
    passage.half_length = std::max(
        passage.half_length,
        std::abs((point - passage.center).dot(passage.axis)));
  return choosePassageHeight(common_top_z, common_bottom_z,
                             start.z(), end.z(), config_.min_descent,
                             passage.target_z);
}

SuspendedObstacleUnderpass::SearchResult SuspendedObstacleUnderpass::trySearch(
    AStar &a_star, const GridMap::Ptr &grid_map, const double step_size,
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    std::vector<Eigen::Vector3d> &path) const
{
  path.clear();
  if (!config_.enabled || !start.allFinite() || !end.allFinite() ||
      !std::isfinite(step_size) || step_size <= 0.0 ||
      (end - start).head<2>().norm() <= 1.0e-3)
    return SearchResult::NOT_APPLICABLE;
  Passage passage;
  if (!findSuspendedPassage(grid_map, step_size, start, end, passage))
    return SearchResult::NOT_APPLICABLE;

  AStarSearchRegion region;
  region.enabled = true;
  region.limit_local_max_z = true;
  region.limit_corridor = true;
  region.max_z = passage.target_z;
  region.corridor_half_width = config_.corridor_half_width;
  region.time_limit = config_.search_timeout;
  region.report_timeout = false;
  region.line_start = start;
  region.line_end = end;
  region.height_origin = passage.center;
  region.height_axis = passage.axis;
  region.height_half_length = passage.half_length + step_size;

  if (a_star.AstarSearch(step_size, start, end, &region) != ASTAR_RET::SUCCESS)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[悬空障碍下穿] 已找到下方空间，但指定高度内搜索失败。");
    return SearchResult::SEARCH_FAILED;
  }

  std::vector<Eigen::Vector3d> candidate = a_star.getPath();
  if (!hasRequiredDescent(candidate, start, end, config_.min_descent))
    return SearchResult::SEARCH_FAILED;

  path = std::move(candidate);
  return SearchResult::SUCCESS;
}

} // namespace diff_planner
