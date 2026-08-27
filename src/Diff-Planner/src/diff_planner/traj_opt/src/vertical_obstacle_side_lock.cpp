#include <optimizer/vertical_obstacle_side_lock.h>

#include <optimizer/suspended_obstacle_underpass.h>

#include <algorithm>
#include <cmath>

namespace diff_planner
{

void VerticalObstacleSideLock::setConfig(const Config &config)
{
  config_ = config;
  if (!std::isfinite(config_.corridor_half_width) ||
      config_.corridor_half_width <= 0.0)
    config_.corridor_half_width = 0.75;
  if (!std::isfinite(config_.width_similarity_tolerance) ||
      config_.width_similarity_tolerance < 0.0)
    config_.width_similarity_tolerance = 0.15;
  if (!std::isfinite(config_.min_side_width) || config_.min_side_width <= 0.0)
    config_.min_side_width = 0.20;
  if (!std::isfinite(config_.min_vertical_gap) ||
      config_.min_vertical_gap <= 0.0)
    config_.min_vertical_gap = 0.35;
  if (!std::isfinite(config_.association_distance) ||
      config_.association_distance <= 0.0)
    config_.association_distance = 0.75;
  if (!std::isfinite(config_.longitudinal_margin) ||
      config_.longitudinal_margin < 0.0)
    config_.longitudinal_margin = 0.30;
  if (!std::isfinite(config_.search_timeout) || config_.search_timeout <= 0.0)
    config_.search_timeout = 0.20;
  if (!config_.enabled)
    lock_.active = false;
}

const VerticalObstacleSideLock::Config &
VerticalObstacleSideLock::config() const
{
  return config_;
}

bool VerticalObstacleSideLock::widthsAreComparable(
    const double left_width, const double right_width,
    const double min_width, const double tolerance)
{
  return std::isfinite(left_width) && std::isfinite(right_width) &&
         std::isfinite(min_width) && std::isfinite(tolerance) &&
         left_width >= min_width && right_width >= min_width &&
         std::abs(left_width - right_width) <= tolerance;
}

Eigen::Vector3d VerticalObstacleSideLock::chooseSideNormal(
    const std::vector<Eigen::Vector3d> &path,
    const Eigen::Vector3d &obstacle_center,
    const Eigen::Vector3d &lateral,
    const double left_width, const double right_width)
{
  double strongest_offset = 0.0;
  for (const Eigen::Vector3d &point : path)
  {
    if (!point.allFinite())
      continue;
    const double offset = (point - obstacle_center).dot(lateral);
    if (std::abs(offset) > std::abs(strongest_offset))
      strongest_offset = offset;
  }

  if (std::abs(strongest_offset) <= 1.0e-6)
    strongest_offset = left_width >= right_width ? 1.0 : -1.0;
  return strongest_offset >= 0.0 ? lateral : -lateral;
}

void VerticalObstacleSideLock::computeCenterBand(
    const double first_free_offset, const double last_free_offset,
    const double step_size, double &center_offset, double &band_half_width)
{
  center_offset = 0.5 * (first_free_offset + last_free_offset);
  const double free_width = last_free_offset - first_free_offset + step_size;
  band_half_width = std::max(0.5 * step_size, 0.25 * free_width);
}

bool VerticalObstacleSideLock::isGroundConnected(
    const GridMap::Ptr &grid_map, const double step_size,
    const Eigen::Vector3d &point) const
{
  const int sample_count =
      std::max(1, static_cast<int>(std::ceil(2.0 / step_size)));
  std::vector<int> downward_occupancy;
  downward_occupancy.reserve(sample_count);
  for (int index = 1; index <= sample_count; ++index)
  {
    Eigen::Vector3d below = point;
    below.z() -= index * step_size;
    downward_occupancy.push_back(grid_map->getInflateOccupancy(below));
  }
  return !SuspendedObstacleUnderpass::hasVerticalGap(
      downward_occupancy, step_size, config_.min_vertical_gap);
}

double VerticalObstacleSideLock::measureSideWidth(
    const GridMap::Ptr &grid_map, const double step_size,
    const Eigen::Vector3d &center, const Eigen::Vector3d &direction,
    double &center_offset, double &band_half_width) const
{
  bool reached_free_space = false;
  double free_width = 0.0;
  double first_free_offset = 0.0;
  double last_free_offset = 0.0;
  const int sample_count = std::max(
      1, static_cast<int>(std::ceil(config_.corridor_half_width / step_size)));
  for (int index = 1; index <= sample_count; ++index)
  {
    const Eigen::Vector3d point = center + index * step_size * direction;
    const int occupancy = grid_map->getInflateOccupancy(point);
    if (!reached_free_space)
    {
      if (occupancy == 0)
      {
        reached_free_space = true;
        first_free_offset = index * step_size;
        last_free_offset = first_free_offset;
        free_width = step_size;
      }
      continue;
    }
    if (occupancy != 0)
      break;
    last_free_offset = index * step_size;
    free_width += step_size;
  }
  computeCenterBand(first_free_offset, last_free_offset, step_size,
                    center_offset, band_half_width);
  return free_width;
}

bool VerticalObstacleSideLock::detectCandidate(
    const GridMap::Ptr &grid_map, const double step_size,
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    Candidate &candidate) const
{
  const Eigen::Vector3d horizontal(end.x() - start.x(), end.y() - start.y(), 0.0);
  const double horizontal_length = horizontal.norm();
  if (!grid_map || horizontal_length <= 1.0e-3)
    return false;

  candidate.axis = horizontal / horizontal_length;
  candidate.lateral =
      Eigen::Vector3d(-candidate.axis.y(), candidate.axis.x(), 0.0);

  const Eigen::Vector3d segment = end - start;
  const int sample_count = std::max(
      1, static_cast<int>(std::ceil(segment.norm() / step_size)));
  std::vector<Eigen::Vector3d> occupied_points;
  for (int index = 0; index <= sample_count; ++index)
  {
    const Eigen::Vector3d point =
        start + segment * (static_cast<double>(index) / sample_count);
    if (grid_map->getInflateOccupancy(point) != 0)
      occupied_points.push_back(point);
  }
  if (occupied_points.empty())
    return false;

  candidate.center.setZero();
  for (const Eigen::Vector3d &point : occupied_points)
    candidate.center += point;
  candidate.center /= occupied_points.size();
  if (!isGroundConnected(grid_map, step_size, candidate.center))
    return false;

  candidate.half_length = 0.0;
  for (const Eigen::Vector3d &point : occupied_points)
  {
    candidate.half_length = std::max(
        candidate.half_length,
        std::abs((point - candidate.center).dot(candidate.axis)));
  }
  candidate.left_width = measureSideWidth(
      grid_map, step_size, candidate.center, candidate.lateral,
      candidate.left_center_offset, candidate.left_band_half_width);
  candidate.right_width = measureSideWidth(
      grid_map, step_size, candidate.center, -candidate.lateral,
      candidate.right_center_offset, candidate.right_band_half_width);
  return widthsAreComparable(
      candidate.left_width, candidate.right_width,
      config_.min_side_width, config_.width_similarity_tolerance);
}

bool VerticalObstacleSideLock::trySearch(
    AStar &a_star, const GridMap::Ptr &grid_map, const double step_size,
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    const std::vector<Eigen::Vector3d> &unconstrained_path,
    std::vector<Eigen::Vector3d> &path)
{
  if (!config_.enabled || !std::isfinite(step_size) || step_size <= 0.0 ||
      !start.allFinite() || !end.allFinite() || unconstrained_path.empty())
    return false;

  Candidate candidate;
  if (!detectCandidate(grid_map, step_size, start, end, candidate))
    return false;

  const bool same_obstacle = lock_.active &&
      (candidate.center - lock_.center).head<2>().norm() <=
          config_.association_distance;
  if (!same_obstacle)
  {
    lock_.active = true;
    lock_.center = candidate.center;
    lock_.axis = candidate.axis;
    lock_.normal = chooseSideNormal(
        unconstrained_path, candidate.center, candidate.lateral,
        candidate.left_width, candidate.right_width);
    ROS_INFO("[接地障碍侧锁] 左右净宽 %.2fm / %.2fm，锁定%s侧中心带。",
             candidate.left_width, candidate.right_width,
             lock_.normal.dot(candidate.lateral) >= 0.0 ? "左" : "右");
  }
  else
  {
    lock_.center = candidate.center;
  }
  const bool selected_left = lock_.normal.dot(candidate.lateral) >= 0.0;
  lock_.passage_center_offset = selected_left
      ? candidate.left_center_offset : candidate.right_center_offset;
  lock_.passage_band_half_width = selected_left
      ? candidate.left_band_half_width : candidate.right_band_half_width;

  AStarSearchRegion region;
  region.enabled = true;
  region.limit_corridor = true;
  region.limit_side = true;
  region.limit_side_band = true;
  region.corridor_half_width = config_.corridor_half_width;
  region.side_half_length = candidate.half_length + config_.longitudinal_margin;
  region.time_limit = config_.search_timeout;
  region.report_timeout = true;
  region.line_start = start;
  region.line_end = end;
  region.side_origin = lock_.center;
  region.side_normal = lock_.normal;
  region.side_axis = lock_.axis;
  region.side_band_center = lock_.passage_center_offset;
  region.side_band_half_width = lock_.passage_band_half_width;

  path.clear();
  if (a_star.AstarSearch(step_size, start, end, &region) != ASTAR_RET::SUCCESS)
  {
    ROS_WARN_THROTTLE(1.0, "[接地障碍侧锁] 已选侧，但中心带内未找到路径。");
    return true;
  }
  path = a_star.getPath();
  return true;
}

} // namespace diff_planner
