#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <deque>
#include <vector>

namespace fast_planner {
namespace inflation_escape {

inline bool shouldStartFromOdometry(bool controlled_escape_position) {
  return controlled_escape_position;
}

inline std::vector<Eigen::Vector3d> buildDirectionalPath(
    const Eigen::Vector3d& current, const Eigen::Vector3d& direction,
    double max_distance, double sample_step) {
  Eigen::Vector3d horizontal = direction;
  horizontal.z() = 0.0;
  if (horizontal.head<2>().norm() < 1e-6 || max_distance <= 0.0) return {};
  horizontal.normalize();
  sample_step = std::max(0.01, sample_step);
  std::vector<Eigen::Vector3d> path{current};
  for (double distance = sample_step; distance < max_distance - 1e-9;
       distance += sample_step)
    path.push_back(current + horizontal * distance);
  path.push_back(current + horizontal * max_distance);
  return path;
}

inline double pathLength(const std::vector<Eigen::Vector3d>& path) {
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) length += (path[i] - path[i - 1]).norm();
  return length;
}

inline std::vector<Eigen::Vector3d> buildBackwardHistoryPath(
    const Eigen::Vector3d& current, const std::deque<Eigen::Vector3d>& history,
    double max_distance, double minimum_point_spacing) {
  std::vector<Eigen::Vector3d> path{current};
  max_distance = std::max(0.0, max_distance);
  minimum_point_spacing = std::max(1e-3, minimum_point_spacing);
  double accumulated = 0.0;
  for (auto it = history.rbegin(); it != history.rend() && accumulated < max_distance - 1e-9;
       ++it) {
    Eigen::Vector3d target = *it;
    const Eigen::Vector3d delta = target - path.back();
    const double segment = delta.norm();
    if (segment < minimum_point_spacing) continue;
    const double remaining = max_distance - accumulated;
    if (segment > remaining) target = path.back() + delta.normalized() * remaining;
    accumulated += (target - path.back()).norm();
    path.push_back(target);
  }
  if (path.size() < 2) path.clear();
  return path;
}

inline std::vector<Eigen::Vector3d> buildForwardTangentPath(
    const Eigen::Vector3d& current, const std::deque<Eigen::Vector3d>& history,
    double max_distance, double sample_step) {
  if (history.size() < 2 || max_distance <= 0.0) return {};
  sample_step = std::max(0.01, sample_step);
  // 只用最近约20cm实飞切线；远处旧直道不能在刚转弯后把脱困方向重新拉回去。
  auto tangent_start = history.end() - 1;
  double tangent_length = 0.0;
  for (auto it = history.end() - 1; it != history.begin() && tangent_length < 0.20;) {
    const auto previous = it - 1;
    tangent_length += (*it - *previous).head<2>().norm();
    tangent_start = previous;
    it = previous;
  }
  Eigen::Vector3d direction = history.back() - *tangent_start;
  direction.z() = 0.0;
  if (direction.head<2>().norm() < 1e-3) return {};
  direction.normalize();

  std::vector<Eigen::Vector3d> path{current};
  for (double distance = sample_step; distance < max_distance - 1e-9;
       distance += sample_step)
    path.push_back(current + direction * distance);
  path.push_back(current + direction * max_distance);
  return path;
}

}  // namespace inflation_escape
}  // namespace fast_planner
