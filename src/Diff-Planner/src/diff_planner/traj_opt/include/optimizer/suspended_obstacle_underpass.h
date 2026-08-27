#ifndef DIFF_PLANNER_SUSPENDED_OBSTACLE_UNDERPASS_H_
#define DIFF_PLANNER_SUSPENDED_OBSTACLE_UNDERPASS_H_

#include <Eigen/Eigen>

#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>

#include <vector>

namespace diff_planner
{

class SuspendedObstacleUnderpass
{
public:
  struct Config
  {
    bool enabled{false};
    double corridor_half_width{0.12};
    double min_descent{0.10};
    double min_vertical_gap{0.35};
    double search_timeout{0.05};
  };

  void setConfig(const Config &config);
  const Config &config() const;

  bool trySearch(AStar &a_star, const GridMap::Ptr &grid_map, double step_size,
                 const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                 std::vector<Eigen::Vector3d> &path) const;

  static bool hasRequiredDescent(const std::vector<Eigen::Vector3d> &path,
                                 const Eigen::Vector3d &start,
                                 const Eigen::Vector3d &end,
                                 double min_descent);
  static bool hasVerticalGap(const std::vector<int> &downward_occupancy,
                             double step_size, double min_vertical_gap);

private:
  bool isSuspendedObstacle(const GridMap::Ptr &grid_map, double step_size,
                           const Eigen::Vector3d &start,
                           const Eigen::Vector3d &end) const;

  Config config_;
};

} // namespace diff_planner

#endif
