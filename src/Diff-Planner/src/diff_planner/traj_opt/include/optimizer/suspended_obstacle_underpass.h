#ifndef DIFF_PLANNER_SUSPENDED_OBSTACLE_UNDERPASS_H_
#define DIFF_PLANNER_SUSPENDED_OBSTACLE_UNDERPASS_H_

#include <Eigen/Eigen>

#include <path_searching/dyn_a_star.h>

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
    double search_timeout{0.05};
  };

  void setConfig(const Config &config);
  const Config &config() const;

  bool trySearch(AStar &a_star, double step_size,
                 const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                 std::vector<Eigen::Vector3d> &path) const;

  static bool hasRequiredDescent(const std::vector<Eigen::Vector3d> &path,
                                 const Eigen::Vector3d &start,
                                 const Eigen::Vector3d &end,
                                 double min_descent);

private:
  Config config_;
};

} // namespace diff_planner

#endif
