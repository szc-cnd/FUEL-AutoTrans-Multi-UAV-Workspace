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
  enum class SearchResult
  {
    NOT_APPLICABLE,
    SUCCESS,
    SEARCH_FAILED
  };

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

  SearchResult trySearch(AStar &a_star, const GridMap::Ptr &grid_map,
                         double step_size, const Eigen::Vector3d &start,
                         const Eigen::Vector3d &end,
                         std::vector<Eigen::Vector3d> &path) const;

  static bool hasRequiredDescent(const std::vector<Eigen::Vector3d> &path,
                                 const Eigen::Vector3d &start,
                                 const Eigen::Vector3d &end,
                                 double min_descent);
  static bool hasVerticalGap(const std::vector<int> &downward_occupancy,
                             double step_size, double min_vertical_gap);
  static bool choosePassageHeight(double top_z, double bottom_z,
                                  double start_z, double end_z,
                                  double min_descent, double &target_z);

private:
  struct Passage
  {
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
    double half_length{0.0};
    double target_z{0.0};
  };

  bool findSuspendedPassage(const GridMap::Ptr &grid_map, double step_size,
                            const Eigen::Vector3d &start,
                            const Eigen::Vector3d &end,
                            Passage &passage) const;

  Config config_;
};

} // namespace diff_planner

#endif
