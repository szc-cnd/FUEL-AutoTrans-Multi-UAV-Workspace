#ifndef DIFF_PLANNER_VERTICAL_OBSTACLE_SIDE_LOCK_H_
#define DIFF_PLANNER_VERTICAL_OBSTACLE_SIDE_LOCK_H_

#include <Eigen/Eigen>

#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>

#include <vector>

namespace diff_planner
{

class VerticalObstacleSideLock
{
public:
  struct Config
  {
    bool enabled{false};
    double corridor_half_width{0.75};
    double width_similarity_tolerance{0.15};
    double min_side_width{0.20};
    double min_vertical_gap{0.35};
    double association_distance{0.75};
    double longitudinal_margin{0.30};
    double search_timeout{0.20};
  };

  void setConfig(const Config &config);
  const Config &config() const;

  bool trySearch(AStar &a_star, const GridMap::Ptr &grid_map, double step_size,
                 const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                 const std::vector<Eigen::Vector3d> &unconstrained_path,
                 std::vector<Eigen::Vector3d> &path);

  static bool widthsAreComparable(double left_width, double right_width,
                                  double min_width, double tolerance);
  static Eigen::Vector3d chooseSideNormal(
      const std::vector<Eigen::Vector3d> &path,
      const Eigen::Vector3d &obstacle_center,
      const Eigen::Vector3d &lateral,
      double left_width, double right_width);
  static double computePassageCenter(double obstacle_boundary_offset,
                                     double wall_boundary_offset);
  static bool isSameObstacle(const Eigen::Vector3d &locked_center,
                             const Eigen::Vector3d &locked_axis,
                             const Eigen::Vector3d &locked_normal,
                             double locked_half_length,
                             const Eigen::Vector3d &candidate_center,
                             double candidate_half_length,
                             double association_distance,
                             double longitudinal_margin);

private:
  struct Candidate
  {
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
    Eigen::Vector3d lateral{Eigen::Vector3d::Zero()};
    double half_length{0.0};
    double left_width{0.0};
    double right_width{0.0};
    double left_center_offset{0.0};
    double right_center_offset{0.0};
  };

  struct Lock
  {
    bool active{false};
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
    double half_length{0.0};
    double passage_center_offset{0.0};
  };

  bool detectCandidate(const GridMap::Ptr &grid_map, double step_size,
                       const Eigen::Vector3d &start,
                       const Eigen::Vector3d &end,
                       Candidate &candidate) const;
  bool isGroundConnected(const GridMap::Ptr &grid_map, double step_size,
                         const Eigen::Vector3d &point) const;
  double measureSideWidth(const GridMap::Ptr &grid_map, double step_size,
                          const Eigen::Vector3d &center,
                          const Eigen::Vector3d &direction,
                          double &center_offset) const;

  Config config_;
  Lock lock_;
};

} // namespace diff_planner

#endif
