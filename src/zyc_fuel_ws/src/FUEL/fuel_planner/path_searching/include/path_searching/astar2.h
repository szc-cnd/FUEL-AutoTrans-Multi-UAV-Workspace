#ifndef _ASTAR2_H
#define _ASTAR2_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include "plan_env/edt_environment.h"
#include <boost/functional/hash.hpp>
#include <queue>
#include <path_searching/matrix_hash.h>
#include <path_searching/directional_progress_policy.h>
namespace fast_planner {
class Node {
public:
  Eigen::Vector3i index;
  Eigen::Vector3d position;
  double g_score, f_score;
  Node* parent;

  /* -------------------- */
  Node() {
    parent = NULL;
  }
  ~Node(){};
};
typedef Node* NodePtr;

class NodeComparator0 {
public:
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

class Astar {
public:
  Astar();
  ~Astar();
  enum { REACH_END = 1, NO_PATH = 2 };

  void init(ros::NodeHandle& nh, const EDTEnvironment::Ptr& env);
  void reset();
  int search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt);
  void setResolution(const double& res);
  void setProgressConstraint(const Eigen::Vector3d& forward_direction,
                             double maximum_regression = 0.0);
  void clearProgressConstraint();
  static double pathLength(const vector<Eigen::Vector3d>& path);

  std::vector<Eigen::Vector3d> getPath();
  std::vector<Eigen::Vector3d> getVisited();
  double getEarlyTerminateCost();

  double lambda_heu_;
  double max_search_time_;

private:
  void backtrack(const NodePtr& end_node, const Eigen::Vector3d& end);
  void posToIndex(const Eigen::Vector3d& pt, Eigen::Vector3i& idx);
  double getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
  double getManhHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
  double getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);

  // main data structure
  vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
  std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> open_set_map_;
  std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> close_set_map_;
  std::vector<Eigen::Vector3d> path_nodes_;
  double early_terminate_cost_;

  EDTEnvironment::Ptr edt_env_;

  // parameter
  double margin_;
  // 2026-07-28: 任务A*高度带，防止三维搜索从未知/稀疏地面下方绕障。
  double min_search_height_, max_search_height_;
  // Prefer wide horizontal corridors instead of the geometrically shortest wall-hugging route.
  double vertical_weight_, preferred_clearance_, clearance_weight_;
  int allocate_num_;
  double tie_breaker_;
  double resolution_, inv_resolution_;
  Eigen::Vector3d map_size_3d_, origin_;
  bool progress_constraint_enabled_{false};
  Eigen::Vector3d progress_direction_{1.0, 0.0, 0.0};
  double maximum_progress_regression_{0.0};
};

}  // namespace fast_planner

#endif
