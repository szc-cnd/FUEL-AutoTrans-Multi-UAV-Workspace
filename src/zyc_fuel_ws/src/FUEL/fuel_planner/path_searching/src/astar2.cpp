#include <path_searching/astar2.h>
#include <sstream>
#include <plan_env/sdf_map.h>

using namespace std;
using namespace Eigen;

namespace fast_planner {
Astar::Astar() {
}

Astar::~Astar() {
  for (int i = 0; i < allocate_num_; i++)
    delete path_node_pool_[i];
}

void Astar::init(ros::NodeHandle& nh, const EDTEnvironment::Ptr& env) {
  nh.param("astar/resolution_astar", resolution_, -1.0);
  nh.param("astar/lambda_heu", lambda_heu_, -1.0);
  nh.param("astar/max_search_time", max_search_time_, -1.0);
  nh.param("astar/allocate_num", allocate_num_, -1);
  // 2026-07-28: 默认不改变通用FUEL；比赛launch显式设置0.45~0.78m飞行高度带。
  nh.param("astar/min_search_height", min_search_height_, -1e6);
  nh.param("astar/max_search_height", max_search_height_, 1e6);
  nh.param("astar/vertical_weight", vertical_weight_, 1.0);
  nh.param("astar/preferred_clearance", preferred_clearance_, 0.0);
  nh.param("astar/clearance_weight", clearance_weight_, 0.0);

  tie_breaker_ = 1.0 + 1.0 / 1000;

  this->edt_env_ = env;

  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  edt_env_->sdf_map_->getRegion(origin_, map_size_3d_);
  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new Node;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
  early_terminate_cost_ = 0.0;
}

void Astar::setResolution(const double& res) {
  resolution_ = res;
  this->inv_resolution_ = 1.0 / resolution_;
}

void Astar::setProgressConstraint(const Eigen::Vector3d& forward_direction,
                                  double maximum_regression) {
  if (forward_direction.head<2>().norm() < 1e-6) {
    clearProgressConstraint();
    return;
  }
  progress_constraint_enabled_ = true;
  progress_direction_.setZero();
  progress_direction_.head<2>() = forward_direction.head<2>().normalized();
  maximum_progress_regression_ = std::max(0.0, maximum_regression);
}

void Astar::clearProgressConstraint() {
  progress_constraint_enabled_ = false;
  maximum_progress_regression_ = 0.0;
}

int Astar::search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt) {
  if (progress_constraint_enabled_ &&
      !directional_progress::isAllowed(start_pt, end_pt, progress_direction_,
                                       maximum_progress_regression_)) {
    ROS_WARN_THROTTLE(0.5,
                      "[Astar] reject goal behind active corridor direction before search: "
                      "start=(%.2f %.2f) goal=(%.2f %.2f) dir=(%.2f %.2f).",
                      start_pt.x(), start_pt.y(), end_pt.x(), end_pt.y(),
                      progress_direction_.x(), progress_direction_.y());
    return NO_PATH;
  }
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  posToIndex(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  open_set_map_.insert(make_pair(cur_node->index, cur_node));
  use_node_num_ += 1;

  const auto t1 = ros::Time::now();
  // 2026-07-16: 保留搜索入口的栅格状态，超时/无连通域时给出可区分诊断，不能再全部只报“No path”。
  const int start_inflate = edt_env_->sdf_map_->getInflateOccupancy(start_pt);
  const int start_occ = edt_env_->sdf_map_->getOccupancy(start_pt);
  const int goal_inflate = edt_env_->sdf_map_->getInflateOccupancy(end_pt);
  const int goal_occ = edt_env_->sdf_map_->getOccupancy(end_pt);
  // 2026-07-27: 漂移或地图更新可能让原始自由起点落入膨胀层；只允许沿EDT净空不下降方向脱困。
  const bool escape_inflated_start =
      start_occ == SDFMap::FREE && start_inflate == 1;
  if (escape_inflated_start) {
    ROS_WARN_THROTTLE(1.0,
                      "[Astar] start in inflated layer; enable monotonic-clearance escape.");
  }

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;
    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    // Early termination if time up
    if ((ros::Time::now() - t1).toSec() > max_search_time_) {
      early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
      // 2026-07-16: 旧配置只有1ms，复杂拐弯会把“尚未搜完”误判成“没有路”，并导致前机永久悬停。
      ROS_WARN_THROTTLE(1.0,
                        "[Astar] TIMEOUT %.4fs nodes=%d iter=%d start=(%.2f %.2f %.2f) "
                        "start_occ=%d/%d goal=(%.2f %.2f %.2f) goal_occ=%d/%d.",
                        (ros::Time::now() - t1).toSec(), use_node_num_, iter_num_,
                        start_pt.x(), start_pt.y(), start_pt.z(), start_occ, start_inflate,
                        end_pt.x(), end_pt.y(), end_pt.z(), goal_occ, goal_inflate);
      return NO_PATH;
    }

    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_map_.insert(make_pair(cur_node->index, 1));
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;

    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
          step << dx, dy, dz;
          if (step.norm() < 1e-3) continue;
          nbr_pos = cur_pos + step;
          // 多机通道的禁回头约束必须在搜索阶段生效。纯横移(dot=0)和斜前绕障
          // 正常展开，旧通道方向上的负进度节点不进入open set。
          if (progress_constraint_enabled_ &&
              !directional_progress::isAllowed(start_pt, nbr_pos, progress_direction_,
                                               maximum_progress_regression_))
            continue;
          // 2026-07-28: A*展开阶段直接约束完整路径高度；若起点尚低于下限，只允许不下降并逐步爬升。
          if (start_pt.z() < min_search_height_ && cur_pos.z() < min_search_height_) {
            if (nbr_pos.z() + 1e-3 < cur_pos.z()) continue;
          } else if (nbr_pos.z() < min_search_height_ - 1e-3) {
            continue;
          }
          // 2026-07-28: 起点偶发高于上限时对称地只允许下降，正常任务路径严禁越过上限。
          if (start_pt.z() > max_search_height_ && cur_pos.z() > max_search_height_) {
            if (nbr_pos.z() - 1e-3 > cur_pos.z()) continue;
          } else if (nbr_pos.z() > max_search_height_ + 1e-3) {
            continue;
          }
          // Check safety
          if (!edt_env_->sdf_map_->isInBox(nbr_pos)) continue;
          const int cur_inflate = edt_env_->sdf_map_->getInflateOccupancy(cur_pos);
          const int nbr_inflate = edt_env_->sdf_map_->getInflateOccupancy(nbr_pos);
          if (edt_env_->sdf_map_->getOccupancy(nbr_pos) == SDFMap::UNKNOWN) continue;
          if (nbr_inflate == 1) {
            // 2026-07-27: 只有仍处在初始膨胀连通域时允许走膨胀格；一旦脱离便严禁重新进入。
            if (!escape_inflated_start || cur_inflate == 0) continue;
            const double cur_clearance = edt_env_->sdf_map_->getDistance(cur_pos);
            const double nbr_clearance = edt_env_->sdf_map_->getDistance(nbr_pos);
            if (nbr_clearance + 0.02 < cur_clearance) continue;
          }

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (progress_constraint_enabled_ &&
                !directional_progress::isAllowed(start_pt, ckpt, progress_direction_,
                                                 maximum_progress_regression_)) {
              safe = false;
              break;
            }
            // 2026-07-28: 对角边内部采样同样遵守高度带，不能只检查端点后从地面穿过。
            if ((start_pt.z() >= min_search_height_ && ckpt.z() < min_search_height_ - 1e-3) ||
                (start_pt.z() <= max_search_height_ && ckpt.z() > max_search_height_ + 1e-3)) {
              safe = false;
              break;
            }
            const int checkpoint_inflate = edt_env_->sdf_map_->getInflateOccupancy(ckpt);
            if (edt_env_->sdf_map_->getOccupancy(ckpt) == SDFMap::UNKNOWN ||
                (checkpoint_inflate == 1 &&
                 (!escape_inflated_start || cur_inflate == 0 ||
                  edt_env_->sdf_map_->getDistance(ckpt) + 0.02 <
                      edt_env_->sdf_map_->getDistance(cur_pos)))) {
              safe = false;
              break;
            }
          }
          if (!safe) continue;

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (close_set_map_.find(nbr_idx) != close_set_map_.end()) continue;

          NodePtr neighbor;
          // Horizontal detours toward open space are safer than short vertical or wall-hugging
          // shortcuts. This is a soft cost: the inflated occupancy remains the hard boundary.
          const double weighted_step =
              sqrt(dx * dx + dy * dy + vertical_weight_ * vertical_weight_ * dz * dz);
          double clearance_penalty = 0.0;
          if (preferred_clearance_ > 1e-3 && clearance_weight_ > 0.0) {
            const double clearance = std::min(edt_env_->sdf_map_->getDistance(cur_pos),
                                              edt_env_->sdf_map_->getDistance(nbr_pos));
            if (clearance < preferred_clearance_) {
              const double deficit =
                  (preferred_clearance_ - std::max(0.0, clearance)) / preferred_clearance_;
              clearance_penalty = clearance_weight_ * weighted_step * deficit * deficit;
            }
          }
          double tmp_g_score = weighted_step + clearance_penalty + cur_node->g_score;
          auto node_iter = open_set_map_.find(nbr_idx);
          if (node_iter == open_set_map_.end()) {
            neighbor = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              cout << "run out of node pool." << endl;
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          open_set_map_[nbr_idx] = neighbor;
        }
  }
  // 2026-07-16: open set 耗尽才是真正的局部自由空间不连通；单独打印以便与超时区分。
  ROS_WARN_THROTTLE(1.0,
                    "[Astar] DISCONNECTED nodes=%d iter=%d start=(%.2f %.2f %.2f) "
                    "start_occ=%d/%d goal=(%.2f %.2f %.2f) goal_occ=%d/%d.",
                    use_node_num_, iter_num_, start_pt.x(), start_pt.y(), start_pt.z(),
                    start_occ, start_inflate, end_pt.x(), end_pt.y(), end_pt.z(),
                    goal_occ, goal_inflate);
  return NO_PATH;
}

double Astar::getEarlyTerminateCost() {
  return early_terminate_cost_;
}

void Astar::reset() {
  open_set_map_.clear();
  close_set_map_.clear();
  path_nodes_.clear();

  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
  open_set_.swap(empty_queue);
  for (int i = 0; i < use_node_num_; i++) {
    path_node_pool_[i]->parent = NULL;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
}

double Astar::pathLength(const vector<Eigen::Vector3d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;
  for (int i = 0; i < path.size() - 1; ++i)
    length += (path[i + 1] - path[i]).norm();
  return length;
}

void Astar::backtrack(const NodePtr& end_node, const Eigen::Vector3d& end) {
  path_nodes_.push_back(end);
  path_nodes_.push_back(end_node->position);
  NodePtr cur_node = end_node;
  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node->position);
  }
  reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<Eigen::Vector3d> Astar::getPath() {
  return path_nodes_;
}

double Astar::getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));
  double h;
  double diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return tie_breaker_ * h;
}

double Astar::getManhHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));
  return tie_breaker_ * (dx + dy + dz);
}

double Astar::getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  return tie_breaker_ * (x2 - x1).norm();
}

std::vector<Eigen::Vector3d> Astar::getVisited() {
  vector<Eigen::Vector3d> visited;
  for (int i = 0; i < use_node_num_; ++i)
    visited.push_back(path_node_pool_[i]->position);
  return visited;
}

void Astar::posToIndex(const Eigen::Vector3d& pt, Eigen::Vector3i& idx) {
  idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
}

}  // namespace fast_planner
