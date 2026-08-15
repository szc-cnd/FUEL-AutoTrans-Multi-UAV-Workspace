#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <std_msgs/Header.h>

#include <ldop/DynamicObjectPredictionArray.h>
#include <ldop/dynamic_object_tracker.h>

namespace ldopcore {

struct PredictionInteractionParams {
  bool enabled{true};                       // P3 主开关；测试或回归时可关闭交互逻辑以保留 P2 行为基线。
  double pair_interaction_weight{1.0};      // 目标间距离、TTC 和交叉冲突能量的总体权重。
  double map_interaction_weight{1.0};       // 静态 occupied 与近障碍软约束的总体权重。
  double corridor_interaction_weight{0.4};  // free corridor 方向先验只作为轻量偏置，避免覆盖运动模型语义。
  double interaction_safe_distance{0.4};    // 在目标尺寸外额外保留的安全距离，单位 m。
  double near_obstacle_distance{0.6};       // 近障碍软惩罚半径，单位 m；穿障仍由 occupied 单独表达。
  int map_query_depth{0};                   // UFOMap 查询深度，0 为叶子；调大可用更粗体素降低局部查询成本。
  double ttc_safe_time{1.5};                // TTC 低于该时间时认为分支存在短时碰撞风险。
  double free_corridor_query_radius{1.2};   // 局部可通行方向采样半径，单位 m。
  double max_correction_distance{0.2};      // 单点均值轨迹最大位置修正，限制交互逻辑不替代 rollout。
  double max_deceleration_ratio{0.35};      // 单点速度最大衰减比例，用于后续轻量减速建议。
  double interaction_energy_tau{1.0};       // 能量转概率缩放的温度，必须为正以避免指数权重退化。
};

struct PredictionInteractionConfig {
  bool enabled{true};
  double pair_interaction_weight{1.0};
  double map_interaction_weight{1.0};
  double corridor_interaction_weight{0.4};
  double interaction_safe_distance{0.4};
  double near_obstacle_distance{0.6};
  std::uint8_t map_query_depth{0U};
  double ttc_safe_time{1.5};
  double free_corridor_query_radius{1.2};
  double max_correction_distance{0.2};
  double max_deceleration_ratio{0.35};
  double interaction_energy_tau{1.0};
};

struct PredictionMapNodeQuery {
  enum class OccupancyState : std::uint8_t {
    Unknown = 0,
    Free = 1,
    Occupied = 2,
    OutOfMap = 3,
  };

  OccupancyState state{OccupancyState::Unknown};
  bool exists{false};
  bool occupied{false};
  bool free{false};
  bool unknown{true};
  bool out_of_map{false};
  bool seen_free{false};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double voxel_size{0.0};
};

struct PredictionLocalOccupiedNode {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double voxel_size{0.0};
  std::uint32_t hits{0U};
};

class PredictionMapQuery {
 public:
  virtual ~PredictionMapQuery() = default;

  virtual bool available() const = 0;
  virtual std::optional<PredictionMapNodeQuery> queryNode(
      const Eigen::Vector3d& point) const = 0;
  virtual bool seenFree(const Eigen::Vector3d& point) const = 0;
  virtual std::vector<PredictionLocalOccupiedNode> queryLocalOccupied(
      const Eigen::Vector3d& center,
      double radius,
      std::size_t max_results) const = 0;
  virtual std::vector<PredictionLocalOccupiedNode> queryLocalOccupied(
      const Eigen::Vector3d& center,
      double radius,
      std::size_t max_results,
      std::uint8_t query_depth) const;
  virtual std::optional<PredictionLocalOccupiedNode> queryNearestOccupied(
      const Eigen::Vector3d& center,
      double radius,
      std::uint8_t query_depth = 0U) const;
};

struct TrajectoryCorrectionHint {
  bool has_position_offset{false};
  bool has_deceleration{false};
  bool has_direction_bias{false};
  Eigen::Vector3d position_offset{Eigen::Vector3d::Zero()};
  double deceleration_ratio{0.0};
  Eigen::Vector3d direction_bias{Eigen::Vector3d::Zero()};
};

enum class PredictionBranchRiskLevel : std::uint8_t {
  Low = 0,
  Medium = 1,
  High = 2,
  Critical = 3,
};

struct PredictionBranchInteraction {
  std::uint32_t object_id{0U};
  std::size_t branch_index{0U};
  double pair_cost{0.0};
  double map_cost{0.0};
  double corridor_cost{0.0};
  double total_energy{0.0};
  double probability_scale{1.0};
  PredictionBranchRiskLevel risk_level{PredictionBranchRiskLevel::Low};
  double nearest_obstacle_distance{std::numeric_limits<double>::infinity()};
  bool occupied_overlap{false};
  bool near_obstacle{false};
  Eigen::Vector3d free_corridor_direction{Eigen::Vector3d::Zero()};
  double free_corridor_confidence{0.0};
  double nearest_pair_distance{std::numeric_limits<double>::infinity()};
  double minimum_ttc{std::numeric_limits<double>::infinity()};
  // 分支级 hint 只作为诊断摘要：记录该分支最强的交互修正证据，便于测试和后续 P4 复用。
  TrajectoryCorrectionHint correction_hint;
  // 真正应用到预测均值轨迹时使用逐点 hint，避免把局部碰撞/近障碍证据扩散成整条轨迹平移。
  std::vector<TrajectoryCorrectionHint> point_correction_hints;
};

struct PredictionInteractionDiagnostics {
  double max_pair_cost{0.0};
  double max_map_cost{0.0};
  double max_corridor_cost{0.0};
  std::size_t colliding_branch_count{0U};
  std::size_t near_obstacle_branch_count{0U};
  std::size_t near_pair_branch_count{0U};
  std::size_t free_corridor_aligned_branch_count{0U};
  bool map_unavailable{false};
};

struct PredictionInteractionResult {
  std::vector<std::vector<PredictionBranchInteraction>> interactions_by_prediction;
  PredictionInteractionDiagnostics diagnostics;
};

PredictionInteractionConfig buildPredictionInteractionConfig(
    const PredictionInteractionParams& params);

class PredictionInteractionContext {
 public:
  explicit PredictionInteractionContext(PredictionInteractionConfig config);

  PredictionInteractionResult evaluate(
      const std_msgs::Header& header,
      const std::vector<TrackPredictionInput>& inputs,
      const ldop::DynamicObjectPredictionArray& predictions,
      const PredictionMapQuery* map_query) const;

 private:
  PredictionInteractionConfig config_;
};

}  // namespace ldopcore
