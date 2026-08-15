#include <ldop/prediction_interaction_context.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace ldopcore {

std::vector<PredictionLocalOccupiedNode> PredictionMapQuery::queryLocalOccupied(
    const Eigen::Vector3d& center,
    const double radius,
    const std::size_t max_results,
    const std::uint8_t /*query_depth*/) const {
  return queryLocalOccupied(center, radius, max_results);
}

std::optional<PredictionLocalOccupiedNode> PredictionMapQuery::queryNearestOccupied(
    const Eigen::Vector3d& center,
    const double radius,
    const std::uint8_t query_depth) const {
  const auto occupied_nodes = queryLocalOccupied(center, radius, 0U, query_depth);
  std::optional<PredictionLocalOccupiedNode> nearest;
  double nearest_surface_distance = std::numeric_limits<double>::infinity();
  for (const auto& node : occupied_nodes) {
    const double surface_distance =
        (node.center - center).norm() - std::max(0.0, node.voxel_size) * 0.5;
    if (surface_distance < nearest_surface_distance) {
      nearest_surface_distance = surface_distance;
      nearest = node;
    }
  }
  return nearest;
}

namespace {

constexpr double kMinDistance{1.0e-6};
constexpr double kMinSpeedSquared{1.0e-6};
constexpr std::size_t kMaxBranchOccupiedResults{64U};

struct BranchSample {
  std::size_t point_index{0U};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

struct BranchSpatialQuery {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
};

std::optional<Eigen::Vector3d> pointPosition(
    const ldop::DynamicObjectPredictionPoint& point) {
  if (point.model_state.size() < 3U) {
    return std::nullopt;
  }
  return Eigen::Vector3d(point.model_state[0], point.model_state[1], point.model_state[2]);
}

std::vector<BranchSample> validBranchSamples(
    const ldop::DynamicObjectPredictionBranch& branch) {
  std::vector<BranchSample> samples;
  samples.reserve(branch.points.size());
  for (std::size_t point_index = 0U; point_index < branch.points.size(); ++point_index) {
    if (const auto position = pointPosition(branch.points[point_index])) {
      BranchSample sample;
      sample.point_index = point_index;
      sample.position = *position;
      samples.push_back(sample);
    }
  }
  return samples;
}

std::optional<BranchSpatialQuery> branchSpatialQuery(
    const std::vector<BranchSample>& samples,
    const double sample_radius) {
  if (samples.empty()) {
    return std::nullopt;
  }

  Eigen::Vector3d min_corner = samples.front().position;
  Eigen::Vector3d max_corner = samples.front().position;
  for (const auto& sample : samples) {
    min_corner = min_corner.cwiseMin(sample.position);
    max_corner = max_corner.cwiseMax(sample.position);
  }

  BranchSpatialQuery query;
  query.center = 0.5 * (min_corner + max_corner);
  double branch_radius = 0.0;
  for (const auto& sample : samples) {
    branch_radius = std::max(branch_radius, (sample.position - query.center).norm());
  }
  // 单次分支级查询覆盖整条短时 rollout 和目标外接圆/近障碍半径，
  // 避免每个采样点都持有 UFOMap 读锁做空间查询。
  query.radius = branch_radius + std::max(0.0, sample_radius);
  return query;
}

Eigen::Vector3d branchRepresentativePosition(
    const std::vector<BranchSample>& samples) {
  // corridor 只是方向先验，用时间中点代表该分支，可把查询规模固定到每分支 8 个方向。
  return samples[samples.size() / 2U].position;
}

double horizontalObjectRadius(const ldop::DynamicObjectPrediction& prediction) {
  // 交互代价只在水平面内比较，使用外接圆半径可让不同 bbox 朝向下的安全距离保持保守。
  const double width = std::max(0.0, prediction.size.x);
  const double length = std::max(0.0, prediction.size.y);
  return 0.5 * std::hypot(width, length);
}

double objectQueryRadius(const ldop::DynamicObjectPrediction& prediction) {
  // 地图查询需要覆盖目标外接圆，而不是只查预测点所在体素；否则贴障但中心未入障碍的分支会漏检。
  return std::max(horizontalObjectRadius(prediction), kMinDistance);
}

Eigen::Vector3d normalizedOrZero(Eigen::Vector3d direction) {
  direction.z() = 0.0;
  const double norm = direction.norm();
  if (norm <= kMinDistance) {
    return Eigen::Vector3d::Zero();
  }
  return direction / norm;
}

double distanceToOccupiedSurface(const Eigen::Vector3d& position,
                                 const PredictionLocalOccupiedNode& occupied,
                                 const double object_radius) {
  Eigen::Vector3d delta = position - occupied.center;
  delta.z() = 0.0;
  // 这里返回带符号距离：负值表示目标外接圆已经侵入 occupied 体素，便于同一逻辑区分碰撞和近障碍。
  return delta.norm() - std::max(0.0, occupied.voxel_size) * 0.5 -
         std::max(0.0, object_radius);
}

std::optional<Eigen::Vector3d> stateVelocity(
    const ldop::DynamicObjectPredictionPoint& point) {
  if (point.model_state.size() < 6U) {
    return std::nullopt;
  }
  return Eigen::Vector3d(point.model_state[3], point.model_state[4], point.model_state[5]);
}

std::optional<Eigen::Vector3d> branchVelocityAt(
    const ldop::DynamicObjectPredictionBranch& branch,
    const std::size_t point_index) {
  if (point_index >= branch.points.size()) {
    return std::nullopt;
  }

  const auto velocity_from_points =
      [&branch](const std::size_t from_index,
                const std::size_t to_index) -> std::optional<Eigen::Vector3d> {
    const auto from = pointPosition(branch.points[from_index]);
    const auto to = pointPosition(branch.points[to_index]);
    const double dt =
        (branch.points[to_index].time_from_start - branch.points[from_index].time_from_start)
            .toSec();
    if (!from || !to || dt <= 0.0) {
      return std::nullopt;
    }
    return (*to - *from) / dt;
  };

  // 分支点中的 model_state 可能来自不同运动模型；用相邻采样反推速度可保持交互判定与实际轨迹形状一致。
  if (point_index + 1U < branch.points.size()) {
    if (const auto velocity = velocity_from_points(point_index, point_index + 1U)) {
      return velocity;
    }
  }
  if (point_index > 0U) {
    if (const auto velocity = velocity_from_points(point_index - 1U, point_index)) {
      return velocity;
    }
  }
  return stateVelocity(branch.points[point_index]);
}

Eigen::Vector3d branchDirection(const ldop::DynamicObjectPredictionBranch& branch) {
  std::optional<Eigen::Vector3d> first_position;
  std::optional<Eigen::Vector3d> last_position;
  for (const auto& point : branch.points) {
    const auto position = pointPosition(point);
    if (!position) {
      continue;
    }
    if (!first_position) {
      first_position = position;
    }
    last_position = position;
  }
  if (first_position && last_position) {
    if (const auto direction = normalizedOrZero(*last_position - *first_position);
        direction.squaredNorm() > 0.0) {
      return direction;
    }
  }
  if (!branch.points.empty()) {
    if (const auto velocity = branchVelocityAt(branch, 0U)) {
      return normalizedOrZero(*velocity);
    }
  }
  return Eigen::Vector3d::Zero();
}

double branchTimeToCollision(const Eigen::Vector3d& position_a,
                             const Eigen::Vector3d& velocity_a,
                             const Eigen::Vector3d& position_b,
                             const Eigen::Vector3d& velocity_b) {
  const Eigen::Vector2d relative_position(position_b.x() - position_a.x(),
                                          position_b.y() - position_a.y());
  const Eigen::Vector2d relative_velocity(velocity_b.x() - velocity_a.x(),
                                          velocity_b.y() - velocity_a.y());
  const double speed_squared = relative_velocity.squaredNorm();
  if (speed_squared <= kMinSpeedSquared) {
    return std::numeric_limits<double>::infinity();
  }

  const double closing_projection = relative_position.dot(relative_velocity);
  if (closing_projection >= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return -closing_projection / speed_squared;
}

double approachClassMultiplier(const ldop::DynamicObjectPrediction& prediction_a,
                               const ldop::DynamicObjectPrediction& prediction_b,
                               const Eigen::Vector3d& velocity_a,
                               const Eigen::Vector3d& velocity_b) {
  const Eigen::Vector2d horizontal_a(velocity_a.x(), velocity_a.y());
  const Eigen::Vector2d horizontal_b(velocity_b.x(), velocity_b.y());
  const double speed_a = horizontal_a.norm();
  const double speed_b = horizontal_b.norm();

  double approach_multiplier = 1.0;
  if (speed_a > kMinDistance && speed_b > kMinDistance) {
    const double direction_alignment = horizontal_a.dot(horizontal_b) / (speed_a * speed_b);
    if (direction_alignment < -0.5) {
      approach_multiplier = 1.8;
    } else if (direction_alignment > 0.5) {
      approach_multiplier = 0.55;
    } else {
      approach_multiplier = 1.25;
    }
  }

  // 人和 UAV 对规划侧更敏感，轻量调高代价；类别未知时保持中性，避免首版规则过度假设。
  const auto class_sensitivity = [](const std::uint8_t object_class) {
    if (object_class == ldop::DynamicObjectPrediction::CLASS_HUMAN ||
        object_class == ldop::DynamicObjectPrediction::CLASS_UAV) {
      return 1.1;
    }
    return 1.0;
  };
  return approach_multiplier * class_sensitivity(prediction_a.object_class) *
         class_sensitivity(prediction_b.object_class);
}

void updateDiagnostics(PredictionInteractionDiagnostics& diagnostics,
                       const PredictionBranchInteraction& interaction) {
  diagnostics.max_pair_cost = std::max(diagnostics.max_pair_cost, interaction.pair_cost);
  diagnostics.max_map_cost = std::max(diagnostics.max_map_cost, interaction.map_cost);
  diagnostics.max_corridor_cost =
      std::max(diagnostics.max_corridor_cost, interaction.corridor_cost);
  if (interaction.pair_cost > 0.0) {
    ++diagnostics.near_pair_branch_count;
  }
  if (interaction.occupied_overlap) {
    ++diagnostics.colliding_branch_count;
  }
  if (interaction.near_obstacle) {
    ++diagnostics.near_obstacle_branch_count;
  }
  if (interaction.free_corridor_confidence > 0.0) {
    ++diagnostics.free_corridor_aligned_branch_count;
  }
}

void applyStrongerPositionOffsetHint(const Eigen::Vector3d& offset,
                                     TrajectoryCorrectionHint& hint) {
  if (offset.squaredNorm() <= kMinDistance * kMinDistance) {
    return;
  }
  if (!hint.has_position_offset || offset.norm() > hint.position_offset.norm()) {
    hint.has_position_offset = true;
    hint.position_offset = offset;
  }
}

void applyLocalPositionOffsetHint(const Eigen::Vector3d& offset,
                                  const std::size_t point_index,
                                  PredictionBranchInteraction& interaction) {
  // correction_hint 是分支摘要，point_correction_hints 才是后续实际应用到对应采样点的局部修正。
  applyStrongerPositionOffsetHint(offset, interaction.correction_hint);
  if (point_index < interaction.point_correction_hints.size()) {
    applyStrongerPositionOffsetHint(offset, interaction.point_correction_hints[point_index]);
  }
}

void applyPositionOffsetHint(const Eigen::Vector3d& position_a,
                             const Eigen::Vector3d& position_b,
                             const PredictionInteractionConfig& config,
                             const double severity,
                             const std::size_t point_index,
                             PredictionBranchInteraction& interaction) {
  if (config.max_correction_distance <= 0.0) {
    return;
  }
  const double clamped_severity = std::clamp(severity, 0.0, 1.0);
  Eigen::Vector3d direction = position_a - position_b;
  direction.z() = 0.0;
  if (direction.squaredNorm() <= kMinDistance * kMinDistance) {
    return;
  }
  applyLocalPositionOffsetHint(
      direction.normalized() * config.max_correction_distance * clamped_severity,
      point_index,
      interaction);
}

void applyLocalDecelerationHint(const double deceleration_ratio,
                                const std::size_t point_index,
                                PredictionBranchInteraction& interaction) {
  const double ratio = std::clamp(deceleration_ratio, 0.0, 1.0);
  if (ratio <= 0.0) {
    return;
  }
  interaction.correction_hint.has_deceleration = true;
  interaction.correction_hint.deceleration_ratio =
      std::max(interaction.correction_hint.deceleration_ratio, ratio);
  if (point_index < interaction.point_correction_hints.size()) {
    auto& point_hint = interaction.point_correction_hints[point_index];
    point_hint.has_deceleration = true;
    point_hint.deceleration_ratio = std::max(point_hint.deceleration_ratio, ratio);
  }
}

void applyDirectionBiasHint(const Eigen::Vector3d& direction_bias,
                            PredictionBranchInteraction& interaction) {
  if (direction_bias.squaredNorm() <= kMinDistance * kMinDistance) {
    return;
  }
  interaction.correction_hint.has_direction_bias = true;
  interaction.correction_hint.direction_bias = direction_bias;
  for (auto& point_hint : interaction.point_correction_hints) {
    point_hint.has_direction_bias = true;
    point_hint.direction_bias = direction_bias;
  }
}

void evaluatePairCosts(const std::vector<TrackPredictionInput>& inputs,
                       const ldop::DynamicObjectPredictionArray& predictions,
                       const PredictionInteractionConfig& config,
                       PredictionInteractionResult& result) {
  // Task 3 首版仅依赖未来分支轨迹和 prediction.size 计算动态目标交互；inputs 保留给 P4
  // 在闭环校准时复用 tracker 原生状态、类别历史和置信度。
  (void)inputs;

  for (std::size_t prediction_index = 0U;
       prediction_index < predictions.predictions.size();
       ++prediction_index) {
    const auto& prediction = predictions.predictions[prediction_index];
    const double radius = horizontalObjectRadius(prediction);

    for (std::size_t branch_index = 0U;
         branch_index < prediction.branches.size();
         ++branch_index) {
      const auto& branch = prediction.branches[branch_index];
      auto& interaction = result.interactions_by_prediction[prediction_index][branch_index];

      for (std::size_t other_prediction_index = 0U;
           other_prediction_index < predictions.predictions.size();
           ++other_prediction_index) {
        if (other_prediction_index == prediction_index) {
          continue;
        }
        const auto& other_prediction = predictions.predictions[other_prediction_index];
        const double other_radius = horizontalObjectRadius(other_prediction);
        const double safe_distance =
            radius + other_radius + config.interaction_safe_distance;
        const double ttc_distance_gate = safe_distance + config.interaction_safe_distance;
        double weighted_branch_cost_sum = 0.0;
        double branch_probability_sum = 0.0;
        double equal_branch_cost_sum = 0.0;
        std::size_t evaluated_branch_count = 0U;
        const bool has_positive_branch_probability =
            std::any_of(other_prediction.branches.begin(),
                        other_prediction.branches.end(),
                        [](const ldop::DynamicObjectPredictionBranch& candidate) {
                          return candidate.probability > 0.0;
                        });

        for (const auto& other_branch : other_prediction.branches) {
          // 同一目标只要有正概率分支，零概率分支就表示该假设已被 GMM 排除；cost、
          // 最近距离、TTC 和修正建议必须一起跳过，避免诊断/控制 hint 与概率语义脱节。
          const double branch_probability = std::max(0.0, other_branch.probability);
          if (has_positive_branch_probability && branch_probability <= 0.0) {
            continue;
          }

          const std::size_t sample_count =
              std::min(branch.points.size(), other_branch.points.size());
          double sample_cost_sum = 0.0;
          std::size_t valid_sample_count = 0U;

          for (std::size_t sample_index = 0U; sample_index < sample_count; ++sample_index) {
            const auto position = pointPosition(branch.points[sample_index]);
            const auto other_position = pointPosition(other_branch.points[sample_index]);
            if (!position || !other_position) {
              continue;
            }
            ++valid_sample_count;
            double sample_cost = 0.0;

            const Eigen::Vector2d delta(other_position->x() - position->x(),
                                        other_position->y() - position->y());
            const double distance = delta.norm();
            interaction.nearest_pair_distance =
                std::min(interaction.nearest_pair_distance, distance);

            const auto velocity = branchVelocityAt(branch, sample_index);
            const auto other_velocity = branchVelocityAt(other_branch, sample_index);
            const double multiplier =
                velocity && other_velocity
                    ? approachClassMultiplier(prediction, other_prediction, *velocity,
                                              *other_velocity)
                    : 1.0;

            if (distance < safe_distance) {
              const double overlap_ratio =
                  std::clamp((safe_distance - distance) /
                                 std::max(safe_distance, kMinDistance),
                             0.0, 1.0);
              sample_cost += multiplier * overlap_ratio * overlap_ratio;
              applyPositionOffsetHint(*position,
                                      *other_position,
                                      config,
                                      overlap_ratio,
                                      sample_index,
                                      interaction);
            }

            if (!velocity || !other_velocity) {
              sample_cost_sum += sample_cost;
              continue;
            }
            const double ttc =
                branchTimeToCollision(*position, *velocity, *other_position, *other_velocity);
            interaction.minimum_ttc = std::min(interaction.minimum_ttc, ttc);
            if (ttc < config.ttc_safe_time && distance <= ttc_distance_gate) {
              const double ttc_ratio =
                  std::clamp((config.ttc_safe_time - ttc) /
                                 std::max(config.ttc_safe_time, kMinDistance),
                             0.0, 1.0);
              sample_cost += multiplier * ttc_ratio;
              applyLocalDecelerationHint(config.max_deceleration_ratio * ttc_ratio,
                                         sample_index,
                                         interaction);
            }
            sample_cost_sum += sample_cost;
          }

          if (valid_sample_count == 0U) {
            continue;
          }
          const double branch_cost =
              sample_cost_sum / static_cast<double>(valid_sample_count);
          equal_branch_cost_sum += branch_cost;
          ++evaluated_branch_count;

          // GMM 分支概率只用于同一目标内部的期望风险；负概率不反向抵消风险，零概率在全零时走等权退化。
          weighted_branch_cost_sum += branch_probability * branch_cost;
          branch_probability_sum += branch_probability;
        }

        if (branch_probability_sum > 0.0) {
          interaction.pair_cost += weighted_branch_cost_sum / branch_probability_sum;
        } else if (evaluated_branch_count > 0U) {
          interaction.pair_cost +=
              equal_branch_cost_sum / static_cast<double>(evaluated_branch_count);
        }
      }

    }
  }
}

void evaluateMapCosts(const ldop::DynamicObjectPredictionArray& predictions,
                      const PredictionMapQuery* map_query,
                      const PredictionInteractionConfig& config,
                      PredictionInteractionResult& result) {
  if (map_query == nullptr || !map_query->available()) {
    result.diagnostics.map_unavailable = true;
    return;
  }

  for (std::size_t prediction_index = 0U;
       prediction_index < predictions.predictions.size();
       ++prediction_index) {
    const auto& prediction = predictions.predictions[prediction_index];
    const double query_radius = objectQueryRadius(prediction);

    for (std::size_t branch_index = 0U;
         branch_index < prediction.branches.size();
         ++branch_index) {
      const auto& branch = prediction.branches[branch_index];
      auto& interaction = result.interactions_by_prediction[prediction_index][branch_index];
      const Eigen::Vector3d fallback_direction = branchDirection(branch);
      const auto samples = validBranchSamples(branch);
      const auto query_region = branchSpatialQuery(
          samples, config.near_obstacle_distance + query_radius);
      if (!query_region) {
        continue;
      }
      const auto occupied_nodes = map_query->queryLocalOccupied(
          query_region->center,
          query_region->radius,
          kMaxBranchOccupiedResults,
          config.map_query_depth);
      double cost_sum = 0.0;
      bool has_correction_hint = false;
      double correction_severity = 0.0;
      double correction_signed_distance = std::numeric_limits<double>::infinity();
      std::size_t correction_point_index = 0U;
      Eigen::Vector3d correction_direction = Eigen::Vector3d::Zero();

      for (const auto& sample : samples) {
        std::optional<PredictionLocalOccupiedNode> nearest_occupied;
        double nearest_signed_distance = std::numeric_limits<double>::infinity();
        for (const auto& occupied : occupied_nodes) {
          const double signed_distance =
              distanceToOccupiedSurface(sample.position, occupied, query_radius);
          if (signed_distance < nearest_signed_distance) {
            nearest_signed_distance = signed_distance;
            nearest_occupied = occupied;
          }
        }

        if (!nearest_occupied) {
          continue;
        }

        interaction.nearest_obstacle_distance =
            std::min(interaction.nearest_obstacle_distance,
                     std::max(0.0, nearest_signed_distance));

        const bool occupied_overlap =
            nearest_signed_distance <= 0.0;
        double severity = 0.0;
        if (occupied_overlap) {
          interaction.occupied_overlap = true;
          severity = 1.0;
          cost_sum += 1.0;
        } else if (nearest_signed_distance < config.near_obstacle_distance) {
          interaction.near_obstacle = true;
          severity =
              std::clamp((config.near_obstacle_distance - nearest_signed_distance) /
                             std::max(config.near_obstacle_distance, kMinDistance),
                         0.0, 1.0);
          cost_sum += severity * severity;
        }

        if (severity <= 0.0 || config.max_correction_distance <= 0.0) {
          continue;
        }
        Eigen::Vector3d push_direction =
            normalizedOrZero(sample.position - nearest_occupied->center);
        if (push_direction.squaredNorm() <= 0.0) {
          if (fallback_direction.squaredNorm() > 0.0) {
            push_direction = -fallback_direction;
          } else {
            push_direction = Eigen::Vector3d::UnitX();
          }
        }
        const bool stronger_correction =
            severity > correction_severity + kMinDistance ||
            (std::abs(severity - correction_severity) <= kMinDistance &&
             nearest_signed_distance < correction_signed_distance);
        if (!has_correction_hint || stronger_correction) {
          // 地图查询已扩大到分支级；控制 hint 仍只取最强局部证据，
          // 避免一个 occupied 候选把整条短时轨迹都推成满幅修正。
          has_correction_hint = true;
          correction_severity = severity;
          correction_signed_distance = nearest_signed_distance;
          correction_point_index = sample.point_index;
          correction_direction = push_direction;
        }
      }

      interaction.map_cost += cost_sum / static_cast<double>(samples.size());
      if (has_correction_hint) {
        applyLocalPositionOffsetHint(
            correction_direction * config.max_correction_distance * correction_severity,
            correction_point_index,
            interaction);
      }
    }
  }
}

void evaluateCorridorCosts(const ldop::DynamicObjectPredictionArray& predictions,
                           const PredictionMapQuery* map_query,
                           const PredictionInteractionConfig& config,
                           PredictionInteractionResult& result) {
  if (map_query == nullptr || !map_query->available()) {
    return;
  }

  const std::array<Eigen::Vector3d, 8U> directions{
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.70710678118, 0.70710678118, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(-0.70710678118, 0.70710678118, 0.0),
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(-0.70710678118, -0.70710678118, 0.0),
      Eigen::Vector3d(0.0, -1.0, 0.0),
      Eigen::Vector3d(0.70710678118, -0.70710678118, 0.0),
  };

  for (std::size_t prediction_index = 0U;
       prediction_index < predictions.predictions.size();
       ++prediction_index) {
    const auto& prediction = predictions.predictions[prediction_index];

    for (std::size_t branch_index = 0U;
         branch_index < prediction.branches.size();
         ++branch_index) {
      const auto& branch = prediction.branches[branch_index];
      auto& interaction = result.interactions_by_prediction[prediction_index][branch_index];
      const auto samples = validBranchSamples(branch);
      if (samples.empty()) {
        continue;
      }
      const Eigen::Vector3d representative_position = branchRepresentativePosition(samples);

      const Eigen::Vector3d motion_direction = branchDirection(branch);
      double best_alignment = -std::numeric_limits<double>::infinity();
      Eigen::Vector3d best_direction = Eigen::Vector3d::Zero();
      std::size_t free_direction_count = 0U;
      for (const auto& direction : directions) {
        const Eigen::Vector3d probe =
            representative_position + direction * config.free_corridor_query_radius;
        if (!map_query->seenFree(probe)) {
          continue;
        }

        ++free_direction_count;
        // 多个方向同为 seenFree 时，用分支自身运动方向做确定性 tie-break；
        // 这样保留 corridor 作为方向先验，又不重新引入昂贵的 per-point occupied 查询。
        const double alignment =
            motion_direction.squaredNorm() > 0.0 ? motion_direction.dot(direction) : 0.0;
        if (alignment > best_alignment) {
          best_alignment = alignment;
          best_direction = direction;
        }
      }

      if (free_direction_count == 0U || free_direction_count == directions.size()) {
        continue;
      }
      interaction.free_corridor_direction = best_direction;
      // corridor bias 只表达“局部可通行方向不是各向同性”；全方向同样 free 时不注入任意方向偏置。
      interaction.free_corridor_confidence =
          1.0 - static_cast<double>(free_direction_count) /
                    static_cast<double>(directions.size());
      applyDirectionBiasHint(best_direction * interaction.free_corridor_confidence,
                             interaction);

      if (motion_direction.squaredNorm() <= 0.0) {
        interaction.corridor_cost += 0.5 * interaction.free_corridor_confidence;
        continue;
      }
      const double alignment =
          std::clamp(motion_direction.dot(best_direction), -1.0, 1.0);
      interaction.corridor_cost +=
          0.5 * (1.0 - alignment) * interaction.free_corridor_confidence;
    }
  }
}

PredictionBranchRiskLevel classifyRiskLevel(
    const PredictionBranchInteraction& interaction,
    const PredictionInteractionConfig& config) {
  if (interaction.occupied_overlap) {
    return PredictionBranchRiskLevel::Critical;
  }

  const bool imminent_ttc =
      interaction.minimum_ttc <= 0.5 * std::max(config.ttc_safe_time, kMinDistance);
  const bool high_pair_risk = interaction.pair_cost >= 0.5;
  const bool high_map_risk = interaction.map_cost >= 0.5;
  const bool high_energy = interaction.total_energy >= 1.0;
  if (imminent_ttc || high_pair_risk || high_map_risk || high_energy) {
    return PredictionBranchRiskLevel::High;
  }

  const bool ttc_inside_gate = interaction.minimum_ttc <= config.ttc_safe_time;
  const bool any_pair_risk = interaction.pair_cost > 0.0;
  const bool any_map_risk = interaction.near_obstacle || interaction.map_cost > 0.0;
  const bool corridor_bias =
      interaction.corridor_cost > 0.0 || interaction.free_corridor_confidence > 0.0;
  if (ttc_inside_gate || any_pair_risk || any_map_risk || corridor_bias) {
    return PredictionBranchRiskLevel::Medium;
  }

  return PredictionBranchRiskLevel::Low;
}

void finalizeInteractions(const PredictionInteractionConfig& config,
                          PredictionInteractionResult& result) {
  const bool map_unavailable = result.diagnostics.map_unavailable;
  result.diagnostics = PredictionInteractionDiagnostics{};
  result.diagnostics.map_unavailable = map_unavailable;

  for (auto& prediction_interactions : result.interactions_by_prediction) {
    for (auto& interaction : prediction_interactions) {
      interaction.total_energy =
          config.pair_interaction_weight * interaction.pair_cost +
          config.map_interaction_weight * interaction.map_cost +
          config.corridor_interaction_weight * interaction.corridor_cost;
      const double tau =
          config.interaction_energy_tau > 0.0 ? config.interaction_energy_tau : 1.0;
      interaction.probability_scale = std::exp(-interaction.total_energy / tau);
      interaction.risk_level = classifyRiskLevel(interaction, config);
      updateDiagnostics(result.diagnostics, interaction);
    }
  }
}

}  // namespace

PredictionInteractionConfig buildPredictionInteractionConfig(
    const PredictionInteractionParams& params) {
  const PredictionInteractionParams defaults;
  PredictionInteractionConfig config;

  config.enabled = params.enabled;
  config.pair_interaction_weight =
      params.pair_interaction_weight >= 0.0 ? params.pair_interaction_weight
                                            : defaults.pair_interaction_weight;
  config.map_interaction_weight =
      params.map_interaction_weight >= 0.0 ? params.map_interaction_weight
                                           : defaults.map_interaction_weight;
  config.corridor_interaction_weight =
      params.corridor_interaction_weight >= 0.0 ? params.corridor_interaction_weight
                                                : defaults.corridor_interaction_weight;
  config.interaction_safe_distance =
      params.interaction_safe_distance >= 0.0 ? params.interaction_safe_distance
                                              : defaults.interaction_safe_distance;
  config.near_obstacle_distance =
      params.near_obstacle_distance >= 0.0 ? params.near_obstacle_distance
                                           : defaults.near_obstacle_distance;
  config.map_query_depth =
      static_cast<std::uint8_t>(std::clamp(params.map_query_depth, 0, 255));
  config.ttc_safe_time =
      params.ttc_safe_time > 0.0 ? params.ttc_safe_time : defaults.ttc_safe_time;
  config.free_corridor_query_radius =
      params.free_corridor_query_radius > 0.0 ? params.free_corridor_query_radius
                                              : defaults.free_corridor_query_radius;
  config.max_correction_distance =
      params.max_correction_distance >= 0.0 ? params.max_correction_distance
                                            : defaults.max_correction_distance;
  config.max_deceleration_ratio =
      std::clamp(params.max_deceleration_ratio, 0.0, 1.0);
  config.interaction_energy_tau =
      params.interaction_energy_tau > 0.0 ? params.interaction_energy_tau
                                          : defaults.interaction_energy_tau;

  return config;
}

PredictionInteractionContext::PredictionInteractionContext(
    const PredictionInteractionConfig config)
    : config_(config) {}

PredictionInteractionResult PredictionInteractionContext::evaluate(
    const std_msgs::Header& /*header*/,
    const std::vector<TrackPredictionInput>& inputs,
    const ldop::DynamicObjectPredictionArray& predictions,
    const PredictionMapQuery* map_query) const {
  PredictionInteractionResult result;
  result.interactions_by_prediction.resize(predictions.predictions.size());
  result.diagnostics.map_unavailable =
      map_query == nullptr || !map_query->available();

  // 先建立 predictor 后续消费所需的二维索引形状；关闭交互时该 identity 形状就是稳定回退契约。
  for (std::size_t prediction_index = 0U;
       prediction_index < predictions.predictions.size();
       ++prediction_index) {
    const auto& prediction = predictions.predictions[prediction_index];
    auto& interactions = result.interactions_by_prediction[prediction_index];
    interactions.reserve(prediction.branches.size());

    for (std::size_t branch_index = 0U;
         branch_index < prediction.branches.size();
         ++branch_index) {
      PredictionBranchInteraction interaction;
      interaction.object_id = prediction.id;
      interaction.branch_index = branch_index;
      interaction.point_correction_hints.resize(
          prediction.branches[branch_index].points.size());
      interactions.push_back(interaction);
    }
  }

  if (!config_.enabled) {
    return result;
  }

  evaluatePairCosts(inputs, predictions, config_, result);
  evaluateMapCosts(predictions, map_query, config_, result);
  evaluateCorridorCosts(predictions, map_query, config_, result);
  finalizeInteractions(config_, result);

  return result;
}

}  // namespace ldopcore
