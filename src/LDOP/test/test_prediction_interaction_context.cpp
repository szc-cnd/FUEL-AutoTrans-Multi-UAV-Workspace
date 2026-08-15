#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <ldop/prediction_interaction_context.h>
#include <ros/ros.h>

namespace ldopcore {
namespace {

class FakePredictionMapQuery final : public PredictionMapQuery {
 public:
  void setAvailable(const bool available) { available_ = available; }

  void setAllSeenFree(const bool all_seen_free) { all_seen_free_ = all_seen_free; }

  void setFreeXThreshold(const double threshold) { free_x_threshold_ = threshold; }

  void addOccupiedNode(const Eigen::Vector3d& center,
                       const double voxel_size,
                       const std::uint32_t hits) {
    PredictionLocalOccupiedNode node;
    node.center = center;
    node.voxel_size = voxel_size;
    node.hits = hits;
    occupied_nodes_.push_back(node);
  }

  bool available() const override { return available_; }

  std::optional<PredictionMapNodeQuery> queryNode(
      const Eigen::Vector3d& point) const override {
    ++query_node_count_;
    if (!available_) {
      return std::nullopt;
    }
    for (const auto& occupied : occupied_nodes_) {
      if ((occupied.center - point).norm() <= occupied.voxel_size * 0.5) {
        PredictionMapNodeQuery node;
        node.exists = true;
        node.occupied = true;
        node.center = occupied.center;
        node.voxel_size = occupied.voxel_size;
        return node;
      }
    }
    PredictionMapNodeQuery node;
    node.exists = true;
    node.occupied = false;
    node.center = point;
    node.voxel_size = 0.2;
    return node;
  }

  bool seenFree(const Eigen::Vector3d& point) const override {
    ++seen_free_count_;
    if (!available_) {
      return false;
    }
    if (all_seen_free_) {
      return true;
    }
    return point.x() > free_x_threshold_;
  }

  std::vector<PredictionLocalOccupiedNode> queryLocalOccupied(
      const Eigen::Vector3d& center,
      const double radius,
      const std::size_t max_results) const override {
    ++local_occupied_query_count_;
    std::vector<PredictionLocalOccupiedNode> nodes;
    if (!available_) {
      return nodes;
    }
    for (const auto& occupied : occupied_nodes_) {
      if ((occupied.center - center).norm() <= radius + occupied.voxel_size * 0.5) {
        nodes.push_back(occupied);
        if (max_results > 0U && nodes.size() >= max_results) {
          return nodes;
        }
      }
    }
    return nodes;
  }

  std::vector<PredictionLocalOccupiedNode> queryLocalOccupied(
      const Eigen::Vector3d& center,
      const double radius,
      const std::size_t max_results,
      const std::uint8_t query_depth) const override {
    last_local_occupied_query_depth_ = query_depth;
    return queryLocalOccupied(center, radius, max_results);
  }

  std::size_t queryNodeCount() const { return query_node_count_; }

  std::size_t seenFreeCount() const { return seen_free_count_; }

  std::size_t localOccupiedQueryCount() const {
    return local_occupied_query_count_;
  }

  std::uint8_t lastLocalOccupiedQueryDepth() const {
    return last_local_occupied_query_depth_;
  }

 private:
  bool available_{true};
  bool all_seen_free_{false};
  double free_x_threshold_{0.0};
  std::vector<PredictionLocalOccupiedNode> occupied_nodes_;
  mutable std::size_t query_node_count_{0U};
  mutable std::size_t seen_free_count_{0U};
  mutable std::size_t local_occupied_query_count_{0U};
  mutable std::uint8_t last_local_occupied_query_depth_{0U};
};

[[maybe_unused]] TrackPredictionInput makeInput(const std::uint32_t id,
                                                const Eigen::Vector3d& position,
                                                const Eigen::Vector3d& velocity) {
  TrackPredictionInput input;
  input.id = id;
  input.object_class = ObjectClass::Human;
  input.motion_model_type = MotionModelType::CV3D;
  input.bbox.size.x = 0.6;
  input.bbox.size.y = 0.6;
  input.bbox.size.z = 1.5;
  input.model_state = Eigen::VectorXd::Zero(6);
  input.model_state.segment<3>(0) = position;
  input.model_state.segment<3>(3) = velocity;
  input.model_covariance = 0.1 * Eigen::MatrixXd::Identity(6, 6);
  input.matched_in_current_frame = true;
  input.hits = 8U;
  input.age = 8U;
  return input;
}

ldop::DynamicObjectPredictionBranch makeBranch(
    const std::vector<Eigen::Vector3d>& positions,
    const double probability = 1.0) {
  ldop::DynamicObjectPredictionBranch branch;
  branch.behavior_type = ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL;
  branch.behavior_value = 0.0;
  branch.probability = probability;
  for (std::size_t index = 0U; index < positions.size(); ++index) {
    ldop::DynamicObjectPredictionPoint point;
    point.time_from_start = ros::Duration(0.1 * static_cast<double>(index + 1U));
    point.model_state = {positions[index].x(), positions[index].y(), positions[index].z(),
                         1.0, 0.0, 0.0};
    point.model_covariance = std::vector<double>(36U, 0.0);
    point.influence_weight = 1.0;
    branch.points.push_back(point);
  }
  return branch;
}

ldop::DynamicObjectPrediction makePrediction(
    const std::uint32_t id,
    const std::vector<ldop::DynamicObjectPredictionBranch>& branches,
    const double size_x = 0.6,
    const double size_y = 0.6) {
  ldop::DynamicObjectPrediction prediction;
  prediction.id = id;
  prediction.size.x = size_x;
  prediction.size.y = size_y;
  prediction.size.z = 1.5;
  prediction.motion_model_type = ldop::DynamicObjectPrediction::MOTION_MODEL_CV3D;
  prediction.object_class = ldop::DynamicObjectPrediction::CLASS_HUMAN;
  prediction.matched_in_current_frame = true;
  prediction.branches = branches;
  return prediction;
}

ldop::DynamicObjectPredictionArray makePredictionArray(
    const std::vector<ldop::DynamicObjectPrediction>& predictions) {
  ldop::DynamicObjectPredictionArray array;
  array.header.frame_id = "map";
  array.predictions = predictions;
  return array;
}

ldop::DynamicObjectPredictionArray makePredictions(
    const std::vector<std::pair<std::uint32_t, ldop::DynamicObjectPredictionBranch>>& branches) {
  std::vector<ldop::DynamicObjectPrediction> predictions;
  for (const auto& [id, branch] : branches) {
    predictions.push_back(makePrediction(id, {branch}));
  }
  return makePredictionArray(predictions);
}

const PredictionBranchInteraction& firstInteraction(
    const PredictionInteractionResult& result,
    const std::size_t prediction_index = 0U,
    const std::size_t branch_index = 0U) {
  return result.interactions_by_prediction.at(prediction_index).at(branch_index);
}

TEST(PredictionInteractionContextTest, DisabledContextReturnsIdentityScales) {
  PredictionInteractionParams params;
  params.enabled = false;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  const auto predictions = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.1, 0.0, 0.0)})},
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);

  ASSERT_EQ(result.interactions_by_prediction.size(), 1U);
  ASSERT_EQ(result.interactions_by_prediction.front().size(), 1U);
  EXPECT_DOUBLE_EQ(result.interactions_by_prediction.front().front().probability_scale, 1.0);
  EXPECT_DOUBLE_EQ(result.interactions_by_prediction.front().front().total_energy, 0.0);
  EXPECT_EQ(result.interactions_by_prediction.front().front().risk_level,
            PredictionBranchRiskLevel::Low);
}

TEST(PredictionInteractionContextTest, CrossingBranchesGetHigherPairCostThanSeparatedBranches) {
  PredictionInteractionParams params;
  params.interaction_safe_distance = 0.35;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;

  const auto crossing = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(-1.0, 0.0, 0.0),
                       Eigen::Vector3d(-0.35, 0.0, 0.0),
                       Eigen::Vector3d(0.0, 0.0, 0.0),
                       Eigen::Vector3d(0.35, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(0.0, -1.0, 0.0),
                       Eigen::Vector3d(0.0, -0.35, 0.0),
                       Eigen::Vector3d(0.0, 0.0, 0.0),
                       Eigen::Vector3d(0.0, 0.35, 0.0)})},
  });
  const auto separated = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(-1.0, 0.0, 0.0),
                       Eigen::Vector3d(-0.35, 0.0, 0.0),
                       Eigen::Vector3d(0.0, 0.0, 0.0),
                       Eigen::Vector3d(0.35, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(0.0, 3.0, 0.0),
                       Eigen::Vector3d(0.35, 3.0, 0.0),
                       Eigen::Vector3d(0.7, 3.0, 0.0),
                       Eigen::Vector3d(1.05, 3.0, 0.0)})},
  });

  const auto crossing_result = context.evaluate(crossing.header, {}, crossing, &map);
  const auto separated_result = context.evaluate(separated.header, {}, separated, &map);

  EXPECT_GT(firstInteraction(crossing_result).pair_cost,
            firstInteraction(separated_result).pair_cost);
  EXPECT_LT(firstInteraction(crossing_result).probability_scale, 1.0);
}

TEST(PredictionInteractionContextTest, HeadOnBranchGetsTtcCostAndCorrectionHint) {
  PredictionInteractionParams params;
  params.ttc_safe_time = 1.5;
  params.max_deceleration_ratio = 0.4;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  const auto predictions = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(-1.0, 0.0, 0.0),
                       Eigen::Vector3d(-0.55, 0.0, 0.0),
                       Eigen::Vector3d(-0.1, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(1.0, 0.0, 0.0),
                       Eigen::Vector3d(0.55, 0.0, 0.0),
                       Eigen::Vector3d(0.1, 0.0, 0.0)})},
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);
  const auto& interaction = firstInteraction(result);

  EXPECT_GT(interaction.pair_cost, 0.0);
  EXPECT_LT(interaction.minimum_ttc, params.ttc_safe_time);
  EXPECT_EQ(interaction.risk_level, PredictionBranchRiskLevel::High);
  EXPECT_TRUE(interaction.correction_hint.has_deceleration ||
              interaction.correction_hint.has_position_offset);
  if (interaction.correction_hint.has_deceleration) {
    EXPECT_GT(interaction.correction_hint.deceleration_ratio, 0.0);
    EXPECT_LT(interaction.correction_hint.deceleration_ratio,
              params.max_deceleration_ratio);
    EXPECT_LE(interaction.correction_hint.deceleration_ratio,
              params.max_deceleration_ratio);
  }
}

TEST(PredictionInteractionContextTest, FollowingCostsLessThanHeadOn) {
  PredictionInteractionParams params;
  params.ttc_safe_time = 1.5;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  const auto head_on = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(-1.0, 0.0, 0.0),
                       Eigen::Vector3d(-0.55, 0.0, 0.0),
                       Eigen::Vector3d(-0.1, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(1.0, 0.0, 0.0),
                       Eigen::Vector3d(0.55, 0.0, 0.0),
                       Eigen::Vector3d(0.1, 0.0, 0.0)})},
  });
  const auto following = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.0, 0.0, 0.0),
                       Eigen::Vector3d(0.4, 0.0, 0.0),
                       Eigen::Vector3d(0.8, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(0.8, 0.0, 0.0),
                       Eigen::Vector3d(1.2, 0.0, 0.0),
                       Eigen::Vector3d(1.6, 0.0, 0.0)})},
  });

  const auto head_on_result = context.evaluate(head_on.header, {}, head_on, &map);
  const auto following_result = context.evaluate(following.header, {}, following, &map);

  EXPECT_LT(firstInteraction(following_result).pair_cost,
            firstInteraction(head_on_result).pair_cost);
}

TEST(PredictionInteractionContextTest, RecedingSeparatedBranchKeepsIdentityScale) {
  PredictionInteractionContext context(
      buildPredictionInteractionConfig(PredictionInteractionParams{}));
  FakePredictionMapQuery map;
  const std::vector<TrackPredictionInput> inputs{
      makeInput(1U, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(4.0, 0.0, 0.0)),
      makeInput(2U, Eigen::Vector3d(-1.5, 0.0, 0.0), Eigen::Vector3d(-4.0, 0.0, 0.0)),
  };
  const auto predictions = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.0, 0.0, 0.0),
                       Eigen::Vector3d(0.4, 0.0, 0.0),
                       Eigen::Vector3d(0.8, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(-1.5, 0.0, 0.0),
                       Eigen::Vector3d(-1.9, 0.0, 0.0),
                       Eigen::Vector3d(-2.3, 0.0, 0.0)})},
  });

  const auto result = context.evaluate(predictions.header, inputs, predictions, &map);
  const auto& interaction = firstInteraction(result);

  EXPECT_DOUBLE_EQ(interaction.pair_cost, 0.0);
  EXPECT_DOUBLE_EQ(interaction.probability_scale, 1.0);
  EXPECT_FALSE(interaction.correction_hint.has_deceleration);
}

TEST(PredictionInteractionContextTest, PositionOffsetPointsAwayFromConflictTarget) {
  PredictionInteractionParams params;
  params.max_correction_distance = 0.2;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  const auto predictions = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.0, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(0.5, 0.0, 0.0)})},
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);
  const auto& interaction = firstInteraction(result);

  ASSERT_TRUE(interaction.correction_hint.has_position_offset);
  EXPECT_LT(interaction.correction_hint.position_offset.x(), 0.0);
  EXPECT_GT(interaction.correction_hint.position_offset.norm(), 0.0);
  EXPECT_LE(interaction.correction_hint.position_offset.norm(), params.max_correction_distance);
}

TEST(PredictionInteractionContextTest, DuplicateOtherBranchesUseProbabilityExpectation) {
  PredictionInteractionContext context(
      buildPredictionInteractionConfig(PredictionInteractionParams{}));
  FakePredictionMapQuery map;
  const auto target_branch = makeBranch({
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(-0.55, 0.0, 0.0),
      Eigen::Vector3d(-0.1, 0.0, 0.0),
  });
  const auto risky_other_branch = makeBranch({
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.55, 0.0, 0.0),
      Eigen::Vector3d(0.1, 0.0, 0.0),
  });
  auto half_probability_branch = risky_other_branch;
  half_probability_branch.probability = 0.5;

  const auto single_branch_predictions = makePredictionArray({
      makePrediction(1U, {target_branch}),
      makePrediction(2U, {risky_other_branch}),
  });
  const auto duplicate_branch_predictions = makePredictionArray({
      makePrediction(1U, {target_branch}),
      makePrediction(2U, {half_probability_branch, half_probability_branch}),
  });

  const auto single_result =
      context.evaluate(single_branch_predictions.header, {}, single_branch_predictions, &map);
  const auto duplicate_result =
      context.evaluate(duplicate_branch_predictions.header, {}, duplicate_branch_predictions, &map);

  EXPECT_NEAR(firstInteraction(duplicate_result).pair_cost,
              firstInteraction(single_result).pair_cost,
              1.0e-9);
}

TEST(PredictionInteractionContextTest, ZeroProbabilityConflictBranchDoesNotLeakSideEffects) {
  PredictionInteractionParams params;
  params.ttc_safe_time = 1.5;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  const auto target_branch = makeBranch({
      Eigen::Vector3d(-1.0, 0.0, 0.0),
      Eigen::Vector3d(-0.55, 0.0, 0.0),
      Eigen::Vector3d(-0.1, 0.0, 0.0),
  });
  const auto safe_positive_branch = makeBranch({
      Eigen::Vector3d(-1.0, 3.0, 0.0),
      Eigen::Vector3d(-0.55, 3.0, 0.0),
      Eigen::Vector3d(-0.1, 3.0, 0.0),
  });
  const auto zero_probability_conflict_branch = makeBranch({
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.55, 0.0, 0.0),
      Eigen::Vector3d(0.1, 0.0, 0.0),
  },
                                                               0.0);
  const auto predictions = makePredictionArray({
      makePrediction(1U, {target_branch}),
      makePrediction(2U, {safe_positive_branch, zero_probability_conflict_branch}),
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);
  const auto& interaction = firstInteraction(result);

  EXPECT_DOUBLE_EQ(interaction.pair_cost, 0.0);
  EXPECT_DOUBLE_EQ(interaction.probability_scale, 1.0);
  EXPECT_FALSE(interaction.correction_hint.has_deceleration);
  EXPECT_FALSE(interaction.correction_hint.has_position_offset);
  EXPECT_FALSE(std::isfinite(interaction.minimum_ttc));
  EXPECT_GT(interaction.nearest_pair_distance, 2.0);
}

TEST(PredictionInteractionContextTest, OccupiedBranchGetsMapCostAndCollisionDiagnostic) {
  PredictionInteractionParams params;
  params.near_obstacle_distance = 0.6;
  params.max_correction_distance = 0.2;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  map.addOccupiedNode(Eigen::Vector3d(0.4, 0.0, 0.0), 0.4, 5U);
  const auto predictions = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.4, 0.0, 0.0)})},
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);
  const auto& interaction = firstInteraction(result);

  EXPECT_GT(interaction.map_cost, 0.0);
  EXPECT_TRUE(interaction.occupied_overlap);
  EXPECT_EQ(interaction.risk_level, PredictionBranchRiskLevel::Critical);
  EXPECT_EQ(result.diagnostics.colliding_branch_count, 1U);
  EXPECT_TRUE(interaction.correction_hint.has_position_offset);
}

TEST(PredictionInteractionContextTest, MapQueryDepthIsForwardedToOccupiedLookup) {
  PredictionInteractionParams params;
  params.map_query_depth = 2;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  map.addOccupiedNode(Eigen::Vector3d(0.4, 0.0, 0.0), 0.4, 5U);
  const auto predictions = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.4, 0.0, 0.0)})},
  });

  static_cast<void>(context.evaluate(predictions.header, {}, predictions, &map));

  EXPECT_GT(map.localOccupiedQueryCount(), 0U);
  EXPECT_EQ(map.lastLocalOccupiedQueryDepth(), 2U);
}

TEST(PredictionInteractionContextTest, NearObstacleCostsLessThanOccupied) {
  PredictionInteractionParams params;
  params.near_obstacle_distance = 0.7;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  map.addOccupiedNode(Eigen::Vector3d(0.0, 0.0, 0.0), 0.4, 5U);
  const auto predictions = makePredictionArray({
      makePrediction(1U, {makeBranch({Eigen::Vector3d(0.0, 0.0, 0.0)})}),
      makePrediction(2U, {makeBranch({Eigen::Vector3d(0.75, 0.0, 0.0)})}),
      makePrediction(3U, {makeBranch({Eigen::Vector3d(2.0, 0.0, 0.0)})}),
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);
  const auto& occupied = firstInteraction(result, 0U, 0U);
  const auto& near = firstInteraction(result, 1U, 0U);
  const auto& free = firstInteraction(result, 2U, 0U);

  EXPECT_GT(occupied.map_cost, near.map_cost);
  EXPECT_GT(near.map_cost, free.map_cost);
  EXPECT_TRUE(near.near_obstacle);
  EXPECT_FALSE(near.occupied_overlap);
  EXPECT_NE(near.risk_level, PredictionBranchRiskLevel::Low);
  EXPECT_NE(near.risk_level, PredictionBranchRiskLevel::Critical);
}

TEST(PredictionInteractionContextTest, RectangularObjectDiagonalCornerUsesCircumscribedRadius) {
  PredictionInteractionParams params;
  params.near_obstacle_distance = 0.35;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  map.addOccupiedNode(Eigen::Vector3d(0.72, 0.72, 0.0), 0.1, 5U);
  const auto predictions = makePredictionArray({
      makePrediction(1U, {makeBranch({Eigen::Vector3d(0.0, 0.0, 0.0)})}, 1.2, 1.2),
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);
  const auto& interaction = firstInteraction(result);

  EXPECT_GT(interaction.map_cost, 0.0);
  EXPECT_TRUE(interaction.occupied_overlap || interaction.near_obstacle);
  EXPECT_LT(interaction.nearest_obstacle_distance, params.near_obstacle_distance);
}

TEST(PredictionInteractionContextTest, FreeCorridorAlignedBranchGetsLowerCorridorCost) {
  PredictionInteractionParams params;
  params.free_corridor_query_radius = 1.0;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  map.setFreeXThreshold(0.0);
  map.addOccupiedNode(Eigen::Vector3d(0.70710678118, 0.70710678118, 0.0), 0.1, 3U);
  map.addOccupiedNode(Eigen::Vector3d(0.70710678118, -0.70710678118, 0.0), 0.1, 3U);
  const auto aligned = makeBranch({
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.4, 0.0, 0.0),
      Eigen::Vector3d(0.8, 0.0, 0.0),
  });
  const auto opposite = makeBranch({
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(-0.4, 0.0, 0.0),
      Eigen::Vector3d(-0.8, 0.0, 0.0),
  });
  const auto predictions = makePredictionArray({
      makePrediction(1U, {aligned, opposite}),
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);
  const auto& aligned_interaction = firstInteraction(result, 0U, 0U);
  const auto& opposite_interaction = firstInteraction(result, 0U, 1U);

  EXPECT_LT(aligned_interaction.corridor_cost, opposite_interaction.corridor_cost);
  EXPECT_GT(aligned_interaction.free_corridor_confidence, 0.0);
  EXPECT_TRUE(aligned_interaction.correction_hint.has_direction_bias);
}

TEST(PredictionInteractionContextTest, UniformFreeSpaceDoesNotCreateDirectionBias) {
  PredictionInteractionParams params;
  params.free_corridor_query_radius = 1.0;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  map.setAllSeenFree(true);
  const auto opposite = makeBranch({
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(-0.4, 0.0, 0.0),
      Eigen::Vector3d(-0.8, 0.0, 0.0),
  });
  const auto predictions = makePredictionArray({
      makePrediction(1U, {opposite}),
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);
  const auto& interaction = firstInteraction(result);

  EXPECT_DOUBLE_EQ(interaction.corridor_cost, 0.0);
  EXPECT_DOUBLE_EQ(interaction.free_corridor_confidence, 0.0);
  EXPECT_FALSE(interaction.correction_hint.has_direction_bias);
}

TEST(PredictionInteractionContextTest, UnavailableMapDisablesMapCostsOnly) {
  PredictionInteractionContext context(
      buildPredictionInteractionConfig(PredictionInteractionParams{}));
  FakePredictionMapQuery map;
  map.setAvailable(false);
  map.addOccupiedNode(Eigen::Vector3d(0.0, 0.0, 0.0), 0.4, 5U);
  const auto predictions = makePredictions({
      {1U, makeBranch({Eigen::Vector3d(0.0, 0.0, 0.0),
                       Eigen::Vector3d(0.4, 0.0, 0.0)})},
      {2U, makeBranch({Eigen::Vector3d(0.3, 0.0, 0.0),
                       Eigen::Vector3d(0.7, 0.0, 0.0)})},
  });

  const auto result = context.evaluate(predictions.header, {}, predictions, &map);
  const auto& interaction = firstInteraction(result);

  EXPECT_GT(interaction.pair_cost, 0.0);
  EXPECT_DOUBLE_EQ(interaction.map_cost, 0.0);
  EXPECT_DOUBLE_EQ(interaction.corridor_cost, 0.0);
  EXPECT_TRUE(result.diagnostics.map_unavailable);
}

TEST(PredictionInteractionContextTest, UsesBranchLevelMapQueryBudget) {
  PredictionInteractionParams params;
  params.corridor_interaction_weight = 0.4;
  params.free_corridor_query_radius = 1.0;
  PredictionInteractionContext context(buildPredictionInteractionConfig(params));
  FakePredictionMapQuery map;
  map.setAllSeenFree(true);

  std::vector<ldop::DynamicObjectPrediction> predictions;
  for (std::uint32_t id = 0U; id < 7U; ++id) {
    std::vector<ldop::DynamicObjectPredictionBranch> branches;
    for (std::size_t branch_index = 0U; branch_index < 3U; ++branch_index) {
      std::vector<Eigen::Vector3d> positions;
      for (std::size_t point_index = 0U; point_index < 10U; ++point_index) {
        positions.emplace_back(0.1 * static_cast<double>(point_index),
                               static_cast<double>(id) + 0.2 * static_cast<double>(branch_index),
                               0.0);
      }
      branches.push_back(makeBranch(positions, 1.0 / 3.0));
    }
    predictions.push_back(makePrediction(id, branches));
  }

  const auto prediction_array = makePredictionArray(predictions);
  const auto result = context.evaluate(prediction_array.header, {}, prediction_array, &map);

  ASSERT_EQ(result.interactions_by_prediction.size(), predictions.size());
  const std::size_t branch_count = predictions.size() * 3U;
  // 运行时预算要求地图查询跟分支数同阶，而不是跟 points * directions 同阶；
  // 当前目标是把 7 个目标的交互上下文稳定压到实时帧预算内。
  EXPECT_LE(map.localOccupiedQueryCount(), branch_count);
  EXPECT_LE(map.seenFreeCount(), branch_count * 8U);
  EXPECT_EQ(map.queryNodeCount(), 0U);
}

}  // namespace
}  // namespace ldopcore

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_prediction_interaction_context");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
