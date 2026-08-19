#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <ldop/DynamicObjectPredictionArray.h>
#include <ldop/dynamic_object_tracker.h>
#include <ldop/multi_model_kalman_filter.h>
#include <ldop/prediction_interaction_context.h>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

namespace ldopcore {

struct PredictionFeedbackParams {
  bool enabled{true};                         // P4 主开关；关闭后保持 P3 行为。
  double max_age{1.5};                        // 超过该时间差的缓存不再用于反馈，单位 s。
  double probability_gain{1.0};               // likelihood bias 对下一轮 mode score 的影响。
  double noise_gain{0.25};                    // NIS 偏离期望值时过程噪声尺度的调整速度。
  double max_noise_scale{3.0};                // predictor 未来 Q 的最大闭环放大尺度。
  double smoothing_alpha{0.35};               // NIS/likelihood 的指数平滑权重。
  double behavior_value_bucket{0.25};         // 按行为强度近似匹配分支语义。
  double max_sample_time_error{0.15};         // 预测采样点与当前 dt 的最大允许差，单位 s。
};

struct PredictionFeedbackConfig {
  bool enabled{true};
  double max_age{1.5};
  double probability_gain{1.0};
  double noise_gain{0.25};
  double max_noise_scale{3.0};
  double smoothing_alpha{0.35};
  double behavior_value_bucket{0.25};
  double max_sample_time_error{0.15};
};

struct PredictionFeedbackDiagnostics {
  std::size_t feedback_evaluated_object_count{0U};
  std::size_t feedback_updated_branch_count{0U};
  std::size_t feedback_skipped_object_count{0U};
  std::size_t feedback_skipped_branch_count{0U};
  std::size_t expired_state_count{0U};
  double max_nis{0.0};
  double mean_nis{0.0};
  // NIS 只能说明归一化残差是否异常；现场判断低 NIS 原因时，还需要同时看 raw residual 和协方差尺度。
  double mean_residual_norm{0.0};
  double max_residual_norm{0.0};
  double mean_predicted_position_covariance_trace{0.0};
  double mean_observed_position_covariance_trace{0.0};
  double mean_innovation_covariance_trace{0.0};
  double min_process_noise_scale{1.0};
  double max_process_noise_scale{1.0};
  double max_abs_probability_bias{0.0};
};

struct CorridorOscillationParams {
  bool enabled{false};
  int min_samples{12};
  int max_history_samples{80};
  double min_motion_span{0.35};
  double max_orthogonal_variance_ratio{0.15};
  double reversal_velocity_epsilon{0.04};
  double min_half_period{0.3};
  double max_half_period{4.0};
  double wall_query_radius{1.5};
  int wall_query_max_results{4096};
  double wall_vertical_window{0.5};
  double wall_min_outside_gap{0.05};
  double wall_clearance_margin{0.08};
};

struct CorridorOscillationConfig {
  bool enabled{false};
  std::size_t min_samples{12U};
  std::size_t max_history_samples{80U};
  double min_motion_span{0.35};
  double max_orthogonal_variance_ratio{0.15};
  double reversal_velocity_epsilon{0.04};
  double min_half_period{0.3};
  double max_half_period{4.0};
  double wall_query_radius{1.5};
  std::size_t wall_query_max_results{4096U};
  double wall_vertical_window{0.5};
  double wall_min_outside_gap{0.05};
  double wall_clearance_margin{0.08};
};

// ROS 参数快照；predictor 和 tracker 各自读取，不共享同一个实例。
struct DynamicObjectPredictorParams {
  double prediction_horizon{1.0};
  int max_branches{6};                         // >=1，限制每个目标的 GMM component 数量。
  double min_branch_probability{0.03};         // [0,1)，过滤极低概率分支后重新归一化。
  int history_window{10};                      // >=1，历史特征估计使用的最近样本数。
  double heading_stable_angle{0.35};           // >0，单位 rad，航向变化小于该值时认为趋势稳定。
  double stop_speed{0.2};                      // >=0，停止分支趋近的速度尺度。
  double turn_yaw_rate{0.5};                   // >0，转向假设的角速度幅值。
  double lateral_speed{0.4};                   // >=0，人类横向避让分支的法向速度幅值。
  double vertical_speed{0.3};                  // >=0，UAV 垂向分支的 z 速度幅值。
  double history_quality_noise_scale{1.5};     // >=1，短历史/漏检/不稳定轨迹的未来噪声放大上限。
  double hypothesis_noise_scale{1.3};          // >=1，假设性更强 mode 的未来噪声放大尺度。
  CorridorOscillationParams corridor_oscillation;
  PredictionFeedbackParams feedback;
  MultiModelKalmanFilterParams filter;
  PredictionInteractionParams interaction;
};

// 运行时配置；构造阶段由 ROS 参数快照转换而来。
struct DynamicObjectPredictorConfig {
  double prediction_horizon{1.0};
  std::size_t max_branches{6U};
  double min_branch_probability{0.03};
  std::size_t history_window{10U};
  double heading_stable_angle{0.35};
  double stop_speed{0.2};
  double turn_yaw_rate{0.5};
  double lateral_speed{0.4};
  double vertical_speed{0.3};
  double history_quality_noise_scale{1.5};
  double hypothesis_noise_scale{1.3};
  CorridorOscillationConfig corridor_oscillation_config;
  PredictionFeedbackConfig feedback_config;
  MultiModelKalmanFilterConfig filter_config;
  PredictionInteractionConfig interaction_config;
};

struct DynamicObjectPredictorTimingStats {
  double predict_dynamic_objects_ms{0.0};
  double apply_interaction_context_ms{0.0};
  double build_prediction_markers_ms{0.0};
  double predict_total_ms{0.0};
};

// 帧级预测结果：包含预测消息、预测 marker 和耗时统计。
struct DynamicObjectPredictorFrameResult {
  ldop::DynamicObjectPredictionArray predictions_msg;
  visualization_msgs::MarkerArray prediction_markers_msg;
  PredictionInteractionDiagnostics interaction_diagnostics;
  PredictionFeedbackDiagnostics feedback_diagnostics;
  std::size_t corridor_oscillation_object_count{0U};
  DynamicObjectPredictorTimingStats timing;
};

class DynamicObjectPredictor {
 public:
  // 从 ROS 参数服务器读取预测参数，并在模块内部构建运行时配置。
  explicit DynamicObjectPredictor(ros::NodeHandle& pnh, bool verbose = false);
  // 直接使用外部配置构造，用于测试。
  explicit DynamicObjectPredictor(const DynamicObjectPredictorConfig& config,
                                  bool verbose = false);

  // 预测模块主流程：对 tracker 输出的稳定目标做轨迹外推，返回预测消息和 marker。
  DynamicObjectPredictorFrameResult predict(
      const std_msgs::Header& header,
      const std::vector<TrackPredictionInput>& inputs,
      const PredictionMapQuery* map_query = nullptr);

 private:
  // P4 内部缓存只服务同一 predictor 实例的短期闭环，不作为 ROS 消息或跨模块契约。
  struct PredictionFeedbackBranchKey {
    std::uint8_t behavior_type{ldop::DynamicObjectPredictionBranch::BEHAVIOR_LONGITUDINAL};
    int behavior_value_bucket{0};

    bool operator==(const PredictionFeedbackBranchKey& other) const {
      return behavior_type == other.behavior_type &&
             behavior_value_bucket == other.behavior_value_bucket;
    }
  };

  struct PredictionFeedbackBranchKeyHash {
    std::size_t operator()(const PredictionFeedbackBranchKey& key) const {
      return (static_cast<std::size_t>(key.behavior_type) << 32U) ^
             static_cast<std::size_t>(key.behavior_value_bucket);
    }
  };

  struct PredictionFeedbackSampleSnapshot {
    double time_from_start{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d position_covariance{Eigen::Matrix3d::Identity()};
  };

  struct PredictionFeedbackBranchSnapshot {
    PredictionFeedbackBranchKey key;
    std::vector<PredictionFeedbackSampleSnapshot> samples;
  };

  struct PredictionFeedbackEvaluation {
    double nis{0.0};
    double log_likelihood{0.0};
    double residual_norm{0.0};
    double predicted_position_covariance_trace{0.0};
    double observed_position_covariance_trace{0.0};
    double innovation_covariance_trace{0.0};
  };

  struct PredictionFeedbackBranchState {
    double smoothed_log_likelihood{0.0};
    double smoothed_nis{3.0};
    double probability_score_bias{0.0};
    double process_noise_scale{1.0};
    bool initialized{false};
  };

  struct PredictionFeedbackObjectState {
    ros::Time last_stamp;
    std::vector<PredictionFeedbackBranchSnapshot> last_prediction;
    std::unordered_map<PredictionFeedbackBranchKey,
                       PredictionFeedbackBranchState,
                       PredictionFeedbackBranchKeyHash>
        branch_states;
  };

  struct PredictionFeedbackModeHint {
    double score_bias{0.0};
    double process_noise_scale{1.0};
    bool has_feedback{false};
  };

  using PredictionFeedbackModeHintMap =
      std::unordered_map<PredictionFeedbackBranchKey,
                         PredictionFeedbackModeHint,
                         PredictionFeedbackBranchKeyHash>;

  void loadParameters();

  // 构建预测可视化 marker：只显示真实 GMM 分支的均值轨迹线，避免额外抽样线误导调试。
  visualization_msgs::MarkerArray buildPredictionMarkers(
      const std_msgs::Header& header,
      const ldop::DynamicObjectPredictionArray& predictions) const;

  PredictionFeedbackBranchKey makeFeedbackBranchKey(std::uint8_t behavior_type,
                                                    double behavior_value) const;
  void storeFeedbackSnapshots(
      const std_msgs::Header& header,
      const ldop::DynamicObjectPredictionArray& predictions);
  void cleanupExpiredFeedbackStates(const ros::Time& stamp,
                                    const std::vector<TrackPredictionInput>& inputs,
                                    PredictionFeedbackDiagnostics& diagnostics);
  const PredictionFeedbackSampleSnapshot* closestFeedbackSample(
      const PredictionFeedbackBranchSnapshot& branch,
      double dt) const;
  std::optional<PredictionFeedbackEvaluation> computeFeedbackEvaluation(
      const Eigen::Vector3d& observed_position,
      const Eigen::Matrix3d& observed_covariance,
      const PredictionFeedbackSampleSnapshot& sample) const;
  double smoothValue(double previous,
                     double observed,
                     double alpha,
                     bool initialized) const;
  std::unordered_map<std::uint32_t, PredictionFeedbackModeHintMap> buildFeedbackHints(
      const std_msgs::Header& header,
      const std::vector<TrackPredictionInput>& inputs,
      PredictionFeedbackDiagnostics& diagnostics);

  ros::NodeHandle pnh_;
  DynamicObjectPredictorParams params_;
  DynamicObjectPredictorConfig config_;
  bool verbose_{false};
  // P4 闭环反馈会在 predict() 内读写该缓存，因此入口保持有状态流程。
  std::unordered_map<std::uint32_t, PredictionFeedbackObjectState> feedback_states_;
};

// 从 ROS 参数快照构建运行时配置。
DynamicObjectPredictorConfig buildPredictorConfig(
    const DynamicObjectPredictorParams& params);

}  // namespace ldopcore
