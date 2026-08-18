#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

#include <ldop/DynamicObjectArray.h>
#include <ldop/multi_model_kalman_filter.h>
#include <ldop/utils.h>

namespace ldopcore {

struct TrackPredictionInput {
  // 该结构是 tracker 预留给未来 predictor 的内部快照，不新增预测话题。
  ros::Time stamp;
  std::uint32_t id{0U};
  ObjectClass object_class{ObjectClass::Unknown};
  MotionModelType motion_model_type{MotionModelType::CV3D};
  BoundingBox3D bbox;
  // 保留当前运动模型自己的完整状态和协方差；未来预测按 motion_model_type 解释，
  // 当前帧位置、速度和不确定性不重复存一份公共投影，避免同一状态在边界里出现两套来源。
  Eigen::VectorXd model_state;
  Eigen::MatrixXd model_covariance;
  std::vector<TrackHistorySample> history;
  // 生命周期信息给未来预测模块判断轨迹质量：短历史、漏检延续和稳定轨迹应有不同权重。
  std::size_t age{0U};
  std::size_t hits{0U};
  std::size_t missed_frames{0U};
  bool matched_in_current_frame{false};
  // 通道候选只使用当前观测，不应送入未来 rollout；provisional 轨迹可按独立参数提前发布。
  bool corridor_realtime_only{false};
  bool corridor_provisional{false};
};

// 跟踪模块内部使用的运行时配置：构造阶段由 ROS 参数快照转换而来。
struct DynamicObjectTrackerConfig {
  // 新轨迹先用最稳妥的 CV3D 承接未知类别，等证据足够后再切到类别模型。
  MotionModelType model_type{MotionModelType::CV3D};
  // 传给每条 TrackState 内部 Kalman filter 的滤波参数。
  MultiModelKalmanFilterConfig filter_config{};
  // 数据关联使用的卡方 gate 阈值，已经由置信度转换为 3 自由度马氏距离平方阈值。
  double association_gate_threshold{11.34};
  // 漏检轨迹重关联时的 gate 放宽倍数，用于给 coasting 目标更大的回收窗口。
  double coasting_gate_relax_factor{1.5};
  // bbox 尺寸只做指数平滑，中心位置仍以滤波器状态为准。
  double box_size_smoothing_alpha{0.5};
  // 马氏距离 gate 失败后的回退关联先看三维中心距离：只有距离判据先通过，才会继续看 IoU；
  // 参数必须大于 0，避免把回退关联意外关掉后导致旋转中心附近反复裂出新 ID。
  double spawn_suppression_distance{0.5};
  // IoU 是距离判据通过后的第二层回退条件，不是与距离并列抢先判断的独立入口。
  // 参数范围为 (0,1]；未匹配 detection 的建轨抑制同样复用该阈值。
  double spawn_suppression_iou_threshold{0.5};
  double duplicate_merge_distance{0.45};
  // 单帧预测 dt 上限，避免时间戳跳变把轨迹一次性外推过远。
  double max_dt{0.5};
  double invalid_cost{1e9}; // 数据关联中表示无效匹配的成本值，必须大于 gate 阈值。
  std::size_t max_coast_frames{5U};
  // 内部轨迹比对外发布多保留一段时间，用于目标短时断帧后恢复原 ID。
  std::size_t max_publish_missed_frames{8U};
  std::size_t max_history_size{100U};
  std::size_t min_hits_to_publish{3U};
  // 保持默认关闭，避免改变通道外目标和旧配置的发布语义。
  bool publish_corridor_provisional{false};
  std::size_t corridor_provisional_min_hits{2U};
  std::size_t corridor_provisional_max_hits{4U};
  std::size_t corridor_realtime_max_publish_missed_frames{3U};
  // realtime 关联只看最后真实观测，避免通用 KF/fallback gate 把不同候选拼成一条轨迹。
  double corridor_realtime_association_gate{0.25};
  double corridor_realtime_association_gate_max{0.35};
  double corridor_realtime_association_gate_missed_increment{0.05};
  // 当前通道场景只允许一个 realtime 目标进入规划器，避免旧 coast 与新
  // provisional 候选同时占用输出并制造短命 ID 洪泛。
  std::size_t max_corridor_realtime_tracks{1U};
  double motion_min_displacement{0.04};
  std::size_t motion_min_evidence_frames{2U};
  double motion_confirmation_speed{0.12};
  bool classification_enabled{true};
  std::size_t classification_start_frame{10U};
  double classification_score_decay{0.85};
  double classification_score_increment{1.0};
  double classification_confirm_score{3.0};
  double classification_switch_margin{1.5};
  double classification_size_change_ratio{0.3};
  double classification_point_count_change_ratio{0.3};
  std::size_t classification_size_change_confirm_frames{5U};
  std::vector<double> classify_human_threshold{1.1, 0.8};
  std::vector<double> classify_vehicle_threshold{1.5, 0.7};
  std::vector<double> classify_uav_threshold{0.6, 1.5};
};

// ROS 参数读取后的原始参数快照；默认值集中写在这里，运行时配置再由 tracker 构建。
struct DynamicObjectTrackerParams {
  bool classification_enabled{true};
  int history_size{100};                                // 大于0
  int max_missed_frames{15};                            // 大于0，内部轨迹保留帧数
  int max_publish_missed_frames{8};                     // 大于等于0，对外保留预测目标的最多漏检帧数
  int min_hits_to_publish{3};                           // 大于0，且小于 history_size，轨迹发布的最小匹配次数
  bool publish_corridor_provisional{false};              // 仅对 corridor_provisional 轨迹生效
  int corridor_provisional_min_hits{2};                  // 通道候选提前发布的最小命中次数
  int corridor_provisional_max_hits{4};                  // 未确认候选最多对外发布的命中次数
  int corridor_realtime_max_publish_missed_frames{3};    // 通道实时目标只按最后观测保留的漏检帧数
  double corridor_realtime_association_gate{0.25};        // 通道实时目标无漏帧时的最后观测 gate，单位m
  double corridor_realtime_association_gate_max{0.35};    // 短时漏帧放宽后的最大 gate，单位m
  double corridor_realtime_association_gate_missed_increment{0.05};  // 每个漏帧增加的 gate，单位m
  int max_corridor_realtime_tracks{1};                    // 当前通道最多输出的 realtime 轨迹数
  double motion_min_displacement{0.04};                 // >0，连续匹配被视为真实运动的最小中心位移，单位m
  int motion_min_evidence_frames{2};                    // >0，对外发布前至少需要的运动证据帧数
  double motion_confirmation_speed{0.12};               // >0，滤波速度达到该值后锁定为动态轨迹，单位m/s
  double association_gate_confidence{0.7};              // (0,1) ，且由卡方分布分位数转换为门限值，不能直接等同于门限值。
  double coasting_gate_relax_factor{1.5};               // >=1.0
  double box_size_smoothing_alpha{0.5};                 // [0,1]
  double spawn_suppression_distance{0.5};               // >0
  double spawn_suppression_iou_threshold{0.5};          // (0,1]
  double duplicate_merge_distance{0.45};                // >0 启用，<=0 关闭
  double default_dt{0.1};                               // >0, 用于初始预测和时间戳异常时的 fallback
  double max_dt{0.5};                                   // >0, 预测时的 dt 上限，避免时间跳变导致状态外推过远
  int classification_start_frame{10};                   // >0, 轨迹达到该帧数后开始尝试分类，避免过早分类导致频繁切换
  double classification_score_decay{0.85};              // (0,1), 分类得分衰减因子，新轨迹的分数都是0
  double classification_score_increment{1.0};           // >0, 分类得分增量
  double classification_confirm_score{3.0};             // >0, 分类确认时的得分阈值，且需要大于 classification_switch_margin 才能切换类别
  double classification_switch_margin{1.5};             // >0, 分类切换的领先得分边际
  // 得分公式：上一帧分数 * classification_score_decay + classification_score_increment(当前观测是否判断为该类别，即四个类别中只有一个等加分)
  double classification_size_change_ratio{0.3};         // [0,1], 尺寸变化比率阈值
  double classification_point_count_change_ratio{0.3};  // [0,1], 点数变化比率阈值
  int classification_size_change_confirm_frames{5};     // >0, 尺寸变化确认帧数，避免短时遮挡或残片导致的尺寸骤变误触发分类切换
  std::vector<double> classify_human_threshold{1.1, 0.8};
  std::vector<double> classify_vehicle_threshold{1.5, 0.7};
  std::vector<double> classify_uav_threshold{0.6, 1.5};
  // 卡尔曼滤波器参数统一放在共享结构中；tracker 和 predictor 各自读取，再通过 buildMultiModelKalmanFilterConfig 转换。
  MultiModelKalmanFilterParams filter;
};

struct DynamicObjectTrackerTimingStats {
  // 单帧跟踪主流程总耗时，单位为毫秒。
  double process_dynamic_tracks_ms{0.0};
  // 跟踪主流程分阶段耗时，单位为毫秒。
  double association_ms{0.0};
  double update_tracks_ms{0.0};
  double build_output_ms{0.0};
  double build_track_markers_ms{0.0};
};

struct DynamicObjectTrackerFrameResult {
  ldop::DynamicObjectArray dynamic_objects_msg;
  // predictor 预留输入只在 C++ 边界内流转；当前不新增预测话题。
  std::vector<TrackPredictionInput> prediction_inputs;
  visualization_msgs::MarkerArray dynamic_track_markers_msg;
  DynamicObjectTrackerTimingStats timing;
};

// 单条轨迹的最小运行状态：稳定 ID、滤波器、bbox、生命周期计数和调试历史。
struct TrackState {
  TrackState(std::uint32_t track_id,
             MotionModelType track_model_type,
             const MultiModelKalmanFilterConfig& filter_config);
  ~TrackState();

  TrackState(const TrackState&) = delete;
  TrackState& operator=(const TrackState&) = delete;
  TrackState(TrackState&&) noexcept;
  TrackState& operator=(TrackState&&) noexcept;

  std::uint32_t id{0U};
  MotionModelType model_type{MotionModelType::CV3D};
  // 每条轨迹独占一个 KF/EKF 实例；基类接口对 tracker 隐藏具体运动模型。
  std::shared_ptr<KalmanFilterBase> filter;
  // bbox.center 会被滤波器位置覆盖，bbox.size 保留检测框尺寸并做平滑。
  BoundingBox3D bbox;
  std::vector<TrackHistorySample> history;
  // age 表示轨迹经历的总帧数，hits 表示成功关联次数，missed_frames 表示连续漏检帧数。
  std::size_t age{0U};
  std::size_t hits{0U};
  std::size_t missed_frames{0U};
  std::size_t motion_evidence_frames{0U};
  bool motion_confirmed{false};
  bool corridor_realtime_only{false};
  bool corridor_provisional{false};
  // 通道实时目标对外使用最后一次原始点簇质心，不用 KF 平滑中心替代当前观测。
  geometry_msgs::Point last_observed_center;
  bool has_last_observed_center{false};
  // 上次处理该轨迹的时间戳，用于下一帧计算预测 dt。
  ros::Time last_stamp;
  // 对外类别和内部证据分开：新轨迹先保持 Unknown，避免把 CV3D 默认模型误报为 Other。
  ObjectClass object_class{ObjectClass::Unknown};
  // 只给 Human/Vehicle/UAV/Other 四个明确类别积分；Unknown 表示证据尚未达标。
  std::array<double, 4U> classification_scores{0.0, 0.0, 0.0, 0.0};
  // 这里的 max_observed_size 是分类用的历史稳定尺寸，不是轨迹 history 的容量。
  geometry_msgs::Vector3 max_observed_size;
  std::size_t last_point_count{0U};
  std::size_t large_size_counter{0U};
  std::size_t small_size_counter{0U};
};

class DynamicObjectTracker {
 public:
  // 构造时直接从 ROS 参数服务器读取跟踪参数，并在模块内部构建运行时配置。
  explicit DynamicObjectTracker(ros::NodeHandle& pnh, bool verbose = false);

  // 跟踪模块自己的主流程：顺序执行关联、状态维护、输出与可视化构建。
  DynamicObjectTrackerFrameResult processDynamicTracks(
      const std_msgs::Header& header,
      const std::vector<DynamicObjectDetection>& detections);

 private:
  // 构造阶段集中读取 tracker 参数并构建运行时配置，保持构造函数主体简洁。
  void loadParameters();

  // 历史线和轨迹头 marker 只在模块内部生成，避免测试代码反向塑形公共接口。
  visualization_msgs::MarkerArray buildTrackMarkers(const std_msgs::Header& header) const;
  void predictTracks(const ros::Time& stamp);
  void updateMatchedTrack(TrackState& track,
                          const DynamicObjectDetection& detection,
                          const ros::Time& stamp);
  geometry_msgs::Vector3 prepareMatchedTrackForUpdate(TrackState& track,
                                                      const DynamicObjectDetection& detection,
                                                      std::size_t matched_hits);
  void switchTrackModel(TrackState& track, ObjectClass target_class);
  // 未匹配且未被 spawn suppression 挡住的 detection 才会创建新稳定 ID。
  void createTrack(const DynamicObjectDetection& detection, const ros::Time& stamp);
  // 漏检轨迹继续按预测状态维护，给后续帧重关联留窗口。
  void coastTrack(TrackState& track, const ros::Time& stamp);
  void mergeDuplicateTracks();
  void limitCorridorRealtimeTracks();
  void deleteExpiredTracks();
  // 对外只发布命中次数达到阈值的稳定轨迹，减少一帧噪声 detection 直接暴露给下游。
  ldop::DynamicObjectArray buildOutput(const std_msgs::Header& header) const;
  // 与 buildOutput 使用同一稳定轨迹集合，为未来 predictor 提供比 ROS 消息更完整的内部快照。
  std::vector<TrackPredictionInput> buildPredictionInputs(const std_msgs::Header& header) const;

  ros::NodeHandle pnh_;
  DynamicObjectTrackerConfig config_;
  DynamicObjectTrackerParams params_;
  std::vector<TrackState> tracks_;
  std::uint32_t next_track_id_{0U};
  bool verbose_{false};
};

}  // namespace ldopcore
