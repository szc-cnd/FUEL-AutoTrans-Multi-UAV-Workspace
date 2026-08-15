#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <ldop/FusedDynamicObjectArray.h>
#include <ldop/FusedDynamicObjectVirtualImuArray.h>
#include <ldop/utils.h>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

namespace ldopcore {

// ROS 参数读取后的原始快照。默认值集中写在结构体里，避免 loadParameters()
// 里散落另一份数字；运行时真正使用的有界值由 buildPoseFusionConfig() 生成。
struct DynamicObjectPoseFusionParams {
  std::vector<std::string> mocap_topics{"/mocap/target_0/pose"};  // 非空，顺序就是 mocap_source_index。
  std::vector<double> mocap_to_ldop_translation{0.0, 0.0, 0.0};   // T_LM 平移项，必须恰好 3 个分量。
  double mocap_sync_tolerance{0.03};                              // >0，LiDAR 帧关联的最近邻时间窗。
  double association_max_distance{1.0};                            // >0，LiDAR 质心到预测融合中心的距离门控。
  double offset_alpha{0.2};                                        // [0,1]，LiDAR offset 观测 EMA 系数。
  double offset_observation_max_norm{2.0};                         // >0，拒绝物理长度异常的 offset。
  double offset_update_max_delta{0.3};                             // >0，拒绝单帧 offset 突变。
  double lidar_anchor_timeout{0.5};                                // >0，超过该时间未刷新锚点则停止发布。
  int bspline_min_samples{8};                                      // 至少 5，插值型三次 B 样条的最小 pose-grid 点数。
  int bspline_future_samples{2};                                   // 固定延迟未来 pose-grid 样本数。
  double bspline_reset_translation_delta{0.2};                     // >0，offset 校正造成大跳变时重置样条窗口。
  double mocap_nominal_rate{50.0};                                 // >0，样条输入 pose grid 频率。
  double virtual_imu_rate_hz{200.0};                               // >0，虚拟 IMU 输出评估频率。
  double mocap_dt_jitter_tolerance{0.005};                         // >=0，原始 knot 间隔抖动门限。
  double gravity_z{-9.80665};                                      // LDOP/world frame 下的重力 z 分量。
  std::string fused_objects_topic{"/ldop/fused_dynamic_objects"};
  std::string virtual_imu_topic{"/ldop/fused_dynamic_object_virtual_imu"};
  std::string marker_topic{"/ldop/fused_dynamic_object_markers"};
};

// 运行时配置只做跨边界参数的简单范围兜底和类型转换；融合数学语义本身留在模块主流程。
struct DynamicObjectPoseFusionConfig {
  std::vector<std::string> mocap_topics;
  Eigen::Vector3d mocap_to_ldop_translation{Eigen::Vector3d::Zero()};
  double mocap_sync_tolerance{0.03};
  double association_max_distance{1.0};
  double offset_alpha{0.2};
  double offset_observation_max_norm{2.0};
  double offset_update_max_delta{0.3};
  double lidar_anchor_timeout{0.5};
  std::size_t bspline_min_samples{8U};
  std::size_t bspline_future_samples{2U};
  double bspline_reset_translation_delta{0.2};
  double mocap_nominal_rate{50.0};
  double virtual_imu_rate_hz{200.0};
  double mocap_dt_jitter_tolerance{0.005};
  double gravity_z{-9.80665};
  std::string fused_objects_topic{"/ldop/fused_dynamic_objects"};
  std::string virtual_imu_topic{"/ldop/fused_dynamic_object_virtual_imu"};
  std::string marker_topic{"/ldop/fused_dynamic_object_markers"};
};

// 模块入口返回的是可直接发布的快照；optional 为空表示这次事件没有对应输出。
struct DynamicObjectPoseFusionSnapshot {
  std::optional<ldop::FusedDynamicObjectArray> fused_objects;
  std::optional<ldop::FusedDynamicObjectVirtualImuArray> virtual_imu;
  std::optional<visualization_msgs::MarkerArray> markers;
};

DynamicObjectPoseFusionConfig buildPoseFusionConfig(
    const DynamicObjectPoseFusionParams& params);

class DynamicObjectPoseFusion {
 public:
  explicit DynamicObjectPoseFusion(ros::NodeHandle& pnh, bool verbose = false);
  explicit DynamicObjectPoseFusion(const DynamicObjectPoseFusionConfig& config,
                                   bool verbose = false);

  const std::vector<std::string>& mocapTopics() const;
  const std::string& fusedObjectsTopic() const;
  const std::string& virtualImuTopic() const;
  const std::string& markerTopic() const;

  DynamicObjectPoseFusionSnapshot processMocapPose(
      std::size_t mocap_source_index,
      const geometry_msgs::PoseStamped& pose_msg);

  DynamicObjectPoseFusionSnapshot updateLidarAnchors(
      const std_msgs::Header& header,
      const std::vector<DynamicObjectDetection>& detections);

 private:
  struct MocapPoseSample {
    ros::Time stamp;
    Eigen::Vector3d position_ldop{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation_ldop{Eigen::Quaterniond::Identity()};
  };

  struct SourceState {
    std::deque<MocapPoseSample> mocap_history;
    std::deque<MocapPoseSample> fused_pose_knots;
    std::deque<MocapPoseSample> pose_grid_knots;
    std::optional<ros::Time> last_published_virtual_imu_eval_stamp;
    std::optional<Eigen::Quaterniond> last_valid_quaternion;
    bool has_anchor{false};
    ros::Time anchor_stamp;
    MocapPoseSample anchor_mocap_pose_ldop;
    geometry_msgs::Pose anchor_fused_pose;
    Eigen::Vector3d offset_body{Eigen::Vector3d::Zero()};
    geometry_msgs::Vector3 latest_bbox_size;
    std::size_t latest_point_count{0U};
  };

  void loadParameters(ros::NodeHandle& pnh);
  void appendFusedPoseKnot(SourceState& source, const MocapPoseSample& fused_pose);
  void extendPoseGrid(SourceState& source);
  std::optional<MocapPoseSample> interpolatePoseKnot(
      const std::deque<MocapPoseSample>& knots,
      const ros::Time& stamp) const;
  std::optional<ldop::FusedDynamicObjectVirtualImuArray> buildVirtualImuArray(
      std::size_t mocap_source_index,
      SourceState& source,
      const std::string& frame_id,
      const ros::Time& available_stamp);
  visualization_msgs::MarkerArray buildMarkers(
      const ldop::FusedDynamicObjectArray& objects) const;

  mutable std::mutex mutex_;
  DynamicObjectPoseFusionParams params_;
  DynamicObjectPoseFusionConfig config_;
  std::vector<SourceState> sources_;
  bool verbose_{false};
};

}  // namespace ldopcore
