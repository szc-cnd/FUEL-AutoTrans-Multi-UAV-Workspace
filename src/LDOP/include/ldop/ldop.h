#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <ldop/DynamicObjectArray.h>
#include <ldop/DynamicObjectPredictionArray.h>
#include <ldop/FusedDynamicObjectArray.h>
#include <ldop/FusedDynamicObjectVirtualImuArray.h>
#include <ldop/dynamic_object_clusterer.h>
#include <ldop/dynamic_object_pose_fusion.h>
#include <ldop/dynamic_object_predictor.h>
#include <ldop/dynamic_object_tracker.h>
#include <ldop/utils.h>
#include <ldop/ufomap_mapper.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

namespace ldopcore {

class Ldop {
 public:
  // 负责订阅点云/里程计、异步处理并发布结果的主类。
  Ldop(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~Ldop();

 private:
  struct LdopTimingStats {
    // 单帧 LDOP 关键函数整体耗时，单位均为毫秒。
    double point_cloud_callback_ms{0.0};
    double processing_loop_ms{0.0};
    double process_frame_ms{0.0};
    double publish_frame_ms{0.0};
    LdopProcessingTimingTotals processing_summary;
  };

  // 输入队列中的原始数据帧，点云到来时顺带缓存一份最近里程计快照。
  struct InputFrame {
    sensor_msgs::PointCloud2 cloud_msg;
    nav_msgs::Odometry odom_msg;
    std::uint64_t sequence{0};
    double point_cloud_callback_ms{0.0};
    double odom_cloud_dt_ms{0.0};
  };

  // 工作线程处理完成后的结果帧，随后会在同一线程里直接发布。
  struct ProcessedFrame {
    sensor_msgs::PointCloud2 static_map_cloud_msg;
    sensor_msgs::PointCloud2 dynamic_cloud_msg;
    ldop::DynamicObjectArray dynamic_objects_msg;
    visualization_msgs::MarkerArray dynamic_object_markers_msg;
    visualization_msgs::MarkerArray dynamic_track_markers_msg;
    ldop::DynamicObjectPredictionArray dynamic_predictions_msg;
    visualization_msgs::MarkerArray dynamic_prediction_markers_msg;
    std::optional<ldop::FusedDynamicObjectArray> fused_objects_msg;
    std::optional<ldop::FusedDynamicObjectVirtualImuArray> fused_virtual_imu_msg;
    std::optional<visualization_msgs::MarkerArray> fused_markers_msg;
    std::vector<UfomapDynamicClusterPoint> dynamic_cluster_points;
    visualization_msgs::MarkerArray static_map_markers_msg;
    UfomapRuntimeStats ufomap_runtime_stats;
    LdopTimingStats timing;
    std::uint64_t sequence{0};
    double odom_cloud_dt_ms{0.0};
  };

  // 初始化流程：读取参数并注册 ROS 接口；唯一后台线程在构造函数中直接启动。
  void loadParameters();
  void setupRosInterfaces();

  // ROS 回调：缓存最新里程计，并把有效的点云/里程计数据对送入输入队列。
  void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void mocapPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg,
                         std::size_t source_index);

  // 工作线程：按输入队列顺序消费，处理完成后在同一线程里直接发布。
  void processingLoop();

  // 当前阶段保留 LDOP 主流程，把 UFOMap 逻辑委托给独立模块。
  ProcessedFrame processFrame(const InputFrame& frame);
  void publishFrame(ProcessedFrame& frame);

  // ROS 节点句柄、订阅器与发布器。
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  std::vector<ros::Subscriber> mocap_pose_subs_;
  ros::Publisher static_cloud_pub_;
  ros::Publisher dynamic_cloud_pub_;
  ros::Publisher dynamic_objects_pub_;
  ros::Publisher dynamic_object_markers_pub_;
  ros::Publisher dynamic_track_markers_pub_;
  ros::Publisher dynamic_predictions_pub_;
  ros::Publisher dynamic_prediction_markers_pub_;
  ros::Publisher static_map_markers_pub_;
  ros::Publisher fused_objects_pub_;
  ros::Publisher fused_virtual_imu_pub_;
  ros::Publisher fused_markers_pub_;

  // 参数服务器读取的话题名与运行配置。
  std::string input_cloud_topic_;
  std::string odom_topic_;
  std::string static_cloud_topic_;
  std::string dynamic_cloud_topic_;
  std::string dynamic_objects_topic_;
  std::string dynamic_object_markers_topic_;
  std::string dynamic_track_markers_topic_;
  std::string dynamic_predictions_topic_;
  std::string dynamic_prediction_markers_topic_;
  std::string static_map_markers_topic_;
  bool verbose_;

  ///////
  // 运行状态与统计数据
  ///////

  // 节点或线程总开关。
  std::atomic<bool> running_;

  // 已接收到的点云、里程计帧总数。
  std::atomic<std::uint64_t> received_cloud_count_;
  std::atomic<std::uint64_t> received_odom_count_;

  // 成功配对或因缺少 odom 而被丢弃的点云帧数。
  std::atomic<std::uint64_t> matched_odom_count_;
  std::atomic<std::uint64_t> missing_odom_count_;

  // 已完成处理或丢弃的点云帧总数。
  std::atomic<std::uint64_t> processed_cloud_count_;
  std::atomic<std::uint64_t> dropped_cloud_count_;  // 这个值越大，越说明当前处理速度可能跟不上输入速度。

  // 保护输入队列 input_queue_ 的互斥锁。
  std::mutex input_mutex_;  // 点云回调负责往里放数据，处理线程负责从里取数据，两边会并发访问。

  // 输入队列的条件变量。
  std::condition_variable input_cv_;  // processingLoop() 在没有输入时会阻塞等待这个条件变量。

  // 保护 latest_odom_ 和 has_latest_odom_ 的互斥锁。
  mutable std::mutex odom_mutex_;
  nav_msgs::Odometry latest_odom_;  // 当前缓存的“最新一帧”里程计。

  // UFOMap 独立模块，不再把具体建图逻辑塞进 ldop 文件里。
  std::unique_ptr<UfomapMapper> ufomap_mapper_;

  // 当前帧动态点聚类模块，消费 UfomapMapper 输出的结构化动态点。
  std::unique_ptr<DynamicObjectClusterer> dynamic_object_clusterer_;

  // 跨帧动态目标跟踪模块，消费 clusterer 的 detection 并输出稳定 track id。
  std::unique_ptr<DynamicObjectTracker> dynamic_object_tracker_;

  // 动态目标轨迹预测模块，消费 tracker 的稳定输出并生成未来轨迹预测。
  std::unique_ptr<DynamicObjectPredictor> dynamic_object_predictor_;

  // mocap 辅助融合模块；内部自带锁，允许点云处理线程和 mocap 回调同时驱动状态更新。
  std::unique_ptr<DynamicObjectPoseFusion> dynamic_object_pose_fusion_;

  // 是否已经缓存过至少一帧可用于配对的最新里程计。
  bool has_latest_odom_;
  std::deque<InputFrame> input_queue_;

  // 后台线程：统一负责处理和发布。
  std::thread processing_thread_;
};

}  // namespace ldopcore
