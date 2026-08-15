#include <ldop/ldop.h>
#include <ldop/utils.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <optional>
#include <vector>

namespace ldopcore {

namespace {

Eigen::Vector3d fromUfoPoint(const ufo::Point& point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

ufo::Point toUfoPoint(const Eigen::Vector3d& point) {
  return ufo::Point(static_cast<float>(point.x()),
                    static_cast<float>(point.y()),
                    static_cast<float>(point.z()));
}

PredictionMapNodeQuery::OccupancyState toPredictionOccupancyState(
    const UfomapOccupancyState state) {
  switch (state) {
    case UfomapOccupancyState::Free:
      return PredictionMapNodeQuery::OccupancyState::Free;
    case UfomapOccupancyState::Occupied:
      return PredictionMapNodeQuery::OccupancyState::Occupied;
    case UfomapOccupancyState::OutOfMap:
      return PredictionMapNodeQuery::OccupancyState::OutOfMap;
    case UfomapOccupancyState::Unknown:
    default:
      return PredictionMapNodeQuery::OccupancyState::Unknown;
  }
}

class UfomapPredictionMapQuery final : public PredictionMapQuery {
 public:
  explicit UfomapPredictionMapQuery(const UfomapMapper& mapper)
      : mapper_(mapper) {}

  bool available() const override { return true; }

  std::optional<PredictionMapNodeQuery> queryNode(
      const Eigen::Vector3d& point) const override {
    const auto result = mapper_.queryNode(toUfoPoint(point));

    PredictionMapNodeQuery node;
    node.state = toPredictionOccupancyState(result.state);
    node.exists = result.exists;
    node.occupied = result.occupied;
    node.free = result.free;
    node.unknown = result.unknown;
    node.out_of_map = result.out_of_map;
    node.seen_free = result.seen_free;
    node.center = fromUfoPoint(result.center);
    node.voxel_size = result.voxel_size;
    return node;
  }

  bool seenFree(const Eigen::Vector3d& point) const override {
    return mapper_.seenFree(toUfoPoint(point));
  }

  std::vector<PredictionLocalOccupiedNode> queryLocalOccupied(
      const Eigen::Vector3d& center,
      const double radius,
      const std::size_t max_results) const override {
    return queryLocalOccupied(center, radius, max_results, 0U);
  }

  std::vector<PredictionLocalOccupiedNode> queryLocalOccupied(
      const Eigen::Vector3d& center,
      const double radius,
      const std::size_t max_results,
      const std::uint8_t query_depth) const override {
    UfomapLocalOccupiedQuery query;
    query.depth = static_cast<ufo::depth_t>(query_depth);
    query.min_hits = 1U;
    query.max_results = max_results;
    query.intersects.emplace_back(toUfoPoint(center), static_cast<float>(radius));

    const auto occupied_nodes = mapper_.queryLocalOccupied(query);
    std::vector<PredictionLocalOccupiedNode> converted_nodes;
    converted_nodes.reserve(occupied_nodes.size());
    for (const auto& occupied : occupied_nodes) {
      PredictionLocalOccupiedNode converted;
      converted.center = fromUfoPoint(occupied.center);
      converted.voxel_size = occupied.voxel_size;
      converted.hits = occupied.hits;
      converted_nodes.push_back(converted);
    }
    return converted_nodes;
  }

  std::optional<PredictionLocalOccupiedNode> queryNearestOccupied(
      const Eigen::Vector3d& center,
      const double radius,
      const std::uint8_t query_depth = 0U) const override {
    const auto occupied = mapper_.queryNearestOccupied(
        toUfoPoint(center), radius, static_cast<ufo::depth_t>(query_depth), 1U);
    if (!occupied.has_value()) {
      return std::nullopt;
    }
    PredictionLocalOccupiedNode converted;
    converted.center = fromUfoPoint(occupied->center);
    converted.voxel_size = occupied->voxel_size;
    converted.hits = occupied->hits;
    return converted;
  }

 private:
  const UfomapMapper& mapper_;
};

}  // namespace

Ldop::Ldop(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      running_(true),
      received_cloud_count_(0U),
      received_odom_count_(0U),
      matched_odom_count_(0U),
      missing_odom_count_(0U),
      processed_cloud_count_(0U),
      dropped_cloud_count_(0U),
      has_latest_odom_(false) {
  loadParameters();
  ufomap_mapper_ = std::make_unique<UfomapMapper>(nh_, pnh_, verbose_);
  dynamic_object_clusterer_ = std::make_unique<DynamicObjectClusterer>(pnh_, verbose_);
  dynamic_object_tracker_ = std::make_unique<DynamicObjectTracker>(pnh_, verbose_);
  dynamic_object_predictor_ = std::make_unique<DynamicObjectPredictor>(pnh_, verbose_);
  dynamic_object_pose_fusion_ = std::make_unique<DynamicObjectPoseFusion>(pnh_, verbose_);
  setupRosInterfaces();
  // 当前只有一个后台处理线程，构造阶段直接启动比再包一层单调用函数更容易按初始化顺序阅读。
  processing_thread_ = std::thread(&Ldop::processingLoop, this);

  ROS_INFO_STREAM("  LDOP is ready.");
  ROS_INFO_STREAM("  input cloud: " << input_cloud_topic_);
  ROS_INFO_STREAM("  odom: " << odom_topic_);
  ROS_INFO_STREAM("  static output: " << static_cloud_topic_);
  ROS_INFO_STREAM("  dynamic output: " << dynamic_cloud_topic_);
  ROS_INFO_STREAM("  dynamic objects: " << dynamic_objects_topic_);
  ROS_INFO_STREAM("  dynamic object markers: " << dynamic_object_markers_topic_);
  ROS_INFO_STREAM("  dynamic track markers: " << dynamic_track_markers_topic_);
  ROS_INFO_STREAM("  dynamic predictions: " << dynamic_predictions_topic_);
  ROS_INFO_STREAM("  dynamic prediction markers: " << dynamic_prediction_markers_topic_);
  ROS_INFO_STREAM("  static voxel markers: " << static_map_markers_topic_);
  ROS_INFO_STREAM("  fused objects: " << dynamic_object_pose_fusion_->fusedObjectsTopic());
  ROS_INFO_STREAM("  fused virtual IMU: " << dynamic_object_pose_fusion_->virtualImuTopic());
  ROS_INFO_STREAM("  fused markers: " << dynamic_object_pose_fusion_->markerTopic());
}

Ldop::~Ldop() {
  // 析构阶段直接收束唯一后台线程，避免把简单退出流程藏在额外薄包装里。
  running_.store(false);
  input_cv_.notify_all();

  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
}

void Ldop::loadParameters() {
  // 坐标系由输入数据自身声明；这里只加载话题名和模块运行开关，避免用参数重贴 frame 标签。
  pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, "/points_raw");
  pnh_.param<std::string>("odom_topic", odom_topic_, "/lidar_slam/odom");
  pnh_.param<std::string>("static_cloud_topic", static_cloud_topic_, "/ldop/static_cloud");
  pnh_.param<std::string>("dynamic_cloud_topic", dynamic_cloud_topic_, "/ldop/dynamic_cloud");
  pnh_.param<std::string>("dynamic_objects_topic", dynamic_objects_topic_, "/ldop/dynamic_objects");
  pnh_.param<std::string>("dynamic_object_markers_topic", dynamic_object_markers_topic_, "/ldop/dynamic_object_markers");
  pnh_.param<std::string>("dynamic_track_markers_topic", dynamic_track_markers_topic_, "/ldop/dynamic_track_markers");
  pnh_.param<std::string>("dynamic_predictions_topic", dynamic_predictions_topic_, "/ldop/dynamic_object_predictions");
  pnh_.param<std::string>("dynamic_prediction_markers_topic", dynamic_prediction_markers_topic_, "/ldop/dynamic_prediction_markers");
  pnh_.param<std::string>("static_map_markers_topic", static_map_markers_topic_, "/ldop/static_map_markers");
  pnh_.param("verbose", verbose_, false);
}

void Ldop::setupRosInterfaces() {
  // 订阅输入，并提前创建 LDOP 自己负责的输出通道。
  cloud_sub_ = nh_.subscribe(input_cloud_topic_, 50, &Ldop::pointCloudCallback, this);
  odom_sub_ = nh_.subscribe(odom_topic_, 200, &Ldop::odomCallback, this);
  mocap_pose_subs_.clear();
  const std::vector<std::string>& mocap_topics = dynamic_object_pose_fusion_->mocapTopics();
  mocap_pose_subs_.reserve(mocap_topics.size());
  for (std::size_t source_index = 0U; source_index < mocap_topics.size(); ++source_index) {
    const std::string& topic = mocap_topics[source_index];
    // source_index 是融合状态的外部身份索引，订阅时固定下来，避免回调里再按 topic 查找。
    mocap_pose_subs_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(
        topic,
        200,
        [this, source_index](const geometry_msgs::PoseStampedConstPtr& pose_msg) {
          mocapPoseCallback(pose_msg, source_index);
        }));
  }

  static_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(static_cloud_topic_, 10);
  dynamic_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(dynamic_cloud_topic_, 10);
  dynamic_objects_pub_ = nh_.advertise<ldop::DynamicObjectArray>(dynamic_objects_topic_, 10);
  dynamic_object_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(dynamic_object_markers_topic_, 10);
  dynamic_track_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(dynamic_track_markers_topic_, 10);
  dynamic_predictions_pub_ = nh_.advertise<ldop::DynamicObjectPredictionArray>(dynamic_predictions_topic_, 10);
  dynamic_prediction_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(dynamic_prediction_markers_topic_, 10);
  static_map_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(static_map_markers_topic_, 10, true);
  fused_objects_pub_ =
      nh_.advertise<ldop::FusedDynamicObjectArray>(dynamic_object_pose_fusion_->fusedObjectsTopic(), 10);
  fused_virtual_imu_pub_ = nh_.advertise<ldop::FusedDynamicObjectVirtualImuArray>(
      dynamic_object_pose_fusion_->virtualImuTopic(), 10);
  fused_markers_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(dynamic_object_pose_fusion_->markerTopic(), 10);
}

void Ldop::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
  if (odom_msg == nullptr) {
    return;
  }

  // 极简版本只缓存最新一帧里程计，供点云回调快速取快照。
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = *odom_msg;
  has_latest_odom_ = true;
  received_odom_count_.fetch_add(1U);
}

void Ldop::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  if (cloud_msg == nullptr) {
    return;
  }

  const auto callback_start = std::chrono::steady_clock::now();
  InputFrame frame;
  const std::uint64_t cloud_sequence = received_cloud_count_.fetch_add(1U) + 1U;

  {
    // 点云入队前先拷贝一份最新里程计，后续处理线程不再访问共享 odom。
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    if (!has_latest_odom_) {
      missing_odom_count_.fetch_add(1U);
      if (verbose_) {
        ROS_WARN_STREAM_THROTTLE(1.0, "skipped cloud #" << cloud_sequence
                                                             << " because no odom is available yet");
      }
      return;
    }
    frame.odom_msg = latest_odom_;
  }
  // 点云进入系统时先编号；只有拿到最新 odom 快照，才允许进入处理队列。
  matched_odom_count_.fetch_add(1U);

  frame.cloud_msg = *cloud_msg;
  frame.sequence = cloud_sequence;
  frame.odom_cloud_dt_ms =
      (frame.odom_msg.header.stamp - cloud_msg->header.stamp).toSec() * 1000.0;

  {
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    // 只有输入队列触顶时才丢掉最旧点云，其余情况由处理线程按先进先出顺序消费。
    while (static_cast<int>(input_queue_.size()) >= 10) {
      input_queue_.pop_front();
      dropped_cloud_count_.fetch_add(1U);
    }
    input_queue_.push_back(std::move(frame));
    input_queue_.back().point_cloud_callback_ms =
        elapsedMs(callback_start, std::chrono::steady_clock::now());
  }
  input_cv_.notify_one();
}

void Ldop::mocapPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg,
                             const std::size_t source_index) {
  if (pose_msg == nullptr) {
    return;
  }

  DynamicObjectPoseFusionSnapshot snapshot =
      dynamic_object_pose_fusion_->processMocapPose(source_index, *pose_msg);
  if (snapshot.fused_objects.has_value()) {
    fused_objects_pub_.publish(*snapshot.fused_objects);
  }
  if (snapshot.virtual_imu.has_value()) {
    fused_virtual_imu_pub_.publish(*snapshot.virtual_imu);
  }
  if (snapshot.markers.has_value()) {
    fused_markers_pub_.publish(*snapshot.markers);
  }
}

void Ldop::processingLoop() {
  while (true) {
    InputFrame frame;

    {
      std::unique_lock<std::mutex> input_lock(input_mutex_);
      input_cv_.wait(input_lock, [this] { return !running_.load() || !input_queue_.empty(); });

      if (!running_.load() && input_queue_.empty()) {
        return;
      }

      // 队列为空时才会等待；只要有完整数据对且写线程已经释放互斥锁，就按先进先出顺序处理最旧一帧。
      frame = std::move(input_queue_.front());
      input_queue_.pop_front();
    }

    const auto loop_start = std::chrono::steady_clock::now();
    ProcessedFrame processed = processFrame(frame);
    processed_cloud_count_.fetch_add(1U);
    publishFrame(processed);
    processed.timing.processing_loop_ms = elapsedMs(loop_start, std::chrono::steady_clock::now());
    processed.timing.processing_summary.ldop_total_ms = processed.timing.processing_loop_ms;

    if (!verbose_) {
      ROS_INFO_STREAM_THROTTLE(
          1.0,
          formatLdopProcessingTimingSummary(processed.sequence,
                                            processed.timing.processing_summary));
    } else {
      ROS_INFO_STREAM_THROTTLE(1.0, "LDOP timing frame #" << processed.sequence
                                    << ": odomCloudDt="
                                    << formatFloatMs(processed.odom_cloud_dt_ms)
                                    << "ms, pointCloudCallback="
                                    << formatFloatMs(processed.timing.point_cloud_callback_ms)
                                    << "ms, processingLoop="
                                    << formatFloatMs(processed.timing.processing_loop_ms)
                                    << "ms, processFrame="
                                    << formatFloatMs(processed.timing.process_frame_ms)
                                    << "ms, publishFrame="
                                    << formatFloatMs(processed.timing.publish_frame_ms)
                                    << "ms");
      ROS_INFO_STREAM_THROTTLE(1.0, "LDOP status frame #" << processed.sequence
                                    << ": received=" << received_cloud_count_.load()
                                    << ", processed=" << processed_cloud_count_.load()
                                    << ", backlog="
                                    << (received_cloud_count_.load() > processed_cloud_count_.load()
                                            ? received_cloud_count_.load() -processed_cloud_count_.load(): 0U)
                                    << ", matched=" << matched_odom_count_.load()
                                    << ", missing=" << missing_odom_count_.load()
                                    << ", dropped=" << dropped_cloud_count_.load()
                                    << ", ufomapNodes=" << processed.ufomap_runtime_stats.node_count);
    }
  }
}

Ldop::ProcessedFrame Ldop::processFrame(const InputFrame& frame) {
  // 主流程只做调度，把世界系点云和里程计委托给独立的 UFOMap 模块。
  const auto processing_start = std::chrono::steady_clock::now();
  ProcessedFrame processed;
  processed.sequence = frame.sequence;
  processed.odom_cloud_dt_ms = frame.odom_cloud_dt_ms;
  processed.timing.point_cloud_callback_ms = frame.point_cloud_callback_ms;

  UfomapFrameResult ufomap_result =
      ufomap_mapper_->processInputCloud(frame.cloud_msg, frame.odom_msg);
  processed.timing.processing_summary.map_module_ms =
      ufomap_result.timing.process_input_cloud_ms;
  processed.static_map_cloud_msg = std::move(ufomap_result.static_map_cloud_msg);
  processed.static_map_markers_msg = std::move(ufomap_result.static_map_markers);
  processed.ufomap_runtime_stats = ufomap_result.runtime_stats;
  processed.dynamic_cloud_msg = std::move(ufomap_result.dynamic_cloud_msg);
  processed.dynamic_cluster_points = std::move(ufomap_result.classification.dynamic_cluster_points);
  DynamicObjectClustererFrameResult cluster_result =
      dynamic_object_clusterer_->processDynamicObjects(processed.dynamic_cloud_msg.header,
                                                      processed.dynamic_cluster_points);
  processed.timing.processing_summary.cluster_module_ms =
      cluster_result.timing.process_dynamic_objects_ms;
  processed.dynamic_object_markers_msg = std::move(cluster_result.dynamic_object_markers_msg);

  DynamicObjectTrackerFrameResult tracker_result =
      dynamic_object_tracker_->processDynamicTracks(processed.dynamic_cloud_msg.header,
                                                    cluster_result.detections);
  processed.timing.processing_summary.tracking_module_ms =
      tracker_result.timing.process_dynamic_tracks_ms;
  processed.dynamic_objects_msg = std::move(tracker_result.dynamic_objects_msg);
  processed.dynamic_track_markers_msg = std::move(tracker_result.dynamic_track_markers_msg);

  const auto prediction_start = std::chrono::steady_clock::now();
  try {
    const UfomapPredictionMapQuery prediction_map_query(*ufomap_mapper_);
    const DynamicObjectPredictorFrameResult prediction_result =
        dynamic_object_predictor_->predict(processed.dynamic_cloud_msg.header,
                                            tracker_result.prediction_inputs,
                                            &prediction_map_query);
    processed.dynamic_predictions_msg = prediction_result.predictions_msg;
    processed.dynamic_prediction_markers_msg = prediction_result.prediction_markers_msg;
    processed.timing.processing_summary.prediction_module_ms =
        prediction_result.timing.predict_total_ms;
  } catch (const std::exception& ex) {
    processed.dynamic_predictions_msg.header = processed.dynamic_cloud_msg.header;
    visualization_msgs::Marker clear_prediction_marker;
    clear_prediction_marker.header = processed.dynamic_cloud_msg.header;
    clear_prediction_marker.ns = "dynamic_predictions";
    clear_prediction_marker.action = visualization_msgs::Marker::DELETEALL;
    processed.dynamic_prediction_markers_msg.markers.push_back(
        std::move(clear_prediction_marker));
    processed.timing.processing_summary.prediction_module_ms =
        elapsedMs(prediction_start, std::chrono::steady_clock::now());
    ROS_WARN_STREAM_THROTTLE(
        1.0, "Dynamic object prediction failed; continuing detection pipeline: "
                 << ex.what());
  }

  DynamicObjectPoseFusionSnapshot fusion_snapshot =
      dynamic_object_pose_fusion_->updateLidarAnchors(processed.dynamic_cloud_msg.header,
                                                      cluster_result.detections);
  processed.fused_objects_msg = std::move(fusion_snapshot.fused_objects);
  processed.fused_virtual_imu_msg = std::move(fusion_snapshot.virtual_imu);
  processed.fused_markers_msg = std::move(fusion_snapshot.markers);
  processed.timing.process_frame_ms =
      elapsedMs(processing_start, std::chrono::steady_clock::now());
  return processed;
}

void Ldop::publishFrame(ProcessedFrame& frame) {
  // 统一从这里发出 LDOP 输出；静态体素 marker 已在处理阶段异步构建完成。
  const auto publish_start = std::chrono::steady_clock::now();
  static_cloud_pub_.publish(frame.static_map_cloud_msg);
  dynamic_cloud_pub_.publish(frame.dynamic_cloud_msg);
  dynamic_objects_pub_.publish(frame.dynamic_objects_msg);
  dynamic_object_markers_pub_.publish(frame.dynamic_object_markers_msg);
  dynamic_track_markers_pub_.publish(frame.dynamic_track_markers_msg);
  dynamic_predictions_pub_.publish(frame.dynamic_predictions_msg);
  dynamic_prediction_markers_pub_.publish(frame.dynamic_prediction_markers_msg);
  if (frame.fused_objects_msg.has_value()) {
    fused_objects_pub_.publish(*frame.fused_objects_msg);
  }
  if (frame.fused_virtual_imu_msg.has_value()) {
    fused_virtual_imu_pub_.publish(*frame.fused_virtual_imu_msg);
  }
  if (frame.fused_markers_msg.has_value()) {
    fused_markers_pub_.publish(*frame.fused_markers_msg);
  }
  static_map_markers_pub_.publish(frame.static_map_markers_msg);
  frame.timing.publish_frame_ms =
      elapsedMs(publish_start, std::chrono::steady_clock::now());
}

}  // namespace ldopcore
