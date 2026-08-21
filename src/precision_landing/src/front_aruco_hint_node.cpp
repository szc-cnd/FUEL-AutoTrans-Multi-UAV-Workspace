#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "precision_landing/aruco_tracker.hpp"
#include "precision_landing/landing_search_core.hpp"
#include "precision_landing/LandingPlatformArray.h"

namespace precision_landing {
namespace {

std::string firstToken(const std::string& text) {
  const std::size_t end = text.find_first_of(" \t\r\n");
  return text.substr(0, end);
}

ArucoTrackerConfig loadTrackerConfig(ros::NodeHandle& node) {
  ArucoTrackerConfig config;
  node.param("marker/size_m", config.marker_size_m, 0.60);
  node.param("marker/stable_frames", config.stable_frames, 5);
  node.param("marker/max_reprojection_error_px",
             config.max_reprojection_error_px, 5.0);
  node.param("marker/min_distance_m", config.min_distance_m, 0.30);
  node.param("marker/max_distance_m", config.max_distance_m, 6.0);
  node.param("marker/max_position_jump_m", config.max_position_jump_m, 0.80);
  node.param("marker/max_tilt_deg", config.max_tilt_deg, 85.0);
  return config;
}

WorldTargetFilterConfig loadWorldFilterConfig(ros::NodeHandle& node) {
  WorldTargetFilterConfig config;
  int stable_samples = 5;
  node.param("world_filter/stable_samples", stable_samples, 5);
  config.stable_samples = static_cast<std::size_t>(std::max(1, stable_samples));
  node.param("world_filter/max_sample_gap_sec", config.max_sample_gap_sec,
             0.25);
  node.param("world_filter/max_position_jump_m", config.max_position_jump_m,
             0.50);
  node.param("world_filter/max_spread_m", config.max_spread_m, 0.30);
  return config;
}

class FrontArucoHintNode {
 public:
  FrontArucoHintNode()
      : node_(),
        private_node_("~"),
        image_transport_(node_),
        tracker_(loadTrackerConfig(private_node_)),
        target_filter_(loadWorldFilterConfig(private_node_)),
        tf_listener_(tf_buffer_) {
    candidate_filter_config_ = loadWorldFilterConfig(private_node_);
    loadConfiguration();
    configureInterfaces();
    publishStatus("等待出通道后启用前视ArUco粗定位");
  }

 private:
  void loadConfiguration() {
    private_node_.param("marker/requested_id", requested_marker_id_, -1);
    private_node_.param("frames/body", body_frame_,
                        std::string("UAV0/body"));
    private_node_.param("frames/output_world", output_world_frame_,
                        std::string("world"));
    private_node_.param("frames/camera_optical", camera_optical_frame_,
                        std::string("camera_color_optical_frame"));
    private_node_.param("mission/require_stage_gate", require_stage_gate_, true);
    private_node_.param("mission/require_search_state_gate",
                        require_search_state_gate_, true);
    private_node_.param("safety/image_timeout_sec", image_timeout_sec_, 0.30);
    private_node_.param("safety/max_header_future_sec", max_header_future_sec_,
                        0.05);
    private_node_.param("safety/tf_timeout_sec", tf_timeout_sec_, 0.05);
    private_node_.param("safety/max_image_odom_delta_sec",
                        max_image_odom_delta_sec_, 0.11);
    private_node_.param("depth_validation/required", require_depth_, true);
    private_node_.param("depth_validation/scale_16uc1", depth_scale_16uc1_,
                        0.001);
    private_node_.param("depth_validation/max_time_delta_sec",
                        max_depth_time_delta_sec_, 0.10);
    private_node_.param("depth_validation/search_radius_px", depth_search_radius_,
                        5);
    private_node_.param("depth_validation/min_depth_m", min_depth_m_, 0.25);
    private_node_.param("depth_validation/max_depth_m", max_depth_m_, 6.0);
    private_node_.param("depth_validation/max_absolute_error_m",
                        max_depth_absolute_error_m_, 0.35);
    private_node_.param("depth_validation/max_relative_error",
                        max_depth_relative_error_, 0.20);

    if (body_frame_.empty() || output_world_frame_.empty() ||
        camera_optical_frame_.empty() || image_timeout_sec_ <= 0.0 ||
        max_header_future_sec_ < 0.0 || tf_timeout_sec_ < 0.0 ||
        max_image_odom_delta_sec_ <= 0.0 ||
        depth_scale_16uc1_ <= 0.0 || max_depth_time_delta_sec_ <= 0.0 ||
        depth_search_radius_ < 0 || min_depth_m_ <= 0.0 ||
        max_depth_m_ <= min_depth_m_ || max_depth_absolute_error_m_ <= 0.0 ||
        max_depth_relative_error_ <= 0.0) {
      throw std::runtime_error("front_aruco_hint参数无效");
    }
  }

  void configureInterfaces() {
    std::string image_topic("/camera/color/image_raw");
    std::string camera_info_topic("/camera/color/camera_info");
    std::string depth_topic("/camera/aligned_depth_to_color/image_raw");
    std::string odometry_topic("/UAV0/fast_lio/Odom_high_freq");
    std::string mission_status_topic("/UAV0/mission/task_status");
    std::string search_state_topic("/landing_diff_search_manager/state");
    std::string hint_topic("/UAV0/landing/front_aruco_hint");
    std::string candidates_topic("/UAV0/landing/front/candidates");
    std::string locked_id_topic("/UAV0/landing/front/locked_id");
    std::string status_topic("/UAV0/landing/front/status");
    std::string debug_image_topic("/UAV0/landing/front/debug_image");
    private_node_.param("topics/image", image_topic, image_topic);
    private_node_.param("topics/camera_info", camera_info_topic,
                        camera_info_topic);
    private_node_.param("topics/aligned_depth", depth_topic, depth_topic);
    private_node_.param("topics/odometry", odometry_topic, odometry_topic);
    private_node_.param("topics/mission_status", mission_status_topic,
                        mission_status_topic);
    private_node_.param("topics/search_state", search_state_topic,
                        search_state_topic);
    private_node_.param("topics/hint_world", hint_topic, hint_topic);
    private_node_.param("topics/candidates", candidates_topic,
                        candidates_topic);
    private_node_.param("topics/locked_id", locked_id_topic, locked_id_topic);
    private_node_.param("topics/status", status_topic, status_topic);
    private_node_.param("topics/debug_image", debug_image_topic,
                        debug_image_topic);

    image_subscriber_ = image_transport_.subscribe(
        image_topic, 1, &FrontArucoHintNode::imageCallback, this);
    camera_info_subscriber_ = node_.subscribe(
        camera_info_topic, 1, &FrontArucoHintNode::cameraInfoCallback, this);
    depth_subscriber_ = node_.subscribe(
        depth_topic, 1, &FrontArucoHintNode::depthCallback, this);
    odometry_subscriber_ = node_.subscribe(
        odometry_topic, 100, &FrontArucoHintNode::odometryCallback, this);
    mission_status_subscriber_ = node_.subscribe(
        mission_status_topic, 5, &FrontArucoHintNode::missionStatusCallback,
        this);
    search_state_subscriber_ = node_.subscribe(
        search_state_topic, 5, &FrontArucoHintNode::searchStateCallback, this);
    hint_publisher_ =
        node_.advertise<geometry_msgs::PoseStamped>(hint_topic, 3);
    candidates_publisher_ =
        node_.advertise<precision_landing::LandingPlatformArray>(
            candidates_topic, 2, true);
    locked_id_publisher_ = node_.advertise<std_msgs::Int32>(locked_id_topic, 1,
                                                            true);
    status_publisher_ =
        node_.advertise<std_msgs::String>(status_topic, 1, true);
    debug_image_publisher_ =
        image_transport_.advertise(debug_image_topic, 1);
  }

  bool stageAllowsHint() const {
    const bool mission_ready =
        !require_stage_gate_ || mission_stage_ == "SEARCH_OUTSIDE_LANDING" ||
        mission_stage_ == "SEARCH_OUTSIDE_QR";
    const bool scan_ready = !require_search_state_gate_ || front_scan_active_;
    return mission_ready && scan_ready;
  }

  static bool isFrontScanState(const std::string& state) {
    return state == "FRONT_ARUCO_INITIAL_WAIT" ||
           state == "FRONT_ARUCO_YAW_SCAN_LEFT" ||
           state == "FRONT_ARUCO_YAW_SCAN_RIGHT" ||
           state == "FRONT_ARUCO_YAW_SCAN_RETURN";
  }

  void searchStateCallback(const std_msgs::StringConstPtr& message) {
    const std::string next_state = firstToken(message->data);
    if (next_state == landing_search_state_) return;
    const bool next_active = isFrontScanState(next_state);
    landing_search_state_ = next_state;
    if (next_active != front_scan_active_) {
      front_scan_active_ = next_active;
      tracker_.reset();
      target_filter_.reset();
      candidate_filters_.clear();
      stable_candidates_.clear();
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      publishCandidates(header);
      publishLockedId();
    }
    ROS_INFO("front_aruco_hint: landing search state -> %s, scan_active=%s",
             landing_search_state_.c_str(), front_scan_active_ ? "true" : "false");
  }

  void missionStatusCallback(const std_msgs::StringConstPtr& message) {
    const std::string next_stage = firstToken(message->data);
    if (next_stage == mission_stage_) return;
    mission_stage_ = next_stage;
    tracker_.reset();
    target_filter_.reset();
    candidate_filters_.clear();
    stable_candidates_.clear();
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    publishCandidates(header);
    publishLockedId();
    ROS_INFO("front_aruco_hint: mission stage -> %s", mission_stage_.c_str());
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    if (message->K[0] <= 0.0 || message->K[4] <= 0.0) {
      ROS_WARN_THROTTLE(2.0, "front_aruco_hint: D435相机内参无效");
      return;
    }
    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        camera_matrix_.at<double>(row, column) = message->K[row * 3 + column];
      }
    }
    distortion_ =
        cv::Mat::zeros(1, static_cast<int>(message->D.size()), CV_64F);
    for (std::size_t index = 0U; index < message->D.size(); ++index) {
      distortion_.at<double>(0, static_cast<int>(index)) = message->D[index];
    }
    camera_info_received_ = true;
  }

  void depthCallback(const sensor_msgs::ImageConstPtr& message) {
    cv_bridge::CvImageConstPtr depth;
    try {
      depth = cv_bridge::toCvShare(message);
    } catch (const cv_bridge::Exception& error) {
      ROS_WARN_THROTTLE(2.0, "front_aruco_hint: 深度图转换失败: %s",
                        error.what());
      return;
    }
    cv::Mat depth_m;
    if (message->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        message->encoding == sensor_msgs::image_encodings::MONO16) {
      depth->image.convertTo(depth_m, CV_32F, depth_scale_16uc1_);
    } else if (message->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      depth->image.convertTo(depth_m, CV_32F);
    } else {
      ROS_WARN_THROTTLE(2.0, "front_aruco_hint: 不支持深度编码 %s",
                        message->encoding.c_str());
      return;
    }
    latest_depth_m_ = depth_m.clone();
    latest_depth_stamp_ = message->header.stamp;
  }

  void odometryCallback(const nav_msgs::OdometryConstPtr& message) {
    if (message->header.stamp.isZero()) return;
    odometry_history_.push_back(*message);
    while (odometry_history_.size() > 200U) odometry_history_.pop_front();
  }

  const nav_msgs::Odometry* nearestOdometry(const ros::Time& stamp,
                                             double* delta_sec) const {
    const nav_msgs::Odometry* nearest = nullptr;
    double best_delta = std::numeric_limits<double>::infinity();
    for (const nav_msgs::Odometry& odometry : odometry_history_) {
      const double delta = std::abs((odometry.header.stamp - stamp).toSec());
      if (delta < best_delta) {
        best_delta = delta;
        nearest = &odometry;
      }
    }
    if (delta_sec != nullptr) *delta_sec = best_delta;
    return nearest;
  }

  bool validateDepth(const TargetObservation& observation,
                     const ros::Time& image_stamp, double* measured_depth) const {
    if (!require_depth_) return true;
    if (latest_depth_m_.empty() || latest_depth_stamp_.isZero() ||
        std::abs((latest_depth_stamp_ - image_stamp).toSec()) >
            max_depth_time_delta_sec_) {
      return false;
    }
    const int center_x = static_cast<int>(std::lround(observation.image_center_px.x()));
    const int center_y = static_cast<int>(std::lround(observation.image_center_px.y()));
    std::vector<float> samples;
    for (int y = center_y - depth_search_radius_;
         y <= center_y + depth_search_radius_; ++y) {
      if (y < 0 || y >= latest_depth_m_.rows) continue;
      for (int x = center_x - depth_search_radius_;
           x <= center_x + depth_search_radius_; ++x) {
        if (x < 0 || x >= latest_depth_m_.cols) continue;
        const float value = latest_depth_m_.at<float>(y, x);
        if (std::isfinite(value) && value >= min_depth_m_ &&
            value <= max_depth_m_) {
          samples.push_back(value);
        }
      }
    }
    if (samples.empty()) return false;
    const std::size_t middle = samples.size() / 2U;
    std::nth_element(samples.begin(), samples.begin() + middle, samples.end());
    const double depth_m = samples[middle];
    if (measured_depth != nullptr) *measured_depth = depth_m;
    const double allowed_error =
        std::max(max_depth_absolute_error_m_, max_depth_relative_error_ * depth_m);
    return std::abs(depth_m - observation.position_camera.z()) <= allowed_error;
  }

  void publishCandidates(const std_msgs::Header& source_header) {
    precision_landing::LandingPlatformArray message;
    message.header = source_header;
    message.header.frame_id = output_world_frame_;
    for (const auto& item : stable_candidates_) {
      precision_landing::LandingPlatform platform = item.second;
      platform.header = message.header;
      platform.pose.header = message.header;
      message.platforms.push_back(platform);
    }
    candidates_publisher_.publish(message);
  }

  void collectStableCandidates(
      const std_msgs::Header& image_header,
      const geometry_msgs::TransformStamped& camera_to_body,
      const nav_msgs::Odometry& odometry) {
    const geometry_msgs::Point& body_position = odometry.pose.pose.position;
    const geometry_msgs::Quaternion& body_attitude =
        odometry.pose.pose.orientation;
    Eigen::Quaterniond body_orientation_world(
        body_attitude.w, body_attitude.x, body_attitude.y, body_attitude.z);
    if (!body_orientation_world.coeffs().array().isFinite().all() ||
        body_orientation_world.norm() < 1.0e-6) {
      return;
    }
    body_orientation_world.normalize();

    for (const DetectedTarget& detected : tracker_.detectedTargets()) {
      TargetObservation observation;
      observation.valid = true;
      observation.id = detected.id;
      observation.position_camera = detected.position_camera;
      observation.image_center_px = detected.image_center_px;
      if (!validateDepth(observation, image_header.stamp, nullptr)) continue;

      geometry_msgs::PointStamped point_camera;
      point_camera.header = image_header;
      if (point_camera.header.frame_id.empty()) {
        point_camera.header.frame_id = camera_optical_frame_;
      }
      point_camera.point.x = detected.position_camera.x();
      point_camera.point.y = detected.position_camera.y();
      point_camera.point.z = detected.position_camera.z();
      geometry_msgs::PointStamped point_body;
      tf2::doTransform(point_camera, point_body, camera_to_body);
      const Eigen::Vector3d body_point(
          point_body.point.x, point_body.point.y, point_body.point.z);
      const Eigen::Vector3d world =
          body_orientation_world * body_point +
          Eigen::Vector3d(body_position.x, body_position.y, body_position.z);

      auto filter = candidate_filters_.find(detected.id);
      if (filter == candidate_filters_.end()) {
        filter = candidate_filters_
                     .emplace(detected.id,
                              std::unique_ptr<WorldTargetFilter>(
                                  new WorldTargetFilter(candidate_filter_config_)))
                     .first;
      }
      if (!filter->second->add(detected.id, world,
                               image_header.stamp.toSec())) {
        continue;
      }

      const Eigen::Vector3d filtered = filter->second->filteredPoint();
      precision_landing::LandingPlatform platform;
      platform.header = image_header;
      platform.header.frame_id = output_world_frame_;
      platform.id = detected.id;
      platform.pose.header = platform.header;
      platform.pose.pose.position.x = filtered.x();
      platform.pose.pose.position.y = filtered.y();
      platform.pose.pose.position.z = filtered.z();
      platform.pose.pose.orientation.w = 1.0;
      platform.score = detected.score;
      stable_candidates_[detected.id] = platform;
    }
    publishCandidates(image_header);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    const double image_age_sec =
        (ros::Time::now() - message->header.stamp).toSec();
    if (!camera_info_received_ || message->header.stamp.isZero() ||
        image_age_sec < -max_header_future_sec_ ||
        image_age_sec > image_timeout_sec_) {
      ROS_WARN_THROTTLE(2.0,
                        "front_aruco_hint: 等待带时间戳的D435图像和CameraInfo");
      return;
    }

    cv_bridge::CvImageConstPtr image;
    try {
      image = cv_bridge::toCvShare(message, "bgr8");
    } catch (const cv_bridge::Exception& error) {
      ROS_WARN_THROTTLE(2.0, "front_aruco_hint: 彩色图转换失败: %s",
                        error.what());
      return;
    }
    if (!stageAllowsHint()) {
      annotateAndPublish(image->image, message->header, "WAITING FOR EXIT");
      return;
    }

    const TargetObservation observation = tracker_.process(
        image->image, camera_matrix_, distortion_, message->header.stamp.toSec(),
        requested_marker_id_);
    publishLockedId();
    if (tracker_.detectedTargets().empty()) {
      target_filter_.reset();
      annotateAndPublish(tracker_.debugImage(), message->header,
                         "SEARCHING FRONT ARUCO");
      return;
    }
    geometry_msgs::TransformStamped camera_to_body;
    try {
      camera_to_body = tf_buffer_.lookupTransform(
          body_frame_,
          message->header.frame_id.empty() ? camera_optical_frame_
                                           : message->header.frame_id,
          message->header.stamp, ros::Duration(tf_timeout_sec_));
    } catch (const tf2::TransformException& error) {
      target_filter_.reset();
      ROS_WARN_THROTTLE(1.0, "front_aruco_hint: 等待D435到FAST-LIO机体的静态TF: %s",
                        error.what());
      annotateAndPublish(tracker_.debugImage(), message->header,
                         "WAITING FOR CAMERA-FASTLIO TF");
      return;
    }

    double odometry_delta_sec = std::numeric_limits<double>::infinity();
    const nav_msgs::Odometry* odometry =
        nearestOdometry(message->header.stamp, &odometry_delta_sec);
    if (odometry == nullptr || odometry_delta_sec > max_image_odom_delta_sec_) {
      target_filter_.reset();
      ROS_WARN_THROTTLE(1.0,
                        "front_aruco_hint: D435图像与FAST-LIO不同步 %.3fs",
                        odometry_delta_sec);
      annotateAndPublish(tracker_.debugImage(), message->header,
                         "WAITING FOR SYNCED FAST-LIO ODOM");
      return;
    }
    collectStableCandidates(message->header, camera_to_body, *odometry);

    // 候选累计与旧的单目标提示彼此独立。即使跟踪器仍锁定先看到的
    // ID、当前画面只剩另一个 ID，也必须继续把后者跨偏航扫描累计。
    if (!observation.valid || tracker_.lockedId() < 0) {
      target_filter_.reset();
      annotateAndPublish(tracker_.debugImage(), message->header,
                         "ACCUMULATING FRONT ARUCO CANDIDATES");
      return;
    }

    double measured_depth = std::numeric_limits<double>::quiet_NaN();
    if (!validateDepth(observation, message->header.stamp, &measured_depth)) {
      target_filter_.reset();
      publishStatus("前视ArUco等待D435深度一致性验证");
      annotateAndPublish(tracker_.debugImage(), message->header,
                         "WAITING FOR DEPTH VALIDATION");
      return;
    }

    geometry_msgs::PointStamped point_camera;
    point_camera.header = message->header;
    if (point_camera.header.frame_id.empty()) {
      point_camera.header.frame_id = camera_optical_frame_;
    }
    point_camera.point.x = observation.position_camera.x();
    point_camera.point.y = observation.position_camera.y();
    point_camera.point.z = observation.position_camera.z();
    geometry_msgs::PointStamped point_body;
    tf2::doTransform(point_camera, point_body, camera_to_body);
    const geometry_msgs::Point& body_position = odometry->pose.pose.position;
    const geometry_msgs::Quaternion& body_attitude =
        odometry->pose.pose.orientation;
    Eigen::Quaterniond body_orientation_world(
        body_attitude.w, body_attitude.x, body_attitude.y, body_attitude.z);
    if (!body_orientation_world.coeffs().array().isFinite().all() ||
        body_orientation_world.norm() < 1.0e-6) {
      target_filter_.reset();
      return;
    }
    body_orientation_world.normalize();
    const Eigen::Vector3d body_point(point_body.point.x, point_body.point.y,
                                     point_body.point.z);
    const Eigen::Vector3d world =
        body_orientation_world * body_point +
        Eigen::Vector3d(body_position.x, body_position.y, body_position.z);
    if (!target_filter_.add(observation.id, world,
                            message->header.stamp.toSec())) {
      std::ostringstream text;
      text << "STABILIZING FRONT HINT " << std::fixed << std::setprecision(2)
           << target_filter_.spread() << "m";
      annotateAndPublish(tracker_.debugImage(), message->header, text.str());
      return;
    }

    const Eigen::Vector3d filtered = target_filter_.filteredPoint();
    geometry_msgs::PoseStamped hint;
    hint.header.stamp = message->header.stamp;
    hint.header.frame_id = output_world_frame_;
    hint.pose.position.x = filtered.x();
    hint.pose.position.y = filtered.y();
    hint.pose.position.z = filtered.z();
    hint.pose.orientation.w = 1.0;
    hint_publisher_.publish(hint);

    std::ostringstream status;
    status << "前视平台粗定位 ID=" << observation.id << " world=("
           << std::fixed << std::setprecision(2) << filtered.x() << ", "
           << filtered.y() << ", " << filtered.z() << ")";
    if (std::isfinite(measured_depth)) {
      status << " depth=" << measured_depth << "m";
    }
    publishStatus(status.str());
    annotateAndPublish(tracker_.debugImage(), message->header,
                       "FRONT ARUCO HINT READY");
  }

  void publishLockedId() {
    std_msgs::Int32 message;
    message.data = tracker_.lockedId();
    locked_id_publisher_.publish(message);
  }

  void publishStatus(const std::string& text) {
    std_msgs::String message;
    message.data = text;
    status_publisher_.publish(message);
  }

  void annotateAndPublish(const cv::Mat& input,
                          const std_msgs::Header& header,
                          const std::string& text) {
    if (input.empty() || debug_image_publisher_.getNumSubscribers() == 0U) return;
    cv::Mat output = input.clone();
    cv::putText(output, text, cv::Point(10, std::max(25, output.rows - 18)),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 0), 2,
                cv::LINE_AA);
    debug_image_publisher_.publish(
        cv_bridge::CvImage(header, "bgr8", output).toImageMsg());
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  image_transport::ImageTransport image_transport_;
  ArucoTracker tracker_;
  WorldTargetFilter target_filter_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  image_transport::Subscriber image_subscriber_;
  ros::Subscriber camera_info_subscriber_;
  ros::Subscriber depth_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber mission_status_subscriber_;
  ros::Subscriber search_state_subscriber_;
  ros::Publisher hint_publisher_;
  ros::Publisher candidates_publisher_;
  ros::Publisher locked_id_publisher_;
  ros::Publisher status_publisher_;
  image_transport::Publisher debug_image_publisher_;

  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  cv::Mat latest_depth_m_;
  ros::Time latest_depth_stamp_;
  std::deque<nav_msgs::Odometry> odometry_history_;
  WorldTargetFilterConfig candidate_filter_config_;
  std::map<int, std::unique_ptr<WorldTargetFilter>> candidate_filters_;
  std::map<int, precision_landing::LandingPlatform> stable_candidates_;
  bool camera_info_received_{false};
  bool require_stage_gate_{true};
  bool require_search_state_gate_{true};
  bool front_scan_active_{false};
  bool require_depth_{true};
  int requested_marker_id_{-1};
  int depth_search_radius_{5};
  double image_timeout_sec_{0.30};
  double max_header_future_sec_{0.05};
  double tf_timeout_sec_{0.05};
  double max_image_odom_delta_sec_{0.11};
  double depth_scale_16uc1_{0.001};
  double max_depth_time_delta_sec_{0.10};
  double min_depth_m_{0.25};
  double max_depth_m_{6.0};
  double max_depth_absolute_error_m_{0.35};
  double max_depth_relative_error_{0.20};
  std::string body_frame_;
  std::string output_world_frame_;
  std::string camera_optical_frame_;
  std::string mission_stage_;
  std::string landing_search_state_;
};

}  // namespace
}  // namespace precision_landing

int main(int argc, char** argv) {
  ros::init(argc, argv, "front_aruco_hint_node");
  try {
    precision_landing::FrontArucoHintNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("front_aruco_hint启动失败: %s", error.what());
    return 1;
  }
  return 0;
}
