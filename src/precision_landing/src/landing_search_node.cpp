#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "precision_landing/LandingPlatformArray.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <map>

#include "precision_landing/aruco_tracker.hpp"
#include "precision_landing/landing_search_core.hpp"

namespace precision_landing {
namespace {

bool finite(double value) { return std::isfinite(value); }

std::string firstToken(const std::string &text) {
  const std::size_t separator = text.find_first_of(" \t\r\n");
  return text.substr(0U, separator);
}

std::vector<double> requiredVector(ros::NodeHandle &node,
                                   const std::string &name,
                                   std::size_t expected_size) {
  std::vector<double> values;
  if (!node.getParam(name, values) || values.size() != expected_size ||
      !std::all_of(values.begin(), values.end(), finite)) {
    throw std::runtime_error("~" + name + " must contain " +
                             std::to_string(expected_size) + " finite values");
  }
  return values;
}

bool validQuaternion(const geometry_msgs::Quaternion &value) {
  const double norm = std::sqrt(value.x * value.x + value.y * value.y +
                                value.z * value.z + value.w * value.w);
  return finite(norm) && norm > 1.0e-6;
}

} // namespace

class LandingSearchNode {
public:
  LandingSearchNode()
      : private_node_("~"), image_transport_(node_),
        tracker_(loadTrackerConfig()), target_filter_(loadFilterConfig()) {
    loadConfiguration();
    configureInterfaces();
    publishLandingTrigger(false);
    publishStatus("等待任务进入门外降落平台搜索阶段");
    request_timer_ = node_.createTimer(
        ros::Duration(0.05), &LandingSearchNode::requestTimerCallback, this);
    ROS_INFO("landing_search ready: marker=%d, output=%s, trigger=%s",
             requested_marker_id_, marker_world_topic_.c_str(),
             landing_trigger_topic_.c_str());
  }

private:
  ArucoTrackerConfig loadTrackerConfig() {
    ArucoTrackerConfig config;
    private_node_.param("marker/size_m", config.marker_size_m, 0.60);
    private_node_.param("marker/max_reprojection_error_px",
                        config.max_reprojection_error_px, 4.0);
    private_node_.param("marker/min_distance_m", config.min_distance_m, 0.15);
    private_node_.param("marker/max_distance_m", config.max_distance_m, 6.0);
    private_node_.param("marker/max_position_jump_m",
                        config.max_position_jump_m, 0.60);
    private_node_.param("marker/max_tilt_deg", config.max_tilt_deg, 60.0);
    private_node_.param("marker/stable_frames", config.stable_frames, 5);
    return config;
  }

  WorldTargetFilterConfig loadFilterConfig() {
    WorldTargetFilterConfig config;
    int stable_samples = 8;
    private_node_.param("world_filter/stable_samples", stable_samples, 8);
    config.stable_samples =
        static_cast<std::size_t>(std::max(1, stable_samples));
    private_node_.param("world_filter/max_sample_gap_sec",
                        config.max_sample_gap_sec, 0.25);
    private_node_.param("world_filter/max_position_jump_m",
                        config.max_position_jump_m, 0.35);
    private_node_.param("world_filter/max_spread_m", config.max_spread_m, 0.12);
    return config;
  }

  void loadConfiguration() {
    private_node_.param("marker/requested_id", requested_marker_id_, -1);
    private_node_.param("safety/max_image_odom_delta_sec",
                        max_image_odom_delta_sec_, 0.11);
    private_node_.param("safety/image_timeout_sec", image_timeout_sec_, 0.30);
    private_node_.param("safety/odom_timeout_sec", odom_timeout_sec_, 0.30);
    private_node_.param("safety/target_timeout_sec", target_timeout_sec_, 0.40);
    private_node_.param("safety/max_header_future_sec", max_header_future_sec_,
                        0.05);
    private_node_.param("handoff/max_xy_error_m", handoff_max_xy_error_m_,
                        0.35);
    private_node_.param("handoff/approach_height_m", approach_height_m_, 0.65);
    private_node_.param("handoff/max_height_error_m", max_height_error_m_,
                        0.25);
    private_node_.param("mission/require_stage_gate", require_stage_gate_,
                        true);
    private_node_.param("mission/require_platform_assignment",
                        require_platform_assignment_, false);
    private_node_.param("mission/early_handoff_on_assigned_marker",
                        early_handoff_on_assigned_marker_, false);
    private_node_.param("frames/world", world_frame_, std::string("world"));

    if (max_image_odom_delta_sec_ <= 0.0 || image_timeout_sec_ <= 0.0 ||
        odom_timeout_sec_ <= 0.0 || target_timeout_sec_ <= 0.0 ||
        max_header_future_sec_ < 0.0 || handoff_max_xy_error_m_ <= 0.0 ||
        approach_height_m_ <= 0.0 || max_height_error_m_ <= 0.0) {
      throw std::runtime_error("landing_search safety values must be positive");
    }

    const std::vector<double> translation =
        requiredVector(private_node_, "camera_to_body/translation_m", 3U);
    const std::vector<double> quaternion =
        requiredVector(private_node_, "camera_to_body/quaternion_xyzw", 4U);
    camera_translation_body_ =
        Eigen::Vector3d(translation[0], translation[1], translation[2]);
    camera_orientation_body_ = Eigen::Quaterniond(quaternion[3], quaternion[0],
                                                  quaternion[1], quaternion[2]);
    const double quaternion_norm = camera_orientation_body_.norm();
    if (!finite(quaternion_norm) || quaternion_norm < 1.0e-6 ||
        std::abs(quaternion_norm - 1.0) > 0.02) {
      throw std::runtime_error(
          "~camera_to_body/quaternion_xyzw must be a unit quaternion");
    }
    camera_orientation_body_.normalize();
  }

  void configureInterfaces() {
    std::string image_topic{"/UAV0/down_camera/image_raw"};
    std::string camera_info_topic{"/UAV0/down_camera/camera_info"};
    std::string odom_topic{"/UAV0/fast_lio/Odom_high_freq"};
    std::string mission_status_topic{"/UAV0/mission/task_status"};
    std::string landing_request_topic{"/UAV0/mission/landing_request"};
    std::string early_handoff_authorization_topic{
        "/dual_uav_landing/release_uav1"};
    std::string debug_image_topic{"/UAV0/landing/search/debug_image"};
    std::string status_topic{"/UAV0/landing/search/status"};
    std::string locked_id_topic{"/UAV0/landing/search/locked_id"};
    std::string target_id_topic{"/UAV0/landing/target_id"};
    std::string assigned_id_topic{"/UAV0/landing/assigned_id"};
    std::string excluded_id_topic{"/UAV0/landing/excluded_id"};
    std::string candidates_topic{"/UAV0/landing/search/candidates"};
    std::string filtered_target_topic{"/UAV0/landing/search/target_world"};
    private_node_.param("topics/image", image_topic, image_topic);
    private_node_.param("topics/camera_info", camera_info_topic,
                        camera_info_topic);
    private_node_.param("topics/odometry", odom_topic, odom_topic);
    private_node_.param("topics/mission_status", mission_status_topic,
                        mission_status_topic);
    private_node_.param("topics/marker_world", marker_world_topic_,
                        std::string("/UAV0/mission/detection/final_aruco"));
    private_node_.param("topics/landing_request", landing_request_topic,
                        landing_request_topic);
    private_node_.param("topics/early_handoff_authorization",
                        early_handoff_authorization_topic,
                        early_handoff_authorization_topic);
    private_node_.param("topics/landing_trigger", landing_trigger_topic_,
                        std::string("/UAV0/need_to_land"));
    private_node_.param("topics/debug_image", debug_image_topic,
                        debug_image_topic);
    private_node_.param("topics/status", status_topic, status_topic);
    private_node_.param("topics/locked_id", locked_id_topic, locked_id_topic);
    private_node_.param("topics/target_id", target_id_topic, target_id_topic);
    private_node_.param("topics/assigned_id", assigned_id_topic,
                        assigned_id_topic);
    private_node_.param("topics/excluded_id", excluded_id_topic,
                        excluded_id_topic);
    private_node_.param("topics/candidates", candidates_topic,
                        candidates_topic);
    private_node_.param("topics/filtered_target", filtered_target_topic,
                        filtered_target_topic);

    image_subscriber_ = image_transport_.subscribe(
        image_topic, 1, &LandingSearchNode::imageCallback, this);
    camera_info_subscriber_ = node_.subscribe(
        camera_info_topic, 1, &LandingSearchNode::cameraInfoCallback, this);
    odom_subscriber_ = node_.subscribe(odom_topic, 100,
                                       &LandingSearchNode::odomCallback, this);
    mission_status_subscriber_ =
        node_.subscribe(mission_status_topic, 5,
                        &LandingSearchNode::missionStatusCallback, this);
    landing_request_subscriber_ =
        node_.subscribe(landing_request_topic, 2,
                        &LandingSearchNode::landingRequestCallback, this);
    early_handoff_authorization_subscriber_ = node_.subscribe(
        early_handoff_authorization_topic, 2,
        &LandingSearchNode::earlyHandoffAuthorizationCallback, this);
    assigned_id_subscriber_ =
        node_.subscribe(assigned_id_topic, 2,
                        &LandingSearchNode::assignedIdCallback, this);
    excluded_id_subscriber_ =
        node_.subscribe(excluded_id_topic, 2,
                        &LandingSearchNode::excludedIdCallback, this);
    candidates_publisher_ =
        node_.advertise<precision_landing::LandingPlatformArray>(
            candidates_topic, 2, true);

    marker_world_publisher_ =
        node_.advertise<geometry_msgs::PoseStamped>(marker_world_topic_, 5);
    filtered_target_publisher_ = node_.advertise<geometry_msgs::PoseStamped>(
        filtered_target_topic, 2, true);
    landing_trigger_publisher_ =
        node_.advertise<std_msgs::Bool>(landing_trigger_topic_, 1, true);
    status_publisher_ =
        node_.advertise<std_msgs::String>(status_topic, 1, true);
    locked_id_publisher_ =
        node_.advertise<std_msgs::Int32>(locked_id_topic, 1, true);
    target_id_publisher_ =
        node_.advertise<std_msgs::Int32>(target_id_topic, 1, true);
    debug_image_publisher_ = image_transport_.advertise(debug_image_topic, 1);
  }

  bool stageAllowsDetection() const {
    if (!require_stage_gate_) {
      return true;
    }
    return mission_stage_ == "SEARCH_OUTSIDE_LANDING" ||
           mission_stage_ == "SEARCH_OUTSIDE_QR" ||
           mission_stage_ == "APPROACH_LANDING" || mission_stage_ == "LANDING";
  }

  bool stageAllowsHandoff() const {
    return !require_stage_gate_ || mission_stage_ == "SEARCH_OUTSIDE_LANDING" ||
           mission_stage_ == "SEARCH_OUTSIDE_QR" ||
           mission_stage_ == "APPROACH_LANDING" ||
           mission_stage_ == "LANDING";
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &message) {
    if (message->K[0] <= 0.0 || message->K[4] <= 0.0) {
      ROS_WARN_THROTTLE(2.0, "landing_search: invalid CameraInfo intrinsics");
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

  void odomCallback(const nav_msgs::OdometryConstPtr &message) {
    const double age_sec = (ros::Time::now() - message->header.stamp).toSec();
    if (message->header.stamp.isZero() || age_sec < -max_header_future_sec_ ||
        !validQuaternion(message->pose.pose.orientation)) {
      ROS_WARN_THROTTLE(2.0,
                        "landing_search: reject invalid FAST-LIO odometry");
      return;
    }
    odom_history_.push_back(*message);
    while (odom_history_.size() > 200U) {
      odom_history_.pop_front();
    }
  }

  void missionStatusCallback(const std_msgs::StringConstPtr &message) {
    const std::string next_stage = firstToken(message->data);
    if (next_stage != mission_stage_) {
      mission_stage_ = next_stage;
      tracker_.reset();
      target_filter_.reset();
      candidate_records_.clear();
      ROS_INFO("landing_search: mission stage -> %s", mission_stage_.c_str());
    }
  }

  void landingRequestCallback(const std_msgs::BoolConstPtr &message) {
    landing_request_active_ = message->data;
    if (!message->data && !trigger_sent_) {
      publishLandingTrigger(false);
    }
  }

  void earlyHandoffAuthorizationCallback(
      const std_msgs::BoolConstPtr &message) {
    early_handoff_authorized_ = message->data;
  }

  void assignedIdCallback(const std_msgs::Int32ConstPtr &message) {
    if (message->data < -1) {
      ROS_WARN("landing_search: reject invalid assigned platform ID=%d",
               message->data);
      return;
    }
    if (trigger_sent_) {
      if (message->data != requested_marker_id_) {
        ROS_ERROR("landing_search: ignore platform reassignment %d -> %d after precision handoff",
                  requested_marker_id_, message->data);
      }
      return;
    }
    if (message->data == requested_marker_id_) {
      return;
    }

    if (message->data >= 0 && tracker_.lockedId() == message->data) {
      requested_marker_id_ = message->data;
      publishTargetId(requested_marker_id_);
      ROS_INFO("landing_search: confirmed current locked platform ID=%d",
               requested_marker_id_);
      publishStatus("当前锁定平台已由双机协调器确认");
      return;
    }

    requested_marker_id_ = message->data;
    tracker_.reset();
    target_filter_.reset();
    candidate_records_.clear();
    stable_target_received_ = false;
    last_stable_target_ = geometry_msgs::PoseStamped();
    last_stable_target_receive_time_ = ros::Time(0);
    publishLockedId();
    publishTargetId(requested_marker_id_);
    if (requested_marker_id_ >= 0) {
      ROS_WARN("landing_search: assigned platform ID=%d; old target cleared",
               requested_marker_id_);
      publishStatus("已接收双机平台分配，等待指定 ArUco");
    } else {
      ROS_WARN("landing_search: platform assignment cleared; automatic selection enabled");
      publishStatus("平台分配已清除，恢复自动选择");
    }
  }

  void excludedIdCallback(const std_msgs::Int32ConstPtr &message) {
    if (message->data < -1) {
      ROS_WARN("landing_search: reject invalid excluded platform ID=%d",
               message->data);
      return;
    }
    if (trigger_sent_ || message->data == excluded_marker_id_) {
      return;
    }

    excluded_marker_id_ = message->data;
    if (requested_marker_id_ >= 0 &&
        requested_marker_id_ == excluded_marker_id_) {
      requested_marker_id_ = -1;
      publishTargetId(-1);
    }
    if (tracker_.lockedId() == excluded_marker_id_) {
      tracker_.reset();
      target_filter_.reset();
      candidate_records_.clear();
      stable_target_received_ = false;
      last_stable_target_ = geometry_msgs::PoseStamped();
      last_stable_target_receive_time_ = ros::Time(0);
      publishLockedId();
      publishStatus("已排除另一架无人机占用的平台，继续搜索");
    }
    ROS_INFO("landing_search: excluded platform ID=%d", excluded_marker_id_);
  }

  const nav_msgs::Odometry *nearestOdometry(const ros::Time &stamp,
                                            double *delta_sec) const {
    const nav_msgs::Odometry *nearest = nullptr;
    double best_delta = std::numeric_limits<double>::infinity();
    for (const nav_msgs::Odometry &odometry : odom_history_) {
      const double delta = std::abs((odometry.header.stamp - stamp).toSec());
      if (delta < best_delta) {
        best_delta = delta;
        nearest = &odometry;
      }
    }
    if (delta_sec != nullptr) {
      *delta_sec = best_delta;
    }
    return nearest;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &message) {
    const double image_age_sec =
        (ros::Time::now() - message->header.stamp).toSec();
    if (!camera_info_received_ || message->header.stamp.isZero() ||
        image_age_sec < -max_header_future_sec_ ||
        image_age_sec > image_timeout_sec_) {
      ROS_WARN_THROTTLE(
          2.0, "landing_search: waiting for stamped image and CameraInfo");
      return;
    }

    cv_bridge::CvImageConstPtr image;
    try {
      image = cv_bridge::toCvShare(message, "bgr8");
    } catch (const cv_bridge::Exception &error) {
      ROS_WARN_THROTTLE(2.0, "landing_search: image conversion failed: %s",
                        error.what());
      return;
    }

    if (!stageAllowsDetection()) {
      annotateAndPublish(image->image, "WAITING FOR CHANNEL EXIT");
      return;
    }

    const TargetObservation observation =
        tracker_.process(image->image, camera_matrix_, distortion_,
                         message->header.stamp.toSec(), requested_marker_id_,
                         true, excluded_marker_id_);
    publishLockedId();

    double odom_delta_sec = std::numeric_limits<double>::infinity();
    const nav_msgs::Odometry *odometry =
        nearestOdometry(message->header.stamp, &odom_delta_sec);
    if (odometry == nullptr || odom_delta_sec > max_image_odom_delta_sec_) {
      target_filter_.reset();
      ROS_WARN_THROTTLE(1.0,
                        "landing_search: image/odometry unsynchronized (%.3fs)",
                        odom_delta_sec);
      annotateAndPublish(tracker_.debugImage(), "WAITING FOR SYNCED FAST-LIO ODOM");
      return;
    }

    publishCandidates(*message, *odometry);

    if (!observation.valid || tracker_.lockedId() < 0 ||
        (require_platform_assignment_ && requested_marker_id_ < 0)) {
      target_filter_.reset();
      annotateAndPublish(
          tracker_.debugImage(),
          require_platform_assignment_ && requested_marker_id_ < 0
              ? "COLLECTING TWO LANDING PLATFORMS"
              : "SEARCHING FOR LANDING ARUCO");
      return;
    }

    const geometry_msgs::Point &body_position = odometry->pose.pose.position;
    const geometry_msgs::Quaternion &body_attitude =
        odometry->pose.pose.orientation;
    const Eigen::Vector3d point_world = cameraPointToWorld(
        observation.position_camera, camera_translation_body_,
        camera_orientation_body_,
        Eigen::Vector3d(body_position.x, body_position.y, body_position.z),
        Eigen::Quaterniond(body_attitude.w, body_attitude.x, body_attitude.y,
                           body_attitude.z));

    if (!target_filter_.add(observation.id, point_world,
                            message->header.stamp.toSec())) {
      std::ostringstream state;
      state << "STABILIZING WORLD TARGET spread=" << std::fixed
            << std::setprecision(3)
            << target_filter_.spread() << "m";
      annotateAndPublish(tracker_.debugImage(), state.str());
      return;
    }

    const Eigen::Vector3d filtered = target_filter_.filteredPoint();
    geometry_msgs::PoseStamped target;
    target.header.stamp = message->header.stamp;
    target.header.frame_id = world_frame_;
    target.pose.position.x = filtered.x();
    target.pose.position.y = filtered.y();
    target.pose.position.z = filtered.z();
    target.pose.orientation.w = 1.0;
    marker_world_publisher_.publish(target);
    filtered_target_publisher_.publish(target);
    publishTargetId(observation.id);
    last_stable_target_ = target;
    last_stable_target_receive_time_ = ros::Time::now();
    stable_target_received_ = true;

    std::ostringstream state;
    state << "平台已锁定 ID=" << observation.id << " world=(" << std::fixed
          << std::setprecision(2) << filtered.x() << ", " << filtered.y()
          << ", " << filtered.z() << ")";
    publishStatus(state.str());
    std::ostringstream image_state;
    image_state << "TARGET ID=" << observation.id << " world=(" << std::fixed
                << std::setprecision(2) << filtered.x() << ", " << filtered.y()
                << ", " << filtered.z() << ")";
    annotateAndPublish(tracker_.debugImage(), image_state.str());

    // UAV1 已被 UAV0 释放后，只要下视稳定识别到分配的同一平台 ID，
    // 就立即把控制权交给精降，不再继续追踪 UAV0 提供的粗平台航点。
    // UAV0 默认关闭此功能，仍维持“到达粗航点后请求精降”的原流程。
    if (early_handoff_on_assigned_marker_ && early_handoff_authorized_ &&
        !trigger_sent_ && stageAllowsHandoff() && requested_marker_id_ >= 0 &&
        observation.id == requested_marker_id_) {
      trigger_sent_ = true;
      publishLandingTrigger(true);
      publishStatus("已识别到分配的同码 ArUco，提前由精确降落接管");
      ROS_ERROR("landing_search: EARLY HANDOFF TO PRECISION LANDING, "
                "assigned_id=%d world=(%.3f, %.3f, %.3f)",
                requested_marker_id_, filtered.x(), filtered.y(), filtered.z());
    }
  }

  void requestTimerCallback(const ros::TimerEvent &) {
    if (!landing_request_active_ || trigger_sent_) {
      return;
    }
    if (!stageAllowsHandoff()) {
      publishStatus("拒绝降落接管：任务尚未进入平台接近阶段");
      return;
    }
    const ros::Time now = ros::Time::now();
    if (!stable_target_received_ ||
        (now - last_stable_target_receive_time_).toSec() >
            target_timeout_sec_) {
      publishStatus("拒绝降落接管：ArUco 世界位置已过期");
      return;
    }
    const double odom_age_sec =
        odom_history_.empty()
            ? std::numeric_limits<double>::infinity()
            : (now - odom_history_.back().header.stamp).toSec();
    if (odom_history_.empty() || odom_age_sec < -max_header_future_sec_ ||
        odom_age_sec > odom_timeout_sec_) {
      publishStatus("拒绝降落接管：FAST-LIO 里程计已过期");
      return;
    }

    const geometry_msgs::Point &current =
        odom_history_.back().pose.pose.position;
    const geometry_msgs::Point &target = last_stable_target_.pose.position;
    const double xy_error =
        std::hypot(current.x - target.x, current.y - target.y);
    const double target_approach_z = target.z + approach_height_m_;
    const double height_error = std::abs(current.z - target_approach_z);
    if (xy_error > handoff_max_xy_error_m_ ||
        height_error > max_height_error_m_) {
      ROS_WARN_THROTTLE(
          1.0,
          "landing_search: handoff rejected, xy=%.3f/%.3f height=%.3f/%.3f",
          xy_error, handoff_max_xy_error_m_, height_error, max_height_error_m_);
      publishStatus("等待规划器到达平台正上方");
      return;
    }

    trigger_sent_ = true;
    publishLandingTrigger(true);
    publishStatus("已到达平台正上方，精确降落接管");
    ROS_ERROR("landing_search: HANDOFF TO PRECISION LANDING, xy=%.3fm "
              "height_error=%.3fm",
              xy_error, height_error);
  }

  void publishLandingTrigger(bool active) {
    std_msgs::Bool message;
    message.data = active;
    landing_trigger_publisher_.publish(message);
  }

  void publishLockedId() {
    std_msgs::Int32 message;
    message.data = tracker_.lockedId();
    locked_id_publisher_.publish(message);
  }

  void publishCandidates(const sensor_msgs::Image& image,
                         const nav_msgs::Odometry& odometry) {
    precision_landing::LandingPlatformArray message;
    message.header.stamp = image.header.stamp;
    message.header.frame_id = world_frame_;
    const Eigen::Vector3d body_position(
        odometry.pose.pose.position.x, odometry.pose.pose.position.y,
        odometry.pose.pose.position.z);
    const Eigen::Quaterniond body_attitude(
        odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.x,
        odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z);
    for (const DetectedTarget& candidate : tracker_.detectedTargets()) {
      const Eigen::Vector3d world_position = cameraPointToWorld(
          candidate.position_camera, camera_translation_body_,
          camera_orientation_body_, body_position, body_attitude);
      CandidateRecord& record = candidate_records_[candidate.id];
      const double gap_sec = record.last_seen.isZero()
                                 ? 0.0
                                 : (message.header.stamp - record.last_seen).toSec();
      if (gap_sec > 0.30) record.count = 0;
      if (record.count == 0 ||
          (world_position - record.position_world).norm() <= 0.35) {
        record.position_world = world_position;
        record.score = candidate.score;
        ++record.count;
      } else {
        record.count = 1;
        record.position_world = world_position;
        record.score = candidate.score;
      }
      record.last_seen = message.header.stamp;
    }
    for (const auto& item : candidate_records_) {
      const int id = item.first;
      const CandidateRecord& record = item.second;
      if (record.count < 5) continue;
      precision_landing::LandingPlatform platform;
      platform.header = message.header;
      platform.id = id;
      platform.pose.header = message.header;
      platform.pose.pose.position.x = record.position_world.x();
      platform.pose.pose.position.y = record.position_world.y();
      platform.pose.pose.position.z = record.position_world.z();
      platform.score = record.score;
      message.platforms.push_back(platform);
    }
    candidates_publisher_.publish(message);
  }

  void publishTargetId(int marker_id) {
    std_msgs::Int32 message;
    message.data = marker_id;
    target_id_publisher_.publish(message);
  }

  void publishStatus(const std::string &status) {
    if (status == last_status_) {
      return;
    }
    last_status_ = status;
    std_msgs::String message;
    message.data = status;
    status_publisher_.publish(message);
  }

  void annotateAndPublish(const cv::Mat &input, const std::string &text) {
    if (input.empty()) {
      return;
    }
    cv::Mat annotated = input.clone();
    cv::putText(annotated, text, cv::Point(10, annotated.rows - 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(255, 255, 0), 1,
                cv::LINE_AA);
    cv_bridge::CvImage debug;
    debug.header.stamp = ros::Time::now();
    debug.header.frame_id = world_frame_;
    debug.encoding = "bgr8";
    debug.image = annotated;
    debug_image_publisher_.publish(debug.toImageMsg());
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  image_transport::ImageTransport image_transport_;
  ArucoTracker tracker_;
  WorldTargetFilter target_filter_;

  image_transport::Subscriber image_subscriber_;
  ros::Subscriber camera_info_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber mission_status_subscriber_;
  ros::Subscriber landing_request_subscriber_;
  ros::Subscriber early_handoff_authorization_subscriber_;
  ros::Subscriber assigned_id_subscriber_;
  ros::Subscriber excluded_id_subscriber_;
  ros::Publisher marker_world_publisher_;
  ros::Publisher filtered_target_publisher_;
  ros::Publisher landing_trigger_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher candidates_publisher_;
  ros::Publisher locked_id_publisher_;
  ros::Publisher target_id_publisher_;
  image_transport::Publisher debug_image_publisher_;
  ros::Timer request_timer_;

  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  std::deque<nav_msgs::Odometry> odom_history_;
  geometry_msgs::PoseStamped last_stable_target_;
  ros::Time last_stable_target_receive_time_;
  Eigen::Vector3d camera_translation_body_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond camera_orientation_body_{Eigen::Quaterniond::Identity()};

  std::string marker_world_topic_;
  std::string landing_trigger_topic_;
  std::string world_frame_{"world"};
  std::string mission_stage_;
  std::string last_status_;
  int requested_marker_id_{-1};
  int excluded_marker_id_{-1};
  struct CandidateRecord {
    Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
    double score{0.0};
    int count{0};
    ros::Time last_seen;
  };
  std::map<int, CandidateRecord> candidate_records_;
  double max_image_odom_delta_sec_{0.11};
  double image_timeout_sec_{0.30};
  double odom_timeout_sec_{0.30};
  double target_timeout_sec_{0.40};
  double max_header_future_sec_{0.05};
  double handoff_max_xy_error_m_{0.35};
  double approach_height_m_{0.65};
  double max_height_error_m_{0.25};
  bool require_stage_gate_{true};
  bool require_platform_assignment_{false};
  bool early_handoff_on_assigned_marker_{false};
  bool camera_info_received_{false};
  bool stable_target_received_{false};
  bool landing_request_active_{false};
  bool early_handoff_authorized_{false};
  bool trigger_sent_{false};
};

} // namespace precision_landing

int main(int argc, char **argv) {
  ros::init(argc, argv, "landing_search_node");
  try {
    precision_landing::LandingSearchNode node;
    ros::spin();
  } catch (const std::exception &error) {
    ROS_FATAL("landing_search startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
