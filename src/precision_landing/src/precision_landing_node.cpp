#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcValue.h>

#include "precision_landing/aruco_tracker.hpp"
#include "precision_landing/landing_controller.hpp"
#include "precision_landing/landing_state_machine.hpp"
#include "precision_landing/mode_request_authorization.hpp"
#include "precision_landing/types.hpp"

namespace precision_landing {
namespace {

template <typename T>
void loadParameter(ros::NodeHandle& private_node,
                   const std::string& name,
                   T& value) {
  private_node.param(name, value, value);
}

template <typename T>
void loadParameter(ros::NodeHandle& private_node,
                   const std::string& canonical_name,
                   const std::string& legacy_name,
                   T& value) {
  if (private_node.hasParam(canonical_name)) {
    private_node.getParam(canonical_name, value);
    return;
  }
  loadParameter(private_node, legacy_name, value);
}

double xmlRpcNumber(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  throw std::runtime_error("rotation entries must be numeric");
}

void requireFinitePositive(double value, const std::string& parameter) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error("~" + parameter +
                             " must be finite and positive");
  }
}

void requireFiniteNonNegative(double value, const std::string& parameter) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::runtime_error("~" + parameter +
                             " must be finite and non-negative");
  }
}

void validateControllerConfig(const ControllerConfig& config) {
  requireFinitePositive(config.kp_xy, "control/kp_xy");
  requireFinitePositive(config.max_xy_speed, "control/max_xy_speed_mps");
  requireFinitePositive(config.max_xy_accel, "control/max_xy_accel_mps2");
  requireFiniteNonNegative(config.error_deadband, "control/error_deadband_m");
  if (!std::isfinite(config.filter_alpha) ||
      config.filter_alpha < 0.0 || config.filter_alpha > 1.0) {
    throw std::runtime_error("~control/filter_alpha must be in [0, 1]");
  }
}

ControllerConfig loadControllerConfig(ros::NodeHandle& private_node) {
  ControllerConfig config;
  loadParameter(private_node, "control/kp_xy", "controller/kp_xy", config.kp_xy);
  loadParameter(private_node, "control/max_xy_speed_mps",
                "controller/max_xy_speed", config.max_xy_speed);
  loadParameter(private_node, "control/max_xy_accel_mps2",
                "controller/max_xy_accel", config.max_xy_accel);
  loadParameter(private_node, "control/error_deadband_m",
                "controller/error_deadband", config.error_deadband);
  loadParameter(private_node, "control/filter_alpha", "controller/filter_alpha",
                config.filter_alpha);
  validateControllerConfig(config);
  return config;
}

void validateStateMachineConfig(const StateMachineConfig& config) {
  requireFinitePositive(config.acquire_stable_sec,
                        "state_machine/acquire_stable_sec");
  requireFinitePositive(config.align_stable_sec,
                        "stages/auto_land/stable_sec");
  requireFinitePositive(config.align_error_m,
                        "state_machine/align_error_m");
  requireFinitePositive(config.high_height_m, "stages/high/min_height_m");
  requireFinitePositive(config.auto_land_height_m,
                        "stages/auto_land/height_m");
  requireFinitePositive(config.high_align_error_m, "stages/high/error_m");
  requireFinitePositive(config.auto_land_error_m,
                        "stages/auto_land/error_m");
  requireFinitePositive(config.high_descent_mps, "stages/high/descent_mps");
  requireFinitePositive(config.fixed_descent_mps,
                        "stages/fixed_xy_descent/descent_mps");
  requireFinitePositive(config.target_loss_timeout_sec,
                        "safety/target_loss_timeout_sec");
  requireFinitePositive(config.reacquire_timeout_sec,
                        "safety/reacquire_timeout_sec");
  requireFinitePositive(config.total_timeout_sec,
                        "safety/total_timeout_sec");

  if (std::abs(config.auto_land_height_m - config.high_height_m) > 1.0e-6) {
    throw std::runtime_error(
        "visual descent and fixed-XY descent must share the same handoff height");
  }
  if (config.auto_land_error_m > config.high_align_error_m) {
    throw std::runtime_error(
        "auto-land error gate must not exceed the high-stage error gate");
  }
  if (!(config.target_loss_timeout_sec <= config.reacquire_timeout_sec &&
        config.reacquire_timeout_sec <= config.total_timeout_sec)) {
    throw std::runtime_error(
        "landing timeouts must satisfy target_loss <= reacquire <= total");
  }
}

StateMachineConfig loadStateMachineConfig(ros::NodeHandle& private_node) {
  StateMachineConfig config;
  loadParameter(private_node, "state_machine/acquire_stable_sec",
                config.acquire_stable_sec);
  loadParameter(private_node, "stages/auto_land/stable_sec",
                "state_machine/align_stable_sec", config.align_stable_sec);
  loadParameter(private_node, "state_machine/align_error_m",
                config.align_error_m);
  loadParameter(private_node, "stages/high/min_height_m",
                "state_machine/high_height_m", config.high_height_m);
  loadParameter(private_node, "stages/auto_land/height_m",
                "state_machine/auto_land_height_m", config.auto_land_height_m);
  loadParameter(private_node, "stages/high/error_m",
                "state_machine/high_align_error_m", config.high_align_error_m);
  loadParameter(private_node, "stages/auto_land/error_m",
                "state_machine/auto_land_error_m", config.auto_land_error_m);
  loadParameter(private_node, "stages/high/descent_mps",
                "state_machine/high_descent_mps", config.high_descent_mps);
  loadParameter(private_node, "stages/fixed_xy_descent/descent_mps",
                config.fixed_descent_mps);
  loadParameter(private_node, "safety/target_loss_timeout_sec",
                "state_machine/target_loss_timeout_sec",
                config.target_loss_timeout_sec);
  loadParameter(private_node, "safety/reacquire_timeout_sec",
                "state_machine/reacquire_timeout_sec", config.reacquire_timeout_sec);
  loadParameter(private_node, "safety/total_timeout_sec",
                "state_machine/total_timeout_sec", config.total_timeout_sec);
  validateStateMachineConfig(config);
  return config;
}

void validateTrackerConfig(const ArucoTrackerConfig& config) {
  requireFinitePositive(config.marker_size_m, "marker/size_m");
  requireFinitePositive(config.max_reprojection_error_px,
                        "marker/max_reprojection_error_px");
  requireFinitePositive(config.min_distance_m, "marker/min_distance_m");
  requireFinitePositive(config.max_distance_m, "marker/max_distance_m");
  requireFinitePositive(config.max_position_jump_m,
                        "marker/max_position_jump_m");
  if (config.max_distance_m <= config.min_distance_m) {
    throw std::runtime_error(
        "~marker/max_distance_m must exceed marker/min_distance_m");
  }
  if (config.stable_frames < 1) {
    throw std::runtime_error("~marker/stable_frames must be positive");
  }
  if (!std::isfinite(config.max_tilt_deg) ||
      config.max_tilt_deg < 0.0 || config.max_tilt_deg > 90.0) {
    throw std::runtime_error("~marker/max_tilt_deg must be in [0, 90]");
  }
}

ArucoTrackerConfig loadTrackerConfig(ros::NodeHandle& private_node) {
  ArucoTrackerConfig config;
  loadParameter(private_node, "marker/size_m", config.marker_size_m);
  loadParameter(private_node, "marker/max_reprojection_error_px",
                config.max_reprojection_error_px);
  loadParameter(private_node, "marker/min_distance_m", config.min_distance_m);
  loadParameter(private_node, "marker/max_distance_m", config.max_distance_m);
  loadParameter(private_node, "marker/max_position_jump_m",
                config.max_position_jump_m);
  loadParameter(private_node, "marker/max_tilt_deg",
                config.max_tilt_deg);
  loadParameter(private_node, "marker/stable_frames", config.stable_frames);
  validateTrackerConfig(config);
  return config;
}

bool isControlState(LandingState state) {
  return state == LandingState::ACQUIRE ||
         state == LandingState::ALIGN ||
         state == LandingState::DESCEND_HIGH ||
         state == LandingState::FIXED_XY_DESCENT ||
         state == LandingState::DESCEND_MID ||
         state == LandingState::DESCEND_FINAL ||
         state == LandingState::REACQUIRE ||
         state == LandingState::REQUEST_AUTO_LAND ||
         state == LandingState::ABORT_HOLD;
}

bool isDescentState(LandingState state) {
  return state == LandingState::DESCEND_HIGH ||
         state == LandingState::FIXED_XY_DESCENT ||
         state == LandingState::DESCEND_MID ||
         state == LandingState::DESCEND_FINAL;
}

bool missionKeepsTrackerActive(LandingState state) {
  return state == LandingState::ALIGN ||
         isDescentState(state) ||
         state == LandingState::REACQUIRE ||
         state == LandingState::REQUEST_AUTO_LAND;
}

bool isFinite(double value) {
  return std::isfinite(value);
}

bool validCameraInfo(const sensor_msgs::CameraInfo& info) {
  if (info.width == 0U || info.height == 0U) {
    return false;
  }
  for (double entry : info.K) {
    if (!isFinite(entry)) {
      return false;
    }
  }
  for (double entry : info.D) {
    if (!isFinite(entry)) {
      return false;
    }
  }

  constexpr double kMinimumMagnitude = 1.0e-9;
  return info.K[0] > kMinimumMagnitude &&
         std::abs(info.K[2]) > kMinimumMagnitude &&
         info.K[4] > kMinimumMagnitude &&
         std::abs(info.K[5]) > kMinimumMagnitude &&
         std::abs(info.K[8]) > kMinimumMagnitude;
}

bool validPose(const geometry_msgs::PoseStamped& pose) {
  const auto& position = pose.pose.position;
  const auto& orientation = pose.pose.orientation;
  if (!isFinite(position.x) || !isFinite(position.y) ||
      !isFinite(position.z) || !isFinite(orientation.x) ||
      !isFinite(orientation.y) || !isFinite(orientation.z) ||
      !isFinite(orientation.w)) {
    return false;
  }
  const double quaternion_norm_squared =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
  return quaternion_norm_squared > 1.0e-12;
}

struct ModeRequestSharedState {
  std::mutex mutex;
  std::condition_variable condition;
  bool shutdown{false};
  ModeRequestAuthorization authorization;
  bool pending{false};
  bool in_flight{false};
  bool has_call_start{false};
  ModeRequestClock::time_point last_call_start;
  ModeRequestClock::duration minimum_call_start_interval{
      std::chrono::milliseconds(500)};
  ModeRequestClock::duration authorization_delay{
      std::chrono::milliseconds(50)};
};

}  // namespace

class PrecisionLandingNode {
 public:
  PrecisionLandingNode()
      : node_(),
        private_node_("~"),
        image_transport_(node_),
        controller_(loadControllerConfig(private_node_)),
        state_machine_(loadStateMachineConfig(private_node_)),
        tracker_(loadTrackerConfig(private_node_)) {
    loadRuntimeParameters();
    configureRosInterfaces();
    control_timer_ = node_.createTimer(
        ros::Duration(1.0 / control_rate_hz_),
        &PrecisionLandingNode::controlTimerCallback, this);
  }

  ~PrecisionLandingNode() {
    control_timer_.stop();
    shutdownModeRequests();
  }

 private:
  void loadRuntimeParameters() {
    loadParameter(private_node_, "safety/require_camera_info",
                  require_camera_info_);
    if (!require_camera_info_) {
      throw std::runtime_error(
          "~safety/require_camera_info must be true for precision landing");
    }
    loadParameter(private_node_, "safety/image_timeout_sec", "timeouts/image_sec",
                  image_timeout_sec_);
    camera_info_timeout_sec_ = image_timeout_sec_;
    loadParameter(private_node_, "safety/pose_timeout_sec", "timeouts/pose_sec",
                  pose_timeout_sec_);
    loadParameter(private_node_, "safety/state_timeout_sec", "timeouts/state_sec",
                  state_timeout_sec_);
    loadParameter(private_node_, "safety/max_header_future_sec",
                  max_header_future_sec_);
    loadParameter(private_node_, "safety/total_timeout_sec",
                  search_warning_sec_);
    loadParameter(private_node_, "control/publish_rate_hz", "control/rate_hz",
                  control_rate_hz_);
    loadParameter(private_node_, "auto_land/request_rate_hz",
                  auto_land_request_rate_hz_);
    loadParameter(private_node_, "auto_land/authorization_delay_sec",
                  auto_land_authorization_delay_sec_);
    loadParameter(private_node_, "stages/fixed_xy_descent/cutoff_height_m",
                  cutoff_height_m_);
    loadParameter(private_node_,
                  "stages/fixed_xy_descent/ground_target_offset_m",
                  ground_target_offset_m_);
    loadParameter(private_node_, "marker/requested_id",
                  active_requested_id_);
    next_requested_id_ = active_requested_id_;

    requireFinitePositive(image_timeout_sec_, "safety/image_timeout_sec");
    requireFinitePositive(camera_info_timeout_sec_,
                          "safety/image_timeout_sec");
    requireFinitePositive(pose_timeout_sec_, "safety/pose_timeout_sec");
    requireFinitePositive(state_timeout_sec_, "safety/state_timeout_sec");
    requireFiniteNonNegative(max_header_future_sec_,
                             "safety/max_header_future_sec");
    requireFinitePositive(search_warning_sec_,
                          "safety/total_timeout_sec");
    requireFinitePositive(control_rate_hz_, "control/publish_rate_hz");
    requireFinitePositive(auto_land_request_rate_hz_,
                          "auto_land/request_rate_hz");
    requireFiniteNonNegative(auto_land_authorization_delay_sec_,
                             "auto_land/authorization_delay_sec");
    requireFinitePositive(cutoff_height_m_,
                          "stages/fixed_xy_descent/cutoff_height_m");
    requireFiniteNonNegative(
        ground_target_offset_m_,
        "stages/fixed_xy_descent/ground_target_offset_m");
    if (std::abs(control_rate_hz_ - 20.0) > 1.0e-9) {
      ROS_WARN("Precision landing control is required to run at 20 Hz; "
               "ignoring configured control/publish_rate_hz=%g", control_rate_hz_);
      control_rate_hz_ = 20.0;
    }
    const double bounded_request_rate_hz =
        std::min(2.0, auto_land_request_rate_hz_);
    auto_land_request_period_sec_ = 1.0 / bounded_request_rate_hz;
    mode_request_state_->minimum_call_start_interval =
        std::chrono::duration_cast<ModeRequestClock::duration>(
            std::chrono::duration<double>(auto_land_request_period_sec_));
    mode_request_state_->authorization_delay =
        std::chrono::duration_cast<ModeRequestClock::duration>(
            std::chrono::duration<double>(
                auto_land_authorization_delay_sec_));

    std::string dictionary{"DICT_4X4_250"};
    loadParameter(private_node_, "marker/dictionary", dictionary);
    if (dictionary != "DICT_4X4_250") {
      throw std::runtime_error("~marker/dictionary must be DICT_4X4_250");
    }

    XmlRpc::XmlRpcValue rotation;
    if (private_node_.getParam("camera_to_body/rotation", rotation)) {
      if (rotation.getType() != XmlRpc::XmlRpcValue::TypeArray ||
          rotation.size() != 3) {
        throw std::runtime_error(
            "~camera_to_body/rotation must be a 3 by 3 numeric matrix");
      }
      for (int row = 0; row < rotation.size(); ++row) {
        const XmlRpc::XmlRpcValue& row_value = rotation[row];
        if (row_value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            row_value.size() != 3) {
          throw std::runtime_error(
              "~camera_to_body/rotation must be a 3 by 3 numeric matrix");
        }
        for (int column = 0; column < row_value.size(); ++column) {
          const double entry = xmlRpcNumber(row_value[column]);
          if (!isFinite(entry)) {
            throw std::runtime_error(
                "~camera_to_body/rotation must contain finite values");
          }
          camera_to_body_rotation_(row, column) = entry;
        }
      }
      const Eigen::Matrix3d orthogonality_error =
          camera_to_body_rotation_.transpose() *
              camera_to_body_rotation_ -
          Eigen::Matrix3d::Identity();
      if (orthogonality_error.norm() > 1.0e-3 ||
          std::abs(camera_to_body_rotation_.determinant() - 1.0) > 1.0e-3) {
        throw std::runtime_error(
            "~camera_to_body/rotation must be a proper rotation");
      }
      use_camera_to_body_rotation_ = true;
    }
    loadParameter(private_node_, "camera_to_body/translation_m",
                  camera_to_body_translation_m_);
    if (camera_to_body_translation_m_.size() != 3U ||
        !std::all_of(camera_to_body_translation_m_.begin(),
                     camera_to_body_translation_m_.end(), isFinite)) {
      throw std::runtime_error(
          "~camera_to_body/translation_m must contain 3 finite values");
    }
    camera_to_body_translation_ = Eigen::Vector3d(
        camera_to_body_translation_m_[0], camera_to_body_translation_m_[1],
        camera_to_body_translation_m_[2]);
  }

  void configureRosInterfaces() {
    std::string image_topic{"/usb_cam/image_raw"};
    std::string camera_info_topic{"/usb_cam/camera_info"};
    std::string pose_topic{"/mavros/local_position/pose"};
    std::string mavros_state_topic{"/mavros/state"};
    std::string trigger_topic{"/need_to_land"};
    std::string target_id_topic{"/landing/target_id"};
    std::string setpoint_topic{"/mavros/setpoint_raw/local"};
    std::string landing_state_topic{"/landing/state"};
    std::string operator_status_topic{"/landing/status"};
    std::string locked_id_topic{"/landing/locked_id"};
    std::string target_pose_topic{"/landing/target_pose"};
    std::string error_xy_topic{"/landing/error_xy"};
    std::string debug_image_topic{"/landing/debug_image"};
    std::string success_topic{"/landing/success"};
    std::string set_mode_service{"/mavros/set_mode"};

    loadParameter(private_node_, "topics/image", image_topic);
    loadParameter(private_node_, "topics/camera_info", camera_info_topic);
    loadParameter(private_node_, "topics/local_pose", "topics/pose", pose_topic);
    loadParameter(private_node_, "topics/mavros_state", mavros_state_topic);
    loadParameter(private_node_, "topics/trigger", trigger_topic);
    loadParameter(private_node_, "topics/target_id", target_id_topic);
    loadParameter(private_node_, "topics/setpoint", setpoint_topic);
    loadParameter(private_node_, "topics/landing_state", landing_state_topic);
    loadParameter(private_node_, "topics/locked_id", locked_id_topic);
    loadParameter(private_node_, "topics/target_pose", target_pose_topic);
    loadParameter(private_node_, "topics/error_xy", error_xy_topic);
    loadParameter(private_node_, "topics/debug_image", debug_image_topic);
    loadParameter(private_node_, "topics/success", success_topic);
    loadParameter(private_node_, "topics/set_mode_service", "services/set_mode",
                  set_mode_service);

    image_subscriber_ = image_transport_.subscribe(
        image_topic, 1, &PrecisionLandingNode::imageCallback, this);
    camera_info_subscriber_ = node_.subscribe(
        camera_info_topic, 1, &PrecisionLandingNode::cameraInfoCallback, this);
    pose_subscriber_ = node_.subscribe(
        pose_topic, 1, &PrecisionLandingNode::poseCallback, this);
    mavros_state_subscriber_ = node_.subscribe(
        mavros_state_topic, 1, &PrecisionLandingNode::mavrosStateCallback, this);
    trigger_subscriber_ = node_.subscribe(
        trigger_topic, 1, &PrecisionLandingNode::triggerCallback, this);
    target_id_subscriber_ = node_.subscribe(
        target_id_topic, 1, &PrecisionLandingNode::targetIdCallback, this);

    setpoint_publisher_ =
        node_.advertise<mavros_msgs::PositionTarget>(setpoint_topic, 1);
    landing_state_publisher_ =
        node_.advertise<std_msgs::String>(landing_state_topic, 1);
    operator_status_publisher_ =
        node_.advertise<std_msgs::String>(operator_status_topic, 1);
    locked_id_publisher_ =
        node_.advertise<std_msgs::Int32>(locked_id_topic, 1);
    target_pose_publisher_ =
        node_.advertise<geometry_msgs::PoseStamped>(target_pose_topic, 1);
    error_xy_publisher_ =
        node_.advertise<geometry_msgs::PointStamped>(error_xy_topic, 1);
    debug_image_publisher_ = image_transport_.advertise(debug_image_topic, 1);
    success_publisher_ = node_.advertise<std_msgs::Bool>(success_topic, 1);
    set_mode_service_name_ = node_.resolveName(set_mode_service);
  }

  bool receiveFresh(bool received,
                    const ros::Time& receive_time,
                    double timeout_sec,
                    const ros::Time& now) const {
    if (!received) {
      return false;
    }
    const double age_sec = (now - receive_time).toSec();
    return age_sec >= 0.0 && age_sec <= timeout_sec;
  }

  bool headerFresh(const ros::Time& header_stamp,
                   double timeout_sec,
                   const ros::Time& now) const {
    // Zero stamps cannot prove sample age and are rejected. A small bounded
    // future skew is tolerated for synchronized ROS/PX4 producers.
    if (header_stamp.isZero()) {
      return false;
    }
    const double age_sec = (now - header_stamp).toSec();
    return age_sec >= -max_header_future_sec_ && age_sec <= timeout_sec;
  }

  bool messageFresh(bool received,
                    const ros::Time& receive_time,
                    const ros::Time& header_stamp,
                    double timeout_sec,
                    const ros::Time& now) const {
    return receiveFresh(received, receive_time, timeout_sec, now) &&
           headerFresh(header_stamp, timeout_sec, now);
  }

  double messageFreshnessRemaining(bool received,
                                   const ros::Time& receive_time,
                                   const ros::Time& header_stamp,
                                   double timeout_sec,
                                   const ros::Time& now) const {
    if (!messageFresh(received, receive_time, header_stamp,
                      timeout_sec, now)) {
      return 0.0;
    }
    const double receive_remaining =
        timeout_sec - (now - receive_time).toSec();
    const double header_remaining =
        timeout_sec - (now - header_stamp).toSec();
    return std::max(0.0, std::min(receive_remaining, header_remaining));
  }

  bool imageFresh(const ros::Time& now) const {
    return image_valid_ &&
           messageFresh(image_received_, image_receive_time_,
                        image_header_stamp_, image_timeout_sec_, now);
  }

  bool cameraInfoFresh(const ros::Time& now) const {
    return camera_info_valid_ &&
           messageFresh(camera_info_received_, camera_info_receive_time_,
                        camera_info_header_stamp_,
                        camera_info_timeout_sec_, now);
  }

  bool poseFresh(const ros::Time& now) const {
    return pose_valid_ &&
           messageFresh(pose_received_, pose_receive_time_,
                        pose_header_stamp_, pose_timeout_sec_, now);
  }

  bool mavrosStateFresh(const ros::Time& now) const {
    return mavros_state_received_ &&
           messageFresh(mavros_state_received_, mavros_state_receive_time_,
                        mavros_state_header_stamp_,
                        state_timeout_sec_, now);
  }

  bool offboardOwned(const ros::Time& now) const {
    return mavrosStateFresh(now) && mavros_state_.connected &&
           mavros_state_.armed && mavros_state_.mode == "OFFBOARD";
  }

  bool currentYaw(double& yaw_rad) const {
    if (!pose_valid_) {
      return false;
    }
    tf2::Quaternion orientation;
    tf2::fromMsg(local_pose_.pose.orientation, orientation);
    orientation.normalize();
    yaw_rad = tf2::getYaw(orientation);
    return isFinite(yaw_rad);
  }

  bool captureCurrentYaw() {
    yaw_captured_ = currentYaw(captured_yaw_rad_);
    return yaw_captured_;
  }

  bool captureHoldPosition() {
    if (!pose_valid_) {
      return false;
    }
    hold_position_ = Eigen::Vector3d(
        local_pose_.pose.position.x,
        local_pose_.pose.position.y,
        local_pose_.pose.position.z);
    hold_position_valid_ = hold_position_.allFinite();
    if (hold_position_valid_) {
      ROS_INFO("POSITION HOLD CAPTURED: x=%.3f y=%.3f z=%.3f",
               hold_position_.x(), hold_position_.y(), hold_position_.z());
    }
    return hold_position_valid_;
  }

  void captureYawDuringPrecheck(const ros::Time& now) {
    if (trigger_ && previous_state_ == LandingState::PRECHECK &&
        !yaw_captured_ && poseFresh(now)) {
      if (captureCurrentYaw()) {
        captureHoldPosition();
      }
    }
  }

  bool trackerMutationAllowed() const {
    return trigger_ || missionKeepsTrackerActive(previous_state_);
  }

  Eigen::Vector2d bodyError(const TargetObservation& observation) const {
    if (use_camera_to_body_rotation_) {
      return (camera_to_body_rotation_ *
              observation.position_camera + camera_to_body_translation_).head<2>();
    }
    return Eigen::Vector2d(
        -observation.position_camera.y(),
        -observation.position_camera.x());
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    image_received_ = true;
    image_receive_time_ = ros::Time::now();
    image_header_stamp_ = message->header.stamp;
    latest_image_header_ = message->header;

    try {
      latest_image_ = cv_bridge::toCvCopy(
          message, sensor_msgs::image_encodings::BGR8)->image;
      image_valid_ = !latest_image_.empty();
    } catch (const cv_bridge::Exception& error) {
      ROS_WARN_THROTTLE(1.0, "Could not convert landing image: %s",
                        error.what());
      latest_image_.release();
      image_valid_ = false;
      latest_observation_ = TargetObservation();
      revokeModeRequestAuthorization();
      return;
    }
    const bool fresh_message =
        messageFresh(image_received_, image_receive_time_,
                     image_header_stamp_, image_timeout_sec_,
                     image_receive_time_);
    if (!fresh_message) {
      revokeModeRequestAuthorization();
    }
    processLatestImage(
        trackerMutationAllowed() && fresh_message &&
        cameraInfoFresh(image_receive_time_));
  }

  void processLatestImage(bool allow_lock_mutation) {
    if (!image_valid_) {
      latest_observation_ = TargetObservation();
      return;
    }

    const double stamp_sec = latest_image_header_.stamp.toSec();
    try {
      if (camera_info_valid_) {
        latest_observation_ = tracker_.process(
            latest_image_, camera_matrix_, distortion_, stamp_sec,
            active_requested_id_, allow_lock_mutation);
      } else {
        latest_observation_ = tracker_.process(
            latest_image_, cv::Mat(), cv::Mat(), stamp_sec,
            active_requested_id_, false);
      }
    } catch (const cv::Exception& error) {
      ROS_WARN_THROTTLE(1.0, "ArUco processing failed: %s", error.what());
      latest_observation_ = TargetObservation();
    }
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    camera_info_received_ = true;
    camera_info_receive_time_ = ros::Time::now();
    camera_info_header_stamp_ = message->header.stamp;
    camera_info_valid_ = validCameraInfo(*message);
    if (!camera_info_valid_) {
      camera_matrix_.release();
      distortion_.release();
      latest_observation_ = TargetObservation();
      revokeModeRequestAuthorization();
      return;
    }
    if (!messageFresh(camera_info_received_, camera_info_receive_time_,
                      camera_info_header_stamp_,
                      camera_info_timeout_sec_,
                      camera_info_receive_time_)) {
      revokeModeRequestAuthorization();
    }

    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (std::size_t index = 0U; index < message->K.size(); ++index) {
      camera_matrix_.at<double>(
          static_cast<int>(index / 3U),
          static_cast<int>(index % 3U)) = message->K[index];
    }
    distortion_ = cv::Mat(
        1, static_cast<int>(message->D.size()), CV_64F);
    for (std::size_t index = 0U; index < message->D.size(); ++index) {
      distortion_.at<double>(0, static_cast<int>(index)) =
          message->D[index];
    }
  }

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& message) {
    pose_received_ = true;
    pose_receive_time_ = ros::Time::now();
    pose_header_stamp_ = message->header.stamp;
    pose_valid_ = validPose(*message);
    if (pose_valid_) {
      local_pose_ = *message;
      if (!messageFresh(pose_received_, pose_receive_time_,
                        pose_header_stamp_, pose_timeout_sec_,
                        pose_receive_time_)) {
        revokeModeRequestAuthorization();
      } else {
        captureYawDuringPrecheck(pose_receive_time_);
      }
    } else {
      revokeModeRequestAuthorization();
    }
  }

  void mavrosStateCallback(const mavros_msgs::StateConstPtr& message) {
    mavros_state_received_ = true;
    mavros_state_receive_time_ = ros::Time::now();
    mavros_state_header_stamp_ = message->header.stamp;
    mavros_state_ = *message;
    if (!messageFresh(mavros_state_received_,
                      mavros_state_receive_time_,
                      mavros_state_header_stamp_,
                      state_timeout_sec_, mavros_state_receive_time_) ||
        !message->connected || !message->armed ||
        message->mode != "OFFBOARD") {
      revokeModeRequestAuthorization();
    }
  }

  void triggerCallback(const std_msgs::BoolConstPtr& message) {
    const bool rising_edge = message->data && !trigger_;
    const bool falling_edge = !message->data && trigger_;
    trigger_ = message->data;
    if (rising_edge && previous_state_ == LandingState::IDLE) {
      const ros::Time now = ros::Time::now();
      tracker_.reset();
      controller_.reset();
      active_requested_id_ = next_requested_id_;
      latest_observation_ = TargetObservation();
      last_error_body_.setZero();
      last_marker_height_m_ = 0.0;
      yaw_captured_ = poseFresh(now) && captureCurrentYaw();
      captureHoldPosition();
      mission_trigger_time_ = now;
      search_timeout_warning_logged_ = false;
      last_target_visible_for_hold_ = false;
      target_visibility_initialized_ = false;
      last_logged_locked_id_ = -1;
      ROS_WARN("LANDING TRIGGERED: requested_id=%d px4_mode=%s "
               "pose_fresh=%s yaw_captured=%s",
               active_requested_id_, mavros_state_.mode.c_str(),
               poseFresh(now) ? "yes" : "no",
               yaw_captured_ ? "yes" : "no");
    } else if (rising_edge) {
      ROS_WARN("LANDING TRIGGER IGNORED: current_state=%s",
               toString(previous_state_));
    }
    if (falling_edge) {
      hold_position_valid_ = false;
      mission_trigger_time_ = ros::Time(0);
      ROS_WARN("LANDING TRIGGER CLEARED: current_state=%s",
               toString(previous_state_));
    }
  }

  void targetIdCallback(const std_msgs::Int32ConstPtr& message) {
    if (next_requested_id_ != message->data) {
      ROS_INFO("NEXT LANDING TARGET ID: %d", message->data);
    }
    next_requested_id_ = message->data;
  }

  void controlTimerCallback(const ros::TimerEvent& event) {
    const ros::Time now = event.current_real;
    const bool fresh_image = imageFresh(now);
    const bool fresh_camera_info = cameraInfoFresh(now);
    const bool fresh_pose = poseFresh(now);
    const bool fresh_mavros_state = mavrosStateFresh(now);
    double current_vehicle_yaw_rad = 0.0;
    const bool current_yaw_valid =
        fresh_pose && currentYaw(current_vehicle_yaw_rad);
    captureYawDuringPrecheck(now);
    const bool owns_offboard = offboardOwned(now);
    const bool perception_inputs_fresh =
        fresh_image && fresh_camera_info && fresh_pose;
    const bool critical_inputs_fresh =
        perception_inputs_fresh && fresh_mavros_state &&
        current_yaw_valid;
    const double remaining_freshness_sec = std::max(
        0.0,
        std::min({
            messageFreshnessRemaining(
                image_received_, image_receive_time_,
                image_header_stamp_, image_timeout_sec_, now),
            messageFreshnessRemaining(
                camera_info_received_, camera_info_receive_time_,
                camera_info_header_stamp_, camera_info_timeout_sec_, now),
            messageFreshnessRemaining(
                pose_received_, pose_receive_time_, pose_header_stamp_,
                pose_timeout_sec_, now),
            messageFreshnessRemaining(
                mavros_state_received_, mavros_state_receive_time_,
                mavros_state_header_stamp_, state_timeout_sec_, now)}));
    const ModeRequestClock::time_point authorization_expiry =
        ModeRequestClock::now() +
        std::chrono::duration_cast<ModeRequestClock::duration>(
            std::chrono::duration<double>(remaining_freshness_sec));
    const int locked_id = tracker_.lockedId();
    bool target_visible =
        perception_inputs_fresh && latest_observation_.valid &&
        locked_id >= 0 && latest_observation_.id == locked_id;

    Eigen::Vector2d current_error_body = last_error_body_;
    double marker_height_m = last_marker_height_m_;
    if (target_visible) {
      double candidate_marker_height_m = 0.0;
      if (use_camera_to_body_rotation_) {
        const Eigen::Vector3d position_body =
            camera_to_body_rotation_ *
            latest_observation_.position_camera + camera_to_body_translation_;
        current_error_body = position_body.head<2>();
        candidate_marker_height_m = -position_body.z();
      } else {
        current_error_body = bodyError(latest_observation_);
        candidate_marker_height_m =
            latest_observation_.position_camera.z();
      }
      if (!isFinite(candidate_marker_height_m) ||
          candidate_marker_height_m <= 0.0) {
        target_visible = false;
      }
      if (target_visible) {
        marker_height_m = candidate_marker_height_m;
        last_error_body_ = current_error_body;
        last_marker_height_m_ = marker_height_m;
      }
    }

    if (target_visible) {
      last_target_visible_for_hold_ = true;
    } else if (last_target_visible_for_hold_) {
      captureHoldPosition();
      last_target_visible_for_hold_ = false;
    }

    bool landing_contact = false;
    if (previous_state_ == LandingState::FIXED_XY_DESCENT &&
        fixed_xy_descent_valid_ && fresh_pose) {
      const double height_above_ground =
          local_pose_.pose.position.z - fixed_xy_ground_z_;
      landing_contact = height_above_ground <= cutoff_height_m_;
    }
    if (landing_contact) {
      ROS_WARN("LOW HEIGHT CUTOFF: height=%.3f z=%.3f threshold=%.3f; "
               "requesting AUTO.LAND mode",
               local_pose_.pose.position.z - fixed_xy_ground_z_,
               local_pose_.pose.position.z, cutoff_height_m_);
    }

    StateInput input(
        trigger_,
        fresh_image && fresh_camera_info && fresh_pose &&
            fresh_mavros_state && owns_offboard && yaw_captured_ &&
            current_yaw_valid,
        owns_offboard,
        target_visible,
        fresh_mavros_state && mavros_state_.connected &&
            mavros_state_.mode == "AUTO.LAND",
        current_error_body.norm(),
        marker_height_m,
        now.toSec(),
        critical_inputs_fresh,
        landing_contact);
    const StateOutput output = state_machine_.update(input);

    if (output.state == LandingState::FIXED_XY_DESCENT &&
        previous_state_ != LandingState::FIXED_XY_DESCENT &&
        fresh_pose && target_visible) {
      fixed_xy_position_ = Eigen::Vector2d(
          local_pose_.pose.position.x, local_pose_.pose.position.y);
      fixed_xy_ground_z_ =
          local_pose_.pose.position.z - marker_height_m;
      fixed_xy_target_z_ = local_pose_.pose.position.z;
      fixed_xy_descent_valid_ =
          fixed_xy_position_.allFinite() &&
          isFinite(fixed_xy_ground_z_) &&
          isFinite(fixed_xy_target_z_);
      if (fixed_xy_descent_valid_) {
        ROS_WARN("FIXED XYZ DESCENT START: hold x=%.3f y=%.3f, "
                 "estimated ground z=%.3f, ramp Z target down",
                 fixed_xy_position_.x(), fixed_xy_position_.y(),
                 fixed_xy_ground_z_);
      }
    }

    if (output.reset_target_lock) {
      tracker_.reset();
      active_requested_id_ = next_requested_id_;
      controller_.reset();
      yaw_captured_ = false;
      last_error_body_.setZero();
      last_marker_height_m_ = 0.0;
      hold_position_valid_ = false;
      fixed_xy_descent_valid_ = false;
      mission_trigger_time_ = ros::Time(0);
    }
    if (output.reset_target_jump_history) {
      tracker_.beginReacquisition();
    }
    if (output.reset_controller) {
      controller_.reset();
    }

    const bool was_controlling = isControlState(previous_state_);
    const bool is_controlling = isControlState(output.state);
    if (is_controlling && !was_controlling) {
      controller_.reset();
    } else if (!is_controlling && was_controlling) {
      controller_.reset();
      yaw_captured_ = false;
    }

    double dt_sec = 1.0 / control_rate_hz_;
    if (!last_control_time_.isZero()) {
      dt_sec = std::max(0.001, (now - last_control_time_).toSec());
    }
    last_control_time_ = now;

    if (is_controlling && owns_offboard && yaw_captured_) {
      Eigen::Vector3d velocity_enu = Eigen::Vector3d::Zero();
      const bool position_hold_state =
          output.state == LandingState::ACQUIRE ||
          output.state == LandingState::REACQUIRE ||
          output.state == LandingState::ABORT_HOLD ||
          (!target_visible &&
           output.state != LandingState::FIXED_XY_DESCENT);
      if (output.state == LandingState::FIXED_XY_DESCENT &&
          fixed_xy_descent_valid_) {
        fixed_xy_target_z_ = std::max(
            fixed_xy_ground_z_ - ground_target_offset_m_,
            fixed_xy_target_z_ - output.descent_speed_mps * dt_sec);
        publishPositionHold(
            Eigen::Vector3d(fixed_xy_position_.x(),
                            fixed_xy_position_.y(),
                            fixed_xy_target_z_),
            captured_yaw_rad_, now);
      } else if (output.state == LandingState::REQUEST_AUTO_LAND &&
                 fixed_xy_descent_valid_) {
        publishPositionHold(
            Eigen::Vector3d(fixed_xy_position_.x(),
                            fixed_xy_position_.y(),
                            fixed_xy_target_z_),
            captured_yaw_rad_, now);
      } else if (position_hold_state && hold_position_valid_) {
        publishPositionHold(hold_position_, captured_yaw_rad_, now);
      } else if (output.state == LandingState::ALIGN && target_visible &&
                 current_yaw_valid && hold_position_valid_) {
        velocity_enu = controller_.compute(
            current_error_body, current_vehicle_yaw_rad, 0.0,
            dt_sec).velocity_enu;
        velocity_enu.z() = 0.0;
        publishHorizontalVelocityAltitudeHold(
            velocity_enu, hold_position_.z(), captured_yaw_rad_, now);
      } else if (isDescentState(output.state) && target_visible &&
                 current_yaw_valid) {
        velocity_enu = controller_.compute(
            current_error_body, current_vehicle_yaw_rad,
            output.descent_speed_mps, dt_sec).velocity_enu;
        if (fresh_pose) {
          hold_position_ = Eigen::Vector3d(
              local_pose_.pose.position.x,
              local_pose_.pose.position.y,
              local_pose_.pose.position.z);
          hold_position_valid_ = hold_position_.allFinite();
        }
        if (output.descent_speed_mps > 0.0) {
          publishSetpoint(velocity_enu, captured_yaw_rad_, now);
        } else if (hold_position_valid_) {
          publishHorizontalVelocityAltitudeHold(
              velocity_enu, hold_position_.z(), captured_yaw_rad_, now);
        }
      } else if (hold_position_valid_) {
        publishPositionHold(hold_position_, captured_yaw_rad_, now);
      }
    }

    const bool searching =
        output.state == LandingState::ACQUIRE ||
        output.state == LandingState::REACQUIRE;
    if (searching && !mission_trigger_time_.isZero() &&
        !search_timeout_warning_logged_ &&
        (now - mission_trigger_time_).toSec() >= search_warning_sec_) {
      ROS_WARN("ARUCO SEARCH EXCEEDED %.1f s: continuing position hold "
               "and automatic search",
               search_warning_sec_);
      search_timeout_warning_logged_ = true;
    }

    const bool mode_request_authorized =
        output.request_auto_land && owns_offboard &&
        critical_inputs_fresh;
    setModeRequestAuthorization(
        mode_request_authorized, fresh_mavros_state,
        mavros_state_.connected, mavros_state_.armed,
        mavros_state_.mode == "OFFBOARD", authorization_expiry);
    if (mode_request_authorized) {
      requestAutoLand();
    }
    logFlightPosition(output, fresh_pose, fresh_mavros_state,
                      current_yaw_valid, current_vehicle_yaw_rad, now);
    publishDiagnostics(output, current_error_body, target_visible, now);
    previous_state_ = output.state;
  }

  void publishSetpoint(const Eigen::Vector3d& velocity_enu,
                       double yaw_rad,
                       const ros::Time& now) {
    mavros_msgs::PositionTarget setpoint;
    setpoint.header.stamp = now;
    // MAVROS accepts local setpoints in ROS ENU and converts them to FCU NED.
    setpoint.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint.type_mask =
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    setpoint.velocity.x = velocity_enu.x();
    setpoint.velocity.y = velocity_enu.y();
    setpoint.velocity.z = velocity_enu.z();
    setpoint.yaw = yaw_rad;
    setpoint_publisher_.publish(setpoint);
  }

  void publishPositionHold(const Eigen::Vector3d& position_enu,
                           double yaw_rad,
                           const ros::Time& now) {
    mavros_msgs::PositionTarget setpoint;
    setpoint.header.stamp = now;
    setpoint.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint.type_mask =
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    setpoint.position.x = position_enu.x();
    setpoint.position.y = position_enu.y();
    setpoint.position.z = position_enu.z();
    setpoint.yaw = yaw_rad;
    setpoint_publisher_.publish(setpoint);
  }

  void publishHorizontalVelocityAltitudeHold(
      const Eigen::Vector3d& velocity_enu,
      double altitude_enu,
      double yaw_rad,
      const ros::Time& now) {
    mavros_msgs::PositionTarget setpoint;
    setpoint.header.stamp = now;
    setpoint.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint.type_mask =
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    setpoint.position.z = altitude_enu;
    setpoint.velocity.x = velocity_enu.x();
    setpoint.velocity.y = velocity_enu.y();
    setpoint.yaw = yaw_rad;
    setpoint_publisher_.publish(setpoint);
  }

  void setModeRequestAuthorization(bool authorized,
                                   bool state_snapshot_fresh = false,
                                   bool state_connected = false,
                                   bool state_armed = false,
                                   bool state_offboard = false,
                                   ModeRequestClock::time_point
                                       authorization_expiry =
                                           ModeRequestClock::time_point::min()) {
    const std::shared_ptr<ModeRequestSharedState> shared =
        mode_request_state_;
    if (!shared) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(shared->mutex);
      if (shared->shutdown) {
        return;
      }
      const bool changed =
          shared->authorization.authorized != authorized ||
          shared->authorization.state_snapshot_fresh !=
              state_snapshot_fresh ||
          shared->authorization.state_connected != state_connected ||
          shared->authorization.state_armed != state_armed ||
          shared->authorization.state_offboard != state_offboard;
      shared->authorization.authorized = authorized;
      shared->authorization.state_snapshot_fresh =
          state_snapshot_fresh;
      shared->authorization.state_connected = state_connected;
      shared->authorization.state_armed = state_armed;
      shared->authorization.state_offboard = state_offboard;
      shared->authorization.expiry = authorization_expiry;
      if (changed) {
        ++shared->authorization.generation;
      }
      if (!authorized) {
        shared->pending = false;
      }
    }
    shared->condition.notify_all();
  }

  void revokeModeRequestAuthorization() {
    setModeRequestAuthorization(false);
  }

  void shutdownModeRequests() {
    const std::shared_ptr<ModeRequestSharedState> shared =
        mode_request_state_;
    if (!shared) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(shared->mutex);
      shared->shutdown = true;
      shared->authorization.authorized = false;
      shared->authorization.state_snapshot_fresh = false;
      shared->authorization.state_connected = false;
      shared->authorization.state_armed = false;
      shared->authorization.state_offboard = false;
      shared->authorization.expiry =
          ModeRequestClock::time_point::min();
      shared->pending = false;
      ++shared->authorization.generation;
    }
    shared->condition.notify_all();
    mode_request_state_.reset();
  }

  void requestAutoLand() {
    const std::shared_ptr<ModeRequestSharedState> shared =
        mode_request_state_;
    if (!shared) {
      return;
    }

    std::uint64_t authorization_generation = 0U;
    {
      std::lock_guard<std::mutex> lock(shared->mutex);
      const ModeRequestClock::time_point enqueue_time =
          ModeRequestClock::now();
      if (shared->shutdown ||
          !shared->authorization.validAt(
              shared->authorization.generation, enqueue_time) ||
          shared->pending || shared->in_flight) {
        return;
      }
      shared->pending = true;
      authorization_generation = shared->authorization.generation;
    }

    const std::string service_name = set_mode_service_name_;
    try {
      std::thread(
          [shared, service_name, authorization_generation]() {
            std::unique_lock<std::mutex> lock(shared->mutex);
            const auto reject_pending = [shared,
                                         authorization_generation]() {
              if (!shared->shutdown &&
                  shared->authorization.generation ==
                      authorization_generation) {
                shared->pending = false;
              }
            };
            ModeRequestClock::time_point earliest_call_start =
                ModeRequestClock::now() + shared->authorization_delay;
            if (shared->has_call_start) {
              earliest_call_start = std::max(
                  earliest_call_start,
                  shared->last_call_start +
                      shared->minimum_call_start_interval);
            }

            while (true) {
              const ModeRequestClock::time_point steady_now =
                  ModeRequestClock::now();
              if (shared->shutdown || !shared->pending ||
                  !shared->authorization.validAt(
                      authorization_generation, steady_now)) {
                reject_pending();
                return;
              }
              if (steady_now >= earliest_call_start) {
                break;
              }
              shared->condition.wait_until(
                  lock,
                  std::min(earliest_call_start,
                           shared->authorization.expiry));
            }

            lock.unlock();

            bool transport_ok = false;
            bool mode_sent = false;
            bool call_started = false;
            std::string exception_message;
            try {
              ros::NodeHandle worker_node;
              ros::ServiceClient client =
                  worker_node.serviceClient<mavros_msgs::SetMode>(
                      service_name);
              mavros_msgs::SetMode request;
              request.request.base_mode = 0U;
              request.request.custom_mode = "AUTO.LAND";

              lock.lock();
              const ModeRequestClock::time_point call_start =
                  ModeRequestClock::now();
              if (shared->shutdown || !shared->pending ||
                  !shared->authorization.validAt(
                      authorization_generation, call_start)) {
                reject_pending();
                return;
              }
              shared->pending = false;
              shared->in_flight = true;
              shared->has_call_start = true;
              // This timestamp and in_flight publication are the logical
              // call-start boundary. Unlocking before the potentially
              // unbounded call leaves an unavoidable micro-TOCTOU window,
              // while keeping node destruction nonblocking.
              shared->last_call_start = call_start;
              call_started = true;
              lock.unlock();

              transport_ok = client.call(request);
              mode_sent = request.response.mode_sent;
            } catch (const std::exception& error) {
              exception_message = error.what();
            }

            lock.lock();
            if (shared->shutdown) {
              return;
            }
            if (call_started) {
              shared->in_flight = false;
            } else if (shared->authorization.generation ==
                       authorization_generation) {
              shared->pending = false;
            }
            lock.unlock();
            shared->condition.notify_all();

            if (!exception_message.empty()) {
              ROS_WARN("AUTO.LAND mode request failed: %s",
                       exception_message.c_str());
            } else if (!transport_ok) {
              ROS_WARN("AUTO.LAND mode request transport failed");
            } else if (!mode_sent) {
              ROS_WARN("PX4 rejected AUTO.LAND mode request");
            } else {
              ROS_WARN("PX4 accepted AUTO.LAND mode request");
            }
          })
          .detach();
    } catch (const std::system_error& error) {
      {
        std::lock_guard<std::mutex> lock(shared->mutex);
        if (!shared->shutdown &&
            shared->authorization.generation ==
                authorization_generation) {
          shared->pending = false;
        }
      }
      shared->condition.notify_all();
      ROS_WARN("Could not start AUTO.LAND request worker: %s",
               error.what());
    }
  }

  std::string diagnosticState(const StateOutput& output,
                              const ros::Time& now) const {
    std::string state = output.reason;
    if (!trigger_) {
      return state;
    }
    if (!camera_info_received_ || !camera_info_valid_) {
      return state + ":CAMERA_NOT_CALIBRATED";
    }
    if (!cameraInfoFresh(now)) {
      return state + ":STALE_CAMERA_INFO";
    }
    if (!imageFresh(now)) {
      return state + ":STALE_IMAGE";
    }
    if (!poseFresh(now)) {
      return state + ":STALE_POSE";
    }
    if (!mavrosStateFresh(now)) {
      return state + ":STALE_MAVROS_STATE";
    }
    return state;
  }

  std::string operatorState(const std::string& diagnostic_state) const {
    const std::size_t separator = diagnostic_state.find(':');
    const std::string internal_state =
        diagnostic_state.substr(0, separator);
    const std::string detail =
        separator == std::string::npos
            ? std::string()
            : " - " + diagnostic_state.substr(separator + 1);
    static const std::vector<std::pair<std::string, std::string>>
        display_names{
            {"IDLE", "LANDING NOT STARTED"},
            {"PRECHECK", "CHECKING SYSTEM"},
            {"ACQUIRE", "SEARCHING FOR ARUCO"},
            {"ALIGN", "MOVING TO TARGET"},
            {"DESCEND_HIGH", "FOLLOWING ARUCO DOWN"},
            {"FIXED_XY_DESCENT", "HOLDING XY, DESCENDING"},
            {"REQUEST_AUTO_LAND", "SWITCHING TO AUTO.LAND"},
            {"DONE", "LANDING COMPLETE, MOTORS STOPPED"},
            {"PASSIVE_ABORT", "LANDING STOPPED"},
        };
    for (const auto& display_name : display_names) {
      if (internal_state == display_name.first) {
        return display_name.second + detail;
      }
    }
    return diagnostic_state;
  }

  void logFlightPosition(const StateOutput& output,
                         bool fresh_pose,
                         bool fresh_mavros_state,
                         bool yaw_valid,
                         double yaw_rad,
                         const ros::Time& now) {
    const bool flight_active =
        trigger_ ||
        (fresh_mavros_state && mavros_state_.connected &&
         mavros_state_.armed);
    if (!flight_active || !fresh_pose) {
      flight_position_log_initialized_ = false;
      last_flight_position_log_time_ = ros::Time(0);
      return;
    }
    if (!last_flight_position_log_time_.isZero() &&
        (now - last_flight_position_log_time_).toSec() < 1.0) {
      return;
    }

    const Eigen::Vector3d position(
        local_pose_.pose.position.x,
        local_pose_.pose.position.y,
        local_pose_.pose.position.z);
    Eigen::Vector3d change = Eigen::Vector3d::Zero();
    if (flight_position_log_initialized_) {
      change = position - last_logged_flight_position_;
    }
    const double yaw_deg =
        yaw_valid ? yaw_rad * 180.0 / 3.14159265358979323846 : 0.0;

    ROS_INFO("FLIGHT POSITION: x=%.3f y=%.3f z=%.3f "
             "change=(%.3f,%.3f,%.3f) yaw_deg=%.1f yaw_valid=%s "
             "px4_mode=%s armed=%s landing_state=%s",
             position.x(), position.y(), position.z(),
             change.x(), change.y(), change.z(), yaw_deg,
             yaw_valid ? "yes" : "no", mavros_state_.mode.c_str(),
             mavros_state_.armed ? "yes" : "no", output.reason.c_str());

    last_logged_flight_position_ = position;
    flight_position_log_initialized_ = true;
    last_flight_position_log_time_ = now;
  }

  void publishDiagnostics(const StateOutput& output,
                          const Eigen::Vector2d& error_body,
                          bool target_visible,
                          const ros::Time& now) {
    std_msgs::String state_message;
    state_message.data = diagnosticState(output, now);
    landing_state_publisher_.publish(state_message);

    std_msgs::String operator_message;
    operator_message.data = operatorState(state_message.data);
    operator_status_publisher_.publish(operator_message);

    std_msgs::Int32 locked_id_message;
    locked_id_message.data = tracker_.lockedId();
    locked_id_publisher_.publish(locked_id_message);

    if (state_message.data != last_logged_diagnostic_state_) {
      const bool warning_state =
          state_message.data.find("ABORT") != std::string::npos ||
          state_message.data.find("STALE") != std::string::npos ||
          state_message.data.find("NOT_CALIBRATED") != std::string::npos ||
          output.state == LandingState::REQUEST_AUTO_LAND;
      if (warning_state) {
        ROS_WARN("LANDING STATE: %s | display=\"%s\" id=%d visible=%s "
                 "height=%.3f error=%.3f px4_mode=%s",
                 state_message.data.c_str(), operator_message.data.c_str(),
                 locked_id_message.data, target_visible ? "yes" : "no",
                 last_marker_height_m_, error_body.norm(),
                 mavros_state_.mode.c_str());
      } else {
        ROS_INFO("LANDING STATE: %s | display=\"%s\" id=%d visible=%s "
                 "height=%.3f error=%.3f px4_mode=%s",
                 state_message.data.c_str(), operator_message.data.c_str(),
                 locked_id_message.data, target_visible ? "yes" : "no",
                 last_marker_height_m_, error_body.norm(),
                 mavros_state_.mode.c_str());
      }
      last_logged_diagnostic_state_ = state_message.data;
    }

    if (locked_id_message.data != last_logged_locked_id_) {
      if (locked_id_message.data >= 0) {
        ROS_INFO("ARUCO LOCKED: id=%d height=%.3f error=%.3f",
                 locked_id_message.data, last_marker_height_m_,
                 error_body.norm());
      } else if (last_logged_locked_id_ >= 0) {
        ROS_WARN("ARUCO LOCK CLEARED: previous_id=%d",
                 last_logged_locked_id_);
      }
      last_logged_locked_id_ = locked_id_message.data;
    }

    if (!target_visibility_initialized_) {
      target_visibility_initialized_ = true;
      last_logged_target_visible_ = target_visible;
      if (target_visible) {
        ROS_INFO("ARUCO VISIBLE: id=%d height=%.3f error=%.3f",
                 locked_id_message.data, last_marker_height_m_,
                 error_body.norm());
      }
    } else if (target_visible != last_logged_target_visible_ &&
               (now - last_visibility_log_time_).toSec() >= 1.0) {
      if (target_visible) {
        ROS_INFO("ARUCO REACQUIRED: id=%d height=%.3f error=%.3f",
                 locked_id_message.data, last_marker_height_m_,
                 error_body.norm());
      } else {
        ROS_WARN("ARUCO LOST: locked_id=%d last_height=%.3f "
                 "last_error=%.3f",
                 locked_id_message.data, last_marker_height_m_,
                 error_body.norm());
      }
      last_logged_target_visible_ = target_visible;
      last_visibility_log_time_ = now;
    }

    std_msgs::Bool success_message;
    success_message.data = output.success;
    success_publisher_.publish(success_message);

    if (target_visible) {
      geometry_msgs::PoseStamped target_pose;
      target_pose.header = latest_image_header_;
      target_pose.pose.position.x =
          latest_observation_.position_camera.x();
      target_pose.pose.position.y =
          latest_observation_.position_camera.y();
      target_pose.pose.position.z =
          latest_observation_.position_camera.z();
      target_pose.pose.orientation.w = 1.0;
      target_pose_publisher_.publish(target_pose);

      geometry_msgs::PointStamped error_message;
      error_message.header.stamp = now;
      error_message.header.frame_id = "base_link";
      error_message.point.x = error_body.x();
      error_message.point.y = error_body.y();
      error_message.point.z = 0.0;
      error_xy_publisher_.publish(error_message);
    }

    const cv::Mat& tracker_debug_image = tracker_.debugImage();
    if (!tracker_debug_image.empty()) {
      cv::Mat annotated_image = tracker_debug_image.clone();
      cv::putText(annotated_image, operator_message.data, cv::Point(10, 68),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0),
                  1, cv::LINE_AA);
      cv::putText(
          annotated_image,
          "error_xy " + std::to_string(error_body.x()) + " " +
              std::to_string(error_body.y()),
          cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.45,
          cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
      debug_image_publisher_.publish(
          cv_bridge::CvImage(
              latest_image_header_,
              sensor_msgs::image_encodings::BGR8,
              annotated_image).toImageMsg());
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  image_transport::ImageTransport image_transport_;

  LandingController controller_;
  LandingStateMachine state_machine_;
  ArucoTracker tracker_;

  image_transport::Subscriber image_subscriber_;
  ros::Subscriber camera_info_subscriber_;
  ros::Subscriber pose_subscriber_;
  ros::Subscriber mavros_state_subscriber_;
  ros::Subscriber trigger_subscriber_;
  ros::Subscriber target_id_subscriber_;
  ros::Publisher setpoint_publisher_;
  ros::Publisher landing_state_publisher_;
  ros::Publisher operator_status_publisher_;
  ros::Publisher locked_id_publisher_;
  ros::Publisher target_pose_publisher_;
  ros::Publisher error_xy_publisher_;
  image_transport::Publisher debug_image_publisher_;
  ros::Publisher success_publisher_;
  ros::Timer control_timer_;
  std::shared_ptr<ModeRequestSharedState> mode_request_state_{
      std::make_shared<ModeRequestSharedState>()};
  std::string set_mode_service_name_;

  bool image_received_{false};
  bool camera_info_received_{false};
  bool pose_received_{false};
  bool mavros_state_received_{false};
  bool image_valid_{false};
  bool camera_info_valid_{false};
  bool pose_valid_{false};
  ros::Time image_receive_time_;
  ros::Time camera_info_receive_time_;
  ros::Time pose_receive_time_;
  ros::Time mavros_state_receive_time_;
  ros::Time image_header_stamp_;
  ros::Time camera_info_header_stamp_;
  ros::Time pose_header_stamp_;
  ros::Time mavros_state_header_stamp_;
  ros::Time last_control_time_;

  std_msgs::Header latest_image_header_;
  cv::Mat latest_image_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  geometry_msgs::PoseStamped local_pose_;
  mavros_msgs::State mavros_state_;
  TargetObservation latest_observation_;
  Eigen::Vector2d last_error_body_{Eigen::Vector2d::Zero()};
  double last_marker_height_m_{0.0};

  bool trigger_{false};
  int active_requested_id_{-1};
  int next_requested_id_{-1};
  LandingState previous_state_{LandingState::IDLE};
  Eigen::Vector3d hold_position_{Eigen::Vector3d::Zero()};
  bool hold_position_valid_{false};
  Eigen::Vector2d fixed_xy_position_{Eigen::Vector2d::Zero()};
  double fixed_xy_ground_z_{0.0};
  double fixed_xy_target_z_{0.0};
  bool fixed_xy_descent_valid_{false};
  bool last_target_visible_for_hold_{false};
  ros::Time mission_trigger_time_;
  bool search_timeout_warning_logged_{false};
  std::string last_logged_diagnostic_state_;
  int last_logged_locked_id_{-1};
  bool target_visibility_initialized_{false};
  bool last_logged_target_visible_{false};
  ros::Time last_visibility_log_time_;
  bool flight_position_log_initialized_{false};
  Eigen::Vector3d last_logged_flight_position_{Eigen::Vector3d::Zero()};
  ros::Time last_flight_position_log_time_;
  bool yaw_captured_{false};
  double captured_yaw_rad_{0.0};

  double image_timeout_sec_{0.50};
  double camera_info_timeout_sec_{2.0};
  double pose_timeout_sec_{0.50};
  double state_timeout_sec_{0.50};
  double max_header_future_sec_{0.05};
  double search_warning_sec_{30.0};
  double control_rate_hz_{20.0};
  double auto_land_request_rate_hz_{2.0};
  double auto_land_request_period_sec_{0.50};
  double auto_land_authorization_delay_sec_{0.05};
  double cutoff_height_m_{0.30};
  double ground_target_offset_m_{0.03};
  bool require_camera_info_{true};
  bool use_camera_to_body_rotation_{true};
  Eigen::Matrix3d camera_to_body_rotation_{
      (Eigen::Matrix3d() << 0.0, -1.0, 0.0,
                           -1.0, 0.0, 0.0,
                            0.0, 0.0, -1.0).finished()};
  std::vector<double> camera_to_body_translation_m_{0.0, 0.0, 0.0};
  Eigen::Vector3d camera_to_body_translation_{Eigen::Vector3d::Zero()};
};

}  // namespace precision_landing

int main(int argc, char** argv) {
  ros::init(argc, argv, "precision_landing_node");
  try {
    precision_landing::PrecisionLandingNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("precision_landing_node failed to start: %s", error.what());
    return 1;
  }
  return 0;
}
