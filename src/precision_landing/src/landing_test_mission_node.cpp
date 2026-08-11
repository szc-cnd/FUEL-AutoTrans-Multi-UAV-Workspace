#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

namespace precision_landing {
namespace {

bool startsWith(const std::string& value, const std::string& prefix) {
  return value.compare(0, prefix.size(), prefix) == 0;
}

bool landingStateOwnsSetpoints(const std::string& state) {
  static const std::vector<std::string> control_states{
      "ACQUIRE", "ALIGN", "DESCEND_HIGH", "FIXED_XY_DESCENT",
      "DESCEND_MID", "DESCEND_FINAL",
      "REACQUIRE", "REQUEST_AUTO_LAND", "ABORT_HOLD"};
  for (const auto& control_state : control_states) {
    if (startsWith(state, control_state)) {
      return true;
    }
  }
  return false;
}

bool validPose(const geometry_msgs::PoseStamped& pose) {
  const auto& position = pose.pose.position;
  const auto& orientation = pose.pose.orientation;
  const double quaternion_norm =
      std::sqrt(orientation.x * orientation.x +
                orientation.y * orientation.y +
                orientation.z * orientation.z +
                orientation.w * orientation.w);
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) && std::isfinite(quaternion_norm) &&
         quaternion_norm > 0.5 && quaternion_norm < 1.5;
}

}  // namespace

class LandingTestMissionNode {
 public:
  LandingTestMissionNode()
      : node_(), private_node_("~") {
    allow_arming_ = private_node_.param<bool>("allow_arming", false);
    auto_start_ = private_node_.param<bool>("auto_start", false);
    auto_start_delay_sec_ =
        private_node_.param<double>("auto_start_delay_sec", 5.0);
    takeoff_height_m_ =
        private_node_.param<double>("takeoff_height_m", 2.05);
    publish_rate_hz_ =
        private_node_.param<double>("publish_rate_hz", 20.0);
    prestream_sec_ = private_node_.param<double>("prestream_sec", 2.0);
    hover_stable_sec_ =
        private_node_.param<double>("hover_stable_sec", 0.10);
    altitude_tolerance_m_ =
        private_node_.param<double>("altitude_tolerance_m", 0.05);
    xy_tolerance_m_ =
        private_node_.param<double>("xy_tolerance_m", 0.20);
    state_timeout_sec_ =
        private_node_.param<double>("state_timeout_sec", 1.50);
    pose_timeout_sec_ =
        private_node_.param<double>("pose_timeout_sec", 0.50);
    operation_timeout_sec_ =
        private_node_.param<double>("operation_timeout_sec", 30.0);

    validateParameters();

    const std::string mavros_state_topic =
        private_node_.param<std::string>("mavros_state_topic", "/mavros/state");
    const std::string local_pose_topic =
        private_node_.param<std::string>("local_pose_topic",
                                         "/mavros/local_position/pose");
    const std::string position_setpoint_topic =
        private_node_.param<std::string>("position_setpoint_topic",
                                         "/mavros/setpoint_position/local");
    const std::string landing_trigger_topic =
        private_node_.param<std::string>("landing_trigger_topic",
                                         "/need_to_land");
    const std::string landing_state_topic =
        private_node_.param<std::string>("landing_state_topic",
                                         "/landing/state");
    const std::string arming_service =
        private_node_.param<std::string>("arming_service",
                                         "/mavros/cmd/arming");
    const std::string set_mode_service =
        private_node_.param<std::string>("set_mode_service",
                                         "/mavros/set_mode");

    mavros_state_subscriber_ = node_.subscribe(
        mavros_state_topic, 1, &LandingTestMissionNode::mavrosStateCallback,
        this);
    pose_subscriber_ = node_.subscribe(
        local_pose_topic, 1, &LandingTestMissionNode::poseCallback, this);
    landing_state_subscriber_ = node_.subscribe(
        landing_state_topic, 1,
        &LandingTestMissionNode::landingStateCallback, this);

    setpoint_publisher_ =
        node_.advertise<geometry_msgs::PoseStamped>(position_setpoint_topic, 1);
    trigger_publisher_ =
        node_.advertise<std_msgs::Bool>(landing_trigger_topic, 1);
    status_publisher_ =
        private_node_.advertise<std_msgs::String>("status", 1, true);

    start_service_ = private_node_.advertiseService(
        "start", &LandingTestMissionNode::startCallback, this);
    abort_service_ = private_node_.advertiseService(
        "abort", &LandingTestMissionNode::abortCallback, this);
    arming_client_ =
        node_.serviceClient<mavros_msgs::CommandBool>(arming_service);
    set_mode_client_ =
        node_.serviceClient<mavros_msgs::SetMode>(set_mode_service);

    timer_ = node_.createTimer(ros::Duration(1.0 / publish_rate_hz_),
                               &LandingTestMissionNode::timerCallback, this);
    publishTrigger(false);
    publishStatus();

    if (auto_start_) {
      ROS_WARN("REAL VEHICLE AUTO-START ENABLED: the vehicle will arm and "
               "take off %.1f s after all prerequisites become ready",
               auto_start_delay_sec_);
    } else if (!allow_arming_) {
      ROS_WARN("Landing test node is SAFE/INERT: set allow_arming:=true and "
               "call /landing_test/start to authorize real arming");
    }
  }

 private:
  enum class Phase {
    WAITING,
    PRESTREAM,
    REQUEST_OFFBOARD,
    REQUEST_ARM,
    TAKEOFF,
    HOVER,
    HANDOFF,
    COMPLETE,
    FAILSAFE_HOLD
  };

  void validateParameters() const {
    if ((auto_start_ && !allow_arming_) ||
        !std::isfinite(auto_start_delay_sec_) ||
        auto_start_delay_sec_ < 3.0 ||
        !std::isfinite(takeoff_height_m_) || takeoff_height_m_ <= 1.20 ||
        takeoff_height_m_ > 5.0 ||
        !std::isfinite(publish_rate_hz_) || publish_rate_hz_ < 10.0 ||
        !std::isfinite(prestream_sec_) || prestream_sec_ < 1.0 ||
        !std::isfinite(hover_stable_sec_) || hover_stable_sec_ <= 0.0 ||
        !std::isfinite(altitude_tolerance_m_) ||
        altitude_tolerance_m_ <= 0.0 ||
        !std::isfinite(xy_tolerance_m_) || xy_tolerance_m_ <= 0.0 ||
        !std::isfinite(state_timeout_sec_) || state_timeout_sec_ <= 0.0 ||
        !std::isfinite(pose_timeout_sec_) || pose_timeout_sec_ <= 0.0 ||
        !std::isfinite(operation_timeout_sec_) ||
        operation_timeout_sec_ <= prestream_sec_) {
      throw std::runtime_error("invalid landing-test safety parameters");
    }
  }

  bool inputsFresh(const ros::Time& now) const {
    return state_received_ && pose_received_ && mavros_state_.connected &&
           validPose(local_pose_) &&
           (now - state_receive_time_).toSec() <= state_timeout_sec_ &&
           (now - pose_receive_time_).toSec() <= pose_timeout_sec_;
  }

  bool startCallback(std_srvs::Trigger::Request&,
                     std_srvs::Trigger::Response& response) {
    const ros::Time now = ros::Time::now();
    if (phase_ != Phase::WAITING) {
      response.success = false;
      response.message = "test already started; restart node before retry";
      return true;
    }
    std::string reason;
    if (!autoStartReady(now, reason)) {
      response.success = false;
      response.message = reason;
      return true;
    }

    beginTest(now);
    response.success = true;
    response.message =
        "authorized: prestreaming OFFBOARD setpoint before arming";
    return true;
  }

  bool autoStartReady(const ros::Time& now, std::string& reason) {
    if (!allow_arming_) {
      reason = "allow_arming is false; refusing real vehicle start";
      return false;
    }
    if (!inputsFresh(now)) {
      reason = "MAVROS state/local pose missing, stale, or disconnected";
      return false;
    }
    if (mavros_state_.armed) {
      reason = "vehicle is already armed; refusing ambiguous start";
      return false;
    }
    if (setpoint_publisher_.getNumSubscribers() == 0U ||
        !arming_client_.exists() || !set_mode_client_.exists()) {
      reason = "required MAVROS setpoint/services are unavailable";
      return false;
    }
    if (landing_state_subscriber_.getNumPublishers() == 0U) {
      reason = "precision_landing_node is not publishing state";
      return false;
    }
    reason.clear();
    return true;
  }

  void beginTest(const ros::Time& now) {
    target_pose_ = local_pose_;
    target_pose_.header.frame_id =
        local_pose_.header.frame_id.empty() ? "map" : local_pose_.header.frame_id;
    target_pose_.pose.position.z += takeoff_height_m_;
    target_valid_ = true;
    operation_started_ = now;
    phase_started_ = now;
    stable_since_ = ros::Time(0);
    last_service_request_ = ros::Time(0);
    phase_ = Phase::PRESTREAM;
    publishTrigger(false);
    publishStatus();

    ROS_WARN("REAL VEHICLE LANDING TEST AUTHORIZED: target relative altitude "
             "%.2f m", takeoff_height_m_);
  }

  bool abortCallback(std_srvs::Trigger::Request&,
                     std_srvs::Trigger::Response& response) {
    if (phase_ == Phase::WAITING || phase_ == Phase::COMPLETE) {
      response.success = false;
      response.message = "no active takeoff controller to abort";
      return true;
    }
    enterFailsafeHold("operator requested abort; take over with RC");
    response.success = true;
    response.message =
        "landing trigger cleared; holding last target until RC takeover";
    return true;
  }

  void mavrosStateCallback(const mavros_msgs::StateConstPtr& message) {
    mavros_state_ = *message;
    state_received_ = true;
    state_receive_time_ = ros::Time::now();
  }

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& message) {
    local_pose_ = *message;
    pose_received_ = true;
    pose_receive_time_ = ros::Time::now();
  }

  void landingStateCallback(const std_msgs::StringConstPtr& message) {
    landing_state_ = message->data;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();

    if (phase_ == Phase::WAITING) {
      publishTrigger(false);
      if (auto_start_) {
        std::string reason;
        if (!autoStartReady(now, reason)) {
          auto_start_ready_since_ = ros::Time(0);
          ROS_WARN_THROTTLE(2.0, "Auto-start waiting: %s", reason.c_str());
          return;
        }
        if (auto_start_ready_since_.isZero()) {
          auto_start_ready_since_ = now;
          ROS_WARN("All prerequisites ready; automatic takeoff begins in %.1f s",
                   auto_start_delay_sec_);
          return;
        }
        const double ready_sec = (now - auto_start_ready_since_).toSec();
        ROS_WARN_THROTTLE(1.0, "Automatic takeoff countdown: %.1f s",
                          std::max(0.0, auto_start_delay_sec_ - ready_sec));
        if (ready_sec >= auto_start_delay_sec_) {
          beginTest(now);
        }
      }
      return;
    }
    if (phase_ == Phase::COMPLETE) {
      publishTrigger(true);
      return;
    }
    if (phase_ == Phase::FAILSAFE_HOLD) {
      publishTrigger(false);
      publishTarget(now);
      return;
    }

    if (!inputsFresh(now)) {
      enterFailsafeHold("MAVROS state or local pose became stale");
      return;
    }
    if ((now - operation_started_).toSec() > operation_timeout_sec_) {
      enterFailsafeHold("takeoff/handoff operation timed out");
      return;
    }

    if ((phase_ == Phase::REQUEST_ARM || phase_ == Phase::TAKEOFF ||
         phase_ == Phase::HOVER || phase_ == Phase::HANDOFF) &&
        mavros_state_.mode != "OFFBOARD") {
      enterFailsafeHold("PX4 left OFFBOARD before landing handoff");
      return;
    }
    if ((phase_ == Phase::TAKEOFF || phase_ == Phase::HOVER ||
         phase_ == Phase::HANDOFF) &&
        !mavros_state_.armed) {
      enterFailsafeHold("vehicle disarmed before landing handoff");
      return;
    }

    publishTarget(now);

    if (phase_ == Phase::PRESTREAM) {
      if ((now - phase_started_).toSec() >= prestream_sec_) {
        phase_ = Phase::REQUEST_OFFBOARD;
        phase_started_ = now;
        publishStatus();
      }
      return;
    }

    if (phase_ == Phase::REQUEST_OFFBOARD) {
      if (mavros_state_.mode == "OFFBOARD") {
        phase_ = Phase::REQUEST_ARM;
        phase_started_ = now;
        publishStatus();
      } else {
        requestOffboard(now);
      }
      return;
    }

    if (phase_ == Phase::REQUEST_ARM) {
      if (mavros_state_.armed) {
        phase_ = Phase::TAKEOFF;
        phase_started_ = now;
        publishStatus();
      } else {
        requestArm(now);
      }
      return;
    }

    if (phase_ == Phase::TAKEOFF || phase_ == Phase::HOVER) {
      if (atTarget()) {
        if (stable_since_.isZero()) {
          stable_since_ = now;
          phase_ = Phase::HOVER;
          publishStatus();
        } else if ((now - stable_since_).toSec() >= hover_stable_sec_) {
          phase_ = Phase::HANDOFF;
          phase_started_ = now;
          publishStatus();
        }
      } else {
        stable_since_ = ros::Time(0);
        phase_ = Phase::TAKEOFF;
      }
      return;
    }

    if (phase_ == Phase::HANDOFF) {
      publishTrigger(true);
      if (landingStateOwnsSetpoints(landing_state_)) {
        phase_ = Phase::COMPLETE;
        publishStatus();
        ROS_WARN("Landing node accepted handoff in state %s; stopping position "
                 "setpoints", landing_state_.c_str());
      }
    }
  }

  void requestOffboard(const ros::Time& now) {
    if (!serviceRequestDue(now)) {
      return;
    }
    mavros_msgs::SetMode request;
    request.request.base_mode = 0U;
    request.request.custom_mode = "OFFBOARD";
    if (!set_mode_client_.call(request) || !request.response.mode_sent) {
      ROS_WARN("PX4 rejected OFFBOARD request; retrying");
    }
  }

  void requestArm(const ros::Time& now) {
    if (!serviceRequestDue(now)) {
      return;
    }
    mavros_msgs::CommandBool request;
    request.request.value = true;
    if (!arming_client_.call(request) || !request.response.success) {
      ROS_WARN("PX4 rejected arming request; retrying");
    }
  }

  bool serviceRequestDue(const ros::Time& now) {
    if (!last_service_request_.isZero() &&
        (now - last_service_request_).toSec() < 1.0) {
      return false;
    }
    last_service_request_ = now;
    return true;
  }

  bool atTarget() const {
    const double dx =
        local_pose_.pose.position.x - target_pose_.pose.position.x;
    const double dy =
        local_pose_.pose.position.y - target_pose_.pose.position.y;
    const double dz =
        local_pose_.pose.position.z - target_pose_.pose.position.z;
    return std::hypot(dx, dy) <= xy_tolerance_m_ &&
           std::abs(dz) <= altitude_tolerance_m_;
  }

  void publishTarget(const ros::Time& now) {
    if (!target_valid_) {
      return;
    }
    target_pose_.header.stamp = now;
    setpoint_publisher_.publish(target_pose_);
  }

  void publishTrigger(bool value) {
    std_msgs::Bool message;
    message.data = value;
    trigger_publisher_.publish(message);
  }

  std::string phaseName() const {
    switch (phase_) {
      case Phase::WAITING:
        return "WAITING";
      case Phase::PRESTREAM:
        return "PRESTREAM";
      case Phase::REQUEST_OFFBOARD:
        return "REQUEST_OFFBOARD";
      case Phase::REQUEST_ARM:
        return "REQUEST_ARM";
      case Phase::TAKEOFF:
        return "TAKEOFF";
      case Phase::HOVER:
        return "HOVER";
      case Phase::HANDOFF:
        return "HANDOFF";
      case Phase::COMPLETE:
        return "COMPLETE";
      case Phase::FAILSAFE_HOLD:
        return "FAILSAFE_HOLD";
    }
    return "UNKNOWN";
  }

  void publishStatus() {
    std_msgs::String message;
    message.data = phaseName();
    status_publisher_.publish(message);
  }

  void enterFailsafeHold(const std::string& reason) {
    if (phase_ == Phase::FAILSAFE_HOLD) {
      return;
    }
    phase_ = Phase::FAILSAFE_HOLD;
    publishTrigger(false);
    publishStatus();
    ROS_ERROR("LANDING TEST FAILSAFE_HOLD: %s. Use RC mode takeover; do not "
              "kill this node while it owns OFFBOARD.", reason.c_str());
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber mavros_state_subscriber_;
  ros::Subscriber pose_subscriber_;
  ros::Subscriber landing_state_subscriber_;
  ros::Publisher setpoint_publisher_;
  ros::Publisher trigger_publisher_;
  ros::Publisher status_publisher_;
  ros::ServiceServer start_service_;
  ros::ServiceServer abort_service_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::Timer timer_;

  Phase phase_{Phase::WAITING};
  bool allow_arming_{false};
  bool auto_start_{false};
  bool state_received_{false};
  bool pose_received_{false};
  bool target_valid_{false};
  double takeoff_height_m_{2.05};
  double auto_start_delay_sec_{5.0};
  double publish_rate_hz_{20.0};
  double prestream_sec_{2.0};
  double hover_stable_sec_{0.10};
  double altitude_tolerance_m_{0.05};
  double xy_tolerance_m_{0.20};
  double state_timeout_sec_{1.50};
  double pose_timeout_sec_{0.50};
  double operation_timeout_sec_{30.0};

  mavros_msgs::State mavros_state_;
  geometry_msgs::PoseStamped local_pose_;
  geometry_msgs::PoseStamped target_pose_;
  std::string landing_state_;
  ros::Time state_receive_time_;
  ros::Time pose_receive_time_;
  ros::Time operation_started_;
  ros::Time phase_started_;
  ros::Time stable_since_;
  ros::Time last_service_request_;
  ros::Time auto_start_ready_since_;
};

}  // namespace precision_landing

int main(int argc, char** argv) {
  ros::init(argc, argv, "landing_test");
  try {
    precision_landing::LandingTestMissionNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("landing_test failed to start: %s", error.what());
    return 1;
  }
  return 0;
}
