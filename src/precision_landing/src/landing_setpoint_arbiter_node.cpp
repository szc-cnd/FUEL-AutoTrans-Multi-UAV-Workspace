#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <string>

namespace {

class LandingSetpointArbiter {
 public:
  LandingSetpointArbiter() : nh_(), pnh_("~") {
    std::string trigger_topic;
    std::string controller_position_input;
    std::string controller_attitude_input;
    std::string landing_position_input;
    std::string mavros_position_output;
    std::string mavros_attitude_output;
    std::string active_topic;
    std::string owner_topic;

    pnh_.param<std::string>("trigger_topic", trigger_topic, "/need_to_land");
    pnh_.param<std::string>("controller_position_input",
                            controller_position_input,
                            "/control/position_setpoint");
    pnh_.param<std::string>("controller_attitude_input",
                            controller_attitude_input,
                            "/control/attitude_setpoint");
    pnh_.param<std::string>("landing_position_input",
                            landing_position_input,
                            "/landing/setpoint_raw/local");
    pnh_.param<std::string>("mavros_position_output",
                            mavros_position_output,
                            "/mavros/setpoint_raw/local");
    pnh_.param<std::string>("mavros_attitude_output",
                            mavros_attitude_output,
                            "/mavros/setpoint_raw/attitude");
    pnh_.param<std::string>("active_topic", active_topic,
                            "/landing/control_active");
    pnh_.param<std::string>("owner_topic", owner_topic,
                            "/landing/control_owner");

    position_output_ =
        nh_.advertise<mavros_msgs::PositionTarget>(mavros_position_output, 10);
    attitude_output_ =
        nh_.advertise<mavros_msgs::AttitudeTarget>(mavros_attitude_output, 10);
    active_output_ = nh_.advertise<std_msgs::Bool>(active_topic, 1, true);
    owner_output_ = nh_.advertise<std_msgs::String>(owner_topic, 1, true);

    trigger_sub_ = nh_.subscribe(trigger_topic, 2,
                                 &LandingSetpointArbiter::triggerCallback, this);
    controller_position_sub_ = nh_.subscribe(
        controller_position_input, 10,
        &LandingSetpointArbiter::controllerPositionCallback, this);
    controller_attitude_sub_ = nh_.subscribe(
        controller_attitude_input, 10,
        &LandingSetpointArbiter::controllerAttitudeCallback, this);
    landing_position_sub_ = nh_.subscribe(
        landing_position_input, 10,
        &LandingSetpointArbiter::landingPositionCallback, this);

    publishOwner("CONTROLLER");
    ROS_WARN("Landing setpoint arbiter ready: controller owns MAVROS output");
  }

 private:
  void triggerCallback(const std_msgs::BoolConstPtr& message) {
    if (landing_owned_) {
      if (!message->data) {
        ROS_WARN_THROTTLE(
            2.0,
            "Landing control is latched; false trigger cannot restore controller");
      }
      return;
    }
    trigger_pending_ = message->data;
    publishOwner(trigger_pending_ ? "LANDING_PENDING" : "CONTROLLER");
  }

  void controllerPositionCallback(
      const mavros_msgs::PositionTargetConstPtr& message) {
    if (!landing_owned_) {
      position_output_.publish(*message);
    }
  }

  void controllerAttitudeCallback(
      const mavros_msgs::AttitudeTargetConstPtr& message) {
    if (!landing_owned_) {
      attitude_output_.publish(*message);
    }
  }

  void landingPositionCallback(
      const mavros_msgs::PositionTargetConstPtr& message) {
    if (!trigger_pending_ && !landing_owned_) {
      ROS_ERROR_THROTTLE(
          2.0, "Rejected landing setpoint before need_to_land trigger");
      return;
    }
    if (!landing_owned_) {
      landing_owned_ = true;
      trigger_pending_ = false;
      publishOwner("LANDING");
      ROS_ERROR("LANDING CONTROL TAKEOVER LATCHED: controller output blocked");
    }
    position_output_.publish(*message);
  }

  void publishOwner(const std::string& owner) {
    std_msgs::Bool active;
    active.data = landing_owned_;
    active_output_.publish(active);
    std_msgs::String owner_message;
    owner_message.data = owner;
    owner_output_.publish(owner_message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber trigger_sub_;
  ros::Subscriber controller_position_sub_;
  ros::Subscriber controller_attitude_sub_;
  ros::Subscriber landing_position_sub_;
  ros::Publisher position_output_;
  ros::Publisher attitude_output_;
  ros::Publisher active_output_;
  ros::Publisher owner_output_;
  bool trigger_pending_{false};
  bool landing_owned_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "landing_setpoint_arbiter");
  LandingSetpointArbiter arbiter;
  ros::spin();
  return 0;
}
