#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <string>

namespace {

class DualUavLandingCoordinator {
 public:
  DualUavLandingCoordinator() : private_node_("~") {
    std::string uav0_locked_topic{"/UAV0/landing/search/locked_id"};
    std::string uav1_locked_topic{"/UAV1/landing/search/locked_id"};
    std::string uav0_assigned_topic{"/UAV0/landing/assigned_id"};
    std::string uav1_assigned_topic{"/UAV1/landing/assigned_id"};
    std::string uav1_excluded_topic{"/UAV1/landing/excluded_id"};
    std::string ready_topic{"/dual_uav_landing/assignments_ready"};
    std::string status_topic{"/dual_uav_landing/status"};
    private_node_.param("topics/uav0_locked", uav0_locked_topic,
                        uav0_locked_topic);
    private_node_.param("topics/uav1_locked", uav1_locked_topic,
                        uav1_locked_topic);
    private_node_.param("topics/uav0_assigned", uav0_assigned_topic,
                        uav0_assigned_topic);
    private_node_.param("topics/uav1_assigned", uav1_assigned_topic,
                        uav1_assigned_topic);
    private_node_.param("topics/uav1_excluded", uav1_excluded_topic,
                        uav1_excluded_topic);
    private_node_.param("topics/assignments_ready", ready_topic, ready_topic);
    private_node_.param("topics/status", status_topic, status_topic);

    uav0_assigned_publisher_ =
        node_.advertise<std_msgs::Int32>(uav0_assigned_topic, 1, true);
    uav1_assigned_publisher_ =
        node_.advertise<std_msgs::Int32>(uav1_assigned_topic, 1, true);
    uav1_excluded_publisher_ =
        node_.advertise<std_msgs::Int32>(uav1_excluded_topic, 1, true);
    ready_publisher_ = node_.advertise<std_msgs::Bool>(ready_topic, 1, true);
    status_publisher_ =
        node_.advertise<std_msgs::String>(status_topic, 1, true);

    uav0_locked_subscriber_ =
        node_.subscribe(uav0_locked_topic, 5,
                        &DualUavLandingCoordinator::uav0LockedCallback, this);
    uav1_locked_subscriber_ =
        node_.subscribe(uav1_locked_topic, 5,
                        &DualUavLandingCoordinator::uav1LockedCallback, this);

    publishInt(uav0_assigned_publisher_, -1);
    publishInt(uav1_assigned_publisher_, -1);
    publishInt(uav1_excluded_publisher_, -1);
    publishState("等待 UAV0 锁定降落平台");
  }

 private:
  void uav0LockedCallback(const std_msgs::Int32ConstPtr &message) {
    if (message->data < 0) {
      if (uav0_id_ >= 0) {
        resetAssignments();
      }
      return;
    }
    if (message->data == uav0_id_) {
      return;
    }

    uav0_id_ = message->data;
    uav1_assigned_id_ = -1;
    publishInt(uav0_assigned_publisher_, uav0_id_);
    publishInt(uav1_assigned_publisher_, -1);
    publishInt(uav1_excluded_publisher_, uav0_id_);
    evaluateUav1Lock();
  }

  void uav1LockedCallback(const std_msgs::Int32ConstPtr &message) {
    uav1_observed_id_ = message->data;
    evaluateUav1Lock();
  }

  void evaluateUav1Lock() {
    if (uav0_id_ < 0) {
      publishState("等待 UAV0 锁定降落平台");
      return;
    }
    if (uav1_observed_id_ < 0) {
      publishState("UAV0 已分配，等待 UAV1 搜索另一平台");
      return;
    }
    if (uav1_observed_id_ == uav0_id_) {
      uav1_assigned_id_ = -1;
      publishInt(uav1_assigned_publisher_, -1);
      publishInt(uav1_excluded_publisher_, uav0_id_);
      publishState("检测到平台冲突，已令 UAV1 排除 UAV0 平台");
      return;
    }

    if (uav1_assigned_id_ != uav1_observed_id_) {
      uav1_assigned_id_ = uav1_observed_id_;
      publishInt(uav1_assigned_publisher_, uav1_assigned_id_);
    }
    publishState("双机已分配不同降落平台");
  }

  void resetAssignments() {
    uav0_id_ = -1;
    uav1_observed_id_ = -1;
    uav1_assigned_id_ = -1;
    publishInt(uav0_assigned_publisher_, -1);
    publishInt(uav1_assigned_publisher_, -1);
    publishInt(uav1_excluded_publisher_, -1);
    publishState("任务重置，等待 UAV0 锁定降落平台");
  }

  void publishState(const std::string &status) {
    const bool ready = uav0_id_ >= 0 && uav1_assigned_id_ >= 0 &&
                       uav0_id_ != uav1_assigned_id_;
    std_msgs::Bool ready_message;
    ready_message.data = ready;
    ready_publisher_.publish(ready_message);
    if (status == last_status_) {
      return;
    }
    last_status_ = status;
    std_msgs::String status_message;
    status_message.data = status;
    status_publisher_.publish(status_message);
    ROS_INFO_STREAM("dual_uav_landing: " << status << " (UAV0=" << uav0_id_
                                         << ", UAV1=" << uav1_assigned_id_
                                         << ")");
  }

  static void publishInt(ros::Publisher &publisher, int value) {
    std_msgs::Int32 message;
    message.data = value;
    publisher.publish(message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber uav0_locked_subscriber_;
  ros::Subscriber uav1_locked_subscriber_;
  ros::Publisher uav0_assigned_publisher_;
  ros::Publisher uav1_assigned_publisher_;
  ros::Publisher uav1_excluded_publisher_;
  ros::Publisher ready_publisher_;
  ros::Publisher status_publisher_;
  int uav0_id_{-1};
  int uav1_observed_id_{-1};
  int uav1_assigned_id_{-1};
  std::string last_status_;
};

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "dual_uav_landing_coordinator");
  DualUavLandingCoordinator coordinator;
  ros::spin();
  return 0;
}
