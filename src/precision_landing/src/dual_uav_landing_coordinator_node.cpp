#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "precision_landing/LandingPlatformArray.h"

namespace {

class DualUavLandingCoordinator {
 public:
  DualUavLandingCoordinator() : private_node_("~") {
    std::string candidates_topic{"/UAV0/landing/search/candidates"};
    std::string exit_topic{"/UAV0/mission/final_exit"};
    std::string success_topic{"/UAV0/landing/success"};
    private_node_.param("topics/candidates", candidates_topic, candidates_topic);
    private_node_.param("topics/final_exit", exit_topic, exit_topic);
    private_node_.param("topics/uav0_success", success_topic, success_topic);
    private_node_.param("topics/uav0_assigned", uav0_assigned_topic_, std::string("/UAV0/landing/assigned_id"));
    private_node_.param("topics/uav1_assigned", uav1_assigned_topic_, std::string("/UAV1/landing/assigned_id"));
    private_node_.param("topics/uav0_target", uav0_target_topic_, std::string("/UAV0/landing/assigned_target"));
    private_node_.param("topics/uav1_target", uav1_target_topic_, std::string("/UAV1/landing/assigned_target"));
    private_node_.param("topics/release_uav1", release_topic_, std::string("/dual_uav_landing/release_uav1"));
    private_node_.param("topics/assignments_ready", ready_topic_, std::string("/dual_uav_landing/assignments_ready"));
    private_node_.param("topics/status", status_topic_, std::string("/dual_uav_landing/status"));

    uav0_assigned_pub_ = node_.advertise<std_msgs::Int32>(uav0_assigned_topic_, 1, true);
    uav1_assigned_pub_ = node_.advertise<std_msgs::Int32>(uav1_assigned_topic_, 1, true);
    uav0_target_pub_ = node_.advertise<geometry_msgs::PoseStamped>(uav0_target_topic_, 1, true);
    uav1_target_pub_ = node_.advertise<geometry_msgs::PoseStamped>(uav1_target_topic_, 1, true);
    release_pub_ = node_.advertise<std_msgs::Bool>(release_topic_, 1, true);
    ready_pub_ = node_.advertise<std_msgs::Bool>(ready_topic_, 1, true);
    status_pub_ = node_.advertise<std_msgs::String>(status_topic_, 1, true);
    candidates_sub_ = node_.subscribe(candidates_topic, 2, &DualUavLandingCoordinator::candidatesCallback, this);
    exit_sub_ = node_.subscribe(exit_topic, 1, &DualUavLandingCoordinator::exitCallback, this);
    success_sub_ = node_.subscribe(success_topic, 2, &DualUavLandingCoordinator::successCallback, this);
    target_timer_ = node_.createTimer(
        ros::Duration(0.20), &DualUavLandingCoordinator::targetTimerCallback, this);
    publishId(uav0_assigned_pub_, -1);
    publishId(uav1_assigned_pub_, -1);
    publishBool(release_pub_, false);
    publishBool(ready_pub_, false);
    publishStatus("等待 UAV0 稳定确认两个降落平台");
  }

 private:
  void exitCallback(const geometry_msgs::PoseStampedConstPtr& message) {
    exit_position_ = message->pose.position;
    have_exit_ = true;
  }

  void candidatesCallback(const precision_landing::LandingPlatformArrayConstPtr& message) {
    if (assignments_ready_ || uav0_success_ || !have_exit_ ||
        message->platforms.size() < 2U) return;
    std::vector<precision_landing::LandingPlatform> candidates = message->platforms;
    std::sort(candidates.begin(), candidates.end(), [this](const auto& lhs, const auto& rhs) {
      const auto distance = [this](const auto& item) {
        const auto& p = item.pose.pose.position;
        return std::hypot(p.x - exit_position_.x, p.y - exit_position_.y);
      };
      return distance(lhs) < distance(rhs);
    });
    const auto& near_platform = candidates.front();
    const auto& far_platform = candidates.back();
    if (near_platform.id == far_platform.id) return;
    if (uav0_id_ == far_platform.id && uav1_id_ == near_platform.id) return;
    uav0_id_ = far_platform.id;
    uav1_id_ = near_platform.id;
    assignments_ready_ = true;
    uav0_target_ = far_platform.pose;
    uav1_target_ = near_platform.pose;
    uav0_target_.header = message->header;
    uav1_target_.header = message->header;
    publishTargets();
    publishId(uav0_assigned_pub_, uav0_id_);
    publishId(uav1_assigned_pub_, uav1_id_);
    publishBool(ready_pub_, true);
    publishStatus("已找到两个平台：UAV0 远平台，UAV1 近平台；等待 UAV0 降落");
  }

  void successCallback(const std_msgs::BoolConstPtr& message) {
    if (!message->data || uav0_id_ < 0 || uav1_id_ < 0 || uav0_success_) return;
    uav0_success_ = true;
    publishBool(release_pub_, true);
    publishStatus("UAV0 已完成降落，已释放 UAV1 出口等待点");
  }

  void targetTimerCallback(const ros::TimerEvent&) {
    if (assignments_ready_ && !uav0_success_) publishTargets();
  }

  void publishTargets() {
    const ros::Time now = ros::Time::now();
    uav0_target_.header.stamp = now;
    uav1_target_.header.stamp = now;
    uav0_target_pub_.publish(uav0_target_);
    uav1_target_pub_.publish(uav1_target_);
  }

  void publishStatus(const std::string& value) {
    if (value == last_status_) return;
    last_status_ = value;
    std_msgs::String message;
    message.data = value;
    status_pub_.publish(message);
    ROS_INFO_STREAM("dual_uav_landing: " << value << " (UAV0=" << uav0_id_ << ", UAV1=" << uav1_id_ << ")");
  }
  static void publishId(ros::Publisher& publisher, int value) {
    std_msgs::Int32 message; message.data = value; publisher.publish(message);
  }
  static void publishBool(ros::Publisher& publisher, bool value) {
    std_msgs::Bool message; message.data = value; publisher.publish(message);
  }

  ros::NodeHandle node_, private_node_;
  ros::Subscriber candidates_sub_, exit_sub_, success_sub_;
  ros::Timer target_timer_;
  ros::Publisher uav0_assigned_pub_, uav1_assigned_pub_, uav0_target_pub_, uav1_target_pub_;
  ros::Publisher release_pub_, ready_pub_, status_pub_;
  std::string uav0_assigned_topic_, uav1_assigned_topic_, uav0_target_topic_, uav1_target_topic_;
  std::string release_topic_, ready_topic_, status_topic_;
  geometry_msgs::Point exit_position_;
  geometry_msgs::PoseStamped uav0_target_, uav1_target_;
  bool have_exit_{false}, uav0_success_{false}, assignments_ready_{false};
  int uav0_id_{-1}, uav1_id_{-1};
  std::string last_status_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "dual_uav_landing_coordinator");
  DualUavLandingCoordinator coordinator;
  ros::spin();
  return 0;
}
