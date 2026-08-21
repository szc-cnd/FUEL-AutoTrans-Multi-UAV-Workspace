#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "precision_landing/LandingPlatformArray.h"

namespace {

std::string firstToken(const std::string& text) {
  const std::size_t end = text.find_first_of(" \t\r\n");
  return text.substr(0, end);
}

class DualUavLandingCoordinator {
 public:
  DualUavLandingCoordinator() : private_node_("~") {
    std::string candidates_topic{"/UAV0/landing/search/candidates"};
    std::string front_candidates_topic{"/UAV0/landing/front/candidates"};
    std::string exit_topic{"/UAV0/mission/final_exit"};
    std::string success_topic{"/UAV0/landing/success"};
    std::string search_state_topic{"/landing_diff_search_manager/state"};
    private_node_.param("topics/candidates", candidates_topic, candidates_topic);
    private_node_.param("topics/front_candidates", front_candidates_topic,
                        front_candidates_topic);
    private_node_.param("topics/final_exit", exit_topic, exit_topic);
    private_node_.param("topics/uav0_success", success_topic, success_topic);
    private_node_.param("topics/search_state", search_state_topic,
                        search_state_topic);
    private_node_.param("topics/uav0_assigned", uav0_assigned_topic_, std::string("/UAV0/landing/assigned_id"));
    private_node_.param("topics/uav1_assigned", uav1_assigned_topic_, std::string("/UAV1/landing/assigned_id"));
    private_node_.param("topics/uav0_target", uav0_target_topic_, std::string("/UAV0/landing/assigned_target"));
    private_node_.param("topics/uav1_target", uav1_target_topic_, std::string("/UAV1/landing/assigned_target"));
    private_node_.param("topics/release_uav1", release_topic_, std::string("/dual_uav_landing/release_uav1"));
    private_node_.param("topics/assignments_ready", ready_topic_, std::string("/dual_uav_landing/assignments_ready"));
    private_node_.param("topics/status", status_topic_, std::string("/dual_uav_landing/status"));
    private_node_.param("candidate_message_timeout_sec",
                        candidate_message_timeout_sec_, 0.50);
    private_node_.param("max_header_future_sec", max_header_future_sec_, 0.05);

    if (candidate_message_timeout_sec_ <= 0.0 || max_header_future_sec_ < 0.0) {
      throw std::runtime_error("dual_uav_landing: invalid candidate freshness parameters");
    }

    uav0_assigned_pub_ = node_.advertise<std_msgs::Int32>(uav0_assigned_topic_, 1, true);
    uav1_assigned_pub_ = node_.advertise<std_msgs::Int32>(uav1_assigned_topic_, 1, true);
    uav0_target_pub_ = node_.advertise<geometry_msgs::PoseStamped>(uav0_target_topic_, 1, true);
    uav1_target_pub_ = node_.advertise<geometry_msgs::PoseStamped>(uav1_target_topic_, 1, true);
    release_pub_ = node_.advertise<std_msgs::Bool>(release_topic_, 1, true);
    ready_pub_ = node_.advertise<std_msgs::Bool>(ready_topic_, 1, true);
    status_pub_ = node_.advertise<std_msgs::String>(status_topic_, 1, true);
    candidates_sub_ = node_.subscribe(candidates_topic, 2, &DualUavLandingCoordinator::candidatesCallback, this);
    front_candidates_sub_ = node_.subscribe(
        front_candidates_topic, 2,
        &DualUavLandingCoordinator::frontCandidatesCallback, this);
    exit_sub_ = node_.subscribe(exit_topic, 1, &DualUavLandingCoordinator::exitCallback, this);
    success_sub_ = node_.subscribe(success_topic, 2, &DualUavLandingCoordinator::successCallback, this);
    search_state_sub_ = node_.subscribe(
        search_state_topic, 5,
        &DualUavLandingCoordinator::searchStateCallback, this);
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
    tryAssignPlatforms();
  }

  void candidatesCallback(const precision_landing::LandingPlatformArrayConstPtr& message) {
    updateCandidates(*message, true);
  }

  void frontCandidatesCallback(
      const precision_landing::LandingPlatformArrayConstPtr& message) {
    updateCandidates(*message, false);
  }

  static bool frontCandidatesAllowed(const std::string& state) {
    return state == "FRONT_ARUCO_INITIAL_WAIT" ||
           state == "FRONT_ARUCO_YAW_SCAN_LEFT" ||
           state == "FRONT_ARUCO_YAW_SCAN_RIGHT" ||
           state == "FRONT_ARUCO_YAW_SCAN_RETURN";
  }

  static bool downwardCandidatesAllowed(const std::string& state) {
    return state == "FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP" ||
           state == "FRONT_ARUCO_HINT_DIFF_APPROACH" ||
           state == "FRONT_ARUCO_HINT_RETURN_COMPLETE_APPROACH" ||
           state == "FRONT_HINT_REACHED_WAIT_DOWN_CAMERA" ||
           state == "FRONT_HINT_TIMEOUT_FALLBACK_DOWN_SWEEP" ||
           state == "FINAL_ARUCO_TIMEOUT_FALLBACK_DOWN_SWEEP" ||
           state == "ARUCO_LOCKED_DIFF_APPROACH";
  }

  static bool frontScanCompleted(const std::string& state) {
    return state == "FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP" ||
           state == "FRONT_ARUCO_HINT_DIFF_APPROACH" ||
           state == "FRONT_ARUCO_HINT_RETURN_COMPLETE_APPROACH" ||
           state == "FRONT_HINT_REACHED_WAIT_DOWN_CAMERA" ||
           state == "FRONT_HINT_TIMEOUT_FALLBACK_DOWN_SWEEP" ||
           state == "FINAL_ARUCO_TIMEOUT_FALLBACK_DOWN_SWEEP" ||
           state == "ARUCO_LOCKED_DIFF_APPROACH" ||
           state == "ASSIGNED_TARGET_REACHED_WAIT_DOWN_CONFIRMATION";
  }

  void resetSearchCandidates() {
    candidates_by_id_.clear();
    candidate_order_.clear();
    downward_ids_.clear();
    front_scan_completed_ = false;
  }

  void searchStateCallback(const std_msgs::StringConstPtr& message) {
    const std::string next_state = firstToken(message->data);
    if (next_state == "FRONT_ARUCO_FORWARD_APPROACH" &&
        landing_search_state_ != next_state && !assignments_ready_) {
      // 新一轮 CH9 搜索开始时清空上轮或锁存话题带来的候选。前飞阶段不采集，
      // 到达扫描点后再保留本轮前视相机给出的粗位置。
      resetSearchCandidates();
    }
    landing_search_state_ = next_state;
    if (frontScanCompleted(next_state)) {
      front_scan_completed_ = true;
      tryAssignPlatforms();
    }
  }

  void updateCandidates(const precision_landing::LandingPlatformArray& message,
                        bool downward_source) {
    if (assignments_ready_ || uav0_success_) return;
    if ((downward_source &&
         !downwardCandidatesAllowed(landing_search_state_)) ||
        (!downward_source &&
         !frontCandidatesAllowed(landing_search_state_))) {
      return;
    }
    const double message_age_sec =
        (ros::Time::now() - message.header.stamp).toSec();
    if (message.header.stamp.isZero() ||
        message_age_sec < -max_header_future_sec_ ||
        message_age_sec > candidate_message_timeout_sec_) {
      ROS_WARN_THROTTLE(
          1.0,
          "dual_uav_landing: reject stale candidate array age=%.3fs",
          message_age_sec);
      return;
    }
    for (const auto& platform : message.platforms) {
      const auto& point = platform.pose.pose.position;
      if (platform.id < 0 || !std::isfinite(point.x) ||
          !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (candidates_by_id_.count(platform.id) == 0U) {
        candidate_order_.push_back(platform.id);
      }
      if (downward_source) {
        downward_ids_.insert(platform.id);
        candidates_by_id_[platform.id] = platform;
      } else if (downward_ids_.count(platform.id) == 0U) {
        candidates_by_id_[platform.id] = platform;
      }
    }
    tryAssignPlatforms();
  }

  void tryAssignPlatforms() {
    if (assignments_ready_ || uav0_success_ || !have_exit_ ||
        !front_scan_completed_ || candidate_order_.size() < 2U) return;
    const auto first = candidates_by_id_.find(candidate_order_[0]);
    const auto second = candidates_by_id_.find(candidate_order_[1]);
    if (first == candidates_by_id_.end() || second == candidates_by_id_.end() ||
        first->first == second->first) {
      return;
    }
    // 前机负责完成双码搜索：第一个稳定确认的平台交给后机，前机在确认
    // 第二个平台后前往其粗位置，并由下视相机完成最终定位与精降。
    const auto& uav1_platform = first->second;
    const auto& uav0_platform = second->second;
    uav0_id_ = uav0_platform.id;
    uav1_id_ = uav1_platform.id;
    assignments_ready_ = true;
    uav0_target_ = uav0_platform.pose;
    uav1_target_ = uav1_platform.pose;
    uav0_target_.header = uav0_platform.pose.header;
    uav1_target_.header = uav1_platform.pose.header;
    publishTargets();
    publishId(uav0_assigned_pub_, uav0_id_);
    publishId(uav1_assigned_pub_, uav1_id_);
    publishBool(ready_pub_, true);
    publishStatus("左右扫描完成并确认两个平台：UAV1 第一个，UAV0 第二个；等待 UAV0 降落");
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
  ros::Subscriber candidates_sub_, front_candidates_sub_, exit_sub_, success_sub_,
      search_state_sub_;
  ros::Timer target_timer_;
  ros::Publisher uav0_assigned_pub_, uav1_assigned_pub_, uav0_target_pub_, uav1_target_pub_;
  ros::Publisher release_pub_, ready_pub_, status_pub_;
  std::string uav0_assigned_topic_, uav1_assigned_topic_, uav0_target_topic_, uav1_target_topic_;
  std::string release_topic_, ready_topic_, status_topic_;
  geometry_msgs::Point exit_position_;
  geometry_msgs::PoseStamped uav0_target_, uav1_target_;
  std::map<int, precision_landing::LandingPlatform> candidates_by_id_;
  std::vector<int> candidate_order_;
  std::set<int> downward_ids_;
  bool have_exit_{false}, uav0_success_{false}, assignments_ready_{false};
  bool front_scan_completed_{false};
  int uav0_id_{-1}, uav1_id_{-1};
  double candidate_message_timeout_sec_{0.50};
  double max_header_future_sec_{0.05};
  std::string last_status_;
  std::string landing_search_state_{"WAIT_EXIT_SWITCH"};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "dual_uav_landing_coordinator");
  DualUavLandingCoordinator coordinator;
  ros::spin();
  return 0;
}
