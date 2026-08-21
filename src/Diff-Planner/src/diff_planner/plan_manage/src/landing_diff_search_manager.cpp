#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace {

std::string firstToken(const std::string& text) {
  const std::size_t end = text.find_first_of(" \t\r\n");
  return text.substr(0, end);
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

class LandingDiffSearchManager {
 public:
  LandingDiffSearchManager() : nh_(), private_nh_("~") {
    private_nh_.param("search_height", search_height_, 2.0);
    private_nh_.param("lane_spacing", lane_spacing_, 1.0);
    private_nh_.param("search_width", search_width_, 3.0);
    private_nh_.param("search_depth", search_depth_, 4.0);
    private_nh_.param("arrive_xy", arrive_xy_, 0.35);
    private_nh_.param("arrive_z", arrive_z_, 0.20);
    private_nh_.param("goal_reissue_sec", goal_reissue_sec_, 1.0);
    private_nh_.param("front_hint_dwell_sec", front_hint_dwell_sec_, 2.0);
    private_nh_.param("front_hint_initial_wait_sec",
                      front_hint_initial_wait_sec_, 1.0);
    private_nh_.param("front_yaw_scan_half_angle_rad",
                      front_yaw_scan_half_angle_rad_, 0.78539816339);
    private_nh_.param("front_yaw_scan_rate_rad_s",
                      front_yaw_scan_rate_rad_s_, 0.35);
    private_nh_.param("front_hint_forward_margin", front_hint_forward_margin_, 1.0);
    private_nh_.param("front_hint_lateral_margin", front_hint_lateral_margin_, 1.0);
    private_nh_.param("front_hint_min_z", front_hint_min_z_, -0.50);
    private_nh_.param("front_hint_max_z", front_hint_max_z_, 1.50);
    private_nh_.param("marker_timeout_sec", marker_timeout_sec_, 0.50);
    private_nh_.param("front_hint_timeout_sec", front_hint_timeout_sec_, 0.50);
    private_nh_.param("assigned_target_timeout_sec",
                      assigned_target_timeout_sec_, 0.50);
    private_nh_.param("enable_single_front_hint_approach",
                      enable_single_front_hint_approach_, true);
    private_nh_.param("max_header_future_sec", max_header_future_sec_, 0.05);

    std::string odom_topic("/UAV0/fast_lio/Odometry");
    std::string stage_topic("/UAV0/mission/task_status");
    std::string marker_topic("/UAV0/mission/detection/final_aruco");
    std::string front_hint_topic("/UAV0/landing/front_aruco_hint");
    std::string assigned_target_topic("/UAV0/landing/assigned_target");
    std::string subgoal_topic("/UAV0/landing_diff/subgoal");
    std::string trigger_topic("/UAV0/landing_diff/trigger");
    std::string yaw_topic("/UAV0/landing_diff/yaw");
    std::string request_topic("/UAV0/mission/landing_request");
    std::string target_frame("world");
    private_nh_.param("odom_topic", odom_topic, odom_topic);
    private_nh_.param("stage_topic", stage_topic, stage_topic);
    private_nh_.param("marker_topic", marker_topic, marker_topic);
    private_nh_.param("front_hint_topic", front_hint_topic, front_hint_topic);
    private_nh_.param("assigned_target_topic", assigned_target_topic,
                      assigned_target_topic);
    private_nh_.param("subgoal_topic", subgoal_topic, subgoal_topic);
    private_nh_.param("trigger_topic", trigger_topic, trigger_topic);
    private_nh_.param("yaw_topic", yaw_topic, yaw_topic);
    private_nh_.param("landing_request_topic", request_topic, request_topic);
    private_nh_.param("target_frame", target_frame, target_frame);
    target_frame_ = target_frame;

    if (target_frame_.empty() || marker_timeout_sec_ <= 0.0 ||
        front_hint_timeout_sec_ <= 0.0 || assigned_target_timeout_sec_ <= 0.0 ||
        front_hint_initial_wait_sec_ < 0.0 ||
        front_yaw_scan_half_angle_rad_ < 0.0 ||
        front_yaw_scan_rate_rad_s_ <= 0.0 || max_header_future_sec_ < 0.0) {
      throw std::runtime_error("landing_diff_search: invalid target freshness parameters");
    }

    odom_sub_ = nh_.subscribe(odom_topic, 20, &LandingDiffSearchManager::odomCallback, this);
    stage_sub_ = nh_.subscribe(stage_topic, 10, &LandingDiffSearchManager::stageCallback, this);
    marker_sub_ = nh_.subscribe(marker_topic, 10, &LandingDiffSearchManager::markerCallback, this);
    front_hint_sub_ = nh_.subscribe(front_hint_topic, 10,
                                    &LandingDiffSearchManager::frontHintCallback, this);
    assigned_target_sub_ = nh_.subscribe(
        assigned_target_topic, 10,
        &LandingDiffSearchManager::assignedTargetCallback, this);
    subgoal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(subgoal_topic, 2);
    trigger_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(trigger_topic, 2, true);
    yaw_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(yaw_topic, 2);
    landing_request_pub_ = nh_.advertise<std_msgs::Bool>(request_topic, 2, true);
    state_pub_ = private_nh_.advertise<std_msgs::String>("state", 2, true);
    timer_ = nh_.createTimer(ros::Duration(0.10), &LandingDiffSearchManager::timerCallback, this);
    publishRequest(false);
    publishState("WAIT_EXIT_SWITCH");
  }

 private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    odom_ = *msg;
    have_odom_ = true;
    if (switch_pending_ && !active_) activate();
  }

  void stageCallback(const std_msgs::StringConstPtr& msg) {
    const std::string stage = firstToken(msg->data);
    if (!switched_ && (stage == "SEARCH_OUTSIDE_LANDING" ||
                       stage == "SEARCH_OUTSIDE_QR")) {
      switch_pending_ = true;
      if (have_odom_) activate();
    }
    if (switched_ && stage == "LANDING") active_ = false;
  }

  void markerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    if (!switched_) return;
    if (!validTarget(*msg, marker_timeout_sec_, "final ArUco")) return;
    marker_ = *msg;
    have_marker_ = true;
    publishState("ARUCO_LOCKED_DIFF_APPROACH");
  }

  void frontHintCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    if (!enable_single_front_hint_approach_ || !switched_ ||
        front_hint_exhausted_ || have_marker_ || have_assigned_target_) return;
    if (!validTarget(*msg, front_hint_timeout_sec_, "front ArUco hint")) return;
    const geometry_msgs::Point& point = msg->pose.position;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || point.z < front_hint_min_z_ ||
        point.z > front_hint_max_z_) {
      ROS_WARN_THROTTLE(1.0, "landing_diff_search: reject invalid front ArUco hint");
      return;
    }
    const double dx = point.x - anchor_x_;
    const double dy = point.y - anchor_y_;
    const double forward = dx * forward_x_ + dy * forward_y_;
    const double lateral = dx * lateral_x_ + dy * lateral_y_;
    if (forward < -front_hint_forward_margin_ ||
        forward > search_depth_ + front_hint_forward_margin_ ||
        std::fabs(lateral) > 0.5 * search_width_ + front_hint_lateral_margin_) {
      ROS_WARN_THROTTLE(
          1.0,
          "landing_diff_search: reject front hint outside search fence forward=%.2f lateral=%.2f",
          forward, lateral);
      return;
    }
    if (!have_front_hint_ ||
        std::hypot(point.x - front_hint_.pose.position.x,
                   point.y - front_hint_.pose.position.y) > 0.20) {
      front_hint_reached_since_ = ros::Time(0);
    }
    front_hint_ = *msg;
    have_front_hint_ = true;
    front_hint_returning_ =
        std::fabs(front_yaw_scan_yaw_ - locked_yaw_) > 1.0e-3;
    publishState("FRONT_ARUCO_HINT_DIFF_APPROACH");
  }

  void assignedTargetCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    if (!switched_ || landing_requested_) return;
    if (!validTarget(*msg, assigned_target_timeout_sec_,
                     "assigned landing target")) return;
    assigned_target_ = *msg;
    have_assigned_target_ = true;
    have_front_hint_ = false;
    front_hint_returning_ = false;
    publishState("TWO_ARUCOS_ASSIGNED_APPROACH_FAR_PLATFORM");
  }

  void activate() {
    switched_ = true;
    active_ = true;
    switch_pending_ = false;
    anchor_x_ = odom_.pose.pose.position.x;
    anchor_y_ = odom_.pose.pose.position.y;
    anchor_z_ = odom_.pose.pose.position.z;
    const double yaw = yawFromQuaternion(odom_.pose.pose.orientation);
    locked_yaw_ = yaw;
    activated_at_ = ros::Time::now();
    initial_front_wait_complete_ = front_hint_initial_wait_sec_ <= 0.0;
    front_yaw_scan_yaw_ = locked_yaw_;
    front_yaw_scan_phase_ = 0;
    front_yaw_scan_complete_ = front_yaw_scan_half_angle_rad_ <= 0.0;
    last_front_yaw_scan_update_ = activated_at_;
    forward_x_ = std::cos(yaw);
    forward_y_ = std::sin(yaw);
    lateral_x_ = -forward_y_;
    lateral_y_ = forward_x_;
    buildSweepGoals();
    publishTrigger();
    publishState("FRONT_ARUCO_INITIAL_WAIT");
    ROS_ERROR("landing_diff_search: FUEL -> Diff switch latched at "
              "(%.2f, %.2f, %.2f), yaw=%.3f",
              anchor_x_, anchor_y_, anchor_z_, locked_yaw_);
  }

  void buildSweepGoals() {
    sweep_goals_.clear();
    geometry_msgs::Point climb;
    climb.x = anchor_x_;
    climb.y = anchor_y_;
    climb.z = search_height_;
    sweep_goals_.push_back(climb);
    const int rows = std::max(1, static_cast<int>(std::ceil(search_depth_ / lane_spacing_)));
    const double half_width = 0.5 * search_width_;
    for (int row = 0; row <= rows; ++row) {
      const double forward = std::min(search_depth_, row * lane_spacing_);
      const double first_lateral = (row % 2 == 0) ? -half_width : half_width;
      const double second_lateral = -first_lateral;
      for (double lateral : {first_lateral, second_lateral}) {
        geometry_msgs::Point point;
        point.x = anchor_x_ + forward * forward_x_ + lateral * lateral_x_;
        point.y = anchor_y_ + forward * forward_y_ + lateral * lateral_y_;
        point.z = search_height_;
        sweep_goals_.push_back(point);
      }
    }
    sweep_index_ = 0;
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!active_ || !have_odom_) return;
    // The precision-landing handoff is pending; do not resume sweep goals.
    if (landing_requested_) {
      publishSearchYaw();
      publishTrigger();
      return;
    }
    expireTargets();
    if (holdingForInitialFrontSearch()) {
      publishSearchYaw();
      publishTrigger();
      // 控制器在没有Diff轨迹时会自行保持当前位置。这里不能把当前位置
      // 作为子目标交给MINCO，否则零长度轨迹会产生零时长并触发断言。
      return;
    }
    publishSearchYaw();
    publishTrigger();
    geometry_msgs::Point goal;
    if (have_marker_) {
      goal = marker_.pose.position;
      goal.z += search_height_;
      if (arrived(goal)) {
        if (!landing_requested_) {
          publishRequest(true);
          publishState("DIFF_APPROACH_REACHED_LANDING_REQUESTED");
        }
        return;
      }
    } else if (have_assigned_target_) {
      goal = assigned_target_.pose.position;
      goal.z += search_height_;
      if (arrived(goal)) {
        publishState("ASSIGNED_TARGET_REACHED_WAIT_DOWN_CONFIRMATION");
      }
    } else if (have_front_hint_) {
      goal = front_hint_.pose.position;
      goal.z += search_height_;
      if (arrived(goal)) {
        if (front_hint_reached_since_.isZero()) {
          front_hint_reached_since_ = ros::Time::now();
          publishState("FRONT_HINT_REACHED_WAIT_DOWN_CAMERA");
        } else if ((ros::Time::now() - front_hint_reached_since_).toSec() >=
                   front_hint_dwell_sec_) {
          have_front_hint_ = false;
          front_hint_exhausted_ = true;
          front_hint_reached_since_ = ros::Time(0);
          selectNearestSweepGoal();
          publishState("FRONT_HINT_TIMEOUT_FALLBACK_DOWN_SWEEP");
        }
      } else {
        front_hint_reached_since_ = ros::Time(0);
      }
      if (!have_front_hint_) {
        if (sweep_goals_.empty()) return;
        goal = sweep_goals_[sweep_index_];
      }
    } else {
      if (sweep_goals_.empty()) return;
      if (arrived(sweep_goals_[sweep_index_])) {
        sweep_index_ = (sweep_index_ + 1) % sweep_goals_.size();
      }
      goal = sweep_goals_[sweep_index_];
    }
    publishGoalIfDue(goal);
  }

  bool holdingForInitialFrontSearch() {
    if (front_hint_returning_) {
      return returnFrontYawToExitHeading();
    }
    if (initial_front_wait_complete_ || have_marker_ || have_front_hint_ ||
        have_assigned_target_) {
      return !front_yaw_scan_complete_ && advanceFrontYawScan();
    }
    if ((ros::Time::now() - activated_at_).toSec() <
        front_hint_initial_wait_sec_) {
      return true;
    }
    initial_front_wait_complete_ = true;
    if (!front_yaw_scan_complete_) {
      publishState("FRONT_ARUCO_YAW_SCAN_LEFT");
    }
    return !front_yaw_scan_complete_ && advanceFrontYawScan();
  }

  bool advanceFrontYawScan() {
    if (front_yaw_scan_complete_ || have_marker_ || have_front_hint_ ||
        have_assigned_target_ ||
        front_hint_exhausted_) {
      return false;
    }
    const ros::Time now = ros::Time::now();
    const double elapsed = std::max(
        0.0, (now - last_front_yaw_scan_update_).toSec());
    last_front_yaw_scan_update_ = now;

    double target_yaw = locked_yaw_;
    if (front_yaw_scan_phase_ == 0) {
      target_yaw = locked_yaw_ - front_yaw_scan_half_angle_rad_;
    } else if (front_yaw_scan_phase_ == 1) {
      target_yaw = locked_yaw_ + front_yaw_scan_half_angle_rad_;
    }
    const double remaining = target_yaw - front_yaw_scan_yaw_;
    const double step = front_yaw_scan_rate_rad_s_ * elapsed;
    if (std::fabs(remaining) <= step) {
      front_yaw_scan_yaw_ = target_yaw;
      ++front_yaw_scan_phase_;
      if (front_yaw_scan_phase_ >= 3) {
        front_yaw_scan_complete_ = true;
        publishState("FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP");
        return false;
      }
      if (front_yaw_scan_phase_ == 1) {
        publishState("FRONT_ARUCO_YAW_SCAN_RIGHT");
      } else {
        publishState("FRONT_ARUCO_YAW_SCAN_RETURN");
      }
    } else {
      front_yaw_scan_yaw_ += std::copysign(step, remaining);
    }
    return true;
  }

  bool returnFrontYawToExitHeading() {
    const ros::Time now = ros::Time::now();
    const double elapsed = std::max(
        0.0, (now - last_front_yaw_scan_update_).toSec());
    last_front_yaw_scan_update_ = now;
    const double remaining = locked_yaw_ - front_yaw_scan_yaw_;
    const double step = front_yaw_scan_rate_rad_s_ * elapsed;
    if (std::fabs(remaining) <= step) {
      front_yaw_scan_yaw_ = locked_yaw_;
      front_hint_returning_ = false;
      publishState("FRONT_ARUCO_HINT_RETURN_COMPLETE_APPROACH");
      return false;
    }
    front_yaw_scan_yaw_ += std::copysign(step, remaining);
    return true;
  }

  void selectNearestSweepGoal() {
    if (sweep_goals_.empty() || !have_odom_) return;
    const geometry_msgs::Point& current = odom_.pose.pose.position;
    double best_distance = std::numeric_limits<double>::infinity();
    std::size_t best_index = 0U;
    for (std::size_t index = 0U; index < sweep_goals_.size(); ++index) {
      const geometry_msgs::Point& candidate = sweep_goals_[index];
      const double distance = std::hypot(candidate.x - current.x,
                                         candidate.y - current.y);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = index;
      }
    }
    sweep_index_ = best_index;
  }

  bool arrived(const geometry_msgs::Point& goal) const {
    const geometry_msgs::Point& current = odom_.pose.pose.position;
    return std::hypot(current.x - goal.x, current.y - goal.y) <= arrive_xy_ &&
           std::fabs(current.z - goal.z) <= arrive_z_;
  }

  bool validTarget(const geometry_msgs::PoseStamped& target,
                   double timeout_sec,
                   const char* label) const {
    const geometry_msgs::Point& point = target.pose.position;
    if (target.header.frame_id != target_frame_ || target.header.stamp.isZero()) {
      ROS_WARN_THROTTLE(1.0,
                        "landing_diff_search: reject %s with invalid frame or stamp",
                        label);
      return false;
    }
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      ROS_WARN_THROTTLE(1.0,
                        "landing_diff_search: reject %s with non-finite position",
                        label);
      return false;
    }
    const double age_sec = (ros::Time::now() - target.header.stamp).toSec();
    if (age_sec < -max_header_future_sec_ || age_sec > timeout_sec) {
      ROS_WARN_THROTTLE(1.0,
                        "landing_diff_search: reject stale %s age=%.3fs",
                        label, age_sec);
      return false;
    }
    return true;
  }

  void expireTargets() {
    if (have_marker_ && !validTarget(marker_, marker_timeout_sec_, "final ArUco")) {
      have_marker_ = false;
      publishState("FINAL_ARUCO_TIMEOUT_FALLBACK_DOWN_SWEEP");
    }
    // 前视相机提示在接收时已经过时间戳、世界系和搜索边界校验。
    // 平台在世界系中静止，飞近后离开 D435 视野是正常现象，因此粗提示
    // 必须锁存到到达或被下视 final ArUco 取代，不按消息年龄中途清除。
  }

  void publishGoal(const geometry_msgs::Point& point) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position = point;
    goal.pose.orientation.w = std::cos(0.5 * commandedYaw());
    goal.pose.orientation.z = std::sin(0.5 * commandedYaw());
    subgoal_pub_.publish(goal);
    last_goal_time_ = goal.header.stamp;
  }

  void publishGoalIfDue(const geometry_msgs::Point& point) {
    if (last_goal_time_.isZero() ||
        (ros::Time::now() - last_goal_time_).toSec() >= goal_reissue_sec_) {
      publishGoal(point);
    }
  }

  void publishTrigger() {
    if (!last_trigger_time_.isZero() &&
        (ros::Time::now() - last_trigger_time_).toSec() < 1.0) return;
    geometry_msgs::PoseStamped trigger;
    trigger.header.stamp = ros::Time::now();
    trigger.header.frame_id = "world";
    trigger.pose = odom_.pose.pose;
    trigger_pub_.publish(trigger);
    last_trigger_time_ = trigger.header.stamp;
  }

  void publishRequest(bool active) {
    std_msgs::Bool request;
    request.data = active;
    landing_request_pub_.publish(request);
    landing_requested_ = active;
  }

  double commandedYaw() const {
    if (front_hint_returning_) return front_yaw_scan_yaw_;
    return front_yaw_scan_complete_ || have_marker_ || have_front_hint_ ||
                   have_assigned_target_ ||
                   front_hint_exhausted_
               ? locked_yaw_
               : front_yaw_scan_yaw_;
  }

  void publishSearchYaw() {
    quadrotor_msgs::PositionCommand command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = "world";
    command.yaw = commandedYaw();
    command.yaw_dot = 0.0;
    yaw_pub_.publish(command);
  }

  void publishState(const std::string& state) {
    std_msgs::String msg;
    msg.data = state;
    state_pub_.publish(msg);
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber odom_sub_, stage_sub_, marker_sub_, front_hint_sub_,
      assigned_target_sub_;
  ros::Publisher subgoal_pub_, trigger_pub_, yaw_pub_, landing_request_pub_, state_pub_;
  ros::Timer timer_;
  nav_msgs::Odometry odom_;
  geometry_msgs::PoseStamped marker_;
  geometry_msgs::PoseStamped front_hint_;
  geometry_msgs::PoseStamped assigned_target_;
  std::vector<geometry_msgs::Point> sweep_goals_;
  std::size_t sweep_index_{0};
  bool have_odom_{false}, switch_pending_{false}, switched_{false}, active_{false};
  bool have_marker_{false}, landing_requested_{false};
  bool have_front_hint_{false}, front_hint_exhausted_{false};
  bool have_assigned_target_{false};
  bool enable_single_front_hint_approach_{true};
  double search_height_{2.0}, lane_spacing_{1.0}, search_width_{3.0}, search_depth_{4.0};
  double arrive_xy_{0.35}, arrive_z_{0.20}, goal_reissue_sec_{1.0};
  double front_hint_dwell_sec_{2.0}, front_hint_forward_margin_{1.0};
  double front_hint_initial_wait_sec_{1.0};
  double front_yaw_scan_half_angle_rad_{0.78539816339};
  double front_yaw_scan_rate_rad_s_{0.35};
  double front_hint_lateral_margin_{1.0}, front_hint_min_z_{-0.50};
  double front_hint_max_z_{1.50};
  double marker_timeout_sec_{0.50}, front_hint_timeout_sec_{0.50};
  double assigned_target_timeout_sec_{0.50};
  double max_header_future_sec_{0.05};
  double anchor_x_{0.0}, anchor_y_{0.0}, anchor_z_{0.0};
  double forward_x_{1.0}, forward_y_{0.0};
  double lateral_x_{0.0}, lateral_y_{1.0};
  double locked_yaw_{0.0};
  std::string target_frame_{"world"};
  bool initial_front_wait_complete_{false}, front_yaw_scan_complete_{false};
  bool front_hint_returning_{false};
  int front_yaw_scan_phase_{0};
  double front_yaw_scan_yaw_{0.0};
  ros::Time activated_at_, last_goal_time_, last_trigger_time_,
      front_hint_reached_since_, last_front_yaw_scan_update_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "landing_diff_search_manager");
  LandingDiffSearchManager manager;
  ros::spin();
  return 0;
}
