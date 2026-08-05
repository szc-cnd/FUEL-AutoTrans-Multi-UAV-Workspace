#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class GoalHeightBridge
{
public:
  GoalHeightBridge()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh.param<std::string>("input_goal_topic", input_goal_topic_, "/move_base_simple/goal");
    pnh.param<std::string>("output_goal_topic", output_goal_topic_, "/goal");
    pnh.param<std::string>("output_frame_id", output_frame_id_, "world");

    // 2026-07-06: 记录当前里程计高度，用于把 RViz 的 2D 目标补成当前高度的 3D 目标。
    odom_sub_ = nh.subscribe(odom_topic_, 10, &GoalHeightBridge::odomCallback, this);
    goal_sub_ = nh.subscribe(input_goal_topic_, 10, &GoalHeightBridge::goalCallback, this);
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(output_goal_topic_, 10);
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    current_z_ = msg->pose.pose.position.z;
    have_odom_ = true;
  }

  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    if (!have_odom_)
    {
      ROS_WARN_THROTTLE(1.0, "goal_height_bridge: waiting for odometry before forwarding goal");
      return;
    }

    geometry_msgs::PoseStamped goal = *msg;
    // 2026-07-06: 保留 RViz 点击得到的平面目标，Z 轴对齐到当前飞行高度，避免 2D Nav Goal 把高度压到地面。
    goal.pose.position.z = current_z_;
    if (goal.header.frame_id.empty())
      goal.header.frame_id = output_frame_id_;
    goal.header.stamp = ros::Time::now();

    goal_pub_.publish(goal);
    ROS_INFO("goal_height_bridge: forward goal x=%.2f y=%.2f z=%.2f frame=%s",
             goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal.header.frame_id.c_str());
  }

  ros::Subscriber odom_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher goal_pub_;

  std::string odom_topic_;
  std::string input_goal_topic_;
  std::string output_goal_topic_;
  std::string output_frame_id_;

  double current_z_{0.0};
  bool have_odom_{false};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_height_bridge");
  GoalHeightBridge node;
  ros::spin();
  return 0;
}
