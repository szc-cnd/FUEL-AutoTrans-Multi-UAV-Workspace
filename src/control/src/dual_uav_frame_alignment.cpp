#include <cmath>
#include <stdexcept>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "dual_uav_frame_alignment");
  ros::NodeHandle nh;

  const std::string prefix = "/dual_uav_frame_alignment";
  std::string mission_frame;
  std::string follower_frame;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  if (!nh.getParam(prefix + "/mission_frame", mission_frame) ||
      !nh.getParam(prefix + "/follower_frame", follower_frame) ||
      !nh.getParam(prefix + "/follower/x", x) ||
      !nh.getParam(prefix + "/follower/y", y) ||
      !nh.getParam(prefix + "/follower/z", z) ||
      !nh.getParam(prefix + "/follower/yaw_rad", yaw)) {
    ROS_FATAL("dual_uav_frame_alignment: incomplete alignment parameters under %s",
              prefix.c_str());
    return 1;
  }
  if (mission_frame.empty() || follower_frame.empty() ||
      mission_frame == follower_frame || !std::isfinite(x) ||
      !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(yaw)) {
    ROS_FATAL("dual_uav_frame_alignment: invalid frame or transform parameters");
    return 1;
  }

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = mission_frame;
  transform.child_frame_id = follower_frame;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  transform.transform.rotation.x = rotation.x();
  transform.transform.rotation.y = rotation.y();
  transform.transform.rotation.z = rotation.z();
  transform.transform.rotation.w = rotation.w();

  tf2_ros::StaticTransformBroadcaster broadcaster;
  broadcaster.sendTransform(transform);
  ROS_WARN("dual_uav_frame_alignment: %s -> %s xyz=(%.3f, %.3f, %.3f) yaw=%.3f rad",
           mission_frame.c_str(), follower_frame.c_str(), x, y, z, yaw);
  ros::spin();
  return 0;
}
