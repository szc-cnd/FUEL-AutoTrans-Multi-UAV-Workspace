#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

#include "plan_manage/corridor_dynamic_rechecker.h"

class CorridorDynamicRecheckerNode {
 public:
  CorridorDynamicRecheckerNode() : nh_(), pnh_("~") {
    std::string cloud_topic, odom_topic, object_topic, marker_topic;
    pnh_.param("cloud_topic", cloud_topic, std::string("/UAV1/fast_lio/cloud_registered"));
    pnh_.param("odom_topic", odom_topic, std::string("/UAV1/fast_lio/Odom_high_freq"));
    pnh_.param("object_topic", object_topic, std::string("/UAV1/corridor_dynamic_objects"));
    pnh_.param("marker_topic", marker_topic, std::string("/UAV1/corridor_dynamic_markers"));
    diff_planner::CorridorDynamicRecheckerConfig config;
    pnh_.param("corridor_width", config.corridor_width, config.corridor_width);
    pnh_.param("wall_clearance", config.wall_clearance, config.wall_clearance);
    pnh_.param("roi_min_z", config.roi_min_z, config.roi_min_z);
    pnh_.param("roi_max_z", config.roi_max_z, config.roi_max_z);
    pnh_.param("voxel_resolution", config.voxel_resolution, config.voxel_resolution);
    pnh_.param("history_frames", config.history_frames, config.history_frames);
    pnh_.param("min_confirm_hits", config.min_confirm_hits, config.min_confirm_hits);
    pnh_.param("min_lateral_speed", config.min_lateral_speed, config.min_lateral_speed);
    pnh_.param("max_forward_speed", config.max_forward_speed, config.max_forward_speed);
    pnh_.param("max_vertical_speed", config.max_vertical_speed, config.max_vertical_speed);
    pnh_.param("track_timeout", config.track_timeout, config.track_timeout);
    pnh_.param("min_cluster_voxels", config.min_cluster_voxels, config.min_cluster_voxels);
    pnh_.param("min_wall_points", config.min_wall_points, config.min_wall_points);
    rechecker_.reset(new diff_planner::CorridorDynamicRechecker(config));
    objects_pub_ = nh_.advertise<ldop::DynamicObjectArray>(object_topic, 5);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic, 2);
    odom_sub_ = nh_.subscribe(odom_topic, 10, &CorridorDynamicRecheckerNode::odomCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic, 5, &CorridorDynamicRecheckerNode::cloudCallback, this);
  }

 private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg) { latest_odom_ = *msg; have_odom_ = true; }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!have_odom_) return;
    std::vector<Eigen::Vector3d> points;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
      for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        if (std::isfinite(*ix) && std::isfinite(*iy) && std::isfinite(*iz))
          points.emplace_back(*ix, *iy, *iz);
      }
    } catch (const std::runtime_error&) { ROS_WARN_THROTTLE(2.0, "cloud has no x/y/z fields"); return; }
    const Eigen::Vector3d sensor_position(latest_odom_.pose.pose.position.x,
                                           latest_odom_.pose.pose.position.y,
                                           latest_odom_.pose.pose.position.z);
    const Eigen::Vector3d odom_velocity(latest_odom_.twist.twist.linear.x,
                                         latest_odom_.twist.twist.linear.y,
                                         latest_odom_.twist.twist.linear.z);
    const auto detected = rechecker_->process(points, msg->header.stamp, sensor_position, odom_velocity);
    ldop::DynamicObjectArray output; output.header = msg->header;
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear; clear.action = visualization_msgs::Marker::DELETEALL; markers.markers.push_back(clear);
    for (const auto& object : detected) {
      ldop::DynamicObject msg_object; msg_object.id = object.id; msg_object.size.x = object.size.x();
      msg_object.size.y = object.size.y(); msg_object.size.z = object.size.z();
      msg_object.model_state = {object.position.x(), object.position.y(), object.position.z(),
                                object.velocity.x(), object.velocity.y(), object.velocity.z()};
      msg_object.motion_model_type = ldop::DynamicObject::MOTION_MODEL_CV3D;
      msg_object.object_class = ldop::DynamicObject::CLASS_UNKNOWN; output.objects.push_back(msg_object);
      visualization_msgs::Marker marker; marker.header = msg->header; marker.ns = "corridor_dynamic_rechecker";
      marker.id = static_cast<int>(object.id); marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD; marker.pose.position.x = object.position.x();
      marker.pose.position.y = object.position.y(); marker.pose.position.z = object.position.z(); marker.pose.orientation.w = 1.0;
      marker.scale.x = object.size.x(); marker.scale.y = object.size.y(); marker.scale.z = object.size.z();
      marker.color.r = 1.0; marker.color.g = 0.15; marker.color.a = 0.8; markers.markers.push_back(marker);
    }
    objects_pub_.publish(output); markers_pub_.publish(markers);
  }

  ros::NodeHandle nh_, pnh_; ros::Subscriber cloud_sub_, odom_sub_;
  ros::Publisher objects_pub_, markers_pub_; nav_msgs::Odometry latest_odom_; bool have_odom_ = false;
  std::shared_ptr<diff_planner::CorridorDynamicRechecker> rechecker_;
};

int main(int argc, char** argv) { ros::init(argc, argv, "corridor_dynamic_rechecker"); CorridorDynamicRecheckerNode node; ros::spin(); return 0; }
