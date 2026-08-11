#!/usr/bin/env python3
"""Broadcast FAST-LIO nav_msgs/Odometry as a dynamic ROS TF transform."""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdometryTfBridge:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.parent_override = rospy.get_param("~parent_frame", "").strip()
        self.child_override = rospy.get_param("~child_frame", "").strip()
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.subscriber = rospy.Subscriber(
            self.odom_topic, Odometry, self._callback, queue_size=100
        )
        rospy.loginfo("Broadcasting Odometry TF from %s", self.odom_topic)

    def _callback(self, message):
        parent_frame = self.parent_override or message.header.frame_id
        child_frame = self.child_override or message.child_frame_id
        if not parent_frame or not child_frame:
            rospy.logwarn_throttle(2.0, "Odometry message has empty TF frame names")
            return
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = parent_frame.lstrip("/")
        transform.child_frame_id = child_frame.lstrip("/")
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rospy.init_node("odometry_to_tf")
    OdometryTfBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
