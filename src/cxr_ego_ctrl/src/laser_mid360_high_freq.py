#!/usr/bin/env python3

import math


def is_valid_pose(pose):
    """检查位置与 body->world 四元数是否为有限有效值。"""
    q = pose.orientation
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        q.x, q.y, q.z, q.w,
    )
    if not all(math.isfinite(value) for value in values):
        return False

    q_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    return q_norm > 1.0e-6


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry

    rospy.init_node('UAV1_fastlio_high_freq_vision_bridge')

    input_topic = rospy.get_param(
        '~input_topic', '/UAV1/fast_lio/Odom_high_freq')
    output_topic = rospy.get_param(
        '~output_topic', '/UAV1/mavros/vision_pose/pose')
    expected_frame_id = rospy.get_param(
        '~expected_frame_id', 'UAV1/camera_init')

    # MAVROS/PX4 外部视觉输入。位置单位 m；四元数顺序 x,y,z,w，表示机体到
    # UAV1/camera_init 世界系的姿态。该节点不进行坐标轴或 ENU/NED 手工转换。
    pose_pub = rospy.Publisher(output_topic, PoseStamped, queue_size=1)

    def odom_callback(msg):
        if msg.header.frame_id != expected_frame_id:
            rospy.logwarn_throttle(
                1.0,
                '[高频视觉转发] 拒绝 frame_id=%s，期望 %s。',
                msg.header.frame_id,
                expected_frame_id)
            return

        if not is_valid_pose(msg.pose.pose):
            rospy.logwarn_throttle(1.0, '[高频视觉转发] 位姿或四元数无效，停止本帧发布。')
            return

        source_q = msg.pose.pose.orientation
        q_norm = math.sqrt(
            source_q.x * source_q.x + source_q.y * source_q.y +
            source_q.z * source_q.z + source_q.w * source_q.w)

        vision_pose = PoseStamped()
        vision_pose.header.stamp = (
            msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now())
        vision_pose.header.frame_id = expected_frame_id
        vision_pose.pose.position = msg.pose.pose.position
        vision_pose.pose.orientation.x = source_q.x / q_norm
        vision_pose.pose.orientation.y = source_q.y / q_norm
        vision_pose.pose.orientation.z = source_q.z / q_norm
        vision_pose.pose.orientation.w = source_q.w / q_norm
        pose_pub.publish(vision_pose)

    rospy.Subscriber(
        input_topic,
        Odometry,
        odom_callback,
        queue_size=1,
        tcp_nodelay=True)

    rospy.loginfo(
        '[高频视觉转发] 输入=%s，输出=%s，坐标系=%s。',
        input_topic,
        output_topic,
        expected_frame_id)
    rospy.spin()


if __name__ == '__main__':
    main()
