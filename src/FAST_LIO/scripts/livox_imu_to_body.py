#!/usr/bin/env python3

"""将带安装倾角的 Livox IMU 数据旋转到无人机机体系。"""

import math


def rotation_y(pitch_deg):
    pitch = math.radians(pitch_deg)
    cosine = math.cos(pitch)
    sine = math.sin(pitch)
    return (
        (cosine, 0.0, sine),
        (0.0, 1.0, 0.0),
        (-sine, 0.0, cosine),
    )


def rotate_vector(rotation, values):
    return tuple(
        sum(rotation[row][column] * values[column] for column in range(3))
        for row in range(3)
    )


def rotate_covariance(rotation, covariance):
    """计算 R C R^T；ROS 用首项 -1 表示协方差未知。"""
    if len(covariance) != 9 or covariance[0] < 0.0:
        return tuple(covariance)

    source = tuple(tuple(covariance[3 * row + column] for column in range(3))
                   for row in range(3))
    left = tuple(
        tuple(sum(rotation[row][k] * source[k][column] for k in range(3))
              for column in range(3))
        for row in range(3)
    )
    result = tuple(
        tuple(sum(left[row][k] * rotation[column][k] for k in range(3))
              for column in range(3))
        for row in range(3)
    )
    return tuple(result[row][column] for row in range(3) for column in range(3))


def main():
    import rospy
    from sensor_msgs.msg import Imu

    rospy.init_node("livox_imu_to_body")
    input_topic = rospy.get_param("~input_topic", "/UAV1/livox/imu")
    output_topic = rospy.get_param("~output_topic", "/UAV1/livox/imu_body")
    output_frame_id = rospy.get_param("~output_frame_id", "UAV1/body")
    pitch_deg = float(rospy.get_param("~pitch_deg", 15.0))
    rotation = rotation_y(pitch_deg)

    publisher = rospy.Publisher(output_topic, Imu, queue_size=200)

    def callback(source):
        target = Imu()
        target.header = source.header
        target.header.frame_id = output_frame_id
        target.orientation = source.orientation
        target.orientation_covariance = source.orientation_covariance

        angular = rotate_vector(
            rotation,
            (source.angular_velocity.x,
             source.angular_velocity.y,
             source.angular_velocity.z))
        target.angular_velocity.x, target.angular_velocity.y, target.angular_velocity.z = angular
        target.angular_velocity_covariance = rotate_covariance(
            rotation, source.angular_velocity_covariance)

        acceleration = rotate_vector(
            rotation,
            (source.linear_acceleration.x,
             source.linear_acceleration.y,
             source.linear_acceleration.z))
        (target.linear_acceleration.x,
         target.linear_acceleration.y,
         target.linear_acceleration.z) = acceleration
        target.linear_acceleration_covariance = rotate_covariance(
            rotation, source.linear_acceleration_covariance)
        publisher.publish(target)

    rospy.Subscriber(input_topic, Imu, callback, queue_size=200, tcp_nodelay=True)
    rospy.loginfo(
        "[Livox IMU坐标转换] 输入=%s，输出=%s，绕Y轴旋转=%.3f度，frame_id=%s。",
        input_topic, output_topic, pitch_deg, output_frame_id)
    rospy.spin()


if __name__ == "__main__":
    main()
