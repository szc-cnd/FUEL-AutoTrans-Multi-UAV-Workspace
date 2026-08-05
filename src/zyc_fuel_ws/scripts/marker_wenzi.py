#!/usr/bin/env python
#手动定义小球坐标，检测到小球后延时3秒显示小球
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

# 全局变量存储检测状态和上次检测时间
detection_active = False
last_detection_time = None

def detection_callback(msg):
    """检测状态的回调函数"""
    global detection_active, last_detection_time
    if msg.data and not detection_active:  # 如果是新的检测
        last_detection_time = rospy.Time.now()
    detection_active = msg.data

def publish_marker():
    global detection_active, last_detection_time
    
    rospy.init_node('visualization_marker_demo', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    detection_sub = rospy.Subscriber('/detection_status', Bool, detection_callback)
    
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        if detection_active and last_detection_time:
            # 检查是否已经过了3秒
            if (rospy.Time.now() - last_detection_time).to_sec() >= 3.0:
                # 创建小球标记
                ball_marker = Marker()
                ball_marker.header.frame_id = "world"
                ball_marker.header.stamp = rospy.Time.now()
                ball_marker.ns = "demo"
                ball_marker.id = 0
                ball_marker.type = Marker.SPHERE
                ball_marker.action = Marker.ADD

                ball_marker.pose.position.x = 3.22
                ball_marker.pose.position.y = -3.11
                ball_marker.pose.position.z = 0.78

                ball_marker.scale.x = 0.50
                ball_marker.scale.y = 0.50  
                ball_marker.scale.z = 0.50

                ball_marker.color.r = 1.0
                ball_marker.color.g = 0.0
                ball_marker.color.b = 0.0
                ball_marker.color.a = 1.0

                marker_pub.publish(ball_marker)

                # 创建文本标记
                text_marker = Marker()
                text_marker.header.frame_id = "world"
                text_marker.header.stamp = rospy.Time.now()
                text_marker.ns = "demo"
                text_marker.id = 1
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD

                text_marker.pose.position.x = ball_marker.pose.position.x
                text_marker.pose.position.y = ball_marker.pose.position.y
                text_marker.pose.position.z = ball_marker.pose.position.z + 1.0

                text_marker.text = "x: {:.2f}m, y: {:.2f}m, z: {:.2f}m".format(
                    ball_marker.pose.position.x,
                    ball_marker.pose.position.y,
                    ball_marker.pose.position.z
                )

                text_marker.scale.z = 0.5
                text_marker.color.r = 0.0
                text_marker.color.g = 0.0
                text_marker.color.b = 0.0
                text_marker.color.a = 1.0

                marker_pub.publish(text_marker)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_marker()
    except rospy.ROSInterruptException:
        pass