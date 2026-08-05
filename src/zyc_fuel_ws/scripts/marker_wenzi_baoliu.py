import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import numpy as np

class BallPoseTracker:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('ball_pose_tracker', anonymous=True)

        # 创建发布器，用于发布可视化标记
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # 订阅小球坐标的话题
        rospy.Subscriber('/ball_pose_tracking', PoseStamped, self.pose_callback)

        # 用于存储接收到的小球坐标
        self.positions = []
        self.history_positions = []  # 存储历史位置
        self.next_id = 0  # 用于给每个小球分配唯一ID
        self.latest_pose_time = rospy.Time.now()

        # 发布频率
        self.rate = rospy.Rate(50)

    def pose_callback(self, msg):
        # 接收到坐标后存储 x, y, z 坐标
        self.positions.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.latest_pose_time = rospy.Time.now()

    def process_positions(self):
        if len(self.positions) < 3:
            return None
        
        positions_np = np.array(self.positions)
        mean = np.mean(positions_np, axis=0)
        std_dev = np.std(positions_np, axis=0)

        filtered_positions = [pos for pos in positions_np if np.all(np.abs(pos - mean) <= 2 * std_dev)]

        if len(filtered_positions) == 0:
            return None

        filtered_positions_np = np.array(filtered_positions)
        average_position = np.mean(filtered_positions_np, axis=0)

        # 将新位置添加到历史记录
        self.history_positions.append({
            'position': average_position,
            'id': self.next_id
        })
        self.next_id += 1

        return average_position

    def publish_markers(self):
        # 遍历并显示所有历史位置
        for pos_data in self.history_positions:
            # 发布小球标记
            ball_marker = Marker()
            ball_marker.header.frame_id = "world"
            ball_marker.header.stamp = rospy.Time.now()
            ball_marker.ns = "demo"
            ball_marker.id = pos_data['id'] * 2  # 每个小球占用两个ID(球体和文本)
            ball_marker.type = Marker.SPHERE
            ball_marker.action = Marker.ADD

            pos = pos_data['position']
            ball_marker.pose.position.x = pos[0]
            ball_marker.pose.position.y = pos[1]
            ball_marker.pose.position.z = pos[2]

            ball_marker.scale.x = 1.0
            ball_marker.scale.y = 1.0
            ball_marker.scale.z = 1.0

            ball_marker.color.r = 1.0
            ball_marker.color.g = 0.0
            ball_marker.color.b = 0.0
            ball_marker.color.a = 1.0

            self.marker_pub.publish(ball_marker)

            # 发布文本标记
            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "demo"
            text_marker.id = pos_data['id'] * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = pos[0]
            text_marker.pose.position.y = pos[1]
            text_marker.pose.position.z = pos[2] + 1.5

            text_marker.text = "Ball {}\nx: {:.2f}m, y: {:.2f}m, z: {:.2f}m".format(
                pos_data['id'], pos[0], pos[1], pos[2])
            
            text_marker.scale.z = 1.0
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0

            self.marker_pub.publish(text_marker)

    def run(self):
        while not rospy.is_shutdown():
            # 如果超过1秒没有接收到新的坐标，则处理已有坐标
            if (rospy.Time.now() - self.latest_pose_time).to_sec() > 1.0 and len(self.positions) > 0:
                average_position = self.process_positions()
                if average_position is not None:
                    self.publish_markers()  # 更新显示所有历史位置
                # 清空当前处理的坐标数据
                self.positions = []

            self.rate.sleep()

if __name__ == '__main__':
    try:
        tracker = BallPoseTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass