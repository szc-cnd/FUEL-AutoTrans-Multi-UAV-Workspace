import rospy
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np

print("sys.argv 内容：", sys.argv)


# 从命令行参数获取车辆类型、ID和激光SLAM类型
vehicle_type = sys.argv[1]  # 第一个参数：车辆类型
vehicle_id = sys.argv[2]  # 第二个参数：车辆ID
laser_slam_type = sys.argv[3]  # 第三个参数：激光SLAM类型
truth_odom_switch = sys.argv[4] if len(sys.argv) > 4 else 'off'  # 第四个参数：是否输出真值里程计，默认关闭
truth_odom_mode = truth_odom_switch.lower()
enable_truth_odom = truth_odom_mode in ('on', 'true', '1', 'gt', 'truth', 'aligned', 'raw')
model_name = vehicle_type + '_' + vehicle_id
# 2026-07-27: Gazebo/PX4 model 仍按 iris_0/iris_1 查找，仅将 ROS 感知与 MAVROS 前缀统一为 UAV0/UAV1。
ros_vehicle_name = 'UAV' + vehicle_id


# 2026-07-27: 节点名跟随新的 UAV 前缀，命令行第一个 iris 参数仍只表示 Gazebo 机型。
rospy.init_node(ros_vehicle_name+'_'+laser_slam_type+'_laser_transfer')

# 2026-07-27: 感知桥接的里程计、真值、MAVROS 和 TF 全部切换到 UAV0/UAV1。
vehicle_ns = '/' + ros_vehicle_name
odom_topic = rospy.get_param('~odom_topic', vehicle_ns + '/fast_lio/Odometry')
truth_odom_topic = rospy.get_param('~truth_odom_topic', vehicle_ns + '/ground_truth_odom')
vision_pose_topic = rospy.get_param('~vision_pose_topic', vehicle_ns + '/mavros/vision_pose/pose')
fastlio_world_frame = rospy.get_param('~fastlio_world_frame', ros_vehicle_name + '/camera_init')
fastlio_body_frame = rospy.get_param('~fastlio_body_frame', ros_vehicle_name + '/body')

# 创建发布者，用于发布视觉姿态信息
pose_pub = rospy.Publisher(vision_pose_topic, PoseStamped, queue_size=1)
truth_odom_pub = None
truth_model_index = None
latest_fastlio_odom = None
truth_alignment = None
truth_output_frame = 'map'
truth_child_frame = 'base_link'

if enable_truth_odom:
    truth_odom_pub = rospy.Publisher(truth_odom_topic, Odometry, queue_size=10)

# 设置TF监听器，用于获取坐标变换信息
tfBuffer = Buffer()
tflistener = TransformListener(tfBuffer)

# 初始化局部姿态
local_pose = PoseStamped()
local_pose.header.frame_id = 'map'  # 设置坐标系为'map'

# 初始化Hector SLAM姿态和高度
hector = PoseStamped()
height = 0



def hector_callback(data):
    """
    Hector SLAM回调函数，更新全局hector姿态
    :param data: 接收到的PoseStamped消息
    """
    global hector
    hector = data

def height_distance_callback(msg):
    """
    高度距离回调函数，更新全局高度值
    :param msg: 接收到的LaserScan消息
    """
    global height
    height = msg.ranges[0]  # 获取第一个激光测距值作为高度
    if(height==float("inf")):  # 如果高度值为无穷大，设为0
        height = 0


def gazebo_truth_callback(msg):
    """
    Gazebo真值回调函数
    从 /gazebo/model_states 中提取当前无人机的真值，并直接发布到独立真值话题。
    """
    global truth_model_index

    if truth_odom_pub is None:
        return

    if truth_model_index is None:
        try:
            truth_model_index = msg.name.index(model_name)
            rospy.loginfo('[laser_mid360] Truth odom enabled. Model %s found in /gazebo/model_states.', model_name)
        except ValueError:
            rospy.logwarn_throttle(2.0, '[laser_mid360] Model %s not found in /gazebo/model_states.', model_name)
            return

    if truth_model_index >= len(msg.pose) or truth_model_index >= len(msg.twist):
        rospy.logwarn_throttle(2.0, '[laser_mid360] Truth odom index out of range for %s.', model_name)
        return

    truth_odom = build_truth_odom(msg.pose[truth_model_index], msg.twist[truth_model_index])
    if truth_odom is None:
        return

    truth_odom.pose.covariance = [0.0] * 36
    truth_odom.twist.covariance = [0.0] * 36
    truth_odom_pub.publish(truth_odom)
    # 2026-07-28: 仿真参数 fastlio on 不再只旁路记录真值；同步把对齐真值送入 PX4 外部视觉，
    # 避免 FAST-LIO 在窄通道退化时把 MAVROS local 估计拉到数十米外。实机/off 模式仍使用 FAST-LIO TF。
    truth_pose = PoseStamped()
    truth_pose.header = truth_odom.header
    truth_pose.pose = truth_odom.pose.pose
    pose_pub.publish(truth_pose)


def odom_callback(msg):
    """
    保存当前 FastLIO /Odometry，用于把 Gazebo 真值对齐到当前 odom 坐标系。
    """
    global latest_fastlio_odom
    latest_fastlio_odom = msg


def pose_to_matrix(pose):
    transform = quaternion_matrix([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ])
    transform[0, 3] = pose.position.x
    transform[1, 3] = pose.position.y
    transform[2, 3] = pose.position.z
    return transform


def matrix_to_pose(transform, pose_msg):
    quat = quaternion_from_matrix(transform)
    pose_msg.position.x = float(transform[0, 3])
    pose_msg.position.y = float(transform[1, 3])
    pose_msg.position.z = float(transform[2, 3])
    pose_msg.orientation.x = float(quat[0])
    pose_msg.orientation.y = float(quat[1])
    pose_msg.orientation.z = float(quat[2])
    pose_msg.orientation.w = float(quat[3])


def vector_to_numpy(vec):
    return np.array([vec.x, vec.y, vec.z], dtype=float)


def ensure_truth_alignment(truth_pose):
    """
    在 aligned/on 模式下，用首次同步到的 /Odometry 和 Gazebo 真值建立固定坐标替换。
    """
    global truth_alignment, truth_output_frame, truth_child_frame

    if truth_odom_mode == 'raw':
        truth_output_frame = 'map'
        truth_child_frame = 'base_link'
        return True

    if truth_alignment is not None:
        return True

    if latest_fastlio_odom is None:
        rospy.logwarn_throttle(2.0, '[laser_mid360] Waiting for /Odometry before aligning truth odom.')
        return False

    odom_transform = pose_to_matrix(latest_fastlio_odom.pose.pose)
    truth_transform = pose_to_matrix(truth_pose)
    truth_alignment = np.dot(odom_transform, np.linalg.inv(truth_transform))
    truth_output_frame = latest_fastlio_odom.header.frame_id or 'camera_init'
    truth_child_frame = latest_fastlio_odom.child_frame_id or 'body'
    rospy.loginfo(
        '[laser_mid360] Truth odom aligned to /Odometry. frame=%s child=%s mode=%s',
        truth_output_frame, truth_child_frame, truth_odom_mode)
    return True


def build_truth_odom(truth_pose, truth_twist):
    truth_odom = Odometry()
    truth_odom.header.stamp = rospy.Time.now()

    if not ensure_truth_alignment(truth_pose):
        return None

    if truth_odom_mode == 'raw':
        truth_odom.header.frame_id = 'map'
        truth_odom.child_frame_id = 'base_link'
        truth_odom.pose.pose = truth_pose
        truth_odom.twist.twist = truth_twist
        return truth_odom

    aligned_transform = np.dot(truth_alignment, pose_to_matrix(truth_pose))
    truth_odom.header.frame_id = truth_output_frame
    truth_odom.child_frame_id = truth_child_frame
    matrix_to_pose(aligned_transform, truth_odom.pose.pose)

    rotation = truth_alignment[:3, :3]
    linear = rotation.dot(vector_to_numpy(truth_twist.linear))
    angular = rotation.dot(vector_to_numpy(truth_twist.angular))
    truth_odom.twist.twist.linear.x = float(linear[0])
    truth_odom.twist.twist.linear.y = float(linear[1])
    truth_odom.twist.twist.linear.z = float(linear[2])
    truth_odom.twist.twist.angular.x = float(angular[0])
    truth_odom.twist.twist.angular.y = float(angular[1])
    truth_odom.twist.twist.angular.z = float(angular[2])
    return truth_odom
    
def hector_slam():
    """
    Hector SLAM主函数
    订阅姿态信息，并以100Hz的频率发布更新后的姿态
    """
    global local_pose, height
    pose2d_sub = rospy.Subscriber(vehicle_type+'_'+ vehicle_id+"/pose", PoseStamped, hector_callback,queue_size=1)
    rate = rospy.Rate(100)  # 设置循环频率为100Hz
    while True:
        local_pose = hector  # 更新局部姿态
        local_pose.pose.position.z = height  # 更新高度信息
        pose_pub.publish(local_pose)  # 发布更新后的姿态
        rate.sleep()  # 等待下一个循环
        
def cartographer():
    """
    Cartographer SLAM主函数
    以30Hz的频率获取并发布更新后的姿态
    """
    global local_pose, height
    rate = rospy.Rate(30)  # 设置循环频率为30Hz
    while not rospy.is_shutdown():
        try:
            # 尝试获取从'map'到'base_link'的坐标变换
            tfstamped = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
        except:
            continue  # 如果获取失败，继续下一次循环
        local_pose.header.stamp = rospy.Time().now()  # 更新时间戳
        local_pose.pose.position = tfstamped.transform.translation  # 更新位置
        local_pose.pose.position.z = height  # 更新高度
        local_pose.pose.orientation = tfstamped.transform.rotation  # 更新朝向
        pose_pub.publish(local_pose)  # 发布更新后的姿态
        rate.sleep()  # 等待下一个循环
        
def aloam():
    """
    A-LOAM SLAM主函数
    以30Hz的频率获取并发布更新后的姿态
    """
    global local_pose
    rate = rospy.Rate(30)  # 设置循环频率为30Hz
    while not rospy.is_shutdown():
        try:
            # 尝试获取从'camera_init'到'aft_mapped'的坐标变换
            tfstamped = tfBuffer.lookup_transform('camera_init', 'aft_mapped', rospy.Time(0))
        except:
            continue  # 如果获取失败，继续下一次循环
        local_pose.header.stamp = rospy.Time().now()  # 更新时间戳
        local_pose.pose.position = tfstamped.transform.translation  # 更新位置
        local_pose.pose.orientation = tfstamped.transform.rotation  # 更新朝向
        pose_pub.publish(local_pose)  # 发布更新后的姿态
        rate.sleep()  # 等待下一个循环

def fastlio():
    """
    fastlio SLAM主函数
    以50Hz的频率获取并发布更新后的姿态
    """
    global local_pose
    rate = rospy.Rate(50)  # 设置循环频率为50Hz
    while not rospy.is_shutdown():
        try:
            # 尝试获取从'camera_init'到'aft_mapped'的坐标变换，即fastlio里程计中对应的坐标系变换
            tfstamped = tfBuffer.lookup_transform(fastlio_world_frame, fastlio_body_frame, rospy.Time(0))
        except:
            continue  # 如果获取失败，继续下一次循环
        local_pose.header.stamp = rospy.Time().now()  # 更新时间戳
        local_pose.pose.position = tfstamped.transform.translation  # 更新位置
        local_pose.pose.orientation = tfstamped.transform.rotation  # 更新朝向
        # 2026-07-28: on/aligned 仿真模式的 MAVROS 外部视觉由 gazebo_truth_callback 独占发布，
        # 禁止与 FAST-LIO TF 两路 PoseStamped 交替灌入 PX4；off 模式保持原有真实雷达链路。
        if not enable_truth_odom:
            pose_pub.publish(local_pose)  # 发布更新后的姿态
        rate.sleep()  # 等待下一个循环
    
if __name__ == '__main__':
    if enable_truth_odom:
        rospy.Subscriber(odom_topic, Odometry, odom_callback, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, gazebo_truth_callback, queue_size=1)
        rospy.loginfo(
            '[laser_mid360] Truth odom output is ON. mode=%s, odom=%s truth=%s frames=%s->%s.',
            truth_odom_mode, odom_topic, truth_odom_topic, fastlio_world_frame, fastlio_body_frame)
    else:
        rospy.loginfo('[laser_mid360] Truth odom output is OFF.')

    # 根据指定的SLAM类型执行相应的函数
    if laser_slam_type == 'hector':
        # 订阅距离信息，用于更新高度
        height_distance_sub = rospy.Subscriber(vehicle_type+'_'+ vehicle_id+"/distance", LaserScan, height_distance_callback,queue_size=1)
        rospy.loginfo(f"{laser_slam_type} :: start translating ! ! ! ")
        hector_slam()  # 执行Hector SLAM
        
    elif laser_slam_type == 'cartographer':
        # 订阅距离信息，用于更新高度
        height_distance_sub = rospy.Subscriber(vehicle_type+'_'+ vehicle_id+"/distance", LaserScan, height_distance_callback,queue_size=1)
        rospy.loginfo(f"{laser_slam_type} :: start translating ! ! ! ")
        cartographer()  # 执行Cartographer SLAM

    elif laser_slam_type == 'aloam':
        rospy.loginfo(f"{laser_slam_type} :: start translating ! ! ! ")
        aloam()  # 执行A-LOAM SLAM

    elif laser_slam_type == 'fastlio':
        rospy.loginfo(f"{laser_slam_type} :: start translating ! ! ! ")
        fastlio()  # 执行fastlio SLAM

    else:
        print('input error')  # 如果输入的SLAM类型不正确，打印错误信息
        
