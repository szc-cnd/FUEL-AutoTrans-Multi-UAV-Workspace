#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, time, threading
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header, Bool, Empty
from geometry_msgs.msg import PoseStamped

import sys, select, termios, tty

# 2026-07-14: 第三个命令行参数支持 hold_only/follow；follow 在悬停后把后机控制权交给安全路径跟随器。
if len(sys.argv) < 3:
    raise SystemExit("Usage: ego_start.py <vehicle_type> <vehicle_id> [mission|follow|hold_only]")

run_mode = sys.argv[3].lower() if len(sys.argv) > 3 else "mission"
if run_mode not in ("mission", "follow", "hold", "hold_only"):
    raise SystemExit("Unknown mode '{}'; use mission, follow or hold_only".format(run_mode))


# ========= 加入 raw 键盘模式 =========
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)          # 进入 raw 模式（不需要回车即可读取按键）
# ===================================


rospy.init_node(sys.argv[1] + '_' + sys.argv[2] + "_offboard_manager")
rate = rospy.Rate(30)


class Communication:

    def __init__(self, vehicle_type, vehicle_id, mode):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        # 2026-07-14: hold_only 不解锁规划器，也不把 setpoint 控制权交给尚未实现的后机跟随器。
        self.hold_only = mode in ("hold", "hold_only")
        # 2026-07-14: follow 与 mission 使用同一安全交接握手，但日志明确区分后机不启动 FUEL。
        self.follow_mode = mode == "follow"
        # 2026-07-13: 起飞交接信号按车辆隔离，防止 iris_1 的悬停完成或轨迹状态误触发 iris_0。
        self.vehicle_ns = "/" + vehicle_type + '_' + vehicle_id
        self.traj_started_topic = rospy.get_param(
            '~traj_started_topic', self.vehicle_ns + "/planning/traj_started")
        self.start_after_hover_topic = rospy.get_param(
            '~start_after_hover_topic', self.vehicle_ns + "/start_after_hover")

        self.current_state = State()
        self.current_z = 0.0

        self.hover_z = rospy.get_param('~hover_z', 0.7)
        self.hover_stable_time = rospy.get_param('~hover_stable_time', 1.0)
        self.alt_tol = rospy.get_param('~alt_tol', 0.1)

        # 状态标志
        self.offboard_started = False
        self.airborne_detected = False
        self.start_sent = False
        self.offboard_start_time = None
        self.traj_started_received = False
        self.last_start_pub_time = None
        self.start_republish_interval = 1.0

        # 服务
        self.arm_srv = rospy.ServiceProxy(self.ns("/mavros/cmd/arming"), CommandBool)
        self.mode_srv = rospy.ServiceProxy(self.ns("/mavros/set_mode"), SetMode)

        # 订阅
        rospy.Subscriber(self.ns("/mavros/state"), State, self.state_cb)
        rospy.Subscriber(self.ns("/mavros/local_position/pose"), PoseStamped, self.pose_cb)
        rospy.Subscriber(self.traj_started_topic, Empty, self.traj_started_cb)

        # 发布
        self.setpoint_pub = rospy.Publisher(self.ns("/mavros/setpoint_raw/local"), PositionTarget, queue_size=20)
        self.start_pub = rospy.Publisher(self.start_after_hover_topic, Bool, queue_size=1, latch=True)

        mode_name = "hold_only" if self.hold_only else ("follow" if self.follow_mode else "mission")
        rospy.loginfo("%s offboard manager initialized, mode=%s, hover_z=%.2f",
                      self.ns(""), mode_name, self.hover_z)

    def ns(self, suffix):
        return self.vehicle_type + '_' + self.vehicle_id + suffix

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_z = msg.pose.position.z

    def traj_started_cb(self, _msg):
        if not self.traj_started_received:
            rospy.loginfo("Received %s. Stop republishing %s.",
                          self.traj_started_topic, self.start_after_hover_topic)
        self.traj_started_received = True

    def publish_start_signal(self, first_publish):
        self.start_pub.publish(Bool(data=True))
        self.last_start_pub_time = time.time()
        if first_publish:
            if self.follow_mode:
                # 2026-07-14: 后机只解锁 leader_safe_path_follower，不宣称启动规划器。
                rospy.loginfo("Hover done. Safe-path follower unlocked.")
                rospy.logwarn("Control handed over to follower controller. Offboard manager STOP publishing.")
            else:
                rospy.loginfo("Hover done. Planning unlocked.")
                rospy.logwarn("Control handed over to controller. Offboard manager STOP publishing.")
        else:
            rospy.logwarn("No %s yet. Republish %s=true.",
                          self.traj_started_topic, self.start_after_hover_topic)

    def arm(self):
        rospy.wait_for_service(self.ns("/mavros/cmd/arming"))
        ok = self.arm_srv(True)
        if ok and getattr(ok, "success", False):
            rospy.loginfo("Armed.")
            return True
        rospy.logerr("Arming failed.")
        return False

    def set_offboard_mode(self):
        rospy.wait_for_service(self.ns("/mavros/set_mode"))
        ok = self.mode_srv(custom_mode='OFFBOARD')
        if ok and getattr(ok, "mode_sent", False):
            rospy.loginfo("OFFBOARD enabled")
            return True
        rospy.logerr("Set OFFBOARD failed")
        return False

    def make_pose_sp(self):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.FORCE |
            PositionTarget.IGNORE_YAW_RATE
        )
        msg.position.x = 0
        msg.position.y = 0
        msg.position.z = self.hover_z
        msg.yaw = 0
        return msg

    def warmup_setpoints(self, seconds=4.0):
        rospy.loginfo("Warmup setpoints...")
        end_t = rospy.Time.now() + rospy.Duration.from_sec(seconds)
        while rospy.Time.now() < end_t and not rospy.is_shutdown():
            self.setpoint_pub.publish(self.make_pose_sp())
            rate.sleep()

    def initialize_and_run(self):
        rospy.loginfo("Waiting FCU...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        rospy.loginfo("FCU connected.")

        self.warmup_setpoints()

        rospy.loginfo("Press 'o' to arm + offboard + takeoff...")

        # ========= 用非阻塞键盘读取 =========
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0)[0]:   # 有按键可读
                key = sys.stdin.read(1)
                if key.lower() == 'o':
                    if self.arm() and self.set_offboard_mode():
                        self.offboard_started = True
                        rospy.loginfo("Takeoff started.")
                        break
            rate.sleep()
        # ==================================

        while not rospy.is_shutdown():

            if not self.offboard_started:
                rate.sleep()
                continue

            # 起飞阶段：只发悬停 setpoint
            if not self.airborne_detected:
                self.setpoint_pub.publish(self.make_pose_sp())
                if self.current_z >= self.hover_z - self.alt_tol:
                    self.airborne_detected = True
                    self.offboard_start_time = time.time()
                    rospy.loginfo("Reached hover height. Holding...")
                rate.sleep()
                continue

            # 悬停计时
            if not self.start_sent:
                if time.time() - self.offboard_start_time < self.hover_stable_time:
                    self.setpoint_pub.publish(self.make_pose_sp())
                    rate.sleep()
                    continue

                # 2026-07-14: 双机悬停验证期间持续发布位置 setpoint，不触发 FUEL，也不产生控制空窗。
                if self.hold_only:
                    rospy.logwarn("Hover stable. HOLD_ONLY active; keep publishing %s setpoints.",
                                  self.ns("/mavros/setpoint_raw/local"))
                    self.start_sent = True
                    continue

                # 悬停完毕 → 给规划器发启动信号
                self.publish_start_signal(first_publish=True)
                self.start_sent = True
                continue

            if self.hold_only:
                self.setpoint_pub.publish(self.make_pose_sp())
                rate.sleep()
                continue

            if not self.traj_started_received and self.last_start_pub_time is not None:
                if time.time() - self.last_start_pub_time >= self.start_republish_interval:
                    self.publish_start_signal(first_publish=False)

            # 进入控制器阶段：彻底不 publish 任何 setpoint
            rate.sleep()

        return True


if __name__ == "__main__":
    try:
        comm = Communication(sys.argv[1], sys.argv[2], run_mode)
        comm.initialize_and_run()
    finally:
        # ========= 退出时恢复终端模式 =========
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # ====================================
