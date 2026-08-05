#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Synchronously take off UAV0 and UAV1, then hand control to both controllers.

2026-07-27: ROS/MAVROS namespaces were renamed from iris_0/iris_1 to UAV0/UAV1.
"""

import math
import select
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State, StatusText
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Bool


# 2026-07-22: 新增双机统一起飞状态机，避免两个 ego_start.py 分别解锁造成初始化竞态；
# 只有两机都确认 OFFBOARD、armed 并稳定到达悬停高度后，才分别发布 /start_after_hover。


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Vehicle(object):
    def __init__(self, namespace):
        self.ns = namespace.rstrip('/')
        self.state = State()
        self.extended_state = ExtendedState()
        self.pose = None
        self.pose_wall_time = 0.0
        self.pose_count = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.7
        self.target_yaw = 0.0

        self.setpoint_pub = rospy.Publisher(
            self.ns + '/mavros/setpoint_raw/local', PositionTarget, queue_size=20)
        self.start_pub = rospy.Publisher(
            self.ns + '/start_after_hover', Bool, queue_size=1, latch=True)
        self.arm_srv = rospy.ServiceProxy(
            self.ns + '/mavros/cmd/arming', CommandBool)
        self.mode_srv = rospy.ServiceProxy(
            self.ns + '/mavros/set_mode', SetMode)

        rospy.Subscriber(self.ns + '/mavros/state', State, self._state_cb, queue_size=10)
        rospy.Subscriber(
            self.ns + '/mavros/extended_state', ExtendedState,
            self._extended_state_cb, queue_size=10)
        rospy.Subscriber(
            self.ns + '/mavros/local_position/pose', PoseStamped,
            self._pose_cb, queue_size=20)
        rospy.Subscriber(
            self.ns + '/mavros/statustext/recv', StatusText,
            self._status_text_cb, queue_size=20)

    def _state_cb(self, msg):
        self.state = msg

    def _extended_state_cb(self, msg):
        self.extended_state = msg

    def _pose_cb(self, msg):
        self.pose = msg
        self.pose_wall_time = time.monotonic()
        self.pose_count += 1

    def _status_text_cb(self, msg):
        rospy.logwarn('[%s PX4] severity=%d: %s', self.ns, msg.severity, msg.text)

    def pose_age(self):
        if self.pose is None:
            return float('inf')
        return time.monotonic() - self.pose_wall_time

    def ready(self, pose_timeout):
        if not self.state.connected or self.pose is None or self.pose_count < 10:
            return False
        if self.pose_age() > pose_timeout:
            return False
        p = self.pose.pose.position
        return all(math.isfinite(v) for v in (p.x, p.y, p.z))

    def latch_ground_hold_target(self):
        p = self.pose.pose.position
        self.target_x = p.x
        self.target_y = p.y
        # 2026-07-22: 双机都确认解锁之前锁定当前地面高度，防止先解锁的一架提前单独离地。
        self.target_z = p.z
        self.target_yaw = quaternion_to_yaw(self.pose.pose.orientation)
        rospy.loginfo(
            '[%s] pre-takeoff hold latched: x=%.3f y=%.3f z=%.3f yaw=%.1f deg',
            self.ns, self.target_x, self.target_y, self.target_z,
            math.degrees(self.target_yaw))

    def begin_climb(self, hover_z):
        self.target_z = hover_z
        rospy.loginfo('[%s] synchronized climb target enabled: z=%.3f',
                      self.ns, self.target_z)

    def make_setpoint(self):
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
            PositionTarget.IGNORE_YAW_RATE)
        msg.position.x = self.target_x
        msg.position.y = self.target_y
        msg.position.z = self.target_z
        msg.yaw = self.target_yaw
        return msg

    def publish_setpoint(self):
        self.setpoint_pub.publish(self.make_setpoint())

    def altitude(self):
        return self.pose.pose.position.z if self.pose is not None else float('-inf')

    def at_hover(self, tolerance):
        return self.pose_age() <= 0.5 and abs(self.altitude() - self.target_z) <= tolerance

    def request_offboard(self):
        try:
            result = self.mode_srv(base_mode=0, custom_mode='OFFBOARD')
            rospy.loginfo('[%s] OFFBOARD request: mode_sent=%s current_mode=%s',
                          self.ns, result.mode_sent, self.state.mode)
            return bool(result.mode_sent)
        except rospy.ServiceException as exc:
            rospy.logerr('[%s] OFFBOARD service failed: %s', self.ns, exc)
            return False

    def request_arm(self, value=True):
        try:
            result = self.arm_srv(value)
            rospy.loginfo('[%s] arm request=%s: success=%s state.armed=%s',
                          self.ns, value, result.success, self.state.armed)
            return bool(result.success)
        except rospy.ServiceException as exc:
            rospy.logerr('[%s] arming service failed: %s', self.ns, exc)
            return False


class DualTakeoffManager(object):
    def __init__(self):
        # 2026-07-27: 双机起飞、MAVROS 服务和交接信号统一使用 UAV0/UAV1 前缀。
        self.vehicles = [Vehicle('/UAV0'), Vehicle('/UAV1')]
        self.rate_hz = rospy.get_param('~rate', 30.0)
        self.hover_z = rospy.get_param('~hover_z', 0.7)
        self.altitude_tolerance = rospy.get_param('~altitude_tolerance', 0.10)
        self.pose_timeout = rospy.get_param('~pose_timeout', 0.50)
        self.warmup_seconds = rospy.get_param('~warmup_seconds', 4.0)
        self.mode_timeout = rospy.get_param('~mode_timeout', 8.0)
        self.arm_timeout = rospy.get_param('~arm_timeout', 8.0)
        self.takeoff_timeout = rospy.get_param('~takeoff_timeout', 25.0)
        self.hover_stable_seconds = rospy.get_param('~hover_stable_seconds', 1.5)
        self.request_interval = rospy.get_param('~request_interval', 1.0)
        self.rate = rospy.Rate(self.rate_hz)

    def publish_all(self):
        for vehicle in self.vehicles:
            vehicle.publish_setpoint()

    def wait_until_ready(self):
        rospy.loginfo('Waiting for both FCUs and fresh local poses...')
        last_report = 0.0
        while not rospy.is_shutdown():
            if all(v.ready(self.pose_timeout) for v in self.vehicles):
                rospy.loginfo('Both vehicles are connected and local poses are fresh.')
                return True
            now = time.monotonic()
            if now - last_report >= 1.0:
                for v in self.vehicles:
                    rospy.loginfo(
                        '[%s] ready=%s connected=%s pose_count=%d pose_age=%.2fs '
                        'armed=%s mode=%s system_status=%d landed_state=%d',
                        v.ns, v.ready(self.pose_timeout), v.state.connected,
                        v.pose_count, v.pose_age(), v.state.armed, v.state.mode,
                        v.state.system_status, v.extended_state.landed_state)
                last_report = now
            self.rate.sleep()
        return False

    def warmup(self):
        for vehicle in self.vehicles:
            vehicle.latch_ground_hold_target()
        rospy.loginfo('Streaming both ground-hold setpoints for %.1f seconds...', self.warmup_seconds)
        end = time.monotonic() + self.warmup_seconds
        while not rospy.is_shutdown() and time.monotonic() < end:
            self.publish_all()
            self.rate.sleep()

    def wait_for_key(self):
        rospy.loginfo("Press 'o' once to take off BOTH vehicles together...")
        while not rospy.is_shutdown():
            self.publish_all()
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'o':
                    return True
            self.rate.sleep()
        return False

    def establish_offboard(self):
        deadline = time.monotonic() + self.mode_timeout
        last_request = 0.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.publish_all()
            pending = [v for v in self.vehicles if v.state.mode != 'OFFBOARD']
            if not pending:
                rospy.loginfo('Both vehicles confirmed mode=OFFBOARD.')
                return True
            now = time.monotonic()
            if now - last_request >= self.request_interval:
                for vehicle in pending:
                    vehicle.request_offboard()
                last_request = now
            self.rate.sleep()
        rospy.logerr('OFFBOARD confirmation timeout: %s',
                     ', '.join('%s=%s' % (v.ns, v.state.mode) for v in self.vehicles))
        return False

    def establish_armed(self):
        deadline = time.monotonic() + self.arm_timeout
        last_request = 0.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.publish_all()
            if any(v.state.mode != 'OFFBOARD' for v in self.vehicles):
                rospy.logerr('A vehicle left OFFBOARD while arming; abort synchronized takeoff.')
                # 2026-07-22: 若另一架已经解锁，同步流程失败时立即撤销其解锁，避免单机意外离地。
                for vehicle in self.vehicles:
                    if vehicle.state.armed:
                        vehicle.request_arm(False)
                return False
            pending = [v for v in self.vehicles if not v.state.armed]
            if not pending:
                rospy.loginfo('Both vehicles confirmed armed=True.')
                return True
            now = time.monotonic()
            if now - last_request >= self.request_interval:
                for vehicle in pending:
                    vehicle.request_arm(True)
                last_request = now
            self.rate.sleep()

        # 2026-07-22: 同步解锁未完成时撤销已成功车辆的解锁，防止只起飞一架。
        rospy.logerr('Arming confirmation timeout; disarming any partially armed vehicle.')
        for vehicle in self.vehicles:
            if vehicle.state.armed:
                vehicle.request_arm(False)
        return False

    def climb_and_verify(self):
        deadline = time.monotonic() + self.takeoff_timeout
        stable_since = None
        last_recovery = 0.0
        last_report = 0.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.publish_all()
            now = time.monotonic()

            if any(v.pose_age() > self.pose_timeout for v in self.vehicles):
                rospy.logerr_throttle(1.0, 'Takeoff waiting: one or more local poses are stale.')

            # 2026-07-22: 起飞中检测掉 OFFBOARD/自动解锁，继续发 setpoint 并限频恢复，而不是假性卡死。
            if now - last_recovery >= self.request_interval:
                for vehicle in self.vehicles:
                    if vehicle.state.mode != 'OFFBOARD':
                        vehicle.request_offboard()
                    elif not vehicle.state.armed:
                        vehicle.request_arm(True)
                last_recovery = now

            both_stable = all(
                v.state.armed and v.state.mode == 'OFFBOARD' and
                v.at_hover(self.altitude_tolerance)
                for v in self.vehicles)
            if both_stable:
                if stable_since is None:
                    stable_since = now
                    rospy.loginfo('Both vehicles reached hover height; verifying stability...')
                elif now - stable_since >= self.hover_stable_seconds:
                    return True
            else:
                stable_since = None

            if now - last_report >= 1.0:
                for vehicle in self.vehicles:
                    rospy.loginfo(
                        '[%s takeoff] armed=%s mode=%s z=%.3f target=%.3f '
                        'pose_age=%.2fs landed_state=%d',
                        vehicle.ns, vehicle.state.armed, vehicle.state.mode,
                        vehicle.altitude(), vehicle.target_z, vehicle.pose_age(),
                        vehicle.extended_state.landed_state)
                last_report = now
            self.rate.sleep()

        rospy.logerr('Synchronized takeoff timed out after %.1f seconds; keeping setpoints active.',
                     self.takeoff_timeout)
        return False

    def handover(self):
        # 2026-07-22: 两个 latched 启动信号在同一循环发布，确保各自控制器只在双机悬停验证后接管。
        stamp = time.monotonic()
        for vehicle in self.vehicles:
            vehicle.start_pub.publish(Bool(data=True))
        rospy.logwarn(
            'Both /start_after_hover signals published at %.3f; controllers now own MAVROS setpoints.',
            stamp)
        # 2026-07-22: 交接后短时重发 Bool（不再发 MAVROS setpoint），确保 TCPROS 队列在脚本退出前送达。
        end = time.monotonic() + 1.0
        handover_rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < end:
            for vehicle in self.vehicles:
                vehicle.start_pub.publish(Bool(data=True))
            handover_rate.sleep()

    def run(self):
        if not self.wait_until_ready():
            return
        self.warmup()
        while not rospy.is_shutdown():
            if not self.wait_for_key():
                return
            # Refresh XY/yaw immediately before the synchronized attempt.
            for vehicle in self.vehicles:
                if not vehicle.ready(self.pose_timeout):
                    rospy.logerr('[%s] pose/FCU no longer ready; returning to readiness wait.', vehicle.ns)
                    if not self.wait_until_ready():
                        return
                vehicle.latch_ground_hold_target()
            if not self.establish_offboard():
                rospy.logerr("Takeoff not started. Keep both setpoints active and press 'o' to retry.")
                continue
            if not self.establish_armed():
                rospy.logerr("Takeoff not started. Keep both setpoints active and press 'o' to retry.")
                continue
            # 2026-07-22: 只在两架 state.armed 均已确认后同时开放 0.7m 爬升目标。
            for vehicle in self.vehicles:
                vehicle.begin_climb(self.hover_z)
            if not self.climb_and_verify():
                rospy.logerr("Hover not confirmed. Inspect the status above, then press 'o' to retry.")
                continue
            self.handover()
            return


def main():
    rospy.init_node('dual_offboard_takeoff_manager')
    if not sys.stdin.isatty():
        raise SystemExit('dual_ego_start.py requires an interactive terminal for the o key')
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        DualTakeoffManager().run()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    main()
