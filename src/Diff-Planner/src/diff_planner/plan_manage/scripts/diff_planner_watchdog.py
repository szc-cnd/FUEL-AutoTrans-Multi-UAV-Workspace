#!/usr/bin/env python3
"""Hard-restart a local Diff planner whose single ROS callback thread is stuck."""

import os
import re
import signal
import threading
import time
import xmlrpc.client

import rosgraph
import rosnode
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty, String


_STAMP_PATTERN = re.compile(r"(?:^|\s)goal_stamp_ns=(\d+)(?:\s|$)")
_TERMINAL_STATUSES = {
    "TRAJECTORY_PUBLISHED",
    "PLANNING_FAILED",
    "GOAL_REJECTED_OUTSIDE_MAP",
}


class DiffPlannerWatchdog:
    def __init__(self):
        self._planner_node = rospy.get_param(
            "~planner_node", "/drone_1_diff_planner_node"
        )
        self._expected_executable = rospy.get_param(
            "~expected_executable", "diff_planner_node"
        )
        self._heartbeat_timeout = float(rospy.get_param("~heartbeat_timeout", 1.2))
        self._minimum_goal_age = float(rospy.get_param("~minimum_goal_age", 0.4))
        self._lock = threading.Lock()
        self._pending_goal_stamp_ns = 0
        self._pending_since = 0.0
        self._last_heartbeat = time.monotonic()
        self._restart_in_progress = False

        goal_topic = rospy.get_param("~goal_topic", "/UAV1/planning/goal")
        heartbeat_topic = rospy.get_param(
            "~heartbeat_topic", "/drone_1_planning/heartbeat"
        )
        status_topic = rospy.get_param(
            "~status_topic", "/drone_1_planning/status"
        )
        safety_hold_topic = rospy.get_param(
            "~safety_hold_topic", "/UAV1/planning/safety_hold"
        )
        self._status_pub = rospy.Publisher(status_topic, String, queue_size=10, latch=True)
        # 不锁存：真正的锁存状态由safe_follower统一维护，避免多个锁存发布者在
        # 控制器重连时以不确定顺序重放true/false。
        self._safety_hold_pub = rospy.Publisher(
            safety_hold_topic, Bool, queue_size=1, latch=False
        )
        self._goal_sub = rospy.Subscriber(
            goal_topic, PoseStamped, self._goal_callback, queue_size=10
        )
        self._heartbeat_sub = rospy.Subscriber(
            heartbeat_topic, Empty, self._heartbeat_callback, queue_size=20
        )
        self._status_sub = rospy.Subscriber(
            status_topic, String, self._status_callback, queue_size=20
        )
        self._timer = rospy.Timer(rospy.Duration(0.1), self._timer_callback)

    def _goal_callback(self, msg):
        stamp_ns = msg.header.stamp.to_nsec()
        if stamp_ns <= 0:
            return
        now = time.monotonic()
        with self._lock:
            self._pending_goal_stamp_ns = stamp_ns
            self._pending_since = now
            self._restart_in_progress = False

    def _heartbeat_callback(self, _msg):
        with self._lock:
            self._last_heartbeat = time.monotonic()

    def _status_callback(self, msg):
        fields = msg.data.split(None, 1)
        if not fields or fields[0] not in _TERMINAL_STATUSES:
            return
        match = _STAMP_PATTERN.search(msg.data)
        if match is None:
            return
        response_stamp_ns = int(match.group(1))
        with self._lock:
            if response_stamp_ns == self._pending_goal_stamp_ns:
                self._pending_goal_stamp_ns = 0
                self._pending_since = 0.0
                self._restart_in_progress = False

    def _local_planner_pid(self):
        master = rosgraph.Master(rospy.get_name())
        uri = rosnode.get_api_uri(master, self._planner_node, skip_cache=True)
        if not uri:
            raise RuntimeError("planner node is not registered with ROS master")
        code, message, pid = xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
        if code != 1 or not isinstance(pid, int) or pid <= 1:
            raise RuntimeError("planner getPid failed: {}".format(message))

        cmdline_path = "/proc/{}/cmdline".format(pid)
        with open(cmdline_path, "rb") as stream:
            cmdline = stream.read().replace(b"\x00", b" ").decode(
                "utf-8", errors="replace"
            )
        if self._expected_executable not in cmdline:
            raise RuntimeError(
                "refuse to kill pid {} with unexpected command: {}".format(pid, cmdline)
            )
        return pid

    def _timer_callback(self, _event):
        now = time.monotonic()
        with self._lock:
            goal_stamp_ns = self._pending_goal_stamp_ns
            goal_age = now - self._pending_since if self._pending_since else 0.0
            heartbeat_age = now - self._last_heartbeat
            if (
                goal_stamp_ns == 0
                or self._restart_in_progress
                or goal_age < self._minimum_goal_age
                or heartbeat_age < self._heartbeat_timeout
            ):
                return
            self._restart_in_progress = True

        try:
            pid = self._local_planner_pid()
            rospy.logerr(
                "[Diff看门狗] 目标%u等待%.2fs且规划心跳中断%.2fs；"
                "终止假死进程pid=%d，由roslaunch自动拉起。",
                goal_stamp_ns,
                goal_age,
                heartbeat_age,
                pid,
            )
            # planner与traj_server是两个进程。先让AutoTrans丢弃traj_server残留的
            # 旧轨迹并锁当前位置，再终止规划进程；上层收到失败回执后会继续锁点。
            self._safety_hold_pub.publish(Bool(data=True))
            os.kill(pid, signal.SIGKILL)
            self._status_pub.publish(
                String(data="PLANNING_FAILED goal_stamp_ns={}".format(goal_stamp_ns))
            )
            with self._lock:
                if self._pending_goal_stamp_ns == goal_stamp_ns:
                    self._pending_goal_stamp_ns = 0
                    self._pending_since = 0.0
        except (OSError, RuntimeError, rosnode.ROSNodeIOException, xmlrpc.client.Error) as exc:
            rospy.logerr_throttle(1.0, "[Diff看门狗] 无法重启假死规划器：%s", exc)
            with self._lock:
                self._restart_in_progress = False


def main():
    rospy.init_node("uav1_diff_planner_watchdog")
    DiffPlannerWatchdog()
    rospy.spin()


if __name__ == "__main__":
    main()
