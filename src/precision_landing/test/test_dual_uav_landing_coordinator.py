#!/usr/bin/env python3

import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32, String

from precision_landing.msg import LandingPlatform, LandingPlatformArray


class DualUavLandingCoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.ready = False
        self.uav0_id = -1
        self.uav1_id = -1
        self.uav0_target = None
        self.uav1_target = None

        rospy.Subscriber("/test/assignments_ready", Bool, self._ready_callback)
        rospy.Subscriber("/test/uav0_assigned", Int32, self._uav0_id_callback)
        rospy.Subscriber("/test/uav1_assigned", Int32, self._uav1_id_callback)
        rospy.Subscriber("/test/uav0_target", PoseStamped, self._uav0_target_callback)
        rospy.Subscriber("/test/uav1_target", PoseStamped, self._uav1_target_callback)

        self.front_candidates_pub = rospy.Publisher(
            "/test/front_candidates", LandingPlatformArray, queue_size=1
        )
        self.exit_pub = rospy.Publisher(
            "/test/final_exit", PoseStamped, queue_size=1
        )
        self.state_pub = rospy.Publisher(
            "/test/search_state", String, queue_size=1
        )

        self.assertTrue(
            self._wait_for(
                lambda: self.front_candidates_pub.get_num_connections() > 0
                and self.exit_pub.get_num_connections() > 0
                and self.state_pub.get_num_connections() > 0
            ),
            "coordinator did not subscribe to test inputs",
        )

    def _ready_callback(self, message):
        self.ready = message.data

    def _uav0_id_callback(self, message):
        self.uav0_id = message.data

    def _uav1_id_callback(self, message):
        self.uav1_id = message.data

    def _uav0_target_callback(self, message):
        self.uav0_target = message

    def _uav1_target_callback(self, message):
        self.uav1_target = message

    @staticmethod
    def _wait_for(predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return True
            rospy.sleep(0.02)
        return False

    @staticmethod
    def _candidate_array(stamp):
        message = LandingPlatformArray()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        for marker_id, x, y in ((31, 1.0, 2.0), (17, 3.0, 4.0)):
            platform = LandingPlatform()
            platform.header = message.header
            platform.id = marker_id
            platform.pose.header = message.header
            platform.pose.pose.position.x = x
            platform.pose.pose.position.y = y
            platform.pose.pose.orientation.w = 1.0
            platform.score = 1.0
            message.platforms.append(platform)
        return message

    def test_assignment_waits_for_full_scan_and_preserves_detection_order(self):
        self.state_pub.publish(String(data="FRONT_ARUCO_FORWARD_APPROACH"))
        exit_pose = PoseStamped()
        exit_pose.header.stamp = rospy.Time.now()
        exit_pose.header.frame_id = "world"
        exit_pose.pose.orientation.w = 1.0
        self.exit_pub.publish(exit_pose)
        rospy.sleep(0.1)

        self.state_pub.publish(String(data="FRONT_ARUCO_INITIAL_WAIT"))
        self.front_candidates_pub.publish(
            self._candidate_array(rospy.Time.now() - rospy.Duration(2.0))
        )
        rospy.sleep(0.15)
        self.assertFalse(self.ready, "stale latched candidates must not assign")

        self.front_candidates_pub.publish(self._candidate_array(rospy.Time.now()))
        rospy.sleep(0.15)
        self.assertFalse(self.ready, "front candidates must not interrupt yaw scan")
        self.assertEqual(self.uav0_id, -1)
        self.assertEqual(self.uav1_id, -1)

        self.state_pub.publish(String(data="FRONT_ARUCO_YAW_SCAN_RIGHT"))
        rospy.sleep(0.1)
        self.assertFalse(self.ready, "right scan is not full scan completion")

        self.state_pub.publish(
            String(data="FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP")
        )
        self.assertTrue(self._wait_for(lambda: self.ready), "assignment was not released")
        self.assertEqual(self.uav1_id, 31)
        self.assertEqual(self.uav0_id, 17)
        self.assertIsNotNone(self.uav1_target)
        self.assertIsNotNone(self.uav0_target)
        self.assertAlmostEqual(self.uav1_target.pose.position.x, 1.0)
        self.assertAlmostEqual(self.uav0_target.pose.position.x, 3.0)


if __name__ == "__main__":
    rospy.init_node("dual_uav_landing_coordinator_test")
    rostest.rosrun(
        "precision_landing",
        "dual_uav_landing_coordinator_test",
        DualUavLandingCoordinatorTest,
    )
