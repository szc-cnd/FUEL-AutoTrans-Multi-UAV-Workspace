#!/usr/bin/env python3

import math
import threading
import time
import unittest

import cv2
import numpy as np
import rospy
import rostest
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, SetModeResponse
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int32, String


def generate_aruco_marker(dictionary, marker_id, marker_size_px):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(
            dictionary, marker_id, marker_size_px
        )

    marker = np.zeros((marker_size_px, marker_size_px), dtype=np.uint8)
    cv2.aruco.drawMarker(
        dictionary, marker_id, marker_size_px, marker, 1
    )
    return marker


class PrecisionLandingNodeTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.setpoints = []
        self.states = []
        self.locked_ids = []
        self.mode_requests = []
        self.mode_request_start_times = []
        self.mode_requests_completed = 0
        self.mode_delay_sec = 0.0
        self.bridge = CvBridge()

        self.setpoint_sub = rospy.Subscriber(
            "/test/setpoint", PositionTarget, self._setpoint_cb
        )
        self.state_sub = rospy.Subscriber(
            "/test/landing_state", String, self._state_cb
        )
        self.locked_id_sub = rospy.Subscriber(
            "/test/locked_id", Int32, self._locked_id_cb
        )
        self.mode_service = rospy.Service(
            "/test/set_mode", SetMode, self._mode_cb
        )
        self.addCleanup(self.mode_service.shutdown, "test complete")

        self.image_pub = rospy.Publisher("/test/image", Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher(
            "/test/camera_info", CameraInfo, queue_size=1
        )
        self.pose_pub = rospy.Publisher(
            "/test/pose", PoseStamped, queue_size=1
        )
        self.mavros_state_pub = rospy.Publisher(
            "/test/mavros_state", State, queue_size=1
        )
        self.trigger_pub = rospy.Publisher(
            "/test/trigger", Bool, queue_size=1
        )
        self.target_id_pub = rospy.Publisher(
            "/test/target_id", Int32, queue_size=1
        )

        publishers = (
            self.image_pub,
            self.camera_info_pub,
            self.pose_pub,
            self.mavros_state_pub,
            self.trigger_pub,
            self.target_id_pub,
        )
        self.assertTrue(
            self.wait_for(
                lambda: all(pub.get_num_connections() > 0 for pub in publishers)
            ),
            "precision_landing_node did not subscribe to every test input",
        )

    def tearDown(self):
        self.publish_vehicle_context(armed=True, mode="POSCTL")
        self.trigger_pub.publish(Bool(data=False))
        rospy.sleep(0.2)
        self.setpoint_sub.unregister()
        self.state_sub.unregister()
        self.locked_id_sub.unregister()

    def _setpoint_cb(self, msg):
        with self.lock:
            self.setpoints.append(msg)

    def _state_cb(self, msg):
        with self.lock:
            self.states.append(msg.data)

    def _locked_id_cb(self, msg):
        with self.lock:
            self.locked_ids.append(msg.data)

    def _mode_cb(self, request):
        with self.lock:
            self.mode_requests.append(request.custom_mode)
            self.mode_request_start_times.append(time.monotonic())
        if self.mode_delay_sec > 0.0:
            rospy.sleep(self.mode_delay_sec)
        with self.lock:
            self.mode_requests_completed += 1
        return SetModeResponse(mode_sent=True)

    def _setpoint_count(self):
        with self.lock:
            return len(self.setpoints)

    def _setpoint_snapshot(self):
        with self.lock:
            return list(self.setpoints)

    def _state_snapshot(self):
        with self.lock:
            return list(self.states)

    def _locked_id_snapshot(self):
        with self.lock:
            return list(self.locked_ids)

    def _mode_request_count(self):
        with self.lock:
            return len(self.mode_requests)

    def _mode_request_complete_count(self):
        with self.lock:
            return self.mode_requests_completed

    def wait_for(self, predicate, timeout=2.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return True
            rate.sleep()
        return False

    def publish_vehicle_context(
        self, armed, mode, yaw_rad=0.0, header_offset_sec=0.0
    ):
        deadline = rospy.Time.now() + rospy.Duration(0.2)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            now = rospy.Time.now()

            state = State()
            state.header.stamp = now + rospy.Duration(header_offset_sec)
            state.connected = True
            state.armed = armed
            state.mode = mode
            self.mavros_state_pub.publish(state)

            pose = PoseStamped()
            pose.header.stamp = now + rospy.Duration(header_offset_sec)
            pose.header.frame_id = "map"
            pose.pose.position.z = 2.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw_rad * 0.5)
            pose.pose.orientation.w = math.cos(yaw_rad * 0.5)
            self.pose_pub.publish(pose)
            rate.sleep()

    def publish_mavros_state(self, armed, mode):
        deadline = rospy.Time.now() + rospy.Duration(0.2)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            state = State()
            state.header.stamp = rospy.Time.now()
            state.connected = True
            state.armed = armed
            state.mode = mode
            self.mavros_state_pub.publish(state)
            rate.sleep()

    def publish_camera_info(self, valid, header_offset_sec=0.0):
        info = CameraInfo()
        info.header.stamp = (
            rospy.Time.now() + rospy.Duration(header_offset_sec)
        )
        info.header.frame_id = "camera"
        info.width = 640
        info.height = 480
        info.K = (
            [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
            if valid
            else [0.0] * 9
        )
        info.D = [0.0] * 5
        self.camera_info_pub.publish(info)

    def publish_centered_marker(
        self,
        marker_id,
        frames,
        center_x=320,
        center_y=240,
        header_offset_sec=0.0,
        marker_size_px=200,
    ):
        marker = generate_aruco_marker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250),
            marker_id,
            marker_size_px,
        )
        canvas = np.full((480, 640), 255, dtype=np.uint8)
        half_size = marker_size_px // 2
        canvas[
            center_y - half_size:center_y + half_size,
            center_x - half_size:center_x + half_size,
        ] = marker

        rate = rospy.Rate(20)
        for _ in range(frames):
            image = self.bridge.cv2_to_imgmsg(canvas, encoding="mono8")
            image.header.stamp = (
                rospy.Time.now() + rospy.Duration(header_offset_sec)
            )
            image.header.frame_id = "camera"
            self.image_pub.publish(image)
            rate.sleep()

    def test_no_setpoint_before_trigger(self):
        start_count = self._setpoint_count()
        rospy.sleep(0.5)
        self.assertEqual(start_count, self._setpoint_count())

    def test_invalid_camera_info_never_descends(self):
        self.publish_vehicle_context(armed=True, mode="OFFBOARD")
        self.publish_camera_info(valid=False)
        self.publish_centered_marker(marker_id=37, frames=8)
        self.trigger_pub.publish(Bool(data=True))
        rospy.sleep(0.5)
        self.assertFalse(
            any(msg.velocity.z < 0.0 for msg in self._setpoint_snapshot())
        )
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    "CAMERA_NOT_CALIBRATED" in state
                    for state in self._state_snapshot()
                )
            ),
            "invalid K was not reported as the sole precheck blocker",
        )

    def test_replayed_message_headers_never_complete_precheck(self):
        self.publish_vehicle_context(
            armed=True,
            mode="OFFBOARD",
            header_offset_sec=-5.0,
        )
        self.publish_camera_info(valid=True, header_offset_sec=-5.0)
        self.publish_centered_marker(
            marker_id=37,
            frames=8,
            header_offset_sec=-5.0,
        )
        self.trigger_pub.publish(Bool(data=True))
        rospy.sleep(0.4)

        self.assertEqual(self._setpoint_count(), 0)
        self.assertFalse(
            any(
                state.startswith("ACQUIRE")
                for state in self._state_snapshot()
            )
        )
        self.assertTrue(
            any(
                "STALE_CAMERA_INFO" in state
                for state in self._state_snapshot()
            )
        )

    def test_held_yaw_is_captured_at_trigger_rising_edge(self):
        self.publish_vehicle_context(
            armed=True, mode="OFFBOARD", yaw_rad=0.0
        )
        self.trigger_pub.publish(Bool(data=True))
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    not state.startswith("IDLE")
                    for state in self._state_snapshot()
                )
            ),
            "node did not process the rising trigger",
        )

        self.publish_vehicle_context(
            armed=True, mode="OFFBOARD", yaw_rad=math.pi / 2.0
        )
        self.publish_camera_info(valid=True)
        self.publish_centered_marker(
            marker_id=37, frames=8, center_y=190
        )
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    msg.velocity.y > 0.01
                    and abs(msg.velocity.y) > abs(msg.velocity.x)
                    for msg in self._setpoint_snapshot()
                )
            ),
            "body-forward correction did not rotate with disturbed live yaw",
        )
        rotated_setpoint = next(
            msg
            for msg in reversed(self._setpoint_snapshot())
            if msg.velocity.y > 0.01
            and abs(msg.velocity.y) > abs(msg.velocity.x)
        )
        self.assertAlmostEqual(
            rotated_setpoint.yaw, 0.0, delta=1.0e-3
        )
        self.publish_centered_marker(marker_id=37, frames=6)
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("REQUEST_AUTO_LAND")
                    for state in self._state_snapshot()
                )
            ),
            "yaw regression did not reach the normal handoff state",
        )
        self.publish_vehicle_context(armed=True, mode="AUTO.LAND")
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("DONE")
                    for state in self._state_snapshot()
                )
            )
        )

    def test_held_yaw_is_captured_from_first_fresh_pose_during_precheck(self):
        self.publish_mavros_state(armed=True, mode="OFFBOARD")
        invalid_pose = PoseStamped()
        invalid_pose.header.stamp = rospy.Time.now()
        invalid_pose.pose.orientation.w = float("nan")
        self.pose_pub.publish(invalid_pose)
        rospy.sleep(0.1)

        self.trigger_pub.publish(Bool(data=True))
        rospy.sleep(0.2)
        self.assertEqual(
            self._setpoint_count(),
            0,
            "node published control before receiving a valid fresh pose",
        )

        expected_yaw = 0.4
        self.publish_vehicle_context(
            armed=True, mode="OFFBOARD", yaw_rad=expected_yaw
        )
        self.publish_camera_info(valid=True)
        self.publish_centered_marker(marker_id=37, frames=10)
        self.assertTrue(
            self.wait_for(lambda: self._setpoint_count() > 0),
            "later fresh pose did not unblock PRECHECK control",
        )
        self.assertAlmostEqual(
            self._setpoint_snapshot()[-1].yaw,
            expected_yaw,
            delta=1.0e-3,
        )

        self.publish_vehicle_context(
            armed=True, mode="OFFBOARD", yaw_rad=1.0
        )
        rospy.sleep(0.1)
        self.assertAlmostEqual(
            self._setpoint_snapshot()[-1].yaw,
            expected_yaw,
            delta=1.0e-3,
            msg="held yaw was captured more than once for one mission",
        )
        self.publish_centered_marker(marker_id=37, frames=8)
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("REQUEST_AUTO_LAND")
                    for state in self._state_snapshot()
                )
            ),
            "late-yaw regression did not reach the normal handoff state",
        )
        self.publish_vehicle_context(armed=True, mode="AUTO.LAND")
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("DONE")
                    for state in self._state_snapshot()
                )
            )
        )

    def test_idle_detection_cannot_bind_next_mission_target(self):
        self.publish_vehicle_context(armed=True, mode="OFFBOARD")
        self.publish_camera_info(valid=True)
        self.publish_centered_marker(marker_id=37, frames=8)
        self.assertFalse(
            any(marker_id == 37 for marker_id in self._locked_id_snapshot()),
            "detector-only frames bound a mission target while IDLE",
        )

        self.target_id_pub.publish(Int32(data=12))
        self.trigger_pub.publish(Bool(data=True))
        self.publish_centered_marker(marker_id=12, frames=10)
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    marker_id == 12
                    for marker_id in self._locked_id_snapshot()
                )
            ),
            "requested ID did not take effect at the mission boundary",
        )
        self.assertFalse(
            any(marker_id == 37 for marker_id in self._locked_id_snapshot())
        )

        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("REQUEST_AUTO_LAND")
                    for state in self._state_snapshot()
                )
            ),
            "mission-boundary regression did not reach the handoff state",
        )
        self.publish_vehicle_context(armed=True, mode="AUTO.LAND")
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("DONE")
                    for state in self._state_snapshot()
                )
            ),
            "mission-boundary regression did not finish cleanly",
        )
        self.target_id_pub.publish(Int32(data=-1))

    def test_stale_critical_input_aborts_without_auto_land(self):
        self.publish_vehicle_context(
            armed=True, mode="OFFBOARD", yaw_rad=0.0
        )
        self.publish_camera_info(valid=True)
        self.trigger_pub.publish(Bool(data=True))
        self.publish_centered_marker(
            marker_id=37, frames=12, center_x=365
        )
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("DESCEND_FINAL")
                    for state in self._state_snapshot()
                )
            ),
            "node did not reach final descent for stale-input regression",
        )
        self.assertEqual(self._mode_request_count(), 0)

        self.publish_vehicle_context(
            armed=True, mode="OFFBOARD", yaw_rad=1.0
        )
        self.trigger_pub.publish(Bool(data=False))
        rospy.sleep(0.1)
        count_before_retrigger = self._setpoint_count()
        self.trigger_pub.publish(Bool(data=True))
        self.assertTrue(
            self.wait_for(
                lambda: self._setpoint_count() > count_before_retrigger
            ),
            "active mission stopped publishing after trigger pulse",
        )
        self.assertAlmostEqual(
            self._setpoint_snapshot()[-1].yaw, 0.0, delta=1.0e-3,
            msg="active-mission trigger pulse overwrote held yaw",
        )

        setpoint_start = self._setpoint_count()
        rospy.sleep(2.2)
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("ABORT_HOLD")
                    for state in self._state_snapshot()
                )
            ),
            "stale critical input did not enter ABORT_HOLD",
        )
        self.assertEqual(self._mode_request_count(), 0)
        self.assertFalse(
            any(
                msg.velocity.z < 0.0
                for msg in self._setpoint_snapshot()[setpoint_start:]
            )
        )

    def test_authorization_expires_before_delayed_call(self):
        self.publish_vehicle_context(armed=True, mode="OFFBOARD")
        self.publish_camera_info(valid=True)
        self.trigger_pub.publish(Bool(data=True))
        self.publish_centered_marker(
            marker_id=37, frames=12, center_x=365
        )
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("DESCEND_FINAL")
                    for state in self._state_snapshot()
                )
            ),
            "node did not reach final descent for expiry regression",
        )
        self.assertEqual(self._mode_request_count(), 0)

        self.publish_vehicle_context(armed=True, mode="OFFBOARD")
        self.publish_camera_info(valid=True)
        context_stopped_at = time.monotonic()
        while time.monotonic() - context_stopped_at < 1.55:
            rospy.sleep(0.01)

        self.publish_centered_marker(marker_id=37, frames=3)
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("REQUEST_AUTO_LAND")
                    for state in self._state_snapshot()
                )
            ),
            "fresh near-expiry context did not enqueue AUTO.LAND",
        )
        self.assertLess(
            time.monotonic() - context_stopped_at,
            2.0,
            "AUTO.LAND was not authorized before input expiry",
        )

        rospy.sleep(0.7)
        self.assertEqual(
            self._mode_request_count(), 0,
            "cached authorization survived its freshness deadline",
        )

    def test_auto_land_calls_start_at_most_2hz(self):
        self.mode_delay_sec = 0.05
        self.publish_vehicle_context(armed=True, mode="OFFBOARD")
        self.publish_camera_info(valid=True)
        self.trigger_pub.publish(Bool(data=True))
        self.publish_centered_marker(marker_id=37, frames=12)
        self.assertTrue(
            self.wait_for(lambda: self._mode_request_count() >= 2, timeout=3.0),
            "two AUTO.LAND attempts did not start",
        )
        with self.lock:
            start_delta = (
                self.mode_request_start_times[1]
                - self.mode_request_start_times[0]
            )
            self.assertGreaterEqual(start_delta, 0.49)
            self.assertEqual(self.mode_requests[:2], ["AUTO.LAND"] * 2)

        self.publish_vehicle_context(armed=True, mode="AUTO.LAND")
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("DONE")
                    for state in self._state_snapshot()
                )
            ),
            "rate-limit scenario did not accept mode confirmation",
        )
        self.assertTrue(
            self.wait_for(lambda: self._mode_request_complete_count() >= 2),
            "rate-limit service callbacks did not finish",
        )

    def test_auto_land_request_does_not_block_control_loop(self):
        self.mode_delay_sec = 0.5
        self.publish_vehicle_context(armed=True, mode="OFFBOARD")
        self.publish_camera_info(valid=True)
        self.trigger_pub.publish(Bool(data=True))
        self.publish_centered_marker(marker_id=37, frames=12)
        self.assertTrue(
            self.wait_for(lambda: self._mode_request_count() > 0),
            "AUTO.LAND was not requested",
        )

        count_at_request = self._setpoint_count()
        rospy.sleep(0.2)
        self.assertGreaterEqual(
            self._setpoint_count(),
            count_at_request + 2,
            "delayed SetMode service blocked the 20 Hz control loop",
        )
        with self.lock:
            self.assertEqual(self.mode_requests, ["AUTO.LAND"])

        self.publish_vehicle_context(armed=True, mode="AUTO.LAND")
        self.assertTrue(
            self.wait_for(
                lambda: any(state.startswith("DONE")
                            for state in self._state_snapshot())
            ),
            "MAVROS mode confirmation did not complete landing state",
        )
        self.assertTrue(
            self.wait_for(lambda: self._mode_request_complete_count() == 1),
            "mock SetMode callback did not finish before test teardown",
        )

    def test_manual_mode_change_stops_stream(self):
        self.publish_vehicle_context(armed=True, mode="OFFBOARD")
        self.publish_camera_info(valid=True)
        self.trigger_pub.publish(Bool(data=True))
        self.publish_centered_marker(marker_id=37, frames=12)
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("REQUEST_AUTO_LAND")
                    for state in self._state_snapshot()
                )
            ),
            "AUTO.LAND request was not enqueued before manual takeover",
        )
        self.assertEqual(self._mode_request_count(), 0)
        self.publish_vehicle_context(armed=True, mode="POSCTL")
        count_after_change = self._setpoint_count()
        rospy.sleep(0.6)
        self.assertLessEqual(self._setpoint_count(), count_after_change + 1)
        self.assertEqual(
            self._mode_request_count(), 0,
            "manual takeover did not cancel pending AUTO.LAND",
        )

    def test_falling_trigger_keeps_tracker_mutating_for_active_mission(self):
        self.publish_vehicle_context(armed=True, mode="OFFBOARD")
        self.publish_camera_info(valid=True)
        self.trigger_pub.publish(Bool(data=True))
        self.publish_centered_marker(
            marker_id=37, frames=10, marker_size_px=150
        )
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("DESCEND_FINAL")
                    for state in self._state_snapshot()
                )
            ),
            "node did not reach active visual descent",
        )

        self.trigger_pub.publish(Bool(data=False))
        for center_x in (360, 400, 440, 480, 520):
            self.publish_centered_marker(
                marker_id=37,
                frames=3,
                center_x=center_x,
                marker_size_px=150,
            )
        self.publish_vehicle_context(armed=True, mode="OFFBOARD")
        self.publish_camera_info(valid=True)
        self.publish_centered_marker(
            marker_id=37,
            frames=12,
            center_x=520,
            marker_size_px=150,
        )

        self.assertFalse(
            any(
                state.startswith("REACQUIRE")
                for state in self._state_snapshot()
            ),
            "falling trigger froze jump history and caused false target loss",
        )
        self.assertTrue(
            any(
                abs(msg.velocity.x) > 0.05 or abs(msg.velocity.y) > 0.05
                for msg in self._setpoint_snapshot()
            ),
            "active mission stopped consuming updated target poses",
        )

        for center_x in (480, 440, 400, 360, 320):
            self.publish_centered_marker(
                marker_id=37,
                frames=3,
                center_x=center_x,
                marker_size_px=150,
            )
        self.publish_vehicle_context(armed=True, mode="OFFBOARD")
        self.publish_camera_info(valid=True)
        for marker_size_px in (160, 180, 200):
            self.publish_centered_marker(
                marker_id=37,
                frames=3,
                marker_size_px=marker_size_px,
            )
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("REQUEST_AUTO_LAND")
                    for state in self._state_snapshot()
                )
            ),
            "falling-trigger regression did not reach a clean handoff",
        )
        self.publish_vehicle_context(armed=True, mode="AUTO.LAND")
        self.assertTrue(
            self.wait_for(
                lambda: any(
                    state.startswith("DONE")
                    for state in self._state_snapshot()
                )
            )
        )


if __name__ == "__main__":
    test_order = {
        "test_no_setpoint_before_trigger": 0,
        "test_invalid_camera_info_never_descends": 1,
        "test_replayed_message_headers_never_complete_precheck": 2,
        "test_held_yaw_is_captured_at_trigger_rising_edge": 3,
        "test_held_yaw_is_captured_from_first_fresh_pose_during_precheck": 4,
        "test_idle_detection_cannot_bind_next_mission_target": 5,
        "test_stale_critical_input_aborts_without_auto_land": 6,
        "test_authorization_expires_before_delayed_call": 7,
        "test_auto_land_calls_start_at_most_2hz": 8,
        "test_auto_land_request_does_not_block_control_loop": 9,
        "test_falling_trigger_keeps_tracker_mutating_for_active_mission": 10,
        "test_manual_mode_change_stops_stream": 11,
    }

    def compare_test_names(left, right):
        left_order = test_order[left]
        right_order = test_order[right]
        return (left_order > right_order) - (left_order < right_order)

    unittest.TestLoader.sortTestMethodsUsing = staticmethod(compare_test_names)
    rospy.init_node("precision_landing_node_test")
    rostest.rosrun(
        "precision_landing",
        "precision_landing_node_test",
        PrecisionLandingNodeTest,
    )
