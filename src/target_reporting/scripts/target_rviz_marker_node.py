#!/usr/bin/env python3
"""Convert target_reporting observations into RViz MarkerArray messages.

The planner does not subscribe to this topic.  It is deliberately a display
adapter only: target_reporting has already transformed detector points into
the configured report frame, and this node renders those coordinates plus a
small filtered PointCloud2 around the reported targets.
"""

import json
import math
import threading

import rospy
import tf2_geometry_msgs  # noqa: F401  (register PointStamped TF support)
import tf2_ros
from geometry_msgs.msg import PointStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class TargetRvizMarkerNode:
    TARGETS = {
        "color_tag": (0, "颜色标签"),
        "qr_code": (1, "二维码"),
        "thermal_source": (2, "热源"),
    }

    def __init__(self):
        self.observation_topic = rospy.get_param(
            "~observation_topic", "/UAV0/target_reporting/observation"
        )
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/UAV0/target_reporting/markers"
        )
        # The competition setup uses identical world/channel coordinates.  A
        # separate parameter keeps the display adapter usable if the RViz
        # fixed frame is renamed later.
        self.marker_frame = rospy.get_param("~marker_frame", "world")
        self.sphere_scale = max(0.05, float(rospy.get_param("~sphere_scale", 0.28)))
        self.text_height = max(0.05, float(rospy.get_param("~text_height", 0.28)))
        self.text_offset = float(rospy.get_param("~text_offset", 0.35))

        # The camera cloud is an input only.  RViz receives a filtered cloud
        # containing points near the currently reported target positions.
        self.cloud_topic = rospy.get_param(
            "~cloud_topic", "/camera/depth/color/points"
        )
        self.object_cloud_topic = rospy.get_param(
            "~object_cloud_topic",
            "/UAV0/target_reporting/detected_object_cloud",
        )
        self.object_cloud_radius = max(
            0.03, float(rospy.get_param("~object_cloud_radius", 0.25))
        )
        self.object_cloud_timeout = max(
            0.05, float(rospy.get_param("~object_cloud_timeout", 0.8))
        )
        self.object_cloud_period = max(
            0.05, float(rospy.get_param("~object_cloud_period", 0.2))
        )
        self.object_cloud_max_points = max(
            0, int(rospy.get_param("~object_cloud_max_points", 12000))
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.cloud_publisher = rospy.Publisher(
            self.object_cloud_topic, PointCloud2, queue_size=1
        )
        self.cloud_subscription = rospy.Subscriber(
            self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1
        )
        self._state_lock = threading.RLock()
        self._last_cloud_process = rospy.Time(0)

        self.publisher = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1, latch=True
        )
        self.subscription = rospy.Subscriber(
            self.observation_topic, String, self.observation_callback, queue_size=20
        )
        self.states = {name: {"candidate": None, "confirmed": None} for name in self.TARGETS}
        self.visible_ids = set()
        rospy.loginfo(
            "target_rviz_marker_node: observation=%s marker=%s frame=%s "
            "cloud=%s filtered_cloud=%s radius=%.2fm",
            self.observation_topic,
            self.marker_topic,
            self.marker_frame,
            self.cloud_topic,
            self.object_cloud_topic,
            self.object_cloud_radius,
        )

    @staticmethod
    def _finite(value):
        try:
            return math.isfinite(float(value))
        except (TypeError, ValueError):
            return False

    def observation_callback(self, message):
        try:
            observation = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(2.0, "Invalid detection observation JSON: %s", exc)
            return

        target_type = observation.get("target_type")
        if target_type not in self.TARGETS:
            return
        position = observation.get("position") or {}
        if not all(self._finite(position.get(axis)) for axis in ("x", "y", "z")):
            rospy.logwarn_throttle(2.0, "Ignore %s observation with invalid position", target_type)
            return

        state = {
            "x": float(position["x"]),
            "y": float(position["y"]),
            "z": float(position["z"]),
            "hits": int(observation.get("hits", 0) or 0),
            "target_id": str(observation.get("target_id", "")),
            "result": observation.get("result") or {},
            "timestamp": self._observation_timestamp(observation),
        }
        with self._state_lock:
            if bool(observation.get("confirmed", False)):
                self.states[target_type]["confirmed"] = state
                self.states[target_type]["candidate"] = None
            elif bool(observation.get("candidate", False)):
                self.states[target_type]["candidate"] = state
            else:
                return
        self.publish_markers()

    @staticmethod
    def _observation_timestamp(observation):
        try:
            timestamp = float(observation.get("timestamp"))
            if math.isfinite(timestamp) and timestamp > 0.0:
                return timestamp
        except (TypeError, ValueError):
            pass
        return rospy.Time.now().to_sec()

    @staticmethod
    def _frame_name(frame):
        return str(frame or "").lstrip("/")

    def _target_points_in_cloud_frame(self, cloud):
        now = rospy.Time.now().to_sec()
        cloud_frame = self._frame_name(cloud.header.frame_id)
        if not cloud_frame:
            rospy.logwarn_throttle(2.0, "D435 point cloud has empty frame_id")
            return []

        with self._state_lock:
            states = []
            for target_type in self.TARGETS:
                target_state = self.states[target_type]
                confirmed = target_state.get("confirmed")
                candidate = target_state.get("candidate")
                # A confirmed target remains registered like its marker.  A
                # candidate is temporary and must disappear if frames stop.
                if confirmed is not None:
                    states.append(confirmed)
                elif candidate is not None:
                    age = now - float(candidate.get("timestamp", now))
                    if age <= self.object_cloud_timeout:
                        states.append(candidate)

        points = []
        for state in states:
            source = PointStamped()
            source.header.frame_id = self._frame_name(self.marker_frame)
            source.header.stamp = cloud.header.stamp
            source.point.x = state["x"]
            source.point.y = state["y"]
            source.point.z = state["z"]
            try:
                if self._frame_name(source.header.frame_id) == cloud_frame:
                    transformed = source
                else:
                    transformed = self.tf_buffer.transform(
                        source, cloud_frame, rospy.Duration(0.08)
                    )
                points.append(
                    (
                        float(transformed.point.x),
                        float(transformed.point.y),
                        float(transformed.point.z),
                    )
                )
            except Exception as exc:
                rospy.logwarn_throttle(
                    2.0,
                    "Cannot transform detection marker %s from %s to %s: %s",
                    state.get("target_id", "unknown"),
                    source.header.frame_id,
                    cloud_frame,
                    exc,
                )
        return points

    def cloud_callback(self, cloud):
        now = rospy.Time.now()
        if (
            self._last_cloud_process != rospy.Time(0)
            and (now - self._last_cloud_process).to_sec() < self.object_cloud_period
        ):
            return
        self._last_cloud_process = now

        target_points = self._target_points_in_cloud_frame(cloud)
        selected = []
        if target_points:
            radius_sq = self.object_cloud_radius * self.object_cloud_radius
            try:
                for point in point_cloud2.read_points(
                    cloud, field_names=("x", "y", "z"), skip_nans=True
                ):
                    x, y, z = (float(point[0]), float(point[1]), float(point[2]))
                    if any(
                        (x - target[0]) ** 2
                        + (y - target[1]) ** 2
                        + (z - target[2]) ** 2
                        <= radius_sq
                        for target in target_points
                    ):
                        selected.append((x, y, z))
                        if (
                            self.object_cloud_max_points > 0
                            and len(selected) >= self.object_cloud_max_points
                        ):
                            break
            except (KeyError, ValueError) as exc:
                rospy.logwarn_throttle(2.0, "Cannot filter D435 point cloud: %s", exc)
                selected = []

        output = point_cloud2.create_cloud_xyz32(cloud.header, selected)
        self.cloud_publisher.publish(output)

    def _label(self, target_type, state, confirmed):
        _, display_name = self.TARGETS[target_type]
        result = state.get("result") or {}
        detail = result.get("data") or result.get("color") or ""
        if isinstance(detail, (dict, list)):
            detail = ""
        detail = str(detail).strip()
        if detail:
            display_name = "%s(%s)" % (display_name, detail)
        if confirmed:
            status = "已确认"
        else:
            status = "候选 %d/3" % max(0, int(state.get("hits", 0)))
        return "%s  %s" % (display_name, status)

    def _marker(self, marker_id, namespace, marker_type, state, color, text=""):
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.type = marker_type
        marker.pose.position.x = state["x"]
        marker.pose.position.y = state["y"]
        marker.pose.position.z = state["z"]
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        if marker_type == Marker.SPHERE:
            marker.scale.x = marker.scale.y = marker.scale.z = self.sphere_scale
        else:
            marker.scale.z = self.text_height
            marker.pose.position.z += self.text_offset
            marker.text = text
        return marker

    @staticmethod
    def _delete(marker_id, namespace):
        marker = Marker()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def publish_markers(self):
        array = MarkerArray()
        desired_ids = set()
        with self._state_lock:
            states = {
                target_type: dict(self.states[target_type])
                for target_type in self.TARGETS
            }
        for target_type, (index, _display_name) in self.TARGETS.items():
            target_states = states[target_type]
            candidate = target_states["candidate"]
            confirmed = target_states["confirmed"]
            if candidate is not None:
                sphere_id, text_id = index, 20 + index
                array.markers.append(
                    self._marker(
                        sphere_id,
                        "target_reporting/candidate",
                        Marker.SPHERE,
                        candidate,
                        (1.0, 0.75, 0.0, 0.95),
                    )
                )
                array.markers.append(
                    self._marker(
                        text_id,
                        "target_reporting/candidate_text",
                        Marker.TEXT_VIEW_FACING,
                        candidate,
                        (1.0, 0.85, 0.1, 1.0),
                        self._label(target_type, candidate, False),
                    )
                )
                desired_ids.update((sphere_id, text_id))
            if confirmed is not None:
                sphere_id, text_id = 10 + index, 30 + index
                array.markers.append(
                    self._marker(
                        sphere_id,
                        "target_reporting/confirmed",
                        Marker.SPHERE,
                        confirmed,
                        (0.1, 1.0, 0.2, 0.95),
                    )
                )
                array.markers.append(
                    self._marker(
                        text_id,
                        "target_reporting/confirmed_text",
                        Marker.TEXT_VIEW_FACING,
                        confirmed,
                        (0.2, 1.0, 0.3, 1.0),
                        self._label(target_type, confirmed, True),
                    )
                )
                desired_ids.update((sphere_id, text_id))

        for marker_id in self.visible_ids - desired_ids:
            namespace = (
                "target_reporting/candidate"
                if marker_id < 10
                else "target_reporting/confirmed"
            )
            if 20 <= marker_id < 30:
                namespace = "target_reporting/candidate_text"
            elif marker_id >= 30:
                namespace = "target_reporting/confirmed_text"
            array.markers.append(self._delete(marker_id, namespace))
        self.visible_ids = desired_ids
        self.publisher.publish(array)


def main():
    rospy.init_node("target_rviz_marker")
    TargetRvizMarkerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
