#!/usr/bin/env python3
"""Convert target_reporting observations into RViz MarkerArray messages.

The planner does not subscribe to this topic.  It is deliberately a display
adapter only: target_reporting has already transformed detector points into
the configured report frame, and this node only renders those coordinates.
"""

import json
import math

import rospy
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

        self.publisher = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1, latch=True
        )
        self.subscription = rospy.Subscriber(
            self.observation_topic, String, self.observation_callback, queue_size=20
        )
        self.states = {name: {"candidate": None, "confirmed": None} for name in self.TARGETS}
        self.visible_ids = set()
        rospy.loginfo(
            "target_rviz_marker_node: observation=%s marker=%s frame=%s",
            self.observation_topic,
            self.marker_topic,
            self.marker_frame,
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
        }
        if bool(observation.get("confirmed", False)):
            self.states[target_type]["confirmed"] = state
            self.states[target_type]["candidate"] = None
        elif bool(observation.get("candidate", False)):
            self.states[target_type]["candidate"] = state
        else:
            return
        self.publish_markers()

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
        for target_type, (index, _display_name) in self.TARGETS.items():
            target_states = self.states[target_type]
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
