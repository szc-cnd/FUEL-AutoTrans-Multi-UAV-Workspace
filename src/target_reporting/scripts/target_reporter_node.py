#!/usr/bin/env python3
import json
import os
import threading

import rospy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from target_reporting.adapters import parse_color_status, parse_qr_status, parse_thermal_status
from target_reporting.candidates import CandidateTracker
from target_reporting.evidence import TimestampedImageCache, build_evidence_jpeg, image_metadata
from target_reporting.model import Position, TargetEvent
from target_reporting.store import MissionStore
from target_reporting.transport import ImageAckClient, JsonAckClient, LatestJsonClient


class TargetReporterNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.candidate_lock = threading.Lock()
        self.drone_id = rospy.get_param("~drone_id", "uav1")
        self.localization_frame = rospy.get_param("~localization_frame", "camera_init")
        self.report_frame = rospy.get_param("~report_frame", "channel")
        self.use_detector_candidates = bool(
            rospy.get_param("~use_detector_candidates", True)
        )
        self.image_tolerance = float(rospy.get_param("~image_tolerance_s", 0.5))
        self.mission_id = rospy.get_param("~mission_id", "competition_current")
        self.store = MissionStore(
            rospy.get_param("~record_root", "~/target_reports"), self.drone_id,
            mission_id=self.mission_id,
        )
        self.tracker = CandidateTracker(
            rospy.get_param("~dedup_distance_m", 0.30), rospy.get_param("~confirm_hits", 3)
        )
        self.images = TimestampedImageCache(rospy.get_param("~image_cache_items", 40))
        host = rospy.get_param("~remote_host", "192.168.10.100")
        remote_port = rospy.get_param("~remote_port", 5000)
        self.json_client = JsonAckClient(
            host, remote_port, ack_callback=self.store.mark_event_acked
        )
        self.image_client = ImageAckClient(
            host, rospy.get_param("~image_port", 5001),
            ack_callback=self.store.mark_image_acked,
        )
        self.realtime_client = LatestJsonClient(host, remote_port)
        self.json_client.start()
        self.image_client.start()
        self.realtime_client.start()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.observation_pub = rospy.Publisher("/UAV0/target_reporting/observation", String, queue_size=10)
        self.retry_timer = None
        self.latest_point = {}
        self.latest_status = {}
        self.point_version = {}
        self.status_version = {}
        self.last_processed_pair = {}
        self.seq = 0
        existing = self.store.all_events()
        if existing:
            self.seq = max(int(event["seq"]) for event in existing)
        for event in self.store.pending_events():
            self.json_client.enqueue(event)
        for image_item in self.store.pending_images():
            self.image_client.enqueue(image_item)

        if self.use_detector_candidates:
            self._subscribe(
                "color",
                rospy.get_param(
                    "~color_candidate_point_topic",
                    "/UAV0/color_tag_detector/candidate_point_camera",
                ),
                PointStamped,
                rospy.get_param(
                    "~color_candidate_status_topic",
                    "/UAV0/color_tag_detector/candidate_text",
                ),
                String,
                "/UAV0/color_tag_detector/debug_image",
            )
            self._subscribe(
                "qr",
                rospy.get_param(
                    "~qr_candidate_point_topic",
                    "/UAV0/vision/qr_candidate_pose_camera",
                ),
                PoseStamped,
                rospy.get_param(
                    "~qr_candidate_status_topic",
                    "/UAV0/vision/qr_candidate_detected",
                ),
                String,
                "/UAV0/vision/qr_debug_image",
            )
            self._subscribe(
                "thermal",
                rospy.get_param(
                    "~thermal_candidate_point_topic",
                    "/UAV0/thermal/target_candidate_camera_point",
                ),
                PointStamped,
                rospy.get_param(
                    "~thermal_candidate_status_topic",
                    "/UAV0/thermal/target_candidate_detected",
                ),
                Bool,
                "/UAV0/thermal/debug_image",
            )
        else:
            # Compatibility path for old detector nodes without candidate topics.
            self._subscribe(
                "color", "/UAV0/color_tag_detector/target_point_camera", PointStamped,
                "/UAV0/color_tag_detector/result_text", String, "/UAV0/color_tag_detector/debug_image"
            )
            self._subscribe(
                "qr", "/UAV0/vision/qr_pose_camera", PoseStamped,
                "/UAV0/vision/qr_detected", String, "/UAV0/vision/qr_debug_image"
            )
            self._subscribe(
                "thermal", "/UAV0/thermal/target_camera_point", PointStamped,
                "/UAV0/thermal/fusion_valid", Bool, "/UAV0/thermal/debug_image"
            )
        # TF 可能在节点启动后的短时间内尚未建立。候选不能因为一次 TF
        # 查询失败就丢失，因此定时重试尚未处理的点/状态配对。
        self.retry_timer = rospy.Timer(rospy.Duration(0.10), self._retry_pending)
        rospy.on_shutdown(self.shutdown)

    def _subscribe(self, source, point_topic, point_type, status_topic, status_type, image_topic):
        rospy.Subscriber(point_topic, point_type, lambda msg: self._point_cb(source, msg), queue_size=5)
        rospy.Subscriber(status_topic, status_type, lambda msg: self._status_cb(source, msg), queue_size=5)
        rospy.Subscriber(image_topic, Image, lambda msg: self._image_cb(source, msg), queue_size=2)

    def _point_cb(self, source, msg):
        point = msg
        if isinstance(msg, PoseStamped):
            point = PointStamped(header=msg.header)
            point.point = msg.pose.position
        with self.lock:
            self.latest_point[source] = point
            self.point_version[source] = self.point_version.get(source, 0) + 1
        self._try_process(source)

    def _status_cb(self, source, msg):
        try:
            if source == "color":
                status = parse_color_status(msg.data)
            elif source == "qr":
                status = parse_qr_status(msg.data)
            else:
                status = parse_thermal_status(msg.data)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(2.0, "Invalid %s status: %s", source, exc)
            return
        with self.lock:
            self.latest_status[source] = status
            self.status_version[source] = self.status_version.get(source, 0) + 1
        self._try_process(source)

    def _image_cb(self, source, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.images.add(source, msg.header.stamp.to_sec(), image)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Cannot cache %s image: %s", source, exc)

    def _retry_pending(self, _event):
        for source in ("color", "qr", "thermal"):
            self._try_process(source)

    def _try_process(self, source):
        with self.lock:
            point = self.latest_point.get(source)
            status = self.latest_status.get(source)
            pair = (self.point_version.get(source, 0), self.status_version.get(source, 0))
        if point is None or status is None or not point.header.frame_id:
            return
        with self.lock:
            previous = self.last_processed_pair.get(source, (0, 0))
            if pair[0] <= previous[0] or pair[1] <= previous[1]:
                return
        result, confidence = status
        try:
            world = self.tf_buffer.transform(point, self.localization_frame, rospy.Duration(0.10))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "TF %s -> %s failed: %s", point.header.frame_id,
                                   self.localization_frame, exc)
            return
        # Only mark a pair processed after TF succeeds. The second check keeps
        # the callback and retry timer from processing the same pair twice.
        with self.lock:
            previous = self.last_processed_pair.get(source, (0, 0))
            if pair[0] <= previous[0] or pair[1] <= previous[1]:
                return
            self.last_processed_pair[source] = pair
        position = {"x": world.point.x, "y": world.point.y, "z": world.point.z}
        target_type = {"color": "color_tag", "qr": "qr_code", "thermal": "thermal_source"}[source]
        with self.candidate_lock:
            candidate, confirmed = self.tracker.update(target_type, result, position)
            candidate_number = candidate["local_number"]
            candidate_hits = candidate["hits"]
            if confirmed:
                self.seq += 1
                event_seq = self.seq
            else:
                event_seq = None
        observation = {
            "version": 1, "message_type": "observation", "target_type": target_type,
            "timestamp": point.header.stamp.to_sec(), "drone_id": self.drone_id,
            "target_id": "{}-{}-{:03d}".format(self.drone_id, target_type, candidate_number),
            "result": result,
            "position": {"frame_id": self.report_frame, "unit": "m", **position},
            "hits": candidate_hits,
            "candidate": not confirmed,
            "confirmed": confirmed,
        }
        self.observation_pub.publish(String(data=json.dumps(observation, ensure_ascii=False)))
        self.realtime_client.publish(observation)
        if not confirmed:
            return
        target_id = observation["target_id"]
        image = self.images.nearest(source, point.header.stamp.to_sec(), self.image_tolerance)
        image_id = "{}_seq{:06d}_{}.jpg".format(self.drone_id, event_seq, target_type)
        event = TargetEvent(
            seq=event_seq, timestamp=point.header.stamp.to_sec(), drone_id=self.drone_id,
            message_type="confirmed_target", target_id=target_id, target_type=target_type,
            result=result, position=Position(self.report_frame, world.point.x, world.point.y, world.point.z),
            confidence=confidence,
        ).to_dict()
        jpeg = None
        if image is not None:
            jpeg = build_evidence_jpeg(image, event,
                                       (point.point.x, point.point.y, point.point.z))
            event["image"] = image_metadata(image_id, jpeg)
            self.store.save_image(image_id, jpeg)
        if self.store.append_event(event):
            self.json_client.enqueue(event)
            if jpeg is not None:
                self.image_client.enqueue((image_id, jpeg))
            rospy.loginfo("Confirmed %s %s at channel [%.3f %.3f %.3f]", target_type,
                          target_id, world.point.x, world.point.y, world.point.z)

    def shutdown(self):
        if self.retry_timer is not None:
            self.retry_timer.shutdown()
        self.json_client.stop()
        self.image_client.stop()
        self.realtime_client.stop()


def main():
    rospy.init_node("target_reporter")
    TargetReporterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
