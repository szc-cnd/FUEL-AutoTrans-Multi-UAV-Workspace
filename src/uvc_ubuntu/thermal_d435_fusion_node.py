#!/usr/bin/env python3
import math
import os
import threading

import cv2
import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401  Registers PointStamped transforms.
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
import yaml


class ThermalD435FusionNode:
    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        default_config_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "config", "thermal_d435_fusion.yaml"
        )
        self.config_path = rospy.get_param("~config_path", default_config_path)
        self.config = self.load_config(self.config_path)

        self.thermal_target_detected_topic = self.get_param(
            "thermal_target_detected_topic", "/UAV0/thermal/target_detected"
        )
        self.thermal_target_pixel_topic = self.get_param(
            "thermal_target_pixel_topic", "/UAV0/thermal/target_pixel"
        )
        self.thermal_candidate_detected_topic = self.get_param(
            "thermal_candidate_detected_topic", "/UAV0/thermal/target_candidate_detected"
        )
        self.thermal_candidate_pixel_topic = self.get_param(
            "thermal_candidate_pixel_topic", "/UAV0/thermal/target_candidate_pixel"
        )
        self.thermal_candidate_camera_point_topic = self.get_param(
            "thermal_candidate_camera_point_topic",
            "/UAV0/thermal/target_candidate_camera_point",
        )
        self.d435_camera_info_topic = self.get_param(
            "d435_camera_info_topic", "/camera/color/camera_info"
        )
        self.d435_depth_topic = self.get_param(
            "d435_depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.d435_color_topic = self.get_param(
            "d435_color_topic", "/camera/color/image_raw"
        )
        self.d435_debug_image_topic = self.get_param(
            "d435_debug_image_topic", "/UAV0/thermal/d435_debug_image"
        )

        self.homography = np.array(
            self.get_param(
                "thermal_to_d435_homography",
                [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            ),
            dtype=np.float64,
        )
        if self.homography.shape != (3, 3):
            rospy.logwarn("Invalid homography shape %s, using identity", self.homography.shape)
            self.homography = np.eye(3, dtype=np.float64)

        self.depth_scale = float(self.get_param("depth_scale", 0.001))
        self.depth_search_radius = int(self.get_param("depth_search_radius", 5))
        self.min_valid_depth = float(self.get_param("min_valid_depth", 0.2))
        self.max_valid_depth = float(self.get_param("max_valid_depth", 5.0))
        self.target_frame = str(self.get_param("target_frame", "odom"))
        self.d435_optical_frame = str(
            self.get_param("d435_optical_frame", "camera_color_optical_frame")
        )
        self.use_tf = bool(self.get_param("use_tf", True))
        self.publish_rate = float(self.get_param("publish_rate", 20))
        self.debug = bool(self.get_param("debug", True))

        self.target_detected = False
        self.thermal_pixel_msg = None
        self.candidate_target_detected = False
        self.candidate_thermal_pixel_msg = None
        self.camera_info = None
        self.depth_image = None
        self.depth_encoding = None
        self.depth_stamp = rospy.Time(0)
        self.color_image = None
        self.color_header = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.d435_pixel_pub = rospy.Publisher(
            "/UAV0/thermal/target_d435_pixel", PointStamped, queue_size=1
        )
        self.camera_point_pub = rospy.Publisher(
            "/UAV0/thermal/target_camera_point", PointStamped, queue_size=1
        )
        self.candidate_camera_point_pub = rospy.Publisher(
            self.thermal_candidate_camera_point_topic, PointStamped, queue_size=1
        )
        self.channel_position_pub = rospy.Publisher(
            "/UAV0/thermal/target_channel_position", PointStamped, queue_size=1
        )
        self.fusion_valid_pub = rospy.Publisher("/UAV0/thermal/fusion_valid", Bool, queue_size=1)
        self.d435_debug_image_pub = rospy.Publisher(
            self.d435_debug_image_topic, Image, queue_size=1
        )

        rospy.Subscriber(self.thermal_target_detected_topic, Bool, self.detected_callback)
        rospy.Subscriber(self.thermal_target_pixel_topic, rospy.AnyMsg, self.pixel_callback)
        rospy.Subscriber(
            self.thermal_candidate_detected_topic,
            Bool,
            self.candidate_detected_callback,
        )
        rospy.Subscriber(
            self.thermal_candidate_pixel_topic,
            rospy.AnyMsg,
            self.candidate_pixel_callback,
        )
        rospy.Subscriber(self.d435_camera_info_topic, CameraInfo, self.camera_info_callback)
        rospy.Subscriber(self.d435_depth_topic, Image, self.depth_callback, queue_size=1)
        rospy.Subscriber(self.d435_color_topic, Image, self.color_callback, queue_size=1)

        period = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.05
        self.timer = rospy.Timer(rospy.Duration(period), self.timer_callback)

    def load_config(self, path):
        if not path or not os.path.exists(path):
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                loaded = yaml.safe_load(f)
        except Exception as exc:
            rospy.logwarn("Failed to load fusion config %s: %s", path, exc)
            return {}
        return loaded if isinstance(loaded, dict) else {}

    def get_param(self, name, default):
        private_name = "~" + name
        if rospy.has_param(private_name):
            return rospy.get_param(private_name)
        if rospy.has_param(name):
            return rospy.get_param(name)
        return self.config.get(name, default)

    def detected_callback(self, msg):
        with self.lock:
            self.target_detected = bool(msg.data)

    def pixel_callback(self, msg):
        self._pixel_callback(msg, candidate=False)

    def candidate_detected_callback(self, msg):
        with self.lock:
            self.candidate_target_detected = bool(msg.data)

    def candidate_pixel_callback(self, msg):
        self._pixel_callback(msg, candidate=True)

    def _pixel_callback(self, msg, candidate):
        msg_type = msg._connection_header.get("type", "")
        try:
            if msg_type == "geometry_msgs/PointStamped":
                parsed = PointStamped()
                parsed.deserialize(msg._buff)
                point_msg = parsed
            elif msg_type == "geometry_msgs/Point":
                parsed = Point()
                parsed.deserialize(msg._buff)
                point_msg = PointStamped()
                point_msg.header.stamp = rospy.Time.now()
                point_msg.header.frame_id = "thermal_camera"
                point_msg.point = parsed
            else:
                rospy.logwarn_throttle(2.0, "Unsupported thermal target pixel type: %s", msg_type)
                return
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse thermal target pixel: %s", exc)
            return

        with self.lock:
            if candidate:
                self.candidate_thermal_pixel_msg = point_msg
            else:
                self.thermal_pixel_msg = point_msg

    def camera_info_callback(self, msg):
        with self.lock:
            self.camera_info = msg

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to convert depth image: %s", exc)
            return

        with self.lock:
            self.depth_image = np.array(depth, copy=True)
            self.depth_encoding = msg.encoding
            self.depth_stamp = msg.header.stamp

    def color_callback(self, msg):
        try:
            color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to convert D435 color image: %s", exc)
            return

        with self.lock:
            self.color_image = np.array(color, copy=True)
            self.color_header = msg.header

    def publish_valid(self, valid):
        self.fusion_valid_pub.publish(Bool(data=bool(valid)))

    def map_thermal_to_d435(self, u_t, v_t):
        thermal_pt = np.array([[[float(u_t), float(v_t)]]], dtype=np.float32)
        d435_pt = cv2.perspectiveTransform(thermal_pt, self.homography.astype(np.float32))
        u_d = int(round(float(d435_pt[0, 0, 0])))
        v_d = int(round(float(d435_pt[0, 0, 1])))
        return u_d, v_d

    def depth_to_meters(self, value, encoding=None):
        if value is None:
            return None
        depth = float(value)
        if math.isnan(depth) or math.isinf(depth):
            return None
        encoding = self.depth_encoding if encoding is None else encoding
        if encoding in ("16UC1", "mono16"):
            depth *= self.depth_scale
        elif encoding == "32FC1":
            pass
        else:
            depth *= self.depth_scale
        return depth

    def lookup_depth(self, depth_image, u, v, encoding=None):
        h, w = depth_image.shape[:2]
        if u < 0 or v < 0 or u >= w or v >= h:
            return None

        r = max(0, self.depth_search_radius)
        x0 = max(0, u - r)
        x1 = min(w, u + r + 1)
        y0 = max(0, v - r)
        y1 = min(h, v + r + 1)

        valid_depths = []
        for yy in range(y0, y1):
            for xx in range(x0, x1):
                depth = self.depth_to_meters(depth_image[yy, xx], encoding)
                if depth is None:
                    continue
                if self.min_valid_depth <= depth <= self.max_valid_depth:
                    valid_depths.append(depth)

        if not valid_depths:
            return None
        return float(np.median(np.array(valid_depths, dtype=np.float32)))

    def pixel_to_camera_point(self, u, v, depth, camera_info):
        fx = float(camera_info.K[0])
        fy = float(camera_info.K[4])
        cx = float(camera_info.K[2])
        cy = float(camera_info.K[5])

        if fx == 0.0 or fy == 0.0:
            return None

        point = PointStamped()
        point.header.stamp = self.depth_stamp if self.depth_stamp != rospy.Time(0) else rospy.Time.now()
        point.header.frame_id = self.d435_optical_frame
        point.point.x = (float(u) - cx) * depth / fx
        point.point.y = (float(v) - cy) * depth / fy
        point.point.z = depth
        return point

    def publish_d435_debug_image(
        self,
        color_image,
        color_header,
        detected,
        thermal_pixel_msg,
        candidate_detected,
        candidate_pixel_msg,
        depth_image,
        depth_encoding,
    ):
        if color_image is None:
            return

        debug_image = color_image.copy()
        source = None
        status = "NO THERMAL TARGET"
        if detected and thermal_pixel_msg is not None and thermal_pixel_msg.point.z > 0.0:
            source = thermal_pixel_msg
            status = "CONFIRMED"
        elif (
            candidate_detected
            and candidate_pixel_msg is not None
            and candidate_pixel_msg.point.z > 0.0
        ):
            source = candidate_pixel_msg
            status = "CANDIDATE"

        if source is None:
            cv2.putText(
                debug_image,
                status,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2,
            )
        else:
            u_t = float(source.point.x)
            v_t = float(source.point.y)
            u_d, v_d = self.map_thermal_to_d435(u_t, v_t)
            depth = None
            if depth_image is not None and depth_encoding is not None:
                depth = self.lookup_depth(depth_image, u_d, v_d, depth_encoding)

            height, width = debug_image.shape[:2]
            in_image = 0 <= u_d < width and 0 <= v_d < height
            color = (0, 255, 0) if depth is not None and in_image else (0, 0, 255)
            if in_image:
                cv2.circle(
                    debug_image,
                    (u_d, v_d),
                    max(1, self.depth_search_radius),
                    color,
                    2,
                )
                cv2.drawMarker(
                    debug_image,
                    (u_d, v_d),
                    color,
                    markerType=cv2.MARKER_CROSS,
                    markerSize=24,
                    thickness=2,
                )

            depth_text = "invalid" if depth is None else "%.3f m" % depth
            cv2.putText(
                debug_image,
                "%s thermal=(%.1f,%.1f) d435=(%d,%d) depth=%s"
                % (status, u_t, v_t, u_d, v_d, depth_text),
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )

        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        if color_header is not None:
            debug_msg.header = color_header
        self.d435_debug_image_pub.publish(debug_msg)

    def build_camera_point(self, thermal_pixel_msg, camera_info, depth_image, depth_encoding):
        """Fuse one thermal pixel with the latest D435 depth, without TF."""
        if (
            thermal_pixel_msg is None
            or thermal_pixel_msg.point.z <= 0.0
            or camera_info is None
            or depth_image is None
            or depth_encoding is None
        ):
            return None
        u_d, v_d = self.map_thermal_to_d435(
            thermal_pixel_msg.point.x, thermal_pixel_msg.point.y
        )
        depth = self.lookup_depth(depth_image, u_d, v_d, depth_encoding)
        if depth is None:
            return None
        return self.pixel_to_camera_point(u_d, v_d, depth, camera_info)

    def timer_callback(self, _event):
        with self.lock:
            detected = self.target_detected
            thermal_pixel_msg = self.thermal_pixel_msg
            candidate_detected = self.candidate_target_detected
            candidate_pixel_msg = self.candidate_thermal_pixel_msg
            camera_info = self.camera_info
            depth_image = None if self.depth_image is None else self.depth_image.copy()
            depth_encoding = self.depth_encoding
            color_image = None if self.color_image is None else self.color_image.copy()
            color_header = self.color_header

        self.publish_d435_debug_image(
            color_image,
            color_header,
            detected,
            thermal_pixel_msg,
            candidate_detected,
            candidate_pixel_msg,
            depth_image,
            depth_encoding,
        )

        # Candidate fusion is independent of the package's stable 2/3-frame
        # stream. It only requires a valid raw hotspot pixel and valid depth;
        # target_reporting performs the later world-frame tracking/confirmation.
        if candidate_detected:
            candidate_camera_point = self.build_camera_point(
                candidate_pixel_msg, camera_info, depth_image, depth_encoding
            )
            if candidate_camera_point is not None:
                self.candidate_camera_point_pub.publish(candidate_camera_point)

        if not detected or thermal_pixel_msg is None or thermal_pixel_msg.point.z <= 0.0:
            self.publish_valid(False)
            return

        if camera_info is None:
            rospy.logwarn_throttle(2.0, "Waiting for D435 camera_info")
            self.publish_valid(False)
            return

        if depth_image is None or depth_encoding is None:
            rospy.logwarn_throttle(2.0, "Waiting for aligned D435 depth image")
            self.publish_valid(False)
            return

        u_t = thermal_pixel_msg.point.x
        v_t = thermal_pixel_msg.point.y
        u_d, v_d = self.map_thermal_to_d435(u_t, v_t)

        depth = self.lookup_depth(depth_image, u_d, v_d, depth_encoding)
        d435_pixel = PointStamped()
        d435_pixel.header.stamp = rospy.Time.now()
        d435_pixel.header.frame_id = "d435_color_image"
        d435_pixel.point.x = float(u_d)
        d435_pixel.point.y = float(v_d)
        d435_pixel.point.z = 0.0 if depth is None else float(depth)
        self.d435_pixel_pub.publish(d435_pixel)

        if depth is None:
            rospy.logwarn_throttle(
                2.0,
                "Invalid depth near D435 pixel (%d, %d); search_radius=%d",
                u_d,
                v_d,
                self.depth_search_radius,
            )
            self.publish_valid(False)
            return

        camera_point = self.pixel_to_camera_point(u_d, v_d, depth, camera_info)
        if camera_point is None:
            rospy.logwarn_throttle(2.0, "Invalid D435 camera intrinsics")
            self.publish_valid(False)
            return

        self.camera_point_pub.publish(camera_point)

        if not self.use_tf:
            channel_point = PointStamped()
            channel_point.header = camera_point.header
            channel_point.point = camera_point.point
            self.channel_position_pub.publish(channel_point)
            self.publish_valid(True)
            return

        try:
            channel_point = self.tf_buffer.transform(
                camera_point, self.target_frame, rospy.Duration(0.05)
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "TF transform failed from %s to %s: %s",
                camera_point.header.frame_id,
                self.target_frame,
                exc,
            )
            self.publish_valid(False)
            return

        self.channel_position_pub.publish(channel_point)
        self.publish_valid(True)

        if self.debug:
            rospy.loginfo_throttle(
                1.0,
                "thermal=(%.1f, %.1f) d435=(%d, %d) depth=%.3f %s=(%.3f, %.3f, %.3f)",
                u_t,
                v_t,
                u_d,
                v_d,
                depth,
                channel_point.header.frame_id,
                channel_point.point.x,
                channel_point.point.y,
                channel_point.point.z,
            )


def main():
    rospy.init_node("thermal_d435_fusion_node", anonymous=False)
    ThermalD435FusionNode()
    rospy.loginfo("thermal_d435_fusion_node started")
    rospy.spin()


if __name__ == "__main__":
    main()
