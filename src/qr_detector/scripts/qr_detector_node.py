#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OpenCV QR Code detector baseline for a ROS1 / RealSense D435 pipeline.

This node is deliberately small and explicit:
  - cv2.QRCodeDetector handles ordinary QR Code detection.
  - Aligned depth near the QR center gives the center point in camera frame.
  - JSON status and a debug image make field tuning quick.

The QR detection section is isolated so a later YOLO fallback can be inserted
without changing the ROS IO, depth, projection, or temporal filtering code.
"""

import json
import os
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class QRDetectorNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )

        self.opencv_num_threads = int(rospy.get_param("~opencv_num_threads", 2))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))
        self.depth_window_size = int(rospy.get_param("~depth_window_size", 11))
        self.min_area = float(rospy.get_param("~min_area", 100.0))
        self.min_side_length = float(rospy.get_param("~min_side_length", 12.0))
        self.max_side_ratio = float(rospy.get_param("~max_side_ratio", 8.0))
        self.max_angle_cos = float(rospy.get_param("~max_angle_cos", 0.90))
        self.qr_eps_x = float(rospy.get_param("~qr_eps_x", 0.25))
        self.qr_eps_y = float(rospy.get_param("~qr_eps_y", 0.25))
        self.depth_min = float(rospy.get_param("~depth_min", 0.15))
        self.depth_max = float(rospy.get_param("~depth_max", 8.0))
        self.preprocess_mode = rospy.get_param("~preprocess_mode", "gray")
        self.upscale_factor = float(rospy.get_param("~upscale_factor", 1.5))
        self.enable_preprocess_fallbacks = bool(
            rospy.get_param("~enable_preprocess_fallbacks", False)
        )
        self.decode_qr_data = bool(rospy.get_param("~decode_qr_data", False))
        self.draw_raw_candidates = bool(rospy.get_param("~draw_raw_candidates", False))
        self.confirm_frames = int(rospy.get_param("~confirm_frames", 3))
        self.lost_hold_time = float(rospy.get_param("~lost_hold_time", 0.3))
        self.ema_alpha = float(rospy.get_param("~ema_alpha", 0.35))

        self.save_failed_frame = bool(rospy.get_param("~save_failed_frame", False))
        self.failed_frame_dir = os.path.expanduser(
            rospy.get_param("~failed_frame_dir", "~/qr_failed_frames")
        )
        self.failed_frame_interval = float(rospy.get_param("~failed_frame_interval", 1.0))

        self.depth_window_size = max(3, self.depth_window_size)
        if self.depth_window_size % 2 == 0:
            self.depth_window_size += 1
        self.confirm_frames = max(1, self.confirm_frames)
        self.ema_alpha = min(max(self.ema_alpha, 0.0), 1.0)
        self.upscale_factor = max(1.0, self.upscale_factor)
        self.configure_opencv_threads()
        self.configure_qr_detector()

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame_id = ""

        self.latest_depth_msg = None
        self.consecutive_detect_count = 0
        self.last_confirmed_result = None
        self.last_confirmed_time = None
        self.filtered_xyz = None
        self.last_failed_save_time = rospy.Time(0)

        if self.save_failed_frame and not os.path.isdir(self.failed_frame_dir):
            os.makedirs(self.failed_frame_dir)

        self.pose_pub = rospy.Publisher(
            "/UAV0/vision/qr_pose_camera", PoseStamped, queue_size=1
        )
        self.detected_pub = rospy.Publisher(
            "/UAV0/vision/qr_detected", String, queue_size=1
        )
        self.debug_pub = rospy.Publisher(
            "/UAV0/vision/qr_debug_image", Image, queue_size=1
        )

        rospy.Subscriber(
            self.camera_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1,
        )
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        rospy.loginfo("qr_detector_node started")
        rospy.loginfo("OpenCV version: %s", cv2.__version__)
        rospy.loginfo("OpenCV threads: %d", cv2.getNumThreads())
        rospy.loginfo("color image: %s", self.image_topic)
        rospy.loginfo("aligned depth: %s", self.depth_topic)
        rospy.loginfo("camera_info: %s", self.camera_info_topic)

    def configure_opencv_threads(self):
        if self.opencv_num_threads > 0:
            cv2.setNumThreads(self.opencv_num_threads)

    def configure_qr_detector(self):
        # Epsilon controls QR finder-pattern scan tolerance in OpenCV. A slightly
        # larger value often helps with small, blurred, or perspective-skewed QR
        # codes, while values that are too large can increase false positives.
        if hasattr(self.qr_detector, "setEpsX"):
            self.qr_detector.setEpsX(self.qr_eps_x)
        if hasattr(self.qr_detector, "setEpsY"):
            self.qr_detector.setEpsY(self.qr_eps_y)

    def camera_info_callback(self, msg):
        if msg.K[0] <= 0.0 or msg.K[4] <= 0.0:
            rospy.logwarn_throttle(2.0, "Invalid camera_info intrinsics")
            return

        self.fx = float(msg.K[0])
        self.fy = float(msg.K[4])
        self.cx = float(msg.K[2])
        self.cy = float(msg.K[5])
        self.camera_frame_id = msg.header.frame_id

    def depth_callback(self, msg):
        self.latest_depth_msg = msg

    def image_callback(self, image_msg):
        processing_start = time.time()
        try:
            color_bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(1.0, "Color image conversion failed: %s", exc)
            self.publish_failure(image_msg.header, None, "cv_bridge_color_error")
            return

        raw_detection = self.detect_qr(color_bgr)
        raw_valid = raw_detection is not None

        if raw_valid:
            self.consecutive_detect_count += 1
        else:
            self.consecutive_detect_count = 0

        result = self.build_result(raw_detection, image_msg.header)
        confirmed = raw_valid and self.consecutive_detect_count >= self.confirm_frames

        if confirmed:
            self.last_confirmed_result = result
            self.last_confirmed_time = image_msg.header.stamp or rospy.Time.now()
            publish_result = result
            publish_detected = True
            reason = "confirmed"
        else:
            publish_result, publish_detected, reason = self.hold_or_fail(
                image_msg.header, raw_detection
            )

        debug_image = self.draw_debug_image(
            color_bgr.copy(), raw_detection, publish_result, publish_detected, reason
        )
        if self.publish_debug_image:
            self.publish_debug(debug_image, image_msg.header)
        self.publish_status(publish_result, publish_detected, reason)

        if publish_detected and publish_result.get("z") is not None:
            self.publish_pose(publish_result, image_msg.header)
        elif not publish_detected:
            self.maybe_save_failed_frame(color_bgr, image_msg.header)

        processing_ms = (time.time() - processing_start) * 1000.0
        rospy.logdebug("QR frame processing time: %.1f ms", processing_ms)

    def detect_qr(self, color_bgr):
        gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)

        # Try the configured preprocessing first, then fall back to a few cheap
        # alternatives. This keeps the OpenCV baseline useful under field light
        # changes without changing the downstream depth/projection pipeline.
        if self.enable_preprocess_fallbacks:
            modes = [self.preprocess_mode, "gray", "equalize", "adaptive"]
        else:
            modes = [self.preprocess_mode]
        tried_modes = []
        data = ""
        points = None
        used_mode = self.preprocess_mode

        for mode in modes:
            if mode in tried_modes:
                continue
            tried_modes.append(mode)
            processed, scale = self.preprocess_for_qr(gray, mode)
            data, points = self.run_qr_detector(processed)
            points = self.normalize_points(points, scale)
            if points is not None:
                used_mode = mode
                break

        if points is None:
            return None

        area = abs(float(cv2.contourArea(points.astype(np.float32))))
        if area < self.min_area:
            return None

        ok, side_px = self.is_reasonable_quad(points)
        if not ok:
            return None

        center_u = float(np.mean(points[:, 0]))
        center_v = float(np.mean(points[:, 1]))

        return {
            "detected": True,
            "data": data or "",
            "points": points,
            "center_u": center_u,
            "center_v": center_v,
            "area": area,
            "side_px": side_px,
            "method": "opencv_qrcode_detector",
            "preprocess": used_mode,
        }

    def preprocess_for_qr(self, gray, mode):
        scale = self.upscale_factor
        work = gray
        if scale > 1.0:
            work = cv2.resize(
                gray,
                None,
                fx=scale,
                fy=scale,
                interpolation=cv2.INTER_CUBIC,
            )

        if mode == "equalize":
            return cv2.equalizeHist(work), scale

        if mode == "adaptive":
            blurred = cv2.GaussianBlur(work, (3, 3), 0)
            return cv2.adaptiveThreshold(
                blurred,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                31,
                3,
            ), scale

        if mode == "sharpen":
            blurred = cv2.GaussianBlur(work, (0, 0), 1.0)
            return cv2.addWeighted(work, 1.6, blurred, -0.6, 0), scale

        return work, scale

    def run_qr_detector(self, image):
        data = ""
        points = None

        # Some embedded OpenCV builds are compiled without QUIRC. In that case
        # detectAndDecode prints noisy warnings and cannot decode payloads, but
        # corner detection still works. The competition only needs target type
        # and position, so detection-only is the default.
        if not self.decode_qr_data:
            try:
                ok, detected_points = self.qr_detector.detect(image)
                if ok:
                    points = detected_points
            except cv2.error as exc:
                rospy.logwarn_throttle(1.0, "QRCodeDetector detect failed: %s", exc)
            return data, points

        try:
            data, points, _ = self.qr_detector.detectAndDecode(image)
        except cv2.error as exc:
            rospy.logwarn_throttle(1.0, "QRCodeDetector detectAndDecode failed: %s", exc)

        # OpenCV can sometimes detect corner points even when decoding fails.
        if points is None or len(points) == 0:
            try:
                ok, detected_points = self.qr_detector.detect(image)
                if ok:
                    points = detected_points
            except cv2.error as exc:
                rospy.logwarn_throttle(1.0, "QRCodeDetector detect failed: %s", exc)

        return data or "", points

    def normalize_points(self, points, scale=1.0):
        if points is None:
            return None

        # OpenCV normally returns an ndarray, while older detector adapters can
        # return the published JSON form [{"u": ..., "v": ...}]. Normalize both
        # forms before reshape so a dictionary-shaped corner list cannot crash
        # the image callback.
        try:
            if isinstance(points, (list, tuple)) and points:
                if isinstance(points[0], dict):
                    points = [[point["u"], point["v"]] for point in points]
            pts = np.asarray(points, dtype=np.float32)
        except (KeyError, TypeError, ValueError):
            return None
        if pts.size < 8:
            return None

        # detectAndDecode returns (1, 4, 2); detect often returns the same shape.
        pts = pts.reshape(-1, 2)
        if pts.shape[0] < 4:
            return None
        pts = pts[:4] / float(scale)
        if not np.all(np.isfinite(pts)):
            return None
        return self.order_quad_points(pts)

    def order_quad_points(self, points):
        pts = np.asarray(points, dtype=np.float32)
        ordered = np.zeros((4, 2), dtype=np.float32)

        sums = pts.sum(axis=1)
        diffs = np.diff(pts, axis=1).reshape(-1)
        ordered[0] = pts[np.argmin(sums)]
        ordered[2] = pts[np.argmax(sums)]
        ordered[1] = pts[np.argmin(diffs)]
        ordered[3] = pts[np.argmax(diffs)]
        return ordered

    def is_reasonable_quad(self, points):
        pts = np.asarray(points, dtype=np.float32)
        if not cv2.isContourConvex(pts):
            return False, 0.0

        sides = []
        for i in range(4):
            p0 = pts[i]
            p1 = pts[(i + 1) % 4]
            sides.append(float(np.linalg.norm(p1 - p0)))

        min_side = min(sides)
        max_side = max(sides)
        if min_side < self.min_side_length:
            return False, min_side
        if max_side / max(min_side, 1e-6) > self.max_side_ratio:
            return False, min_side

        # Reject extremely skinny or self-inconsistent quadrilaterals. Perspective
        # can skew a QR code, so this is intentionally permissive.
        for i in range(4):
            prev_pt = pts[(i - 1) % 4]
            curr_pt = pts[i]
            next_pt = pts[(i + 1) % 4]
            v1 = prev_pt - curr_pt
            v2 = next_pt - curr_pt
            denom = np.linalg.norm(v1) * np.linalg.norm(v2)
            if denom < 1e-6:
                return False, min_side
            angle_cos = abs(float(np.dot(v1, v2) / denom))
            if angle_cos > self.max_angle_cos:
                return False, min_side

        return True, min_side

    def build_result(self, detection, header):
        if detection is None:
            return self.empty_result()

        x = y = z = None
        depth_msg = self.latest_depth_msg
        if depth_msg is not None:
            try:
                depth_raw = self.bridge.imgmsg_to_cv2(
                    depth_msg, desired_encoding="passthrough"
                )
                z = self.depth_from_window(
                    depth_raw,
                    depth_msg.encoding,
                    detection["center_u"],
                    detection["center_v"],
                )
            except CvBridgeError as exc:
                rospy.logwarn_throttle(1.0, "Depth image conversion failed: %s", exc)

        if z is not None and self.has_camera_info():
            x, y, z = self.back_project(detection["center_u"], detection["center_v"], z)
            x, y, z = self.filter_xyz(x, y, z)
        elif z is not None and not self.has_camera_info():
            rospy.logwarn_throttle(2.0, "Waiting for camera_info intrinsics")

        return {
            "detected": True,
            "data": detection["data"],
            "points": self.points_to_list(detection["points"]),
            "center_u": detection["center_u"],
            "center_v": detection["center_v"],
            "x": x,
            "y": y,
            "z": z,
            "method": detection["method"],
            "area": detection["area"],
            "side_px": detection.get("side_px"),
            "preprocess": detection.get("preprocess"),
            "stamp": header.stamp.to_sec() if header.stamp else None,
        }

    def empty_result(self):
        return {
            "detected": False,
            "data": "",
            "points": [],
            "center_u": None,
            "center_v": None,
            "x": None,
            "y": None,
            "z": None,
            "method": "opencv_qrcode_detector",
        }

    def depth_from_window(self, depth_raw, depth_encoding, center_u, center_v):
        if depth_raw is None:
            return None

        h, w = depth_raw.shape[:2]
        u = int(round(center_u))
        v = int(round(center_v))
        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        half = self.depth_window_size // 2
        x0 = max(0, u - half)
        x1 = min(w, u + half + 1)
        y0 = max(0, v - half)
        y1 = min(h, v + half + 1)
        roi = depth_raw[y0:y1, x0:x1]

        depth_m = self.depth_samples_to_meters(roi.reshape(-1), depth_encoding)
        valid = np.isfinite(depth_m)
        valid &= depth_m >= self.depth_min
        valid &= depth_m <= self.depth_max

        valid_values = depth_m[valid]
        if valid_values.size == 0:
            return None
        return float(np.median(valid_values))

    def depth_samples_to_meters(self, samples, depth_encoding):
        arr = np.asarray(samples)
        encoding = (depth_encoding or "").upper()

        if "16UC1" in encoding or arr.dtype == np.uint16:
            return arr.astype(np.float32) / 1000.0
        if "32FC1" in encoding or arr.dtype == np.float32 or arr.dtype == np.float64:
            return arr.astype(np.float32)

        # RealSense depth is normally 16UC1 in millimeters or 32FC1 in meters.
        if np.issubdtype(arr.dtype, np.integer):
            return arr.astype(np.float32) / 1000.0
        return arr.astype(np.float32)

    def has_camera_info(self):
        return (
            self.fx is not None
            and self.fy is not None
            and self.cx is not None
            and self.cy is not None
        )

    def back_project(self, center_u, center_v, depth_m):
        x = (float(center_u) - self.cx) * depth_m / self.fx
        y = (float(center_v) - self.cy) * depth_m / self.fy
        z = depth_m
        return float(x), float(y), float(z)

    def filter_xyz(self, x, y, z):
        current = np.array([x, y, z], dtype=np.float32)
        if self.filtered_xyz is None:
            self.filtered_xyz = current
        else:
            self.filtered_xyz = (
                self.ema_alpha * current + (1.0 - self.ema_alpha) * self.filtered_xyz
            )
        return [float(v) for v in self.filtered_xyz]

    def hold_or_fail(self, header, raw_detection):
        now = header.stamp if header.stamp else rospy.Time.now()
        if (
            self.last_confirmed_result is not None
            and self.last_confirmed_time is not None
            and (now - self.last_confirmed_time).to_sec() <= self.lost_hold_time
        ):
            held = dict(self.last_confirmed_result)
            # Keep the previous geometry for the debug image only. A held
            # result is not a fresh observation and must not publish a pose.
            held["detected"] = False
            held["held"] = True
            return held, False, "held_recent_result"

        if raw_detection is not None:
            return self.empty_result(), False, "not_confirmed_yet"

        self.filtered_xyz = None
        return self.empty_result(), False, "no_qr"

    def publish_pose(self, result, header):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = header.stamp
        pose_msg.header.frame_id = self.camera_frame_id or header.frame_id
        pose_msg.pose.position.x = float(result["x"])
        pose_msg.pose.position.y = float(result["y"])
        pose_msg.pose.position.z = float(result["z"])
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

    def publish_status(self, result, detected, reason):
        status = {
            "detected": bool(detected),
            "held": bool(result.get("held", False)),
            "data": result.get("data", ""),
            "points": result.get("points", []),
            "center_u": self.round_or_none(result.get("center_u"), 2),
            "center_v": self.round_or_none(result.get("center_v"), 2),
            "x": self.round_or_none(result.get("x"), 4),
            "y": self.round_or_none(result.get("y"), 4),
            "z": self.round_or_none(result.get("z"), 4),
            "method": result.get("method", "opencv_qrcode_detector"),
            "area": self.round_or_none(result.get("area"), 1),
            "side_px": self.round_or_none(result.get("side_px"), 1),
            "preprocess": result.get("preprocess"),
            "reason": reason,
        }
        self.detected_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def publish_failure(self, header, color_bgr, reason):
        result = self.empty_result()
        self.publish_status(result, False, reason)
        if color_bgr is not None:
            debug_image = self.draw_debug_image(color_bgr.copy(), None, result, False, reason)
            self.publish_debug(debug_image, header)

    def publish_debug(self, debug_image, header):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(1.0, "Failed to publish debug image: %s", exc)

    def draw_debug_image(self, image, raw_detection, result, detected, reason):
        draw_points = None
        draw_center = None

        if raw_detection is not None and (detected or self.draw_raw_candidates):
            draw_points = raw_detection["points"]
            draw_center = (
                raw_detection["center_u"],
                raw_detection["center_v"],
            )
        elif reason == "held_recent_result":
            # Keep the last confirmed geometry visible during the short lost
            # hold interval. This prevents the debug box from disappearing
            # even though the published detection result is still held.
            held_points = self.points_to_array(result.get("points", []))
            if held_points is not None:
                draw_points = held_points
                draw_center = (result.get("center_u"), result.get("center_v"))

        if draw_points is not None:
            pts = self.points_to_array(draw_points)
            if pts is not None:
                box_color = (0, 255, 0) if detected else (0, 255, 255)
                cv2.polylines(
                    image, [pts.astype(np.int32)], True, box_color, 2, cv2.LINE_AA
                )
                if draw_center[0] is not None and draw_center[1] is not None:
                    center = (
                        int(round(draw_center[0])),
                        int(round(draw_center[1])),
                    )
                    cv2.circle(image, center, 5, (0, 0, 255), -1)

        lines = ["detected: {}".format(str(bool(detected)).lower())]
        lines.append("reason: {}".format(reason))

        data = result.get("data", "") if detected else ""
        if data:
            lines.append("data: {}".format(data[:60]))
        elif raw_detection is not None:
            lines.append("data: <empty>")

        if raw_detection is not None:
            lines.append(
                "area: {:.0f} side: {:.1f}px {}".format(
                    raw_detection.get("area", 0.0),
                    raw_detection.get("side_px", 0.0),
                    raw_detection.get("preprocess", ""),
                )
            )

        center_u = result.get("center_u")
        center_v = result.get("center_v")
        if center_u is not None and center_v is not None:
            lines.append("center: ({:.1f}, {:.1f})".format(center_u, center_v))

        x = result.get("x")
        y = result.get("y")
        z = result.get("z")
        if x is not None and y is not None and z is not None:
            lines.append("camera xyz: {:.3f}, {:.3f}, {:.3f} m".format(x, y, z))
        elif raw_detection is not None:
            lines.append("camera xyz: unavailable")

        self.draw_text_block(image, lines)
        return image

    def draw_text_block(self, image, lines):
        y = 26
        for line in lines:
            cv2.putText(
                image,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                image,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            y += 24

    def maybe_save_failed_frame(self, color_bgr, header):
        if not self.save_failed_frame:
            return

        now = header.stamp if header.stamp else rospy.Time.now()
        if (now - self.last_failed_save_time).to_sec() < self.failed_frame_interval:
            return

        self.last_failed_save_time = now
        stamp = now.to_nsec() if now else int(time.time() * 1e9)
        filename = os.path.join(self.failed_frame_dir, "qr_failed_{}.jpg".format(stamp))
        ok = cv2.imwrite(filename, color_bgr)
        if not ok:
            rospy.logwarn_throttle(2.0, "Failed to save failed QR frame: %s", filename)

    def points_to_list(self, points):
        return [
            {"u": round(float(point[0]), 2), "v": round(float(point[1]), 2)}
            for point in points
        ]

    def points_to_array(self, points):
        """Convert raw numpy points or published {u, v} points to a quad."""
        if points is None:
            return None

        try:
            if isinstance(points, (list, tuple)) and points:
                if isinstance(points[0], dict):
                    points = [[point["u"], point["v"]] for point in points]
            points = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        except (KeyError, TypeError, ValueError):
            return None

        if points.shape[0] < 4:
            return None
        points = points[:4]
        if not np.all(np.isfinite(points)):
            return None
        return points

    def round_or_none(self, value, digits):
        if value is None:
            return None
        return round(float(value), digits)


def main():
    rospy.init_node("qr_detector_node")
    QRDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
