#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OpenCV QR Code detector baseline for a ROS1 / RealSense D435 pipeline.

This node is deliberately small and explicit:
  - cv2.QRCodeDetector supplies immediate geometric candidates. One successful
    decode authenticates a short, spatially continuous track so motion blur in
    later frames does not reset confirmation unnecessarily.
  - Aligned depth near the QR center gives the center point in camera frame.
  - JSON status and a debug image make field tuning quick.

The QR detection section is isolated so a later YOLO fallback can be inserted
without changing the ROS IO, depth, projection, or temporal filtering code.
The decoded payload can be displayed/reported without changing planner inputs.
"""

import ctypes
import ctypes.util
import json
import math
import os
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class ZBarQRDecoder(object):
    """Minimal libzbar QR decoder used when OpenCV has no QUIRC support."""

    ZBAR_NONE = 0
    ZBAR_QRCODE = 64
    ZBAR_CFG_ENABLE = 0
    Y800 = ord("Y") | (ord("8") << 8) | (ord("0") << 16) | (ord("0") << 24)

    def __init__(self):
        self.library_name = ctypes.util.find_library("zbar")
        self.library = None
        self.scanner = None
        if not self.library_name:
            return

        try:
            self.library = ctypes.CDLL(self.library_name)
            self._configure_api()
            self.scanner = self.library.zbar_image_scanner_create()
            if not self.scanner:
                self.library = None
                return
            self.library.zbar_image_scanner_set_config(
                self.scanner, self.ZBAR_NONE, self.ZBAR_CFG_ENABLE, 0
            )
            self.library.zbar_image_scanner_set_config(
                self.scanner, self.ZBAR_QRCODE, self.ZBAR_CFG_ENABLE, 1
            )
        except (AttributeError, OSError):
            self.close()
            self.library = None

    @property
    def available(self):
        return self.library is not None and self.scanner is not None

    def _configure_api(self):
        pointer = ctypes.c_void_p
        self.library.zbar_image_scanner_create.restype = pointer
        self.library.zbar_image_scanner_destroy.argtypes = [pointer]
        self.library.zbar_image_scanner_set_config.argtypes = [
            pointer,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
        ]
        self.library.zbar_image_create.restype = pointer
        self.library.zbar_image_destroy.argtypes = [pointer]
        self.library.zbar_image_set_format.argtypes = [pointer, ctypes.c_ulong]
        self.library.zbar_image_set_size.argtypes = [
            pointer,
            ctypes.c_uint,
            ctypes.c_uint,
        ]
        self.library.zbar_image_set_data.argtypes = [
            pointer,
            pointer,
            ctypes.c_ulong,
            pointer,
        ]
        self.library.zbar_scan_image.argtypes = [pointer, pointer]
        self.library.zbar_scan_image.restype = ctypes.c_int
        self.library.zbar_image_first_symbol.argtypes = [pointer]
        self.library.zbar_image_first_symbol.restype = pointer
        self.library.zbar_symbol_next.argtypes = [pointer]
        self.library.zbar_symbol_next.restype = pointer
        self.library.zbar_symbol_get_type.argtypes = [pointer]
        self.library.zbar_symbol_get_type.restype = ctypes.c_int
        self.library.zbar_symbol_get_data.argtypes = [pointer]
        self.library.zbar_symbol_get_data.restype = pointer
        self.library.zbar_symbol_get_data_length.argtypes = [pointer]
        self.library.zbar_symbol_get_data_length.restype = ctypes.c_uint
        self.library.zbar_symbol_get_loc_size.argtypes = [pointer]
        self.library.zbar_symbol_get_loc_size.restype = ctypes.c_uint
        self.library.zbar_symbol_get_loc_x.argtypes = [pointer, ctypes.c_uint]
        self.library.zbar_symbol_get_loc_x.restype = ctypes.c_int
        self.library.zbar_symbol_get_loc_y.argtypes = [pointer, ctypes.c_uint]
        self.library.zbar_symbol_get_loc_y.restype = ctypes.c_int

    def decode(self, image):
        if not self.available or image is None:
            return "", None

        if image.ndim == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        gray = np.ascontiguousarray(gray, dtype=np.uint8)
        zbar_image = self.library.zbar_image_create()
        if not zbar_image:
            return "", None

        try:
            height, width = gray.shape[:2]
            self.library.zbar_image_set_format(zbar_image, self.Y800)
            self.library.zbar_image_set_size(zbar_image, width, height)
            # gray remains alive until scanning and symbol extraction finish.
            self.library.zbar_image_set_data(
                zbar_image, gray.ctypes.data, gray.nbytes, None
            )
            if self.library.zbar_scan_image(self.scanner, zbar_image) <= 0:
                return "", None

            symbol = self.library.zbar_image_first_symbol(zbar_image)
            while symbol:
                if self.library.zbar_symbol_get_type(symbol) == self.ZBAR_QRCODE:
                    length = self.library.zbar_symbol_get_data_length(symbol)
                    data_ptr = self.library.zbar_symbol_get_data(symbol)
                    payload = (
                        ctypes.string_at(data_ptr, length).decode(
                            "utf-8", errors="replace"
                        )
                        if data_ptr and length
                        else ""
                    )
                    point_count = self.library.zbar_symbol_get_loc_size(symbol)
                    points = [
                        [
                            self.library.zbar_symbol_get_loc_x(symbol, index),
                            self.library.zbar_symbol_get_loc_y(symbol, index),
                        ]
                        for index in range(point_count)
                    ]
                    if payload:
                        return payload, self._quad_from_points(points)
                symbol = self.library.zbar_symbol_next(symbol)
            return "", None
        finally:
            self.library.zbar_image_destroy(zbar_image)

    @staticmethod
    def _quad_from_points(points):
        if len(points) < 4:
            return None
        points = np.asarray(points, dtype=np.float32)
        if len(points) == 4:
            return points.reshape(1, 4, 2)
        rectangle = cv2.minAreaRect(points)
        return cv2.boxPoints(rectangle).reshape(1, 4, 2)

    def close(self):
        if self.library is not None and self.scanner is not None:
            self.library.zbar_image_scanner_destroy(self.scanner)
        self.scanner = None

    def __del__(self):
        self.close()


class QRDetectorNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()
        self.zbar_decoder = ZBarQRDecoder()

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
        self.max_quad_area_ratio = float(
            rospy.get_param("~max_quad_area_ratio", 0.35)
        )
        self.max_quad_width_ratio = float(
            rospy.get_param("~max_quad_width_ratio", 0.90)
        )
        self.max_quad_height_ratio = float(
            rospy.get_param("~max_quad_height_ratio", 0.90)
        )
        self.qr_eps_x = float(rospy.get_param("~qr_eps_x", 0.25))
        self.qr_eps_y = float(rospy.get_param("~qr_eps_y", 0.25))
        self.depth_min = float(rospy.get_param("~depth_min", 0.15))
        self.depth_max = float(rospy.get_param("~depth_max", 8.0))
        self.preprocess_mode = rospy.get_param("~preprocess_mode", "gray")
        self.upscale_factor = float(rospy.get_param("~upscale_factor", 1.0))
        self.enable_preprocess_fallbacks = bool(
            rospy.get_param("~enable_preprocess_fallbacks", False)
        )
        self.decode_qr_data = bool(rospy.get_param("~decode_qr_data", True))
        # The QR payload is not part of the competition result, but decoding
        # is used internally as a strong authenticity gate.  Raw corner
        # candidates remain available for local RViz observation.
        self.require_decode_for_confirmation = bool(
            rospy.get_param("~require_decode_for_confirmation", True)
        )
        # A single successful decode authenticates the same nearby QR track
        # for a short period. Subsequent frames still need valid geometry and
        # depth, but do not all need to decode under flight vibration.
        self.decode_verification_hold_seconds = max(
            0.0,
            float(rospy.get_param("~decode_verification_hold_seconds", 2.0)),
        )
        self.decode_verification_max_center_shift_px = max(
            1.0,
            float(
                rospy.get_param(
                    "~decode_verification_max_center_shift_px", 120.0
                )
            ),
        )
        # RealSense auto-exposure needs a short period after the first color
        # frame.  Do not let a dark startup frame become a confirmed/uploaded
        # QR target; raw candidates remain available locally during warm-up.
        self.startup_warmup_seconds = max(
            0.0, float(rospy.get_param("~startup_warmup_seconds", 5.0))
        )
        self.draw_raw_candidates = bool(rospy.get_param("~draw_raw_candidates", False))
        self.confirm_frames = int(rospy.get_param("~confirm_frames", 3))
        self.lost_hold_time = float(rospy.get_param("~lost_hold_time", 0.3))
        self.ema_alpha = float(rospy.get_param("~ema_alpha", 0.35))

        self.depth_corner_window_size = int(
            rospy.get_param("~depth_corner_window_size", 7)
        )
        self.min_depth_valid_corners = int(
            rospy.get_param("~min_depth_valid_corners", 3)
        )
        self.min_depth_valid_ratio = float(
            rospy.get_param("~min_depth_valid_ratio", 0.55)
        )
        self.max_corner_depth_std = float(
            rospy.get_param("~max_corner_depth_std", 0.25)
        )
        self.max_corner_depth_range = float(
            rospy.get_param("~max_corner_depth_range", 0.60)
        )

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
        self.first_image_wall_time = None
        self.startup_warmup_finished = self.startup_warmup_seconds <= 0.0
        self.consecutive_detect_count = 0
        self.consecutive_valid_count = 0
        self.last_decode_verification_time = None
        self.last_decode_center = None
        self.last_decoded_data = ""
        self.last_confirmed_result = None
        self.last_confirmed_time = None
        self.filtered_xyz = None
        self.last_failed_save_time = rospy.Time(0)

        if self.save_failed_frame and not os.path.isdir(self.failed_frame_dir):
            os.makedirs(self.failed_frame_dir)

        self.pose_pub = rospy.Publisher(
            "/UAV0/vision/qr_pose_camera", PoseStamped, queue_size=1
        )
        # 候选只供机载 target_reporting/RViz 观察，不直接连接远程发送器；
        # target_reporting 的 send_candidate_observations 默认仍为 false。
        self.candidate_pose_pub = rospy.Publisher(
            rospy.get_param(
                "~candidate_pose_topic",
                "/UAV0/vision/qr_candidate_pose_camera",
            ),
            PoseStamped,
            queue_size=1,
        )
        self.detected_pub = rospy.Publisher(
            "/UAV0/vision/qr_detected", String, queue_size=1
        )
        self.candidate_detected_pub = rospy.Publisher(
            rospy.get_param(
                "~candidate_detected_topic",
                "/UAV0/vision/qr_candidate_detected",
            ),
            String,
            queue_size=1,
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
        if self.zbar_decoder.available:
            rospy.loginfo("QR decode fallback: %s", self.zbar_decoder.library_name)
        else:
            rospy.logwarn(
                "libzbar is unavailable; this OpenCV build may detect but not decode QR"
            )
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
        startup_warmup = self.startup_warmup_active()
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
        validation_valid = raw_valid and bool(result.get("confirmable", False))

        if validation_valid and not startup_warmup:
            self.consecutive_valid_count += 1
        else:
            self.consecutive_valid_count = 0

        confirmed = (
            not startup_warmup
            and raw_valid
            and bool(result.get("confirmable", False))
            and self.consecutive_valid_count >= self.confirm_frames
        )

        # 原始角点和深度一出现就发布本机候选，供 target_reporting 在 RViz
        # 显示观察位姿；候选是否可确认由 validated/confirmable 门控，稳定
        # 结果仍由下面的 confirmed 分支负责正式上报。
        if raw_valid and result.get("z") is not None:
            self.publish_candidate(result, image_msg.header, confirmed)

        if startup_warmup:
            # Keep candidate geometry local for RViz, but do not publish a
            # stable result or feed target_reporting during exposure warm-up.
            publish_result = self.empty_result()
            publish_detected = False
            reason = "startup_warmup"
        elif confirmed:
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
        self.publish_status(publish_result, publish_detected, reason, image_msg.header)

        if publish_detected and publish_result.get("z") is not None:
            self.publish_pose(publish_result, image_msg.header)
        elif not publish_detected:
            self.maybe_save_failed_frame(color_bgr, image_msg.header)

        processing_ms = (time.time() - processing_start) * 1000.0
        rospy.logdebug("QR frame processing time: %.1f ms", processing_ms)

    def startup_warmup_active(self, now=None):
        """Return whether the camera is still in first-frame exposure warm-up."""
        if self.startup_warmup_seconds <= 0.0:
            return False
        if now is None:
            now = time.monotonic()
        if self.first_image_wall_time is None:
            self.first_image_wall_time = float(now)
            rospy.loginfo(
                "QR detector exposure warm-up: %.1f s; candidates stay local",
                self.startup_warmup_seconds,
            )
        elapsed = float(now) - float(self.first_image_wall_time)
        if elapsed < self.startup_warmup_seconds:
            return True
        if not self.startup_warmup_finished:
            self.startup_warmup_finished = True
            rospy.loginfo("QR detector exposure warm-up finished")
        return False

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
            detection = self.run_qr_detector(processed)
            points = self.normalize_points(detection.get("points"), scale)
            decoded_points = self.normalize_points(
                detection.get("decoded_points"), scale
            )
            if points is not None:
                used_mode = mode
                break

        if points is None:
            return None

        area = abs(float(cv2.contourArea(points.astype(np.float32))))
        if area < self.min_area:
            return None

        ok, side_px = self.is_reasonable_quad(
            points, image_w=color_bgr.shape[1], image_h=color_bgr.shape[0]
        )
        if not ok:
            return None

        # Prefer the decoder's own corners after validation.  For an
        # unvalidated raw candidate, retain detect() corners so the candidate
        # can still be shown locally in RViz without entering the report chain.
        if detection.get("decoded_valid") and decoded_points is not None:
            points = decoded_points
            area = abs(float(cv2.contourArea(points.astype(np.float32))))
            ok, side_px = self.is_reasonable_quad(
                points, image_w=color_bgr.shape[1], image_h=color_bgr.shape[0]
            )
            if not ok:
                return None

        center_u = float(np.mean(points[:, 0]))
        center_v = float(np.mean(points[:, 1]))

        validated, validation_reason, verified_data = self.apply_decode_verification(
            detection, center_u, center_v
        )
        if not self.require_decode_for_confirmation:
            # This compatibility mode keeps the old detection-only behavior,
            # while still applying the stricter quad geometry checks above.
            validated = True
            validation_reason = "geometry_only"

        return {
            "detected": True,
            # Payload publication is independent from the authenticity gate.
            "data": verified_data if self.decode_qr_data else "",
            "points": points,
            "center_u": center_u,
            "center_v": center_v,
            "area": area,
            "side_px": side_px,
            "method": "opencv_qrcode_detector",
            "preprocess": used_mode,
            "validated": validated,
            "validation_reason": validation_reason,
        }

    def apply_decode_verification(
        self, detection, center_u, center_v, now=None
    ):
        """Authenticate one spatially continuous QR track with one decode."""
        decoded_data = str(detection.get("decoded_data", "") or "")
        decoded_valid = bool(detection.get("decoded_valid", False))
        current_center = (float(center_u), float(center_v))
        current_time = time.monotonic() if now is None else float(now)

        if decoded_valid:
            self.last_decode_verification_time = current_time
            self.last_decode_center = current_center
            self.last_decoded_data = decoded_data
            return True, "decoded", decoded_data

        verified_time = getattr(self, "last_decode_verification_time", None)
        verified_center = getattr(self, "last_decode_center", None)
        if verified_time is None or verified_center is None:
            return False, "decode_not_verified", ""

        age = current_time - float(verified_time)
        max_age = float(getattr(self, "decode_verification_hold_seconds", 0.0))
        shift_limit = float(
            getattr(self, "decode_verification_max_center_shift_px", 1.0)
        )
        center_shift = math.hypot(
            current_center[0] - float(verified_center[0]),
            current_center[1] - float(verified_center[1]),
        )
        if 0.0 <= age <= max_age and center_shift <= shift_limit:
            return (
                True,
                "recent_decode",
                str(getattr(self, "last_decoded_data", "") or ""),
            )
        return False, "decode_not_verified", ""

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
        detected_points = None
        decoded_data = ""
        decoded_points = None

        # Reuse detectAndDecode() corners as the geometric candidate so OpenCV
        # scans the image only once when decoding is enabled.
        if self.require_decode_for_confirmation or self.decode_qr_data:
            try:
                decoded_data, decoded_points, _ = self.qr_detector.detectAndDecode(
                    image
                )
                detected_points = decoded_points
            except cv2.error as exc:
                rospy.logwarn_throttle(
                    1.0, "QRCodeDetector detectAndDecode failed: %s", exc
                )

            # Ubuntu's OpenCV 4.2 package can be built without QUIRC: it still
            # detects QR corners, but always returns an empty payload.  Decode
            # the same image with the already-installed libzbar in that case.
            zbar_decoder = getattr(self, "zbar_decoder", None)
            if not decoded_data and zbar_decoder is not None:
                try:
                    decoded_data, zbar_points = zbar_decoder.decode(image)
                    if decoded_data and zbar_points is not None:
                        decoded_points = zbar_points
                except (AttributeError, ValueError, ctypes.Error) as exc:
                    rospy.logwarn_throttle(1.0, "libzbar QR decode failed: %s", exc)
        else:
            try:
                ok, candidate_points = self.qr_detector.detect(image)
                if ok:
                    detected_points = candidate_points
            except cv2.error as exc:
                rospy.logwarn_throttle(1.0, "QRCodeDetector detect failed: %s", exc)

        if detected_points is None and decoded_points is not None:
            detected_points = decoded_points

        return {
            "points": detected_points,
            "decoded_points": decoded_points,
            "decoded_data": decoded_data or "",
            "decoded_valid": bool(decoded_data and decoded_points is not None),
        }

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

    def is_reasonable_quad(self, points, image_w=None, image_h=None):
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

        if image_w is not None and image_h is not None:
            image_area = float(max(int(image_w) * int(image_h), 1))
            quad_area = abs(float(cv2.contourArea(pts)))
            x, y, width, height = cv2.boundingRect(pts)
            if quad_area / image_area > self.max_quad_area_ratio:
                return False, min_side
            if float(width) / float(max(image_w, 1)) > self.max_quad_width_ratio:
                return False, min_side
            if float(height) / float(max(image_h, 1)) > self.max_quad_height_ratio:
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
        depth_quality = {
            "validated": False,
            "valid_corners": 0,
            "valid_ratio": 0.0,
            "std": None,
            "range": None,
        }
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
                depth_quality = self.depth_quality_from_quad(
                    depth_raw,
                    depth_msg.encoding,
                    detection["points"],
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
            "validated": bool(detection.get("validated", False)),
            "validation_reason": detection.get("validation_reason", ""),
            "depth_validated": bool(depth_quality["validated"]),
            "depth_valid_corners": int(depth_quality["valid_corners"]),
            "depth_valid_ratio": round(float(depth_quality["valid_ratio"]), 3),
            "depth_corner_std": (
                round(float(depth_quality["std"]), 4)
                if depth_quality["std"] is not None
                else None
            ),
            "depth_corner_range": (
                round(float(depth_quality["range"]), 4)
                if depth_quality["range"] is not None
                else None
            ),
            "confirmable": bool(
                detection.get("validated", False)
                and depth_quality["validated"]
            ),
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

    def depth_quality_from_quad(
        self, depth_raw, depth_encoding, points, center_u, center_v
    ):
        """Check that the QR corners lie on one reliable depth surface.

        A false quadrilateral assembled from a net, floor, or background often
        spans several depth layers.  The center depth alone cannot detect that
        case, so confirmation uses the center and all four corners while the
        center value remains available for local candidate visualization.
        """
        result = {
            "validated": False,
            "valid_corners": 0,
            "valid_ratio": 0.0,
            "std": None,
            "range": None,
        }
        if depth_raw is None:
            return result

        try:
            quad = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        except (TypeError, ValueError):
            return result
        if quad.shape[0] < 4:
            return result

        samples = [(float(center_u), float(center_v))]
        samples.extend((float(point[0]), float(point[1])) for point in quad[:4])
        center_values = []
        corner_values = []
        ratios = []
        for index, (u, v) in enumerate(samples):
            stats = self.depth_stats_from_window(
                depth_raw,
                depth_encoding,
                u,
                v,
                self.depth_corner_window_size,
            )
            if stats[0] is not None:
                if index == 0:
                    center_values.append(float(stats[0]))
                else:
                    corner_values.append(float(stats[0]))
            ratios.append(float(stats[1]))

        result["valid_corners"] = len(corner_values)
        result["valid_ratio"] = float(np.mean(ratios)) if ratios else 0.0
        values = center_values + corner_values
        if not values:
            return result

        values_array = np.asarray(values, dtype=np.float32)
        result["std"] = float(np.std(values_array))
        result["range"] = float(np.max(values_array) - np.min(values_array))
        result["validated"] = bool(
            result["valid_corners"] >= self.min_depth_valid_corners
            and result["valid_ratio"] >= self.min_depth_valid_ratio
            and result["std"] <= self.max_corner_depth_std
            and result["range"] <= self.max_corner_depth_range
        )
        return result

    def depth_from_window(self, depth_raw, depth_encoding, center_u, center_v):
        return self.depth_stats_from_window(
            depth_raw, depth_encoding, center_u, center_v, self.depth_window_size
        )[0]

    def depth_stats_from_window(
        self, depth_raw, depth_encoding, center_u, center_v, window_size
    ):
        if depth_raw is None:
            return None, 0.0, None

        h, w = depth_raw.shape[:2]
        u = int(round(center_u))
        v = int(round(center_v))
        if u < 0 or u >= w or v < 0 or v >= h:
            return None, 0.0, None

        half = max(1, int(window_size) // 2)
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
        valid_ratio = float(valid_values.size) / float(max(depth_m.size, 1))
        if valid_values.size == 0:
            return None, valid_ratio, None
        return (
            float(np.median(valid_values)),
            valid_ratio,
            float(np.std(valid_values)),
        )

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

    def publish_candidate(self, result, header, source_confirmed):
        confirmable = bool(source_confirmed and result.get("confirmable", False))
        pose_msg = PoseStamped()
        pose_msg.header.stamp = header.stamp
        pose_msg.header.frame_id = self.camera_frame_id or header.frame_id
        pose_msg.pose.position.x = float(result["x"])
        pose_msg.pose.position.y = float(result["y"])
        pose_msg.pose.position.z = float(result["z"])
        pose_msg.pose.orientation.w = 1.0
        self.candidate_pose_pub.publish(pose_msg)

        status = {
            "detected": True,
            "candidate": True,
            "stable": bool(source_confirmed),
            "validated": bool(result.get("validated", False)),
            "confirmable": confirmable,
            "stable_count": int(self.consecutive_valid_count),
            "stable_window": int(self.confirm_frames),
            "stamp": header.stamp.to_sec() if header.stamp else None,
            "held": False,
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
            "validation_reason": result.get("validation_reason", ""),
            "depth_validated": bool(result.get("depth_validated", False)),
            "reason": (
                "stable_candidate"
                if source_confirmed
                else (
                    "validated_candidate"
                    if result.get("validated", False)
                    else "raw_candidate_waiting_validation"
                )
            ),
        }
        self.candidate_detected_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False))
        )

    def publish_status(self, result, detected, reason, header=None):
        status = {
            "detected": bool(detected),
            "candidate": bool(detected),
            "stable": bool(detected),
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
            "validated": bool(result.get("validated", False)),
            "confirmable": bool(result.get("confirmable", False)),
            "stable_count": int(self.consecutive_valid_count if detected else 0),
            "stable_window": int(self.confirm_frames),
            "stamp": (
                header.stamp.to_sec()
                if header is not None and header.stamp
                else None
            ),
            "depth_validated": bool(result.get("depth_validated", False)),
            "reason": reason,
        }
        self.detected_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def publish_failure(self, header, color_bgr, reason):
        result = self.empty_result()
        self.publish_status(result, False, reason, header)
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

        candidate = raw_detection is not None
        stable = bool(detected and result.get("confirmable", False))
        lines = [
            "candidate: {}  stable: {}".format(
                str(candidate).lower(), str(stable).lower()
            ),
            "detected: {}".format(str(bool(detected)).lower()),
        ]
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
