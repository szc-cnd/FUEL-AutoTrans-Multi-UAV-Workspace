#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS1 Noetic color tag detector for Intel RealSense D435.

Core idea:
  - Segment several colors in HSV space.
  - Clean masks with morphology.
  - Find contour candidates with only weak geometry constraints.
  - Read a small depth ROI around the color centroid, reject invalid depth,
    and use the median depth.
  - Back-project the centroid to camera coordinates using camera_info.
  - Use a short multi-frame history before publishing a stable target point.

This node intentionally does not use PnP, YOLO, or any known physical tag size.
"""

import json
import math
from collections import deque

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


DEFAULT_COLORS = {
    "red": {
        # OpenCV HSV uses H in [0, 179]. Red usually wraps around 0, so two
        # ranges are used. Tune S/V first if lighting changes at the venue.
        "ranges": [
            {"lower": [0, 135, 60], "upper": [10, 255, 255]},
            {"lower": [170, 115, 60], "upper": [179, 255, 255]},
        ],
        "exclude_ranges": [
            {"lower": [0, 20, 70], "upper": [25, 135, 255]},
        ],
        "min_mean_saturation": 150,
        "draw_bgr": [0, 0, 255],
    },
    "yellow": {
        "ranges": [{"lower": [18, 80, 80], "upper": [38, 255, 255]}],
        "min_mean_saturation": 90,
        "min_mean_value": 80,
        "draw_bgr": [0, 255, 255],
    },
    "green": {
        "ranges": [{"lower": [38, 35, 105], "upper": [84, 255, 255]}],
        "max_mean_hue": 82,
        "min_mean_saturation": 45,
        "min_mean_value": 115,
        "area_max_ratio": 0.06,
        "max_bbox_width_ratio": 0.35,
        "max_bbox_height_ratio": 0.35,
        "draw_bgr": [0, 255, 0],
    },
    "blue": {
        "ranges": [{"lower": [86, 35, 105], "upper": [128, 255, 255]}],
        "min_mean_hue": 88,
        "min_mean_saturation": 45,
        "min_mean_value": 115,
        "area_max_ratio": 0.06,
        "max_bbox_width_ratio": 0.35,
        "max_bbox_height_ratio": 0.35,
        "draw_bgr": [255, 0, 0],
    },
}


class ColorTagDetector(object):
    def __init__(self):
        self.bridge = CvBridge()

        # Input topics. Defaults match realsense2_camera with aligned depth.
        self.color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )

        # HSV thresholds and all important tuning parameters are loaded from YAML.
        self.colors = rospy.get_param("~colors", DEFAULT_COLORS)
        self.target_color = str(rospy.get_param("~target_color", "")).strip().lower()
        self.enabled_colors = rospy.get_param("~enabled_colors", [])
        self.debug_draw_mode = int(rospy.get_param("~debug_draw_mode", 1))
        self.max_debug_candidates = int(rospy.get_param("~max_debug_candidates", 3))
        self.max_contours_per_color = int(rospy.get_param("~max_contours_per_color", 6))
        self.colors = self.filter_enabled_colors(self.colors)
        self.area_min = float(rospy.get_param("~area_min", 1200.0))
        self.area_max_ratio = float(rospy.get_param("~area_max_ratio", 0.12))
        self.aspect_ratio_min = float(rospy.get_param("~aspect_ratio_min", 0.35))
        self.aspect_ratio_max = float(rospy.get_param("~aspect_ratio_max", 3.0))
        self.fill_ratio_min = float(rospy.get_param("~fill_ratio_min", 0.45))
        self.min_extent = float(rospy.get_param("~min_extent", self.fill_ratio_min))
        self.min_solidity = float(rospy.get_param("~min_solidity", 0.45))
        self.max_bbox_width_ratio = float(rospy.get_param("~max_bbox_width_ratio", 0.55))
        self.max_bbox_height_ratio = float(rospy.get_param("~max_bbox_height_ratio", 0.55))
        self.use_center_roi = bool(rospy.get_param("~use_center_roi", False))
        self.center_roi_width_ratio = float(rospy.get_param("~center_roi_width_ratio", 0.80))
        self.center_roi_height_ratio = float(rospy.get_param("~center_roi_height_ratio", 0.80))
        self.roi_x_min_ratio = float(rospy.get_param("~roi_x_min_ratio", 0.20))
        self.roi_x_max_ratio = float(rospy.get_param("~roi_x_max_ratio", 0.80))
        self.roi_y_min_ratio = float(rospy.get_param("~roi_y_min_ratio", 0.20))
        self.roi_y_max_ratio = float(rospy.get_param("~roi_y_max_ratio", 0.80))
        self.depth_min = float(rospy.get_param("~depth_min", 0.6))
        self.depth_max = float(rospy.get_param("~depth_max", 4.0))
        self.roi_size = int(rospy.get_param("~roi_size", 15))
        self.min_valid_depth_ratio = float(
            rospy.get_param(
                "~min_valid_depth_ratio",
                rospy.get_param("~min_depth_valid_ratio", 0.65),
            )
        )
        self.max_depth_std = float(rospy.get_param("~max_depth_std", 0.12))
        self.min_real_width = float(rospy.get_param("~min_real_width", 0.10))
        self.min_real_height = float(rospy.get_param("~min_real_height", 0.10))
        self.max_real_width = float(rospy.get_param("~max_real_width", 0.60))
        self.max_real_height = float(rospy.get_param("~max_real_height", 0.60))
        self.score_threshold = float(rospy.get_param("~score_threshold", 0.78))
        self.switch_score_margin = float(rospy.get_param("~switch_score_margin", 0.12))
        self.lost_keep_frames = int(rospy.get_param("~lost_keep_frames", 5))
        self.stable_window = int(rospy.get_param("~stable_window", 12))
        self.stable_min_count = int(rospy.get_param("~stable_min_count", 8))
        self.max_pixel_jump = float(rospy.get_param("~max_pixel_jump", 40.0))
        self.max_depth_jump = float(rospy.get_param("~max_depth_jump", 0.20))

        # Morphology is deliberately configurable because HSV masks often need
        # quick venue-side tuning when illumination changes.
        self.morph_open_kernel = int(rospy.get_param("~morph_open_kernel", 3))
        self.morph_close_kernel = int(rospy.get_param("~morph_close_kernel", 7))
        self.min_depth_valid_ratio = self.min_valid_depth_ratio
        self.sync_queue_size = int(rospy.get_param("~sync_queue_size", 10))
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.08))

        self.roi_size = max(3, self.roi_size)
        if self.roi_size % 2 == 0:
            self.roi_size += 1
        self.stable_window = max(1, self.stable_window)
        self.stable_min_count = min(max(1, self.stable_min_count), self.stable_window)
        self.debug_draw_mode = max(0, min(self.debug_draw_mode, 2))
        self.max_debug_candidates = max(1, self.max_debug_candidates)
        self.max_contours_per_color = max(1, self.max_contours_per_color)
        self.lost_keep_frames = max(0, self.lost_keep_frames)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame_id = ""

        self.history = {
            color_name: deque(maxlen=self.stable_window)
            for color_name in self.colors.keys()
        }
        self.last_best = None
        self.last_best_missing_count = 0

        self.debug_pub = rospy.Publisher(
            "/UAV0/color_tag_detector/debug_image", Image, queue_size=1
        )
        self.mask_pub = rospy.Publisher(
            "/UAV0/color_tag_detector/mask", Image, queue_size=1
        )
        self.point_pub = rospy.Publisher(
            "/UAV0/color_tag_detector/target_point_camera", PointStamped, queue_size=1
        )
        self.text_pub = rospy.Publisher(
            "/UAV0/color_tag_detector/result_text", String, queue_size=1
        )
        # Candidate output is intentionally separate from the stable result:
        # target_reporting can transform and forward a first valid geometric
        # observation while the planner moves to a safe observation pose.
        self.candidate_point_pub = rospy.Publisher(
            rospy.get_param(
                "~candidate_point_topic",
                "/UAV0/color_tag_detector/candidate_point_camera",
            ),
            PointStamped,
            queue_size=1,
        )
        self.candidate_text_pub = rospy.Publisher(
            rospy.get_param(
                "~candidate_text_topic",
                "/UAV0/color_tag_detector/candidate_text",
            ),
            String,
            queue_size=1,
        )

        rospy.Subscriber(
            self.camera_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1,
        )

        color_sub = message_filters.Subscriber(self.color_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop,
        )
        self.sync.registerCallback(self.image_callback)

        rospy.loginfo("color_tag_detector started")
        rospy.loginfo("color image: %s", self.color_topic)
        rospy.loginfo("aligned depth: %s", self.depth_topic)
        rospy.loginfo("camera_info: %s", self.camera_info_topic)
        rospy.loginfo("enabled colors: %s", ", ".join(self.colors.keys()))

    def filter_enabled_colors(self, colors):
        if self.target_color:
            if self.target_color in colors:
                return {self.target_color: colors[self.target_color]}
            rospy.logwarn("target_color '%s' is not in configured colors", self.target_color)
            return colors

        if self.enabled_colors:
            enabled = set(str(color).strip().lower() for color in self.enabled_colors)
            filtered = {
                color_name: color_cfg
                for color_name, color_cfg in colors.items()
                if color_name.lower() in enabled
            }
            if filtered:
                return filtered
            rospy.logwarn("enabled_colors did not match configured colors")

        return colors

    def camera_info_callback(self, msg):
        if msg.K[0] <= 0.0 or msg.K[4] <= 0.0:
            rospy.logwarn_throttle(2.0, "Invalid camera_info intrinsics")
            return
        self.fx = float(msg.K[0])
        self.fy = float(msg.K[4])
        self.cx = float(msg.K[2])
        self.cy = float(msg.K[5])
        self.camera_frame_id = msg.header.frame_id

    def image_callback(self, color_msg, depth_msg):
        try:
            color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(1.0, "cv_bridge conversion failed: %s", exc)
            self.publish_no_detection(color_msg.header, "cv_bridge_error")
            return

        if color_bgr is None or depth_raw is None:
            self.publish_no_detection(color_msg.header, "empty_image")
            return

        hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
        image_h, image_w = color_bgr.shape[:2]
        image_area = float(image_h * image_w)
        image_center = (image_w * 0.5, image_h * 0.5)
        max_center_dist = math.hypot(image_center[0], image_center[1])

        combined_mask = np.zeros((image_h, image_w), dtype=np.uint8)
        raw_candidates = []
        filtered_candidates = []
        best_per_color = {}

        for color_name, color_cfg in self.colors.items():
            color_mask = self.build_color_mask(hsv, color_cfg)
            color_mask = self.clean_mask(color_mask)
            combined_mask = cv2.bitwise_or(combined_mask, color_mask)

            color_candidates = self.find_candidates_for_color(
                color_name=color_name,
                color_cfg=color_cfg,
                hsv=hsv,
                mask=color_mask,
                depth_raw=depth_raw,
                depth_encoding=depth_msg.encoding,
                image_area=image_area,
                image_w=image_w,
                image_h=image_h,
                image_center=image_center,
                max_center_dist=max_center_dist,
            )

            raw_candidates.extend(color_candidates)
            passed_candidates = [
                candidate for candidate in color_candidates if candidate["passed_filter"]
            ]
            filtered_candidates.extend(passed_candidates)
            scoring_candidates = [
                candidate for candidate in passed_candidates if candidate["base_score"] >= self.score_threshold
            ]
            if scoring_candidates:
                best_per_color[color_name] = max(
                    scoring_candidates, key=lambda item: item["base_score"]
                )

        stable_info = self.update_stability(best_per_color)

        best = self.select_best_candidate(filtered_candidates, stable_info)
        debug_image = self.draw_debug_image(
            color_bgr.copy(),
            raw_candidates,
            filtered_candidates,
            best,
            stable_info,
            depth_msg.encoding,
        )

        self.publish_mask(combined_mask, color_msg.header)
        self.publish_debug(debug_image, color_msg.header)
        self.publish_result(best, stable_info, color_msg.header)

    def build_color_mask(self, hsv, color_cfg):
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for hsv_range in color_cfg.get("ranges", []):
            lower = np.array(hsv_range["lower"], dtype=np.uint8)
            upper = np.array(hsv_range["upper"], dtype=np.uint8)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        for hsv_range in color_cfg.get("exclude_ranges", []):
            lower = np.array(hsv_range["lower"], dtype=np.uint8)
            upper = np.array(hsv_range["upper"], dtype=np.uint8)
            excluded = cv2.inRange(hsv, lower, upper)
            mask = cv2.bitwise_and(mask, cv2.bitwise_not(excluded))
        return mask

    def clean_mask(self, mask):
        if self.morph_open_kernel > 1:
            k = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.morph_open_kernel, self.morph_open_kernel)
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        if self.morph_close_kernel > 1:
            k = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.morph_close_kernel, self.morph_close_kernel)
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def find_candidates_for_color(
        self,
        color_name,
        color_cfg,
        hsv,
        mask,
        depth_raw,
        depth_encoding,
        image_area,
        image_w,
        image_h,
        image_center,
        max_center_dist,
    ):
        contour_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_result[0] if len(contour_result) == 2 else contour_result[1]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[
            : self.max_contours_per_color
        ]
        candidates = []
        area_min = float(color_cfg.get("area_min", self.area_min))
        area_max_ratio = float(color_cfg.get("area_max_ratio", self.area_max_ratio))
        fill_ratio_min = float(color_cfg.get("fill_ratio_min", self.fill_ratio_min))
        min_valid_depth_ratio = float(
            color_cfg.get("min_valid_depth_ratio", self.min_valid_depth_ratio)
        )
        max_depth_std = float(color_cfg.get("max_depth_std", self.max_depth_std))
        min_real_width = float(color_cfg.get("min_real_width", self.min_real_width))
        min_real_height = float(color_cfg.get("min_real_height", self.min_real_height))
        max_real_width = float(color_cfg.get("max_real_width", self.max_real_width))
        max_real_height = float(color_cfg.get("max_real_height", self.max_real_height))
        max_bbox_width_ratio = float(
            color_cfg.get("max_bbox_width_ratio", self.max_bbox_width_ratio)
        )
        max_bbox_height_ratio = float(
            color_cfg.get("max_bbox_height_ratio", self.max_bbox_height_ratio)
        )
        max_area = max(area_min, image_area * area_max_ratio)

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area <= 1.0:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if h <= 0:
                continue

            bbox_area = float(max(w * h, 1))
            fill_ratio = area / bbox_area
            aspect_ratio = float(w) / float(h)
            moments = cv2.moments(contour)
            if abs(moments["m00"]) < 1e-6:
                continue
            u = int(round(moments["m10"] / moments["m00"]))
            v = int(round(moments["m01"] / moments["m00"]))

            cheap_reject = (
                area <= area_min
                or area >= max_area
                or float(w) / float(image_w) > max_bbox_width_ratio
                or float(h) / float(image_h) > max_bbox_height_ratio
                or aspect_ratio < self.aspect_ratio_min
                or aspect_ratio > self.aspect_ratio_max
                or fill_ratio < fill_ratio_min
                or not self.is_inside_detection_roi(u, v, image_w, image_h)
            )
            if cheap_reject:
                continue

            hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(hull))
            solidity = area / hull_area if hull_area > 1e-6 else 0.0
            min_solidity = float(color_cfg.get("min_solidity", self.min_solidity))

            hsv_stats = self.contour_hsv_stats(hsv, contour)
            min_mean_h = color_cfg.get("min_mean_hue", None)
            max_mean_h = color_cfg.get("max_mean_hue", None)
            min_mean_s = float(color_cfg.get("min_mean_saturation", 0.0))
            min_mean_v = float(color_cfg.get("min_mean_value", 0.0))

            depth, valid_ratio, depth_std, valid_count, total_count = self.depth_from_roi(
                depth_raw, depth_encoding, u, v, mask
            )

            real_width = None
            real_height = None
            point = None
            if depth is not None and self.has_camera_info():
                real_width = float(w) * depth / self.fx
                real_height = float(h) * depth / self.fy
                point = self.back_project(u, v, depth)

            center_dist = math.hypot(u - image_center[0], v - image_center[1])
            center_score = 1.0 - min(center_dist / max_center_dist, 1.0)
            area_score = self.interval_score(area, area_min, max_area)
            depth_score = valid_ratio
            depth_consistency_score = (
                1.0 - min(depth_std / max(max_depth_std, 1e-6), 1.0)
                if depth_std is not None
                else 0.0
            )
            size_score = 0.0
            if real_width is not None and real_height is not None:
                width_score = self.interval_score(real_width, min_real_width, max_real_width)
                height_score = self.interval_score(real_height, min_real_height, max_real_height)
                size_score = 0.5 * width_score + 0.5 * height_score
            history_count = self.history_count(color_name)
            history_score = min(float(history_count) / float(self.stable_min_count), 1.0)

            reject_reasons = []
            if area <= area_min:
                reject_reasons.append("area_small")
            if area >= max_area:
                reject_reasons.append("area_large")
            if float(w) / float(image_w) > max_bbox_width_ratio:
                reject_reasons.append("bbox_too_wide")
            if float(h) / float(image_h) > max_bbox_height_ratio:
                reject_reasons.append("bbox_too_tall")
            if aspect_ratio < self.aspect_ratio_min or aspect_ratio > self.aspect_ratio_max:
                reject_reasons.append("aspect_ratio")
            if fill_ratio < fill_ratio_min:
                reject_reasons.append("fill_ratio")
            if solidity < min_solidity:
                reject_reasons.append("solidity")
            if min_mean_h is not None and hsv_stats["mean_h"] < float(min_mean_h):
                reject_reasons.append("mean_hue_low")
            if max_mean_h is not None and hsv_stats["mean_h"] > float(max_mean_h):
                reject_reasons.append("mean_hue_high")
            if hsv_stats["mean_s"] < min_mean_s:
                reject_reasons.append("mean_saturation")
            if hsv_stats["mean_v"] < min_mean_v:
                reject_reasons.append("mean_value")
            if not self.is_inside_detection_roi(u, v, image_w, image_h):
                reject_reasons.append("outside_roi")
            if depth is None:
                reject_reasons.append("invalid_depth")
            else:
                if valid_ratio <= min_valid_depth_ratio:
                    reject_reasons.append("valid_depth_ratio")
                if depth <= self.depth_min or depth >= self.depth_max:
                    reject_reasons.append("depth_range")
                if depth_std is None or depth_std >= max_depth_std:
                    reject_reasons.append("depth_std")
            if not self.has_camera_info():
                reject_reasons.append("missing_camera_info")
            elif real_width is None or real_height is None:
                reject_reasons.append("missing_real_size")
            else:
                if real_width <= min_real_width or real_width >= max_real_width:
                    reject_reasons.append("real_width")
                if real_height <= min_real_height or real_height >= max_real_height:
                    reject_reasons.append("real_height")

            passed_filter = len(reject_reasons) == 0

            # Candidate scoring is intentionally multi-factor, not "largest blob wins".
            # It favors real tag-like size, clean depth, central position, and history.
            base_score = (
                0.15 * area_score
                + 0.30 * depth_score
                + 0.15 * depth_consistency_score
                + 0.20 * center_score
                + 0.10 * size_score
                + 0.10 * history_score
            )
            if not passed_filter:
                base_score = min(base_score, self.score_threshold - 0.01)

            candidates.append(
                {
                    "color_name": color_name,
                    "color": color_name,
                    "draw_bgr": tuple(int(c) for c in color_cfg.get("draw_bgr", [255, 255, 255])),
                    "contour": contour,
                    "bbox": (x, y, w, h),
                    "bbox_x": x,
                    "bbox_y": y,
                    "bbox_w": w,
                    "bbox_h": h,
                    "u": u,
                    "v": v,
                    "center_u": u,
                    "center_v": v,
                    "depth": depth,
                    "depth_median": depth,
                    "depth_valid": depth is not None and valid_ratio > 0.0,
                    "depth_valid_ratio": valid_ratio,
                    "valid_depth_ratio": valid_ratio,
                    "depth_std": depth_std,
                    "depth_valid_count": valid_count,
                    "depth_total_count": total_count,
                    "point_camera": point,
                    "area": area,
                    "pixel_area": area,
                    "aspect_ratio": aspect_ratio,
                    "fill_ratio": fill_ratio,
                    "extent": fill_ratio,
                    "solidity": solidity,
                    "real_width": real_width,
                    "real_height": real_height,
                    "center_distance_to_image_center": center_dist,
                    "mean_hue": hsv_stats["mean_h"],
                    "mean_saturation": hsv_stats["mean_s"],
                    "mean_value": hsv_stats["mean_v"],
                    "passed_filter": passed_filter,
                    "reject_reasons": reject_reasons,
                    "base_score": base_score,
                    "final_score": base_score,
                    "score": base_score,
                }
            )

        return candidates

    def is_inside_detection_roi(self, u, v, image_w, image_h):
        x0 = image_w * max(0.0, min(self.roi_x_min_ratio, 1.0))
        x1 = image_w * max(0.0, min(self.roi_x_max_ratio, 1.0))
        y0 = image_h * max(0.0, min(self.roi_y_min_ratio, 1.0))
        y1 = image_h * max(0.0, min(self.roi_y_max_ratio, 1.0))
        return x0 <= u <= x1 and y0 <= v <= y1

    def interval_score(self, value, lower, upper):
        if value is None or upper <= lower or value <= lower or value >= upper:
            return 0.0
        mid = 0.5 * (lower + upper)
        half = 0.5 * (upper - lower)
        centered = 1.0 - min(abs(value - mid) / max(half, 1e-6), 1.0)
        return 0.4 + 0.6 * centered

    def contour_hsv_stats(self, hsv, contour):
        contour_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
        pixels = hsv[contour_mask > 0]
        if pixels.size == 0:
            return {"mean_h": 0.0, "mean_s": 0.0, "mean_v": 0.0}
        return {
            "mean_h": float(np.mean(pixels[:, 0])),
            "mean_s": float(np.mean(pixels[:, 1])),
            "mean_v": float(np.mean(pixels[:, 2])),
        }

    def depth_from_roi(self, depth_raw, depth_encoding, u, v, mask):
        h, w = depth_raw.shape[:2]
        half = self.roi_size // 2
        x0 = max(0, u - half)
        x1 = min(w, u + half + 1)
        y0 = max(0, v - half)
        y1 = min(h, v + half + 1)

        if x0 >= x1 or y0 >= y1:
            return None, 0.0, None, 0, 0

        roi_depth = depth_raw[y0:y1, x0:x1]
        roi_mask = mask[y0:y1, x0:x1] > 0

        # Prefer depth pixels that also belong to the color mask. If the mask is
        # very thin near the centroid, fall back to the whole local ROI.
        if np.count_nonzero(roi_mask) > 0:
            samples = roi_depth[roi_mask]
            total_count = int(np.count_nonzero(roi_mask))
        else:
            samples = roi_depth.reshape(-1)
            total_count = int(samples.size)

        depth_m = self.depth_samples_to_meters(samples, depth_encoding)
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        valid &= depth_m >= self.depth_min
        valid &= depth_m <= self.depth_max

        valid_values = depth_m[valid]
        valid_count = int(valid_values.size)
        valid_ratio = float(valid_count) / float(max(total_count, 1))
        if valid_count == 0:
            return None, valid_ratio, None, valid_count, total_count

        return (
            float(np.median(valid_values)),
            valid_ratio,
            float(np.std(valid_values)),
            valid_count,
            total_count,
        )

    def depth_samples_to_meters(self, samples, depth_encoding):
        arr = np.asarray(samples)
        encoding = (depth_encoding or "").upper()

        if "16UC1" in encoding or arr.dtype == np.uint16:
            return arr.astype(np.float32) / 1000.0
        if "32FC1" in encoding or arr.dtype == np.float32 or arr.dtype == np.float64:
            return arr.astype(np.float32)

        # Fallback for unusual encodings. Most RealSense aligned depth streams
        # are 16UC1 in millimeters or 32FC1 in meters.
        if np.issubdtype(arr.dtype, np.integer):
            return arr.astype(np.float32) / 1000.0
        return arr.astype(np.float32)

    def has_camera_info(self):
        return self.fx is not None and self.fy is not None and self.cx is not None and self.cy is not None

    def back_project(self, u, v, depth_m):
        x = (float(u) - self.cx) * depth_m / self.fx
        y = (float(v) - self.cy) * depth_m / self.fy
        z = depth_m
        return [float(x), float(y), float(z)]

    def history_count(self, color_name):
        return sum(1 for obs in self.history.get(color_name, []) if obs is not None)

    def update_stability(self, best_per_color):
        stable_info = {}

        for color_name in self.colors.keys():
            hist = self.history[color_name]
            obs = best_per_color.get(color_name)

            if obs is None or not obs["depth_valid"] or obs["point_camera"] is None:
                hist.append(None)
                stable_info[color_name] = {
                    "stable": False,
                    "count": self.history_count(color_name),
                    "reference": self.median_observation(hist),
                }
                continue

            # A large jump resets stability. The new observation starts a fresh
            # history so a real moved target can become stable again after N frames.
            last = self.last_valid_observation(hist)
            if last is not None and self.is_jump_too_large(last, obs):
                hist.clear()
                hist.append(self.compact_observation(obs))
                stable_info[color_name] = {
                    "stable": False,
                    "count": 1,
                    "reference": self.median_observation(hist),
                }
                continue

            hist.append(self.compact_observation(obs))
            count = self.history_count(color_name)
            stable = count >= self.stable_min_count
            stable_info[color_name] = {
                "stable": stable,
                "count": count,
                "reference": self.median_observation(hist),
            }

        return stable_info

    def compact_observation(self, candidate):
        return {
            "u": float(candidate["u"]),
            "v": float(candidate["v"]),
            "depth": float(candidate["depth"]),
            "point_camera": list(candidate["point_camera"]),
            "area": float(candidate["area"]),
        }

    def last_valid_observation(self, hist):
        for obs in reversed(hist):
            if obs is not None:
                return obs
        return None

    def median_observation(self, hist):
        valid = [obs for obs in hist if obs is not None]
        if not valid:
            return None

        points = np.array([obs["point_camera"] for obs in valid], dtype=np.float32)
        return {
            "u": float(np.median([obs["u"] for obs in valid])),
            "v": float(np.median([obs["v"] for obs in valid])),
            "depth": float(np.median([obs["depth"] for obs in valid])),
            "point_camera": [
                float(np.median(points[:, 0])),
                float(np.median(points[:, 1])),
                float(np.median(points[:, 2])),
            ],
        }

    def is_jump_too_large(self, previous, current):
        pixel_jump = math.hypot(current["u"] - previous["u"], current["v"] - previous["v"])
        depth_jump = abs(current["depth"] - previous["depth"])
        return pixel_jump > self.max_pixel_jump or depth_jump > self.max_depth_jump

    def candidate_matches_stable_reference(self, candidate, info):
        if not info.get("stable", False):
            return False
        reference = info.get("reference")
        if reference is None or not candidate["depth_valid"] or candidate["point_camera"] is None:
            return False

        pixel_jump = math.hypot(candidate["u"] - reference["u"], candidate["v"] - reference["v"])
        depth_jump = abs(candidate["depth"] - reference["depth"])
        return pixel_jump <= self.max_pixel_jump and depth_jump <= self.max_depth_jump

    def select_best_candidate(self, candidates, stable_info):
        if not candidates:
            if self.last_best is not None and self.last_best_missing_count < self.lost_keep_frames:
                self.last_best_missing_count += 1
                held = dict(self.last_best)
                held["held"] = True
                return held
            self.last_best = None
            return None

        selectable = []
        for candidate in candidates:
            info = stable_info.get(candidate["color"], {"stable": False, "count": 0})
            stable_bonus = 0.05 if self.candidate_matches_stable_reference(candidate, info) else 0.0
            valid_depth_bonus = 0.0 if candidate["point_camera"] is not None else -0.25
            candidate["final_score"] = candidate["base_score"] + stable_bonus + valid_depth_bonus
            candidate["score"] = candidate["final_score"]
            if candidate["final_score"] >= self.score_threshold:
                selectable.append(candidate)

        if not selectable:
            if self.last_best is not None and self.last_best_missing_count < self.lost_keep_frames:
                self.last_best_missing_count += 1
                held = dict(self.last_best)
                held["held"] = True
                return held
            self.last_best = None
            return None

        top = max(selectable, key=lambda item: item["final_score"])
        matched_previous = self.find_matching_previous(selectable)
        if matched_previous is not None:
            if top["color"] != matched_previous["color"]:
                if top["final_score"] < matched_previous["final_score"] + self.switch_score_margin:
                    top = matched_previous
            elif top is not matched_previous:
                if top["final_score"] < matched_previous["final_score"] + self.switch_score_margin:
                    top = matched_previous

        top["held"] = False
        self.last_best = dict(top)
        self.last_best_missing_count = 0
        return top

    def find_matching_previous(self, candidates):
        if self.last_best is None:
            return None

        matches = []
        for candidate in candidates:
            if candidate["color"] != self.last_best["color"]:
                continue
            pixel_jump = math.hypot(
                candidate["u"] - self.last_best["u"],
                candidate["v"] - self.last_best["v"],
            )
            if self.last_best.get("depth") is None or candidate.get("depth") is None:
                depth_jump = 0.0
            else:
                depth_jump = abs(candidate["depth"] - self.last_best["depth"])
            if pixel_jump <= self.max_pixel_jump and depth_jump <= self.max_depth_jump:
                matches.append(candidate)

        if not matches:
            return None
        return max(matches, key=lambda item: item["final_score"])

    def draw_debug_image(
        self,
        image,
        raw_candidates,
        filtered_candidates,
        best,
        stable_info,
        depth_encoding,
    ):
        image_h, image_w = image.shape[:2]
        roi_x0 = int(image_w * self.roi_x_min_ratio)
        roi_x1 = int(image_w * self.roi_x_max_ratio)
        roi_y0 = int(image_h * self.roi_y_min_ratio)
        roi_y1 = int(image_h * self.roi_y_max_ratio)
        cv2.rectangle(image, (roi_x0, roi_y0), (roi_x1, roi_y1), (180, 180, 180), 1)

        stable = False
        if best is not None:
            stable = self.candidate_matches_stable_reference(
                best, stable_info.get(best["color"], {"stable": False})
            )

        if self.debug_draw_mode == 0:
            drawable_candidates = []
        elif self.debug_draw_mode == 1:
            drawable_candidates = sorted(
                filtered_candidates,
                key=lambda item: item["final_score"],
                reverse=True,
            )[: self.max_debug_candidates]
        else:
            drawable_candidates = list(filtered_candidates)

        if best is not None and stable and best not in drawable_candidates:
            drawable_candidates.append(best)

        for candidate in drawable_candidates:
            if candidate is best and stable:
                continue
            x, y, w, h = candidate["bbox"]
            candidate_color = candidate["draw_bgr"]
            cv2.rectangle(image, (x, y), (x + w, y + h), candidate_color, 2)
            cv2.circle(image, (candidate["u"], candidate["v"]), 4, candidate_color, -1)
            self.draw_label(
                image,
                self.format_candidate_label(candidate),
                x,
                y - 6,
                candidate_color,
                scale=0.45,
            )

        if best is not None and stable:
            x, y, w, h = best["bbox"]
            cv2.rectangle(image, (x, y), (x + w, y + h), best["draw_bgr"], 4)
            cv2.circle(image, (best["u"], best["v"]), 5, best["draw_bgr"], -1)
            self.draw_label(
                image,
                "CONFIRMED " + self.format_candidate_label(best),
                x,
                y - 8,
                best["draw_bgr"],
                scale=0.5,
            )

        lines = []
        if best is not None:
            info = stable_info.get(best["color"], {"stable": False, "count": 0})
            lines.append(
                "{} score:{:.2f} candidate:true stable:{} {}/{}".format(
                    best["color"],
                    best["final_score"],
                    stable,
                    info.get("count", 0),
                    self.stable_window,
                )
            )
            if stable:
                lines.append(
                    "u:{} v:{} Z:{:.2f}m valid:{:.0f}% std:{:.2f}".format(
                        best["u"],
                        best["v"],
                        best["depth"] if best["depth"] is not None else -1.0,
                        best["valid_depth_ratio"] * 100.0,
                        best["depth_std"] if best["depth_std"] is not None else -1.0,
                    )
                )
                if best["real_width"] is not None and best["real_height"] is not None:
                    lines.append(
                        "real W:{:.2f}m H:{:.2f}m".format(
                            best["real_width"], best["real_height"]
                        )
                    )
                if best["point_camera"] is not None:
                    p = best["point_camera"]
                    lines.append("P:[{:.2f}, {:.2f}, {:.2f}]m".format(p[0], p[1], p[2]))
                elif not self.has_camera_info():
                    lines.append("waiting camera_info")
            else:
                lines.append("waiting for detector confirmation")
        else:
            lines.append("candidate:false stable:false")
            lines.append("waiting for candidate")

        if self.debug_draw_mode > 0:
            lines.append(
                "drawable:{} mode:{} depth:{}".format(
                    len(filtered_candidates),
                    self.debug_draw_mode,
                    depth_encoding,
                )
            )

        y = 24
        for line in lines:
            cv2.putText(image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            y += 24
        return image

    def draw_label(self, image, text, x, y, color, scale=0.5):
        y = max(14, int(y))
        x = max(2, int(x))
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

    def format_candidate_label(self, candidate):
        return "{} s:{:.2f} z:{:.2f} w:{:.2f} h:{:.2f} std:{:.2f}".format(
            candidate["color"],
            candidate["final_score"],
            candidate["depth"] if candidate["depth"] is not None else -1.0,
            candidate["real_width"] if candidate["real_width"] is not None else -1.0,
            candidate["real_height"] if candidate["real_height"] is not None else -1.0,
            candidate["depth_std"] if candidate["depth_std"] is not None else -1.0,
        )

    def publish_mask(self, mask, header):
        try:
            msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            msg.header = header
            self.mask_pub.publish(msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(1.0, "failed to publish mask: %s", exc)

    def publish_debug(self, image, header):
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            msg.header = header
            self.debug_pub.publish(msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(1.0, "failed to publish debug image: %s", exc)

    def publish_candidate(self, best, stable_info, header):
        """Publish a fresh valid target before the temporal filter confirms it."""
        if best is None or best.get("held", False):
            return
        point_out = best.get("point_camera")
        if not best.get("depth_valid", False) or point_out is None:
            return

        color_info = stable_info.get(best["color"], {"stable": False, "count": 0})
        stable = self.candidate_matches_stable_reference(best, color_info)
        point_msg = PointStamped()
        point_msg.header.stamp = header.stamp
        point_msg.header.frame_id = self.camera_frame_id or header.frame_id
        point_msg.point.x = float(point_out[0])
        point_msg.point.y = float(point_out[1])
        point_msg.point.z = float(point_out[2])
        self.candidate_point_pub.publish(point_msg)

        candidate = {
            "detected": True,
            "candidate": True,
            "stable": bool(stable),
            "confirmable": bool(stable),
            "held": False,
            "color": best["color"],
            "u": int(round(best["u"])),
            "v": int(round(best["v"])),
            "bbox": [int(value) for value in best["bbox"]],
            "depth": round(float(best["depth"]), 4),
            "point_camera": [round(float(value), 4) for value in point_out],
            "score": round(float(best["final_score"]), 3),
            "stable_count": int(color_info.get("count", 0)),
            "stable_window": int(self.stable_window),
            "stamp": header.stamp.to_sec() if header.stamp else None,
            "reason": "stable_candidate" if stable else "raw_candidate",
        }
        self.candidate_text_pub.publish(
            String(data=json.dumps(candidate, ensure_ascii=False))
        )

    def publish_result(self, best, stable_info, header):
        # Keep the raw candidate stream independent from the stable result
        # stream. The latter remains backward compatible for existing users.
        self.publish_candidate(best, stable_info, header)
        if best is None:
            self.publish_result_text(
                {
                    "detected": False,
                    "stable": False,
                    "color": None,
                    "u": None,
                    "v": None,
                    "depth": None,
                    "point_camera": None,
                    "area": None,
                    "reason": "no_candidate",
                }
            )
            return

        color_info = stable_info.get(best["color"], {"stable": False, "count": 0})
        stable = self.candidate_matches_stable_reference(best, color_info)
        if not stable:
            self.publish_result_text(
                {
                    "detected": False,
                    "stable": False,
                    "reason": "not_stable_yet",
                    "best_color": best["color"],
                    "best_score": round(float(best["final_score"]), 3),
                    "stable_count": int(color_info.get("count", 0)),
                }
            )
            return

        stable_reference = color_info.get("reference") if stable else None
        u_out = stable_reference["u"] if stable_reference is not None else best["u"]
        v_out = stable_reference["v"] if stable_reference is not None else best["v"]
        depth_out = stable_reference["depth"] if stable_reference is not None else best["depth"]
        point_out = (
            stable_reference["point_camera"]
            if stable_reference is not None
            else best["point_camera"]
        )

        result = {
            "detected": True,
            "stable": True,
            "confirmable": True,
            "color": best["color"],
            "u": int(round(u_out)),
            "v": int(round(v_out)),
            "bbox": [int(value) for value in best["bbox"]],
            "depth": round(float(depth_out), 4) if depth_out is not None else None,
            "point_camera": [round(float(x), 4) for x in point_out]
            if point_out is not None
            else None,
            "score": round(float(best["final_score"]), 3),
            "real_width": round(float(best["real_width"]), 4)
            if best["real_width"] is not None
            else None,
            "real_height": round(float(best["real_height"]), 4)
            if best["real_height"] is not None
            else None,
            "valid_depth_ratio": round(float(best["valid_depth_ratio"]), 3),
            "depth_std": round(float(best["depth_std"]), 4)
            if best["depth_std"] is not None
            else None,
            "area": round(float(best["area"]), 1),
            "pixel_area": round(float(best["pixel_area"]), 1),
            "fill_ratio": round(float(best["fill_ratio"]), 3),
            "stable_count": int(color_info["count"]),
            "stable_window": int(self.stable_window),
        }

        point_msg = PointStamped()
        point_msg.header.stamp = header.stamp
        point_msg.header.frame_id = self.camera_frame_id or header.frame_id
        point_msg.point.x = float(point_out[0])
        point_msg.point.y = float(point_out[1])
        point_msg.point.z = float(point_out[2])
        self.point_pub.publish(point_msg)

        self.publish_result_text(result)

    def publish_no_detection(self, header, reason):
        self.publish_result_text(
            {
                "detected": False,
                "stable": False,
                "color": None,
                "u": None,
                "v": None,
                "depth": None,
                "point_camera": None,
                "area": None,
                "reason": reason,
            }
        )

    def publish_result_text(self, result):
        self.text_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))


def main():
    rospy.init_node("color_tag_detector")
    ColorTagDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
