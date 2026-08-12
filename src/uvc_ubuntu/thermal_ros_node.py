#!/usr/bin/env python3
import json
import os
import sys
from collections import deque

import cv2
import numpy as np
import rospkg
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
try:
    PACKAGE_DIR = rospkg.RosPack().get_path("uvc_ubuntu")
except rospkg.ResourceNotFound:
    PACKAGE_DIR = SCRIPT_DIR
if not sys.path or sys.path[0] != PACKAGE_DIR:
    sys.path.insert(0, PACKAGE_DIR)

from thermal_detect import (
    detect_hotspot,
    draw_debug,
    read_frame,
    shift_image_no_wrap,
    start_uvc_demo,
    stop_process,
    yuyv_to_gray,
)


class StableTargetFilter:
    def __init__(self, enabled=True, max_len=3, min_hits=2, max_pixel_jump=25.0):
        self.enabled = enabled
        self.max_len = max(1, int(max_len))
        self.min_hits = max(1, int(min_hits))
        self.max_pixel_jump = float(max_pixel_jump)
        self.history = deque(maxlen=self.max_len)

    def detected_count(self):
        """Number of detected samples currently in the stability window."""
        return sum(1 for item in self.history if item["detected"])

    def update(self, detection):
        sample = {
            "detected": bool(detection["detected"]),
            "cx": float(detection["cx"]),
            "cy": float(detection["cy"]),
        }
        self.history.append(sample)

        if not self.enabled:
            return sample["detected"], sample["cx"], sample["cy"]

        hits = [item for item in self.history if item["detected"]]
        if len(hits) < self.min_hits:
            return False, -1.0, -1.0

        max_jump = 0.0
        for i in range(len(hits)):
            for j in range(i + 1, len(hits)):
                dx = hits[i]["cx"] - hits[j]["cx"]
                dy = hits[i]["cy"] - hits[j]["cy"]
                jump = (dx * dx + dy * dy) ** 0.5
                max_jump = max(max_jump, jump)

        if max_jump > self.max_pixel_jump:
            return False, -1.0, -1.0

        cx = sum(item["cx"] for item in hits) / len(hits)
        cy = sum(item["cy"] for item in hits) / len(hits)
        return True, cx, cy


def numpy_to_image_msg(image, encoding, stamp, frame_id):
    if image is None:
        raise ValueError("image is None")

    arr = np.ascontiguousarray(image)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(arr.shape[0])
    msg.width = int(arr.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0

    if arr.ndim == 2:
        channels = 1
    elif arr.ndim == 3:
        channels = int(arr.shape[2])
    else:
        raise ValueError(f"unsupported image shape: {arr.shape}")

    msg.step = int(msg.width * channels * arr.dtype.itemsize)
    msg.data = arr.tobytes()
    return msg


class ThermalDetectorNode:
    def __init__(self):
        self.width = int(rospy.get_param("~width", 384))
        self.height = int(rospy.get_param("~height", 288))
        self.shift_x = int(rospy.get_param("~shift_x", 0))
        self.shift_y = int(rospy.get_param("~shift_y", 0))
        self.roi_margin_x = int(rospy.get_param("~roi_margin_x", 30))
        self.roi_margin_y = int(rospy.get_param("~roi_margin_y", 20))
        self.threshold_k = float(rospy.get_param("~threshold_k", 2.0))
        self.min_area = float(rospy.get_param("~min_area", 20))
        self.max_area = float(rospy.get_param("~max_area", 5000))
        self.publish_rate = float(rospy.get_param("~publish_rate", 20))
        self.display = bool(rospy.get_param("~display", False))
        self.uvc_demo_path = str(rospy.get_param("~uvc_demo_path", "./uvc_demo"))
        self.uvc_offset_fix = rospy.get_param("~uvc_offset_fix", None)
        if self.uvc_offset_fix is not None:
            self.uvc_offset_fix = int(self.uvc_offset_fix)
        self.stable_filter_enable = bool(rospy.get_param("~stable_filter_enable", True))
        self.stable_max_len = int(rospy.get_param("~stable_max_len", 3))
        self.stable_min_hits = int(rospy.get_param("~stable_min_hits", 2))
        self.stable_max_pixel_jump = float(rospy.get_param("~stable_max_pixel_jump", 25.0))

        self.frame_size = self.width * self.height * 2
        self.process = None
        self.stable_filter = StableTargetFilter(
            enabled=self.stable_filter_enable,
            max_len=self.stable_max_len,
            min_hits=self.stable_min_hits,
            max_pixel_jump=self.stable_max_pixel_jump,
        )

        self.image_pub = rospy.Publisher("/UAV0/thermal/image_raw", Image, queue_size=1)
        self.debug_pub = rospy.Publisher("/UAV0/thermal/debug_image", Image, queue_size=1)
        self.detected_pub = rospy.Publisher("/UAV0/thermal/target_detected", Bool, queue_size=1)
        self.pixel_pub = rospy.Publisher("/UAV0/thermal/target_pixel", PointStamped, queue_size=1)
        self.candidate_detected_pub = rospy.Publisher(
            rospy.get_param(
                "~candidate_detected_topic", "/UAV0/thermal/target_candidate_detected"
            ),
            Bool,
            queue_size=1,
        )
        self.candidate_status_pub = rospy.Publisher(
            rospy.get_param(
                "~candidate_status_topic", "/UAV0/thermal/target_candidate_status"
            ),
            String,
            queue_size=1,
        )
        self.candidate_pixel_pub = rospy.Publisher(
            rospy.get_param(
                "~candidate_pixel_topic", "/UAV0/thermal/target_candidate_pixel"
            ),
            PointStamped,
            queue_size=1,
        )

        rospy.on_shutdown(self.shutdown)

    def start(self):
        self.process = start_uvc_demo(self.uvc_demo_path, self.uvc_offset_fix)
        rospy.loginfo(
            "thermal_detector_node started; stable_filter=%s max_len=%d min_hits=%d max_jump=%.1f",
            self.stable_filter_enable,
            self.stable_max_len,
            self.stable_min_hits,
            self.stable_max_pixel_jump,
        )

    def publish_detection(self, gray, debug, detection):
        stamp = rospy.Time.now()
        stable_detected, stable_cx, stable_cy = self.stable_filter.update(detection)
        # Render the same detector-level candidate/stable state that is sent
        # in candidate_status.  The caller's legacy debug argument is kept
        # for API compatibility with older launch wrappers.
        debug = draw_debug(
            gray,
            detection,
            stable_detected=stable_detected,
            stable_count=self.stable_filter.detected_count(),
            stable_window=self.stable_max_len,
        )

        image_msg = numpy_to_image_msg(gray, "mono8", stamp, "thermal_camera")
        debug_msg = numpy_to_image_msg(debug, "bgr8", stamp, "thermal_camera")

        detected_msg = Bool(data=bool(stable_detected))

        point_msg = PointStamped()
        point_msg.header.stamp = stamp
        point_msg.header.frame_id = "thermal_camera"
        point_msg.point.x = float(stable_cx) if stable_detected else -1.0
        point_msg.point.y = float(stable_cy) if stable_detected else -1.0
        point_msg.point.z = 1.0 if stable_detected else 0.0

        self.image_pub.publish(image_msg)
        self.debug_pub.publish(debug_msg)
        self.detected_pub.publish(detected_msg)
        self.pixel_pub.publish(point_msg)

        # The stable stream above preserves the original package contract.
        # The candidate stream exposes the current valid hotspot immediately,
        # before the rolling 2/3-frame filter confirms it.
        candidate_detected = bool(detection["detected"])
        stable_count = self.stable_filter.detected_count()
        self.candidate_detected_pub.publish(Bool(data=candidate_detected))
        candidate_status = {
            "detected": candidate_detected,
            "candidate": candidate_detected,
            "stable": bool(stable_detected),
            "confirmable": bool(stable_detected),
            "stable_count": int(stable_count),
            "stable_window": int(self.stable_max_len),
            "cx": int(detection["cx"]) if candidate_detected else None,
            "cy": int(detection["cy"]) if candidate_detected else None,
            "bbox": list(detection["bbox"]) if candidate_detected else None,
            "area": float(detection["area"]) if candidate_detected else 0.0,
            "reason": "stable_candidate" if stable_detected else "raw_candidate",
        }
        self.candidate_status_pub.publish(
            String(data=json.dumps(candidate_status, ensure_ascii=False))
        )
        if candidate_detected:
            candidate_msg = PointStamped()
            candidate_msg.header.stamp = stamp
            candidate_msg.header.frame_id = "thermal_camera"
            candidate_msg.point.x = float(detection["cx"])
            candidate_msg.point.y = float(detection["cy"])
            candidate_msg.point.z = 1.0
            self.candidate_pixel_pub.publish(candidate_msg)

    def spin(self):
        self.start()
        rate = rospy.Rate(self.publish_rate) if self.publish_rate > 0 else None

        while not rospy.is_shutdown():
            raw_data = read_frame(self.process, self.frame_size)
            if raw_data is None:
                if self.process is not None and self.process.poll() is not None:
                    rospy.logerr("uvc_demo exited, stopping thermal ROS node")
                    break
                continue

            try:
                gray = yuyv_to_gray(raw_data, self.width, self.height)
            except ValueError as exc:
                rospy.logwarn("frame parse error: %s", exc)
                continue

            gray = shift_image_no_wrap(gray, self.shift_x, self.shift_y, fill_value=0)
            detection = detect_hotspot(
                gray,
                roi_margin_x=self.roi_margin_x,
                roi_margin_y=self.roi_margin_y,
                k=self.threshold_k,
                min_area=self.min_area,
                max_area=self.max_area,
            )
            debug = draw_debug(gray, detection)
            self.publish_detection(gray, debug, detection)

            if self.display:
                cv2.imshow("Thermal ROS Debug", debug)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    rospy.signal_shutdown("user requested exit")

            if rate is not None:
                rate.sleep()

    def shutdown(self):
        stop_process(self.process)
        if self.display:
            cv2.destroyAllWindows()


def main():
    rospy.init_node("thermal_detector_node", anonymous=False)
    node = ThermalDetectorNode()
    try:
        node.spin()
    except FileNotFoundError as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
