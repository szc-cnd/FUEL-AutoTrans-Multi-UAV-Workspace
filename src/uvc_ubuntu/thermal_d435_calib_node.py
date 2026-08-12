#!/usr/bin/env python3
import os
import threading

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ThermalD435CalibNode:
    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()

        self.thermal_debug_topic = rospy.get_param("~thermal_debug_topic", "/UAV0/thermal/debug_image")
        self.d435_color_topic = rospy.get_param("~d435_color_topic", "/camera/color/image_raw")
        default_config = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "config", "thermal_d435_fusion.yaml"
        )
        self.config_path = rospy.get_param("~config_path", default_config)

        self.thermal_image = None
        self.color_image = None
        self.thermal_points = []
        self.d435_points = []
        self.pending_thermal_point = None
        self.last_layout = None
        self.last_error_summary = None

        rospy.Subscriber(self.thermal_debug_topic, Image, self.thermal_callback, queue_size=1)
        rospy.Subscriber(self.d435_color_topic, Image, self.color_callback, queue_size=1)

        self.window_name = "Thermal-D435 Homography Calibration"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

    def thermal_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to convert thermal debug image: %s", exc)
            return
        with self.lock:
            self.thermal_image = image

    def color_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "Failed to convert D435 color image: %s", exc)
                return
        with self.lock:
            self.color_image = image

    def make_display(self):
        with self.lock:
            thermal = None if self.thermal_image is None else self.thermal_image.copy()
            color = None if self.color_image is None else self.color_image.copy()

        if thermal is None or color is None:
            canvas = np.zeros((360, 960, 3), dtype=np.uint8)
            cv2.putText(
                canvas,
                "Waiting for /UAV0/thermal/debug_image and /camera/color/image_raw",
                (20, 180),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            self.last_layout = None
            return canvas

        target_h = 480
        t_scale = target_h / float(thermal.shape[0])
        c_scale = target_h / float(color.shape[0])
        thermal_show = cv2.resize(
            thermal, None, fx=t_scale, fy=t_scale, interpolation=cv2.INTER_NEAREST
        )
        color_show = cv2.resize(
            color, None, fx=c_scale, fy=c_scale, interpolation=cv2.INTER_NEAREST
        )
        gap = 8
        h = max(thermal_show.shape[0], color_show.shape[0])
        w = thermal_show.shape[1] + gap + color_show.shape[1]
        canvas = np.zeros((h + 46, w, 3), dtype=np.uint8)

        t_origin = (0, 46)
        c_origin = (thermal_show.shape[1] + gap, 46)
        canvas[t_origin[1] : t_origin[1] + thermal_show.shape[0], 0 : thermal_show.shape[1]] = thermal_show
        canvas[
            c_origin[1] : c_origin[1] + color_show.shape[0],
            c_origin[0] : c_origin[0] + color_show.shape[1],
        ] = color_show

        self.last_layout = {
            "thermal_origin": t_origin,
            "color_origin": c_origin,
            "thermal_size": (thermal_show.shape[1], thermal_show.shape[0]),
            "color_size": (color_show.shape[1], color_show.shape[0]),
            "thermal_scale": t_scale,
            "color_scale": c_scale,
        }

        cv2.putText(
            canvas,
            "Click thermal point, then same D435 color point. e=error s=save r=reset q/ESC=quit",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )
        if self.last_error_summary is not None:
            text = (
                f"pairs={self.last_error_summary['count']} "
                f"mean={self.last_error_summary['mean']:.2f}px "
                f"max={self.last_error_summary['max']:.2f}px"
            )
            cv2.putText(
                canvas,
                text,
                (10, 44),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 200, 255),
                2,
            )
        self.draw_points(canvas)
        return canvas

    def draw_points(self, canvas):
        if self.last_layout is None:
            return
        t_origin = self.last_layout["thermal_origin"]
        c_origin = self.last_layout["color_origin"]
        t_scale = self.last_layout["thermal_scale"]
        c_scale = self.last_layout["color_scale"]

        for idx, (pt_t, pt_c) in enumerate(zip(self.thermal_points, self.d435_points), start=1):
            tx = int(round(t_origin[0] + pt_t[0] * t_scale))
            ty = int(round(t_origin[1] + pt_t[1] * t_scale))
            cx = int(round(c_origin[0] + pt_c[0] * c_scale))
            cy = int(round(c_origin[1] + pt_c[1] * c_scale))
            cv2.circle(canvas, (tx, ty), 5, (0, 255, 0), -1)
            cv2.circle(canvas, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(canvas, str(idx), (tx + 7, ty - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(canvas, str(idx), (cx + 7, cy - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if self.pending_thermal_point is not None:
            tx = int(round(t_origin[0] + self.pending_thermal_point[0] * t_scale))
            ty = int(round(t_origin[1] + self.pending_thermal_point[1] * t_scale))
            cv2.circle(canvas, (tx, ty), 6, (0, 0, 255), 2)

    def mouse_callback(self, event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN or self.last_layout is None:
            return

        layout = self.last_layout
        t_origin = layout["thermal_origin"]
        c_origin = layout["color_origin"]
        tw, th = layout["thermal_size"]
        cw, ch = layout["color_size"]

        in_thermal = t_origin[0] <= x < t_origin[0] + tw and t_origin[1] <= y < t_origin[1] + th
        in_color = c_origin[0] <= x < c_origin[0] + cw and c_origin[1] <= y < c_origin[1] + ch

        if in_thermal:
            u = (x - t_origin[0]) / layout["thermal_scale"]
            v = (y - t_origin[1]) / layout["thermal_scale"]
            self.pending_thermal_point = (float(u), float(v))
            rospy.loginfo("Selected thermal point: (%.1f, %.1f)", u, v)
            return

        if in_color:
            if self.pending_thermal_point is None:
                rospy.logwarn("Click a thermal-image point first")
                return
            u = (x - c_origin[0]) / layout["color_scale"]
            v = (y - c_origin[1]) / layout["color_scale"]
            self.thermal_points.append(self.pending_thermal_point)
            self.d435_points.append((float(u), float(v)))
            rospy.loginfo(
                "Added pair %d: thermal=(%.1f, %.1f), d435=(%.1f, %.1f)",
                len(self.thermal_points),
                self.pending_thermal_point[0],
                self.pending_thermal_point[1],
                u,
                v,
            )
            self.pending_thermal_point = None

    def reset_points(self):
        self.thermal_points = []
        self.d435_points = []
        self.pending_thermal_point = None
        self.last_error_summary = None
        rospy.loginfo("Cleared calibration points")

    def compute_homography(self):
        if len(self.thermal_points) < 4:
            rospy.logwarn("Need at least 4 point pairs, currently %d", len(self.thermal_points))
            return None
        thermal_pts = np.array(self.thermal_points, dtype=np.float32)
        d435_pts = np.array(self.d435_points, dtype=np.float32)
        h, mask = cv2.findHomography(thermal_pts, d435_pts, cv2.RANSAC)
        if h is None:
            rospy.logwarn("cv2.findHomography failed")
            return None
        inliers = int(mask.sum()) if mask is not None else len(self.thermal_points)
        rospy.loginfo("Computed H with %d/%d inliers", inliers, len(self.thermal_points))
        return h

    def compute_reprojection_errors(self, h):
        thermal_pts = np.array(self.thermal_points, dtype=np.float32).reshape(-1, 1, 2)
        d435_pts = np.array(self.d435_points, dtype=np.float32)
        projected = cv2.perspectiveTransform(thermal_pts, h).reshape(-1, 2)
        errors = np.linalg.norm(projected - d435_pts, axis=1)
        return projected, errors

    def report_reprojection_error(self, h=None):
        if h is None:
            h = self.compute_homography()
        if h is None:
            return None

        projected, errors = self.compute_reprojection_errors(h)
        if len(errors) == 0:
            return None

        mean_error = float(np.mean(errors))
        max_error = float(np.max(errors))
        self.last_error_summary = {
            "count": len(errors),
            "mean": mean_error,
            "max": max_error,
        }

        rospy.loginfo("Homography reprojection error:")
        for idx, (src, dst, pred, err) in enumerate(
            zip(self.thermal_points, self.d435_points, projected, errors), start=1
        ):
            rospy.loginfo(
                "  pair %d: thermal=(%.1f, %.1f) clicked_d435=(%.1f, %.1f) "
                "projected=(%.1f, %.1f) error=%.2f px",
                idx,
                src[0],
                src[1],
                dst[0],
                dst[1],
                pred[0],
                pred[1],
                err,
            )
        rospy.loginfo("  mean error: %.2f px", mean_error)
        rospy.loginfo("  max error: %.2f px", max_error)

        if mean_error <= 5.0 and max_error <= 15.0:
            rospy.loginfo("  quality: good")
        elif mean_error <= 10.0 and max_error <= 20.0:
            rospy.loginfo("  quality: usable")
        else:
            rospy.logwarn("  quality: poor; consider reselecting points")

        return self.last_error_summary

    def save_homography(self):
        h = self.compute_homography()
        if h is None:
            return
        self.report_reprojection_error(h)

        data = {}
        if os.path.exists(self.config_path):
            with open(self.config_path, "r", encoding="utf-8") as f:
                loaded = yaml.safe_load(f)
                if isinstance(loaded, dict):
                    data = loaded

        data["thermal_to_d435_homography"] = h.tolist()
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        with open(self.config_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        rospy.loginfo("Saved homography to %s", self.config_path)

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            display = self.make_display()
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                self.reset_points()
            if key == ord("e"):
                self.report_reprojection_error()
            if key == ord("s"):
                self.save_homography()
            rate.sleep()
        cv2.destroyAllWindows()


def main():
    rospy.init_node("thermal_d435_calib_node", anonymous=False)
    node = ThermalD435CalibNode()
    rospy.loginfo("thermal_d435_calib_node started")
    node.spin()


if __name__ == "__main__":
    main()
