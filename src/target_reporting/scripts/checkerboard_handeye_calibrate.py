#!/usr/bin/env python3
"""Offline checkerboard hand-eye calibration for FAST-LIO and RealSense."""

import argparse
import bisect
import os
import sys

import cv2
import numpy as np
import rosbag
import yaml
from cv_bridge import CvBridge

from target_reporting.calibration import build_result_document, default_calibration_config
from target_reporting.handeye import (
    checkerboard_object_points,
    lookup_transform_from_edges,
    matrix_to_quaternion,
    pose_matrix,
    quaternion_to_matrix,
    reparent_camera_transform,
    select_diverse_samples,
    solve_opencv_handeye,
)


def parse_args(argv=None):
    config = default_calibration_config()
    parser = argparse.ArgumentParser(
        description="Solve body-to-camera extrinsics from a ROS bag and a checkerboard."
    )
    parser.add_argument("--bag", required=True, help="input ROS bag")
    parser.add_argument("--output", required=True, help="output YAML path")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--tf-static-topic", default="/tf_static")
    parser.add_argument("--pattern-cols", type=int, default=config.pattern_cols)
    parser.add_argument("--pattern-rows", type=int, default=config.pattern_rows)
    parser.add_argument("--square-size", type=float, default=config.square_size_m)
    parser.add_argument("--max-time-diff", type=float, default=0.08)
    parser.add_argument("--max-reprojection-error-px", type=float, default=3.0)
    parser.add_argument("--min-translation-m", type=float, default=0.02)
    parser.add_argument("--min-rotation-deg", type=float, default=3.0)
    parser.add_argument("--max-samples", type=int, default=80)
    parser.add_argument("--method", choices=("tsai", "park", "horaud", "andreff", "daniilidis"), default="park")
    parser.add_argument("--camera-optical-frame", default="camera_color_optical_frame")
    parser.add_argument("--camera-link-frame", default="camera_link")
    parser.add_argument("--expected-odom-parent", default="camera_init")
    parser.add_argument("--expected-odom-child", default="body")
    parser.add_argument("--visualize", action="store_true")
    return parser.parse_args(argv)


def _stamp(message):
    return float(message.header.stamp.to_sec())


def _odom_matrix(message):
    pose = message.pose.pose
    return pose_matrix(
        quaternion_to_matrix(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        ),
        [pose.position.x, pose.position.y, pose.position.z],
    )


def _tf_matrix(message):
    transform = message.transform
    return pose_matrix(
        quaternion_to_matrix(
            [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        ),
        [transform.translation.x, transform.translation.y, transform.translation.z],
    )


def _gray_image(bridge, message):
    image = bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
    if image.ndim == 2:
        return image
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def _find_corners(gray, pattern):
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=0)
        if found:
            return corners.astype(np.float64)
        return None
    found, corners = cv2.findChessboardCorners(
        gray,
        pattern,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found:
        return None
    refined = cv2.cornerSubPix(
        gray,
        corners,
        winSize=(11, 11),
        zeroZone=(-1, -1),
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3),
    )
    return refined.astype(np.float64)


def _solve_board_pose(object_points, corners, camera_matrix, distortion):
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        corners,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        return None, None
    rotation, _ = cv2.Rodrigues(rvec)
    pose = pose_matrix(rotation, tvec.reshape(3))
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion
    )
    observed = corners.reshape(-1, 2)
    predicted = projected.reshape(-1, 2)
    error = np.sqrt(np.mean(np.sum((observed - predicted) ** 2, axis=1)))
    return pose, float(error)


def _rotation_angle_deg(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def _select_diverse(samples, min_translation_m, min_rotation_deg, max_samples):
    return select_diverse_samples(
        samples,
        min_translation_m=min_translation_m,
        min_rotation_deg=min_rotation_deg,
        max_samples=max_samples,
    )


def _nearest_odom(stamps, poses, timestamp):
    index = bisect.bisect_left(stamps, timestamp)
    candidates = []
    if index < len(stamps):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    if not candidates:
        return None, None
    best = min(candidates, key=lambda item: abs(stamps[item] - timestamp))
    return abs(stamps[best] - timestamp), poses[best]


def _read_bag_inputs(args):
    odom_stamps = []
    odom_poses = []
    static_edges = []
    camera_info = None
    with rosbag.Bag(args.bag, "r") as bag:
        for topic, message, _ in bag.read_messages(
            topics=[args.odom_topic, args.camera_info_topic, args.tf_static_topic]
        ):
            if topic == args.odom_topic:
                if not odom_stamps:
                    parent = str(message.header.frame_id).lstrip("/")
                    child = str(message.child_frame_id).lstrip("/")
                    if args.expected_odom_parent and parent != args.expected_odom_parent.lstrip("/"):
                        raise RuntimeError(
                            "Odometry parent is {}, expected {}".format(
                                parent, args.expected_odom_parent
                            )
                        )
                    if args.expected_odom_child and child != args.expected_odom_child.lstrip("/"):
                        raise RuntimeError(
                            "Odometry child is {}, expected {}".format(
                                child, args.expected_odom_child
                            )
                        )
                odom_stamps.append(_stamp(message))
                odom_poses.append(_odom_matrix(message))
            elif topic == args.camera_info_topic and camera_info is None:
                camera_info = message
            elif topic == args.tf_static_topic:
                for transform in message.transforms:
                    static_edges.append(
                        (
                            transform.header.frame_id,
                            transform.child_frame_id,
                            _tf_matrix(transform),
                        )
                    )
    if camera_info is None:
        raise RuntimeError("no CameraInfo messages found on {}".format(args.camera_info_topic))
    camera_frame = str(camera_info.header.frame_id).lstrip("/")
    if camera_frame and camera_frame != args.camera_optical_frame.lstrip("/"):
        raise RuntimeError(
            "CameraInfo frame is {}, expected {}".format(
                camera_frame, args.camera_optical_frame
            )
        )
    if len(odom_stamps) < 3:
        raise RuntimeError("need at least 3 Odometry messages")
    return odom_stamps, odom_poses, static_edges, camera_info


def _collect_samples(args, odom_stamps, odom_poses, camera_info):
    bridge = CvBridge()
    object_points = checkerboard_object_points(
        (args.pattern_cols, args.pattern_rows), args.square_size
    ).astype(np.float32)
    camera_matrix = np.asarray(camera_info.K, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(camera_info.D, dtype=np.float64)
    samples = []
    total_images = 0
    with rosbag.Bag(args.bag, "r") as bag:
        for _, message, _ in bag.read_messages(topics=[args.image_topic]):
            total_images += 1
            timestamp = _stamp(message)
            time_diff, body_pose = _nearest_odom(odom_stamps, odom_poses, timestamp)
            if body_pose is None or time_diff > args.max_time_diff:
                continue
            gray = _gray_image(bridge, message)
            corners = _find_corners(gray, (args.pattern_cols, args.pattern_rows))
            if corners is None:
                continue
            board_pose, reprojection_error = _solve_board_pose(
                object_points, corners, camera_matrix, distortion
            )
            if board_pose is None or reprojection_error > args.max_reprojection_error_px:
                continue
            samples.append(
                {
                    "timestamp": timestamp,
                    "time_diff": time_diff,
                    "body_pose": body_pose,
                    "board_pose": board_pose,
                    "reprojection_error": reprojection_error,
                }
            )
            if args.visualize:
                preview = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(
                    preview,
                    (args.pattern_cols, args.pattern_rows),
                    corners.astype(np.float32),
                    True,
                )
                cv2.imshow("checkerboard hand-eye", preview)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
    if args.visualize:
        cv2.destroyAllWindows()
    return samples, total_images


def _quality_metrics(samples, T_body_camera_optical):
    world_board = [sample["body_pose"] @ T_body_camera_optical @ sample["board_pose"] for sample in samples]
    positions = np.asarray([pose[:3, 3] for pose in world_board], dtype=np.float64)
    center = np.mean(positions, axis=0)
    translation_rms = float(np.sqrt(np.mean(np.sum((positions - center) ** 2, axis=1))))
    reference = world_board[0][:3, :3]
    rotation_errors = [
        _rotation_angle_deg(reference.T @ pose[:3, :3]) for pose in world_board
    ]
    return {
        "reprojection_rms_px": float(
            np.sqrt(np.mean([sample["reprojection_error"] ** 2 for sample in samples]))
        ),
        "reprojection_max_px": float(
            max(sample["reprojection_error"] for sample in samples)
        ),
        "target_translation_rms_m": translation_rms,
        "target_rotation_rms_deg": float(np.sqrt(np.mean(np.square(rotation_errors)))),
        "timestamp_diff_max_s": float(max(sample["time_diff"] for sample in samples)),
    }


def run(args):
    if args.pattern_cols != 11 or args.pattern_rows != 8 or abs(args.square_size - 0.04) > 1e-9:
        raise RuntimeError("this calibration is configured for an 11x8, 0.040 m checkerboard")
    odom_stamps, odom_poses, static_edges, camera_info = _read_bag_inputs(args)
    detected_samples, total_images = _collect_samples(
        args, odom_stamps, odom_poses, camera_info
    )
    samples = _select_diverse(
        detected_samples,
        args.min_translation_m,
        args.min_rotation_deg,
        args.max_samples,
    )
    if len(samples) < 3:
        raise RuntimeError(
            "only {} diverse samples from {} valid detections; collect at least 3, "
            "preferably 10-20".format(
                len(samples), len(detected_samples)
            )
        )
    T_body_camera_optical = solve_opencv_handeye(
        [sample["body_pose"] for sample in samples],
        [sample["board_pose"] for sample in samples],
        method=args.method,
    )
    T_camera_link_optical = lookup_transform_from_edges(
        static_edges, args.camera_link_frame, args.camera_optical_frame
    )
    T_body_camera_link = reparent_camera_transform(
        T_body_camera_optical, T_camera_link_optical
    )
    metrics = _quality_metrics(samples, T_body_camera_optical)
    metrics["valid_images"] = float(len(detected_samples))
    metrics["selected_samples"] = float(len(samples))
    metrics["total_images"] = float(total_images)
    document = build_result_document(
        T_body_camera_link,
        default_calibration_config(),
        samples_used=len(samples),
        metrics=metrics,
        frames={"parent_frame": "body", "child_frame": "camera_link"},
        metadata={
            "method": args.method,
            "image_topic": args.image_topic,
            "camera_info_topic": args.camera_info_topic,
            "odom_topic": args.odom_topic,
            "camera_optical_frame": args.camera_optical_frame,
            "camera_link_frame": args.camera_link_frame,
            "odom_parent_frame": args.expected_odom_parent,
            "odom_child_frame": args.expected_odom_child,
            "camera_matrix": [float(value) for value in camera_info.K],
            "distortion": [float(value) for value in camera_info.D],
        },
    )
    output_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(output_dir, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as stream:
        yaml.safe_dump(document, stream, sort_keys=False, allow_unicode=True)
    print("calibration saved to {}".format(args.output))
    print("detected_samples={}".format(len(detected_samples)))
    print("samples_used={}".format(len(samples)))
    for key, value in sorted(metrics.items()):
        print("{}={}".format(key, value))


def main(argv=None):
    args = parse_args(argv)
    try:
        run(args)
    except Exception as exc:
        print("calibration failed: {}".format(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
