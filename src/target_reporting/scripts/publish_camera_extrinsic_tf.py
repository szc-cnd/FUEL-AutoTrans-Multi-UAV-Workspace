#!/usr/bin/env python3
"""Publish a validated body-to-camera_link calibration YAML as static TF."""

import argparse

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped

from target_reporting.runtime_tf import validate_calibration_document


def _args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration-file", help="calibration YAML path")
    return parser.parse_args(argv)


def main():
    rospy.init_node("publish_camera_extrinsic_tf")
    calibration_file = rospy.get_param("~calibration_file", "")
    if not calibration_file:
        calibration_file = _args(rospy.myargv()[1:]).calibration_file
    if not calibration_file:
        rospy.logfatal("~calibration_file is required")
        return 2
    try:
        with open(calibration_file, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        translation, quaternion = validate_calibration_document(document)
    except Exception as exc:
        rospy.logfatal("Invalid calibration file %s: %s", calibration_file, exc)
        return 2
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = document["parent_frame"].lstrip("/")
    transform.child_frame_id = document["child_frame"].lstrip("/")
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transform)
    rospy.loginfo(
        "Published calibrated TF %s -> %s from %s",
        transform.header.frame_id,
        transform.child_frame_id,
        calibration_file,
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
