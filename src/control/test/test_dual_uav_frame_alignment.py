#!/usr/bin/env python3
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "config" / "dual_uav_frame_alignment.yaml"
LAUNCH = ROOT / "launch" / "leader_safe_path_follower.launch"
FOLLOWER = ROOT / "src" / "leader_safe_path_follower.cpp"
PUBLISHER = ROOT / "src" / "dual_uav_frame_alignment.cpp"


def test_single_alignment_config_is_used_by_tf_and_follower():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "dual_uav_frame_alignment"
    ]
    assert config["mission_frame"] == "world"
    assert config["follower_frame"] == "UAV1/camera_init"
    assert config["follower"] == {
        "x": -1.2,
        "y": 0.0,
        "z": 0.0,
        "yaw_rad": 0.0,
    }

    launch = LAUNCH.read_text(encoding="utf-8")
    assert "dual_uav_frame_alignment.yaml" in launch
    assert 'type="dual_uav_frame_alignment"' in launch
    assert "follower_offset_x" not in launch


def test_waypoint_conversion_uses_rigid_transform_and_inverse():
    source = FOLLOWER.read_text(encoding="utf-8")
    assert 'alignment_prefix + "/follower/yaw_rad"' in source
    assert "follower_alignment_cos_ * local.x - follower_alignment_sin_ * local.y" in source
    assert "follower_alignment_cos_ * dx + follower_alignment_sin_ * dy" in source
    assert "worldYawToFollower" in source
    assert "follower_offset_x" not in source

    publisher = PUBLISHER.read_text(encoding="utf-8")
    assert 'prefix + "/follower/x"' in publisher
    assert "StaticTransformBroadcaster" in publisher


def test_relay_waypoints_require_one_meter_actual_separation():
    root = ET.parse(LAUNCH).getroot()
    follower = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "leader_safe_path_follower"
    )
    params = {
        item.attrib["name"]: item.attrib["value"]
        for item in follower.findall("param")
    }
    assert params["follow_distance"] == "1.00"
    assert params["release_path_length"] == "1.00"
    assert params["waypoint_release_min_separation"] == "1.00"
    assert params["door_release_inside_distance"] == "1.00"
    assert params["relay_release_distance"] == "1.00"

    source = FOLLOWER.read_text(encoding="utf-8")
    gate = source.split("bool relayWaypointSeparationReady", 1)[1].split(
        "geometry_msgs::Point useFollowerCruiseHeight", 1
    )[0]
    assert "have_leader_odom_" in gate
    assert "have_follower_odom_" in gate
    assert "followerToWorld" in gate
    assert "waypoint_release_min_separation_" in gate
    assert 'relayWaypointSeparationReady("DOOR")' in source
    assert 'relayWaypointSeparationReady("INTERNAL")' in source
    assert 'relayWaypointSeparationReady("EXIT")' in source


def test_uav1_dynamic_obstacle_detection_is_disabled_by_default():
    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    assert args["enable_dynamic_obstacle_detection"] == (
        "$(optenv UAV1_ENABLE_DYNAMIC_OBSTACLE_DETECTION false)"
    )
    follower = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "leader_safe_path_follower"
    )
    params = {
        item.attrib["name"]: item.attrib["value"]
        for item in follower.findall("param")
    }
    assert params["enable_dynamic_obstacle_detection"] == (
        "$(arg enable_dynamic_obstacle_detection)"
    )

    source = FOLLOWER.read_text(encoding="utf-8")
    assert "if (enable_dynamic_obstacle_detection_)" in source
    assert "if (!enable_dynamic_obstacle_detection_) return;" in source
    assert "bool enable_dynamic_obstacle_detection_{false};" in source
