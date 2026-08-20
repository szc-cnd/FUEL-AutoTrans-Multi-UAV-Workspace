#!/usr/bin/env python3
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "config" / "dual_uav_frame_alignment.yaml"
LAUNCH = ROOT / "launch" / "leader_safe_path_follower.launch"
FOLLOWER = ROOT / "src" / "leader_safe_path_follower.cpp"
PUBLISHER = ROOT / "src" / "dual_uav_frame_alignment.cpp"
RELAY_LAUNCH = (
    ROOT.parent
    / "Diff-Planner/src/diff_planner/plan_manage/launch/exp/run_uav1_relay_diff.launch"
)
FAST_LIO_LAUNCH = ROOT.parent / "FAST_LIO/launch/mapping_mid360.launch"


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


def test_collaboration_launch_has_one_alignment_tf_and_no_second_rviz():
    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    assert args["follower_odom_topic"] == "/UAV1/fast_lio/Odom_high_freq"

    relay_include = next(
        item for item in root.findall("include")
        if "run_uav1_relay_diff.launch" in item.attrib.get("file", "")
    )
    include_args = {
        item.attrib["name"]: item.attrib["value"]
        for item in relay_include.findall("arg")
    }
    assert include_args["publish_world_to_follower_tf"] == "false"
    assert include_args["enable_rviz"] == "false"

    relay_args = {
        item.attrib["name"]: item.attrib["default"]
        for item in ET.parse(RELAY_LAUNCH).getroot().findall("arg")
    }
    assert "publish_world_to_follower_tf" in relay_args
    assert "enable_rviz" in relay_args


def test_fast_lio_imu_adapter_node_name_is_vehicle_specific():
    root = ET.parse(FAST_LIO_LAUNCH).getroot()
    adapter = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "livox_imu_to_body.py"
    )
    assert adapter.attrib["name"] == "$(arg vehicle_ns)_livox_imu_to_body"


def test_uav1_stays_parked_until_uav0_landing_release():
    source = FOLLOWER.read_text(encoding="utf-8")
    release_gate = source.split("void tryReleaseFinalExitWaypoint", 1)[1].split(
        "void finalExitPoseCallback", 1
    )[0]
    assert "!release_uav1_" in release_gate

    timer = source.split("void timerCallback", 1)[1].split(
        "ros::Subscriber", 1
    )[0]
    wait = 'if (leader_outside_exit_ && !release_uav1_)'
    assert wait in timer
    assert 'hold("UAV1 parked until UAV0 finds two ArUcos and lands")' in timer
    assert timer.index(wait) < timer.index("handleStuckRecovery(now)")

    release = source.split("void releaseUav1Callback", 1)[1].split(
        "bool getRouteForwardDirection", 1
    )[0]
    assert 'tryReleaseFinalExitWaypoint("UAV0 landing success release")' in release
