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
UAV1_DIFF_RVIZ = (
    ROOT.parent
    / "Diff-Planner/src/diff_planner/plan_manage/launch/include/exp.rviz"
)
UAV1_SIX_SCRIPT = ROOT.parents[1] / "shfiles/start_uav1_six_terminator.sh"


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


def test_relay_waypoints_release_after_seven_tenths_meter_clearance():
    root = ET.parse(LAUNCH).getroot()
    follower = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "leader_safe_path_follower"
    )
    params = {
        item.attrib["name"]: item.attrib["value"]
        for item in follower.findall("param")
    }
    assert params["follow_distance"] == "0.70"
    assert params["release_path_length"] == "0.70"
    assert params["waypoint_release_min_separation"] == "0.70"
    assert params["door_release_inside_distance"] == "0.70"
    assert params["relay_release_distance"] == "0.70"

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


def test_relay_waypoints_are_cached_and_only_consumed_after_arrival():
    source = FOLLOWER.read_text(encoding="utf-8")
    append = source.split("void appendRelayWaypoint", 1)[1].split(
        "void leaderLandingTargetCallback", 1
    )[0]
    assert "relay_waypoints_.push_back(follower_point)" in append
    publish = source.split("void publishRelayPath", 1)[1].split(
        "void appendRelayWaypoint", 1
    )[0]
    assert "for (const RoutePoint& point : relay_waypoints_)" in publish
    assert "relay_waypoints_.erase" not in source
    assert "relay_waypoints_.pop_back" not in source
    assert "relay_waypoints_.clear" not in source

    continuous = source.split("bool handleContinuousFollowBeforeExit", 1)[1].split(
        "bool handleDiffPlannerExecution", 1
    )[0]
    assert "++active_relay_index_" not in continuous
    assert "CONTINUOUS passed relay waypoint" not in continuous
    assert "SKIP unreachable internal waypoint" not in source
    assert "skipped unreachable internal waypoint" not in source
    assert source.count("++active_relay_index_") == 2


def test_leader_odometry_uses_latest_low_latency_sample_and_rejects_delay():
    source = FOLLOWER.read_text(encoding="utf-8")
    assert "leader_odom_topic_, 1" in source
    assert "ros::TransportHints().tcpNoDelay()" in source
    callback = source.split("void leaderOdomCallback", 1)[1].split(
        "void followerOdomCallback", 1
    )[0]
    assert "transport_age > leader_odom_max_transport_age_" in callback
    assert "reject delayed leader odometry" in callback

    root = ET.parse(LAUNCH).getroot()
    follower = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "leader_safe_path_follower"
    )
    params = {
        item.attrib["name"]: item.attrib["value"]
        for item in follower.findall("param")
    }
    assert params["leader_odom_max_transport_age"] == "0.50"


def test_down_search_releases_uav1_to_front_anchor_after_measured_climb():
    root = ET.parse(LAUNCH).getroot()
    follower = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "leader_safe_path_follower"
    )
    params = {
        item.attrib["name"]: item.attrib["value"]
        for item in follower.findall("param")
    }
    assert params["fixed_follow_height"] == "0.60"
    assert params["landing_search_state_topic"] == (
        "/landing_diff_search_manager/state"
    )
    assert params["front_scan_anchor_topic"] == (
        "/UAV0/landing/front_scan_anchor"
    )
    assert params["down_search_release_height"] == "1.80"
    assert params["down_search_min_vertical_separation"] == "1.00"

    source = FOLLOWER.read_text(encoding="utf-8")
    callback = source.split("void landingSearchStateCallback", 1)[1].split(
        "void tryReleaseFrontSearchWaitWaypoint", 1
    )[0]
    assert "FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP" in callback
    assert "FRONT_ARUCO_HINT_DIFF_APPROACH" in callback
    assert "TWO_ARUCOS_ASSIGNED_APPROACH_FAR_PLATFORM" not in callback

    release = source.split("void tryReleaseFrontSearchWaitWaypoint", 1)[1].split(
        "void publishRelayPath", 1
    )[0]
    assert "leader_world.z" in release
    assert "down_search_release_height_" in release
    assert "down_search_min_vertical_separation_" in release
    assert 'relayWaypointSeparationReady("OUTSIDE_WAIT")' in release
    assert "have_front_scan_anchor_" in release
    assert 'appendRelayWaypoint(front_scan_anchor_, "OUTSIDE_WAIT")' in release
    assert "exit_waypoint_released_ = true" in release

    timer = source.split("void timerCallback", 1)[1].split(
        "void publishTarget", 1
    )[0]
    assert timer.index("tryReleaseFrontSearchWaitWaypoint()") < timer.index(
        "!outside_wait_waypoint_released_"
    )
    assert "DIFF_WAIT_FRONT_SEARCH_ANCHOR" in source


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


def test_collaboration_launch_has_one_alignment_tf_and_enables_uav1_diff_rviz():
    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    assert args["follower_odom_topic"] == "/UAV1/fast_lio/Odom_high_freq"
    assert args["enable_diff_rviz"] == "true"

    relay_include = next(
        item for item in root.findall("include")
        if "run_uav1_relay_diff.launch" in item.attrib.get("file", "")
    )
    include_args = {
        item.attrib["name"]: item.attrib["value"]
        for item in relay_include.findall("arg")
    }
    assert include_args["publish_world_to_follower_tf"] == "false"
    assert include_args["enable_rviz"] == "$(arg enable_diff_rviz)"

    relay_args = {
        item.attrib["name"]: item.attrib["default"]
        for item in ET.parse(RELAY_LAUNCH).getroot().findall("arg")
    }
    assert "publish_world_to_follower_tf" in relay_args
    assert "enable_rviz" in relay_args

    rviz_node = next(
        node for node in ET.parse(RELAY_LAUNCH).getroot().findall("node")
        if node.attrib.get("type") == "rviz"
    )
    assert rviz_node.attrib["name"] == "UAV1_diff_rviz"


def test_uav1_six_starts_down_camera_and_shows_combined_image_in_diff_rviz():
    script = UAV1_SIX_SCRIPT.read_text(encoding="utf-8")
    planner = script.split("run_planner_pane()", 1)[1].split(
        "run_controller_pane()", 1
    )[0]
    assert 'run_uav1_sensor_stack.sh" landing &' in planner
    assert 'wait_for_topic_message "${DOWN_CAMERA_TOPIC}"' in planner
    assert "stop_owned_down_camera" in planner

    rviz = yaml.safe_load(UAV1_DIFF_RVIZ.read_text(encoding="utf-8"))

    def dictionaries(value):
        if isinstance(value, dict):
            yield value
            for child in value.values():
                yield from dictionaries(child)
        elif isinstance(value, list):
            for child in value:
                yield from dictionaries(child)

    image = next(
        item for item in dictionaries(rviz)
        if item.get("Class") == "rviz/Image"
        and item.get("Image Topic") == "/UAV1/landing/combined_debug_image"
    )
    assert image["Enabled"] is True
    assert image["Value"] is True


def test_fast_lio_imu_adapter_node_name_is_vehicle_specific():
    root = ET.parse(FAST_LIO_LAUNCH).getroot()
    adapter = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "livox_imu_to_body.py"
    )
    assert adapter.attrib["name"] == "$(arg vehicle_ns)_livox_imu_to_body"


def test_uav1_only_leaves_front_anchor_for_platform_after_landing_release():
    source = FOLLOWER.read_text(encoding="utf-8")
    release_gate = source.split("void tryReleaseFinalExitWaypoint", 1)[1].split(
        "void finalExitPoseCallback", 1
    )[0]
    assert "!release_uav1_" in release_gate

    timer = source.split("void timerCallback", 1)[1].split(
        "ros::Subscriber", 1
    )[0]
    wait = (
        "if (leader_outside_exit_ && !release_uav1_ && "
        "!outside_wait_waypoint_released_)"
    )
    assert wait in timer
    assert "tryReleaseFrontSearchWaitWaypoint()" in timer
    assert timer.index(wait) < timer.index("handleStuckRecovery(now)")
    assert "DIFF_WAIT_FRONT_SEARCH_ANCHOR" in source

    release = source.split("void releaseUav1Callback", 1)[1].split(
        "bool getRouteForwardDirection", 1
    )[0]
    assert 'tryReleaseFinalExitWaypoint("UAV0 landing success release")' in release
