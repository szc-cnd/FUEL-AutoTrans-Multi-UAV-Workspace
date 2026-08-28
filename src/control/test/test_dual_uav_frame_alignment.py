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
    / "Diff-Planner/src/diff_planner/plan_manage/launch/include/uav1_lite.rviz"
)
RUN_SWARM_RVIZ = (
    ROOT.parent
    / "Diff-Planner/src/diff_planner/plan_manage/launch/include/exp.rviz"
)
UAV1_SIX_SCRIPT = ROOT.parents[1] / "shfiles/start_uav1_six_terminator.sh"
UAV1_SEVEN_SCRIPT = (
    ROOT.parents[1] / "shfiles/start_uav1_seven_terminator.sh"
)
UAV1_SEVEN_LAYOUT = ROOT.parents[1] / "shfiles/terminator_uav1_six.conf"
UAV1_SENSOR_STACK = ROOT.parents[1] / "shfiles/run_uav1_sensor_stack.sh"
UAV1_DETECTION_LAUNCH = (
    ROOT.parent
    / "uav0_competition_bringup/launch/uav1_detection_stack.launch"
)


def test_uav1_seven_uses_local_master_and_standalone_diff():
    script = UAV1_SEVEN_SCRIPT.read_text(encoding="utf-8")
    planner = script.split("run_planner_pane()", 1)[1].split(
        "run_controller_pane()", 1
    )[0]
    rviz = RUN_SWARM_RVIZ.read_text(encoding="utf-8")
    rviz_config = yaml.safe_load(rviz)

    assert 'LOCAL_MASTER_IP="${UAV1_SINGLE_ROS_MASTER_IP:-10.54.87.232}"' in script
    assert 'export ROS_MASTER_URI="http://${LOCAL_MASTER_IP}:11311"' in script
    assert 'export ROS_IP="${LOCAL_ROS_IP}"' in script
    assert "start_or_reuse_local_master" in script
    assert "roscore" in script
    assert "roslaunch diff_planner run_swarm.launch" in script
    assert "run_swarm.launch enable_rviz:=false" not in script
    assert "launch/include/uav1_lite.rviz" not in script
    assert 'PLANNER_HEARTBEAT_TOPIC="/drone_1_traj_server/heartbeat"' in script
    assert 'run_uav1_sensor_stack.sh" landing &' in script
    assert 'wait_for_topic_message "${DOWN_CAMERA_TOPIC}"' not in planner
    assert "下视相机为可选显示，不作为 Diff 启动条件" in planner
    assert "stop_owned_down_camera" in script
    assert "leader_safe_path_follower.launch" not in script
    assert "/UAV0/" not in script
    assert "/UAV1/landing/control_owner" not in script
    assert "setpoint_topic:=/UAV1/mavros/setpoint_raw/attitude" in script
    for topic in (
        "/UAV1/target_reporting/markers",
        "/UAV1/color_tag_detector/debug_image",
        "/UAV1/vision/qr_debug_image",
        "/UAV1/thermal/debug_image",
        "/UAV1/landing/front/debug_image",
        "/UAV1/down_camera/image_raw",
    ):
        assert topic in rviz

    def dictionaries(value):
        if isinstance(value, dict):
            yield value
            for child in value.values():
                yield from dictionaries(child)
        elif isinstance(value, list):
            for child in value:
                yield from dictionaries(child)

    displays = list(dictionaries(rviz_config))
    for topic in (
        "/drone__diff_planner_node/goal_point",
        "/diff_planner_node/global_list",
    ):
        marker = next(item for item in displays if item.get("Marker Topic") == topic)
        assert marker["Enabled"] is False
        assert marker["Value"] is False


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


def test_leader_height_does_not_filter_follower_xy_targets():
    launch = LAUNCH.read_text(encoding="utf-8")
    source = FOLLOWER.read_text(encoding="utf-8")

    assert "leader_route_height_tolerance" not in launch
    assert "normal_cruise_height" not in source
    assert "leader was outside nominal" not in source

    selection = source.split("auto accept_candidate", 1)[1].split(
        "// UAV0确实到达并停稳的B-spline段终点优先", 1
    )[0]
    assert "useFollowerCruiseHeight(worldToFollower(candidate.position))" in selection


def test_relay_waypoints_release_after_xy_clearance():
    root = ET.parse(LAUNCH).getroot()
    follower = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "leader_safe_path_follower"
    )
    params = {
        item.attrib["name"]: item.attrib["value"]
        for item in follower.findall("param")
    }
    assert params["follow_distance"] == "1.50"
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
    assert "std::hypot(leader_world.x - follower_world.x" in gate
    assert "leader_world.z" not in gate
    assert "follower_world.z" not in gate
    assert 'relayWaypointSeparationReady("DOOR")' in source
    assert 'relayWaypointSeparationReady("INTERNAL")' in source
    assert 'relayWaypointSeparationReady("EXIT")' in source


def test_diff_execution_continuously_holds_and_retreats_for_uav_spacing():
    root = ET.parse(LAUNCH).getroot()
    follower = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "leader_safe_path_follower"
    )
    params = {
        item.attrib["name"]: item.attrib["value"]
        for item in follower.findall("param")
    }
    assert params["enable_diff_separation_safety"] == "true"
    assert params["min_separation"] == "0.70"
    assert params["separation_recovery_distance"] == "0.60"
    assert params["separation_release_distance"] == "0.80"

    source = FOLLOWER.read_text(encoding="utf-8")
    fifo_spacing = source.split(
        "bool handleSimpleDiffPlannerExecution", 1
    )[1].split("const bool terminal_arrival", 1)[0]
    assert "std::hypot(leader_world.x - desired_world.position.x" in fifo_spacing
    assert "waypoint_release_min_separation_" in fifo_spacing
    assert "std::hypot(leader_world.x - current_world.x" in fifo_spacing
    assert "leader_world.z" not in fifo_spacing
    assert "follower_world.z" not in fifo_spacing

    safety = source.split("bool handleDiffSeparationSafety", 1)[1].split(
        "bool handleDiffPlannerExecution", 1
    )[0]
    assert "std::hypot(leader_world.x - follower_world.x" in safety
    assert "separation >= min_separation_" in safety
    assert "separation <= separation_recovery_distance_" in safety
    assert "separation >= separation_release_distance_" in safety
    assert "setDiffWaitPositionHold(true" in safety
    assert "getSeparationRecoveryTarget" in safety
    assert "diff_goal_pub_.publish(goal)" in safety

    execution = source.split("bool handleDiffPlannerExecution", 1)[1].split(
        "void timerCallback", 1
    )[0]
    assert execution.index("handleDiffSeparationSafety(now)") < execution.index(
        "active_relay_index_ >= relay_waypoints_.size()"
    )
    status = source.split("void diffStatusCallback", 1)[1].split(
        "bool getLaggedTarget", 1
    )[0]
    assert "ignore stale trajectory during separation hold" in status
    assert "separation retreat trajectory published" in status
    assert "accepted_improves_separation" in status


def test_diff_recovery_subgoal_cannot_complete_relay_and_status_is_sequenced():
    source = FOLLOWER.read_text(encoding="utf-8")
    execution = source.split("bool handleDiffPlannerExecution", 1)[1].split(
        "void timerCallback", 1
    )[0]
    assert "const bool normal_relay_arrival_enabled" in execution
    assert "!diff_recovery_requested_" in execution
    assert "!diff_recovery_retreat_requested_" in execution
    assert "!diff_recovery_goal_valid_" in execution
    assert "normal_relay_arrival_enabled && position_reached" in execution
    assert "normal_relay_arrival_enabled && !terminal_relay" in execution

    status = source.split("void diffStatusCallback", 1)[1].split(
        "bool getLaggedTarget", 1
    )[0]
    assert 'const std::string stamp_prefix = "goal_stamp_ns="' in status
    assert "response_goal_stamp_ns == diff_active_goal_stamp_ns_" in status
    assert "response_matches_active_goal" in status
    assert "late_success_for_active_goal" in status
    assert "ignore stale UAV1 Diff status" in status
    # 旧路线执行、间距退让和简化FIFO直达各自发布带时间戳的Diff目标。
    assert source.count("stampDiffGoalId(&goal);") == 3


def test_internal_diff_relay_uses_verified_route_subgoals_without_consuming_relay():
    source = FOLLOWER.read_text(encoding="utf-8")
    execution = source.split("bool handleDiffPlannerExecution", 1)[1].split(
        "void timerCallback", 1
    )[0]
    assert "selectForwardRouteCandidate(followerToWorld(current_local), desired_world" in execution
    assert "diff_route_subgoal_local_ = candidate" in execution
    assert "diff_route_subgoal_valid_ ? diff_route_subgoal_local_ : desired_local" in execution
    assert '"nonterminal relay requires verified history candidate"' in execution

    intermediate = execution.split(
        "if ((diff_route_subgoal_valid_ &&", 1
    )[1].split("if (terminal_relay)", 1)[0]
    assert "++active_relay_index_" not in intermediate
    assert "diff_goal_published_ = false" in intermediate
    assert "diff_accepted_goal_valid_ = false" in intermediate


def test_diff_clipped_endpoint_cannot_complete_original_relay():
    root = ET.parse(LAUNCH).getroot()
    follower = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "leader_safe_path_follower"
    )
    params = {
        item.attrib["name"]: item.attrib["value"]
        for item in follower.findall("param")
    }
    assert params["diff_accepted_goal_tolerance"] == "0.20"

    source = FOLLOWER.read_text(encoding="utf-8")
    execution = source.split("bool handleDiffPlannerExecution", 1)[1].split(
        "void timerCallback", 1
    )[0]
    assert "distance3d(diff_accepted_goal_local_, command_local)" in execution
    assert "diff_accepted_goal_tolerance_" in execution
    assert "clipped endpoint reached; choose flexible retry" in execution
    assert "diff_clipped_retry_count_ >= diff_clipped_retry_limit_" in execution
    assert "blacklistRouteCandidate" in execution


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
    simple_execution = source.split(
        "bool handleSimpleDiffPlannerExecution", 1
    )[1].split("bool getLaggedTarget", 1)[0]
    consume = source.split("void consumeSimpleRelayFront", 1)[1].split(
        "bool handleSimpleDiffPlannerExecution", 1
    )[0]
    assert "consumeSimpleRelayFront();" in simple_execution
    assert "relay_waypoints_.erase" in consume
    assert simple_execution.index("UAV1 FIFO %s") < simple_execution.index(
        "consumeSimpleRelayFront();"
    )
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


def test_diff_terminal_clearance_is_optional_and_nonterminal_uses_candidate_search():
    root = ET.parse(LAUNCH).getroot()
    follower = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "leader_safe_path_follower"
    )
    params = {
        item.attrib["name"]: item.attrib["value"]
        for item in follower.findall("param")
    }
    assert params["enable_relay_goal_clearance"] == "false"
    assert params["relay_goal_clearance_radius"] == "0.20"
    assert params["relay_goal_clearance_min_points"] == "2"
    assert params["relay_goal_backtrack_max_distance"] == "1.00"

    source = FOLLOWER.read_text(encoding="utf-8")
    selector = source.split("bool chooseClearRelayGoal", 1)[1].split(
        "void publishCommand", 1
    )[0]
    assert "route_.rbegin()" in selector
    assert "relay_goal_backtrack_max_distance_" in selector
    assert "previous_progress + path_sample_spacing_" in selector
    assert "relayGoalHasClearance" in selector

    execution = source.split("bool handleDiffPlannerExecution", 1)[1].split(
        "void timerCallback", 1
    )[0]
    terminal_adjustment = execution.split(
        "if (enable_relay_goal_clearance_ && terminal_relay", 1
    )[1].split("const geometry_msgs::Point desired_local", 1)[0]
    assert "relay_waypoints_[active_relay_index_] = selected_world" in terminal_adjustment
    assert "++active_relay_index_" not in terminal_adjustment
    assert "selectForwardRouteCandidate" in execution


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


def test_history_snapshot_does_not_reject_leader_route_sample_gaps():
    root = ET.parse(LAUNCH).getroot()
    follower = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "leader_safe_path_follower"
    )
    params = {
        item.attrib["name"]: item.attrib["value"]
        for item in follower.findall("param")
    }
    assert params["leader_history_path_topic"] == "$(arg leader_history_path_topic)"
    assert "history_path_join_tolerance" not in params

    source = FOLLOWER.read_text(encoding="utf-8")
    callback = source.split("void leaderHistoryPathCallback", 1)[1].split(
        "void leaderTrajectoryCallback", 1
    )[0]
    assert "history_path_join_tolerance" not in callback
    assert "appendAcceptedRoutePoint(&sample)" in callback
    assert "\n    route_.clear" not in callback

    odom_callback = source.split("void leaderOdomCallback", 1)[1].split(
        "void followerOdomCallback", 1
    )[0]
    assert "reject direct UAV0 odometry gap" not in odom_callback

    timer = source.split("void timerCallback", 1)[1].split(
        "void publishTarget", 1
    )[0]
    assert 'hold("leader odometry stale")' not in timer
    assert "continue only along cached connected history" in timer


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


def test_collaboration_launch_owns_unique_uav1_visualization():
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
    assert include_args["enable_rviz"] == "false"

    relay_args = {
        item.attrib["name"]: item.attrib["default"]
        for item in ET.parse(RELAY_LAUNCH).getroot().findall("arg")
    }
    assert "publish_world_to_follower_tf" in relay_args
    assert "enable_rviz" in relay_args

    throttle = next(
        node for node in root.findall("node")
        if node.attrib.get("name") == "uav1_visualization_odom_throttle"
    )
    assert throttle.attrib["pkg"] == "topic_tools"
    assert throttle.attrib["type"] == "throttle"
    assert throttle.attrib["args"] == (
        "messages /UAV1/fast_lio/Odom_high_freq 30.0 "
        "/UAV1/visualization/odom_30hz"
    )

    visualizer = next(
        node for node in root.findall("node")
        if node.attrib.get("name") == "uav1_local_odom_visualization"
    )
    remaps = {
        item.attrib["from"]: item.attrib["to"]
        for item in visualizer.findall("remap")
    }
    assert remaps["~odom"] == "/UAV1/visualization/odom_30hz"
    assert remaps["~path"] == "/drone_1_odom_visualization/path"
    assert remaps["~robot"] == "/drone_1_odom_visualization/robot"

    rviz_node = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "rviz"
    )
    assert rviz_node.attrib["name"] == "UAV1_diff_rviz"
    rviz_env = {
        item.attrib["name"]: item.attrib["value"]
        for item in rviz_node.findall("env")
    }
    assert rviz_env["GALLIUM_DRIVER"] == "softpipe"
    assert rviz_env["LP_NUM_THREADS"] == "1"
    assert rviz_env["MESA_GLTHREAD"] == "false"


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


def test_uav1_compatibility_script_opens_seven_panes_with_detection_before_planner():
    script = UAV1_SIX_SCRIPT.read_text(encoding="utf-8")
    layout = UAV1_SEVEN_LAYOUT.read_text(encoding="utf-8")
    assert "run_detection_pane()" in script
    assert "uav1_detection_stack.launch" in script
    assert 'enable_thermal:="${THERMAL}"' in script
    assert layout.count("type = Terminal") == 7
    pane_order = [
        "mavros",
        "mid360",
        "fastlio",
        "pose",
        "detection",
        "planner",
        "controller",
    ]
    positions = [layout.index("--pane {}".format(pane)) for pane in pane_order]
    assert positions == sorted(positions)


def test_uav1_detection_stack_uses_isolated_topics_and_only_observes_front_aruco():
    root = ET.parse(UAV1_DETECTION_LAUNCH).getroot()
    args = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    assert args["enable_thermal"] == "true"
    assert args["odom_topic"] == "/UAV1/fast_lio/Odom_high_freq"
    assert args["world_frame"] == "UAV1/camera_init"
    assert args["camera_namespace"] == "UAV1/camera"
    assert args["camera_tf_prefix"] == "UAV1_camera"

    realsense = next(
        include for include in root.findall("include")
        if "realsense2_camera" in include.attrib.get("file", "")
    )
    realsense_args = {
        item.attrib["name"]: item.attrib["value"] for item in realsense.findall("arg")
    }
    assert realsense_args["color_fps"] == "15"
    assert realsense_args["depth_fps"] == "15"

    nodes = list(root.iter("node"))
    by_type = {node.attrib["type"]: node for node in nodes}
    for node_type in (
        "color_tag_detector.py",
        "qr_detector_node.py",
        "front_aruco_hint_node",
        "target_reporter_node.py",
        "target_rviz_marker_node.py",
    ):
        assert node_type in by_type

    front = by_type["front_aruco_hint_node"]
    front_params = {
        item.attrib["name"]: item.attrib["value"] for item in front.findall("param")
    }
    assert front_params["topics/debug_image"] == "/UAV1/landing/front/debug_image"
    assert front_params["topics/hint_world"] == "/UAV1/detection/front_aruco_hint"
    assert front_params["mission/require_stage_gate"] == "false"

    marker = by_type["target_rviz_marker_node.py"]
    marker_params = {
        item.attrib["name"]: item.attrib["value"] for item in marker.findall("param")
    }
    assert marker_params["marker_topic"] == "/UAV1/target_reporting/markers"
    assert marker_params["enable_object_cloud"] == "true"
    assert marker_params["object_cloud_topic"] == (
        "/UAV1/target_reporting/detected_object_cloud"
    )
    assert marker_params["object_box_topic"] == (
        "/UAV1/target_reporting/detected_object_boxes"
    )


def test_uav1_down_camera_waits_for_busy_device_and_retries_without_killing_owner():
    script = UAV1_SENSOR_STACK.read_text(encoding="utf-8")
    landing = script.split("run_landing()", 1)[1].split(
        '[ "$#" -eq 1 ]', 1
    )[0]
    assert "UAV1_DOWN_CAMERA_START_TIMEOUT" in script
    assert 'fuser "$camera_device"' in landing
    assert "camera_launch_attempt=$((camera_launch_attempt + 1))" in landing
    assert "while [ \"$(date +%s)\" -lt \"$camera_start_deadline\" ]" in landing
    assert 'kill ' not in landing


def test_uav1_diff_rviz_shows_throttled_odometry_and_moving_drone_mesh():
    rviz = yaml.safe_load(UAV1_DIFF_RVIZ.read_text(encoding="utf-8"))

    def dictionaries(value):
        if isinstance(value, dict):
            yield value
            for child in value.values():
                yield from dictionaries(child)
        elif isinstance(value, list):
            for child in value:
                yield from dictionaries(child)

    displays = list(dictionaries(rviz))
    odometry = next(
        item for item in displays
        if item.get("Class") == "rviz/Odometry"
        and item.get("Topic") == "/UAV1/visualization/odom_30hz"
    )
    assert odometry["Enabled"] is True
    assert odometry["Value"] is True

    robot = next(
        item for item in displays
        if item.get("Class") == "rviz/Marker"
        and item.get("Marker Topic") == "/drone_1_odom_visualization/robot"
    )
    assert robot["Enabled"] is True
    assert robot["Value"] is True

    relay_root = ET.parse(RELAY_LAUNCH).getroot()
    visualizer = next(
        node for node in relay_root.findall("node")
        if node.attrib.get("type") == "odom_visualization"
    )
    remaps = {
        item.attrib["from"]: item.attrib["to"]
        for item in visualizer.findall("remap")
    }
    params = {
        item.attrib["name"]: item.attrib["value"]
        for item in visualizer.findall("param")
    }
    assert remaps["~odom"] == "$(arg odom_topic)"
    assert params["robot_scale"] == "0.35"


def test_fast_lio_imu_adapter_node_name_is_vehicle_specific():
    root = ET.parse(FAST_LIO_LAUNCH).getroot()
    adapter = next(
        node for node in root.findall("node")
        if node.attrib.get("type") == "livox_imu_to_body.py"
    )
    assert adapter.attrib["name"] == "$(arg vehicle_ns)_livox_imu_to_body"


def test_uav1_reaches_door_and_outside_hover_before_platform_release():
    source = FOLLOWER.read_text(encoding="utf-8")
    release_gate = source.split("void tryReleaseFinalExitWaypoint", 1)[1].split(
        "void finalExitPoseCallback", 1
    )[0]
    assert "!release_uav1_" not in release_gate
    assert 'appendRelayWaypoint(confirmed_exit_, "EXIT")' in release_gate
    assert 'appendRelayWaypoint(outside_wait, "OUTSIDE_WAIT")' in release_gate
    assert "outside_door_distance_ * std::cos(confirmed_exit_.yaw)" in release_gate
    assert "outside_door_distance_ * std::sin(confirmed_exit_.yaw)" in release_gate

    release = source.split("void releaseUav1Callback", 1)[1].split(
        "bool getRouteForwardDirection", 1
    )[0]
    assert "release_uav1_ = true" in release
    assert "tryReleaseFinalExitWaypoint" not in release
    assert "tryQueueFollowerTerminalTarget()" in release

    terminal_gate = source.split("void tryQueueFollowerTerminalTarget", 1)[1].split(
        "void queueFollowerTerminalTarget", 1
    )[0]
    assert "!release_uav1_ || !outside_wait_arrived_" in terminal_gate
    assert "!have_assigned_follower_target_" in terminal_gate

    execution = source.split("bool handleDiffPlannerExecution", 1)[1].split(
        "void timerCallback", 1
    )[0]
    assert "outside_wait_relay" in execution
    assert "history_relay = !terminal_relay && !outside_wait_relay" in execution
    assert "active_relay_index_ != exit_waypoint_index_" in execution
    assert "outside_wait_arrived_ = true" in execution
    assert "DIFF_WAIT_RELEASE_OUTSIDE_DOOR" in source

    selector = source.split("bool selectForwardRouteCandidate", 1)[1].split(
        "bool getFollowerHistoryRetreatTarget", 1
    )[0]
    assert "allow_beyond_relay" in selector
    assert "? route_.back().progress : relay_progress" in selector
