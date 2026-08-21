from pathlib import Path
import math
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_search_config_uses_calibrated_down_camera_extrinsic():
    config = yaml.safe_load((PACKAGE / "config/landing_search.yaml").read_text())
    transform = config["camera_to_body"]
    assert transform["translation_m"] == [
        0.07383116536212123,
        -0.03902001736630811,
        -0.13346814265233625,
    ]
    quaternion = transform["quaternion_xyzw"]
    assert math.isclose(sum(value * value for value in quaternion), 1.0, abs_tol=1e-9)
    assert config["mission"]["require_stage_gate"] is True
    assert config["topics"]["landing_trigger"] == "/UAV0/need_to_land"
    assert config["handoff"]["approach_height_m"] == 2.0


def test_search_launch_wires_mission_request_to_precision_landing_trigger():
    root = ET.parse(PACKAGE / "launch/precision_landing.launch").getroot()
    args = {arg.attrib["name"]: arg.attrib["default"] for arg in root.findall("arg")}
    assert args["landing_search_config"] == "$(find precision_landing)/config/landing_search.yaml"
    search_nodes = [
        node for node in root.findall("node")
        if node.attrib.get("type") == "landing_search_node"
    ]
    assert len(search_nodes) == 1
    params = {
        param.attrib["name"]: param.attrib["value"]
        for param in search_nodes[0].findall("param")
    }
    assert params["topics/landing_request"] == "$(arg mission_landing_request_topic)"
    assert params["topics/landing_trigger"] == "$(arg trigger_topic)"
    assert params["topics/marker_world"] == "$(arg landing_marker_world_topic)"
    assert params["topics/target_id"] == "$(arg target_id_topic)"
    assert params["topics/assigned_id"] == "$(arg assigned_id_topic)"
    assert params["topics/excluded_id"] == "$(arg excluded_id_topic)"
    assert params["topics/candidates"] == "$(arg candidates_topic)"


def test_competition_entries_explicitly_share_search_extrinsic():
    expected = "$(find precision_landing)/config/landing_search.yaml"
    for relative_path in (
        "../control/launch/leader_safe_path_follower.launch",
        "../uav0_competition_bringup/launch/uav0_detection_landing_stack.launch",
        "../uav0_competition_bringup/launch/uav0_search_landing_test.launch",
    ):
        root = ET.parse((PACKAGE / relative_path).resolve()).getroot()
        precision_includes = [
            include for include in root.iter("include")
            if include.attrib.get("file")
            == "$(find precision_landing)/launch/precision_landing.launch"
        ]
        assert precision_includes
        for include in precision_includes:
            args = {
                arg.attrib["name"]: arg.attrib["value"]
                for arg in include.findall("arg")
            }
            assert args["landing_search_config"] == expected


def test_competition_search_landing_waits_for_uav0_rc_trigger():
    uav0_root = ET.parse(
        (PACKAGE / "../uav0_competition_bringup/launch/uav0_detection_landing_stack.launch").resolve()
    ).getroot()
    uav0_args = {
        arg.attrib["name"]: arg.attrib["default"] for arg in uav0_root.findall("arg")
    }
    assert uav0_args["enable_search_landing"] == "$(optenv UAV0_ENABLE_SEARCH_LANDING true)"
    search_group = next(
        group for group in uav0_root.findall("group")
        if group.attrib.get("if") == "$(arg enable_search_landing)"
    )
    gated_uav0_includes = search_group.findall("include")
    assert any("precision_landing.launch" in item.attrib["file"]
               and item.attrib.get("if") == "$(arg enable_precision_landing)"
               for item in gated_uav0_includes)
    assert any("dual_uav_landing_coordinator.launch" in item.attrib["file"]
               and item.attrib.get("if") == "$(arg enable_dual_uav_landing_coordinator)"
               for item in gated_uav0_includes)

    uav1_root = ET.parse(
        (PACKAGE / "../control/launch/leader_safe_path_follower.launch").resolve()
    ).getroot()
    uav1_args = {
        arg.attrib["name"]: arg.attrib["default"] for arg in uav1_root.findall("arg")
    }
    assert uav1_args["enable_search_landing"] == "$(optenv UAV1_ENABLE_SEARCH_LANDING true)"
    precision_include = next(
        item for item in uav1_root.findall("include")
        if "precision_landing.launch" in item.attrib.get("file", "")
    )
    assert precision_include.attrib["if"] == "$(arg enable_search_landing)"

    follower = next(
        node for node in uav1_root.findall("node")
        if node.attrib.get("type") == "leader_safe_path_follower"
    )
    follower_params = {
        param.attrib["name"]: param.attrib["value"] for param in follower.findall("param")
    }
    assert follower_params["enable_search_landing"] == "$(arg enable_search_landing)"


def test_search_runtime_assignment_resets_old_lock_before_handoff():
    source = (PACKAGE / "src/landing_search_node.cpp").read_text()
    callback = source.split("void assignedIdCallback", maxsplit=1)[1].split(
        "const nav_msgs::Odometry", maxsplit=1
    )[0]
    assert "if (trigger_sent_)" in callback
    assert "tracker_.lockedId() == message->data" in callback
    assert "requested_marker_id_ = message->data" in callback
    assert "tracker_.reset()" in callback
    assert "target_filter_.reset()" in callback
    assert "stable_target_received_ = false" in callback


def test_search_runtime_exclusion_clears_conflicting_lock():
    source = (PACKAGE / "src/landing_search_node.cpp").read_text()
    callback = source.split("void excludedIdCallback", maxsplit=1)[1].split(
        "const nav_msgs::Odometry", maxsplit=1
    )[0]
    assert "tracker_.lockedId() == excluded_marker_id_" in callback
    assert "tracker_.reset()" in callback
    assert "publishTargetId(-1)" in callback
    assert "true, excluded_marker_id_" in source


def test_dual_uav_coordinator_assigns_after_scan_in_detection_order():
    source = (PACKAGE / "src/dual_uav_landing_coordinator_node.cpp").read_text()
    assert "!front_scan_completed_ || candidate_order_.size() < 2U" in source
    assert "downward_ids_.insert(platform.id)" in source
    assert "downward_ids_.count(platform.id) == 0U" in source
    assert "candidate_order_.push_back(platform.id)" in source
    assert "uav1_id_ = uav1_platform.id" in source
    assert "uav0_id_ = uav0_platform.id" in source
    assert "publishBool(release_pub_, true)" in source

    root = ET.parse(
        PACKAGE / "launch/dual_uav_landing_coordinator.launch"
    ).getroot()
    nodes = root.findall("node")
    assert len(nodes) == 1
    assert nodes[0].attrib["type"] == "dual_uav_landing_coordinator_node"
    params = {
        param.attrib["name"]: param.attrib["value"]
        for param in nodes[0].findall("param")
    }
    assert params["topics/front_candidates"] == "/UAV0/landing/front/candidates"
    assert params["topics/search_state"] == "/landing_diff_search_manager/state"
    assert params["topics/uav0_landing_request"] == "/UAV0/mission/landing_request"
    assert params["topics/uav0_target"] == "/UAV0/landing/assigned_target"
    assert params["topics/uav0_target"] != "/UAV0/mission/detection/final_aruco"

    request_callback = source.split(
        "void landingRequestCallback", maxsplit=1
    )[1].split("void tryReleaseUav1", maxsplit=1)[0]
    assert "uav0_landing_requested_ = true" in request_callback
    release = source.split("void tryReleaseUav1", maxsplit=1)[1].split(
        "void successCallback", maxsplit=1
    )[0]
    assert "!assignments_ready_ || !uav0_landing_requested_" in release
    assert "publishBool(release_pub_, true)" in release

    update = source.split("void updateCandidates", maxsplit=1)[1].split(
        "void tryAssignPlatforms", maxsplit=1
    )[0]
    assert "downwardCandidatesAllowed(landing_search_state_)" in update
    assert "frontCandidatesAllowed(landing_search_state_)" in update
    assert "reject stale candidate array" in update
    forward = source.split("static bool frontCandidatesAllowed", maxsplit=1)[1].split(
        "static bool downwardCandidatesAllowed", maxsplit=1
    )[0]
    assert "FRONT_ARUCO_FORWARD_APPROACH" not in forward

    state_callback = source.split("void searchStateCallback", maxsplit=1)[1].split(
        "void updateCandidates", maxsplit=1
    )[0]
    assert 'next_state == "FRONT_ARUCO_FORWARD_APPROACH"' in state_callback
    assert "resetSearchCandidates()" in state_callback
    assert "frontScanCompleted(next_state)" in state_callback
    assert "tryAssignPlatforms()" in state_callback


def test_front_hint_uses_d435_depth_tf_and_separate_coarse_topic():
    config = yaml.safe_load(
        (PACKAGE / "config/front_aruco_hint.yaml").read_text()
    )
    assert config["marker"]["size_m"] == 0.60
    assert config["marker"]["stable_frames"] == 5
    assert config["depth_validation"]["required"] is True
    assert config["frames"]["body"] == "UAV0/body"
    assert config["frames"]["output_world"] == "world"
    assert config["topics"]["hint_world"] == "/UAV0/landing/front_aruco_hint"
    assert config["topics"]["candidates"] == "/UAV0/landing/front/candidates"
    assert config["topics"]["search_state"] == (
        "/landing_diff_search_manager/state"
    )
    assert config["mission"]["require_search_state_gate"] is True
    assert config["topics"]["hint_world"] != config["topics"].get("marker_world")


def test_front_candidates_accumulate_distinct_stable_ids_across_scan():
    source = (PACKAGE / "src/front_aruco_hint_node.cpp").read_text()
    assert "for (const DetectedTarget& detected : tracker_.detectedTargets())" in source
    assert "validateDepth(observation, image_header.stamp" in source
    assert "candidate_filters_.find(detected.id)" in source
    assert "stable_candidates_[detected.id] = platform" in source
    assert "stable_candidates_.clear()" in source
    assert "LandingPlatformArray" in source


def test_precision_launch_wires_optional_front_hint_without_final_marker_access():
    root = ET.parse(PACKAGE / "launch/precision_landing.launch").getroot()
    nodes = [
        node for node in root.findall("node")
        if node.attrib.get("type") == "front_aruco_hint_node"
    ]
    assert len(nodes) == 1
    params = {
        param.attrib["name"]: param.attrib["value"]
        for param in nodes[0].findall("param")
    }
    assert params["topics/image"] == "$(arg front_image_topic)"
    assert params["topics/aligned_depth"] == "$(arg front_depth_topic)"
    assert params["topics/odometry"] == "$(arg odometry_topic)"
    assert params["topics/search_state"] == "/landing_diff_search_manager/state"
    assert params["topics/hint_world"] == "$(arg front_aruco_hint_topic)"
    assert params["topics/candidates"] == "$(arg front_aruco_candidates_topic)"
    assert "topics/marker_world" not in params
    assert "topics/landing_trigger" not in params


def test_front_detection_waits_until_forward_approach_is_complete():
    source = (PACKAGE / "src/front_aruco_hint_node.cpp").read_text()
    gate = source.split("bool stageAllowsHint() const", maxsplit=1)[1].split(
        "static bool isFrontScanState", maxsplit=1
    )[0]
    assert "front_scan_active_" in gate
    states = source.split("static bool isFrontScanState", maxsplit=1)[1].split(
        "void searchStateCallback", maxsplit=1
    )[0]
    assert "FRONT_ARUCO_FORWARD_APPROACH" not in states
    assert "FRONT_ARUCO_INITIAL_WAIT" in states
    assert "FRONT_ARUCO_YAW_SCAN_LEFT" in states
    assert "FRONT_ARUCO_YAW_SCAN_RIGHT" in states
    callback = source.split("void searchStateCallback", maxsplit=1)[1].split(
        "void missionStatusCallback", maxsplit=1
    )[0]
    assert "tracker_.reset()" in callback
    assert "stable_candidates_.clear()" in callback
    cmake = (PACKAGE / "CMakeLists.txt").read_text()
    dependency = cmake.split(
        "add_executable(front_aruco_hint_node", maxsplit=1
    )[1].split("target_link_libraries(front_aruco_hint_node", maxsplit=1)[0]
    assert "add_dependencies(front_aruco_hint_node" in dependency
    assert "${${PROJECT_NAME}_EXPORTED_TARGETS}" in dependency


def test_precision_launch_records_aruco_decisions_on_every_start():
    root = ET.parse(PACKAGE / "launch/precision_landing.launch").getroot()
    args = {arg.attrib["name"]: arg.attrib for arg in root.findall("arg")}
    assert args["enable_aruco_rosbag"]["default"] == "true"
    assert args["aruco_rosbag_prefix"]["default"] == (
        "$(env HOME)/.ros/aruco_detection_$(arg vehicle_ns)"
    )

    recorders = [
        node for node in root.findall("node")
        if node.attrib.get("pkg") == "rosbag"
        and node.attrib.get("type") == "record"
    ]
    assert len(recorders) == 1
    recorder = recorders[0]
    assert recorder.attrib["if"] == "$(arg enable_aruco_rosbag)"
    assert recorder.attrib["ns"] == "$(arg vehicle_ns)"

    record_args = recorder.attrib["args"]
    required_topics = {
        "$(arg front_aruco_candidates_topic)",
        "$(arg topic_prefix)/landing/front/status",
        "$(arg candidates_topic)",
        "$(arg topic_prefix)/landing/search/status",
        "$(arg assigned_id_topic)",
        "$(arg mission_status_topic)",
        "$(arg landing_marker_world_topic)",
        "$(arg odometry_topic)",
        "/dual_uav_landing/status",
        "/landing_diff_search_manager/state",
        "/UAV0/landing/front_scan_anchor",
        "/tf",
        "/tf_static",
        "/rosout_agg",
    }
    for topic in required_topics:
        assert topic in record_args
    assert "$(arg topic_prefix)/landing/front/debug_image" not in record_args
    assert "$(arg topic_prefix)/landing/search/debug_image" not in record_args


def test_precision_launch_combines_downward_debug_views():
    root = ET.parse(PACKAGE / "launch/precision_landing.launch").getroot()
    nodes = [
        node for node in root.findall("node")
        if node.attrib.get("type") == "landing_debug_image_mux_node"
    ]
    assert len(nodes) == 1
    params = {
        param.attrib["name"]: param.attrib["value"]
        for param in nodes[0].findall("param")
    }
    assert params == {
        "search_image_topic": "$(arg topic_prefix)/landing/search/debug_image",
        "precision_image_topic": "$(arg topic_prefix)/landing/debug_image",
        "trigger_topic": "$(arg trigger_topic)",
        "output_topic": "$(arg topic_prefix)/landing/combined_debug_image",
    }

    rviz_config = (
        PACKAGE.parent
        / "zyc_fuel_ws/src/FUEL/fuel_planner/plan_manage/config/traj.rviz"
    ).read_text()
    assert rviz_config.count(
        "Image Topic: /UAV0/landing/combined_debug_image"
    ) == 1
    assert "Name: landing_down_search_image" not in rviz_config
    assert "Name: precision_landing_image" not in rviz_config
