from pathlib import Path
import xml.etree.ElementTree as ET


LAUNCH = Path(__file__).resolve().parents[1] / "launch/exp/run_swarm_indoor1_fuel_exploration.launch"
ADVANCED_PARAMS = Path(__file__).resolve().parents[1] / "launch/include/advanced_param_exp.xml"
UAV0_TEST_LAUNCH = (
    Path(__file__).resolve().parents[5]
    / "uav0_competition_bringup/launch/uav0_search_landing_test.launch"
)
TRAJ_CONTAINER = (
    Path(__file__).resolve().parents[2]
    / "traj_utils/include/traj_utils/plan_container.hpp"
)


def test_landing_search_switches_from_fuel_to_diff_with_separate_commands():
    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    assert args["fuel_max_vel"] == "0.20"
    assert args["landing_search_max_vel"] == "0.30"
    assert args["landing_search_max_acc"] == "0.60"
    assert args["enable_search_landing"] == "$(optenv UAV0_ENABLE_SEARCH_LANDING true)"
    nodes = {node.attrib.get("name"): node for node in root.findall("node")}
    assert "fuel_traj_server" in nodes
    assert "landing_diff_traj_server" in nodes
    assert "planner_command_arbiter" in nodes
    assert "landing_diff_search_manager" in nodes
    assert nodes["landing_diff_traj_server"].attrib["if"] == "$(arg enable_search_landing)"
    assert nodes["landing_diff_search_manager"].attrib["if"] == "$(arg enable_search_landing)"

    fuel_remaps = {item.attrib["from"]: item.attrib["to"] for item in nodes["fuel_traj_server"].findall("remap")}
    diff_remaps = {item.attrib["from"]: item.attrib["to"] for item in nodes["landing_diff_traj_server"].findall("remap")}
    assert fuel_remaps["/position_cmd"] == "/UAV0/fuel/planning/pos_cmd"
    assert diff_remaps["position_cmd"] == "/UAV0/diff/planning/pos_cmd"
    assert diff_remaps["~planning/yaw"] == "/UAV0/landing_diff/yaw"

    manager_params = {item.attrib["name"]: item.attrib["value"] for item in nodes["landing_diff_search_manager"].findall("param")}
    assert manager_params["search_height"] == "2.0"
    assert manager_params["subgoal_topic"] == "/UAV0/landing_diff/subgoal"
    assert manager_params["marker_topic"] == "/UAV0/mission/detection/final_aruco"
    assert manager_params["front_hint_topic"] == "/UAV0/landing/front_aruco_hint"
    assert manager_params["assigned_target_topic"] == "/UAV0/landing/assigned_target"
    assert manager_params["enable_single_front_hint_approach"] == "false"
    assert manager_params["yaw_topic"] == "/UAV0/landing_diff/yaw"
    assert manager_params["front_hint_dwell_sec"] == "2.0"
    assert manager_params["front_hint_initial_wait_sec"] == "1.0"
    assert manager_params["front_yaw_scan_half_angle_rad"] == "0.7854"
    assert manager_params["front_yaw_scan_rate_rad_s"] == "0.35"
    assert manager_params["target_frame"] == "world"
    assert manager_params["marker_timeout_sec"] == "0.50"
    assert manager_params["front_hint_timeout_sec"] == "0.50"
    assert manager_params["assigned_target_timeout_sec"] == "0.50"

    manager_source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    assert "ASSIGNED_TARGET_REACHED_WAIT_DOWN_CONFIRMATION" in manager_source
    assert "DIFF_APPROACH_REACHED_LANDING_REQUESTED" in manager_source
    assert manager_source.index("if (have_marker_)") < manager_source.index(
        "else if (have_assigned_target_)"
    )


def test_fuel_declares_external_landing_planner():
    root = ET.parse(LAUNCH).getroot()
    values = {item.attrib.get("param"): (item.text or "").strip() for item in root.findall("rosparam")}
    params = {item.attrib.get("name"): item for item in root.findall("param")}
    assert values["/exploration_node/mission/task_search/exit/external_landing_planner"] == "true"
    assert values["/exploration_node/mission/task_search/exit/enabled"] == "false"
    assert params["/exploration_node/mission/task_search/exit/rc_trigger/enabled"].attrib == {
        "name": "/exploration_node/mission/task_search/exit/rc_trigger/enabled",
        "value": "$(arg enable_search_landing)",
        "type": "bool",
    }
    assert values["/exploration_node/mission/task_search/exit/rc_trigger/topic"] == "/UAV0/mavros/rc/in"
    assert values["/exploration_node/mission/task_search/exit/rc_trigger/channel_index"] == "8"
    assert values["/exploration_node/mission/task_search/exit/rc_trigger/low_pwm"] == "1300"
    assert values["/exploration_node/mission/task_search/exit/rc_trigger/high_pwm"] == "1800"
    assert values["/exploration_node/mission/task_search/exit/rc_trigger/hold_sec"] == "0.5"


def test_follower_frame_alignment_is_not_duplicated_in_planner_launch():
    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"] for item in root.findall("arg")}
    nodes = {node.attrib.get("name") for node in root.findall("node")}
    assert "follower_offset_x" not in args
    assert "follower_offset_y" not in args
    assert "follower_offset_z" not in args
    assert "world_to_UAV1_camera_init" not in nodes


def test_dynamic_obstacle_detection_is_disabled_by_default():
    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    assert args["enable_uav0_dynamic_obstacle_detection"] == (
        "$(optenv UAV0_ENABLE_DYNAMIC_OBSTACLE_DETECTION false)"
    )
    assert args["enable_uav1_dynamic_obstacle_detection"] == (
        "$(optenv UAV1_ENABLE_DYNAMIC_OBSTACLE_DETECTION false)"
    )

    arbiter = next(
        node for node in root.findall("node")
        if node.attrib.get("name") == "planner_command_arbiter"
    )
    params = {item.attrib["name"]: item.attrib["value"] for item in arbiter.findall("param")}
    assert params["dynamic_avoidance_enabled"] == (
        "$(arg enable_uav0_dynamic_obstacle_detection)"
    )

    ldop_include = next(
        item for item in root.findall("include")
        if "run_ldop_uav_pair.launch" in item.attrib.get("file", "")
    )
    include_args = {item.attrib["name"]: item.attrib["value"] for item in ldop_include.findall("arg")}
    assert include_args["enable_uav0"] == "$(arg enable_uav0_dynamic_obstacle_detection)"
    assert include_args["enable_uav1"] == "$(arg enable_uav1_dynamic_obstacle_detection)"


def test_landing_targets_require_fresh_world_frame_and_finite_position():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    assert "target.header.frame_id != target_frame_" in source
    assert "target.header.stamp.isZero()" in source
    assert "age_sec > timeout_sec" in source
    assert "!std::isfinite(point.x)" in source


def test_yaw_scan_starts_from_the_exit_locked_yaw():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    activate = source.split("void activate()", maxsplit=1)[1].split(
        "void buildSweepGoals()", maxsplit=1
    )[0]
    assert activate.index("locked_yaw_ = yaw") < activate.index(
        "front_yaw_scan_yaw_ = locked_yaw_"
    )


def test_validated_front_hint_is_latched_while_approaching():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    expiry = source.split("void expireTargets()", maxsplit=1)[1].split(
        "void publishGoal", maxsplit=1
    )[0]
    assert 'validTarget(marker_, marker_timeout_sec_, "final ArUco")' in expiry
    assert "validTarget(front_hint_" not in expiry


def test_initial_front_scan_does_not_send_zero_length_diff_goal():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/landing_diff_search_manager.cpp"
    ).read_text()
    initial_hold = source.split(
        "if (holdingForInitialFrontSearch())", maxsplit=1
    )[1].split("geometry_msgs::Point goal;", maxsplit=1)[0]
    assert "publishGoal" not in initial_hold


def test_local_trajectory_id_is_not_reset_when_global_goal_changes():
    source = TRAJ_CONTAINER.read_text()
    assert source.count("local_traj.traj_id = 0;") == 1
    set_global = source.split("void setGlobalTraj", maxsplit=1)[1].split(
        "void setLocalTraj", maxsplit=1
    )[0]
    assert "local_traj.traj_id = 0;" not in set_global
    assert "local_traj.traj_id++;" in source
