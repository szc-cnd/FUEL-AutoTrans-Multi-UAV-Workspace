#!/usr/bin/env python3
"""Contract checks for AutoTrans front-camera yaw-only search handling."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
FSM = (PACKAGE / "src/mpc_fsm.cpp").read_text(encoding="utf-8")
CONTROLLER = (PACKAGE / "src/mpc_controller.cpp").read_text(encoding="utf-8")
CONTROLLER_HEADER = (
    PACKAGE / "include/payload_mpc_controller/mpc_controller.h"
).read_text(encoding="utf-8")
NODE = (PACKAGE / "src/mpc_controller_node.cpp").read_text(encoding="utf-8")
LAUNCH = PACKAGE / "launch/quad_wind_mpc_controller.launch"


def test_front_scan_states_activate_autotrans_yaw_hold():
    for state in (
        "FRONT_ARUCO_INITIAL_WAIT",
        "FRONT_ARUCO_YAW_SCAN_LEFT",
        "FRONT_ARUCO_YAW_SCAN_RIGHT",
        "FRONT_ARUCO_YAW_SCAN_RETURN",
    ):
        assert f'state == "{state}"' in FSM
    assert "landing_search_yaw_active_ = active" in FSM


def test_yaw_hold_discards_old_translation_and_locks_current_xyz():
    hold = FSM.split("void MPCFSM::processLandingSearchYawHold", maxsplit=1)[1].split(
        "void MPCFSM::clearAutonomousState", maxsplit=1
    )[0]
    assert "update_hover_pose()" in hold
    assert "trajectory_data.exec_traj = 0" in hold
    assert "trajectory_data.traj_queue.clear()" in hold
    assert "entry_command_active_ = false" in hold
    assert "controller_.setHoverReference(hover_pose_, hover_yaw_)" in hold
    assert "controller_.execMPC" in hold


def test_controller_subscribes_to_search_state_and_yaw():
    assert 'subscribe<std_msgs::String>("landing_search_state"' in NODE
    assert 'subscribe<quadrotor_msgs::PositionCommand>("landing_search_yaw"' in NODE

    root = ET.parse(LAUNCH).getroot()
    args = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    assert args["landing_search_state_topic"] == "/landing_diff_search_manager/state"
    assert args["landing_search_yaw_topic"] == "/UAV0/landing_diff/yaw"

    node = root.find("node[@name='mpc_controller_node']")
    remaps = {item.attrib["from"]: item.attrib["to"] for item in node.findall("remap")}
    assert remaps["~landing_search_state"] == "$(arg landing_search_state_topic)"
    assert remaps["~landing_search_yaw"] == "$(arg landing_search_yaw_topic)"


def test_yaw_command_timeout_never_resumes_horizontal_trajectory():
    hold = FSM.split("void MPCFSM::processLandingSearchYawHold", maxsplit=1)[1].split(
        "void MPCFSM::clearAutonomousState", maxsplit=1
    )[0]
    assert "yaw_fresh" in hold
    assert "保持最后航向且不恢复水平轨迹" in hold
    assert hold.index("if (yaw_fresh)") < hold.index(
        "controller_.setHoverReference(hover_pose_, hover_yaw_)"
    )


def test_downward_search_trajectory_keeps_ch9_locked_yaw():
    callback = FSM.split(
        "void MPCFSM::landingSearchStateCallback", maxsplit=1
    )[1].split("void MPCFSM::landingSearchYawCallback", maxsplit=1)[0]
    assert "isLandingSearchMissionState(state)" in callback

    command = FSM.split("void MPCFSM::CMD_CTRL_process", maxsplit=1)[1].split(
        "void MPCFSM::publish_trigger", maxsplit=1
    )[0]
    assert command.count("landingSearchYawReference(hover_yaw_)") >= 5
    assert "entry_command_yaw_ = landingSearchYawReference(" in command
    assert "traj_info->traj, traj_time, reference_yaw" in command
    assert command.count("landing_search_yaw_override_active_);") == 2


def test_fixed_yaw_override_is_local_to_landing_search():
    assert "bool force_fixed_yaw = false" in CONTROLLER_HEADER
    fixed_branch = CONTROLLER.index("if (force_fixed_yaw)")
    planned_branch = CONTROLLER.index("else if (has_planned_yaw)")
    global_branch = CONTROLLER.index("else if (params_.use_fix_yaw_)")
    assert fixed_branch < planned_branch < global_branch

    mission_states = FSM.split(
        "bool isLandingSearchMissionState", maxsplit=1
    )[1].split("double wrapYaw", maxsplit=1)[0]
    assert 'state == "WAIT_EXIT_SWITCH"' not in mission_states
    assert 'state == "LANDING_HANDOFF"' not in mission_states
