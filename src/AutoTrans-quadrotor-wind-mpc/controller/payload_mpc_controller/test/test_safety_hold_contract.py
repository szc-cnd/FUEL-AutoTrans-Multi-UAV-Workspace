#!/usr/bin/env python3
"""Static integration contracts for the UAV1 planning safety hold."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
FSM_HEADER = (PACKAGE / "include" / "payload_mpc_controller" / "mpc_fsm.h").read_text(
    encoding="utf-8"
)
FSM_SOURCE = (PACKAGE / "src" / "mpc_fsm.cpp").read_text(encoding="utf-8")
NODE_SOURCE = (PACKAGE / "src" / "mpc_controller_node.cpp").read_text(encoding="utf-8")
CONTROLLER_LAUNCH = (PACKAGE / "launch" / "quad_wind_mpc_controller.launch").read_text(
    encoding="utf-8"
)
BRIDGE_LAUNCH = (
    PACKAGE.parents[2] / "autotrans_reference_bridge" / "launch" / "uav1_diff_autotrans.launch"
).read_text(encoding="utf-8")


def test_controller_subscribes_to_remappable_safety_hold():
    assert "safetyHoldCallback" in FSM_HEADER
    assert 'nh.subscribe<std_msgs::Bool>("safety_hold"' in NODE_SOURCE
    assert '<arg name="safety_hold_topic" default="/planning/safety_hold"/>' in CONTROLLER_LAUNCH
    assert '<remap from="~safety_hold" to="$(arg safety_hold_topic)"/>' in CONTROLLER_LAUNCH
    assert '<arg name="safety_hold_topic" default="/UAV1/planning/safety_hold"/>' in BRIDGE_LAUNCH


def test_hold_discards_old_trajectory_but_buffers_fresh_plan():
    callback = FSM_SOURCE.split("void MPCFSM::safetyHoldCallback", 1)[1].split(
        "Finite State Machine", 1
    )[0]
    assert "trajectory_data.allowTrajectoryAcceptanceAfter(now);" in callback
    assert "exec_traj_state_ = HOVER;" in callback
    assert "safety_hold_active_ = false;" in callback


def test_cmd_control_never_executes_trajectory_while_hold_is_active():
    process = FSM_SOURCE.split("void MPCFSM::CMD_CTRL_process", 1)[1].split(
        "void MPCFSM::printandresetRMSE", 1
    )[0]
    hold_guard = process.split("if (safety_hold_active_)", 1)[1].split(
        "switch (exec_traj_state_)", 1
    )[0]
    assert "controller_.setHoverReference" in hold_guard
    assert "exec_traj_state_ = HOVER;" in hold_guard
    assert "return;" in hold_guard
