#!/usr/bin/env python3
"""Static integration contracts for the constrained swing-obstacle guard."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
FSM_HEADER = PACKAGE / "include" / "plan_manage" / "diff_replan_fsm.h"
FSM_SOURCE = PACKAGE / "src" / "diff_replan_fsm.cpp"
ADVANCED_LAUNCH = PACKAGE / "launch" / "include" / "advanced_param_exp.xml"
RUN_SWARM = PACKAGE / "launch" / "exp" / "run_swarm.launch"


def test_fsm_consumes_ldop_current_state_instead_of_generic_predictions():
    header = FSM_HEADER.read_text(encoding="utf-8")
    source = FSM_SOURCE.read_text(encoding="utf-8")
    assert "ldop/DynamicObjectArray.h" in header
    assert "dynamicObjectsCallback" in header
    assert "MOTION_MODEL_CV3D" in source
    assert "dynamic_object_predictions" not in source


def test_collision_stops_then_waits_for_a_stable_release_window():
    source = FSM_SOURCE.read_text(encoding="utf-8")
    assert 'publishPlanningStatus("SWING_OBSTACLE_WAIT")' in source
    assert 'changeFSMExecState(EMERGENCY_STOP, "SWING_GUARD")' in source
    assert "now_sec - swing_clear_since_ < swing_release_clear_time_" in source
    assert 'changeFSMExecState(GEN_NEW_TRAJ, "SWING_RELEASE")' in source


def test_run_swarm_enables_bounded_corridor_model_and_map_fading():
    advanced = ADVANCED_LAUNCH.read_text(encoding="utf-8")
    run_swarm = RUN_SWARM.read_text(encoding="utf-8")
    assert 'name="enable_swing_obstacle_guard" default="false"' in advanced
    assert 'name="enable_swing_obstacle_guard" default="true"' in run_swarm
    assert 'name="swing_obstacle_topic" value="/UAV1/ldop/dynamic_objects"' in run_swarm
    assert 'name="swing_underpass_learning_time" value="3.0"' in run_swarm
    assert 'name="swing_corridor_width" value="1.50"' in run_swarm
    assert 'name="fading_time" value="0.8"' in run_swarm


def test_run_swarm_maps_only_ldop_static_cloud():
    run_swarm = RUN_SWARM.read_text(encoding="utf-8")

    assert 'name="enable_ldop" default="true"' in run_swarm
    assert 'name="cloud_topic" default="/UAV1/ldop/static_cloud"' in run_swarm
