#!/usr/bin/env python3
"""Static contracts for Diff occupied-start local recovery."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
FSM_HEADER = PACKAGE / "include" / "plan_manage" / "diff_replan_fsm.h"
FSM_SOURCE = PACKAGE / "src" / "diff_replan_fsm.cpp"
MANAGER_HEADER = PACKAGE / "include" / "plan_manage" / "planner_manager.h"
MANAGER_SOURCE = PACKAGE / "src" / "planner_manager.cpp"
ADVANCED_LAUNCH = PACKAGE / "launch" / "include" / "advanced_param_exp.xml"
RUN_SWARM = PACKAGE / "launch" / "exp" / "run_swarm.launch"


def test_occupied_recovery_is_isolated_from_normal_planning():
    header = FSM_HEADER.read_text(encoding="utf-8")
    source = FSM_SOURCE.read_text(encoding="utf-8")
    assert "OCCUPIED_RECOVERY" in header
    collision_guard = source.split("void DiffReplanFSM::checkCollisionCallback", 1)[1]
    collision_guard = collision_guard.split("bool DiffReplanFSM::callEmergencyStop", 1)[0]
    assert "exec_state_ == OCCUPIED_RECOVERY" in collision_guard
    assert "changeFSMExecState(GEN_NEW_TRAJ, \"OCCUPIED_RECOVERY_DONE\")" in source


def test_recovery_prefers_history_then_non_retreating_lateral_search():
    source = FSM_SOURCE.read_text(encoding="utf-8")
    selector = source.split("bool DiffReplanFSM::selectOccupiedRecoveryTarget", 1)[1]
    selector = selector.split("bool DiffReplanFSM::callOccupiedRecovery", 1)[0]
    assert selector.index("selectHistoryRecoveryTarget") < selector.index(
        "selectLateralRecoveryTarget"
    )

    lateral = source.split("bool DiffReplanFSM::selectLateralRecoveryTarget", 1)[1]
    lateral = lateral.split("bool DiffReplanFSM::selectOccupiedRecoveryTarget", 1)[0]
    assert "forward_progress < -1.0e-6" in lateral
    assert "10.0 * candidate_clearance" in lateral
    assert "90.0, -90.0" in lateral


def test_recovery_path_only_allows_a_short_initial_occupied_prefix():
    source = FSM_SOURCE.read_text(encoding="utf-8")
    validator = source.split("bool DiffReplanFSM::validateRecoverySegment", 1)[1]
    validator = validator.split("bool DiffReplanFSM::selectHistoryRecoveryTarget", 1)[0]
    assert "!allow_initial_occupied || reached_free_space" in validator
    assert "prefix > escape_max_occupied_prefix_" in validator
    assert "return reached_free_space" in validator


def test_recovery_trajectory_is_slow_and_zero_velocity_at_both_ends():
    manager_header = MANAGER_HEADER.read_text(encoding="utf-8")
    manager_source = MANAGER_SOURCE.read_text(encoding="utf-8")
    assert "OccupiedStartRecovery" in manager_header
    recovery = manager_source.split("bool DiffPlannerManager::OccupiedStartRecovery", 1)[1]
    recovery = recovery.split("bool DiffPlannerManager::checkCollision", 1)[0]
    assert "1.875 * distance / max_speed" in recovery
    assert "head_state << start_pos, zero, zero" in recovery
    assert "tail_state << target_pos, zero, zero" in recovery


def test_run_swarm_enables_recovery_with_conservative_defaults():
    advanced = ADVANCED_LAUNCH.read_text(encoding="utf-8")
    run_swarm = RUN_SWARM.read_text(encoding="utf-8")
    assert 'name="enable_occupied_recovery" default="false"' in advanced
    assert 'name="enable_occupied_recovery" default="true"' in run_swarm
    assert 'name="escape_speed" value="0.10"' in run_swarm
    assert 'name="escape_max_distance" value="0.40"' in run_swarm
    assert 'name="escape_max_occupied_prefix" value="0.20"' in run_swarm
