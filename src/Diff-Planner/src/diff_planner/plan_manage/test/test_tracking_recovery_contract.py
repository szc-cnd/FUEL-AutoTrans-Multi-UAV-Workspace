#!/usr/bin/env python3
"""Static contracts for Diff trajectory sequencing and odometry recovery."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
DIFF_ROOT = PACKAGE.parent
FSM_HEADER = PACKAGE / "include" / "plan_manage" / "diff_replan_fsm.h"
FSM_SOURCE = PACKAGE / "src" / "diff_replan_fsm.cpp"
PLAN_CONTAINER = DIFF_ROOT / "traj_utils" / "include" / "traj_utils" / "plan_container.hpp"
ADVANCED_LAUNCH = PACKAGE / "launch" / "include" / "advanced_param_exp.xml"
RELAY_LAUNCH = PACKAGE / "launch" / "exp" / "run_uav1_relay_diff.launch"
MANAGER_SOURCE = PACKAGE / "src" / "planner_manager.cpp"
OPTIMIZER_HEADER = DIFF_ROOT / "traj_opt" / "include" / "optimizer" / "poly_traj_optimizer.h"
OPTIMIZER_SOURCE = DIFF_ROOT / "traj_opt" / "src" / "poly_traj_optimizer.cpp"
WATCHDOG = PACKAGE / "scripts" / "diff_planner_watchdog.py"


def test_local_trajectory_id_does_not_reset_on_global_waypoint_change():
    source = PLAN_CONTAINER.read_text(encoding="utf-8")
    set_global = source.split("void setGlobalTraj", 1)[1].split("void setLocalTraj", 1)[0]
    set_local = source.split("void setLocalTraj", 1)[1]
    assert "local_traj.traj_id = 0" not in set_global
    assert "local_traj.traj_id++" in set_local


def test_tracking_error_parameter_is_exposed_with_030_default():
    header = FSM_HEADER.read_text(encoding="utf-8")
    source = FSM_SOURCE.read_text(encoding="utf-8")
    launch = ADVANCED_LAUNCH.read_text(encoding="utf-8")
    assert "max_tracking_error_" in header
    assert 'nh.param("fsm/max_tracking_error", max_tracking_error_, 0.30)' in source
    assert 'name="max_tracking_error" default="0.30"' in launch


def test_tracking_deviation_replans_from_measured_odometry():
    source = FSM_SOURCE.read_text(encoding="utf-8")
    local_replan = source.split("bool DiffReplanFSM::planFromLocalTraj", 1)[1]
    local_replan = local_replan.split("bool DiffReplanFSM::planNextWaypoint", 1)[0]
    assert "tracking_error > max_tracking_error_" in local_replan
    assert "start_pt_ = odom_pos_;" in local_replan
    assert "start_vel_ = odom_vel_;" in local_replan
    assert "start_acc_.setZero();" in local_replan
    assert 'publishPlanningStatus("TRACKING_DEVIATION_REPLAN")' in local_replan
    assert "callReboundReplan(replan_from_odom, false)" in local_replan


def test_controller_restart_keeps_target_and_replans_from_odometry():
    header = FSM_HEADER.read_text(encoding="utf-8")
    source = FSM_SOURCE.read_text(encoding="utf-8")
    callback = source.split("void DiffReplanFSM::planningRestartCallback", 1)[1]
    callback = callback.split("void DiffReplanFSM::odometryCallback", 1)[0]
    global_replan = source.split("bool DiffReplanFSM::planFromGlobalTraj", 1)[1]
    global_replan = global_replan.split("bool DiffReplanFSM::planFromLocalTraj", 1)[0]

    assert "planning_restart_sub_" in header
    assert 'nh.subscribe("/planning_restart_trigger"' in source
    assert "have_target_ = false" not in callback
    assert "have_trigger_ = false" not in callback
    assert "changeFSMExecState(GEN_NEW_TRAJ" in callback
    assert "start_pt_ = odom_pos_;" in global_replan
    assert "start_vel_ = odom_vel_;" in global_replan
    assert "start_acc_.setZero();" in global_replan


def test_planning_wall_timeout_covers_initial_check_and_lbfgs():
    manager = MANAGER_SOURCE.read_text(encoding="utf-8")
    header = OPTIMIZER_HEADER.read_text(encoding="utf-8")
    optimizer = OPTIMIZER_SOURCE.read_text(encoding="utf-8")
    launch = ADVANCED_LAUNCH.read_text(encoding="utf-8")

    assert "ros::WallTime planning_deadline_" in header
    assert "beginPlanningCycle()" in manager
    assert 'checkPlanningTimeout("trajectory initialization")' in manager
    assert 'checkPlanningTimeout("LBFGS line search")' in optimizer
    assert 'name="planning_timeout" default="0.8"' in launch


def test_uav1_relay_uses_traj_server_heartbeat_safety_only():
    advanced = ADVANCED_LAUNCH.read_text(encoding="utf-8")
    relay = RELAY_LAUNCH.read_text(encoding="utf-8")
    traj_server = (PACKAGE / "src" / "traj_server.cpp").read_text(encoding="utf-8")

    assert 'respawn="$(arg planner_respawn)"' in advanced
    assert 'planner_respawn' not in relay
    assert 'diff_planner_watchdog.py' not in relay
    assert 'value="/drone_1_planning/heartbeat"' in relay
    assert '<remap from="~heartbeat" to="/drone_1_planning/heartbeat"/>' in relay
    assert '<arg name="planning_timeout" value="2.0"/>' in relay
    assert '<remap from="~safety_hold" to="/UAV1/planning/safety_hold"/>' in relay
    assert '<param name="traj_server/heartbeat_timeout" value="2.0" type="double"/>' in relay
    assert '#include <std_msgs/Bool.h>' in traj_server
    assert 'nh.advertise<std_msgs::Bool>("safety_hold"' in traj_server
    assert 'publishSafetyHold(true)' in traj_server
    assert 'publishSafetyHold(false)' in traj_server


def test_trajectory_sampling_rejects_non_progressing_loops():
    optimizer = OPTIMIZER_SOURCE.read_text(encoding="utf-8")
    assert "MIN_SAMPLE_STEP = 1.0e-4" in optimizer
    assert "MAX_TRAJECTORY_SAMPLES = 1000000U" in optimizer
    assert "while (sample_count++ < max_samples)" in optimizer


def test_planning_status_echoes_external_goal_stamp():
    header = FSM_HEADER.read_text(encoding="utf-8")
    source = FSM_SOURCE.read_text(encoding="utf-8")
    callback = source.split("void DiffReplanFSM::waypointCallback", 1)[1].split(
        "void DiffReplanFSM::publishPlanningStatus", 1
    )[0]
    status = source.split("void DiffReplanFSM::publishPlanningStatus", 1)[1].split(
        "void DiffReplanFSM::readGivenWpsAndPlan", 1
    )[0]

    assert "active_external_goal_stamp_ns_" in header
    assert "active_external_goal_stamp_ns_ = msg->header.stamp.toNSec();" in callback
    assert 'stream << status << " goal_stamp_ns=" << active_external_goal_stamp_ns_' in status
    assert 'stream << " " << final_goal_.x()' in status


def test_occupied_goal_is_replaced_and_new_goal_callback_never_nested_spins():
    header = FSM_HEADER.read_text(encoding="utf-8")
    source = FSM_SOURCE.read_text(encoding="utf-8")
    waypoint = source.split("bool DiffReplanFSM::planNextWaypoint", 1)[1].split(
        "bool DiffReplanFSM::mondifyInCollisionFinalGoal", 1
    )[0]
    replace = source.split("bool DiffReplanFSM::mondifyInCollisionFinalGoal", 1)[1].split(
        "void DiffReplanFSM::waypointCallback", 1
    )[0]
    replan = source.split("case REPLAN_TRAJ:", 1)[1].split("case EXEC_TRAJ:", 1)[0]

    assert "external_goal_modified_" in header
    assert "external_goal_modified_ = true;" in replace
    assert 'publishPlanningStatus("TRAJECTORY_PUBLISHED")' in replan
    assert "while (exec_state_ != EXEC_TRAJ)" not in waypoint
    assert "ros::spinOnce()" not in waypoint
    assert 'changeFSMExecState(GEN_NEW_TRAJ, "NEW_EXTERNAL_GOAL")' in waypoint


def test_uav1_relay_enables_short_history_retreat_recovery():
    launch = RELAY_LAUNCH.read_text(encoding="utf-8")
    assert '<arg name="enable_occupied_recovery" value="true"/>' in launch
    assert '<arg name="escape_max_distance" value="0.45"/>' in launch
    assert '<arg name="escape_history_time" value="3.00"/>' in launch
    assert '<arg name="escape_speed" value="0.10"/>' in launch
    assert '<arg name="history_only_occupied_recovery" value="true"/>' in launch
    assert '<arg name="wait_new_target_after_occupied_recovery" value="true"/>' in launch

    source = FSM_SOURCE.read_text(encoding="utf-8")
    history = source.split("bool DiffReplanFSM::selectHistoryRecoveryTarget", 1)[1]
    history = history.split("bool DiffReplanFSM::selectVerticalRecoveryTarget", 1)[0]
    assert "history_distance += std::hypot" in history
    assert "history_distance - escape_max_distance_" in history
    assert "estimateInflatedClearance(candidate)" not in history
    assert "validateRecoverySegment(odom_pos_, candidate" not in history
    assert "occupied_recovery_attempt_count_ > 0" not in history
    occupied_recovery = source.split("case OCCUPIED_RECOVERY:", 1)[1]
    occupied_recovery = occupied_recovery.split("finishProcess();", 1)[0]
    assert "!occupied_recovery_from_history_" in occupied_recovery
    assert "occupied_recovery_from_history_ ||" in occupied_recovery
    assert 'changeFSMExecState(WAIT_TARGET, "OCCUPIED_RECOVERY_WAIT_NEXT_TARGET")' in source
    preempt = source.split("const bool normal_planning_state", 1)[1].split(
        "static int fsm_num", 1
    )[0]
    assert "getInflateOccupancy(odom_pos_)" in preempt
    assert 'changeFSMExecState(EMERGENCY_STOP, "OCCUPIED_START")' in preempt


def test_depth_timeout_waits_for_map_and_automatically_resumes():
    header = FSM_HEADER.read_text(encoding="utf-8")
    source = FSM_SOURCE.read_text(encoding="utf-8")
    collision = source.split("/* ---------- check lost of depth ---------- */", 1)[1].split(
        "if (enable_swing_obstacle_guard_", 1
    )[0]
    emergency = source.split("case EMERGENCY_STOP:", 1)[1].split(
        "case OCCUPIED_RECOVERY:", 1
    )[0]
    assert "depth_timeout_emergency_" in header
    assert "enable_fail_safe_ = false" not in collision
    assert "depth_timeout_emergency_ = true;" in collision
    assert 'changeFSMExecState(GEN_NEW_TRAJ, "DEPTH_RECOVERED")' in emergency


def test_occupied_recovery_budget_starts_another_automatic_round():
    source = FSM_SOURCE.read_text(encoding="utf-8")
    emergency = source.split("case EMERGENCY_STOP:", 1)[1].split(
        "case OCCUPIED_RECOVERY:", 1
    )[0]
    exhausted = emergency.split(
        "occupied_recovery_attempt_count_ >= escape_max_attempts_", 1
    )[1].split("else if", 1)[0]
    assert "occupied_recovery_attempt_count_ = 0;" in exhausted
    assert "publishPlanningStatus(\"OCCUPIED_RECOVERY_FAILED\")" not in exhausted
