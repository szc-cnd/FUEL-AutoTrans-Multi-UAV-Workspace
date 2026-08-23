#!/usr/bin/env python3
"""Static contracts for Diff trajectory sequencing and odometry recovery."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
DIFF_ROOT = PACKAGE.parent
FSM_HEADER = PACKAGE / "include" / "plan_manage" / "diff_replan_fsm.h"
FSM_SOURCE = PACKAGE / "src" / "diff_replan_fsm.cpp"
PLAN_CONTAINER = DIFF_ROOT / "traj_utils" / "include" / "traj_utils" / "plan_container.hpp"
ADVANCED_LAUNCH = PACKAGE / "launch" / "include" / "advanced_param_exp.xml"
MANAGER_SOURCE = PACKAGE / "src" / "planner_manager.cpp"
OPTIMIZER_HEADER = DIFF_ROOT / "traj_opt" / "include" / "optimizer" / "poly_traj_optimizer.h"
OPTIMIZER_SOURCE = DIFF_ROOT / "traj_opt" / "src" / "poly_traj_optimizer.cpp"


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


def test_trajectory_sampling_rejects_non_progressing_loops():
    optimizer = OPTIMIZER_SOURCE.read_text(encoding="utf-8")
    assert "MIN_SAMPLE_STEP = 1.0e-4" in optimizer
    assert "MAX_TRAJECTORY_SAMPLES = 1000000U" in optimizer
    assert "while (sample_count++ < max_samples)" in optimizer
