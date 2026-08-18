#!/usr/bin/env python3
"""Static checks for CAV1 controller safety and hard velocity limits."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[2]
INCLUDE = PACKAGE / "include" / "payload_mpc_controller"
SRC = PACKAGE / "src"
MODEL = PACKAGE / "model"
CONFIG = PACKAGE / "config"


def test_velocity_limits_are_configured_and_forwarded():
    params = (INCLUDE / "mpc_params.h").read_text(encoding="utf-8")
    wrapper_h = (INCLUDE / "mpc_wrapper.h").read_text(encoding="utf-8")
    controller = (SRC / "mpc_controller.cpp").read_text(encoding="utf-8")
    yaml = (CONFIG / "mpc.yaml").read_text(encoding="utf-8")
    assert "max_velocity_xy_" in params
    assert "max_velocity_z_" in params
    assert "max_velocity_xy" in wrapper_h
    assert "max_velocity_z" in wrapper_h
    assert "params_.max_velocity_xy_" in controller
    assert "params_.max_velocity_z_" in controller
    assert "max_velocity_xy: 0.5" in yaml
    assert "max_velocity_z: 0.5" in yaml


def test_solver_exposes_three_affine_velocity_constraints_and_runtime_overrides_them():
    header = (MODEL / "quadrotor_payload_mpc" / "acado_common.h").read_text(encoding="utf-8")
    assert "lbAValues" in header
    assert "ubAValues" in header
    solver = (MODEL / "quadrotor_payload_mpc" / "acado_solver.c").read_text(encoding="utf-8")
    assert "acadoWorkspace.lbA[59]" in solver
    assert "acadoWorkspace.ubA[59]" in solver
    wrapper = (SRC / "mpc_wrapper.cpp").read_text(encoding="utf-8")
    assert "lower_velocity_bounds << -max_velocity_xy, -max_velocity_xy, -max_velocity_z" in wrapper
    assert "upper_velocity_bounds << max_velocity_xy, max_velocity_xy, max_velocity_z" in wrapper
    assert "acado_lower_affine_bounds_ = lower_velocity_bounds.replicate" in wrapper
    assert "acado_upper_affine_bounds_ = upper_velocity_bounds.replicate" in wrapper


def test_controller_has_safe_output_and_solver_failure_path():
    fsm_h = (INCLUDE / "mpc_fsm.h").read_text(encoding="utf-8")
    fsm = (SRC / "mpc_fsm.cpp").read_text(encoding="utf-8")
    controller = (SRC / "mpc_controller.cpp").read_text(encoding="utf-8")
    assert "publish_failsafe_hold" in fsm_h
    assert "odom_failsafe_active_" in fsm_h
    assert "trajectory_data.traj_queue.empty()" in fsm
    assert "last_mpc_solve_success_" in controller


def test_nmpc_recovery_message_is_edge_triggered():
    header = (INCLUDE / "mpc_controller.h").read_text(encoding="utf-8")
    controller = (SRC / "mpc_controller.cpp").read_text(encoding="utf-8")
    assert "mpc_failure_active_" in header
    assert "mpc_failure_active_ = true;" in controller
    assert "if (mpc_failure_active_)" in controller
    assert "mpc_failure_active_ = false;" in controller
    assert "previous_mpc_solve_success" not in controller


def test_nmpc_recovery_latches_hover_and_gates_trajectories():
    fsm_h = (INCLUDE / "mpc_fsm.h").read_text(encoding="utf-8")
    fsm = (SRC / "mpc_fsm.cpp").read_text(encoding="utf-8")
    input_h = (INCLUDE / "mpc_input.h").read_text(encoding="utf-8")
    input_cpp = (SRC / "mpc_input.cpp").read_text(encoding="utf-8")
    params = (INCLUDE / "mpc_params.h").read_text(encoding="utf-8")
    yaml = (CONFIG / "mpc.yaml").read_text(encoding="utf-8")

    assert "MPC_RECOVERY_HOVER" in fsm_h
    assert "hover_pose_ = odom_data.p;" in fsm
    assert "trajectory_data.blockTrajectoryAcceptance();" in fsm
    assert "trajectory_data.allowTrajectoryAcceptanceAfter(now);" in fsm
    assert "resetForHover" in fsm
    assert "mpc_recovery_success_cycles" in params
    assert "mpc_recovery_timeout: 1.0" in yaml
    assert "mpc_recovery_success_cycles: 1" in yaml
    assert "trajectory_acceptance_enabled" in input_h
    assert "pMsg->header.stamp <= accept_trajectory_after" in input_cpp
    assert "planning_restart_pub_.publish(restart_msg);" in fsm


def test_trajectory_order_uses_stamp_and_id_is_diagnostic_only():
    input_cpp = (SRC / "mpc_input.cpp").read_text(encoding="utf-8")
    assert "pMsg->header.stamp <= last_trajectory_stamp" in input_cpp
    assert "traj.trajectory_id <= last_trajectory_id ||" not in input_cpp
    assert "trajectory_id 无效" not in input_cpp
    assert "trajectory_id 重新计数" in input_cpp


def test_recovery_timeout_keeps_retrying_without_px4_land():
    fsm = (SRC / "mpc_fsm.cpp").read_text(encoding="utf-8")
    node = (SRC / "mpc_controller_node.cpp").read_text(encoding="utf-8")
    assert "planning_stop_pub_.publish(stop_msg);" in fsm
    assert "request_px4_auto_land();" in fsm
    assert 'beginDirectAutoLand(now_time, "AUTO_LAND 状态下 NMPC 求解失败")' not in fsm
    assert "beginMpcRecovery(now_time);" in fsm
    assert 'beginDirectAutoLand(now, "NMPC 安全恢复超时")' not in fsm
    assert "不因求解失败自动降落" in fsm
    assert "controller_.lastMpcSolveSuccessful() && (low_enough || timeout)" in fsm
    recovery = fsm.split("void MPCFSM::processMpcRecovery", 1)[1]
    recovery = recovery.split("void MPCFSM::beginDirectAutoLand", 1)[0]
    timeout_handling = recovery.split("const double elapsed", 1)[1]
    assert "beginDirectAutoLand" not in timeout_handling
    assert "elapsed <= params_.safety_.mpc_recovery_timeout" not in recovery
    assert 'advertise<std_msgs::Empty>("/planning_stop_trigger"' in node


def test_short_mpc_hold_uses_only_recent_valid_solver_output():
    header = (INCLUDE / "mpc_controller.h").read_text(encoding="utf-8")
    controller = (SRC / "mpc_controller.cpp").read_text(encoding="utf-8")
    fsm = (SRC / "mpc_fsm.cpp").read_text(encoding="utf-8")
    assert "hasRecentValidControl" in header
    assert "last_valid_control_time_ = ros::Time::now();" in controller
    assert "kLastValidMpcHoldSeconds = 0.2" in fsm
    assert "controller_.lastValidControlInput()" in fsm
    assert "controller_.clearLastValidControl();" in fsm


def test_dynamic_yaw_is_continuous_across_replans_and_advances_once_per_cycle():
    header = (INCLUDE / "mpc_controller.h").read_text(encoding="utf-8")
    controller = (SRC / "mpc_controller.cpp").read_text(encoding="utf-8")
    yaml = (CONFIG / "mpc.yaml").read_text(encoding="utf-8")

    trajectory_reference = controller.split("void MpcController::setTrajectoyReference", 1)[1]
    trajectory_reference = trajectory_reference.split("void MpcController::preparationThread", 1)[0]
    yaw_helper = controller.split("void MpcController::calculate_yaw", 1)[1]
    yaw_helper = yaw_helper.split("void MpcController::computeQuadrotorFlatness", 1)[0]

    assert "yaw_reference_initialized_" in header
    assert "reset last_yaw_ from the first point" not in trajectory_reference
    assert "if (i == 0)" in trajectory_reference
    assert "calculate_yaw(vel_quad, t_step, last_yaw_, last_yaw_dot_)" in trajectory_reference
    assert "calculate_yaw(vel_quad, t_step, predicted_yaw, predicted_yaw_dot)" in trajectory_reference
    assert "constexpr double kYawHoldSpeed = 0.12" in yaw_helper
    assert "vel.head<2>().norm() < kYawHoldSpeed" in yaw_helper
    assert "max_bodyrate_z:     0.5" in yaml


def test_ros_locale_is_initialized_before_ros_init():
    node = (SRC / "mpc_controller_node.cpp").read_text(encoding="utf-8")
    assert "#include <clocale>" in node
    assert 'std::setlocale(LC_ALL, "");' in node
