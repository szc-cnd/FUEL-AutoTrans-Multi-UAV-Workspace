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
    assert "max_velocity_xy: 0.3" in yaml
    assert "max_velocity_z: 0.3" in yaml


def test_generated_solver_exposes_three_affine_velocity_constraints():
    header = (MODEL / "quadrotor_payload_mpc" / "acado_common.h").read_text(encoding="utf-8")
    assert "lbAValues" in header
    assert "ubAValues" in header
    solver = (MODEL / "quadrotor_payload_mpc" / "acado_solver.c").read_text(encoding="utf-8")
    assert "acadoWorkspace.lbA[59]" in solver
    assert "acadoWorkspace.ubA[59]" in solver
    assert "acadoVariables.lbAValues[59] = -2.9999999999999999e-01" in solver
    assert "acadoVariables.ubAValues[59] = 2.9999999999999999e-01" in solver


def test_controller_has_safe_output_and_solver_failure_path():
    fsm_h = (INCLUDE / "mpc_fsm.h").read_text(encoding="utf-8")
    fsm = (SRC / "mpc_fsm.cpp").read_text(encoding="utf-8")
    controller = (SRC / "mpc_controller.cpp").read_text(encoding="utf-8")
    assert "publish_failsafe_hold" in fsm_h
    assert "odom_failsafe_active_" in fsm_h
    assert "trajectory_data.traj_queue.empty()" in fsm
    assert "last_mpc_solve_success_" in controller


def test_ros_locale_is_initialized_before_ros_init():
    node = (SRC / "mpc_controller_node.cpp").read_text(encoding="utf-8")
    assert "#include <clocale>" in node
    assert 'std::setlocale(LC_ALL, "");' in node
