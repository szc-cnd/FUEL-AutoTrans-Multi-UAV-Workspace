#!/usr/bin/env python3
"""静态回归检查：正常速度上限采用有界且高代价的 ACADO 软约束。"""

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class VelocitySoftConstraintStaticTest(unittest.TestCase):
    def test_config_and_runtime_parameter_are_present(self):
        yaml = (ROOT / "config" / "mpc.yaml").read_text(encoding="utf-8")
        model_yaml = (ROOT / "config" / "model.yaml").read_text(encoding="utf-8")
        params = (ROOT / "include" / "payload_mpc_controller" / "mpc_params.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("max_velocity_xy:", yaml)
        self.assertIn("max_velocity_z:", yaml)
        self.assertIn("max_velocity_slack_xy: 1.5", yaml)
        self.assertIn("max_velocity_slack_z: 1.0", yaml)
        self.assertIn("R_velocity_slack_xy: 5000.0", yaml)
        self.assertIn("R_velocity_slack_z: 5000.0", yaml)
        self.assertIn("max_velocity_xy_", params)
        self.assertIn("max_velocity_z_", params)
        self.assertIn("max_velocity_slack_xy_", params)
        self.assertIn("max_velocity_slack_z_", params)
        self.assertIn('"max_velocity_xy"', params)
        self.assertIn('"max_velocity_z"', params)
        # 与 CAV1 一致：运行时速度上限只由 mpc.yaml 定义，避免后加载的 model.yaml 覆盖。
        self.assertNotIn("max_velocity_xy:", model_yaml)
        self.assertNotIn("max_velocity_z:", model_yaml)
        self.assertNotIn("max_velocity_slack_xy:", model_yaml)
        self.assertNotIn("max_velocity_slack_z:", model_yaml)

    def test_acado_model_softens_all_world_velocity_states_with_bounded_slack(self):
        model = (ROOT / "model" / "quadrotor_payload_mpc_with_ext_force.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("max_velocity_xy", model)
        self.assertIn("max_velocity_z", model)
        self.assertIn("Control T, w_x, w_y, w_z, s_vx, s_vy, s_vz;", model)
        self.assertIn("0.0 <= s_vx <= max_velocity_slack_xy", model)
        self.assertIn("v_x - s_vx <= max_velocity_xy", model)
        self.assertIn("-v_x - s_vx <= max_velocity_xy", model)
        self.assertIn("v_y - s_vy <= max_velocity_xy", model)
        self.assertIn("v_z - s_vz <= max_velocity_z", model)

    def test_wrapper_passes_velocity_bounds_to_generated_solver(self):
        header = (ROOT / "include" / "payload_mpc_controller" / "mpc_wrapper.h").read_text(
            encoding="utf-8"
        )
        wrapper = (ROOT / "src" / "mpc_wrapper.cpp").read_text(encoding="utf-8")
        controller = (ROOT / "src" / "mpc_controller.cpp").read_text(encoding="utf-8")
        self.assertIn("max_velocity_xy", header)
        self.assertIn("max_velocity_z", header)
        self.assertIn("kVelocitySoftConstraintSize = 6", header)
        self.assertIn("acado_lower_affine_bounds_", header)
        self.assertIn("acado_upper_affine_bounds_", header)
        self.assertIn("max_velocity_xy", wrapper)
        self.assertIn("max_velocity_z", wrapper)
        self.assertIn("params_.max_velocity_xy_", controller)
        self.assertIn("params_.max_velocity_z_", controller)
        self.assertIn("params_.max_velocity_slack_xy_", controller)
        self.assertIn("params_.max_velocity_slack_z_", controller)

    def test_generated_solver_dimensions_match_soft_constraint_model(self):
        generated_header = (
            ROOT / "model" / "quadrotor_payload_mpc" / "acado_common.h"
        ).read_text(encoding="utf-8")
        solver = (
            ROOT / "model" / "quadrotor_payload_mpc" / "acado_solver.c"
        ).read_text(encoding="utf-8")
        self.assertIn("#define ACADO_NU 7", generated_header)
        self.assertIn("real_t lbValues[ 140 ];", generated_header)
        self.assertIn("real_t lbAValues[ 120 ];", generated_header)
        self.assertIn("acadoWorkspace.lbA[119]", solver)
        self.assertIn("acadoWorkspace.ubA[119]", solver)


if __name__ == "__main__":
    unittest.main()
