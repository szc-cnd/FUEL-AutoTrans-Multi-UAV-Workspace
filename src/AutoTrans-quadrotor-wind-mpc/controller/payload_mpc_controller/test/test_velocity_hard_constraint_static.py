#!/usr/bin/env python3
"""静态回归检查：速度上限必须进入 ACADO 的 NMPC 硬约束。"""

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class VelocityHardConstraintStaticTest(unittest.TestCase):
    def test_config_and_runtime_parameter_are_present(self):
        yaml = (ROOT / "config" / "mpc.yaml").read_text(encoding="utf-8")
        model_yaml = (ROOT / "config" / "model.yaml").read_text(encoding="utf-8")
        params = (ROOT / "include" / "payload_mpc_controller" / "mpc_params.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("max_velocity_xy:", yaml)
        self.assertIn("max_velocity_z:", yaml)
        self.assertIn("max_velocity_xy_", params)
        self.assertIn("max_velocity_z_", params)
        self.assertIn('"max_velocity_xy"', params)
        self.assertIn('"max_velocity_z"', params)
        # 与 CAV1 一致：运行时速度上限只由 mpc.yaml 定义，避免后加载的 model.yaml 覆盖。
        self.assertNotIn("max_velocity_xy:", model_yaml)
        self.assertNotIn("max_velocity_z:", model_yaml)

    def test_acado_model_constrains_all_world_velocity_states(self):
        model = (ROOT / "model" / "quadrotor_payload_mpc_with_ext_force.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("max_velocity_xy", model)
        self.assertIn("max_velocity_z", model)
        self.assertIn("-max_velocity_xy <= v_x <= max_velocity_xy", model)
        self.assertIn("-max_velocity_xy <= v_y <= max_velocity_xy", model)
        self.assertIn("-max_velocity_z <= v_z <= max_velocity_z", model)

    def test_wrapper_passes_velocity_bounds_to_generated_solver(self):
        header = (ROOT / "include" / "payload_mpc_controller" / "mpc_wrapper.h").read_text(
            encoding="utf-8"
        )
        wrapper = (ROOT / "src" / "mpc_wrapper.cpp").read_text(encoding="utf-8")
        controller = (ROOT / "src" / "mpc_controller.cpp").read_text(encoding="utf-8")
        self.assertIn("max_velocity_xy", header)
        self.assertIn("max_velocity_z", header)
        self.assertIn("kStateConstraintSize = 3", header)
        self.assertIn("acado_lower_affine_bounds_", header)
        self.assertIn("acado_upper_affine_bounds_", header)
        self.assertIn("max_velocity_xy", wrapper)
        self.assertIn("max_velocity_z", wrapper)
        self.assertIn("params_.max_velocity_xy_", controller)
        self.assertIn("params_.max_velocity_z_", controller)


if __name__ == "__main__":
    unittest.main()
