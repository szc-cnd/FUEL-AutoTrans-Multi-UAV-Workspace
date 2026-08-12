#!/usr/bin/env python3
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class LogRatePolicyStaticTest(unittest.TestCase):
    def test_state_messages_are_not_periodically_repeated(self):
        fsm = (ROOT / "src/mpc_fsm.cpp").read_text(encoding="utf-8")
        inputs = (ROOT / "src/mpc_input.cpp").read_text(encoding="utf-8")

        self.assertNotIn(
            'ROS_INFO_THROTTLE(1.0, "[推力映射] 门控状态：%s。", name);', fsm
        )
        self.assertNotIn(
            'ROS_INFO_THROTTLE(1.0, "[RC] CH8=%.0f us，当前请求模式：%s。"', inputs
        )
        self.assertNotIn(
            'ROS_INFO_THROTTLE(1.0,\n\t\t\t\t\t\t\t"[CMD] 正在跟踪入口 PositionCommand 目标。")',
            fsm,
        )
        self.assertNotIn(
            'ROS_WARN_THROTTLE(1.0, "[AUTO_LAND] 等待 CH8 回到低位以解除 PX4 AUTO.LAND 锁定。")',
            fsm,
        )
        self.assertNotIn(
            'ROS_INFO_THROTTLE(1.0, "[AUTO_HOVER] 等待通过 QGC/CH6 选择 PX4 OFFBOARD。")',
            fsm,
        )
        self.assertNotIn(
            'ROS_INFO_THROTTLE(1.0, "[CMD_CTRL] 等待通过 QGC/CH6 选择 PX4 OFFBOARD。")',
            fsm,
        )
        self.assertIn("hover_offboard_wait_reported_", header := (
            ROOT / "include/payload_mpc_controller/mpc_fsm.h"
        ).read_text(encoding="utf-8"))
        self.assertIn("cmd_offboard_wait_reported_", header)

    def test_takeoff_reason_changes_are_reported_once(self):
        header = (ROOT / "include/payload_mpc_controller/mpc_fsm.h").read_text(
            encoding="utf-8"
        )
        fsm = (ROOT / "src/mpc_fsm.cpp").read_text(encoding="utf-8")

        self.assertIn("last_takeoff_precondition_reason_", header)
        self.assertIn("reason != last_takeoff_precondition_reason_", fsm)
        self.assertNotIn(
            'ROS_INFO_THROTTLE(1.0, "[AUTO_TAKEOFF] 条件未满足：%s。"', fsm
        )

    def test_diagnostics_and_faults_use_five_second_period(self):
        controller = (ROOT / "src/mpc_controller.cpp").read_text(encoding="utf-8")

        self.assertIn(
            'ROS_ERROR_THROTTLE(5.0, "[OUTPUT] NMPC 求解失败，保持上一安全控制量。")',
            controller,
        )
        self.assertIn("ROS_INFO_THROTTLE(5.0, \"[NMPC] 计算耗时", controller)


if __name__ == "__main__":
    unittest.main()
