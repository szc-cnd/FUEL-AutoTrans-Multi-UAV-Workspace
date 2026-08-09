#!/usr/bin/env python3
"""AutoTrans 输入链路的静态回归检查。

PositionCommand 仍由 logger 记录，但不能再进入 AutoTrans 控制器。
"""

import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
NODE_SOURCE = PACKAGE / "src" / "mpc_controller_node.cpp"
FSM_SOURCE = PACKAGE / "src" / "mpc_fsm.cpp"
INPUT_SOURCE = PACKAGE / "src" / "mpc_input.cpp"
LAUNCH = PACKAGE / "launch" / "quad_wind_mpc_controller.launch"


class AutoTransTrajectoryInputStaticTest(unittest.TestCase):
    def test_controller_subscribes_only_to_polynomial_trajectory(self):
        source = NODE_SOURCE.read_text(encoding="utf-8")
        self.assertIn('subscribe<quadrotor_msgs::PolynomialTraj>("traj"', source)
        self.assertNotIn('subscribe<quadrotor_msgs::PositionCommand>', source)
        self.assertNotIn('subscribe<geometry_msgs::PoseStamped>("cmd_trigger"', source)

    def test_cmd_ctrl_does_not_fallback_to_position_command(self):
        source = FSM_SOURCE.read_text(encoding="utf-8")
        start = source.index("void MPCFSM::CMD_CTRL_process()")
        end = source.index("void MPCFSM::setEstimateState", start)
        cmd_ctrl = source[start:end]
        self.assertNotIn("cmd_data", cmd_ctrl)
        self.assertIn("等待 PolynomialTraj", cmd_ctrl)

    def test_trajectory_queue_is_replaced_atomically_and_guarded(self):
        input_source = INPUT_SOURCE.read_text(encoding="utf-8")
        fsm_source = FSM_SOURCE.read_text(encoding="utf-8")
        self.assertIn("std::deque<oneTraj_Data_t> replacement", input_source)
        self.assertIn("traj_queue.swap(replacement)", input_source)
        self.assertIn("trajectory_data.traj_queue.empty()", fsm_source)
        self.assertIn("trajectory_data.clear()", fsm_source)

    def test_launch_has_safety_inputs_and_no_controller_position_remap(self):
        launch = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('arg name="safety_hold_topic"', launch)
        self.assertIn('arg name="landing_request_topic"', launch)
        self.assertNotIn('remap from="~cmd"', launch)
        self.assertNotIn('remap from="~cmd_trigger"', launch)


if __name__ == "__main__":
    unittest.main()
