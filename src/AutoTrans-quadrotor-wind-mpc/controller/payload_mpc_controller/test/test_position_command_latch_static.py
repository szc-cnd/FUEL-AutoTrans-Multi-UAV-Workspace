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
PARAMS_HEADER = PACKAGE / "include" / "payload_mpc_controller" / "mpc_params.h"
PARAMS_YAML = PACKAGE / "config" / "mpc.yaml"
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

    def test_old_trajectory_messages_are_rejected_without_interrupting_valid_trajectory(self):
        fsm_source = FSM_SOURCE.read_text(encoding="utf-8")
        params_source = PARAMS_HEADER.read_text(encoding="utf-8")
        yaml_source = PARAMS_YAML.read_text(encoding="utf-8")

        self.assertIn("trajectoryTimestampAcceptable", fsm_source)
        self.assertIn("params_.msg_timeout_.trajectory", fsm_source)
        self.assertIn("trajectory_id <= last_accepted_trajectory_id_", fsm_source)
        self.assertIn("last_accepted_trajectory_id_ = trajectory_id", fsm_source)
        self.assertIn("拒绝旧轨迹", fsm_source)
        self.assertIn('read_essential_param(nh, "msg_timeout/trajectory", msg_timeout_.trajectory)',
                      params_source)
        self.assertIn('read_essential_param(nh, "msg_timeout/trajectory_future", msg_timeout_.trajectory_future)',
                      params_source)
        self.assertIn("trajectory: 0.8", yaml_source)
        self.assertIn("trajectory_future: 0.1", yaml_source)

        callback_start = fsm_source.index("void MPCFSM::trajectoryCallback")
        callback_end = fsm_source.index("void MPCFSM::safetyHoldCallback", callback_start)
        callback = fsm_source[callback_start:callback_end]
        guard_end = callback.index("trajectory_data.feed(msg);")
        self.assertIn("trajectoryTimestampAcceptable", callback[:guard_end])
        self.assertIn("return;", callback[:guard_end])

    def test_heartbeat_loss_resets_trajectory_sequence_history(self):
        source = FSM_SOURCE.read_text(encoding="utf-8")
        loss_start = source.index("void MPCFSM::handlePlannerHeartbeatLoss")
        loss_end = source.index("void MPCFSM::plannerHeartbeatCallback", loss_start)
        heartbeat_loss = source[loss_start:loss_end]
        self.assertIn("last_accepted_trajectory_id_ = 0", heartbeat_loss)
        self.assertIn("trajectory_sequence_initialized_ = false", heartbeat_loss)

    def test_launch_has_safety_inputs_and_no_controller_position_remap(self):
        launch = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('arg name="safety_hold_topic"', launch)
        self.assertIn('arg name="landing_request_topic"', launch)
        self.assertNotIn('remap from="~cmd"', launch)
        self.assertNotIn('remap from="~cmd_trigger"', launch)

    def test_manual_output_does_not_use_fixed_low_thrust_in_air(self):
        source = FSM_SOURCE.read_text(encoding="utf-8")
        start = source.index("void MPCFSM::publish_manual_ctrl")
        manual_ctrl = source[start:source.index("void MPCFSM::publish_trigger", start)]

        self.assertIn("LANDED_STATE_IN_AIR", manual_ctrl)
        self.assertIn("has_last_safe_normalized_thrust_", manual_ctrl)
        self.assertIn("ground_placeholder_thrust", manual_ctrl)
        self.assertNotIn("msg.thrust = 0.05", manual_ctrl)
        self.assertIn("last_safe_normalized_thrust_", source)

    def test_solver_failure_publishes_immediate_safe_hover(self):
        fsm_source = FSM_SOURCE.read_text(encoding="utf-8")
        controller_source = (
            (PACKAGE / "src" / "mpc_controller.cpp").read_text(encoding="utf-8")
        )
        controller_header = (
            (PACKAGE / "include" / "payload_mpc_controller" / "mpc_controller.h")
            .read_text(encoding="utf-8")
        )

        self.assertIn("publish_solver_failure_safe_ctrl", fsm_source)
        self.assertIn("!controller_.lastMpcSolveSucceeded()", fsm_source)
        self.assertNotIn("solverFailurePersistent(0.2)", fsm_source)
        self.assertNotIn("last_valid_control_inputs_", controller_source)
        self.assertIn("getHoverNormalizedThrust", controller_header)
        self.assertIn("getHoverNormalizedThrust", controller_source)

        start = fsm_source.index("void MPCFSM::publish_solver_failure_safe_ctrl")
        end = fsm_source.index("void MPCFSM::publish_manual_ctrl", start)
        safe_ctrl = fsm_source[start:end]
        self.assertIn("msg.body_rate.x = 0.0", safe_ctrl)
        self.assertIn("msg.body_rate.y = 0.0", safe_ctrl)
        self.assertIn("msg.body_rate.z = 0.0", safe_ctrl)
        self.assertIn("controller_.getHoverNormalizedThrust()", safe_ctrl)
        self.assertIn("explicitly_landed", safe_ctrl)
        self.assertIn("explicitly_disarmed", safe_ctrl)
        self.assertIn("explicitly_not_offboard", safe_ctrl)
        self.assertNotIn("px4_in_air && px4_offboard_armed", safe_ctrl)


if __name__ == "__main__":
    unittest.main()
