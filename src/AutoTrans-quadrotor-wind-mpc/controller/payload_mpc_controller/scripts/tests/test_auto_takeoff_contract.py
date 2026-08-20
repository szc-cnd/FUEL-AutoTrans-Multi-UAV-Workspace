#!/usr/bin/env python3

import pathlib
import unittest


REPO = pathlib.Path(__file__).resolve().parents[4]
PACKAGE = REPO / "controller" / "payload_mpc_controller"
FSM_HEADER = PACKAGE / "include" / "payload_mpc_controller" / "mpc_fsm.h"
FSM_SOURCE = PACKAGE / "src" / "mpc_fsm.cpp"
INPUT_HEADER = PACKAGE / "include" / "payload_mpc_controller" / "mpc_input.h"
PARAMS_HEADER = PACKAGE / "include" / "payload_mpc_controller" / "mpc_params.h"
CONFIG = PACKAGE / "config" / "mpc.yaml"


class AutoTakeoffContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fsm_header = FSM_HEADER.read_text(encoding="utf-8")
        cls.fsm_source = FSM_SOURCE.read_text(encoding="utf-8")
        cls.input_header = INPUT_HEADER.read_text(encoding="utf-8")
        cls.params_header = PARAMS_HEADER.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")

    def test_rc_input_declares_ch8_takeoff_request(self):
        self.assertIn("mode_channel", self.input_header)
        self.assertIn("is_takeoff_mode", self.input_header)
        self.assertIn("mode_valid", self.input_header)

    def test_takeoff_parameters_only_keep_target_and_climb_rate(self):
        for token in (
            "struct Takeoff",
            'takeoff/target_z',
            'takeoff/climb_rate',
            'takeoff/max_initial_xy_error',
        ):
            self.assertIn(token, self.params_header)
        for token in (
            "takeoff:",
            "target_z: 0.6",
            "climb_rate: 0.15",
            "max_initial_xy_error:",
        ):
            self.assertIn(token, self.config)
        for token in (
            'takeoff/position_tolerance',
            'takeoff/velocity_tolerance',
            'takeoff/settle_time',
            'takeoff/timeout',
            "position_tolerance:",
            "velocity_tolerance:",
            "settle_time:",
        ):
            self.assertNotIn(token, self.params_header + self.config)

    def test_takeoff_has_no_timeout_or_automatic_completion(self):
        takeoff = self.fsm_source.split("case AUTO_TAKEOFF:", 1)[1]
        takeoff = takeoff.split("case AUTO_LAND:", 1)[0]
        self.assertNotIn("params_.takeoff_.timeout", takeoff)
        self.assertNotIn("position_tolerance", takeoff)
        self.assertNotIn("velocity_tolerance", takeoff)
        self.assertNotIn("settle_time", takeoff)
        self.assertNotIn("completeAutoTakeoff", takeoff)

    def test_ch8_selects_takeoff_exit_mode_directly(self):
        takeoff = self.fsm_source.split("case AUTO_TAKEOFF:", 1)[1]
        takeoff = takeoff.split("case AUTO_LAND:", 1)[0]
        self.assertIn("rc_data.is_hover_mode", takeoff)
        self.assertIn("rc_data.is_command_mode", takeoff)
        self.assertIn("fsm_state = AUTO_HOVER", takeoff)
        self.assertIn("fsm_state = CMD_CTRL", takeoff)

    def test_fsm_has_offboard_gated_takeoff(self):
        self.assertIn("case AUTO_TAKEOFF:", self.fsm_source)
        self.assertIn('state_data.current_state.mode != "OFFBOARD"', self.fsm_source)
        self.assertIn("PX4 尚未进入 OFFBOARD", self.fsm_source)
        self.assertIn("clearAppliedDisturbance();", self.fsm_source)

    def test_battery_does_not_gate_takeoff(self):
        preconditions = self.fsm_source.split("bool MPCFSM::takeoffPreconditions", 1)[1]
        preconditions = preconditions.split("void MPCFSM::startAutoTakeoff", 1)[0]
        self.assertNotIn("bat_is_received", preconditions)
        self.assertNotIn("bat_data", preconditions)

    def test_takeoff_does_not_request_offboard_or_arm(self):
        start = self.fsm_source.find("case AUTO_TAKEOFF:")
        self.assertNotEqual(start, -1)
        end = self.fsm_source.index("default:", start)
        takeoff_block = self.fsm_source[start:end]
        self.assertNotIn("toggle_offboard_mode(true)", takeoff_block)
        self.assertNotIn("toggle_arm_disarm(true)", takeoff_block)

    def test_takeoff_blocks_disturbance_and_thrust_learning(self):
        self.assertIn("AUTO_TAKEOFF", self.fsm_source)
        self.assertIn("AUTO_TAKEOFF", self.fsm_header)
        self.assertIn("MODE_BLOCKED", self.fsm_source)
        self.assertIn("thrustModelGate", self.fsm_source)

    def test_prestream_is_low_thrust_and_stops_in_offboard(self):
        manual = self.fsm_source.split("case MANUAL_CTRL:", 1)[1]
        manual = manual.split("case AUTO_HOVER:", 1)[0]
        publish_manual = self.fsm_source.split("void MPCFSM::publish_manual_ctrl", 1)[1]
        publish_manual = publish_manual.split("void MPCFSM::publish_trigger", 1)[0]

        self.assertIn('state_data.current_state.mode != "OFFBOARD"', manual)
        self.assertIn("suppress_manual_setpoint_ = !prestream_allowed", manual)
        self.assertIn("msg.thrust = 0.01", publish_manual)
        self.assertNotIn("params_.thr_map_.hover_percentage", publish_manual)


if __name__ == "__main__":
    unittest.main()
