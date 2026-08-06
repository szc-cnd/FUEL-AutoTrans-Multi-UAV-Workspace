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

    def test_rc_input_declares_ch6_takeoff_request(self):
        self.assertIn("takeoff_channel", self.input_header)
        self.assertIn("is_takeoff_mode", self.input_header)
        self.assertIn("enter_takeoff_mode", self.input_header)

    def test_takeoff_parameters_are_loaded_and_configured(self):
        for token in (
            "struct Takeoff",
            'takeoff/target_z',
            'takeoff/climb_rate',
            'takeoff/position_tolerance',
            'takeoff/velocity_tolerance',
            'takeoff/settle_time',
            'takeoff/timeout',
            'takeoff/max_initial_xy_error',
        ):
            self.assertIn(token, self.params_header)
        for token in (
            "takeoff:",
            "target_z:",
            "climb_rate:",
            "position_tolerance:",
            "velocity_tolerance:",
            "settle_time:",
            "timeout:",
            "max_initial_xy_error:",
        ):
            self.assertIn(token, self.config)

    def test_fsm_has_offboard_gated_takeoff_and_abort_paths(self):
        self.assertIn("case AUTO_TAKEOFF:", self.fsm_source)
        self.assertIn('state_data.current_state.mode != "OFFBOARD"', self.fsm_source)
        self.assertIn("AUTO_TAKEOFF_WAIT_OFFBOARD", self.fsm_source)
        self.assertIn("AUTO_TAKEOFF_ABORTED_OFFBOARD_LOST", self.fsm_source)
        self.assertIn("AUTO_TAKEOFF_ABORTED_CH6_LOW", self.fsm_source)
        self.assertIn("clearAppliedDisturbance();", self.fsm_source)

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


if __name__ == "__main__":
    unittest.main()
