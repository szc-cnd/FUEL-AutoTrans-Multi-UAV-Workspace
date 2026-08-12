#!/usr/bin/env python3
import pathlib
import unittest


REPO = pathlib.Path(__file__).resolve().parents[4]
PACKAGE = REPO / "controller" / "payload_mpc_controller"


class AutoTakeoffContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fsm_source = (PACKAGE / "src/mpc_fsm.cpp").read_text(encoding="utf-8")
        cls.input_header = (
            PACKAGE / "include/payload_mpc_controller/mpc_input.h"
        ).read_text(encoding="utf-8")
        cls.params_header = (
            PACKAGE / "include/payload_mpc_controller/mpc_params.h"
        ).read_text(encoding="utf-8")
        cls.config = (PACKAGE / "config/mpc.yaml").read_text(encoding="utf-8")

    def test_ch8_low_is_the_only_initial_automatic_entry(self):
        manual = self.fsm_source.split("case MANUAL_CTRL:", 1)[1]
        manual = manual.split("case AUTO_HOVER:", 1)[0]
        self.assertIn("rc_data.is_takeoff_mode", manual)
        self.assertIn("publishTakeoffPrestream", manual)
        self.assertNotIn("rc_data.is_hover_mode", manual)
        self.assertNotIn("rc_data.is_command_mode", manual)

    def test_takeoff_parameters_keep_target_and_climb_rate(self):
        for token in ("struct Takeoff", 'takeoff/target_z', 'takeoff/climb_rate'):
            self.assertIn(token, self.params_header)
        self.assertIn("target_z: 0.5", self.config)
        self.assertIn("climb_rate: 0.15", self.config)

    def test_takeoff_reference_generation_is_unchanged(self):
        takeoff = self.fsm_source.split("case AUTO_TAKEOFF:", 1)[1]
        takeoff = takeoff.split("case AUTO_LAND:", 1)[0]
        self.assertIn("hover_pose_ = takeoff_start_pose_;", takeoff)
        self.assertIn("params_.takeoff_.climb_rate * elapsed", takeoff)
        self.assertIn("hover_yaw_ = takeoff_start_yaw_;", takeoff)

    def test_takeoff_does_not_request_offboard_or_arm(self):
        self.assertNotIn("toggle_offboard_mode", self.fsm_source)
        self.assertNotIn("toggle_arm_disarm", self.fsm_source)

    def test_battery_does_not_gate_takeoff(self):
        preconditions = self.fsm_source.split("bool MPCFSM::takeoffPreconditions", 1)[1]
        preconditions = preconditions.split("void MPCFSM::startAutoTakeoff", 1)[0]
        self.assertNotIn("bat_is_received", preconditions)
        self.assertNotIn("bat_data", preconditions)


if __name__ == "__main__":
    unittest.main()
