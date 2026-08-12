#!/usr/bin/env python3
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class FsmSimplificationStaticTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fsm_source = (ROOT / "src/mpc_fsm.cpp").read_text(encoding="utf-8")
        cls.fsm_header = (
            ROOT / "include/payload_mpc_controller/mpc_fsm.h"
        ).read_text(encoding="utf-8")
        cls.input_source = (ROOT / "src/mpc_input.cpp").read_text(encoding="utf-8")
        cls.input_header = (
            ROOT / "include/payload_mpc_controller/mpc_input.h"
        ).read_text(encoding="utf-8")
        cls.node_source = (ROOT / "src/mpc_controller_node.cpp").read_text(
            encoding="utf-8"
        )
        cls.launch = (
            ROOT / "launch/quad_wind_mpc_controller.launch"
        ).read_text(encoding="utf-8")

    def test_rc_invalidity_is_separate_from_requested_mode(self):
        self.assertIn("mode_input_valid", self.input_header)
        feed = self.input_source.split("void RC_Data_t::feed", 1)[1]
        feed = feed.split("void RC_Data_t::check_validity", 1)[0]
        self.assertIn("mode_input_valid = false", feed)
        self.assertIn("mode_input_valid = true", feed)
        self.assertNotIn("is_manual_mode = true", feed)

    def test_manual_state_does_not_publish_low_thrust_placeholder(self):
        self.assertNotIn("publish_manual_ctrl", self.fsm_header)
        self.assertNotIn("publish_manual_ctrl", self.fsm_source)
        self.assertIn("publishTakeoffPrestream", self.fsm_source)

    def test_odom_failure_uses_bounded_last_safe_output(self):
        self.assertIn("startOdomFailsafe", self.fsm_header)
        self.assertIn("publishFailsafeHold", self.fsm_header)
        self.assertIn("safe_output_hold_time", self.fsm_source)
        self.assertIn("last_safe_setpoint_", self.fsm_header)
        self.assertNotIn("state_before_offboard", self.input_header)

    def test_px4_offboard_failsafe_parameters_are_checked(self):
        self.assertIn("mavros_msgs/ParamGet", self.node_source)
        self.assertIn("COM_OBL_RC_ACT", self.node_source)
        self.assertIn("COM_OF_LOSS_T", self.node_source)
        self.assertIn("param_get_service", self.launch)

    def test_auto_land_waits_for_actual_px4_mode(self):
        land_case = self.fsm_source.split("case AUTO_LAND:", 1)[1]
        land_case = land_case.split("default:", 1)[0]
        self.assertIn('current_state.mode == "AUTO.LAND"', land_case)
        self.assertIn("auto_land_request_sent_", land_case)

    def test_takeoff_reference_generation_is_unchanged(self):
        takeoff_case = self.fsm_source.split("case AUTO_TAKEOFF:", 1)[1]
        takeoff_case = takeoff_case.split("case AUTO_LAND:", 1)[0]
        self.assertIn("hover_pose_ = takeoff_start_pose_;", takeoff_case)
        self.assertIn("params_.takeoff_.climb_rate * elapsed", takeoff_case)
        self.assertIn("hover_yaw_ = takeoff_start_yaw_;", takeoff_case)

    def test_battery_is_not_an_automatic_control_precondition(self):
        start = self.fsm_source.index("bool MPCFSM::takeoffPreconditions")
        end = self.fsm_source.index("void MPCFSM::startAutoTakeoff", start)
        preconditions = self.fsm_source[start:end]
        self.assertNotIn("bat_is_received", preconditions)
        self.assertNotIn("bat_data", preconditions)


if __name__ == "__main__":
    unittest.main()
