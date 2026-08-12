#!/usr/bin/env python3
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class ManualHoverReferenceStaticTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (ROOT / "src/mpc_fsm.cpp").read_text(encoding="utf-8")

    def test_takeoff_to_hover_locks_current_mode_reference(self):
        takeoff_case = self.source.split("case AUTO_TAKEOFF:", 1)[1]
        takeoff_case = takeoff_case.split("case AUTO_LAND:", 1)[0]
        hover_branch = takeoff_case.split(
            "if (rc_mode_available && rc_data.is_hover_mode)", 1
        )[1]
        hover_branch = hover_branch.split(
            "if (rc_mode_available && rc_data.is_command_mode)", 1
        )[0]
        self.assertIn("update_mode_hover_pose();", hover_branch)

    def test_command_to_low_or_middle_locks_current_position(self):
        command_case = self.source.split("case CMD_CTRL:", 1)[1]
        command_case = command_case.split("case AUTO_TAKEOFF:", 1)[0]
        self.assertIn(
            "rc_data.is_takeoff_mode || rc_data.is_hover_mode", command_case
        )
        self.assertIn("update_mode_hover_pose();", command_case)


if __name__ == "__main__":
    unittest.main()
