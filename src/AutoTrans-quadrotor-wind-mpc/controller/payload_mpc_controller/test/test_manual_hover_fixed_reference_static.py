#!/usr/bin/env python3
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class ManualHoverFixedReferenceStaticTest(unittest.TestCase):
    def test_manual_hover_edge_reloads_mode_hover_reference(self):
        source = (ROOT / "src/mpc_fsm.cpp").read_text(encoding="utf-8")
        hover_case = source.split("case AUTO_HOVER:", 1)[1]
        hover_case = hover_case.split("case CMD_CTRL:", 1)[0]

        self.assertIn("else if (rc_data.enter_hover_mode)", hover_case)
        edge_body = hover_case.split("else if (rc_data.enter_hover_mode)", 1)[1]
        edge_body = edge_body.split("else if (rc_data.is_command_mode)", 1)[0]
        self.assertIn("update_mode_hover_pose();", edge_body)
        self.assertIn(
            "controller_.setHoverReference(hover_pose_, hover_yaw_);", edge_body
        )

    def test_takeoff_manual_hover_uses_mode_reference(self):
        source = (ROOT / "src/mpc_fsm.cpp").read_text(encoding="utf-8")
        takeoff_case = source.split("case AUTO_TAKEOFF:", 1)[1]
        takeoff_case = takeoff_case.split("case AUTO_LAND:", 1)[0]
        hover_branch = takeoff_case.split("if (rc_data.is_hover_mode)", 1)[1]
        hover_branch = hover_branch.split("if (!rc_data.is_takeoff_mode", 1)[0]

        self.assertIn("update_mode_hover_pose();", hover_branch)
        self.assertNotIn("update_hover_pose();", hover_branch)


if __name__ == "__main__":
    unittest.main()
