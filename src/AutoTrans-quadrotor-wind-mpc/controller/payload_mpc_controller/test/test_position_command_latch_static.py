#!/usr/bin/env python3
"""Regression checks for the persistent FUEL PositionCommand target."""

import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
FSM_HEADER = PACKAGE / "include" / "payload_mpc_controller" / "mpc_fsm.h"
FSM_SOURCE = PACKAGE / "src" / "mpc_fsm.cpp"


class PositionCommandLatchStaticTest(unittest.TestCase):
    def test_last_valid_command_is_stored_independently(self):
        header = FSM_HEADER.read_text(encoding="utf-8")
        self.assertIn("latched_entry_command_", header)
        self.assertIn("entry_command_yaw_", header)
        self.assertIn("last_entry_command_stamp_", header)

    def test_entry_command_does_not_expire_on_msg_timeout(self):
        source = FSM_SOURCE.read_text(encoding="utf-8")
        start = source.index("void MPCFSM::CMD_CTRL_process()")
        end = source.index("void MPCFSM::setEstimateState", start)
        cmd_ctrl = source[start:end]
        self.assertNotIn("params_.msg_timeout_.cmd", cmd_ctrl)
        self.assertNotIn("cmd_timeout_hold_active_", cmd_ctrl)
        self.assertNotIn("timed out", cmd_ctrl)

    def test_position_command_yaw_is_ignored(self):
        source = FSM_SOURCE.read_text(encoding="utf-8")
        start = source.index("void MPCFSM::CMD_CTRL_process()")
        end = source.index("void MPCFSM::setEstimateState", start)
        cmd_ctrl = source[start:end]
        self.assertIn("entry_command_yaw_", cmd_ctrl)
        self.assertNotIn("cmd_data.yaw, cmd_data.yaw_rate", cmd_ctrl)
        self.assertIn("entry_command_yaw_, 0.0", cmd_ctrl)

    def test_streaming_position_command_does_not_relatch_yaw(self):
        source = FSM_SOURCE.read_text(encoding="utf-8")
        start = source.index("void MPCFSM::CMD_CTRL_process()")
        end = source.index("void MPCFSM::setEstimateState", start)
        cmd_ctrl = source[start:end]

        guard = "const bool latch_entry_yaw = !entry_command_active_ ||"
        new_task = (
            "latched_entry_command_.msg.trajectory_id != "
            "cmd_data.msg.trajectory_id;"
        )
        yaw_assignment = "entry_command_yaw_ = landingSearchYawReference("
        activate = "entry_command_active_ = true;"
        self.assertIn(guard, cmd_ctrl)
        self.assertIn(new_task, cmd_ctrl)
        self.assertEqual(cmd_ctrl.count(yaw_assignment), 1)
        self.assertLess(cmd_ctrl.index(guard), cmd_ctrl.index(yaw_assignment))
        self.assertLess(cmd_ctrl.index(new_task), cmd_ctrl.index(yaw_assignment))
        self.assertLess(cmd_ctrl.index(yaw_assignment), cmd_ctrl.index(activate))

        yaw_guard_start = cmd_ctrl.index("if (latch_entry_yaw)")
        yaw_guard_end = cmd_ctrl.index(activate, yaw_guard_start)
        yaw_guard = cmd_ctrl[yaw_guard_start:yaw_guard_end]
        self.assertIn(yaw_assignment, yaw_guard)
        self.assertNotIn("latched_entry_command_ = cmd_data;", yaw_guard)

    def test_position_command_is_forwarded_without_entry_limiter(self):
        source = FSM_SOURCE.read_text(encoding="utf-8")
        header = FSM_HEADER.read_text(encoding="utf-8")
        config = (PACKAGE / "config" / "mpc.yaml").read_text(encoding="utf-8")
        params = (
            PACKAGE / "include" / "payload_mpc_controller" / "mpc_params.h"
        ).read_text(encoding="utf-8")
        start = source.index("void MPCFSM::CMD_CTRL_process()")
        end = source.index("void MPCFSM::setEstimateState", start)
        cmd_ctrl = source[start:end]

        self.assertIn("latched_entry_command_.p, latched_entry_command_.v", cmd_ctrl)
        self.assertIn("latched_entry_command_.a, latched_entry_command_.j", cmd_ctrl)
        self.assertNotIn("EntryCommandReferenceLimiter", header)
        self.assertNotIn("entry_command_reference_limiter_", source)
        self.assertNotIn("entry_command:", config)
        self.assertNotIn("struct EntryCommand", params)


if __name__ == "__main__":
    unittest.main()
