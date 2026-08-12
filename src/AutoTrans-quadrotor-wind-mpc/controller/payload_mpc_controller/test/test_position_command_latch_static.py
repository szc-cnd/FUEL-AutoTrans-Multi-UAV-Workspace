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


if __name__ == "__main__":
    unittest.main()
