#!/usr/bin/env python3
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class TakeoffTimeoutHoverStaticTest(unittest.TestCase):
    def test_timeout_uses_hover_fallback_instead_of_manual_abort(self):
        source = (ROOT / "src/mpc_fsm.cpp").read_text(encoding="utf-8")
        header = (ROOT / "include/payload_mpc_controller/mpc_fsm.h").read_text(
            encoding="utf-8"
        )

        self.assertNotIn("handleAutoTakeoffTimeout", header)
        self.assertNotIn("completeAutoTakeoff", header)
        self.assertNotIn("handleAutoTakeoffTimeout", source)
        self.assertNotIn("completeAutoTakeoff", source)

        takeoff_body = source.split("case AUTO_TAKEOFF:", 1)[1]
        takeoff_body = takeoff_body.split("case AUTO_LAND:", 1)[0]
        self.assertIn("params_.takeoff_.climb_rate", takeoff_body)
        self.assertNotIn("position_tolerance", takeoff_body)
        self.assertNotIn("velocity_tolerance", takeoff_body)
        self.assertNotIn("settle_time", takeoff_body)
        self.assertNotIn("takeoff_.timeout", takeoff_body)

        params = (ROOT / "config/mpc.yaml").read_text(encoding="utf-8")
        takeoff_params = params.split("takeoff:", 1)[1].split("land:", 1)[0]
        self.assertIn("climb_rate: 0.15", takeoff_params)
        self.assertNotIn("position_tolerance:", takeoff_params)
        self.assertNotIn("velocity_tolerance:", takeoff_params)
        self.assertNotIn("settle_time:", takeoff_params)
        self.assertNotIn("timeout:", takeoff_params)


if __name__ == "__main__":
    unittest.main()
