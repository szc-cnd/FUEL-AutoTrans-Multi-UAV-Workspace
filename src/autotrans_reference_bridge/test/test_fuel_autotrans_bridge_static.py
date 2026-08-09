#!/usr/bin/env python3
"""检查 FUEL B-spline 桥接器的失效处理和发布语义。"""

import unittest
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "src" / "fuel_autotrans_bridge_node.cpp"


class FuelAutoTransBridgeStaticTest(unittest.TestCase):
    def setUp(self):
        self.source = SOURCE.read_text(encoding="utf-8")

    def test_output_is_not_latched_and_no_b_spline_timeout_remains(self):
        self.assertIn("advertise<quadrotor_msgs::PolynomialTraj>(output_topic_, 2, false)", self.source)
        self.assertNotIn("trajectory_timeout", self.source)
        self.assertNotIn("timeoutCallback", self.source)

    def test_invalid_input_aborts_without_publishing_action_add(self):
        build_start = self.source.index("bool buildTrajectory")
        callback_start = self.source.index("void bsplineCallback", build_start)
        callback = self.source[callback_start : self.source.index("void publishAbort", callback_start)]
        self.assertIn("if (!buildTrajectory(*input, output))", callback)
        self.assertIn("publishAbort", callback)
        self.assertIn("output_pub_.publish(output)", callback)
        self.assertLess(callback.index("publishAbort"), callback.index("output_pub_.publish(output)"))

    def test_abort_is_explicit_and_non_latched(self):
        self.assertIn("abort_message.action = quadrotor_msgs::PolynomialTraj::ACTION_ABORT", self.source)
        self.assertIn("output_pub_.publish(abort_message)", self.source)
        self.assertIn("ACTION_ABORT", self.source)

    def test_polynomial_coefficients_match_autotrans_descending_convention(self):
        fit_start = self.source.index("bool fitPiece")
        build_start = self.source.index("bool buildTrajectory", fit_start)
        fit_source = self.source[fit_start:build_start]
        serialization_start = fit_source.index("output.data.reserve")
        serialization = fit_source[serialization_start:]

        # AutoTrans 的 Piece 将最高次项放在第 0 列、常数项放在最后一列。
        self.assertRegex(
            serialization,
            r"for\s*\(int order = degree; order >= 0; --order\)",
        )


if __name__ == "__main__":
    unittest.main()
