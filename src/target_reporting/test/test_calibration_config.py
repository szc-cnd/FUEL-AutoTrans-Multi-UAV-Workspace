import os
import sys
import unittest

import numpy as np


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))

from target_reporting.calibration import (  # noqa: E402
    build_result_document,
    default_calibration_config,
)


class CalibrationConfigTests(unittest.TestCase):
    def test_default_board_configuration_matches_physical_board(self):
        config = default_calibration_config()
        self.assertEqual(config.pattern_cols, 11)
        self.assertEqual(config.pattern_rows, 8)
        self.assertEqual(config.square_size_m, 0.04)

    def test_result_document_requires_body_camera_link_and_quality_metrics(self):
        result = build_result_document(
            np.eye(4),
            default_calibration_config(),
            samples_used=12,
            metrics={"reprojection_rms_px": 0.4},
            frames={"parent_frame": "body", "child_frame": "camera_link"},
        )
        self.assertEqual(result["parent_frame"], "body")
        self.assertEqual(result["child_frame"], "camera_link")
        self.assertEqual(result["samples_used"], 12)
        self.assertIn("metrics", result)
        with self.assertRaises(ValueError):
            build_result_document(
                np.eye(4),
                default_calibration_config(),
                samples_used=0,
                metrics={},
                frames={"parent_frame": "body", "child_frame": "camera_link"},
            )


if __name__ == "__main__":
    unittest.main()
