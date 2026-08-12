import os
import sys
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))

from target_reporting.runtime_tf import (  # noqa: E402
    build_odometry_transform,
    validate_calibration_document,
)


class RuntimeTfHelperTests(unittest.TestCase):
    def test_odometry_transform_preserves_original_fields(self):
        transform = build_odometry_transform(
            parent_frame="camera_init",
            child_frame="body",
            translation=[1.0, 2.0, 3.0],
            quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
            stamp=12.5,
        )
        self.assertEqual(transform["header"]["frame_id"], "camera_init")
        self.assertEqual(transform["child_frame_id"], "body")
        self.assertEqual(transform["header"]["stamp"], 12.5)
        self.assertEqual(transform["transform"]["translation"], [1.0, 2.0, 3.0])

    def test_calibration_document_rejects_wrong_runtime_frames(self):
        document = {
            "parent_frame": "camera_init",
            "child_frame": "camera_color_optical_frame",
            "translation_m": [0.0, 0.0, 0.0],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
        with self.assertRaises(ValueError):
            validate_calibration_document(document)


if __name__ == "__main__":
    unittest.main()
