import os
import sys
import unittest

import numpy as np


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))

from target_reporting.handeye import (  # noqa: E402
    checkerboard_object_points,
    reparent_camera_transform,
    select_diverse_samples,
)


class HandEyeMathTests(unittest.TestCase):
    def test_checkerboard_uses_inner_corners_and_square_size(self):
        points = checkerboard_object_points((11, 8), 0.04)
        self.assertEqual(points.shape, (88, 3))
        self.assertTrue(np.allclose(points[0], [0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(points[1], [0.04, 0.0, 0.0]))
        self.assertTrue(np.allclose(points[-1], [0.40, 0.28, 0.0]))

    def test_reparenting_preserves_the_camera_point(self):
        t_body_link = np.eye(4)
        t_body_link[:3, 3] = [0.12, -0.03, 0.08]
        t_link_optical = np.eye(4)
        t_link_optical[:3, 3] = [0.01, 0.02, -0.04]
        t_body_optical = t_body_link @ t_link_optical
        self.assertTrue(
            np.allclose(
                reparent_camera_transform(t_body_optical, t_link_optical),
                t_body_link,
            )
        )

    def test_sample_selection_spans_the_full_sequence(self):
        samples = []
        for index in range(100):
            pose = np.eye(4, dtype=np.float64)
            pose[0, 3] = float(index)
            samples.append({"index": index, "body_pose": pose})

        selected = select_diverse_samples(
            samples,
            min_translation_m=0.0,
            min_rotation_deg=0.0,
            max_samples=5,
        )

        self.assertEqual(
            [sample["index"] for sample in selected],
            [0, 25, 50, 75, 99],
        )


if __name__ == "__main__":
    unittest.main()
