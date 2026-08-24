#!/usr/bin/env python3

import importlib.util
import math
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "livox_imu_to_body.py"
SPEC = importlib.util.spec_from_file_location("livox_imu_to_body", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class LivoxImuToBodyTest(unittest.TestCase):
    def test_pitch_rotation_uses_cav0_install_angle(self):
        rotated = MODULE.rotate_vector(MODULE.rotation_y(15.0), (0.0, 0.0, 1.0))
        self.assertAlmostEqual(rotated[0], math.sin(math.radians(15.0)), places=12)
        self.assertAlmostEqual(rotated[1], 0.0, places=12)
        self.assertAlmostEqual(rotated[2], math.cos(math.radians(15.0)), places=12)

    def test_covariance_rotation_preserves_trace_and_symmetry(self):
        covariance = (1.0, 0.2, 0.0, 0.2, 2.0, 0.1, 0.0, 0.1, 3.0)
        rotated = MODULE.rotate_covariance(MODULE.rotation_y(15.0), covariance)
        self.assertAlmostEqual(rotated[0] + rotated[4] + rotated[8], 6.0, places=12)
        self.assertAlmostEqual(rotated[1], rotated[3], places=12)
        self.assertAlmostEqual(rotated[2], rotated[6], places=12)
        self.assertAlmostEqual(rotated[5], rotated[7], places=12)

    def test_unknown_covariance_is_not_fabricated(self):
        covariance = (-1.0,) + (0.0,) * 8
        self.assertEqual(
            MODULE.rotate_covariance(MODULE.rotation_y(15.0), covariance), covariance
        )


if __name__ == "__main__":
    unittest.main()
