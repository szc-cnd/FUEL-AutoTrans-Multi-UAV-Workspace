#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "livox_imu_to_body.py"
SPEC = importlib.util.spec_from_file_location("livox_imu_to_body", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class LivoxImuToBodyTest(unittest.TestCase):
    def test_static_gravity_is_rotated_to_body_z(self):
        rotation = MODULE.rotation_y(15.0)
        measured = (-2.50754, 0.00754, 9.48024)
        transformed = MODULE.rotate_vector(rotation, measured)
        self.assertAlmostEqual(transformed[0], 0.0312, delta=0.01)
        self.assertAlmostEqual(transformed[1], 0.0075, delta=0.01)
        self.assertAlmostEqual(transformed[2], 9.8062, delta=0.01)

    def test_covariance_rotation_preserves_isotropic_matrix(self):
        rotation = MODULE.rotation_y(15.0)
        covariance = (0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2)
        transformed = MODULE.rotate_covariance(rotation, covariance)
        for actual, expected in zip(transformed, covariance):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_unknown_covariance_is_preserved(self):
        covariance = (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertEqual(
            MODULE.rotate_covariance(MODULE.rotation_y(15.0), covariance),
            covariance)


if __name__ == "__main__":
    unittest.main()
