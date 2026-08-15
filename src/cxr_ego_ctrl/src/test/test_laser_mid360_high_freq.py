#!/usr/bin/env python3

import importlib.util
import math
import os
import unittest


SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'laser_mid360_high_freq.py')


def load_bridge_module():
    spec = importlib.util.spec_from_file_location('laser_mid360_high_freq', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Pose:
    def __init__(self, values):
        self.position = type('Position', (), dict(zip(('x', 'y', 'z'), values[:3])))()
        self.orientation = type(
            'Orientation', (), dict(zip(('x', 'y', 'z', 'w'), values[3:])))()


class HighFrequencyVisionBridgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge = load_bridge_module()

    def test_accepts_finite_normalized_pose(self):
        self.assertTrue(self.bridge.is_valid_pose(Pose([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])))

    def test_rejects_non_finite_pose(self):
        self.assertFalse(
            self.bridge.is_valid_pose(Pose([math.nan, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])))

    def test_rejects_zero_quaternion(self):
        self.assertFalse(self.bridge.is_valid_pose(Pose([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0])))


if __name__ == '__main__':
    unittest.main()
