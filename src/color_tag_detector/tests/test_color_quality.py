#!/usr/bin/env python3
import os
import sys
import unittest

import cv2
import numpy as np


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

try:
    from color_tag_detector import ColorTagDetector
except ModuleNotFoundError:
    ColorTagDetector = None


@unittest.skipIf(ColorTagDetector is None, "ROS Python modules are unavailable")
class ColorQualityTests(unittest.TestCase):
    def test_solid_irregular_tag_has_high_color_purity(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        contour = np.asarray(
            [[[20, 10]], [[85, 25]], [[70, 85]], [[15, 70]]], dtype=np.int32
        )
        cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
        purity = ColorTagDetector.contour_color_purity(mask, contour)
        self.assertGreater(purity, 0.98)

    def test_textured_region_with_holes_has_low_color_purity(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        contour = np.asarray(
            [[[10, 10]], [[90, 10]], [[90, 90]], [[10, 90]]], dtype=np.int32
        )
        cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
        for x in range(20, 90, 20):
            cv2.rectangle(mask, (x, 15), (x + 9, 85), 0, thickness=-1)
        purity = ColorTagDetector.contour_color_purity(mask, contour)
        self.assertLess(purity, 0.78)


if __name__ == "__main__":
    unittest.main()
