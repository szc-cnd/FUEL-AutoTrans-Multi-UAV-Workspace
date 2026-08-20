#!/usr/bin/env python3
import os
import sys
import unittest

import cv2
import numpy as np


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from thermal_detect import detect_hotspot


class HotspotDetectorTests(unittest.TestCase):
    def test_complete_hotspot_does_not_touch_roi_border(self):
        image = np.zeros((288, 384), dtype=np.uint8)
        cv2.rectangle(image, (150, 90), (200, 150), 255, -1)
        result = detect_hotspot(image)
        self.assertTrue(result["detected"])
        self.assertFalse(result["touches_roi_border"])

    def test_clipped_hotspot_is_marked_as_roi_border_touching(self):
        image = np.zeros((288, 384), dtype=np.uint8)
        # The detector ROI ends at x=354, so this hotspot is clipped by it.
        cv2.rectangle(image, (340, 90), (370, 150), 255, -1)
        result = detect_hotspot(image)
        self.assertTrue(result["detected"])
        self.assertTrue(result["touches_roi_border"])

    def test_brightest_valid_hotspot_remains_selected(self):
        image = np.zeros((288, 384), dtype=np.uint8)
        cv2.rectangle(image, (90, 90), (130, 130), 180, -1)
        cv2.rectangle(image, (220, 90), (260, 130), 255, -1)
        result = detect_hotspot(image)
        self.assertTrue(result["detected"])
        self.assertGreater(result["cx"], 200)


if __name__ == "__main__":
    unittest.main()
