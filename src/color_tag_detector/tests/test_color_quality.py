#!/usr/bin/env python3
import os
import sys
import unittest

import cv2
import numpy as np
import yaml


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

try:
    from color_tag_detector import ColorTagDetector
except ModuleNotFoundError:
    ColorTagDetector = None


@unittest.skipIf(ColorTagDetector is None, "ROS Python modules are unavailable")
class ColorQualityTests(unittest.TestCase):
    def test_configured_hue_ranges_cover_full_opencv_circle(self):
        config_path = os.path.join(ROOT, "config", "color_thresholds.yaml")
        with open(config_path, encoding="utf-8") as stream:
            colors = yaml.safe_load(stream)["colors"]
        covered = set()
        for color in colors.values():
            for hsv_range in color.get("ranges", []):
                lower = int(hsv_range["lower"][0])
                upper = int(hsv_range["upper"][0])
                covered.update(range(lower, upper + 1))
        self.assertEqual(covered, set(range(180)))

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

    def test_rectangle_scores_higher_than_round_object(self):
        rectangle = np.asarray(
            [[[10, 10]], [[80, 10]], [[80, 90]], [[10, 90]]], dtype=np.int32
        )
        angles = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
        rounded = np.asarray(
            [
                [[int(round(50 + 30 * np.cos(a))), int(round(50 + 30 * np.sin(a)))]]
                for a in angles
            ],
            dtype=np.int32,
        )
        self.assertGreater(
            ColorTagDetector.contour_rectangularity(rectangle), 0.95
        )
        self.assertLess(ColorTagDetector.contour_rectangularity(rounded), 0.84)

    def test_confirmation_rectangularity_separates_recorded_tags_from_clutter(self):
        config_path = os.path.join(ROOT, "config", "color_thresholds.yaml")
        with open(config_path, encoding="utf-8") as stream:
            threshold = float(
                yaml.safe_load(stream)["min_confirmation_rectangularity"]
            )

        # 2026-08-12 实测：真标签最低约 0.868；按钮、布料分别约 0.83、0.72。
        self.assertTrue(all(value >= threshold for value in (0.868, 0.90, 0.942)))
        self.assertTrue(all(value < threshold for value in (0.83, 0.72)))

    def test_surface_plane_residual_rejects_curved_depth(self):
        detector = ColorTagDetector.__new__(ColorTagDetector)
        detector.surface_depth_max_samples = 2500
        detector.depth_min = 0.3
        detector.depth_max = 5.0
        yy, xx = np.mgrid[0:100, 0:100]
        mask = np.zeros((100, 100), dtype=np.uint8)
        contour = np.asarray(
            [[[15, 15]], [[85, 15]], [[85, 85]], [[15, 85]]], dtype=np.int32
        )
        cv2.drawContours(mask, [contour], -1, 255, thickness=-1)

        planar = (1.2 + 0.001 * xx + 0.0015 * yy).astype(np.float32)
        curved = (planar + 0.00025 * (xx - 50.0) ** 2).astype(np.float32)
        plane_stats = detector.surface_depth_plane_stats(
            planar, "32FC1", mask, contour
        )
        curve_stats = detector.surface_depth_plane_stats(
            curved, "32FC1", mask, contour
        )
        self.assertLess(plane_stats["plane_residual_std"], 0.001)
        self.assertGreater(curve_stats["plane_residual_std"], 0.05)


if __name__ == "__main__":
    unittest.main()
