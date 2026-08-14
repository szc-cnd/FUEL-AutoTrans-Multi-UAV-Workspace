#!/usr/bin/env python3
import os
import sys
import unittest
from collections import deque

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

    def test_end_width_ratio_rejects_road_cone_but_keeps_partial_tag(self):
        road_cone = np.asarray(
            [[[25, 0]], [[75, 0]], [[100, 100]], [[0, 100]]], dtype=np.int32
        )
        partial_tag = np.asarray(
            [[[5, 0]], [[95, 0]], [[100, 100]], [[0, 100]]], dtype=np.int32
        )

        self.assertLess(ColorTagDetector.contour_end_width_ratio(road_cone), 0.83)
        self.assertGreater(
            ColorTagDetector.contour_end_width_ratio(partial_tag), 0.83
        )

    def test_confirmation_rectangularity_separates_recorded_tags_from_clutter(self):
        config_path = os.path.join(ROOT, "config", "color_thresholds.yaml")
        with open(config_path, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
            threshold = float(config["min_confirmation_rectangularity"])
            candidate_threshold = float(config["min_candidate_rectangularity"])

        # 2026-08-12 实测：真标签最低约 0.868；按钮、布料分别约 0.83、0.72。
        self.assertEqual(candidate_threshold, threshold)
        self.assertTrue(all(value >= threshold for value in (0.868, 0.90, 0.942)))
        self.assertTrue(all(value < threshold for value in (0.83, 0.72)))

    def test_physical_shape_rejects_recorded_printed_strips(self):
        # 2026-08-13 保存的蓝色保护膜误检尺寸（m）。
        false_sizes = (
            (0.29, 0.07),
            (0.16, 0.03),
            (0.18, 0.03),
            (0.05, 0.03),
            (0.17, 0.04),
            (0.03, 0.16),
            (0.02, 0.10),
            (0.02, 0.14),
            (0.12, 0.04),
        )
        for width, height in false_sizes:
            reasons = ColorTagDetector.physical_shape_confirmation_reasons(
                width, height, 0.06, 0.30, 3.0
            )
            self.assertTrue(reasons, (width, height))

    def test_physical_shape_accepts_compact_color_tag(self):
        # 红色标签，以及 2026-08-13 第 6 张中未完全入镜的黄色标签。
        for width, height in ((0.14, 0.16), (0.21, 0.15)):
            reasons = ColorTagDetector.physical_shape_confirmation_reasons(
                width, height, 0.06, 0.30, 3.0
            )
            self.assertEqual(reasons, [])

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

    def make_temporal_detector(self):
        detector = ColorTagDetector.__new__(ColorTagDetector)
        detector.colors = {"red": {}}
        detector.stable_window = 3
        detector.stable_min_count = 2
        detector.max_pixel_jump = 50.0
        detector.max_depth_jump = 0.25
        detector.max_area_ratio_jump = 2.0
        detector.history = {"red": deque(maxlen=3)}
        return detector

    @staticmethod
    def observation(**overrides):
        value = {
            "u": 50,
            "v": 50,
            "depth": 1.0,
            "depth_valid": True,
            "point_camera": [0.0, 0.0, 1.0],
            "area": 1200.0,
            "confirmation_quality": True,
        }
        value.update(overrides)
        return value

    def test_unconfirmable_observation_does_not_build_stability(self):
        detector = self.make_temporal_detector()
        observation = self.observation(confirmation_quality=False)

        for _ in range(3):
            info = detector.update_stability({"red": observation})

        self.assertFalse(info["red"]["stable"])
        self.assertEqual(info["red"]["count"], 0)

    def test_area_jump_resets_stability_track(self):
        detector = self.make_temporal_detector()
        detector.update_stability({"red": self.observation()})

        info = detector.update_stability(
            {"red": self.observation(area=3000.0)}
        )

        self.assertFalse(info["red"]["stable"])
        self.assertEqual(info["red"]["count"], 1)


if __name__ == "__main__":
    unittest.main()
