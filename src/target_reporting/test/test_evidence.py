import os
import sys
import unittest
from unittest import mock

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))

from target_reporting import evidence
from target_reporting.evidence import (
    TimestampedImageCache,
    draw_detection_overlay,
    evidence_overlay_layout,
    wrap_evidence_lines,
)


class EvidenceLayoutTests(unittest.TestCase):
    def test_image_cache_returns_only_same_frame_match(self):
        cache = TimestampedImageCache(max_items=4)
        image = np.zeros((8, 8, 3), dtype=np.uint8)
        cache.add("color", 10.0, image)

        self.assertIsNone(cache.nearest_with_stamp("color", 10.20, 0.03))
        match = cache.nearest_with_stamp("color", 10.01, 0.03)
        self.assertIsNotNone(match)
        self.assertAlmostEqual(10.0, match[0])
        self.assertEqual(image.shape, match[1].shape)
        self.assertTrue(cache.has_match("color", 10.01, 0.03))
        self.assertFalse(cache.has_match("color", 10.20, 0.03))
        self.assertFalse(cache.has_match("thermal_d435", 10.0, 0.10))

    def test_overlay_is_reserved_at_bottom_of_image(self):
        y0, panel_height = evidence_overlay_layout((480, 640, 3), line_count=5)

        self.assertGreaterEqual(y0, 300)
        self.assertEqual(y0 + panel_height, 480)
        self.assertGreaterEqual(panel_height, 120)

    def test_confirmed_qr_geometry_is_drawn_on_image(self):
        image = np.zeros((120, 160, 3), dtype=np.uint8)
        event = {
            "target_type": "qr_code",
            "result": {
                "content": "",
                "points": [
                    {"u": 10, "v": 10}, {"u": 100, "v": 10},
                    {"u": 100, "v": 80}, {"u": 10, "v": 80},
                ],
            },
        }
        draw_detection_overlay(image, event)
        # QR overlays are green in BGR; JPEG is not involved in this unit test.
        self.assertGreater(int(image[10, 40, 1]), 200)
        self.assertEqual(int(image[10, 40, 0]), 0)

    def test_confirmed_bbox_geometry_is_drawn_on_image(self):
        image = np.zeros((120, 160, 3), dtype=np.uint8)
        draw_detection_overlay(
            image,
            {
                "target_type": "color_tag",
                "result": {"color": "blue", "bbox": [20, 20, 60, 40]},
            },
        )
        # Color labels use orange in BGR.
        self.assertGreater(int(image[20, 40, 2]), 100)
        self.assertGreater(int(image[20, 40, 1]), 50)

    def test_evidence_jpeg_keeps_detector_image_without_redrawing(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        event = {
            "target_type": "color_tag",
            "drone_id": "uav0",
            "target_id": "uav0-color_tag-001",
            "timestamp": 1.0,
            "result": {"color": "blue", "bbox": [20, 20, 60, 40]},
            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
        }
        with mock.patch.object(
            evidence,
            "draw_detection_overlay",
            side_effect=AssertionError("evidence path must not redraw a box"),
        ):
            jpeg = evidence.build_evidence_jpeg(image, event)
        self.assertTrue(jpeg.startswith(b"\xff\xd8"))
        import cv2
        decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(320, decoded.shape[1])
        self.assertGreater(decoded.shape[0], 240)

    def test_evidence_text_wraps_to_thermal_image_width(self):
        import cv2

        lines = wrap_evidence_lines(
            [
                "thermal_source detected=True, detector_stable=True, "
                "detector_confirmable=True"
            ],
            384 - 24,
        )
        self.assertGreater(len(lines), 1)
        for line in lines:
            width = cv2.getTextSize(
                line, cv2.FONT_HERSHEY_SIMPLEX, 0.50, 1
            )[0][0]
            self.assertLessEqual(width, 384 - 24)


if __name__ == "__main__":
    unittest.main()
