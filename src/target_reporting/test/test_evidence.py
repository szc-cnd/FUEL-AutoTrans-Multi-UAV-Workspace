import os
import sys
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))

from target_reporting.evidence import draw_detection_overlay, evidence_overlay_layout


class EvidenceLayoutTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
