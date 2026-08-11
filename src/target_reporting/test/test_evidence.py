import os
import sys
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))

from target_reporting.evidence import evidence_overlay_layout


class EvidenceLayoutTests(unittest.TestCase):
    def test_overlay_is_reserved_at_bottom_of_image(self):
        y0, panel_height = evidence_overlay_layout((480, 640, 3), line_count=5)

        self.assertGreaterEqual(y0, 300)
        self.assertEqual(y0 + panel_height, 480)
        self.assertGreaterEqual(panel_height, 120)


if __name__ == "__main__":
    unittest.main()
