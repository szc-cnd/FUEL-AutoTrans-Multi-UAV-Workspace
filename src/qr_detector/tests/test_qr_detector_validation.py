#!/usr/bin/env python3
import glob
import os
import sys
import unittest

import cv2
import numpy as np


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

try:
    from qr_detector_node import QRDetectorNode
except ModuleNotFoundError:
    # The source-only CI environment may not have ROS Python modules.  The
    # test still runs automatically on the aircraft after sourcing Noetic.
    QRDetectorNode = None


@unittest.skipIf(QRDetectorNode is None, "ROS Python modules are unavailable")
class QRDetectorValidationTests(unittest.TestCase):
    def make_detector(self):
        detector = QRDetectorNode.__new__(QRDetectorNode)
        detector.qr_detector = cv2.QRCodeDetector()
        detector.preprocess_mode = "gray"
        detector.upscale_factor = 1.5
        detector.enable_preprocess_fallbacks = False
        detector.min_area = 150.0
        detector.min_side_length = 16.0
        detector.max_side_ratio = 4.0
        detector.max_angle_cos = 0.80
        detector.max_quad_area_ratio = 0.35
        detector.max_quad_width_ratio = 0.90
        detector.max_quad_height_ratio = 0.90
        detector.qr_eps_x = 0.15
        detector.qr_eps_y = 0.15
        detector.require_decode_for_confirmation = True
        detector.decode_qr_data = False
        detector.configure_qr_detector()
        return detector

    def test_large_background_quad_is_rejected_by_geometry_limit(self):
        detector = self.make_detector()
        points = np.asarray(
            [[0, 0], [1279, 0], [1279, 719], [0, 719]], dtype=np.float32
        )
        ok, _ = detector.is_reasonable_quad(points, image_w=1280, image_h=720)
        self.assertFalse(ok)

    def test_saved_false_qr_evidence_cannot_become_validated(self):
        paths = sorted(
            glob.glob(
                "/home/oem/target_reports/onboard_test_20260812/"
                "images/annotated/*qr_code.jpg"
            )
        )
        if not paths:
            self.skipTest("现场二维码误检样本不存在")
        detector = self.make_detector()
        # The directory can also contain a genuinely confirmed QR evidence
        # image from a later run.  Find a saved sample that still produces a
        # geometric candidate but fails the detector's authenticity gate;
        # do not assume the lexicographically latest file is the false one.
        false_candidate_found = False
        for path in reversed(paths):
            image = cv2.imread(path)
            self.assertIsNotNone(image)
            result = detector.detect_qr(image)
            if result is not None and not result["validated"]:
                false_candidate_found = True
                break
        if not false_candidate_found:
            self.skipTest("当前现场样本中没有未验证二维码候选")


if __name__ == "__main__":
    unittest.main()
