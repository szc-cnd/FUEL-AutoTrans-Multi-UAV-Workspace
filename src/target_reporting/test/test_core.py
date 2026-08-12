import json
import math
import os
import sys
import tempfile
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))

from target_reporting.model import Position, TargetEvent
from target_reporting.protocol import decode_json_line, encode_json_line, validate_event
from target_reporting.store import MissionStore
from target_reporting.candidates import CandidateTracker
from target_reporting.adapters import parse_color_status, parse_qr_status, parse_thermal_status


class CoreTests(unittest.TestCase):
    def make_event(self, seq=1):
        return TargetEvent(
            seq=seq,
            timestamp=12.5,
            drone_id="uav1",
            message_type="confirmed_target",
            target_id="uav1-color_tag-001",
            target_type="color_tag",
            result={"color": "red"},
            position=Position("channel", 1.0, 2.0, 3.0),
        )

    def test_json_line_round_trip_preserves_utf8_and_newline(self):
        payload = self.make_event().to_dict()
        payload["result"]["label"] = "红色"
        encoded = encode_json_line(payload)
        self.assertTrue(encoded.endswith(b"\n"))
        self.assertIn("红色", encoded.decode("utf-8"))
        self.assertEqual(payload, decode_json_line(encoded))

    def test_non_finite_coordinate_is_rejected(self):
        payload = self.make_event().to_dict()
        payload["position"]["x"] = math.nan
        with self.assertRaises(ValueError):
            validate_event(payload)

    def test_store_rejects_duplicate_drone_and_sequence(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = MissionStore(tmp, "uav1", mission_id="test")
            self.assertTrue(store.append_event(self.make_event().to_dict()))
            self.assertFalse(store.append_event(self.make_event().to_dict()))
            with open(store.events_path, "r", encoding="utf-8") as stream:
                rows = [json.loads(line) for line in stream if line.strip()]
            self.assertEqual(1, len(rows))

    def test_store_saves_image_before_it_is_queued(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = MissionStore(tmp, "uav1", mission_id="test")
            path = store.save_image("uav1-color_tag-001.jpg", b"jpeg-bytes")
            self.assertTrue(os.path.isfile(path))
            with open(path, "rb") as stream:
                self.assertEqual(b"jpeg-bytes", stream.read())

    def test_store_recovers_only_unacknowledged_events_and_images(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = MissionStore(tmp, "uav1", mission_id="test")
            first = self.make_event(seq=1).to_dict()
            second = self.make_event(seq=2).to_dict()
            second["target_id"] = "uav1-color_tag-002"
            second["image"] = {"id": "second.jpg", "encoding": "jpeg", "size": 3}
            store.append_event(first)
            store.append_event(second)
            store.save_image("second.jpg", b"jpg")
            store.mark_event_acked(1)

            reopened = MissionStore(tmp, "uav1", mission_id="test")
            self.assertEqual([2], [event["seq"] for event in reopened.pending_events()])
            self.assertEqual([("second.jpg", b"jpg")], reopened.pending_images())
            reopened.mark_image_acked("second.jpg")
            self.assertEqual([], reopened.pending_images())

    def test_stable_false_target_does_not_block_real_target_elsewhere(self):
        tracker = CandidateTracker(distance_m=0.3, confirm_hits=2)
        first = {"x": 1.0, "y": 0.0, "z": 0.5}
        real = {"x": 4.0, "y": 0.0, "z": 0.5}
        tracker.update("color_tag", {"color": "red"}, first)
        false_candidate, false_confirmed = tracker.update("color_tag", {"color": "red"}, first)
        tracker.update("color_tag", {"color": "red"}, real)
        real_candidate, real_confirmed = tracker.update("color_tag", {"color": "red"}, real)
        self.assertTrue(false_confirmed)
        self.assertTrue(real_confirmed)
        self.assertNotEqual(false_candidate["local_number"], real_candidate["local_number"])

    def test_detector_status_adapters_accept_explicit_candidates(self):
        self.assertIsNone(parse_color_status('{"detected":true,"stable":false,"color":"red"}'))
        color_candidate, confidence = parse_color_status(
            '{"detected":true,"candidate":true,"stable":false,"color":"red","score":0.7}'
        )
        self.assertEqual(0.7, confidence)
        self.assertEqual("red", color_candidate["color"])
        self.assertFalse(color_candidate["_detector_stable"])
        self.assertFalse(color_candidate["_detector_confirmable"])
        color_result, confidence = parse_color_status(
            '{"detected":true,"stable":true,"color":"red","score":0.9}'
        )
        self.assertEqual(0.9, confidence)
        self.assertTrue(color_result["_detector_stable"])
        self.assertTrue(color_result["_detector_confirmable"])
        qr_result, _ = parse_qr_status(
            '{"detected":true,"data":"A123"}'
        )
        self.assertEqual("A123", qr_result["content"])
        self.assertTrue(qr_result["_detector_confirmable"])
        qr_empty, _ = parse_qr_status(
            '{"detected":true,"held":false,"data":""}'
        )
        self.assertEqual("", qr_empty["content"])
        self.assertEqual(
            False,
            parse_thermal_status(
                '{"detected":true,"candidate":true,"stable":false,"cx":12,"cy":8,"bbox":[2,3,20,16]}'
            )[0]["_detector_confirmable"],
        )

    def test_geometry_does_not_split_one_logical_candidate(self):
        tracker = CandidateTracker(distance_m=0.3, confirm_hits=3)
        result_a = {"color": "blue", "bbox": [10, 10, 20, 20], "center_u": 20}
        result_b = {"color": "blue", "bbox": [11, 10, 20, 20], "center_u": 21}
        tracker.update("color_tag", result_a, {"x": 1.0, "y": 0.0, "z": 0.5})
        tracker.update("color_tag", result_b, {"x": 1.0, "y": 0.0, "z": 0.5})
        candidate, confirmed = tracker.update(
            "color_tag", result_a, {"x": 1.0, "y": 0.0, "z": 0.5}
        )
        self.assertTrue(confirmed)
        self.assertEqual(3, candidate["hits"])

    def test_qr_held_result_is_not_reportable(self):
        self.assertIsNone(parse_qr_status(
            '{"detected":true,"held":true,"data":""}'
        ))

    def test_qr_raw_candidate_is_visible_but_cannot_confirm(self):
        result, _ = parse_qr_status(
            '{"detected":true,"candidate":true,"stable":false,'
            '"validated":false,"confirmable":false,"data":"",'
            '"center_u":100,"center_v":80}'
        )
        self.assertFalse(result["validated"])
        self.assertFalse(result["confirmable"])
        self.assertFalse(result["_detector_stable"])
        self.assertFalse(result["_detector_confirmable"])

        tracker = CandidateTracker(distance_m=0.3, confirm_hits=3)
        position = {"x": 1.0, "y": 0.0, "z": 2.0}
        for _ in range(5):
            candidate, confirmed = tracker.update(
                "qr_code", {"content": ""}, position, allow_confirmation=False
            )
            self.assertFalse(confirmed)
            self.assertEqual(0, candidate["hits"])

        for index in range(3):
            candidate, confirmed = tracker.update(
                "qr_code", {"content": ""}, position, allow_confirmation=True
            )
            self.assertEqual(index + 1, candidate["hits"])
        self.assertTrue(confirmed)

    def test_detector_gate_is_the_only_confirmation_gate(self):
        tracker = CandidateTracker(distance_m=0.3, confirm_hits=1)
        position = {"x": 2.0, "y": 0.0, "z": 1.0}
        candidate, confirmed = tracker.update(
            "thermal_source", {"detected": True}, position,
            allow_confirmation=False,
        )
        self.assertFalse(confirmed)
        self.assertEqual(0, candidate["hits"])

        candidate, confirmed = tracker.update(
            "thermal_source", {"detected": True}, position,
            allow_confirmation=True,
        )
        self.assertTrue(confirmed)
        self.assertEqual(1, candidate["hits"])


if __name__ == "__main__":
    unittest.main()
