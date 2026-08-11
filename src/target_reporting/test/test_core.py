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
from target_reporting.adapters import parse_color_status, parse_qr_status


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
        self.assertEqual(({"color": "red"}, 0.7), parse_color_status(
            '{"detected":true,"candidate":true,"stable":false,"color":"red","score":0.7}'
        ))
        self.assertEqual(({"color": "red"}, 0.9), parse_color_status(
            '{"detected":true,"stable":true,"color":"red","score":0.9}'
        ))
        self.assertEqual(({"content": "A123"}, None), parse_qr_status(
            '{"detected":true,"data":"A123"}'
        ))
        self.assertEqual(({"content": ""}, None), parse_qr_status(
            '{"detected":true,"held":false,"data":""}'
        ))

    def test_qr_held_result_is_not_reportable(self):
        self.assertIsNone(parse_qr_status(
            '{"detected":true,"held":true,"data":""}'
        ))


if __name__ == "__main__":
    unittest.main()
