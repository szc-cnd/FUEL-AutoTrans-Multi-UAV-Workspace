import os
import sys
import tempfile
import time
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))

from target_reporting.model import Position, TargetEvent
from target_reporting.store import MissionStore
from target_reporting.transport import ImageAckClient, ImageReportServer, JsonAckClient, JsonReportServer


class EndToEndTests(unittest.TestCase):
    def test_json_and_image_reach_remote_store(self):
        with tempfile.TemporaryDirectory() as local_root, tempfile.TemporaryDirectory() as remote_root:
            local = MissionStore(local_root, "uav1", mission_id="mission")
            remote = MissionStore(remote_root, "receiver", mission_id="mission")
            image_id = "uav1_seq000001_color_tag.jpg"
            jpeg = b"synthetic-jpeg-payload"
            d435_image_id = "uav1_seq000001_thermal_source_d435.jpg"
            d435_jpeg = b"synthetic-d435-jpeg-payload"
            event = TargetEvent(
                seq=1, timestamp=1.5, drone_id="uav1", message_type="confirmed_target",
                target_id="uav1-color_tag-001", target_type="color_tag",
                result={"color": "red"}, position=Position("channel", 1.0, 2.0, 3.0),
                image={"id": image_id, "encoding": "jpeg", "size": len(jpeg)},
            ).to_dict()
            event["d435_image"] = {
                "id": d435_image_id,
                "encoding": "jpeg",
                "size": len(d435_jpeg),
            }
            local.save_image(image_id, jpeg)
            local.save_image(d435_image_id, d435_jpeg)
            local.append_event(event)

            json_server = JsonReportServer("127.0.0.1", 0, remote.append_event)
            image_server = ImageReportServer(
                "127.0.0.1", 0, lambda name, data: bool(remote.save_image(name, data))
            )
            json_server.start()
            image_server.start()
            json_client = JsonAckClient(
                "127.0.0.1", json_server.port, ack_callback=local.mark_event_acked
            )
            image_client = ImageAckClient(
                "127.0.0.1", image_server.port, ack_callback=local.mark_image_acked
            )
            json_client.start()
            image_client.start()
            try:
                json_client.enqueue(event)
                image_client.enqueue((image_id, jpeg))
                image_client.enqueue((d435_image_id, d435_jpeg))
                deadline = time.time() + 3.0
                remote_image = os.path.join(remote.image_dir, image_id)
                remote_d435_image = os.path.join(remote.image_dir, d435_image_id)
                while time.time() < deadline:
                    if (remote.has_event("uav1", 1) and os.path.exists(remote_image)
                            and os.path.exists(remote_d435_image)
                            and not local.pending_events() and not local.pending_images()):
                        break
                    time.sleep(0.02)
                self.assertTrue(remote.has_event("uav1", 1))
                with open(remote_image, "rb") as stream:
                    self.assertEqual(jpeg, stream.read())
                with open(remote_d435_image, "rb") as stream:
                    self.assertEqual(d435_jpeg, stream.read())
                self.assertEqual([], local.pending_events())
                self.assertEqual([], local.pending_images())
            finally:
                json_client.stop()
                image_client.stop()
                json_server.stop()
                image_server.stop()


if __name__ == "__main__":
    unittest.main()
