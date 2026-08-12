import os
import socket
import sys
import tempfile
import threading
import unittest
import hashlib


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))

from target_reporting.protocol import encode_json_line
from target_reporting.transport import ImageReportServer, JsonReportServer, LatestJsonClient, recv_exact


class TransportTests(unittest.TestCase):
    def test_recv_exact_collects_fragmented_bytes(self):
        left, right = socket.socketpair()
        try:
            thread = threading.Thread(target=lambda: (right.sendall(b"ab"), right.sendall(b"cde")))
            thread.start()
            self.assertEqual(b"abcde", recv_exact(left, 5))
            thread.join(1.0)
        finally:
            left.close()
            right.close()

    def test_json_server_saves_once_and_acks_duplicate(self):
        saved = []
        server = JsonReportServer("127.0.0.1", 0, lambda event: saved.append(event) or True)
        server.start()
        try:
            event = {"drone_id": "uav1", "seq": 7, "message_type": "confirmed_target"}
            with socket.create_connection(("127.0.0.1", server.port), timeout=2.0) as client:
                client_file = client.makefile("rb")
                client.sendall(encode_json_line(event) + encode_json_line(event))
                self.assertEqual({"ack": 7}, __import__("json").loads(client_file.readline()))
                self.assertEqual({"ack": 7}, __import__("json").loads(client_file.readline()))
            self.assertEqual(1, len(saved))
        finally:
            server.stop()

    def test_image_server_accepts_header_and_binary_in_one_write(self):
        saved = {}
        server = ImageReportServer(
            "127.0.0.1", 0, lambda image_id, data: saved.setdefault(image_id, data) is data
        )
        server.start()
        try:
            data = b"fake-jpeg-data"
            header = {
                "image_id": "target.jpg",
                "size": len(data),
                "sha256": hashlib.sha256(data).hexdigest(),
            }
            with socket.create_connection(("127.0.0.1", server.port), timeout=2.0) as client:
                client.settimeout(1.0)
                client.sendall(encode_json_line(header) + data)
                ack = client.recv(1024)
            self.assertIn(b'"image_ack":"target.jpg"', ack)
            self.assertEqual(data, saved["target.jpg"])
        finally:
            server.stop()

    def test_realtime_observation_is_displayed_without_persistence_or_ack(self):
        persisted = []
        observed = []
        server = JsonReportServer(
            "127.0.0.1", 0, lambda event: persisted.append(event) or True,
            observe=lambda event: observed.append(event),
        )
        server.start()
        client = LatestJsonClient("127.0.0.1", server.port)
        client.start()
        try:
            client.publish({"message_type": "observation", "target_id": "live-1", "x": 1.0})
            deadline = __import__("time").time() + 2.0
            while not observed and __import__("time").time() < deadline:
                __import__("time").sleep(0.01)
            self.assertEqual("live-1", observed[-1]["target_id"])
            self.assertEqual([], persisted)
        finally:
            client.stop()
            server.stop()


if __name__ == "__main__":
    unittest.main()
