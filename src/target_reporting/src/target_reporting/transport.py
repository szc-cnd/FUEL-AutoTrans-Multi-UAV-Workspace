import hashlib
import json
import queue
import socket
import threading
import time
from typing import Callable, Dict, Optional, Tuple

from .protocol import decode_json_line, encode_json_line


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = []
    remaining = int(size)
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise EOFError("connection closed before frame completed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv_line(sock: socket.socket, max_size: int = 65536) -> bytes:
    data = bytearray()
    while len(data) < max_size:
        chunk = sock.recv(1)
        if not chunk:
            if data:
                raise EOFError("connection closed before newline")
            return b""
        data.extend(chunk)
        if chunk == b"\n":
            return bytes(data)
    raise ValueError("line exceeds maximum size")


class _BaseServer:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = int(port)
        self._stop = threading.Event()
        self._listener = None
        self._thread = None
        self._clients = set()
        self._lock = threading.Lock()

    def start(self):
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((self.host, self.port))
        self.port = self._listener.getsockname()[1]
        self._listener.listen(4)
        self._listener.settimeout(0.2)
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                client, _ = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            client.settimeout(2.0)
            with self._lock:
                self._clients.add(client)
            threading.Thread(target=self._serve_and_close, args=(client,), daemon=True).start()

    def _serve_and_close(self, client):
        try:
            self._serve(client)
        except (EOFError, OSError, ValueError, json.JSONDecodeError):
            pass
        finally:
            with self._lock:
                self._clients.discard(client)
            try:
                client.close()
            except OSError:
                pass

    def _serve(self, client):
        raise NotImplementedError

    def stop(self):
        self._stop.set()
        if self._listener is not None:
            self._listener.close()
        with self._lock:
            clients = list(self._clients)
        for client in clients:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            client.close()
        if self._thread is not None:
            self._thread.join(2.0)


class JsonReportServer(_BaseServer):
    def __init__(self, host: str, port: int, persist: Callable[[Dict], bool],
                 observe: Optional[Callable[[Dict], None]] = None):
        super().__init__(host, port)
        self.persist = persist
        self.observe = observe
        self._seen = set()
        self._seen_lock = threading.Lock()

    def _serve(self, client):
        while not self._stop.is_set():
            line = recv_line(client)
            if not line:
                return
            event = decode_json_line(line)
            if event.get("message_type") == "observation":
                if self.observe is not None:
                    self.observe(event)
                continue
            key = (str(event.get("drone_id")), int(event["seq"]))
            with self._seen_lock:
                duplicate = key in self._seen
                if not duplicate:
                    saved = bool(self.persist(event))
                    if not saved:
                        client.sendall(encode_json_line({"error": "persist_failed", "seq": event["seq"]}))
                        continue
                    self._seen.add(key)
            client.sendall(encode_json_line({"ack": event["seq"]}))


class ImageReportServer(_BaseServer):
    def __init__(self, host: str, port: int, persist: Callable[[str, bytes], bool], max_size=10_000_000):
        super().__init__(host, port)
        self.persist = persist
        self.max_size = int(max_size)

    def _serve(self, client):
        while not self._stop.is_set():
            line = recv_line(client)
            if not line:
                return
            header = decode_json_line(line)
            image_id = str(header["image_id"])
            size = int(header["size"])
            if size < 1 or size > self.max_size:
                raise ValueError("invalid image size")
            data = recv_exact(client, size)
            digest = hashlib.sha256(data).hexdigest()
            if digest != header.get("sha256"):
                client.sendall(encode_json_line({"image_error": image_id, "reason": "sha256"}))
                continue
            if self.persist(image_id, data):
                client.sendall(encode_json_line({"image_ack": image_id}))


class _QueuedClient:
    def __init__(self, host: str, port: int, max_queue=1000):
        self.host = host
        self.port = int(port)
        self.queue = queue.Queue(maxsize=max_queue)
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(3.0)

    def enqueue(self, item):
        self.queue.put_nowait(item)

    def _exchange(self, sock, item):
        raise NotImplementedError

    def _run(self):
        sock = None
        pending = None
        delay = 0.2
        while not self._stop.is_set():
            if pending is None:
                try:
                    pending = self.queue.get(timeout=0.2)
                except queue.Empty:
                    continue
            try:
                if sock is None:
                    sock = socket.create_connection((self.host, self.port), timeout=2.0)
                    sock.settimeout(3.0)
                self._exchange(sock, pending)
                pending = None
                delay = 0.2
            except (OSError, EOFError, ValueError, json.JSONDecodeError):
                if sock is not None:
                    sock.close()
                    sock = None
                self._stop.wait(delay)
                delay = min(delay * 2.0, 3.0)
        if sock is not None:
            sock.close()


class JsonAckClient(_QueuedClient):
    def __init__(self, host: str, port: int, ack_callback=None, max_queue=1000):
        super().__init__(host, port, max_queue=max_queue)
        self.ack_callback = ack_callback

    def _exchange(self, sock, event):
        sock.sendall(encode_json_line(event))
        ack = decode_json_line(recv_line(sock))
        if ack.get("ack") != event.get("seq"):
            raise ValueError("unexpected JSON ACK")
        if self.ack_callback is not None:
            self.ack_callback(int(event["seq"]))


class ImageAckClient(_QueuedClient):
    def __init__(self, host: str, port: int, ack_callback=None, max_queue=1000):
        super().__init__(host, port, max_queue=max_queue)
        self.ack_callback = ack_callback

    def _exchange(self, sock, item: Tuple[str, bytes]):
        image_id, data = item
        header = {
            "image_id": image_id,
            "size": len(data),
            "sha256": hashlib.sha256(data).hexdigest(),
        }
        sock.sendall(encode_json_line(header) + data)
        ack = decode_json_line(recv_line(sock))
        if ack.get("image_ack") != image_id:
            raise ValueError("unexpected image ACK")
        if self.ack_callback is not None:
            self.ack_callback(image_id)


class LatestJsonClient(_QueuedClient):
    def __init__(self, host: str, port: int):
        super().__init__(host, port, max_queue=1)

    def publish(self, event):
        try:
            self.queue.put_nowait(event)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            self.queue.put_nowait(event)

    def _exchange(self, sock, event):
        sock.sendall(encode_json_line(event))
