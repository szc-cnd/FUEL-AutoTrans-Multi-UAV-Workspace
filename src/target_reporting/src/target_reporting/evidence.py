from collections import defaultdict, deque
import hashlib
import threading


class TimestampedImageCache:
    def __init__(self, max_items=40):
        self.max_items = max(1, int(max_items))
        self._items = defaultdict(lambda: deque(maxlen=self.max_items))
        self._lock = threading.Lock()

    def add(self, source, stamp, image):
        with self._lock:
            self._items[source].append((float(stamp), image))

    def nearest(self, source, stamp, tolerance_s):
        with self._lock:
            items = list(self._items.get(source, ()))
        if not items:
            return None
        best_stamp, best_image = min(items, key=lambda item: abs(item[0] - float(stamp)))
        if abs(best_stamp - float(stamp)) > float(tolerance_s):
            return None
        return best_image.copy()


def evidence_overlay_layout(image_shape, line_count, margin=12, line_height=22):
    """Return the top and height of the dedicated bottom evidence panel."""
    if len(image_shape) < 2:
        raise ValueError("image_shape must contain height and width")
    height = int(image_shape[0])
    if height <= 0:
        raise ValueError("image height must be positive")
    line_count = max(1, int(line_count))
    margin = max(0, int(margin))
    line_height = max(1, int(line_height))
    requested_height = 2 * margin + line_count * line_height
    panel_height = min(height, requested_height)
    return height - panel_height, panel_height


def build_evidence_jpeg(image, event, camera_xyz=None, quality=85):
    import cv2

    canvas = image.copy()
    position = event["position"]
    result_text = ", ".join("{}={}".format(k, v) for k, v in event["result"].items())
    lines = [
        "{}  {}".format(event["target_type"], result_text),
        "Channel: X={:.3f} Y={:.3f} Z={:.3f} m".format(
            position["x"], position["y"], position["z"]
        ),
        "UAV: {}  ID: {}".format(event["drone_id"], event["target_id"]),
        "Time: {:.6f}".format(event["timestamp"]),
    ]
    if camera_xyz is not None:
        lines.insert(1, "Camera: X={:.3f} Y={:.3f} Z={:.3f} m".format(*camera_xyz))

    height, width = canvas.shape[:2]
    panel_top, panel_height = evidence_overlay_layout(canvas.shape, len(lines))
    panel = canvas.copy()
    cv2.rectangle(panel, (0, panel_top), (width - 1, height - 1), (0, 0, 0), -1)
    cv2.addWeighted(panel, 0.62, canvas, 0.38, 0, canvas)

    y = panel_top + 29
    for line in lines:
        cv2.putText(canvas, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)
        y += 24
    ok, encoded = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return encoded.tobytes()


def image_metadata(image_id, data):
    return {
        "id": image_id,
        "encoding": "jpeg",
        "size": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
    }
