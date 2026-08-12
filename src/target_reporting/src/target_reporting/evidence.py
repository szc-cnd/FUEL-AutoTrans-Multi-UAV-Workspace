from collections import defaultdict, deque
import hashlib
import math
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


def _finite_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _image_points(value):
    """Convert detector points ({u,v} or [u,v]) into a finite Nx2 array."""
    import numpy as np

    if not isinstance(value, (list, tuple)):
        return None
    points = []
    for point in value:
        if isinstance(point, dict):
            u, v = point.get("u"), point.get("v")
        elif isinstance(point, (list, tuple)) and len(point) >= 2:
            u, v = point[0], point[1]
        else:
            continue
        u, v = _finite_float(u), _finite_float(v)
        if u is not None and v is not None:
            points.append((u, v))
    if len(points) < 4:
        return None
    return np.asarray(points, dtype=np.float32).reshape((-1, 1, 2))


def _image_bbox(value):
    if isinstance(value, dict):
        value = [value.get("x"), value.get("y"), value.get("w"), value.get("h")]
    if not isinstance(value, (list, tuple)) or len(value) < 4:
        return None
    values = [_finite_float(item) for item in value[:4]]
    if any(item is None for item in values):
        return None
    x, y, width, height = values
    if width <= 0.0 or height <= 0.0:
        return None
    return x, y, width, height


def draw_detection_overlay(image, event):
    """Legacy helper for explicit geometry overlays.

    The normal evidence path deliberately does not call this function.  Each
    detector already draws its own color/shape-specific box, which is more
    informative than a second generic confirmation box.  Keep the helper for
    compatibility with existing callers and unit tests.
    """
    import cv2
    import numpy as np

    result = event.get("result") or {}
    target_type = str(event.get("target_type", ""))
    color = {
        "color_tag": (0, 165, 255),      # orange in BGR
        "qr_code": (0, 255, 0),          # green
        "thermal_source": (255, 0, 255),  # magenta
    }.get(target_type, (0, 255, 255))

    points = _image_points(result.get("points"))
    center = None
    if points is not None:
        cv2.polylines(image, [np.round(points).astype(np.int32)], True, color, 3, cv2.LINE_AA)
        center = tuple(np.round(points.reshape(-1, 2).mean(axis=0)).astype(int))
    else:
        bbox = _image_bbox(result.get("bbox"))
        if bbox is not None:
            x, y, width, height = bbox
            top_left = (int(round(x)), int(round(y)))
            bottom_right = (int(round(x + width)), int(round(y + height)))
            cv2.rectangle(image, top_left, bottom_right, color, 3, cv2.LINE_AA)
            center = (
                int(round(x + 0.5 * width)),
                int(round(y + 0.5 * height)),
            )

    if center is None:
        center_u = _finite_float(result.get("center_u"))
        center_v = _finite_float(result.get("center_v"))
        if center_u is not None and center_v is not None:
            center = (int(round(center_u)), int(round(center_v)))

    if center is None:
        return image

    cv2.drawMarker(
        image, center, color, markerType=cv2.MARKER_CROSS,
        markerSize=18, thickness=2, line_type=cv2.LINE_AA,
    )
    label = "CONFIRMED {}".format(target_type)
    label_x = max(4, center[0] - 80)
    label_y = max(24, center[1] - 12)
    cv2.putText(image, label, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX,
                0.60, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, label, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX,
                0.60, color, 1, cv2.LINE_AA)
    return image


def build_evidence_jpeg(image, event, camera_xyz=None, quality=85):
    import cv2

    canvas = image.copy()
    position = event["position"]
    geometry_keys = {
        "points", "bbox", "center_u", "center_v", "u", "v", "cx", "cy",
    }
    result_text = ", ".join(
        "{}={}".format(k, v)
        for k, v in event["result"].items()
        if k not in geometry_keys
    )
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
    # Keep the detector's original debug image untouched: color_tag_detector,
    # qr_detector, and uvc_ubuntu each draw their own target-specific box.
    # target_reporting only adds the bottom evidence information panel and
    # does not redraw a generic orange confirmation box.
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
