import json
import math
from typing import Any, Dict


TARGET_TYPES = {"color_tag", "qr_code", "thermal_source"}
MESSAGE_TYPES = {"observation", "confirmed_target", "target_update"}


def validate_event(event: Dict[str, Any]) -> None:
    required = {
        "version", "seq", "timestamp", "drone_id", "message_type",
        "target_id", "target_type", "result", "position",
    }
    missing = required.difference(event)
    if missing:
        raise ValueError("missing fields: {}".format(", ".join(sorted(missing))))
    if event["target_type"] not in TARGET_TYPES:
        raise ValueError("invalid target_type")
    if event["message_type"] not in MESSAGE_TYPES:
        raise ValueError("invalid message_type")
    if not isinstance(event["seq"], int) or event["seq"] < 0:
        raise ValueError("seq must be a non-negative integer")
    if not isinstance(event["result"], dict):
        raise ValueError("result must be an object")
    position = event["position"]
    if not isinstance(position, dict):
        raise ValueError("position must be an object")
    if position.get("frame_id") != "channel" or position.get("unit") != "m":
        raise ValueError("position must use channel metres")
    for axis in ("x", "y", "z"):
        value = position.get(axis)
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise ValueError("position {} must be finite".format(axis))
    if not math.isfinite(float(event["timestamp"])):
        raise ValueError("timestamp must be finite")


def encode_json_line(payload: Dict[str, Any]) -> bytes:
    return (json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")


def decode_json_line(data: bytes) -> Dict[str, Any]:
    text = data.decode("utf-8").strip()
    if not text:
        raise ValueError("empty JSON line")
    payload = json.loads(text)
    if not isinstance(payload, dict):
        raise ValueError("JSON line must contain an object")
    return payload

