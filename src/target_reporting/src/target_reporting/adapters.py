import json
import math


def _copy_source_stamp(data, result):
    """Keep the detector frame timestamp for point/status pairing."""
    stamp = data.get("stamp", data.get("timestamp"))
    try:
        stamp = float(stamp)
    except (TypeError, ValueError):
        return result
    if math.isfinite(stamp):
        result["_source_stamp"] = stamp
    return result


def _copy_geometry(data, result):
    """Keep detector geometry for evidence images without changing semantics."""
    points = data.get("points")
    if isinstance(points, (list, tuple)) and points:
        result["points"] = points

    bbox = data.get("bbox")
    if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
        result["bbox"] = list(bbox[:4])

    # Color and thermal detectors use u/v or cx/cy; normalize both forms so
    # the evidence renderer can use one schema.
    center_u = data.get("center_u", data.get("u", data.get("cx")))
    center_v = data.get("center_v", data.get("v", data.get("cy")))
    if center_u is not None and center_v is not None:
        result["center_u"] = center_u
        result["center_v"] = center_v
    return result


def parse_color_status(text):
    data = json.loads(text)
    # Candidate messages are valid geometric observations even before the
    # detector's own temporal filter becomes stable. Legacy result_text still
    # requires detected=true and stable=true.
    if not data.get("detected"):
        return None
    if not data.get("stable") and not data.get("candidate"):
        return None
    result = {"color": str(data.get("color", "unknown"))}
    # The detector owns temporal confirmation.  Keep raw candidates visible,
    # but expose the detector gate to target_reporter instead of making the
    # reporting layer guess from repeated spatial observations.
    has_detector_gate = any(
        field in data for field in ("candidate", "stable", "confirmable")
    )
    if has_detector_gate:
        detector_stable = bool(data.get("stable", False))
        detector_confirmable = bool(data.get("confirmable", detector_stable))
    else:
        detector_stable = True
        detector_confirmable = True
    result["_detector_stable"] = detector_stable
    result["_detector_confirmable"] = detector_confirmable
    result = _copy_source_stamp(data, result)
    return _copy_geometry(data, result), data.get("score")


def parse_qr_status(text):
    data = json.loads(text)
    if not data.get("detected") or data.get("held"):
        return None
    result = {"content": str(data.get("data", ""))}
    result = _copy_geometry(data, result)
    # QR raw corners are useful for local RViz observation, but they must not
    # enter CandidateTracker confirmation until the detector has independently
    # validated the QR structure and depth.  Legacy messages without these
    # fields are treated as already-confirmed for backward compatibility.
    if any(
        field in data for field in ("candidate", "stable", "validated", "confirmable")
    ):
        result["validated"] = bool(data.get("validated", False))
        result["confirmable"] = bool(
            data.get("confirmable", data.get("stable", False))
        )
        result["_detector_stable"] = bool(data.get("stable", False))
        result["_detector_confirmable"] = bool(result["confirmable"])
    else:
        # Old /vision/qr_detected messages represented only a stable result.
        result["_detector_stable"] = True
        result["_detector_confirmable"] = True
    result = _copy_source_stamp(data, result)
    return result, None


def parse_thermal_status(detected):
    """Parse legacy Bool and geometry-carrying JSON String messages."""
    if hasattr(detected, "data"):
        detected = detected.data

    if isinstance(detected, str):
        data = json.loads(detected)
        if not data.get("detected"):
            return None
        result = {"detected": True}
        has_detector_gate = any(
            field in data for field in ("candidate", "stable", "confirmable")
        )
        if has_detector_gate:
            detector_stable = bool(data.get("stable", False))
            detector_confirmable = bool(data.get("confirmable", detector_stable))
        else:
            detector_stable = True
            detector_confirmable = True
        result["_detector_stable"] = detector_stable
        result["_detector_confirmable"] = detector_confirmable
        result = _copy_source_stamp(data, result)
        return _copy_geometry(data, result), None

    if not detected:
        return None
    # The legacy Bool topic is published only by the stable detector stream.
    return {
        "detected": True,
        "_detector_stable": True,
        "_detector_confirmable": True,
    }, None
