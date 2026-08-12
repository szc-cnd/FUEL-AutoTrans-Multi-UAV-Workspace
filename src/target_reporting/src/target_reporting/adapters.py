import json


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
        result["_qr_validated"] = bool(data.get("validated", False))
        result["_qr_confirmable"] = bool(
            data.get("confirmable", data.get("stable", False))
        )
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
        return _copy_geometry(data, result), None

    if not detected:
        return None
    return {"detected": True}, None
