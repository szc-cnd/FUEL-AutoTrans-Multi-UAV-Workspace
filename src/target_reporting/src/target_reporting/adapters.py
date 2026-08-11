import json


def parse_color_status(text):
    data = json.loads(text)
    # Candidate messages are valid geometric observations even before the
    # detector's own temporal filter becomes stable. Legacy result_text still
    # requires detected=true and stable=true.
    if not data.get("detected"):
        return None
    if not data.get("stable") and not data.get("candidate"):
        return None
    return {"color": str(data.get("color", "unknown"))}, data.get("score")


def parse_qr_status(text):
    data = json.loads(text)
    if not data.get("detected") or data.get("held"):
        return None
    return {"content": str(data.get("data", ""))}, None


def parse_thermal_status(detected):
    if not detected:
        return None
    return {"detected": True}, None
