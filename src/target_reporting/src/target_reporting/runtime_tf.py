"""Validation and field conversion helpers for runtime TF publishers."""

import math


def _finite_vector(values, length, name):
    values = [float(value) for value in values]
    if len(values) != length or not all(math.isfinite(value) for value in values):
        raise ValueError("{} must contain {} finite values".format(name, length))
    return values


def build_odometry_transform(parent_frame, child_frame, translation, quaternion_xyzw, stamp):
    parent_frame = str(parent_frame).strip()
    child_frame = str(child_frame).strip()
    if not parent_frame or not child_frame:
        raise ValueError("parent_frame and child_frame are required")
    return {
        "header": {"frame_id": parent_frame, "stamp": float(stamp)},
        "child_frame_id": child_frame,
        "transform": {
            "translation": _finite_vector(translation, 3, "translation"),
            "rotation": _finite_vector(quaternion_xyzw, 4, "quaternion_xyzw"),
        },
    }


def validate_calibration_document(document):
    if not isinstance(document, dict):
        raise ValueError("calibration document must be a mapping")
    if document.get("parent_frame") != "body":
        raise ValueError("calibration parent_frame must be body")
    if document.get("child_frame") != "camera_link":
        raise ValueError("calibration child_frame must be camera_link")
    translation = _finite_vector(document.get("translation_m", []), 3, "translation_m")
    quaternion = _finite_vector(
        document.get("quaternion_xyzw", []), 4, "quaternion_xyzw"
    )
    if sum(value * value for value in quaternion) <= 1e-12:
        raise ValueError("quaternion_xyzw must have nonzero norm")
    return translation, quaternion
