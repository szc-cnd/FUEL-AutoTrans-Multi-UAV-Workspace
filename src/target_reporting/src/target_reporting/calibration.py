"""Configuration and result serialization for the hand-eye calibrator."""

from dataclasses import dataclass

import numpy as np

from target_reporting.handeye import matrix_to_quaternion


@dataclass(frozen=True)
class CalibrationConfig:
    pattern_cols: int = 11
    pattern_rows: int = 8
    square_size_m: float = 0.04


def default_calibration_config():
    return CalibrationConfig()


def _matrix_list(matrix):
    return [[float(value) for value in row] for row in np.asarray(matrix)]


def build_result_document(transform, config, samples_used, metrics, frames, metadata=None):
    """Return a complete, YAML-safe calibration result dictionary."""
    if int(samples_used) <= 0:
        raise ValueError("samples_used must be positive")
    parent_frame = str(frames.get("parent_frame", "")).strip()
    child_frame = str(frames.get("child_frame", "")).strip()
    if not parent_frame or not child_frame:
        raise ValueError("parent_frame and child_frame are required")
    matrix = np.asarray(transform, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError("transform must be a 4x4 matrix")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("transform must contain finite values")
    document = {
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "translation_m": [float(value) for value in matrix[:3, 3]],
        "quaternion_xyzw": [
            float(value) for value in matrix_to_quaternion(matrix[:3, :3])
        ],
        "rotation_matrix": _matrix_list(matrix[:3, :3]),
        "board": {
            "inner_corners": [int(config.pattern_cols), int(config.pattern_rows)],
            "square_size_m": float(config.square_size_m),
        },
        "samples_used": int(samples_used),
        "metrics": {str(key): float(value) for key, value in metrics.items()},
    }
    if metadata:
        document["metadata"] = dict(metadata)
    return document
