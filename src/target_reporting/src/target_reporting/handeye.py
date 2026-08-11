"""ROS-independent geometry helpers for camera/body hand-eye calibration."""

from collections import defaultdict, deque

import numpy as np


def _as_float_array(value, shape=None):
    array = np.asarray(value, dtype=np.float64)
    if shape is not None and array.shape != shape:
        raise ValueError("expected shape {}, got {}".format(shape, array.shape))
    return array


def _frame_name(frame):
    return str(frame).strip().lstrip("/")


def checkerboard_object_points(pattern_size, square_size_m):
    """Return planar 3-D points for ``(columns, rows)`` inner corners."""
    if len(pattern_size) != 2:
        raise ValueError("pattern_size must contain columns and rows")
    columns, rows = (int(pattern_size[0]), int(pattern_size[1]))
    square_size_m = float(square_size_m)
    if columns < 2 or rows < 2:
        raise ValueError("checkerboard needs at least 2 inner corners per axis")
    if square_size_m <= 0.0:
        raise ValueError("square_size_m must be positive")
    points = np.zeros((columns * rows, 3), dtype=np.float64)
    points[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    )
    return points


def quaternion_to_matrix(quaternion_xyzw):
    """Convert a ROS-order quaternion ``[x, y, z, w]`` to a 3x3 matrix."""
    x, y, z, w = _as_float_array(quaternion_xyzw, (4,))
    norm = np.linalg.norm([x, y, z, w])
    if norm <= np.finfo(np.float64).eps:
        raise ValueError("quaternion norm must be nonzero")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(rotation):
    """Convert a 3x3 rotation matrix to a normalized ROS-order quaternion."""
    matrix = _as_float_array(rotation, (3, 3))
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    return quaternion / np.linalg.norm(quaternion)


def pose_matrix(rotation, translation):
    """Build a homogeneous transform using ``p_parent = T @ p_child``."""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = _as_float_array(rotation, (3, 3))
    transform[:3, 3] = _as_float_array(translation, (3,))
    return transform


def invert_transform(transform):
    """Invert a rigid homogeneous transform."""
    matrix = _as_float_array(transform, (4, 4))
    rotation = matrix[:3, :3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ matrix[:3, 3]
    return inverse


def _rotation_angle_deg(rotation):
    matrix = _as_float_array(rotation, (3, 3))
    cosine = np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def select_diverse_samples(
    samples, min_translation_m, min_rotation_deg, max_samples
):
    """Select motion-diverse samples across the complete input sequence.

    The input sequence is time ordered. First retain samples that differ from
    the last retained body pose by the requested translation or rotation. If
    that set is larger than ``max_samples``, subsample it uniformly so the
    whole recording contributes instead of only its beginning.
    """
    ordered = list(samples)
    if not ordered:
        return []

    diverse = [ordered[0]]
    for sample in ordered[1:]:
        previous = diverse[-1]["body_pose"]
        current = sample["body_pose"]
        translation_delta = np.linalg.norm(
            current[:3, 3] - previous[:3, 3]
        )
        rotation_delta = _rotation_angle_deg(
            previous[:3, :3].T @ current[:3, :3]
        )
        if (
            translation_delta >= float(min_translation_m)
            or rotation_delta >= float(min_rotation_deg)
        ):
            diverse.append(sample)

    limit = int(max_samples)
    if limit <= 0 or len(diverse) <= limit:
        return diverse

    # Use the full sequence: choose evenly spaced indices and always retain
    # the final pose. This avoids a long recording being represented by its
    # first few seconds only.
    indices = [
        int(round(index * len(diverse) / float(limit - 1)))
        for index in range(limit - 1)
    ]
    indices.append(len(diverse) - 1)
    return [diverse[index] for index in indices]


def lookup_transform_from_edges(edges, target_frame, source_frame):
    """Compose a transform from ``source_frame`` to ``target_frame``.

    Each edge is ``(parent_frame, child_frame, T_parent_child)``. The returned
    matrix maps coordinates in ``source_frame`` into ``target_frame``.
    """
    target = _frame_name(target_frame)
    source = _frame_name(source_frame)
    if target == source:
        return np.eye(4, dtype=np.float64)
    graph = defaultdict(list)
    for parent, child, transform in edges:
        parent = _frame_name(parent)
        child = _frame_name(child)
        matrix = _as_float_array(transform, (4, 4))
        graph[child].append((parent, matrix))
        graph[parent].append((child, invert_transform(matrix)))

    queue = deque([(source, np.eye(4, dtype=np.float64))])
    visited = {source}
    while queue:
        current, source_to_current = queue.popleft()
        for neighbor, current_to_neighbor in graph[current]:
            if neighbor in visited:
                continue
            source_to_neighbor = current_to_neighbor @ source_to_current
            if neighbor == target:
                return source_to_neighbor
            visited.add(neighbor)
            queue.append((neighbor, source_to_neighbor))
    raise ValueError(
        "no transform path from {} to {}".format(source_frame, target_frame)
    )


def reparent_camera_transform(T_body_camera_optical, T_camera_link_optical):
    """Convert ``body→optical`` to ``body→camera_link``.

    Both matrices map the optical-frame point into their named parent frame:
    ``T_body_camera_optical`` maps optical to body and
    ``T_camera_link_optical`` maps optical to camera_link.
    """
    return _as_float_array(T_body_camera_optical, (4, 4)) @ invert_transform(
        T_camera_link_optical
    )


def solve_opencv_handeye(body_poses_world, board_poses_camera, method="park"):
    """Solve ``T_body_camera`` from FAST-LIO and checkerboard poses.

    ``body_poses_world`` are ``T_world_body`` matrices from ``/Odometry``.
    ``board_poses_camera`` are ``T_camera_board`` matrices from ``solvePnP``.
    OpenCV returns ``T_body_camera`` for the optical camera frame.
    """
    if len(body_poses_world) != len(board_poses_camera):
        raise ValueError("body and board pose counts must match")
    if len(body_poses_world) < 3:
        raise ValueError("at least 3 pose pairs are required")
    import cv2

    methods = {
        "tsai": cv2.CALIB_HAND_EYE_TSAI,
        "park": cv2.CALIB_HAND_EYE_PARK,
        "horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    key = str(method).lower()
    if key not in methods:
        raise ValueError("unsupported hand-eye method: {}".format(method))
    robot_rotations = [
        _as_float_array(pose, (4, 4))[:3, :3] for pose in body_poses_world
    ]
    robot_translations = [
        _as_float_array(pose, (4, 4))[:3, 3].reshape(3, 1)
        for pose in body_poses_world
    ]
    target_rotations = [
        _as_float_array(pose, (4, 4))[:3, :3] for pose in board_poses_camera
    ]
    target_translations = [
        _as_float_array(pose, (4, 4))[:3, 3].reshape(3, 1)
        for pose in board_poses_camera
    ]
    rotation, translation = cv2.calibrateHandEye(
        robot_rotations,
        robot_translations,
        target_rotations,
        target_translations,
        method=methods[key],
    )
    return pose_matrix(rotation, np.asarray(translation, dtype=np.float64).reshape(3))
