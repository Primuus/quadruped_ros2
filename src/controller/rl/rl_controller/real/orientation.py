from __future__ import annotations

from typing import Any

import numpy as np


def normalize_quaternion_wxyz(value: Any, field_name: str) -> np.ndarray:
    quaternion = np.asarray(value, dtype=np.float64)
    if quaternion.shape != (4,):
        raise ValueError(f'{field_name} must have shape (4,), got {quaternion.shape}')
    if not np.all(np.isfinite(quaternion)):
        raise ValueError(f'{field_name} must contain only finite values')
    norm = float(np.linalg.norm(quaternion))
    if norm <= np.finfo(np.float64).eps:
        raise ValueError(f'{field_name} must have a non-zero norm')
    normalized = quaternion / norm
    if normalized[0] < 0.0:
        normalized = -normalized
    return normalized.astype(np.float32)


def quaternion_conjugate_wxyz(quaternion: Any) -> np.ndarray:
    result = normalize_quaternion_wxyz(quaternion, 'quaternion').copy()
    result[1:] *= -1.0
    return result


def quaternion_multiply_wxyz(left: Any, right: Any) -> np.ndarray:
    lhs = normalize_quaternion_wxyz(left, 'left quaternion').astype(np.float64)
    rhs = normalize_quaternion_wxyz(right, 'right quaternion').astype(np.float64)
    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs
    product = np.asarray(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=np.float64,
    )
    return normalize_quaternion_wxyz(product, 'quaternion product')


def quaternion_rotate_wxyz(quaternion: Any, vector: Any) -> np.ndarray:
    quat = normalize_quaternion_wxyz(quaternion, 'quaternion')
    value = np.asarray(vector, dtype=np.float32)
    if value.shape != (3,):
        raise ValueError(f'vector must have shape (3,), got {value.shape}')
    if not np.all(np.isfinite(value)):
        raise ValueError('vector must contain only finite values')
    xyz = quat[1:]
    cross = 2.0 * np.cross(xyz, value)
    return (
        value + quat[0] * cross + np.cross(xyz, cross)
    ).astype(np.float32)


def transform_imu_sample_to_body(
    sensor_quaternion_wxyz: Any,
    sensor_angular_velocity: Any,
    sensor_to_body_quaternion_wxyz: Any,
) -> tuple[np.ndarray, np.ndarray]:
    """Transform a world-from-sensor attitude and sensor-frame rate to body."""
    sensor_orientation = normalize_quaternion_wxyz(
        sensor_quaternion_wxyz,
        'sensor_quaternion_wxyz',
    )
    sensor_to_body = normalize_quaternion_wxyz(
        sensor_to_body_quaternion_wxyz,
        'sensor_to_body_quaternion_wxyz',
    )
    world_from_body = quaternion_multiply_wxyz(
        sensor_orientation,
        quaternion_conjugate_wxyz(sensor_to_body),
    )
    body_angular_velocity = quaternion_rotate_wxyz(
        sensor_to_body,
        sensor_angular_velocity,
    )
    return world_from_body, body_angular_velocity
