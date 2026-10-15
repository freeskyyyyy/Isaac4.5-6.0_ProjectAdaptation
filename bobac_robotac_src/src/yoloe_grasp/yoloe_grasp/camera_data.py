"""Timestamp, calibration and coordinate checks for registered RGB-D images."""
from collections import deque
from threading import Lock
import time

import numpy as np


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class CameraSnapshot(tuple):
    def __new__(cls, messages, received_at):
        snapshot = super().__new__(cls, messages)
        snapshot.received_at = received_at
        return snapshot


class CameraFrames:
    def __init__(self, queue_size=20):
        self._lock = Lock()
        self._streams = {key: deque(maxlen=queue_size) for key in ('rgb', 'depth', 'info')}
        self._last_stamp = {}

    def add(self, key, message):
        stamp = stamp_ns(message)
        with self._lock:
            if stamp < self._last_stamp.get(key, 0):
                # Simulation reset: never pair pre-reset messages with the new epoch.
                for stream in self._streams.values():
                    stream.clear()
                self._last_stamp.clear()
            self._last_stamp[key] = stamp
            self._streams[key].append((message, time.monotonic()))

    def snapshot(self, now_ns, tolerance, max_age, static_info=False):
        with self._lock:
            streams = {key: list(value) for key, value in self._streams.items()}
        missing = [key for key, stream in streams.items() if not stream]
        if missing:
            return None, f'Waiting for camera streams: {", ".join(missing)}'
        if now_ns <= 0:
            return None, 'Waiting for nonzero /clock (use_sim_time=true)'
        wall_now = time.monotonic()
        slop_ns = int(tolerance * 1e9)
        max_age_ns = int(max_age * 1e9)
        reason = 'No time-aligned RGB/depth/CameraInfo triplet'
        for rgb, received in reversed(streams['rgb']):
            stamp = stamp_ns(rgb)
            age = now_ns - stamp
            if stamp <= 0 or age < -slop_ns or age > max_age_ns or wall_now - received > max_age:
                reason = 'RGB stamp is zero, stale, or ahead of /clock'
                continue
            depth, depth_received = min(streams['depth'], key=lambda item: abs(stamp_ns(item[0]) - stamp))
            info, info_received = min(streams['info'], key=lambda item: abs(stamp_ns(item[0]) - stamp))
            stamps = [stamp, stamp_ns(depth)]
            if not static_info:
                stamps.append(stamp_ns(info))
            if min(stamps) <= 0 or max(stamps) - min(stamps) > slop_ns:
                reason = f'Camera stamp skew exceeds {tolerance:.3f}s: {stamps}'
                continue
            if wall_now - depth_received > max_age or (not static_info and wall_now - info_received > max_age):
                reason = 'Depth or CameraInfo reception is stale'
                continue
            reception_times = [received, depth_received]
            if not static_info:
                reception_times.append(info_received)
            return CameraSnapshot((rgb, depth, info), min(reception_times)), ''
        return None, reason


def validate_calibration(rgb, depth, info):
    sizes = [(m.width, m.height) for m in (rgb, depth, info)]
    if sizes[0] != sizes[1] or sizes[0] != sizes[2] or min(sizes[0]) <= 0:
        raise ValueError(f'Registered RGB/depth/CameraInfo dimensions must match: {sizes}')
    frames = [m.header.frame_id for m in (rgb, depth, info)]
    if not frames[0] or any(frame != frames[0] for frame in frames):
        raise ValueError(f'Registered camera frame_id mismatch or empty: {frames}')
    k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0 or not np.isclose(k[2, 2], 1):
        raise ValueError(f'CameraInfo has invalid/uncalibrated K: {info.k}')
    if any(not np.isfinite(v) or abs(v) > 1e-8 for v in info.d):
        raise ValueError('Nonzero distortion requires registered rectified images and matching CameraInfo')
    if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
        raise ValueError('Binned/cropped CameraInfo requires adjusted intrinsics')
    return k


def depth_in_metres(array, encoding):
    if encoding == '16UC1':
        depth = np.asarray(array, dtype=np.float32) * 0.001
    elif encoding == '32FC1':
        depth = np.asarray(array, dtype=np.float32).copy()
    else:
        raise ValueError(f'Unsupported depth encoding {encoding}; expected 16UC1 millimetres or 32FC1 metres')
    if depth.ndim != 2:
        raise ValueError(f'Expected single-channel depth, got {depth.shape}')
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    return depth


def optical_to_frame(convention):
    # All backprojection starts in ROS optical axes: right, down, forward.
    matrices = {
        'optical': np.eye(3),
        'usd': np.diag([1., -1., -1.]),  # USD camera: right, up, backward
        'ros_body': np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]),
    }
    if convention not in matrices:
        raise ValueError(f'Unknown camera_frame_convention: {convention}')
    return matrices[convention]


def backproject(u, v, z, k):
    return np.array([(u - k[0, 2]) * z / k[0, 0], (v - k[1, 2]) * z / k[1, 1], z])
