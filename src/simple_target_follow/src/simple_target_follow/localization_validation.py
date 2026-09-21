#!/usr/bin/env python3
"""Pure calculations used by the read-only localization field tests."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class PoseObservation:
    stamp_s: float
    received_s: float
    position: Vector3
    orientation: Quaternion


def quaternion_norm(value: Sequence[float]) -> float:
    return math.sqrt(sum(float(component) ** 2 for component in value))


def quaternion_yaw(value: Sequence[float]) -> float:
    """Return ENU yaw in radians for an XYZW quaternion."""

    x, y, z, w = (float(component) for component in value)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def angle_difference_deg(first_rad: float, second_rad: float) -> float:
    difference = math.atan2(
        math.sin(first_rad - second_rad), math.cos(first_rad - second_rad)
    )
    return abs(math.degrees(difference))


def vector_distance(first: Sequence[float], second: Sequence[float]) -> float:
    return math.sqrt(
        sum((float(left) - float(right)) ** 2 for left, right in zip(first, second))
    )


def _percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    index = max(0, min(len(ordered) - 1, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


def assess_pose_stream(
    observations: Iterable[PoseObservation],
    minimum_rate_hz: float = 9.5,
    maximum_latency_s: float = 0.15,
    maximum_quaternion_norm_error: float = 0.02,
    maximum_jump_m: float = 0.25,
    maximum_stationary_drift_m: Optional[float] = None,
) -> Dict[str, object]:
    """Assess timestamps, values, rate, latency, jumps and optional drift."""

    samples = list(observations)
    finite = all(
        all(
            math.isfinite(value)
            for value in (
                sample.stamp_s,
                sample.received_s,
                *sample.position,
                *sample.orientation,
            )
        )
        for sample in samples
    )
    duration = (
        samples[-1].received_s - samples[0].received_s if len(samples) >= 2 else 0.0
    )
    rate_hz = (len(samples) - 1) / duration if duration > 0.0 else 0.0
    latencies = [
        sample.received_s - sample.stamp_s
        for sample in samples
        if sample.stamp_s > 0.0 and sample.received_s >= sample.stamp_s
    ]
    norm_errors = [
        abs(quaternion_norm(sample.orientation) - 1.0) for sample in samples
    ]
    jumps = [
        vector_distance(previous.position, current.position)
        for previous, current in zip(samples, samples[1:])
    ]
    drift = (
        max(vector_distance(samples[0].position, sample.position) for sample in samples)
        if samples
        else None
    )
    latency_p95 = _percentile(latencies, 0.95)
    max_norm_error = max(norm_errors) if norm_errors else None
    max_jump = max(jumps) if jumps else None

    reasons: List[str] = []
    if len(samples) < 2:
        reasons.append("fewer than two pose samples")
    if not finite:
        reasons.append("pose contains non-finite values")
    if rate_hz < minimum_rate_hz:
        reasons.append("pose rate is below threshold")
    if len(latencies) != len(samples):
        reasons.append("header timestamps are zero, future-dated, or invalid")
    elif latency_p95 is not None and latency_p95 > maximum_latency_s:
        reasons.append("pose latency is above threshold")
    if max_norm_error is None or max_norm_error > maximum_quaternion_norm_error:
        reasons.append("quaternion is not normalized")
    if max_jump is not None and max_jump > maximum_jump_m:
        reasons.append("position jump is above threshold")
    if (
        maximum_stationary_drift_m is not None
        and drift is not None
        and drift > maximum_stationary_drift_m
    ):
        reasons.append("stationary drift is above threshold")

    return {
        "pass": not reasons,
        "reasons": reasons,
        "sample_count": len(samples),
        "duration_s": round(duration, 4),
        "rate_hz": round(rate_hz, 3),
        "latency_p95_s": None if latency_p95 is None else round(latency_p95, 4),
        "quaternion_norm_max_error": (
            None if max_norm_error is None else round(max_norm_error, 6)
        ),
        "maximum_step_jump_m": None if max_jump is None else round(max_jump, 4),
        "drift_from_start_m": None if drift is None else round(drift, 4),
    }


def assess_pose_pairs(
    pairs: Iterable[Tuple[PoseObservation, PoseObservation]],
    maximum_sync_offset_s: float = 0.08,
    maximum_position_error_m: float = 0.20,
    maximum_yaw_error_deg: float = 10.0,
) -> Dict[str, object]:
    """Compare synchronized vision and PX4 local poses."""

    pair_list = list(pairs)
    offsets = [
        abs(vision.stamp_s - local.stamp_s) for vision, local in pair_list
    ]
    position_errors = [
        vector_distance(vision.position, local.position)
        for vision, local in pair_list
    ]
    yaw_errors = [
        angle_difference_deg(
            quaternion_yaw(vision.orientation),
            quaternion_yaw(local.orientation),
        )
        for vision, local in pair_list
    ]
    reasons: List[str] = []
    if not pair_list:
        reasons.append("no synchronized pose pairs")
    if offsets and max(offsets) > maximum_sync_offset_s:
        reasons.append("pose timestamp offset is above threshold")
    position_p95 = _percentile(position_errors, 0.95)
    yaw_p95 = _percentile(yaw_errors, 0.95)
    if position_p95 is not None and position_p95 > maximum_position_error_m:
        reasons.append("vision-to-local position error is above threshold")
    if yaw_p95 is not None and yaw_p95 > maximum_yaw_error_deg:
        reasons.append("vision-to-local yaw error is above threshold")
    return {
        "pass": not reasons,
        "reasons": reasons,
        "pair_count": len(pair_list),
        "sync_offset_max_s": None if not offsets else round(max(offsets), 4),
        "position_error_p95_m": (
            None if position_p95 is None else round(position_p95, 4)
        ),
        "yaw_error_p95_deg": None if yaw_p95 is None else round(yaw_p95, 3),
    }


def assess_axis_motion(
    baseline: Sequence[PoseObservation],
    moved: Sequence[PoseObservation],
    expected_axis: str,
    expected_distance: float,
    distance_tolerance: float = 0.15,
    cross_axis_tolerance: float = 0.10,
    yaw_tolerance_deg: float = 15.0,
) -> Dict[str, object]:
    """Assess one operator-commanded +X/+Y/+Z translation or positive yaw."""

    if not baseline or not moved:
        return {"pass": False, "reasons": ["baseline or moved samples are missing"]}
    start = baseline[-1]
    end = moved[-1]
    displacement = tuple(
        end.position[index] - start.position[index] for index in range(3)
    )
    yaw_change = math.degrees(
        math.atan2(
            math.sin(quaternion_yaw(end.orientation) - quaternion_yaw(start.orientation)),
            math.cos(quaternion_yaw(end.orientation) - quaternion_yaw(start.orientation)),
        )
    )
    reasons: List[str] = []
    axis = expected_axis.lower()
    if axis in ("x", "y", "z"):
        index = {"x": 0, "y": 1, "z": 2}[axis]
        if abs(displacement[index] - expected_distance) > distance_tolerance:
            reasons.append("expected-axis distance or direction is incorrect")
        if max(abs(displacement[i]) for i in range(3) if i != index) > cross_axis_tolerance:
            reasons.append("cross-axis motion is above threshold")
    elif axis == "yaw":
        expected_deg = math.degrees(expected_distance)
        if abs(yaw_change - expected_deg) > yaw_tolerance_deg:
            reasons.append("yaw angle or direction is incorrect")
    else:
        reasons.append("expected_axis must be x, y, z, or yaw")
    return {
        "pass": not reasons,
        "reasons": reasons,
        "expected_axis": axis,
        "displacement_m": [round(value, 4) for value in displacement],
        "yaw_change_deg": round(yaw_change, 3),
    }
