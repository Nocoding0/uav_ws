"""Bounded image-target tracking decisions for the guarded OFFBOARD mission."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class MarkerMeasurement:
    x: float
    y: float
    size: float
    stamp: float


class MarkerLock:
    """Follow only after consecutive complete cross-and-two-ring views."""

    def __init__(self, confirm_frames: int, max_age: float, max_jump: float,
                 max_size_jump: float, alpha: float) -> None:
        if confirm_frames < 1 or not all(math.isfinite(v) and v > 0 for v in
                (max_age, max_jump, max_size_jump, alpha)) or alpha > 1:
            raise ValueError("invalid marker lock limits")
        self.confirm_frames = confirm_frames
        self.max_age = max_age
        self.max_jump = max_jump
        self.max_size_jump = max_size_jump
        self.alpha = alpha
        self.measurement: Optional[MarkerMeasurement] = None
        self.last_frame_stamp = 0.0
        self.streak = 0
        self.confirmed = False

    def observe(self, detection: Optional[Tuple[float, float, float, bool]],
                stamp: float, now: float) -> None:
        if not math.isfinite(stamp) or stamp <= self.last_frame_stamp:
            return
        self.last_frame_stamp = stamp
        if stamp <= 0 or stamp > now + 0.1 or now - stamp > self.max_age:
            self.streak = 0
            self.confirmed = False
            return
        if detection is None:
            self.streak = 0
            self.confirmed = False
            return
        x, y, size, complete = detection
        if not all(math.isfinite(v) for v in (x, y, size)) or not (
                0 <= x <= 1 and 0 <= y <= 1 and 0 < size <= 1):
            self.streak = 0
            self.confirmed = False
            return
        if not complete:
            self.streak = 0
            self.confirmed = False
            return
        previous = self.measurement
        fresh = previous is not None and now - previous.stamp <= self.max_age
        if not fresh:
            self.confirmed = False
            self.streak = 0
        if fresh and (math.hypot(x - previous.x, y - previous.y) > self.max_jump
                      or abs(size - previous.size) > self.max_size_jump):
            if self.confirmed:
                return
            self.streak = 0
        if previous is None or self.streak == 0 or not fresh:
            filtered = MarkerMeasurement(x, y, size, stamp)
        else:
            a = self.alpha
            filtered = MarkerMeasurement(
                previous.x + a * (x - previous.x),
                previous.y + a * (y - previous.y),
                previous.size + a * (size - previous.size), stamp)
        self.measurement = filtered
        self.streak = min(self.confirm_frames, self.streak + 1)
        if self.streak >= self.confirm_frames:
            self.confirmed = True

    def current(self, now: float) -> Optional[MarkerMeasurement]:
        if not self.confirmed or self.measurement is None:
            return None
        if now - self.measurement.stamp > self.max_age or now < self.measurement.stamp:
            return None
        return self.measurement


def follow_setpoint(current_xy: Tuple[float, float], home_xy: Tuple[float, float],
                    yaw: float, measurement: MarkerMeasurement, *,
                    orientation: str, camera_yaw_offset: float,
                    desired_center: Tuple[float, float], desired_size: float,
                    forward_gain: float, lateral_gain: float,
                    max_speed: float, max_radius: float, dt: float) -> Tuple[float, float]:
    """Take one bounded step in map XY while keeping altitude and yaw unchanged."""
    values = (*current_xy, *home_xy, yaw, measurement.x, measurement.y,
              measurement.size, camera_yaw_offset, *desired_center, desired_size,
              forward_gain, lateral_gain, max_speed, max_radius, dt)
    if not all(math.isfinite(v) for v in values) or orientation not in ("downward", "forward"):
        raise ValueError("invalid follow input")
    if min(forward_gain, lateral_gain, max_speed, max_radius, dt) <= 0:
        raise ValueError("follow limits must be positive")
    image_right = lateral_gain * (measurement.x - desired_center[0])
    image_forward = forward_gain * (
        desired_center[1] - measurement.y if orientation == "downward"
        else desired_size - measurement.size)
    forward = math.cos(camera_yaw_offset) * image_forward - math.sin(camera_yaw_offset) * image_right
    right = math.sin(camera_yaw_offset) * image_forward + math.cos(camera_yaw_offset) * image_right
    magnitude = math.hypot(forward, right)
    if magnitude > max_speed:
        forward *= max_speed / magnitude
        right *= max_speed / magnitude
    # ROS local XY is forward/left at yaw zero; image right is body right.
    map_x = current_xy[0] + (math.cos(yaw) * forward + math.sin(yaw) * right) * dt
    map_y = current_xy[1] + (math.sin(yaw) * forward - math.cos(yaw) * right) * dt
    if math.hypot(map_x - home_xy[0], map_y - home_xy[1]) > max_radius:
        return current_xy
    return map_x, map_y
