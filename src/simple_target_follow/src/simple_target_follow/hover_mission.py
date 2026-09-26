#!/usr/bin/env python3
"""Pure helpers for the small OFFBOARD hover acceptance mission."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


class MissionPhase:
    WAITING = "WAITING"
    READY = "READY"
    PRIMING = "PRIMING"
    ENTERING_OFFBOARD = "ENTERING_OFFBOARD"
    TAKEOFF = "TAKEOFF"
    HOVER = "HOVER"
    LANDING = "LANDING"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"
    PILOT_TAKEOVER = "PILOT_TAKEOVER"


ACTIVE_PHASES = {
    MissionPhase.ENTERING_OFFBOARD,
    MissionPhase.TAKEOFF,
    MissionPhase.HOVER,
}


@dataclass(frozen=True)
class HoverMissionLimits:
    takeoff_height: float = 1.2
    hover_duration: float = 40.0
    takeoff_speed: float = 0.25
    setpoint_rate: float = 20.0
    position_tolerance: float = 0.12
    settle_time: float = 1.5
    disturbance_threshold: float = 0.15
    recovery_threshold: float = 0.12
    maximum_horizontal_error: float = 0.8
    maximum_hover_vertical_error: float = 0.4

    def validate(self) -> None:
        positive = {
            "takeoff_height": self.takeoff_height,
            "hover_duration": self.hover_duration,
            "takeoff_speed": self.takeoff_speed,
            "setpoint_rate": self.setpoint_rate,
            "position_tolerance": self.position_tolerance,
            "settle_time": self.settle_time,
            "disturbance_threshold": self.disturbance_threshold,
            "recovery_threshold": self.recovery_threshold,
            "maximum_horizontal_error": self.maximum_horizontal_error,
            "maximum_hover_vertical_error": self.maximum_hover_vertical_error,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        if self.setpoint_rate < 10.0:
            raise ValueError("setpoint_rate must be at least 10 Hz")
        if self.recovery_threshold > self.disturbance_threshold:
            raise ValueError(
                "recovery_threshold cannot exceed disturbance_threshold"
            )
        if self.disturbance_threshold >= self.maximum_horizontal_error:
            raise ValueError(
                "disturbance_threshold must be below maximum_horizontal_error"
            )


def ramped_altitude(
    start_z: float, target_z: float, speed_mps: float, elapsed_s: float
) -> float:
    """Return a rate-limited vertical position command."""

    values = (start_z, target_z, speed_mps, elapsed_s)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("altitude ramp values must be finite")
    if speed_mps <= 0.0 or elapsed_s < 0.0 or target_z < start_z:
        raise ValueError("invalid upward altitude ramp")
    return min(target_z, start_z + speed_mps * elapsed_s)


def position_errors(
    current: Sequence[float], target: Sequence[float]
) -> Tuple[float, float, float]:
    """Return horizontal, signed vertical, and 3-D position error."""

    if len(current) != 3 or len(target) != 3:
        raise ValueError("positions must contain XYZ")
    values = tuple(float(value) for value in tuple(current) + tuple(target))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("positions must be finite")
    dx = values[0] - values[3]
    dy = values[1] - values[4]
    dz = values[2] - values[5]
    horizontal = math.hypot(dx, dy)
    return horizontal, dz, math.sqrt(horizontal * horizontal + dz * dz)


def pose_window_is_stable(
    samples: Sequence[Tuple[float, float, float, float]],
    window_s: float,
    maximum_drift_m: float,
) -> bool:
    """Require a full time window whose XYZ span stays inside a drift bound."""

    if len(samples) < 2 or window_s <= 0.0 or maximum_drift_m <= 0.0:
        return False
    newest = samples[-1]
    if not all(math.isfinite(float(value)) for sample in samples for value in sample):
        return False
    if newest[0] - samples[0][0] < window_s:
        return False
    xs = [sample[1] for sample in samples]
    ys = [sample[2] for sample in samples]
    zs = [sample[3] for sample in samples]
    span = math.sqrt(
        (max(xs) - min(xs)) ** 2
        + (max(ys) - min(ys)) ** 2
        + (max(zs) - min(zs)) ** 2
    )
    return span <= maximum_drift_m


def pilot_took_over(owned_offboard: bool, phase: str, current_mode: str) -> bool:
    """A mode change after acquiring OFFBOARD always belongs to the pilot/PX4."""

    return bool(
        owned_offboard
        and phase in ACTIVE_PHASES
        and str(current_mode).upper() != "OFFBOARD"
    )


class DisturbanceTracker:
    """Track horizontal disturbance events and their recovery times."""

    def __init__(
        self,
        disturbance_threshold: float,
        recovery_threshold: float,
        recovery_settle_s: float,
    ) -> None:
        if not (
            0.0 < recovery_threshold <= disturbance_threshold
            and recovery_settle_s > 0.0
        ):
            raise ValueError("invalid disturbance thresholds")
        self.disturbance_threshold = disturbance_threshold
        self.recovery_threshold = recovery_threshold
        self.recovery_settle_s = recovery_settle_s
        self.count = 0
        self.active_since: Optional[float] = None
        self.recovery_started: Optional[float] = None
        self.last_recovery_s: Optional[float] = None
        self.maximum_error_m = 0.0

    def update(self, horizontal_error_m: float, now_s: float) -> Optional[str]:
        if not math.isfinite(horizontal_error_m) or horizontal_error_m < 0.0:
            raise ValueError("horizontal error must be finite and non-negative")
        if not math.isfinite(now_s):
            raise ValueError("time must be finite")
        self.maximum_error_m = max(self.maximum_error_m, horizontal_error_m)

        if self.active_since is None:
            if horizontal_error_m > self.disturbance_threshold:
                self.count += 1
                self.active_since = now_s
                self.recovery_started = None
                return "disturbance"
            return None

        if horizontal_error_m <= self.recovery_threshold:
            if self.recovery_started is None:
                self.recovery_started = now_s
            elif now_s - self.recovery_started >= self.recovery_settle_s:
                self.last_recovery_s = now_s - self.active_since
                self.active_since = None
                self.recovery_started = None
                return "recovered"
        else:
            self.recovery_started = None
        return None
