"""Safety checks for forwarding an ENU localization pose to MAVROS."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class BridgeDecision:
    accepted: bool
    ready: bool
    reason: str
    orientation: Optional[Quaternion] = None
    age_s: Optional[float] = None
    step_m: Optional[float] = None
    angular_step_deg: Optional[float] = None


class PoseBridgeGate:
    """Reject malformed, stale, discontinuous, or uninitialized poses.

    A rejected continuity sample deliberately does not move the stored
    reference. This keeps the output closed after estimator relocalization
    until localization is explicitly restarted and revalidated.
    """

    def __init__(
        self,
        warmup_samples: int = 10,
        maximum_age_s: float = 0.25,
        future_tolerance_s: float = 0.10,
        maximum_position_m: float = 100.0,
        base_step_m: float = 0.25,
        maximum_speed_mps: float = 3.0,
        base_angular_step_deg: float = 20.0,
        maximum_angular_rate_dps: float = 360.0,
    ) -> None:
        self.warmup_samples = max(1, int(warmup_samples))
        self.maximum_age_s = float(maximum_age_s)
        self.future_tolerance_s = float(future_tolerance_s)
        self.maximum_position_m = float(maximum_position_m)
        self.base_step_m = float(base_step_m)
        self.maximum_speed_mps = float(maximum_speed_mps)
        self.base_angular_step_deg = float(base_angular_step_deg)
        self.maximum_angular_rate_dps = float(maximum_angular_rate_dps)
        self.accepted_samples = 0
        self._last_position: Optional[Vector3] = None
        self._last_orientation: Optional[Quaternion] = None
        self._last_stamp_s: Optional[float] = None

    @staticmethod
    def _finite(values: Sequence[float]) -> bool:
        return all(math.isfinite(float(value)) for value in values)

    def evaluate(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        stamp_s: float,
        now_s: float,
    ) -> BridgeDecision:
        if len(position) != 3 or len(orientation) != 4:
            return BridgeDecision(False, False, "invalid vector dimensions")
        if not self._finite((*position, *orientation, stamp_s, now_s)):
            return BridgeDecision(False, False, "pose contains a non-finite value")

        position_tuple = (
            float(position[0]),
            float(position[1]),
            float(position[2]),
        )
        if max(abs(value) for value in position_tuple) > self.maximum_position_m:
            return BridgeDecision(False, False, "position exceeds safety bound")

        quaternion_norm = math.sqrt(
            sum(float(value) * float(value) for value in orientation)
        )
        if quaternion_norm < 0.5 or quaternion_norm > 1.5:
            return BridgeDecision(False, False, "quaternion norm is invalid")
        normalized = (
            float(orientation[0]) / quaternion_norm,
            float(orientation[1]) / quaternion_norm,
            float(orientation[2]) / quaternion_norm,
            float(orientation[3]) / quaternion_norm,
        )

        age_s = float(now_s) - float(stamp_s)
        if age_s > self.maximum_age_s:
            return BridgeDecision(False, False, "pose timestamp is stale", age_s=age_s)
        if age_s < -self.future_tolerance_s:
            return BridgeDecision(
                False, False, "pose timestamp is in the future", age_s=age_s
            )

        step_m = 0.0
        angular_step_deg = 0.0
        if self._last_stamp_s is not None:
            delta_time_s = float(stamp_s) - self._last_stamp_s
            if delta_time_s <= 0.0:
                return BridgeDecision(
                    False, False, "pose timestamp did not increase", age_s=age_s
                )
            assert self._last_position is not None
            assert self._last_orientation is not None
            step_m = math.sqrt(
                sum(
                    (position_tuple[index] - self._last_position[index]) ** 2
                    for index in range(3)
                )
            )
            maximum_step_m = (
                self.base_step_m + self.maximum_speed_mps * delta_time_s
            )
            if step_m > maximum_step_m:
                return BridgeDecision(
                    False,
                    False,
                    "position discontinuity exceeds safety bound",
                    age_s=age_s,
                    step_m=step_m,
                )

            quaternion_dot = abs(
                sum(
                    normalized[index] * self._last_orientation[index]
                    for index in range(4)
                )
            )
            quaternion_dot = min(1.0, max(0.0, quaternion_dot))
            angular_step_deg = math.degrees(2.0 * math.acos(quaternion_dot))
            maximum_angular_step_deg = (
                self.base_angular_step_deg
                + self.maximum_angular_rate_dps * delta_time_s
            )
            if angular_step_deg > maximum_angular_step_deg:
                return BridgeDecision(
                    False,
                    False,
                    "orientation discontinuity exceeds safety bound",
                    age_s=age_s,
                    step_m=step_m,
                    angular_step_deg=angular_step_deg,
                )

        self._last_position = position_tuple
        self._last_orientation = normalized
        self._last_stamp_s = float(stamp_s)
        self.accepted_samples += 1
        ready = self.accepted_samples >= self.warmup_samples
        return BridgeDecision(
            True,
            ready,
            "ready" if ready else "warming up",
            orientation=normalized,
            age_s=age_s,
            step_m=step_m,
            angular_step_deg=angular_step_deg,
        )
