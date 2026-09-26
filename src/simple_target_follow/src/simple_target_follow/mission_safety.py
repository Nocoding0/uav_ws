#!/usr/bin/env python3
"""Pure safety checks shared by the mission and unit tests."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Sequence, Tuple


MAV_STATE_NAMES = {
    0: "UNINIT",
    1: "BOOT",
    2: "CALIBRATING",
    3: "STANDBY",
    4: "ACTIVE",
    5: "CRITICAL",
    6: "EMERGENCY",
    7: "POWEROFF",
    8: "FLIGHT_TERMINATION",
}


@dataclass(frozen=True)
class HoverPreflightSnapshot:
    """ROS-independent inputs for the guarded hover mission's readiness gate."""

    state_present: bool
    state_fresh: bool
    connected: bool
    armed: bool
    mode: str
    system_status: int
    pose_present: bool
    pose_fresh: bool
    pose_xyz: Optional[Tuple[float, float, float]]
    vision_fresh: bool
    estimator_present: bool
    estimator_fresh: bool
    velocity_horiz_valid: bool
    position_horiz_valid: bool
    constant_position_mode: bool
    bridge_fresh: bool
    bridge_ready: bool
    bridge_progress_fresh: bool
    bridge_rejection_growth: int
    battery_present: bool
    battery_fresh: bool
    battery_voltage: Optional[float]
    battery_percentage: Optional[float]
    pose_stable: bool


def hover_preflight_checks(
    snapshot: HoverPreflightSnapshot,
    minimum_battery_voltage: float,
    minimum_battery_percentage: float,
) -> Dict[str, Optional[str]]:
    """Return every readiness check used by the guarded hover mission."""

    checks: Dict[str, Optional[str]] = {}
    if not snapshot.state_present or not snapshot.state_fresh:
        checks["fcu"] = "FCU state is missing or stale"
    elif not snapshot.connected:
        checks["fcu"] = "FCU is disconnected"
    elif snapshot.armed:
        checks["fcu"] = "FCU is already armed"
    elif str(snapshot.mode).upper() == "OFFBOARD":
        checks["fcu"] = "FCU is already in OFFBOARD mode"
    else:
        checks["fcu"] = fcu_system_status_reason(snapshot.system_status)

    if not snapshot.pose_present or not snapshot.pose_fresh:
        checks["px4_local_pose"] = "PX4 local pose is missing or stale"
    elif snapshot.pose_xyz is None or not all(
        math.isfinite(float(value)) for value in snapshot.pose_xyz
    ):
        checks["px4_local_pose"] = "PX4 local pose contains a non-finite position"
    else:
        checks["px4_local_pose"] = None

    checks["external_vision"] = (
        None if snapshot.vision_fresh else "external vision pose is missing or stale"
    )

    if not snapshot.estimator_present or not snapshot.estimator_fresh:
        checks["px4_estimator"] = "PX4 estimator status is missing or stale"
    elif not snapshot.velocity_horiz_valid:
        checks["px4_estimator"] = "PX4 horizontal velocity is invalid"
    elif not snapshot.position_horiz_valid:
        checks["px4_estimator"] = "PX4 horizontal position is invalid"
    elif snapshot.constant_position_mode:
        checks["px4_estimator"] = "PX4 estimator is in constant-position mode"
    else:
        checks["px4_estimator"] = None

    if not snapshot.bridge_fresh:
        checks["localization_bridge"] = "localization bridge status is missing or stale"
    elif not snapshot.bridge_ready:
        checks["localization_bridge"] = "localization bridge is not ready"
    elif not snapshot.bridge_progress_fresh:
        checks["localization_bridge"] = "localization bridge output is not progressing"
    elif snapshot.bridge_rejection_growth >= 2:
        checks["localization_bridge"] = (
            "localization bridge rejection count is continuously growing"
        )
    else:
        checks["localization_bridge"] = None

    if not snapshot.battery_present or not snapshot.battery_fresh:
        checks["flight_battery"] = "flight battery telemetry is missing or stale"
    else:
        checks["flight_battery"] = battery_safety_reason(
            snapshot.battery_voltage,
            snapshot.battery_percentage,
            True,
            minimum_battery_voltage,
            minimum_battery_percentage,
        )

    checks["stationary_pose"] = (
        None
        if snapshot.pose_stable
        else "local pose has not remained stable for the required window"
    )
    return checks


def first_failed_check(checks: Mapping[str, Optional[str]]) -> Optional[str]:
    """Return the first failure from an ordered readiness-check mapping."""

    return next((reason for reason in checks.values() if reason), None)


def fcu_system_status_reason(system_status: int) -> Optional[str]:
    """Reject FCU lifecycle states that are not ready for normal operation."""

    status = int(system_status)
    if status in (3, 4):
        return None
    name = MAV_STATE_NAMES.get(status, "UNKNOWN")
    return "FCU system status is {} ({})".format(name, status)


def telemetry_is_fresh(
    received_at_s: Optional[float], now_s: float, max_age_s: float
) -> bool:
    """Return true only for a finite, recent local receipt timestamp."""

    if received_at_s is None:
        return False
    if not all(math.isfinite(value) for value in (received_at_s, now_s, max_age_s)):
        return False
    return max_age_s > 0.0 and 0.0 <= now_s - received_at_s <= max_age_s


def flight_telemetry_reason(
    state_received_at_s: Optional[float],
    pose_received_at_s: Optional[float],
    connected: bool,
    now_s: float,
    state_max_age_s: float,
    pose_max_age_s: float,
) -> Optional[str]:
    """Describe an unavailable FCU heartbeat or local-position stream.

    MAVROS publishes ``/mavros/state`` from the approximately 1 Hz FCU
    heartbeat, while local position normally arrives much faster.  Keeping
    separate limits avoids rejecting a healthy heartbeat at its normal period
    without weakening the local-position freshness gate used for flight.
    """

    if state_received_at_s is None:
        return "MAVROS state has not been received"
    if not connected:
        return "MAVROS reports that the FCU is disconnected"
    if not telemetry_is_fresh(state_received_at_s, now_s, state_max_age_s):
        state_age = now_s - state_received_at_s
        if not math.isfinite(state_age):
            return "MAVROS state timestamp is invalid"
        if state_age < 0.0:
            return "MAVROS state timestamp is {:.2f} s in the future".format(
                -state_age
            )
        if state_age > state_max_age_s:
            return "MAVROS state is stale ({:.2f} s > {:.2f} s)".format(
                state_age, state_max_age_s
            )
        return "MAVROS state freshness limit is invalid"

    if pose_received_at_s is None:
        return "MAVROS local position has not been received"
    if not telemetry_is_fresh(pose_received_at_s, now_s, pose_max_age_s):
        pose_age = now_s - pose_received_at_s
        if not math.isfinite(pose_age):
            return "MAVROS local-position timestamp is invalid"
        if pose_age < 0.0:
            return "MAVROS local-position timestamp is {:.2f} s in the future".format(
                -pose_age
            )
        if pose_age > pose_max_age_s:
            return "MAVROS local position is stale ({:.2f} s > {:.2f} s)".format(
                pose_age, pose_max_age_s
            )
        return "MAVROS local-position freshness limit is invalid"
    return None


def health_status_reason(
    status: Optional[Mapping[str, object]],
    received_at_s: Optional[float],
    now_s: float,
    max_age_s: float,
    label: str,
) -> Optional[str]:
    """Describe a missing, stale, malformed, or negative JSON health status."""

    if not telemetry_is_fresh(received_at_s, now_s, max_age_s):
        return "{} status is missing or stale".format(label)
    if not isinstance(status, Mapping) or not isinstance(status.get("ready"), bool):
        return "{} status is malformed".format(label)
    if status["ready"]:
        return None
    reasons = status.get("reasons")
    if isinstance(reasons, list):
        details = "; ".join(str(reason) for reason in reasons if str(reason).strip())
        if details:
            return "{} is not ready: {}".format(label, details)
    return "{} is not ready".format(label)


def battery_safety_reason(
    voltage: Optional[float],
    percentage: Optional[float],
    require_battery: bool,
    minimum_voltage: float,
    minimum_percentage: float,
) -> Optional[str]:
    """Describe an unsafe flight battery reading, or return ``None``.

    MAVROS uses a negative percentage when the FCU does not know state of
    charge. Such a reading is ignored, while a missing/zero voltage is always
    rejected when a flight battery is required.
    """

    if not require_battery:
        return None
    if voltage is None or not math.isfinite(voltage) or voltage <= 1.0:
        return "flight battery is not detected"
    if voltage < minimum_voltage:
        return "battery voltage {:.2f} V is below {:.2f} V".format(
            voltage, minimum_voltage
        )
    if (
        percentage is not None
        and math.isfinite(percentage)
        and 0.0 <= percentage <= 1.0
        and percentage < minimum_percentage
    ):
        return "battery remaining {:.0f}% is below {:.0f}%".format(
            percentage * 100.0, minimum_percentage * 100.0
        )
    return None


def relative_geofence_reason(
    position: Sequence[float],
    home: Sequence[float],
    bounds: Tuple[float, float, float, float, float, float],
) -> Optional[str]:
    """Describe a relative XYZ geofence violation, or return ``None``."""

    values = tuple(position) + tuple(home) + tuple(bounds)
    if len(position) != 3 or len(home) != 3 or len(bounds) != 6:
        return "invalid geofence input"
    if not all(math.isfinite(float(value)) for value in values):
        return "non-finite local position"

    dx = float(position[0]) - float(home[0])
    dy = float(position[1]) - float(home[1])
    dz = float(position[2]) - float(home[2])
    minimum_x, maximum_x, minimum_y, maximum_y, minimum_z, maximum_z = bounds
    if not minimum_x <= dx <= maximum_x:
        return "relative X {:.2f} m is outside [{:.2f}, {:.2f}]".format(
            dx, minimum_x, maximum_x
        )
    if not minimum_y <= dy <= maximum_y:
        return "relative Y {:.2f} m is outside [{:.2f}, {:.2f}]".format(
            dy, minimum_y, maximum_y
        )
    if not minimum_z <= dz <= maximum_z:
        return "relative Z {:.2f} m is outside [{:.2f}, {:.2f}]".format(
            dz, minimum_z, maximum_z
        )
    return None
