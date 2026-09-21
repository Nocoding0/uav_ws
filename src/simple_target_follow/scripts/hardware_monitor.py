#!/usr/bin/env python3
"""Read-only MAVROS health monitor for companion-computer bring-up."""

from __future__ import annotations

import json
import math
from typing import Optional

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from simple_target_follow.mission_safety import (
    MAV_STATE_NAMES,
    battery_safety_reason,
    fcu_system_status_reason,
    telemetry_is_fresh,
)


class HardwareMonitor:
    def __init__(self) -> None:
        self.state = State()
        self.battery = BatteryState()
        self.state_received_at: Optional[rospy.Time] = None
        self.pose_received_at: Optional[rospy.Time] = None
        self.battery_received_at: Optional[rospy.Time] = None

        self.publish_rate = float(rospy.get_param("~publish_rate", 2.0))
        self.state_max_age = float(rospy.get_param("~state_max_age", 3.0))
        self.pose_max_age = float(rospy.get_param("~pose_max_age", 0.5))
        self.battery_max_age = float(rospy.get_param("~battery_max_age", 3.0))
        self.require_battery = bool(rospy.get_param("~require_battery", False))
        self.minimum_battery_voltage = float(
            rospy.get_param("~minimum_battery_voltage", 21.0)
        )
        self.minimum_battery_percentage = float(
            rospy.get_param("~minimum_battery_percentage", 0.15)
        )

        self.publisher = rospy.Publisher(
            "/target_follow/hardware_status", String, queue_size=2, latch=True
        )
        rospy.Subscriber("/mavros/state", State, self._state_callback)
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._pose_callback
        )
        rospy.Subscriber("/mavros/battery", BatteryState, self._battery_callback)

    def _state_callback(self, msg: State) -> None:
        self.state = msg
        self.state_received_at = rospy.Time.now()

    def _pose_callback(self, _msg: PoseStamped) -> None:
        self.pose_received_at = rospy.Time.now()

    def _battery_callback(self, msg: BatteryState) -> None:
        self.battery = msg
        self.battery_received_at = rospy.Time.now()

    @staticmethod
    def _seconds(value: Optional[rospy.Time]) -> Optional[float]:
        return value.to_sec() if value is not None else None

    def _status(self) -> dict:
        now = rospy.Time.now().to_sec()
        state_fresh = telemetry_is_fresh(
            self._seconds(self.state_received_at), now, self.state_max_age
        )
        pose_fresh = telemetry_is_fresh(
            self._seconds(self.pose_received_at), now, self.pose_max_age
        )
        battery_fresh = telemetry_is_fresh(
            self._seconds(self.battery_received_at), now, self.battery_max_age
        )

        voltage = float(self.battery.voltage)
        percentage = float(self.battery.percentage)
        if not battery_fresh:
            voltage = math.nan
            percentage = math.nan
        battery_reason = battery_safety_reason(
            voltage,
            percentage,
            self.require_battery,
            self.minimum_battery_voltage,
            self.minimum_battery_percentage,
        )
        reasons = []
        if not state_fresh or not self.state.connected:
            reasons.append("FCU heartbeat is missing or stale")
        else:
            system_status_reason = fcu_system_status_reason(
                self.state.system_status
            )
            if system_status_reason:
                reasons.append(system_status_reason)
        if not pose_fresh:
            reasons.append("local pose is missing or stale")
        if battery_reason:
            reasons.append(battery_reason)

        return {
            "ready": not reasons,
            "reasons": reasons,
            "connected": bool(self.state.connected and state_fresh),
            "armed": bool(self.state.armed),
            "mode": self.state.mode,
            "system_status": int(self.state.system_status),
            "system_status_name": MAV_STATE_NAMES.get(
                int(self.state.system_status), "UNKNOWN"
            ),
            "pose_fresh": pose_fresh,
            "battery_required": self.require_battery,
            "battery_fresh": battery_fresh,
            "battery_voltage": None if not math.isfinite(voltage) else round(voltage, 3),
            "battery_percentage": (
                None
                if not math.isfinite(percentage) or percentage < 0.0
                else round(percentage, 3)
            ),
        }

    def run(self) -> None:
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            status = self._status()
            self.publisher.publish(
                String(data=json.dumps(status, ensure_ascii=False))
            )
            if status["ready"]:
                rospy.loginfo_throttle(10.0, "Companion hardware status is ready")
            else:
                rospy.logwarn_throttle(
                    5.0, "Companion hardware not ready: %s" % "; ".join(status["reasons"])
                )
            rate.sleep()


def main() -> None:
    rospy.init_node("target_follow_hardware_monitor")
    HardwareMonitor().run()


if __name__ == "__main__":
    main()
