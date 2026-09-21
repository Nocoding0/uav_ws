#!/usr/bin/env python3
"""Read-only freshness monitor for the companion-computer sensor stack."""

from __future__ import annotations

import json
import math
from typing import Dict, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

from simple_target_follow.mission_safety import telemetry_is_fresh


class SensorMonitor:
    def __init__(self) -> None:
        self.max_age = float(rospy.get_param("~max_age", 1.0))
        self.required = {
            "color": bool(rospy.get_param("~require_color", True)),
            "depth": bool(rospy.get_param("~require_depth", True)),
            "xsens_imu": bool(rospy.get_param("~require_xsens_imu", True)),
            "lidar": bool(rospy.get_param("~require_lidar", False)),
            "localization": bool(rospy.get_param("~require_localization", False)),
        }
        self.received_at: Dict[str, Optional[rospy.Time]] = {
            name: None for name in self.required
        }
        self.imu_finite = False

        rospy.Subscriber(
            rospy.get_param("~color_topic", "/camera/color/image_raw"),
            Image,
            lambda _msg: self._mark("color"),
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
            ),
            Image,
            lambda _msg: self._mark("depth"),
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~xsens_imu_topic", "/imu/data"),
            Imu,
            self._imu_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~lidar_topic", "/livox/lidar"),
            rospy.AnyMsg,
            lambda _msg: self._mark("lidar"),
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~localization_topic", "/mavros/vision_pose/pose"),
            PoseStamped,
            lambda _msg: self._mark("localization"),
            queue_size=2,
        )
        self.publisher = rospy.Publisher(
            "/target_follow/sensor_status", String, queue_size=2, latch=True
        )

    def _mark(self, name: str) -> None:
        self.received_at[name] = rospy.Time.now()

    def _imu_callback(self, msg: Imu) -> None:
        values = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        self.imu_finite = all(math.isfinite(value) for value in values)
        self._mark("xsens_imu")

    def _status(self) -> dict:
        now = rospy.Time.now().to_sec()
        fresh = {
            name: telemetry_is_fresh(
                timestamp.to_sec() if timestamp is not None else None,
                now,
                self.max_age,
            )
            for name, timestamp in self.received_at.items()
        }
        reasons = [
            "{} stream is missing or stale".format(name)
            for name, is_required in self.required.items()
            if is_required and not fresh[name]
        ]
        if self.required["xsens_imu"] and fresh["xsens_imu"] and not self.imu_finite:
            reasons.append("xsens_imu contains non-finite acceleration or angular rate")
        return {
            "ready": not reasons,
            "reasons": reasons,
            "required": self.required,
            "fresh": fresh,
            "xsens_imu_finite": self.imu_finite,
        }

    def run(self) -> None:
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            status = self._status()
            self.publisher.publish(String(data=json.dumps(status)))
            if status["ready"]:
                rospy.loginfo_throttle(10.0, "Companion sensor status is ready")
            else:
                rospy.logwarn_throttle(
                    5.0, "Companion sensors not ready: %s" % "; ".join(status["reasons"])
                )
            rate.sleep()


def main() -> None:
    rospy.init_node("target_follow_sensor_monitor")
    SensorMonitor().run()


if __name__ == "__main__":
    main()
