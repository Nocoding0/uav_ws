#!/usr/bin/env python3
"""One-shot, read-only readiness check for the guarded hover mission."""

from __future__ import annotations

import json
import math
import sys
from typing import Dict, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

from simple_target_follow.mission_telemetry import MissionTelemetry


class HoverPreflightCheck(MissionTelemetry):
    """Collect live evidence and exit with a machine-usable result."""

    LABELS = {
        "lidar": "Mid-360 点云",
        "livox_imu": "Mid-360 IMU",
        "raw_lio_pose": "FAST-LIVO 原始位姿",
        "fcu": "MAVROS / 飞控",
        "external_vision": "送入 PX4 的外部位姿",
        "localization_bridge": "LIO 位姿桥接",
        "px4_local_pose": "PX4 本地位姿",
        "px4_estimator": "PX4 估计器",
        "flight_battery": "飞行电池",
        "stationary_pose": "起飞点静止稳定性",
    }

    def __init__(self) -> None:
        self.timeout = float(rospy.get_param("~timeout", 12.0))
        self.sensor_max_age = float(rospy.get_param("~sensor_max_age", 0.5))
        self.pose_max_age = float(rospy.get_param("~pose_max_age", 0.5))
        self.state_max_age = float(rospy.get_param("~state_max_age", 3.0))
        self.estimator_max_age = float(rospy.get_param("~estimator_max_age", 0.5))
        self.bridge_max_age = float(rospy.get_param("~bridge_max_age", 1.0))
        self.bridge_progress_max_age = float(
            rospy.get_param("~bridge_progress_max_age", 1.0)
        )
        self.battery_max_age = float(rospy.get_param("~battery_max_age", 3.0))
        self.preflight_stable_time = float(
            rospy.get_param("~preflight_stable_time", 3.0)
        )
        self.maximum_preflight_drift = float(
            rospy.get_param("~maximum_preflight_drift", 0.10)
        )
        self.minimum_battery_voltage = float(
            rospy.get_param("~minimum_battery_voltage", 21.0)
        )
        self.minimum_battery_percentage = float(
            rospy.get_param("~minimum_battery_percentage", 0.15)
        )
        self.report_path = str(
            rospy.get_param("~report_path", "/tmp/uav_hover_preflight.json")
        )
        for name in (
            "timeout",
            "sensor_max_age",
            "pose_max_age",
            "state_max_age",
            "estimator_max_age",
            "bridge_max_age",
            "bridge_progress_max_age",
            "battery_max_age",
            "preflight_stable_time",
            "maximum_preflight_drift",
            "minimum_battery_voltage",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))

        self._init_telemetry(extra_names=("lidar", "livox_imu", "raw_lio_pose"))
        self.imu_finite = False

        rospy.Subscriber("/livox/lidar", rospy.AnyMsg, lambda _msg: self._mark("lidar"))
        rospy.Subscriber("/livox/imu", Imu, self._imu_callback, queue_size=20)
        rospy.Subscriber(
            "/fast_livo/unconverted_vision_pose",
            PoseStamped,
            lambda _msg: self._mark("raw_lio_pose"),
            queue_size=10,
        )
        self._subscribe_telemetry(queue_sizes={
            "state": 5, "pose": 20, "vision": 20,
            "estimator": 10, "battery": 5, "bridge": 5,
        })

    def _imu_callback(self, message: Imu) -> None:
        values = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        self.imu_finite = all(math.isfinite(value) for value in values)
        self._mark("livox_imu")

    def _checks(self) -> Dict[str, Optional[str]]:
        checks: Dict[str, Optional[str]] = {
            "lidar": None
            if self._fresh("lidar", self.sensor_max_age)
            else "Mid-360 point cloud is missing or stale",
            "livox_imu": None
            if self._fresh("livox_imu", self.sensor_max_age) and self.imu_finite
            else (
                "Mid-360 IMU contains non-finite acceleration or angular rate"
                if self._fresh("livox_imu", self.sensor_max_age)
                else "Mid-360 IMU is missing or stale"
            ),
            "raw_lio_pose": None
            if self._fresh("raw_lio_pose", self.pose_max_age)
            else "FAST-LIVO raw pose is missing or stale",
        }
        checks.update(self._preflight_checks())
        return checks

    def _report(self, checks: Dict[str, Optional[str]]) -> dict:
        failures = [reason for reason in checks.values() if reason]
        return {
            "ready_for_hover_script": not failures,
            "checks": {
                name: {"passed": reason is None, "reason": reason}
                for name, reason in checks.items()
            },
            "fcu_mode": None if self.state is None else self.state.mode,
            "fcu_armed": None if self.state is None else bool(self.state.armed),
            "battery_voltage": None
            if self.battery is None or not math.isfinite(float(self.battery.voltage))
            else round(float(self.battery.voltage), 3),
            "note": "Read-only software gate; RC mapping and physical safety remain operator checks.",
        }

    def run(self) -> bool:
        rospy.loginfo(
            "Running read-only hover preflight check for up to %.1f s", self.timeout
        )
        deadline = self._now() + self.timeout
        checks = self._checks()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and self._now() < deadline:
            checks = self._checks()
            if all(reason is None for reason in checks.values()):
                break
            rate.sleep()

        report = self._report(checks)
        encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
        try:
            with open(self.report_path, "w", encoding="utf-8") as stream:
                stream.write(encoded + "\n")
        except OSError as exc:
            rospy.logerr("Cannot write preflight report %s: %s", self.report_path, exc)

        print("\n自动悬停起飞前自检")
        for name, reason in checks.items():
            label = self.LABELS.get(name, name)
            if reason is None:
                print("[通过] {}".format(label))
            else:
                print("[失败] {}：{}".format(label, reason))
        if report["ready_for_hover_script"]:
            print("\n结论：软件条件通过，可以启动悬停节点；仍需飞手完成现场安全确认。")
        else:
            print("\n结论：禁止启动自动起飞。请先处理以上失败项。")
        print("报告：{}".format(self.report_path))
        return bool(report["ready_for_hover_script"])


def main() -> None:
    try:
        rospy.init_node("hover_preflight_check")
    except Exception as exc:  # rospy can expose the underlying XML-RPC exception.
        report = {
            "ready_for_hover_script": False,
            "checks": {
                "ros_master": {
                    "passed": False,
                    "reason": "ROS master is unavailable: {}".format(exc),
                }
            },
        }
        try:
            with open("/tmp/uav_hover_preflight.json", "w", encoding="utf-8") as stream:
                stream.write(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        except OSError:
            pass
        print("[失败] ROS master：无法连接 ROS master", file=sys.stderr)
        print("结论：禁止启动自动起飞。请先启动完整定位链。", file=sys.stderr)
        raise SystemExit(2)
    try:
        ready = HoverPreflightCheck().run()
    except ValueError as exc:
        rospy.logfatal("Invalid hover preflight configuration: %s", exc)
        raise SystemExit(2)
    raise SystemExit(0 if ready else 2)


if __name__ == "__main__":
    main()
