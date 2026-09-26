#!/usr/bin/env python3
"""One-shot, read-only readiness check for the guarded hover mission."""

from __future__ import annotations

import json
import math
import sys
from collections import deque
from typing import Deque, Dict, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, State
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import String

from simple_target_follow.hover_mission import pose_window_is_stable
from simple_target_follow.mission_safety import (
    HoverPreflightSnapshot,
    hover_preflight_checks,
    telemetry_is_fresh,
)


class HoverPreflightCheck:
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

        self.state: Optional[State] = None
        self.pose: Optional[PoseStamped] = None
        self.estimator: Optional[EstimatorStatus] = None
        self.battery: Optional[BatteryState] = None
        self.received_at: Dict[str, Optional[float]] = {
            name: None
            for name in (
                "lidar",
                "livox_imu",
                "raw_lio_pose",
                "state",
                "pose",
                "vision",
                "estimator",
                "battery",
                "bridge",
            )
        }
        self.imu_finite = False
        self.bridge_status: Dict[str, object] = {}
        self.bridge_last_published = -1
        self.bridge_progress_at: Optional[float] = None
        self.bridge_last_rejected = -1
        self.bridge_rejection_growth = 0
        self.pose_samples: Deque[Tuple[float, float, float, float]] = deque()

        rospy.Subscriber("/livox/lidar", rospy.AnyMsg, lambda _msg: self._mark("lidar"))
        rospy.Subscriber("/livox/imu", Imu, self._imu_callback, queue_size=20)
        rospy.Subscriber(
            "/fast_livo/unconverted_vision_pose",
            PoseStamped,
            lambda _msg: self._mark("raw_lio_pose"),
            queue_size=10,
        )
        rospy.Subscriber("/mavros/state", State, self._state_callback, queue_size=5)
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._pose_callback, queue_size=20
        )
        rospy.Subscriber(
            "/mavros/vision_pose/pose",
            PoseStamped,
            lambda _msg: self._mark("vision"),
            queue_size=20,
        )
        rospy.Subscriber(
            "/mavros/estimator_status",
            EstimatorStatus,
            self._estimator_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            "/mavros/battery", BatteryState, self._battery_callback, queue_size=5
        )
        rospy.Subscriber(
            "/target_follow/localization_bridge_status",
            String,
            self._bridge_callback,
            queue_size=5,
        )

    @staticmethod
    def _now() -> float:
        return rospy.Time.now().to_sec()

    def _mark(self, name: str) -> None:
        self.received_at[name] = self._now()

    def _fresh(self, name: str, maximum_age: float) -> bool:
        return telemetry_is_fresh(self.received_at[name], self._now(), maximum_age)

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

    def _state_callback(self, message: State) -> None:
        self.state = message
        self._mark("state")

    def _pose_callback(self, message: PoseStamped) -> None:
        now = self._now()
        self.pose = message
        self.received_at["pose"] = now
        point = message.pose.position
        self.pose_samples.append((now, point.x, point.y, point.z))
        oldest = now - self.preflight_stable_time - 0.5
        while self.pose_samples and self.pose_samples[0][0] < oldest:
            self.pose_samples.popleft()

    def _estimator_callback(self, message: EstimatorStatus) -> None:
        self.estimator = message
        self._mark("estimator")

    def _battery_callback(self, message: BatteryState) -> None:
        self.battery = message
        self._mark("battery")

    def _bridge_callback(self, message: String) -> None:
        now = self._now()
        self.received_at["bridge"] = now
        try:
            report = json.loads(message.data)
            if not isinstance(report, dict):
                raise ValueError("bridge status is not an object")
            published = int(report.get("published_samples", -1))
            rejected = int(report.get("rejected_samples", -1))
        except (TypeError, ValueError, json.JSONDecodeError):
            self.bridge_status = {}
            return
        self.bridge_status = report
        if published > self.bridge_last_published:
            self.bridge_progress_at = now
        self.bridge_last_published = published
        if self.bridge_last_rejected >= 0 and rejected > self.bridge_last_rejected:
            self.bridge_rejection_growth += 1
        else:
            self.bridge_rejection_growth = 0
        self.bridge_last_rejected = rejected

    def _checks(self) -> Dict[str, Optional[str]]:
        point = None if self.pose is None else self.pose.pose.position
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
        mission_checks = hover_preflight_checks(
            HoverPreflightSnapshot(
                state_present=self.state is not None,
                state_fresh=self._fresh("state", self.state_max_age),
                connected=bool(self.state and self.state.connected),
                armed=bool(self.state and self.state.armed),
                mode="" if self.state is None else self.state.mode,
                system_status=0 if self.state is None else self.state.system_status,
                pose_present=self.pose is not None,
                pose_fresh=self._fresh("pose", self.pose_max_age),
                pose_xyz=None if point is None else (point.x, point.y, point.z),
                vision_fresh=self._fresh("vision", self.pose_max_age),
                estimator_present=self.estimator is not None,
                estimator_fresh=self._fresh("estimator", self.estimator_max_age),
                velocity_horiz_valid=bool(
                    self.estimator and self.estimator.velocity_horiz_status_flag
                ),
                position_horiz_valid=bool(
                    self.estimator and self.estimator.pos_horiz_rel_status_flag
                ),
                constant_position_mode=bool(
                    self.estimator and self.estimator.const_pos_mode_status_flag
                ),
                bridge_fresh=self._fresh("bridge", self.bridge_max_age),
                bridge_ready=bool(self.bridge_status.get("ready", False)),
                bridge_progress_fresh=telemetry_is_fresh(
                    self.bridge_progress_at, self._now(), self.bridge_progress_max_age
                ),
                bridge_rejection_growth=self.bridge_rejection_growth,
                battery_present=self.battery is not None,
                battery_fresh=self._fresh("battery", self.battery_max_age),
                battery_voltage=None
                if self.battery is None
                else float(self.battery.voltage),
                battery_percentage=None
                if self.battery is None
                else float(self.battery.percentage),
                pose_stable=pose_window_is_stable(
                    tuple(self.pose_samples),
                    self.preflight_stable_time,
                    self.maximum_preflight_drift,
                ),
            ),
            self.minimum_battery_voltage,
            self.minimum_battery_percentage,
        )
        checks.update(mission_checks)
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
