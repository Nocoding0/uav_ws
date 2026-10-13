"""Shared PX4 telemetry collection and preflight snapshot construction.

Collectors keep received messages and timestamps. They never publish flight
setpoints or request arming/mode changes. Callers load their safety limits before
initializing and subscribing, and may add sensor-specific checks.
"""

from __future__ import annotations

import json
from collections import deque
from typing import Deque, Dict, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from simple_target_follow.hover_mission import pose_window_is_stable
from simple_target_follow.mission_safety import (
    HoverPreflightSnapshot, hover_preflight_checks, telemetry_is_fresh,
)


class MissionTelemetry:
    """Common telemetry callbacks for read-only checks and flight missions."""

    def _init_telemetry(self, extra_names=()) -> None:
        self.state: Optional[State] = None
        self.pose: Optional[PoseStamped] = None
        self.estimator: Optional[EstimatorStatus] = None
        self.extended_state: Optional[ExtendedState] = None
        self.battery: Optional[BatteryState] = None
        self.received_at: Dict[str, Optional[float]] = {
            "state": None,
            "pose": None,
            "vision": None,
            "estimator": None,
            "extended_state": None,
            "battery": None,
            "bridge": None,
        }
        self.received_at.update({name: None for name in extra_names})
        self.bridge_status: Dict[str, object] = {}
        self.bridge_last_published = -1
        self.bridge_progress_at: Optional[float] = None
        self.bridge_last_rejected = -1
        self.bridge_rejection_growth = 0
        self.pose_samples: Deque[Tuple[float, float, float, float]] = deque()

    def _subscribe_telemetry(self, *, include_extended_state=False,
                             queue_sizes=None) -> None:
        sizes = {} if queue_sizes is None else queue_sizes
        topics = (
            ("state", "/mavros/state", State, self._state_callback),
            ("pose", "/mavros/local_position/pose", PoseStamped, self._pose_callback),
            ("vision", "/mavros/vision_pose/pose", PoseStamped, self._vision_callback),
            ("estimator", "/mavros/estimator_status", EstimatorStatus, self._estimator_callback),
            ("battery", "/mavros/battery", BatteryState, self._battery_callback),
            ("bridge", "/target_follow/localization_bridge_status", String, self._bridge_callback),
        )
        for name, topic, message_type, callback in topics:
            options = {"queue_size": sizes[name]} if name in sizes else {}
            rospy.Subscriber(topic, message_type, callback, **options)
        if include_extended_state:
            rospy.Subscriber("/mavros/extended_state", ExtendedState,
                             self._extended_state_callback)

    def _mark(self, name: str) -> None:
        self.received_at[name] = self._now()

    def _state_callback(self, message: State) -> None:
        self.state = message
        self._mark("state")

    @staticmethod
    def _now() -> float:
        return rospy.Time.now().to_sec()

    def _pose_callback(self, message: PoseStamped) -> None:
        now = self._now()
        self.pose = message
        self.received_at["pose"] = now
        point = message.pose.position
        self.pose_samples.append((now, point.x, point.y, point.z))
        oldest = now - self.preflight_stable_time - 0.5
        while self.pose_samples and self.pose_samples[0][0] < oldest:
            self.pose_samples.popleft()

    def _vision_callback(self, _message: PoseStamped) -> None:
        self.received_at["vision"] = self._now()

    def _estimator_callback(self, message: EstimatorStatus) -> None:
        self.estimator = message
        self.received_at["estimator"] = self._now()

    def _extended_state_callback(self, message: ExtendedState) -> None:
        self.extended_state = message
        self.received_at["extended_state"] = self._now()

    def _battery_callback(self, message: BatteryState) -> None:
        self.battery = message
        self.received_at["battery"] = self._now()

    def _bridge_callback(self, message: String) -> None:
        now = self._now()
        self.received_at["bridge"] = now
        try:
            report = json.loads(message.data)
            if not isinstance(report, dict):
                raise ValueError("bridge status is not an object")
            published = int(report.get("published_samples", -1))
            rejected = int(report.get("rejected_samples", -1))
        except (TypeError, ValueError):
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

    def _fresh(self, name: str, maximum_age: float) -> bool:
        return telemetry_is_fresh(
            self.received_at[name], self._now(), maximum_age
        )

    def _preflight_snapshot(self) -> HoverPreflightSnapshot:
        state = getattr(self, "state", None)
        pose = getattr(self, "pose", None)
        estimator = getattr(self, "estimator", None)
        battery = getattr(self, "battery", None)
        bridge_status = getattr(self, "bridge_status", {})
        point = None if pose is None else pose.pose.position
        return HoverPreflightSnapshot(
            state_present=state is not None,
            state_fresh=self._fresh("state", self.state_max_age),
            connected=bool(state and state.connected),
            armed=bool(state and state.armed),
            mode="" if state is None else state.mode,
            system_status=0 if state is None else state.system_status,
            pose_present=pose is not None,
            pose_fresh=self._fresh("pose", self.pose_max_age),
            pose_xyz=None if point is None else (point.x, point.y, point.z),
            vision_fresh=self._fresh("vision", self.pose_max_age),
            estimator_present=estimator is not None,
            estimator_fresh=self._fresh("estimator", self.estimator_max_age),
            velocity_horiz_valid=bool(
                estimator and estimator.velocity_horiz_status_flag
            ),
            position_horiz_valid=bool(
                estimator and estimator.pos_horiz_rel_status_flag
            ),
            constant_position_mode=bool(
                estimator and estimator.const_pos_mode_status_flag
            ),
            bridge_fresh=self._fresh("bridge", self.bridge_max_age),
            bridge_ready=bool(bridge_status.get("ready", False)),
            bridge_progress_fresh=telemetry_is_fresh(
                getattr(self, "bridge_progress_at", None),
                self._now(),
                self.bridge_progress_max_age,
            ),
            bridge_rejection_growth=getattr(self, "bridge_rejection_growth", 0),
            battery_present=battery is not None,
            battery_fresh=self._fresh("battery", self.battery_max_age),
            battery_voltage=None
            if battery is None
            else float(battery.voltage),
            battery_percentage=None
            if battery is None
            else float(battery.percentage),
            pose_stable=pose_window_is_stable(
                tuple(getattr(self, "pose_samples", ())),
                getattr(self, "preflight_stable_time", 3.0),
                getattr(self, "maximum_preflight_drift", 0.10),
            ),
        )

    def _preflight_checks(self) -> Dict[str, Optional[str]]:
        return hover_preflight_checks(
            self._preflight_snapshot(),
            self.minimum_battery_voltage,
            self.minimum_battery_percentage,
        )
