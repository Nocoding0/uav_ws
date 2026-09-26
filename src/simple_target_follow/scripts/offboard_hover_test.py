#!/usr/bin/env python3
"""Small guarded PX4 OFFBOARD takeoff, hover, disturbance, and land test."""

from __future__ import annotations

import copy
import json
import math
from collections import deque
from typing import Deque, Dict, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from simple_target_follow.hover_mission import (
    DisturbanceTracker,
    HoverMissionLimits,
    MissionPhase,
    pilot_took_over,
    pose_window_is_stable,
    position_errors,
    ramped_altitude,
)
from simple_target_follow.mission_safety import (
    HoverPreflightSnapshot,
    battery_safety_reason,
    first_failed_check,
    fcu_system_status_reason,
    hover_preflight_checks,
    telemetry_is_fresh,
)


class MissionAbort(RuntimeError):
    pass


class PilotTakeover(RuntimeError):
    pass


class OffboardHoverTest:
    def __init__(self) -> None:
        self.allow_arming = bool(rospy.get_param("~allow_arming", False))
        self.limits = HoverMissionLimits(
            takeoff_height=float(rospy.get_param("~takeoff_height", 1.2)),
            hover_duration=float(rospy.get_param("~hover_duration", 40.0)),
            takeoff_speed=float(rospy.get_param("~takeoff_speed", 0.25)),
            setpoint_rate=float(rospy.get_param("~setpoint_rate", 20.0)),
            position_tolerance=float(
                rospy.get_param("~position_tolerance", 0.12)
            ),
            settle_time=float(rospy.get_param("~settle_time", 1.5)),
            disturbance_threshold=float(
                rospy.get_param("~disturbance_threshold", 0.15)
            ),
            recovery_threshold=float(
                rospy.get_param("~recovery_threshold", 0.12)
            ),
            maximum_horizontal_error=float(
                rospy.get_param("~maximum_horizontal_error", 0.8)
            ),
            maximum_hover_vertical_error=float(
                rospy.get_param("~maximum_hover_vertical_error", 0.4)
            ),
        )
        self.limits.validate()
        self.preflight_stable_time = float(
            rospy.get_param("~preflight_stable_time", 3.0)
        )
        self.maximum_preflight_drift = float(
            rospy.get_param("~maximum_preflight_drift", 0.10)
        )
        self.pose_max_age = float(rospy.get_param("~pose_max_age", 0.5))
        self.state_max_age = float(rospy.get_param("~state_max_age", 3.0))
        self.estimator_max_age = float(
            rospy.get_param("~estimator_max_age", 0.5)
        )
        self.bridge_max_age = float(rospy.get_param("~bridge_max_age", 1.0))
        self.bridge_progress_max_age = float(
            rospy.get_param("~bridge_progress_max_age", 1.0)
        )
        self.battery_max_age = float(rospy.get_param("~battery_max_age", 3.0))
        self.minimum_battery_voltage = float(
            rospy.get_param("~minimum_battery_voltage", 21.0)
        )
        self.minimum_battery_percentage = float(
            rospy.get_param("~minimum_battery_percentage", 0.15)
        )
        self.priming_duration = float(rospy.get_param("~priming_duration", 2.0))
        self.command_timeout = float(rospy.get_param("~command_timeout", 20.0))
        self.landing_timeout = float(rospy.get_param("~landing_timeout", 90.0))
        self.landing_mode_timeout = float(
            rospy.get_param("~landing_mode_timeout", 5.0)
        )
        for name in (
            "preflight_stable_time",
            "maximum_preflight_drift",
            "pose_max_age",
            "state_max_age",
            "estimator_max_age",
            "bridge_max_age",
            "bridge_progress_max_age",
            "battery_max_age",
            "minimum_battery_voltage",
            "priming_duration",
            "command_timeout",
            "landing_timeout",
            "landing_mode_timeout",
        ):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))

        self.phase = MissionPhase.WAITING
        self.detail = "waiting for healthy localization and FCU"
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
        self.bridge_status: Dict[str, object] = {}
        self.bridge_last_published = -1
        self.bridge_progress_at: Optional[float] = None
        self.bridge_last_rejected = -1
        self.bridge_rejection_growth = 0
        self.pose_samples: Deque[Tuple[float, float, float, float]] = deque()
        self.target: Optional[PoseStamped] = None
        self.start_requested = False
        self.abort_requested = False
        self.owned_offboard = False
        self.hover_deadline: Optional[float] = None
        self.disturbance = DisturbanceTracker(
            self.limits.disturbance_threshold,
            self.limits.recovery_threshold,
            self.limits.settle_time,
        )

        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            "/target_follow/hover_test/status", String, queue_size=2, latch=True
        )
        rospy.Subscriber("/mavros/state", State, self._state_callback)
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._pose_callback
        )
        rospy.Subscriber(
            "/mavros/vision_pose/pose", PoseStamped, self._vision_callback
        )
        rospy.Subscriber(
            "/mavros/estimator_status", EstimatorStatus, self._estimator_callback
        )
        rospy.Subscriber(
            "/mavros/extended_state", ExtendedState, self._extended_state_callback
        )
        rospy.Subscriber("/mavros/battery", BatteryState, self._battery_callback)
        rospy.Subscriber(
            "/target_follow/localization_bridge_status",
            String,
            self._bridge_callback,
        )
        self.start_service = rospy.Service(
            "/target_follow/hover_test/start", Trigger, self._start_callback
        )
        self.abort_service = rospy.Service(
            "/target_follow/hover_test/abort", Trigger, self._abort_callback
        )
        self.status_timer = rospy.Timer(rospy.Duration(0.2), self._status_timer)
        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.rate = rospy.Rate(self.limits.setpoint_rate)

    @staticmethod
    def _now() -> float:
        return rospy.Time.now().to_sec()

    def _state_callback(self, message: State) -> None:
        self.state = message
        self.received_at["state"] = self._now()
        if (
            self.phase == MissionPhase.ENTERING_OFFBOARD
            and message.mode == "OFFBOARD"
        ):
            self.owned_offboard = True

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

    def _start_callback(self, _request) -> TriggerResponse:
        if self.phase != MissionPhase.READY:
            return TriggerResponse(False, "not ready: {}".format(self.detail))
        if not self.allow_arming:
            return TriggerResponse(False, "arming is blocked by allow_arming=false")
        if self.start_requested:
            return TriggerResponse(False, "mission start was already requested")
        self.start_requested = True
        return TriggerResponse(True, "start accepted; entering guarded OFFBOARD mission")

    def _abort_callback(self, _request) -> TriggerResponse:
        if self.phase not in (
            MissionPhase.PRIMING,
            MissionPhase.ENTERING_OFFBOARD,
            MissionPhase.TAKEOFF,
            MissionPhase.HOVER,
        ):
            return TriggerResponse(False, "mission is not controlling the vehicle")
        self.abort_requested = True
        return TriggerResponse(True, "abort accepted")

    def _fresh(self, name: str, maximum_age: float) -> bool:
        return telemetry_is_fresh(
            self.received_at[name], self._now(), maximum_age
        )

    def _preflight_reason(self) -> Optional[str]:
        state = getattr(self, "state", None)
        pose = getattr(self, "pose", None)
        estimator = getattr(self, "estimator", None)
        battery = getattr(self, "battery", None)
        bridge_status = getattr(self, "bridge_status", {})
        point = None if pose is None else pose.pose.position
        checks = hover_preflight_checks(
            HoverPreflightSnapshot(
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
            ),
            self.minimum_battery_voltage,
            self.minimum_battery_percentage,
        )
        reason = first_failed_check(checks)
        if reason == "local pose has not remained stable for the required window":
            return "local pose has not remained stable for {:.1f} s".format(
                self.preflight_stable_time
            )
        return reason

    def _flight_reason(self) -> Optional[str]:
        if self.state is None or not self._fresh("state", self.state_max_age):
            return "FCU state is missing or stale"
        if not self.state.connected:
            return "FCU disconnected"
        lifecycle = fcu_system_status_reason(self.state.system_status)
        if lifecycle:
            return lifecycle
        if self.pose is None or not self._fresh("pose", self.pose_max_age):
            return "PX4 local pose is missing or stale"
        point = self.pose.pose.position
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            return "PX4 local pose contains a non-finite position"
        if not self._fresh("vision", self.pose_max_age):
            return "external vision pose is missing or stale"
        if self.estimator is None or not self._fresh(
            "estimator", self.estimator_max_age
        ):
            return "PX4 estimator status is missing or stale"
        if (
            not self.estimator.velocity_horiz_status_flag
            or not self.estimator.pos_horiz_rel_status_flag
            or self.estimator.const_pos_mode_status_flag
        ):
            return "PX4 horizontal estimator became invalid"
        if not self._fresh("bridge", self.bridge_max_age) or not telemetry_is_fresh(
            self.bridge_progress_at, self._now(), self.bridge_progress_max_age
        ):
            return "localization bridge stopped"
        if not bool(self.bridge_status.get("ready", False)):
            return "localization bridge is not ready"
        if self.bridge_rejection_growth >= 2:
            return "localization bridge is repeatedly rejecting poses"
        if self.battery is None or not self._fresh("battery", self.battery_max_age):
            return "flight battery telemetry is missing or stale"
        return battery_safety_reason(
            float(self.battery.voltage),
            float(self.battery.percentage),
            True,
            self.minimum_battery_voltage,
            self.minimum_battery_percentage,
        )

    def _target_position(self) -> Optional[Tuple[float, float, float]]:
        if self.target is None:
            return None
        point = self.target.pose.position
        return point.x, point.y, point.z

    def _errors(self) -> Optional[Tuple[float, float, float]]:
        target = self._target_position()
        if self.pose is None or target is None:
            return None
        point = self.pose.pose.position
        try:
            return position_errors((point.x, point.y, point.z), target)
        except ValueError:
            return None

    def _status_timer(self, _event) -> None:
        reason = self._preflight_reason() if self.phase in (
            MissionPhase.WAITING,
            MissionPhase.READY,
        ) else (
            None
            if self.phase in (
                MissionPhase.COMPLETE,
                MissionPhase.ABORTED,
                MissionPhase.PILOT_TAKEOVER,
            )
            else self._flight_reason()
        )
        errors = self._errors()
        remaining = None
        if self.phase == MissionPhase.HOVER and self.hover_deadline is not None:
            remaining = max(0.0, self.hover_deadline - self._now())
        report = {
            "phase": self.phase,
            "ready": self.phase == MissionPhase.READY,
            "detail": self.detail,
            "health_reason": reason,
            "allow_arming": self.allow_arming,
            "start_requested": self.start_requested,
            "target_position": self._target_position(),
            "current_position": None
            if self.pose is None
            else [
                self.pose.pose.position.x,
                self.pose.pose.position.y,
                self.pose.pose.position.z,
            ],
            "horizontal_error_m": None if errors is None else errors[0],
            "vertical_error_m": None if errors is None else errors[1],
            "hover_remaining_s": remaining,
            "landed_state": None
            if self.extended_state is None
            else self.extended_state.landed_state,
            "disturbance_count": self.disturbance.count,
            "maximum_horizontal_error_m": self.disturbance.maximum_error_m,
            "last_recovery_s": self.disturbance.last_recovery_s,
        }
        self.status_pub.publish(
            String(data=json.dumps(report, ensure_ascii=False, sort_keys=True))
        )

    def _update_waiting(self) -> None:
        reason = self._preflight_reason()
        if reason is None:
            self.phase = MissionPhase.READY
            self.detail = "ready; call /target_follow/hover_test/start"
        else:
            self.phase = MissionPhase.WAITING
            self.detail = reason

    def _copy_pose_to_target(self) -> None:
        if self.pose is None:
            raise MissionAbort("local pose disappeared")
        self.target = PoseStamped()
        self.target.header.frame_id = self.pose.header.frame_id or "map"
        self.target.pose = copy.deepcopy(self.pose.pose)

    def _publish_target(self) -> None:
        if self.target is None:
            raise MissionAbort("target pose is not initialized")
        self.target.header.stamp = rospy.Time.now()
        self.setpoint_pub.publish(self.target)

    def _check_active(self) -> None:
        if self.abort_requested:
            raise MissionAbort("operator requested abort")
        if self.state is not None and pilot_took_over(
            self.owned_offboard, self.phase, self.state.mode
        ):
            raise PilotTakeover("flight mode changed to {}".format(self.state.mode))
        reason = self._flight_reason()
        if reason:
            raise MissionAbort(reason)
        if self.owned_offboard and self.state is not None and not self.state.armed:
            raise MissionAbort("vehicle unexpectedly disarmed")
        if self.owned_offboard and self.phase == MissionPhase.TAKEOFF:
            errors = self._errors()
            if errors is None:
                raise MissionAbort("position error is unavailable")
            if errors[0] > self.limits.maximum_horizontal_error:
                raise MissionAbort("horizontal takeoff deviation exceeded safety limit")

    def _hold_for(self, seconds: float, active: bool) -> None:
        deadline = self._now() + seconds
        while not rospy.is_shutdown() and self._now() < deadline:
            self._publish_target()
            if active:
                self._check_active()
            elif self._preflight_reason() is not None:
                raise MissionAbort(self._preflight_reason() or "preflight failed")
            self.rate.sleep()
        if rospy.is_shutdown():
            raise MissionAbort("ROS shut down while holding a setpoint")

    def _enter_offboard_and_arm(self) -> None:
        self.phase = MissionPhase.ENTERING_OFFBOARD
        self.detail = "requesting OFFBOARD and arming"
        deadline = self._now() + self.command_timeout
        last_mode = 0.0
        last_arm = 0.0
        while not rospy.is_shutdown() and self._now() < deadline:
            self._publish_target()
            if self.abort_requested:
                raise MissionAbort("operator requested abort")
            if self.state is not None and pilot_took_over(
                self.owned_offboard, self.phase, self.state.mode
            ):
                raise PilotTakeover(
                    "flight mode changed to {}".format(self.state.mode)
                )
            reason = self._flight_reason()
            if reason:
                raise MissionAbort(reason)
            now = self._now()
            if self.state is not None and self.state.armed and not self.owned_offboard:
                raise PilotTakeover("FCU armed outside this mission")
            if self.state is not None and self.state.mode == "OFFBOARD":
                self.owned_offboard = True
                if self.state.armed:
                    return
                if now - last_arm >= 2.0:
                    response = self.arm_service(True)
                    rospy.loginfo("Arm request success=%s", response.success)
                    last_arm = now
            elif now - last_mode >= 2.0:
                response = self.mode_service(custom_mode="OFFBOARD")
                rospy.loginfo("OFFBOARD request accepted=%s", response.mode_sent)
                last_mode = now
            self.rate.sleep()
        raise MissionAbort("timed out entering OFFBOARD and arming")

    def _takeoff(self, start_z: float, target_z: float) -> None:
        self.phase = MissionPhase.TAKEOFF
        self.detail = "ascending to relative {:.2f} m".format(
            self.limits.takeoff_height
        )
        started = self._now()
        stable_since: Optional[float] = None
        timeout = (
            self.limits.takeoff_height / self.limits.takeoff_speed
            + self.limits.settle_time
            + 15.0
        )
        while not rospy.is_shutdown():
            elapsed = self._now() - started
            assert self.target is not None
            self.target.pose.position.z = ramped_altitude(
                start_z, target_z, self.limits.takeoff_speed, elapsed
            )
            self._publish_target()
            self._check_active()
            assert self.pose is not None
            error = abs(target_z - self.pose.pose.position.z)
            if self.target.pose.position.z >= target_z and error <= self.limits.position_tolerance:
                if stable_since is None:
                    stable_since = self._now()
                elif self._now() - stable_since >= self.limits.settle_time:
                    return
            else:
                stable_since = None
            if elapsed >= timeout:
                raise MissionAbort(
                    "takeoff did not settle; vertical error {:.2f} m".format(error)
                )
            self.rate.sleep()
        raise MissionAbort("ROS shut down during takeoff")

    def _hover(self) -> None:
        self.phase = MissionPhase.HOVER
        self.detail = "holding takeoff point for {:.1f} s".format(
            self.limits.hover_duration
        )
        self.hover_deadline = self._now() + self.limits.hover_duration
        while not rospy.is_shutdown() and self._now() < self.hover_deadline:
            self._publish_target()
            self._check_active()
            errors = self._errors()
            if errors is None:
                raise MissionAbort("position error is unavailable")
            horizontal, vertical, _distance = errors
            event = self.disturbance.update(horizontal, self._now())
            if event == "disturbance":
                rospy.logwarn("Horizontal disturbance detected: %.2f m", horizontal)
            elif event == "recovered":
                rospy.loginfo(
                    "Recovered from disturbance in %.2f s",
                    self.disturbance.last_recovery_s,
                )
            if horizontal > self.limits.maximum_horizontal_error:
                raise MissionAbort(
                    "horizontal error {:.2f} m exceeds {:.2f} m".format(
                        horizontal, self.limits.maximum_horizontal_error
                    )
                )
            if abs(vertical) > self.limits.maximum_hover_vertical_error:
                raise MissionAbort(
                    "vertical error {:.2f} m exceeds {:.2f} m".format(
                        abs(vertical), self.limits.maximum_hover_vertical_error
                    )
                )
            self.rate.sleep()
        if rospy.is_shutdown():
            raise MissionAbort("ROS shut down during hover")

    def _request_land(self, detail: str) -> bool:
        if self.owned_offboard and self.state is not None and self.state.mode != "OFFBOARD":
            self.phase = MissionPhase.PILOT_TAKEOVER
            self.detail = "mode changed before landing request: {}".format(
                self.state.mode
            )
            return False
        self.phase = MissionPhase.LANDING
        self.detail = detail
        try:
            response = self.mode_service(custom_mode="AUTO.LAND")
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logerr("AUTO.LAND request failed: %s", exc)
            return False
        if not response.mode_sent:
            rospy.logerr("PX4 rejected AUTO.LAND request")
            return False
        deadline = self._now() + self.landing_timeout
        mode_deadline = self._now() + self.landing_mode_timeout
        while not rospy.is_shutdown() and self._now() < deadline:
            if self.state is not None and self.state.mode not in (
                "OFFBOARD", "AUTO.LAND"
            ):
                self.phase = MissionPhase.PILOT_TAKEOVER
                self.detail = "mode changed during landing: {}".format(
                    self.state.mode
                )
                return False
            if (
                self.state is not None
                and self._fresh("state", self.state_max_age)
                and not self.state.armed
                and self.extended_state is not None
                and self._fresh("extended_state", self.state_max_age)
                and self.extended_state.landed_state
                == ExtendedState.LANDED_STATE_ON_GROUND
            ):
                self.phase = MissionPhase.COMPLETE
                self.detail = "landed and disarmed"
                return True
            if (
                self.state is not None
                and self.state.mode == "OFFBOARD"
                and self._now() >= mode_deadline
            ):
                self.detail = "PX4 did not enter AUTO.LAND before timeout"
                return False
            if (
                self.state is not None
                and self.state.mode == "OFFBOARD"
                and self._flight_reason() is None
                and self.target is not None
            ):
                # Keep the stream alive while the accepted landing mode is
                # pending. Once AUTO.LAND engages, PX4 owns the descent.
                self._publish_target()
            self.rate.sleep()
        self.detail = "landing was not confirmed before timeout"
        return False

    def run(self) -> None:
        rospy.logwarn(
            "OFFBOARD hover test loaded; allow_arming=%s. No flight starts until "
            "the Trigger service is explicitly called.",
            self.allow_arming,
        )
        while not rospy.is_shutdown() and not self.start_requested:
            self._update_waiting()
            self.rate.sleep()
        if rospy.is_shutdown():
            return

        try:
            reason = self._preflight_reason()
            if reason:
                raise MissionAbort(reason)
            self._copy_pose_to_target()
            self.phase = MissionPhase.PRIMING
            self.detail = "priming OFFBOARD setpoints at current pose"
            self._hold_for(self.priming_duration, active=False)
            self._enter_offboard_and_arm()

            # PX4 may reset the local origin while arming. Re-capture XYZ and yaw.
            self._copy_pose_to_target()
            assert self.pose is not None and self.target is not None
            start_z = self.pose.pose.position.z
            target_z = start_z + self.limits.takeoff_height
            self._hold_for(0.5, active=True)
            self._takeoff(start_z, target_z)
            self.target.pose.position.z = target_z
            self._hover()
            if not self._request_land("hover complete; AUTO.LAND requested"):
                if self.phase == MissionPhase.PILOT_TAKEOVER:
                    rospy.logwarn("Pilot/PX4 took over during landing: %s", self.detail)
                else:
                    self.phase = MissionPhase.ABORTED
                    self.detail = "AUTO.LAND failed or landing was not confirmed"
                    rospy.logerr("%s; setpoints stopped", self.detail)
        except PilotTakeover as exc:
            self.phase = MissionPhase.PILOT_TAKEOVER
            self.detail = str(exc)
            rospy.logwarn("Pilot/PX4 took over: %s; OFFBOARD will not be reclaimed", exc)
        except (MissionAbort, rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("Hover mission aborted: %s", exc)
            self.detail = str(exc)
            if self.owned_offboard and self.state is not None and self.state.armed:
                if not self._request_land("aborted: {}; requesting AUTO.LAND".format(exc)):
                    if self.phase != MissionPhase.PILOT_TAKEOVER:
                        # Deliberately stop publishing setpoints. PX4's configured
                        # OFFBOARD-loss action remains the final fallback.
                        self.phase = MissionPhase.ABORTED
                        self.detail = "{}; AUTO.LAND failed, setpoints stopped".format(exc)
            else:
                self.phase = MissionPhase.ABORTED
        finally:
            self._status_timer(None)
            # Leave the latched result available without allowing a second
            # mission to arm. No further control outputs are emitted.
            if not rospy.is_shutdown():
                rospy.spin()


def main() -> None:
    rospy.init_node("offboard_hover_test")
    try:
        OffboardHoverTest().run()
    except ValueError as exc:
        rospy.logfatal("Invalid hover-test configuration: %s", exc)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
