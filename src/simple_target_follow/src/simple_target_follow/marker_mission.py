"""Cross-and-rings perception, bounded following and return-to-home task.

This task inherits the common OFFBOARD flight lifecycle. Image callbacks publish
observations; only the active tracking stage updates flight position setpoints.
"""

from __future__ import annotations

import cv2
import json
import math
import threading
from typing import Dict, Optional, Tuple

import rospy
from std_msgs.msg import String

from simple_target_follow.hover_mission import MissionPhase
from simple_target_follow.mission_safety import telemetry_is_fresh
from simple_target_follow.offboard_mission import MissionAbort, OffboardHoverTest
from simple_target_follow.marker_detection import detect_concentric_marker
from simple_target_follow.marker_follow import MarkerLock, follow_setpoint
from simple_target_follow.marker_run_log import MarkerRunLog


class OffboardMarkerFollow(OffboardHoverTest):
    """Use the 22 September LIO flight gates with the cross-and-rings detector."""

    def __init__(self) -> None:
        super().__init__()
        from cv_bridge import CvBridge, CvBridgeError
        from sensor_msgs.msg import Image

        self.cv_bridge = CvBridge()
        self.cv_bridge_error = CvBridgeError
        self.tracking_geometry_validated = bool(
            rospy.get_param("~tracking_geometry_validated", False)
        )
        self.camera_orientation = str(rospy.get_param("~camera_orientation", "forward"))
        self.camera_yaw_offset = math.radians(float(
            rospy.get_param("~camera_yaw_offset_deg", 0.0)))
        self.marker_color = str(rospy.get_param("~marker_color", "black"))
        self.marker_minimum_area = float(rospy.get_param("~minimum_marker_area", 120.0))
        self.marker_minimum_size = float(rospy.get_param("~minimum_marker_size", 0.045))
        self.marker_maximum_size = float(rospy.get_param("~maximum_marker_size", 0.70))
        self.marker_timeout = float(rospy.get_param("~marker_timeout", 0.5))
        self.marker_lock = MarkerLock(
            int(rospy.get_param("~marker_confirm_frames", 5)),
            self.marker_timeout,
            float(rospy.get_param("~maximum_center_jump", 0.18)),
            float(rospy.get_param("~maximum_size_jump", 0.12)),
            float(rospy.get_param("~marker_filter_alpha", 0.35)),
        )
        self.tracking_duration = float(rospy.get_param("~tracking_duration", 30.0))
        self.acquisition_timeout = float(rospy.get_param("~marker_acquisition_timeout", 15.0))
        self.loss_timeout = float(rospy.get_param("~marker_loss_timeout", 3.0))
        self.desired_center = (
            float(rospy.get_param("~desired_center_x", 0.5)),
            float(rospy.get_param("~desired_center_y", 0.5)),
        )
        self.desired_size = float(rospy.get_param("~desired_marker_size", 0.22))
        self.forward_gain = float(rospy.get_param("~forward_gain", 0.75))
        self.lateral_gain = float(rospy.get_param("~lateral_gain", 0.75))
        self.max_tracking_speed = float(rospy.get_param("~max_tracking_speed", 0.20))
        self.max_tracking_radius = float(rospy.get_param("~max_tracking_radius", 2.0))
        self.max_follow_error = float(rospy.get_param("~max_follow_error", 0.8))
        self.return_speed = float(rospy.get_param("~return_speed", 0.20))
        self.return_timeout = float(rospy.get_param("~return_timeout", 35.0))
        self.image_at: Optional[float] = None
        self.marker_mutex = threading.Lock()
        self.last_seen: Optional[float] = None
        self.home_xy: Optional[Tuple[float, float]] = None
        self.latest_observation: Optional[Dict[str, object]] = None
        self._last_observation_state: Optional[str] = None
        self._last_observation_log = 0.0
        self._last_phase_log: Optional[Tuple[str, str]] = None
        self._last_hold_log = 0.0
        try:
            self.flight_log = MarkerRunLog(str(rospy.get_param(
                "~log_dir", "~/.ros/marker_follow_logs")))
        except OSError as exc:
            raise ValueError("cannot create marker-follow log: {}".format(exc))
        rospy.on_shutdown(self.flight_log.close)
        rospy.loginfo("Marker-follow log: %s", self.flight_log.path)
        self._record_event("config", image_topic=rospy.get_param(
            "~image_topic", "/camera/color/image_raw"),
            camera_orientation=self.camera_orientation,
            camera_yaw_offset_deg=math.degrees(self.camera_yaw_offset),
            desired_center=list(self.desired_center), desired_marker_size=self.desired_size,
            forward_gain=self.forward_gain, lateral_gain=self.lateral_gain,
            max_tracking_speed=self.max_tracking_speed,
            max_tracking_radius=self.max_tracking_radius,
            marker_confirm_frames=self.marker_lock.confirm_frames,
            marker_acquisition_timeout=self.acquisition_timeout,
            marker_loss_timeout=self.loss_timeout,
            note="forward distance uses image size; depth is not used")
        self.marker_status_pub = rospy.Publisher(
            rospy.get_param("~marker_status_topic", "/target_follow/marker_follow/marker"),
            String, queue_size=2, latch=True)
        self.annotated_pub = rospy.Publisher(
            rospy.get_param("~annotated_topic", "/target_follow/marker_follow/image"),
            Image, queue_size=1)
        rospy.Subscriber(rospy.get_param("~image_topic", "/camera/color/image_raw"),
                         Image, self._image_callback, queue_size=1, buff_size=2 ** 24)
        numeric = (self.camera_yaw_offset, self.marker_minimum_area,
                   self.marker_minimum_size, self.marker_maximum_size,
                   self.tracking_duration, self.acquisition_timeout,
                   self.loss_timeout, self.desired_size, *self.desired_center,
                   self.forward_gain, self.lateral_gain, self.max_tracking_speed,
                   self.max_tracking_radius, self.max_follow_error,
                   self.return_speed, self.return_timeout)
        if (self.camera_orientation not in ("downward", "forward")
                or self.marker_color not in ("black", "blue")
                or not all(math.isfinite(v) for v in numeric)
                or min(numeric[1:]) <= 0
                or self.marker_minimum_size >= self.marker_maximum_size
                or self.marker_maximum_size > 1
                or not all(0 < v < 1 for v in self.desired_center)
                or not 0 < self.desired_size < 1):
            raise ValueError("invalid marker-follow configuration")

    def _record_event(self, event: str, **fields) -> None:
        log = getattr(self, "flight_log", None)
        if log is None:
            return
        if not log.record(event, self._now(), **fields) and log.error:
            rospy.logerr_throttle(5.0, "Marker-follow log write failed: %s", log.error)

    def _status_timer(self, event) -> None:
        super()._status_timer(event)
        key = (self.phase, self.detail)
        if key != getattr(self, "_last_phase_log", None):
            self._last_phase_log = key
            self._record_event("mission_state", phase=self.phase, detail=self.detail,
                               armed=None if self.state is None else self.state.armed,
                               fcu_mode=None if self.state is None else self.state.mode,
                               target_position=self._target_position())

    def _preflight_reason(self) -> Optional[str]:
        reason = super()._preflight_reason()
        if reason:
            return reason
        if not getattr(self, "tracking_geometry_validated", False):
            return "camera and LiDAR mounting geometry has not been validated"
        image_at = getattr(self, "image_at", None)
        if not telemetry_is_fresh(image_at, self._now(),
                                  getattr(self, "marker_timeout", 0.5)):
            return "camera image is missing or stale"
        return None

    def _image_callback(self, message) -> None:
        stamp = message.header.stamp.to_sec()
        now = self._now()
        if (not math.isfinite(stamp) or stamp <= 0 or stamp > now + 0.1
                or now - stamp > self.marker_timeout):
            return
        try:
            image = self.cv_bridge.imgmsg_to_cv2(message, "bgr8")
            detection = detect_concentric_marker(
                image, self.marker_minimum_area, self.marker_color,
                minimum_normalized_size=self.marker_minimum_size,
                maximum_normalized_size=self.marker_maximum_size)
            self.image_at = now
            processing_error = None
        except (self.cv_bridge_error, cv2.error, ValueError) as exc:
            rospy.logwarn_throttle(2.0, "Marker image processing failed: %s", exc)
            detection = None
            image = None
            processing_error = str(exc)
        observed = None
        if detection is not None:
            x, y, size, ellipses, cross_lines = detection
            observed = (x, y, size, bool(ellipses and cross_lines))
        with self.marker_mutex:
            self.marker_lock.observe(observed, stamp, now)
            locked = self.marker_lock.current(now)
            self.latest_observation = {
                "image_stamp": stamp,
                "raw_center": None if observed is None else [observed[0], observed[1]],
                "raw_size": None if observed is None else observed[2],
                "detected": detection is not None,
                "complete_pattern": bool(observed and observed[3]),
                "confirmed": locked is not None,
                "processing_error": processing_error,
            }
            observation = dict(self.latest_observation)
        detection_state = ("image_error" if processing_error is not None else
                           "confirmed" if locked is not None else
                           "complete_unconfirmed" if observed and observed[3] else
                           "partial" if observed else "not_detected")
        if (detection_state != self._last_observation_state or
                now - self._last_observation_log >= 1.0):
            self._last_observation_state = detection_state
            self._last_observation_log = now
            self._record_event("marker_observation", state=detection_state,
                               phase=self.phase, **observation)
        self.marker_status_pub.publish(String(data=json.dumps({
            "stamp": stamp, "detected": detection is not None,
            "complete_pattern": bool(observed and observed[3]),
            "confirmed": locked is not None,
            "center": None if locked is None else [locked.x, locked.y],
            "size": None if locked is None else locked.size,
        }, sort_keys=True)))
        if image is not None and self.annotated_pub.get_num_connections():
            if detection is not None:
                for ellipse in ellipses:
                    cv2.ellipse(image, ellipse, (0, 255, 0), 2)
                for line in cross_lines:
                    cv2.line(image, tuple(map(int, line[:2])),
                             tuple(map(int, line[2:])), (255, 0, 0), 2)
            output = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
            output.header = message.header
            self.annotated_pub.publish(output)

    def _checked_follow_error(self) -> None:
        errors = self._errors()
        if errors is None:
            raise MissionAbort("follow position error is unavailable")
        if errors[0] > self.max_follow_error:
            raise MissionAbort("horizontal follow error exceeded limit")
        if abs(errors[1]) > self.limits.maximum_hover_vertical_error:
            raise MissionAbort("vertical follow error exceeded limit")

    def _run_task(self) -> None:
        super()._run_task()
        assert self.target is not None
        self.landing_detail = "marker follow and return complete; AUTO.LAND requested"
        self.home_xy = (self.target.pose.position.x, self.target.pose.position.y)
        self._track_marker()
        self._return_home()

    def _track_marker(self) -> None:
        assert self.target is not None and self.home_xy is not None
        self.phase = MissionPhase.TRACKING
        self.detail = "holding until marker lock, then following"
        started = self._now()
        previous = started
        end_logged = False
        yaw_q = self.target.pose.orientation
        yaw = math.atan2(2 * (yaw_q.w * yaw_q.z + yaw_q.x * yaw_q.y),
                         1 - 2 * (yaw_q.y ** 2 + yaw_q.z ** 2))
        while not rospy.is_shutdown() and self._now() - started < self.tracking_duration:
            self._check_active()
            self._checked_follow_error()
            now = self._now()
            with self.marker_mutex:
                marker = self.marker_lock.current(now)
                observation = None if self.latest_observation is None else dict(
                    self.latest_observation)
            image_age = None if observation is None else max(
                0.0, now - observation["image_stamp"])
            if marker is not None:
                self.last_seen = now
                dt = min(0.1, max(0.0, now - previous))
                before = [self.target.pose.position.x, self.target.pose.position.y]
                if dt > 0:
                    x, y = follow_setpoint(
                        (self.target.pose.position.x, self.target.pose.position.y),
                        self.home_xy, yaw, marker,
                        orientation=self.camera_orientation,
                        camera_yaw_offset=self.camera_yaw_offset,
                        desired_center=self.desired_center,
                        desired_size=self.desired_size,
                        forward_gain=self.forward_gain,
                        lateral_gain=self.lateral_gain,
                        max_speed=self.max_tracking_speed,
                        max_radius=self.max_tracking_radius, dt=dt)
                    self.target.pose.position.x = x
                    self.target.pose.position.y = y
                self.detail = "following confirmed marker"
                actual = None if self.pose is None else [
                    self.pose.pose.position.x, self.pose.pose.position.y]
                self._record_event(
                    "follow_step", image=observation,
                    filtered_center=[marker.x, marker.y], filtered_size=marker.size,
                    image_stamp=marker.stamp,
                    image_error_x=marker.x - self.desired_center[0],
                    image_size_error=self.desired_size - marker.size,
                    image_age_s=image_age, yaw_rad=yaw, dt=dt,
                    target_xy_before=before,
                    target_xy_after=[self.target.pose.position.x,
                                     self.target.pose.position.y],
                    setpoint_delta_xy=[self.target.pose.position.x - before[0],
                                       self.target.pose.position.y - before[1]],
                    actual_xy=actual)
            elif self.last_seen is None and now - started >= self.acquisition_timeout:
                self.detail = "marker not acquired; returning to takeoff point"
                self._record_event("tracking_end", reason="not_acquired",
                                   target_position=self._target_position())
                end_logged = True
                break
            elif self.last_seen is not None and now - self.last_seen >= self.loss_timeout:
                self.detail = "marker lost; returning to takeoff point"
                self._record_event("tracking_end", reason="marker_lost",
                                   target_position=self._target_position())
                end_logged = True
                break
            else:
                self.detail = "marker unavailable; holding position"
                if now - self._last_hold_log >= 1.0:
                    self._last_hold_log = now
                    self._record_event("tracking_hold", observation=observation,
                                       image_age_s=image_age,
                                       image_available=(image_age is not None and
                                                        image_age <= self.marker_timeout and
                                                        observation.get("processing_error") is None),
                                       target_position=self._target_position())
            previous = now
            self._publish_target()
            self.rate.sleep()
        if rospy.is_shutdown():
            raise MissionAbort("ROS shut down while tracking")
        if not end_logged:
            self._record_event("tracking_end", reason="duration_elapsed",
                               target_position=self._target_position())

    def _return_home(self) -> None:
        assert self.target is not None and self.home_xy is not None
        self.phase = MissionPhase.RETURNING
        self.detail = "returning to takeoff hover point"
        deadline = self._now() + self.return_timeout
        previous = self._now()
        stable_since: Optional[float] = None
        while not rospy.is_shutdown() and self._now() < deadline:
            self._check_active()
            self._checked_follow_error()
            now = self._now()
            dt = min(0.1, max(0.0, now - previous))
            previous = now
            point = self.target.pose.position
            dx = self.home_xy[0] - point.x
            dy = self.home_xy[1] - point.y
            distance = math.hypot(dx, dy)
            if distance > 0:
                step = min(distance, self.return_speed * dt)
                point.x += dx / distance * step
                point.y += dy / distance * step
            self._publish_target()
            assert self.pose is not None
            actual = self.pose.pose.position
            actual_error = math.hypot(actual.x - self.home_xy[0],
                                      actual.y - self.home_xy[1])
            if distance <= self.limits.position_tolerance and actual_error <= self.limits.position_tolerance:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= self.limits.settle_time:
                    return
            else:
                stable_since = None
            self.rate.sleep()
        raise MissionAbort("return to takeoff hover point did not settle")

