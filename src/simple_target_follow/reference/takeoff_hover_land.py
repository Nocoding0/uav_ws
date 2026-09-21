#!/usr/bin/env python3
"""Minimal PX4 OFFBOARD takeoff, hover and AUTO.LAND sequence."""

from __future__ import annotations

import json
import math
import threading
from typing import Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from marker_detection import (
    detect_concentric_marker as detect_marker_pattern,
)
from simple_target_follow.mission_safety import (
    flight_telemetry_reason,
    health_status_reason,
)


class FlightAbort(RuntimeError):
    pass


class TakeoffHoverLand:
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    PX4_FORCE_DISARM_MAGIC = 21196.0

    def __init__(self) -> None:
        self.state = State()
        self.pose = PoseStamped()
        self.state_time: Optional[rospy.Time] = None
        self.pose_time: Optional[rospy.Time] = None

        self.allow_arming = bool(rospy.get_param("~allow_arming", False))
        self.motor_test_mode = bool(
            rospy.get_param("~motor_test_mode", False)
        )
        self.confirm_propellers_removed = bool(
            rospy.get_param("~confirm_propellers_removed", False)
        )
        self.motor_test_duration = float(
            rospy.get_param("~motor_test_duration", 3.0)
        )
        self.takeoff_height = float(rospy.get_param("~takeoff_height", 1.5))
        self.hover_duration = float(rospy.get_param("~hover_duration", 10.0))
        self.takeoff_speed = float(rospy.get_param("~takeoff_speed", 0.3))
        self.setpoint_rate = float(rospy.get_param("~setpoint_rate", 20.0))
        self.position_tolerance = float(
            rospy.get_param("~position_tolerance", 0.10)
        )
        self.settle_time = float(rospy.get_param("~settle_time", 1.5))
        self.preflight_timeout = float(
            rospy.get_param("~preflight_timeout", 60.0)
        )
        self.command_timeout = float(rospy.get_param("~command_timeout", 30.0))
        self.telemetry_max_age = float(
            rospy.get_param("~telemetry_max_age", 1.0)
        )
        self.state_max_age = float(
            rospy.get_param("~state_max_age", self.telemetry_max_age)
        )
        self.pose_max_age = float(
            rospy.get_param("~pose_max_age", self.telemetry_max_age)
        )
        self.tracking_enabled = bool(
            rospy.get_param("~tracking_enabled", False)
        )
        self.vision_only = bool(rospy.get_param("~vision_only", False))
        self.camera_orientation = str(
            rospy.get_param("~camera_orientation", "forward")
        ).strip().lower()
        self.camera_yaw_offset = math.radians(
            float(rospy.get_param("~camera_yaw_offset_deg", 0.0))
        )
        self.desired_center_x = float(
            rospy.get_param("~desired_center_x", 0.5)
        )
        self.desired_center_y = float(
            rospy.get_param("~desired_center_y", 0.5)
        )
        self.tracking_duration = float(
            rospy.get_param("~tracking_duration", 30.0)
        )
        self.tracking_mode = str(
            rospy.get_param("~tracking_mode", "hover")
        ).strip().lower()
        self.follow_confirm_time = float(
            rospy.get_param("~follow_confirm_time", 1.0)
        )
        self.wait_for_start_command = bool(
            rospy.get_param("~wait_for_start_command", False)
        )
        self.start_topic = str(
            rospy.get_param("~start_topic", "/vehicle/start")
        )
        self.car_command_topic = str(
            rospy.get_param("~car_command_topic", "/car/command")
        ).strip()
        self.car_start_command = str(
            rospy.get_param("~car_start_command", "CAR,START")
        ).strip()
        self.start_timeout = float(rospy.get_param("~start_timeout", 0.0))
        self.start_requested = not self.wait_for_start_command
        self.require_marker_before_takeoff = bool(
            rospy.get_param("~require_marker_before_takeoff", False)
        )
        self.marker_preflight_timeout = float(
            rospy.get_param("~marker_preflight_timeout", 20.0)
        )
        self.marker_center_tolerance_x = float(
            rospy.get_param("~marker_center_tolerance_x", 0.04)
        )
        self.marker_center_tolerance_y = float(
            rospy.get_param("~marker_center_tolerance_y", 0.04)
        )
        self.marker_hover_duration = float(
            rospy.get_param("~marker_hover_duration", 5.0)
        )
        self.marker_acquisition_timeout = float(
            rospy.get_param("~marker_acquisition_timeout", 15.0)
        )
        self.marker_loss_timeout = float(
            rospy.get_param("~marker_loss_timeout", 3.0)
        )
        self.marker_timeout = float(rospy.get_param("~marker_timeout", 0.60))
        self.search_delay = float(rospy.get_param("~search_delay", 2.0))
        self.search_yaw_rate = math.radians(
            float(rospy.get_param("~search_yaw_rate_deg", 12.0))
        )
        self.search_forward_speed = float(
            rospy.get_param("~search_forward_speed", 0.0)
        )
        self.search_right_speed = float(
            rospy.get_param("~search_right_speed", 0.0)
        )
        self.desired_marker_size = float(
            rospy.get_param("~desired_marker_size", 0.22)
        )
        self.forward_gain = float(rospy.get_param("~forward_gain", 1.2))
        self.lateral_gain = float(rospy.get_param("~lateral_gain", 0.8))
        self.longitudinal_gain = float(
            rospy.get_param("~longitudinal_gain", 0.8)
        )
        self.max_tracking_speed = float(
            rospy.get_param("~max_tracking_speed", 0.35)
        )
        self.max_tracking_radius = float(
            rospy.get_param("~max_tracking_radius", 4.0)
        )
        self.return_speed = float(rospy.get_param("~return_speed", 0.25))
        self.minimum_marker_area = float(
            rospy.get_param("~minimum_marker_area", 120.0)
        )
        self.minimum_marker_size = float(
            rospy.get_param("~minimum_marker_size", 0.045)
        )
        self.maximum_marker_size = float(
            rospy.get_param("~maximum_marker_size", 0.70)
        )
        self.marker_color = str(
            rospy.get_param("~marker_color", "black")
        ).strip().lower()
        self.blue_hsv_lower = tuple(
            int(value)
            for value in rospy.get_param("~blue_hsv_lower", [90, 70, 40])
        )
        self.blue_hsv_upper = tuple(
            int(value)
            for value in rospy.get_param("~blue_hsv_upper", [140, 255, 255])
        )
        self.marker_confirm_frames = int(
            rospy.get_param("~marker_confirm_frames", 5)
        )
        self.marker_miss_tolerance_frames = int(
            rospy.get_param("~marker_miss_tolerance_frames", 3)
        )
        self.marker_filter_alpha = float(
            rospy.get_param("~marker_filter_alpha", 0.35)
        )
        self.maximum_center_jump = float(
            rospy.get_param("~maximum_center_jump", 0.18)
        )
        self.maximum_size_jump = float(
            rospy.get_param("~maximum_size_jump", 0.12)
        )
        self.tracking_integral_gain = float(
            rospy.get_param("~tracking_integral_gain", 0.0)
        )
        self.tracking_integral_limit = float(
            rospy.get_param("~tracking_integral_limit", 0.40)
        )
        self.align_yaw_to_motion = bool(
            rospy.get_param("~align_yaw_to_motion", False)
        )
        self.maximum_tracking_yaw_rate = math.radians(
            float(rospy.get_param("~maximum_tracking_yaw_rate_deg", 30.0))
        )
        self.annotated_topic = rospy.get_param(
            "~annotated_topic", "/vehicle_marker/image"
        )
        self.require_hardware_status = bool(
            rospy.get_param("~require_hardware_status", False)
        )
        self.hardware_status_max_age = float(
            rospy.get_param("~hardware_status_max_age", 3.0)
        )
        self.require_sensor_status = bool(
            rospy.get_param("~require_sensor_status", False)
        )
        self.sensor_status_max_age = float(
            rospy.get_param("~sensor_status_max_age", 3.0)
        )

        if self.takeoff_height <= 0.0:
            raise ValueError("takeoff_height must be positive")
        if self.motor_test_duration <= 0.0 or self.motor_test_duration > 10.0:
            raise ValueError("motor_test_duration must be in (0, 10] seconds")
        if self.motor_test_mode and not self.confirm_propellers_removed:
            raise ValueError(
                "motor_test_mode requires confirm_propellers_removed:=true"
            )
        if self.hover_duration < 0.0:
            raise ValueError("hover_duration cannot be negative")
        if self.takeoff_speed <= 0.0:
            raise ValueError("takeoff_speed must be positive")
        if self.setpoint_rate < 10.0:
            raise ValueError("setpoint_rate must be at least 10 Hz")
        if self.marker_confirm_frames < 1:
            raise ValueError("marker_confirm_frames must be at least 1")
        if self.marker_miss_tolerance_frames < 0:
            raise ValueError("marker_miss_tolerance_frames cannot be negative")
        if self.tracking_mode not in {"hover", "follow"}:
            raise ValueError("tracking_mode must be 'hover' or 'follow'")
        if self.follow_confirm_time <= 0.0:
            raise ValueError("follow_confirm_time must be positive")
        if self.start_timeout < 0.0:
            raise ValueError("start_timeout cannot be negative")
        if self.car_command_topic and not self.car_start_command:
            raise ValueError(
                "car_start_command cannot be empty when car_command_topic is set"
            )
        if self.tracking_integral_gain < 0.0:
            raise ValueError("tracking_integral_gain cannot be negative")
        if self.tracking_integral_limit <= 0.0:
            raise ValueError("tracking_integral_limit must be positive")
        if self.maximum_tracking_yaw_rate <= 0.0:
            raise ValueError("maximum_tracking_yaw_rate_deg must be positive")
        if not 0.0 < self.minimum_marker_size < self.maximum_marker_size:
            raise ValueError(
                "marker size limits must satisfy 0 < minimum < maximum"
            )
        if self.maximum_marker_size > 1.0:
            raise ValueError("maximum_marker_size cannot exceed 1")
        if self.marker_preflight_timeout <= 0.0:
            raise ValueError("marker_preflight_timeout must be positive")
        if not 0.0 < self.marker_center_tolerance_x < 0.5:
            raise ValueError("marker_center_tolerance_x must be in (0, 0.5)")
        if not 0.0 < self.marker_center_tolerance_y < 0.5:
            raise ValueError("marker_center_tolerance_y must be in (0, 0.5)")
        if self.marker_hover_duration <= 0.0:
            raise ValueError("marker_hover_duration must be positive")
        if self.marker_acquisition_timeout <= 0.0:
            raise ValueError("marker_acquisition_timeout must be positive")
        if self.marker_loss_timeout <= 0.0:
            raise ValueError("marker_loss_timeout must be positive")
        if self.hardware_status_max_age <= 0.0:
            raise ValueError("hardware_status_max_age must be positive")
        if self.sensor_status_max_age <= 0.0:
            raise ValueError("sensor_status_max_age must be positive")
        if self.telemetry_max_age <= 0.0:
            raise ValueError("telemetry_max_age must be positive")
        if self.state_max_age <= 0.0:
            raise ValueError("state_max_age must be positive")
        if self.pose_max_age <= 0.0:
            raise ValueError("pose_max_age must be positive")
        if not 0.0 < self.marker_filter_alpha <= 1.0:
            raise ValueError("marker_filter_alpha must be in (0, 1]")
        if self.marker_color not in {"black", "blue"}:
            raise ValueError("marker_color must be 'black' or 'blue'")
        if self.camera_orientation not in {"forward", "downward"}:
            raise ValueError(
                "camera_orientation must be 'forward' or 'downward'"
            )
        if not 0.0 <= self.desired_center_x <= 1.0:
            raise ValueError("desired_center_x must be in [0, 1]")
        if not 0.0 <= self.desired_center_y <= 1.0:
            raise ValueError("desired_center_y must be in [0, 1]")
        if len(self.blue_hsv_lower) != 3 or len(self.blue_hsv_upper) != 3:
            raise ValueError("blue HSV limits must contain three values")

        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.pose_topic = rospy.get_param(
            "~pose_topic", "/mavros/local_position/pose"
        )
        self.setpoint_topic = rospy.get_param(
            "~setpoint_topic", "/mavros/setpoint_position/local"
        )
        self.arming_service_name = rospy.get_param(
            "~arming_service", "/mavros/cmd/arming"
        )
        self.mode_service_name = rospy.get_param(
            "~mode_service", "/mavros/set_mode"
        )
        self.command_long_service_name = rospy.get_param(
            "~command_long_service", "/mavros/cmd/command"
        )
        self.image_topic = rospy.get_param(
            "~image_topic", "/camera/color/image_raw"
        )
        self.mission_status_topic = rospy.get_param(
            "~mission_status_topic", "/marker_hover/mission_status"
        )

        self.target = PoseStamped()
        self.target.header.frame_id = "map"
        self.setpoint_pub = rospy.Publisher(
            self.setpoint_topic, PoseStamped, queue_size=10
        )
        rospy.Subscriber(self.state_topic, State, self._state_callback)
        rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_callback)
        rospy.Subscriber(self.start_topic, Bool, self._start_callback)
        if self.car_command_topic:
            rospy.Subscriber(
                self.car_command_topic,
                String,
                self._car_command_callback,
                queue_size=10,
            )
        self.bridge = CvBridge()
        self.marker_lock = threading.Lock()
        self.marker_center_x = 0.5
        self.marker_center_y = 0.5
        self.marker_size = 0.0
        self.marker_time: Optional[rospy.Time] = None
        self.marker_detection_streak = 0
        self.marker_missed_frames = 0
        self.marker_confirmed = False
        self.marker_raw_detected = False
        self.hardware_status = None
        self.hardware_status_time: Optional[rospy.Time] = None
        self.sensor_status = None
        self.sensor_status_time: Optional[rospy.Time] = None
        self.mission_status_pub = rospy.Publisher(
            self.mission_status_topic, String, queue_size=5, latch=True
        )
        rospy.Subscriber(
            "/target_follow/hardware_status",
            String,
            self._hardware_status_callback,
        )
        rospy.Subscriber(
            "/target_follow/sensor_status",
            String,
            self._sensor_status_callback,
        )
        if self.tracking_enabled:
            self.annotated_pub = rospy.Publisher(
                self.annotated_topic, Image, queue_size=1
            )
            self.marker_status_pub = rospy.Publisher(
                "/vehicle_marker/status", String, queue_size=5, latch=True
            )
            rospy.Subscriber(
                self.image_topic,
                Image,
                self._image_callback,
                queue_size=1,
                buff_size=2 ** 24,
            )
        self.rate = rospy.Rate(self.setpoint_rate)
        self.arm_service = None
        self.mode_service = None
        self.command_long_service = None
        self.tracking_home = None

    def _state_callback(self, message: State) -> None:
        self.state = message
        self.state_time = rospy.Time.now()

    def _pose_callback(self, message: PoseStamped) -> None:
        self.pose = message
        self.pose_time = rospy.Time.now()

    def _start_callback(self, message: Bool) -> None:
        # Latch a true command.  A later false heartbeat from the vehicle must
        # not cancel a mission that has already started.
        if message.data:
            self._request_start(self.start_topic)

    def _car_command_callback(self, message: String) -> None:
        # Serial-to-ROS bridges commonly leave CR/LF around a received frame.
        # Strip only the frame whitespace and still require an exact command.
        if message.data.strip() == self.car_start_command:
            self._request_start(self.car_command_topic)

    def _request_start(self, source: str) -> None:
        if self.start_requested:
            return
        self.start_requested = True
        rospy.loginfo("Start command accepted from %s", source)

    def _hardware_status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                raise ValueError("hardware status must be a JSON object")
            self.hardware_status = status
        except (json.JSONDecodeError, ValueError):
            self.hardware_status = {}
        self.hardware_status_time = rospy.Time.now()

    def _sensor_status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                raise ValueError("sensor status must be a JSON object")
            self.sensor_status = status
        except (json.JSONDecodeError, ValueError):
            self.sensor_status = {}
        self.sensor_status_time = rospy.Time.now()

    def _publish_mission_status(self, phase: str, detail: str = "") -> None:
        self.mission_status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "stamp": round(rospy.Time.now().to_sec(), 3),
                        "phase": phase,
                        "detail": detail,
                    },
                    ensure_ascii=False,
                )
            )
        )

    @staticmethod
    def _yaw_from_pose(pose: PoseStamped) -> float:
        q = pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def _set_yaw(pose: PoseStamped, yaw: float) -> None:
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)

    @staticmethod
    def detect_concentric_marker(
        image: np.ndarray,
        minimum_area: float,
        color_mode: str = "black",
        blue_hsv_lower=(90, 70, 40),
        blue_hsv_upper=(140, 255, 255),
        minimum_normalized_size: float = 0.045,
        maximum_normalized_size: float = 0.70,
    ):
        """Detect the target from its cross, either ring, or full pattern."""
        return detect_marker_pattern(
            image,
            minimum_area,
            color_mode,
            blue_hsv_lower,
            blue_hsv_upper,
            minimum_normalized_size,
            maximum_normalized_size,
        )

    def _image_callback(self, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            )
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "Camera conversion failed: %s" % exc)
            return
        detection = self.detect_concentric_marker(
            image,
            self.minimum_marker_area,
            self.marker_color,
            self.blue_hsv_lower,
            self.blue_hsv_upper,
            self.minimum_marker_size,
            self.maximum_marker_size,
        )
        now = rospy.Time.now()
        if detection is not None:
            center_x, center_y, marker_size, ellipses, cross_lines = detection
            with self.marker_lock:
                marker_lock_is_fresh = (
                    self.marker_confirmed
                    and self.marker_time is not None
                    and (now - self.marker_time).to_sec()
                    <= self.marker_timeout
                )
                is_outlier = (
                    self.marker_detection_streak > 0
                    and (
                        math.hypot(
                            center_x - self.marker_center_x,
                            center_y - self.marker_center_y,
                        )
                        > self.maximum_center_jump
                        or abs(marker_size - self.marker_size)
                        > self.maximum_size_jump
                    )
                )
                if is_outlier and marker_lock_is_fresh:
                    # A single false contour must not destroy a valid lock.
                    # Keep the filtered position until either a nearby marker
                    # returns or the normal freshness timeout expires.
                    self.marker_raw_detected = False
                    self.marker_missed_frames += 1
                else:
                    if is_outlier:
                        # The old, unconfirmed candidate was probably clutter;
                        # rebase immediately so reacquisition stays quick.
                        self.marker_detection_streak = 0
                        self.marker_confirmed = False
                    self.marker_raw_detected = True
                    self.marker_missed_frames = 0
                    self.marker_detection_streak = min(
                        self.marker_confirm_frames,
                        self.marker_detection_streak + 1,
                    )
                    if self.marker_detection_streak == 1:
                        self.marker_center_x = center_x
                        self.marker_center_y = center_y
                        self.marker_size = marker_size
                    else:
                        alpha = min(max(self.marker_filter_alpha, 0.0), 1.0)
                        self.marker_center_x += alpha * (
                            center_x - self.marker_center_x
                        )
                        self.marker_center_y += alpha * (
                            center_y - self.marker_center_y
                        )
                        self.marker_size += alpha * (
                            marker_size - self.marker_size
                        )
                    if (
                        self.marker_detection_streak
                        >= self.marker_confirm_frames
                    ):
                        self.marker_confirmed = True
                        self.marker_time = now
            for ellipse in ellipses:
                cv2.ellipse(image, ellipse, (0, 255, 0), 2)
            for x1, y1, x2, y2 in cross_lines:
                cv2.line(
                    image,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (255, 0, 0),
                    2,
                )
            cv2.circle(
                image,
                (
                    int(center_x * image.shape[1]),
                    int(center_y * image.shape[0]),
                ),
                5,
                (0, 0, 255),
                -1,
            )
            cv2.putText(
                image,
                "target size={:.3f}".format(marker_size),
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
        else:
            with self.marker_lock:
                self.marker_raw_detected = False
                self.marker_missed_frames += 1
                if (
                    not self.marker_confirmed
                    and self.marker_missed_frames
                    > self.marker_miss_tolerance_frames
                ):
                    self.marker_detection_streak = 0
                if (
                    self.marker_time is None
                    or (now - self.marker_time).to_sec()
                    > self.marker_timeout
                ):
                    self.marker_confirmed = False

        with self.marker_lock:
            marker_is_fresh = bool(
                self.marker_confirmed
                and self.marker_time is not None
                and (now - self.marker_time).to_sec() <= self.marker_timeout
            )
            horizontal_error = (
                self.marker_center_x - self.desired_center_x
            )
            vertical_error = self.desired_center_y - self.marker_center_y
            size_error = self.desired_marker_size - self.marker_size
            if self.camera_orientation == "downward":
                image_forward = self.longitudinal_gain * vertical_error
                image_right = self.lateral_gain * horizontal_error
                suggested_forward = (
                    math.cos(self.camera_yaw_offset) * image_forward
                    - math.sin(self.camera_yaw_offset) * image_right
                )
                suggested_right = (
                    math.sin(self.camera_yaw_offset) * image_forward
                    + math.cos(self.camera_yaw_offset) * image_right
                )
            else:
                suggested_forward = self.forward_gain * size_error
                suggested_right = self.lateral_gain * horizontal_error
            suggested_forward = max(
                -self.max_tracking_speed,
                min(self.max_tracking_speed, suggested_forward),
            )
            suggested_right = max(
                -self.max_tracking_speed,
                min(self.max_tracking_speed, suggested_right),
            )
            status = {
                "stamp": round(now.to_sec(), 3),
                "raw_detected": self.marker_raw_detected,
                "confirmed": marker_is_fresh,
                "confirmation_frames": self.marker_detection_streak,
                "required_confirmation_frames": self.marker_confirm_frames,
                "missed_frames": self.marker_missed_frames,
                "center_x": round(self.marker_center_x, 4),
                "center_y": round(self.marker_center_y, 4),
                "marker_size": round(self.marker_size, 4),
                "horizontal_error": round(horizontal_error, 4),
                "vertical_error": round(vertical_error, 4),
                "size_error": round(size_error, 4),
                "suggested_forward_mps": (
                    round(suggested_forward, 3)
                    if marker_is_fresh
                    else 0.0
                ),
                "suggested_right_mps": (
                    round(suggested_right, 3)
                    if marker_is_fresh
                    else 0.0
                ),
                "control_output_enabled": bool(
                    self.tracking_enabled and not self.vision_only
                ),
                "target_pattern": "cross_or_ring",
                "marker_color": self.marker_color,
                "camera_orientation": self.camera_orientation,
                "tracking_mode": self.tracking_mode,
                "camera_yaw_offset_deg": round(
                    math.degrees(self.camera_yaw_offset), 2
                ),
            }
        self.marker_status_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False))
        )
        state_text = (
            "CONFIRMED"
            if status["confirmed"]
            else "DETECTING {}/{}".format(
                status["confirmation_frames"],
                status["required_confirmation_frames"],
            )
        )
        cv2.putText(
            image,
            state_text,
            (10, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0) if status["confirmed"] else (0, 165, 255),
            2,
        )
        try:
            annotated = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            annotated.header = message.header
            self.annotated_pub.publish(annotated)
        except CvBridgeError:
            pass

    def _telemetry_reason(self) -> Optional[str]:
        now = rospy.Time.now()
        return flight_telemetry_reason(
            self.state_time.to_sec() if self.state_time is not None else None,
            self.pose_time.to_sec() if self.pose_time is not None else None,
            bool(self.state.connected),
            now.to_sec(),
            self.state_max_age,
            self.pose_max_age,
        )

    def _telemetry_ready(self) -> bool:
        return self._telemetry_reason() is None

    def _hardware_status_reason(self) -> Optional[str]:
        if not self.require_hardware_status:
            return None
        received_at = (
            self.hardware_status_time.to_sec()
            if self.hardware_status_time is not None
            else None
        )
        return health_status_reason(
            self.hardware_status,
            received_at,
            rospy.Time.now().to_sec(),
            self.hardware_status_max_age,
            "hardware",
        )

    def _sensor_status_reason(self) -> Optional[str]:
        if not self.require_sensor_status:
            return None
        received_at = (
            self.sensor_status_time.to_sec()
            if self.sensor_status_time is not None
            else None
        )
        return health_status_reason(
            self.sensor_status,
            received_at,
            rospy.Time.now().to_sec(),
            self.sensor_status_max_age,
            "sensors",
        )

    def _check_telemetry(self) -> None:
        telemetry_reason = self._telemetry_reason()
        if telemetry_reason:
            raise FlightAbort(telemetry_reason)
        hardware_reason = self._hardware_status_reason()
        if hardware_reason:
            raise FlightAbort(hardware_reason)
        sensor_reason = self._sensor_status_reason()
        if sensor_reason:
            raise FlightAbort(sensor_reason)

    def _set_target_from_pose(self) -> None:
        self.target.pose = self.pose.pose
        self.target.header.frame_id = (
            self.pose.header.frame_id or self.target.header.frame_id
        )

    def _publish_target(self) -> None:
        self.target.header.stamp = rospy.Time.now()
        self.setpoint_pub.publish(self.target)

    def _wait_for_telemetry(self) -> None:
        deadline = rospy.Time.now() + rospy.Duration(self.preflight_timeout)
        while not rospy.is_shutdown():
            telemetry_reason = self._telemetry_reason()
            hardware_reason = self._hardware_status_reason()
            sensor_reason = self._sensor_status_reason()
            if (
                telemetry_reason is None
                and hardware_reason is None
                and sensor_reason is None
            ):
                return
            if rospy.Time.now() >= deadline:
                if telemetry_reason:
                    raise FlightAbort(
                        "timed out waiting for telemetry: {}".format(
                            telemetry_reason
                        )
                    )
                raise FlightAbort(
                    hardware_reason
                    or sensor_reason
                    or "hardware and sensors are not ready"
                )
            self.rate.sleep()
        raise FlightAbort("ROS shutdown while waiting for telemetry")

    def _marker_is_fresh(self, now: Optional[rospy.Time] = None) -> bool:
        if now is None:
            now = rospy.Time.now()
        with self.marker_lock:
            return bool(
                self.marker_confirmed
                and self.marker_time is not None
                and (now - self.marker_time).to_sec() <= self.marker_timeout
            )

    def _wait_for_marker_before_takeoff(self) -> None:
        if not self.require_marker_before_takeoff:
            return
        if not self.tracking_enabled:
            raise FlightAbort(
                "require_marker_before_takeoff requires tracking_enabled"
            )
        self._publish_mission_status(
            "WAITING_FOR_MARKER",
            "Place the complete marker inside the unobstructed camera view",
        )
        deadline = rospy.Time.now() + rospy.Duration(
            self.marker_preflight_timeout
        )
        while not rospy.is_shutdown():
            self._check_telemetry()
            if self._marker_is_fresh():
                rospy.loginfo("Marker locked before takeoff")
                self._publish_mission_status("MARKER_LOCKED")
                return
            if rospy.Time.now() >= deadline:
                raise FlightAbort(
                    "marker was not confirmed before the preflight timeout"
                )
            self.rate.sleep()
        raise FlightAbort("ROS shutdown while waiting for marker")

    def _wait_for_start(self) -> None:
        if not self.wait_for_start_command or self.start_requested:
            return
        self._publish_mission_status(
            "WAITING_FOR_VEHICLE_START",
            "Waiting for Bool(true) on {} or {!r} on {}".format(
                self.start_topic,
                self.car_start_command,
                self.car_command_topic,
            ),
        )
        started = rospy.Time.now()
        while not rospy.is_shutdown():
            self._check_telemetry()
            if self.start_requested:
                rospy.loginfo("Vehicle start command received")
                self._publish_mission_status("VEHICLE_STARTED")
                return
            if (
                self.start_timeout > 0.0
                and (rospy.Time.now() - started).to_sec() >= self.start_timeout
            ):
                raise FlightAbort("timed out waiting for vehicle start command")
            self.rate.sleep()
        raise FlightAbort("ROS shutdown while waiting for vehicle start")

    def _hold(self, duration: float, require_flight: bool = True) -> None:
        deadline = rospy.Time.now() + rospy.Duration(duration)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_target()
            self._check_telemetry()
            if require_flight and (
                not self.state.armed or self.state.mode != "OFFBOARD"
            ):
                raise FlightAbort(
                    "flight state changed (armed={}, mode={})".format(
                        self.state.armed, self.state.mode
                    )
                )
            self.rate.sleep()

    def _enter_offboard_and_arm(self) -> None:
        if not self.allow_arming:
            raise FlightAbort(
                "arming is blocked; pass allow_arming:=true only after "
                "completing real-vehicle safety checks"
            )

        deadline = rospy.Time.now() + rospy.Duration(self.command_timeout)
        last_mode_request = rospy.Time(0)
        last_arm_request = rospy.Time(0)
        retry = rospy.Duration(2.0)
        while not rospy.is_shutdown():
            self._publish_target()
            self._check_telemetry()
            if (
                self.require_marker_before_takeoff
                and not self._marker_is_fresh()
            ):
                raise FlightAbort("marker lock was lost before arming")
            now = rospy.Time.now()
            if self.state.mode != "OFFBOARD":
                if now - last_mode_request >= retry:
                    response = self.mode_service(custom_mode="OFFBOARD")
                    rospy.loginfo("OFFBOARD request accepted=%s", response.mode_sent)
                    last_mode_request = now
            elif not self.state.armed:
                if now - last_arm_request >= retry:
                    response = self.arm_service(True)
                    rospy.loginfo("Arm request success=%s", response.success)
                    last_arm_request = now
            else:
                return
            if now >= deadline:
                raise FlightAbort("timed out entering OFFBOARD and arming")
            self.rate.sleep()
        raise FlightAbort("ROS shutdown while arming")

    def _force_disarm_motor_test(self) -> None:
        if not (
            self.motor_test_mode
            and self.confirm_propellers_removed
            and self.command_long_service is not None
        ):
            raise FlightAbort(
                "force-disarm is only available for a confirmed "
                "propeller-removed motor test"
            )
        response = self.command_long_service(
            broadcast=False,
            command=self.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=0.0,
            param2=self.PX4_FORCE_DISARM_MAGIC,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )
        rospy.logwarn(
            "Force-disarm request success=%s result=%s",
            response.success,
            response.result,
        )

    def _disarm(self) -> None:
        rospy.loginfo("Requesting PX4 disarm")
        self._publish_mission_status("MOTOR_TEST_DISARMING")
        if self.motor_test_mode and self.mode_service is not None:
            response = self.mode_service(custom_mode="MANUAL")
            rospy.logwarn(
                "Leaving OFFBOARD before motor-test disarm; "
                "MANUAL request accepted=%s",
                response.mode_sent,
            )

        deadline = rospy.Time.now() + rospy.Duration(2.0)
        last_request = rospy.Time(0)
        retry = rospy.Duration(1.0)
        while (
            not rospy.is_shutdown()
            and self.state.armed
            and rospy.Time.now() < deadline
        ):
            self._publish_target()
            now = rospy.Time.now()
            if now - last_request >= retry:
                response = self.arm_service(False)
                rospy.loginfo("Disarm request success=%s", response.success)
                last_request = now
            self.rate.sleep()

        if self.state.armed:
            # PX4 can reject a normal disarm while its land detector still
            # reports airborne. Force-disarm is intentionally restricted to
            # the explicit, propeller-removed motor-test mode.
            rospy.logwarn(
                "Normal disarm was rejected; sending propeller-removed "
                "PX4 force-disarm command"
            )
            self._force_disarm_motor_test()
            force_deadline = rospy.Time.now() + rospy.Duration(3.0)
            while (
                not rospy.is_shutdown()
                and self.state.armed
                and rospy.Time.now() < force_deadline
            ):
                self.rate.sleep()
            if self.state.armed:
                raise FlightAbort("PX4 force-disarm failed after motor test")

        # Do not tear down MAVROS immediately after the first disarmed
        # heartbeat. Keep watching long enough to detect the FCU returning to
        # an armed state, as observed during the 2026-08-01 bench test.
        self._publish_mission_status("MOTOR_TEST_VERIFYING_DISARM")
        stable_since: Optional[rospy.Time] = None
        verification_deadline = rospy.Time.now() + rospy.Duration(12.0)
        last_force_request = rospy.Time(0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.state.armed:
                stable_since = None
                if now - last_force_request >= retry:
                    rospy.logerr(
                        "PX4 returned to armed state during motor-test "
                        "shutdown; forcing disarm again"
                    )
                    self._force_disarm_motor_test()
                    last_force_request = now
            else:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= rospy.Duration(5.0):
                    return
            if now >= verification_deadline:
                raise FlightAbort(
                    "PX4 did not remain disarmed for 5 seconds after "
                    "motor test"
                )
            self.rate.sleep()
        if rospy.is_shutdown() and self.state.armed:
            raise FlightAbort("ROS shutdown while disarming after motor test")

    def _takeoff(self, target_z: float) -> None:
        start_z = self.target.pose.position.z
        started = rospy.Time.now()
        stable_since: Optional[rospy.Time] = None
        timeout = (
            abs(target_z - start_z) / self.takeoff_speed
            + self.settle_time
            + 15.0
        )
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - started).to_sec()
            commanded_z = min(start_z + self.takeoff_speed * elapsed, target_z)
            self.target.pose.position.z = commanded_z
            self._publish_target()
            self._check_telemetry()
            if not self.state.armed or self.state.mode != "OFFBOARD":
                raise FlightAbort("OFFBOARD or armed state lost during takeoff")

            error = abs(target_z - self.pose.pose.position.z)
            if commanded_z >= target_z and error <= self.position_tolerance:
                if stable_since is None:
                    stable_since = rospy.Time.now()
                elif (
                    rospy.Time.now() - stable_since
                ).to_sec() >= self.settle_time:
                    return
            else:
                stable_since = None

            if elapsed >= timeout:
                raise FlightAbort(
                    "takeoff did not settle; altitude error {:.2f} m".format(error)
                )
            self.rate.sleep()

    def _track_marker(self) -> bool:
        """Acquire the vehicle marker, then hover or continuously follow it."""
        rospy.loginfo(
            "%s marker tracking enabled for up to %.1f seconds",
            self.camera_orientation.capitalize(),
            self.tracking_duration,
        )
        home_x = self.target.pose.position.x
        home_y = self.target.pose.position.y
        fixed_z = self.target.pose.position.z
        self.tracking_home = (home_x, home_y, fixed_z)
        started = rospy.Time.now()
        previous = started
        last_seen: Optional[rospy.Time] = None
        centered_since: Optional[rospy.Time] = None
        marker_acquired = False
        follow_established = False
        integral_forward = 0.0
        integral_right = 0.0
        self._publish_mission_status(
            "SEARCHING_MARKER",
            "Holding position until the downward camera confirms the marker",
        )

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            elapsed = (now - started).to_sec()
            if elapsed >= self.tracking_duration:
                if self.tracking_mode == "follow" and follow_established:
                    rospy.loginfo("Companion flight interval completed")
                    self._publish_mission_status(
                        "FOLLOW_COMPLETE",
                        "Vehicle remained under visual tracking",
                    )
                    return True
                rospy.logwarn("Marker centering timed out; returning home")
                self._publish_mission_status(
                    "MARKER_TIMEOUT", "Returning without a completed hover"
                )
                return False
            dt = min(max((now - previous).to_sec(), 0.0), 0.1)
            previous = now
            self._check_telemetry()
            if not self.state.armed or self.state.mode != "OFFBOARD":
                raise FlightAbort("OFFBOARD or armed state lost while tracking")

            with self.marker_lock:
                marker_time = self.marker_time
                center_x = self.marker_center_x
                center_y = self.marker_center_y
                marker_size = self.marker_size
            marker_fresh = (
                marker_time is not None
                and (now - marker_time).to_sec() <= self.marker_timeout
            )

            yaw = self._yaw_from_pose(self.target)
            forward_speed = 0.0
            right_speed = 0.0
            if marker_fresh:
                last_seen = now
                if not marker_acquired:
                    marker_acquired = True
                    rospy.loginfo("Marker acquired after takeoff")
                    self._publish_mission_status("TRACKING_MARKER")
                horizontal_error = center_x - self.desired_center_x
                vertical_error = self.desired_center_y - center_y
                size_error = self.desired_marker_size - marker_size
                centered = (
                    abs(horizontal_error) <= self.marker_center_tolerance_x
                    and abs(vertical_error) <= self.marker_center_tolerance_y
                )
                if centered:
                    if centered_since is None:
                        centered_since = now
                        self._publish_mission_status(
                            "ALIGNING_OVER_VEHICLE"
                            if self.tracking_mode == "follow"
                            else "HOVERING_OVER_MARKER"
                        )
                    elif (
                        now - centered_since
                    ).to_sec() >= (
                        self.follow_confirm_time
                        if self.tracking_mode == "follow"
                        else self.marker_hover_duration
                    ):
                        if self.tracking_mode == "follow":
                            if not follow_established:
                                follow_established = True
                                rospy.loginfo(
                                    "Companion flight established over vehicle"
                                )
                                self._publish_mission_status(
                                    "FOLLOWING_VEHICLE",
                                    "Relative position and speed control active",
                                )
                        else:
                            rospy.loginfo(
                                "Marker-centred hover completed for %.1f seconds",
                                self.marker_hover_duration,
                            )
                            self._publish_mission_status(
                                "MARKER_HOVER_COMPLETE"
                            )
                            return True
                else:
                    centered_since = None
                if self.camera_orientation == "downward":
                    integral_forward = max(
                        -self.tracking_integral_limit,
                        min(
                            self.tracking_integral_limit,
                            integral_forward + vertical_error * dt,
                        ),
                    )
                    integral_right = max(
                        -self.tracking_integral_limit,
                        min(
                            self.tracking_integral_limit,
                            integral_right + horizontal_error * dt,
                        ),
                    )
                    image_forward = (
                        self.longitudinal_gain * vertical_error
                        + self.tracking_integral_gain * integral_forward
                    )
                    image_right = (
                        self.lateral_gain * horizontal_error
                        + self.tracking_integral_gain * integral_right
                    )
                    forward_speed = (
                        math.cos(self.camera_yaw_offset) * image_forward
                        - math.sin(self.camera_yaw_offset) * image_right
                    )
                    right_speed = (
                        math.sin(self.camera_yaw_offset) * image_forward
                        + math.cos(self.camera_yaw_offset) * image_right
                    )
                else:
                    forward_speed = self.forward_gain * size_error
                    right_speed = self.lateral_gain * horizontal_error
                forward_speed = max(
                    -self.max_tracking_speed,
                    min(self.max_tracking_speed, forward_speed),
                )
                right_speed = max(
                    -self.max_tracking_speed,
                    min(self.max_tracking_speed, right_speed),
                )
                rospy.loginfo_throttle(
                    1.0,
                    "TRACK marker x=%.3f y=%.3f size=%.3f "
                    "cmd forward=%.2f right=%.2f"
                    % (
                        center_x,
                        center_y,
                        marker_size,
                        forward_speed,
                        right_speed,
                    ),
                )
            else:
                centered_since = None
                # Retain a little learned target velocity through brief frame
                # drops, but decay it so a lost target cannot cause fly-away.
                integral_forward *= max(0.0, 1.0 - 2.0 * dt)
                integral_right *= max(0.0, 1.0 - 2.0 * dt)
                if (
                    last_seen is None
                    and elapsed >= self.marker_acquisition_timeout
                ):
                    rospy.logwarn(
                        "No marker found within %.1f seconds; returning home",
                        self.marker_acquisition_timeout,
                    )
                    self._publish_mission_status(
                        "MARKER_NOT_FOUND", "Returning to takeoff point"
                    )
                    return False
                if last_seen is None and elapsed >= self.search_delay:
                    forward_speed = max(
                        -self.max_tracking_speed,
                        min(self.max_tracking_speed, self.search_forward_speed),
                    )
                    right_speed = max(
                        -self.max_tracking_speed,
                        min(self.max_tracking_speed, self.search_right_speed),
                    )
                if (
                    last_seen is not None
                    and (now - last_seen).to_sec()
                    >= self.marker_loss_timeout
                ):
                    time_without_marker = (now - last_seen).to_sec()
                    rospy.logwarn(
                        "Marker lost for %.1f seconds; returning home",
                        time_without_marker,
                    )
                    self._publish_mission_status(
                        "MARKER_LOST", "Returning to takeoff point"
                    )
                    return False
            if (
                not marker_fresh
                and (
                    last_seen is None
                    or (now - last_seen).to_sec() >= self.search_delay
                )
            ):
                yaw += self.search_yaw_rate * dt
                self._set_yaw(self.target, yaw)
                rospy.loginfo_throttle(1.0, "SEARCH marker")
            elif not marker_fresh:
                rospy.logwarn_throttle(1.0, "Marker lost; holding position")

            map_vx = (
                math.cos(yaw) * forward_speed
                - math.sin(yaw) * right_speed
            )
            map_vy = (
                math.sin(yaw) * forward_speed
                + math.cos(yaw) * right_speed
            )
            tracking_speed = math.hypot(map_vx, map_vy)
            if (
                self.align_yaw_to_motion
                and follow_established
                and marker_fresh
                and tracking_speed >= 0.03
            ):
                desired_yaw = math.atan2(map_vy, map_vx)
                yaw_error = math.atan2(
                    math.sin(desired_yaw - yaw),
                    math.cos(desired_yaw - yaw),
                )
                yaw += max(
                    -self.maximum_tracking_yaw_rate * dt,
                    min(self.maximum_tracking_yaw_rate * dt, yaw_error),
                )
                self._set_yaw(self.target, yaw)
            candidate_x = self.target.pose.position.x + map_vx * dt
            candidate_y = self.target.pose.position.y + map_vy * dt
            radius = math.hypot(candidate_x - home_x, candidate_y - home_y)
            if radius <= self.max_tracking_radius:
                self.target.pose.position.x = candidate_x
                self.target.pose.position.y = candidate_y
            else:
                rospy.logwarn_throttle(
                    1.0, "Tracking radius limit reached; holding position"
                )
            self.target.pose.position.z = fixed_z
            self._publish_target()
            self.rate.sleep()

    def _return_to_tracking_home(self) -> None:
        if self.tracking_home is None:
            return
        home_x, home_y, fixed_z = self.tracking_home
        rospy.loginfo("Tracking complete; returning to takeoff hover point")
        self._publish_mission_status("RETURNING_HOME")
        deadline = rospy.Time.now() + rospy.Duration(30.0)
        previous = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = min(max((now - previous).to_sec(), 0.0), 0.1)
            previous = now
            self._check_telemetry()
            if not self.state.armed or self.state.mode != "OFFBOARD":
                raise FlightAbort("OFFBOARD or armed state lost during return")

            dx = home_x - self.target.pose.position.x
            dy = home_y - self.target.pose.position.y
            command_distance = math.hypot(dx, dy)
            if command_distance > 1e-6:
                step = min(self.return_speed * dt, command_distance)
                self.target.pose.position.x += dx / command_distance * step
                self.target.pose.position.y += dy / command_distance * step
            self.target.pose.position.z = fixed_z
            self._publish_target()

            actual_distance = math.hypot(
                self.pose.pose.position.x - home_x,
                self.pose.pose.position.y - home_y,
            )
            if actual_distance <= self.position_tolerance:
                self._hold(self.settle_time)
                return
            if now >= deadline:
                raise FlightAbort("timed out returning to takeoff hover point")
            self.rate.sleep()

    def _land(self) -> None:
        rospy.loginfo("Requesting PX4 AUTO.LAND")
        self._publish_mission_status("LANDING")
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        last_request = rospy.Time(0)
        accepted = False
        while not rospy.is_shutdown() and self.state.armed:
            now = rospy.Time.now()
            if not accepted:
                self._publish_target()
            if now - last_request >= rospy.Duration(2.0):
                response = self.mode_service(custom_mode="AUTO.LAND")
                accepted = bool(response.mode_sent)
                last_request = now
            if accepted or self.state.mode == "AUTO.LAND":
                break
            if now >= deadline:
                raise FlightAbort("PX4 did not accept AUTO.LAND")
            self.rate.sleep()

        disarm_deadline = rospy.Time.now() + rospy.Duration(45.0)
        while not rospy.is_shutdown() and self.state.armed:
            if rospy.Time.now() >= disarm_deadline:
                raise FlightAbort("landing was accepted but PX4 did not disarm")
            self.rate.sleep()

    def run(self) -> None:
        if self.vision_only:
            if not self.tracking_enabled:
                raise FlightAbort(
                    "vision_only requires tracking_enabled to be true"
                )
            rospy.loginfo(
                "Vision-only mode: publishing detections on %s",
                self.annotated_topic,
            )
            self._publish_mission_status("VISION_ONLY")
            rospy.spin()
            return
        self._publish_mission_status("PREFLIGHT")
        rospy.wait_for_service(
            self.arming_service_name, timeout=self.preflight_timeout
        )
        rospy.wait_for_service(
            self.mode_service_name, timeout=self.preflight_timeout
        )
        if self.motor_test_mode:
            rospy.wait_for_service(
                self.command_long_service_name,
                timeout=self.preflight_timeout,
            )
        self.arm_service = rospy.ServiceProxy(
            self.arming_service_name, CommandBool
        )
        self.mode_service = rospy.ServiceProxy(self.mode_service_name, SetMode)
        if self.motor_test_mode:
            self.command_long_service = rospy.ServiceProxy(
                self.command_long_service_name,
                CommandLong,
            )

        self._wait_for_telemetry()
        self._wait_for_start()
        self._wait_for_marker_before_takeoff()
        self._set_target_from_pose()
        if not self.allow_arming:
            rospy.logwarn(
                "Preflight is ready but arming is blocked; remaining in "
                "read-only standby"
            )
            self._publish_mission_status(
                "READY_NO_ARM",
                "All required inputs are ready; allow_arming is false",
            )
            while not rospy.is_shutdown():
                self._check_telemetry()
                self.rate.sleep()
            return
        rospy.loginfo("Priming OFFBOARD setpoints at the current pose")
        self._hold(2.0, require_flight=False)
        self._enter_offboard_and_arm()

        # PX4 can move its local EKF origin while arming. Capture home again.
        self._set_target_from_pose()
        if self.motor_test_mode:
            rospy.logwarn(
                "MOTOR TEST: armed with propellers confirmed removed; "
                "holding current position target for %.1f seconds",
                self.motor_test_duration,
            )
            self._publish_mission_status("MOTOR_TEST_ARMED")
            self._hold(self.motor_test_duration)
            self._disarm()
            self._publish_mission_status("MOTOR_TEST_COMPLETE")
            rospy.loginfo("Motor test complete; PX4 is disarmed")
            return
        self._hold(1.0)
        if (
            self.require_marker_before_takeoff
            and not self._marker_is_fresh()
        ):
            raise FlightAbort("marker lock was lost before takeoff")
        target_z = self.pose.pose.position.z + self.takeoff_height
        rospy.loginfo("Taking off %.2f m", self.takeoff_height)
        self._publish_mission_status("TAKING_OFF")
        self._takeoff(target_z)
        rospy.loginfo("Hovering for %.1f seconds", self.hover_duration)
        self._publish_mission_status("TAKEOFF_HOVER")
        self._hold(self.hover_duration)
        if self.tracking_enabled:
            self._track_marker()
            self._return_to_tracking_home()
        self._land()
        self._publish_mission_status("COMPLETE")
        rospy.loginfo("Takeoff-hover-land sequence complete")

    def emergency_land(self, reason: str) -> None:
        rospy.logerr("Flight aborted: %s", reason)
        self._publish_mission_status("ABORTED", reason)
        if (
            self.motor_test_mode
            and self.state.armed
            and self.arm_service is not None
        ):
            try:
                self._disarm()
            except (FlightAbort, rospy.ServiceException) as exc:
                rospy.logerr("Emergency disarm request failed: %s", exc)
        elif self.state.armed and self.mode_service is not None:
            try:
                self._land()
            except (FlightAbort, rospy.ServiceException) as exc:
                rospy.logerr("Emergency AUTO.LAND request failed: %s", exc)


def main() -> None:
    rospy.init_node("takeoff_hover_land")
    controller: Optional[TakeoffHoverLand] = None
    try:
        controller = TakeoffHoverLand()
        controller.run()
    except (FlightAbort, ValueError, rospy.ROSException, rospy.ServiceException) as exc:
        if controller is not None:
            controller.emergency_land(str(exc))
        else:
            rospy.logfatal("%s", exc)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
