#!/usr/bin/env python3
"""Safely forward the validated FAST-LIVO ENU pose to MAVROS."""

from __future__ import annotations

import json

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from simple_target_follow.pose_bridge import PoseBridgeGate


class FastLivoPoseBridge:
    def __init__(self) -> None:
        self.input_topic = rospy.get_param(
            "~input_topic", "/fast_livo/unconverted_vision_pose"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/mavros/vision_pose/pose"
        )
        self.expected_input_frame = rospy.get_param(
            "~expected_input_frame", "camera_init"
        )
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.gate = PoseBridgeGate(
            warmup_samples=rospy.get_param("~warmup_samples", 10),
            maximum_age_s=rospy.get_param("~maximum_age_s", 0.25),
            future_tolerance_s=rospy.get_param("~future_tolerance_s", 0.10),
            maximum_position_m=rospy.get_param("~maximum_position_m", 100.0),
            base_step_m=rospy.get_param("~base_step_m", 0.25),
            maximum_speed_mps=rospy.get_param("~maximum_speed_mps", 3.0),
            base_angular_step_deg=rospy.get_param(
                "~base_angular_step_deg", 20.0
            ),
            maximum_angular_rate_dps=rospy.get_param(
                "~maximum_angular_rate_dps", 360.0
            ),
        )
        self.published = 0
        self.rejected = 0
        self.last_reason = "waiting for FAST-LIVO"
        self.pose_publisher = rospy.Publisher(
            self.output_topic, PoseStamped, queue_size=10
        )
        self.status_publisher = rospy.Publisher(
            "/target_follow/localization_bridge_status",
            String,
            queue_size=1,
            latch=True,
        )
        rospy.Subscriber(
            self.input_topic, PoseStamped, self._pose_callback, queue_size=20
        )
        self.status_timer = rospy.Timer(rospy.Duration(0.5), self._publish_status)

    def _reject(self, reason: str) -> None:
        self.rejected += 1
        self.last_reason = reason
        rospy.logwarn_throttle(2.0, "FAST-LIVO pose rejected: %s", reason)

    def _pose_callback(self, message: PoseStamped) -> None:
        if (
            self.expected_input_frame
            and message.header.frame_id != self.expected_input_frame
        ):
            self._reject(
                "unexpected frame_id {!r}".format(message.header.frame_id)
            )
            return

        pose = message.pose
        decision = self.gate.evaluate(
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            message.header.stamp.to_sec(),
            rospy.Time.now().to_sec(),
        )
        self.last_reason = decision.reason
        if not decision.accepted:
            self._reject(decision.reason)
            return
        if not decision.ready:
            return

        assert decision.orientation is not None
        output = PoseStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.output_frame
        output.pose.position = pose.position
        output.pose.orientation.x = decision.orientation[0]
        output.pose.orientation.y = decision.orientation[1]
        output.pose.orientation.z = decision.orientation[2]
        output.pose.orientation.w = decision.orientation[3]
        self.pose_publisher.publish(output)
        self.published += 1

    def _publish_status(self, _event: rospy.timer.TimerEvent) -> None:
        report = {
            "ready": self.gate.accepted_samples >= self.gate.warmup_samples,
            "input_topic": self.input_topic,
            "output_topic": self.output_topic,
            "accepted_samples": self.gate.accepted_samples,
            "published_samples": self.published,
            "rejected_samples": self.rejected,
            "last_reason": self.last_reason,
        }
        self.status_publisher.publish(
            String(data=json.dumps(report, ensure_ascii=False, sort_keys=True))
        )


def main() -> None:
    rospy.init_node("fast_livo_pose_bridge")
    FastLivoPoseBridge()
    rospy.loginfo(
        "FAST-LIVO guarded ENU bridge started; no control or arming commands are sent"
    )
    rospy.spin()


if __name__ == "__main__":
    main()
