#!/usr/bin/env python3
"""Read-only timed quality test for vision and PX4 local pose streams."""

from __future__ import annotations

import json
from typing import Dict, List

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from simple_target_follow.localization_validation import PoseObservation, assess_pose_stream


def observation(msg: PoseStamped) -> PoseObservation:
    position = msg.pose.position
    orientation = msg.pose.orientation
    return PoseObservation(
        msg.header.stamp.to_sec(),
        rospy.Time.now().to_sec(),
        (position.x, position.y, position.z),
        (orientation.x, orientation.y, orientation.z, orientation.w),
    )


class LocalizationQualityTest:
    def __init__(self) -> None:
        self.duration = float(rospy.get_param("~duration", 30.0))
        self.require_local = bool(rospy.get_param("~require_local", True))
        self.samples: Dict[str, List[PoseObservation]] = {
            "vision": [],
            "local": [],
        }
        self.publisher = rospy.Publisher(
            "/target_follow/localization_quality", String, queue_size=1, latch=True
        )
        rospy.Subscriber(
            rospy.get_param("~vision_topic", "/mavros/vision_pose/pose"),
            PoseStamped,
            lambda msg: self.samples["vision"].append(observation(msg)),
            queue_size=200,
        )
        if self.require_local:
            rospy.Subscriber(
                rospy.get_param("~local_topic", "/mavros/local_position/pose"),
                PoseStamped,
                lambda msg: self.samples["local"].append(observation(msg)),
                queue_size=200,
            )

    def run(self) -> None:
        rospy.loginfo(
            "Keep the aircraft stationary for %.1f seconds; this test is read-only",
            self.duration,
        )
        rospy.sleep(self.duration)
        limits = {
            "minimum_rate_hz": float(rospy.get_param("~minimum_rate_hz", 10.0)),
            "maximum_latency_s": float(rospy.get_param("~maximum_latency_s", 0.15)),
            "maximum_quaternion_norm_error": float(
                rospy.get_param("~maximum_quaternion_norm_error", 0.02)
            ),
            "maximum_jump_m": float(rospy.get_param("~maximum_jump_m", 0.25)),
            "maximum_stationary_drift_m": float(
                rospy.get_param("~maximum_stationary_drift_m", 0.10)
            ),
        }
        selected_samples = {
            name: samples
            for name, samples in self.samples.items()
            if name != "local" or self.require_local
        }
        streams = {
            name: assess_pose_stream(samples, **limits)
            for name, samples in selected_samples.items()
        }
        report = {"pass": all(item["pass"] for item in streams.values()), "streams": streams}
        encoded = json.dumps(report, ensure_ascii=False, indent=2)
        self.publisher.publish(String(data=encoded))
        path = rospy.get_param(
            "~report_path", "/tmp/target_follow_localization_quality.json"
        )
        with open(path, "w", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
        rospy.loginfo("Localization quality report saved to %s\n%s", path, encoded)


def main() -> None:
    rospy.init_node("target_follow_localization_quality_test")
    LocalizationQualityTest().run()


if __name__ == "__main__":
    main()
