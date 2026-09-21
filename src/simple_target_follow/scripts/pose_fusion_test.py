#!/usr/bin/env python3
"""Read-only comparison of FAST-LIVO vision pose and PX4 fused local pose."""

from __future__ import annotations

import json
from typing import List, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from simple_target_follow.localization_validation import (
    PoseObservation,
    assess_pose_pairs,
)
from localization_quality_test import observation


class PoseFusionTest:
    def __init__(self) -> None:
        self.duration = float(rospy.get_param("~duration", 30.0))
        self.sync_slop = float(rospy.get_param("~maximum_sync_offset_s", 0.08))
        self.latest_vision: Optional[PoseObservation] = None
        self.pairs: List[Tuple[PoseObservation, PoseObservation]] = []
        self.publisher = rospy.Publisher(
            "/target_follow/pose_fusion_test", String, queue_size=1, latch=True
        )
        rospy.Subscriber(
            rospy.get_param("~vision_topic", "/mavros/vision_pose/pose"),
            PoseStamped,
            self._vision,
            queue_size=100,
        )
        rospy.Subscriber(
            rospy.get_param("~local_topic", "/mavros/local_position/pose"),
            PoseStamped,
            self._local,
            queue_size=100,
        )

    def _vision(self, msg: PoseStamped) -> None:
        self.latest_vision = observation(msg)

    def _local(self, msg: PoseStamped) -> None:
        local = observation(msg)
        if (
            self.latest_vision is not None
            and abs(self.latest_vision.stamp_s - local.stamp_s) <= self.sync_slop
        ):
            self.pairs.append((self.latest_vision, local))

    def run(self) -> None:
        rospy.loginfo(
            "Gently move and rotate the propeller-free aircraft for %.1f seconds",
            self.duration,
        )
        rospy.sleep(self.duration)
        report = assess_pose_pairs(
            self.pairs,
            maximum_sync_offset_s=self.sync_slop,
            maximum_position_error_m=float(
                rospy.get_param("~maximum_position_error_m", 0.20)
            ),
            maximum_yaw_error_deg=float(
                rospy.get_param("~maximum_yaw_error_deg", 10.0)
            ),
        )
        encoded = json.dumps(report, ensure_ascii=False, indent=2)
        self.publisher.publish(String(data=encoded))
        path = rospy.get_param("~report_path", "/tmp/target_follow_pose_fusion.json")
        with open(path, "w", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
        rospy.loginfo("Pose fusion report saved to %s\n%s", path, encoded)


def main() -> None:
    rospy.init_node("target_follow_pose_fusion_test")
    PoseFusionTest().run()


if __name__ == "__main__":
    main()
