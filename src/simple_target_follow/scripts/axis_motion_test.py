#!/usr/bin/env python3
"""Read-only guided test for the sign and scale of one localization axis."""

from __future__ import annotations

import json
from typing import List

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from simple_target_follow.localization_validation import PoseObservation, assess_axis_motion
from localization_quality_test import observation


class AxisMotionTest:
    def __init__(self) -> None:
        self.axis = str(rospy.get_param("~axis", "x")).lower()
        default_distance = 1.5707963268 if self.axis == "yaw" else 0.5
        self.expected_distance = float(
            rospy.get_param("~expected_distance", default_distance)
        )
        self.baseline_seconds = float(rospy.get_param("~baseline_seconds", 3.0))
        self.motion_seconds = float(rospy.get_param("~motion_seconds", 10.0))
        self.phase = "waiting"
        self.baseline: List[PoseObservation] = []
        self.moved: List[PoseObservation] = []
        self.publisher = rospy.Publisher(
            "/target_follow/axis_motion_test", String, queue_size=1, latch=True
        )
        rospy.Subscriber(
            rospy.get_param("~pose_topic", "/mavros/local_position/pose"),
            PoseStamped,
            self._pose,
            queue_size=100,
        )

    def _pose(self, msg: PoseStamped) -> None:
        if self.phase == "baseline":
            self.baseline.append(observation(msg))
        elif self.phase == "motion":
            self.moved.append(observation(msg))

    def run(self) -> None:
        rospy.loginfo(
            "Hold still for %.1f seconds; this test is read-only", self.baseline_seconds
        )
        self.phase = "baseline"
        rospy.sleep(self.baseline_seconds)
        self.phase = "motion"
        instruction = (
            "rotate approximately +90 degrees counter-clockwise"
            if self.axis == "yaw"
            else "move +%s by %.2f m" % (self.axis.upper(), self.expected_distance)
        )
        rospy.logwarn("Now %s within %.1f seconds", instruction, self.motion_seconds)
        rospy.sleep(self.motion_seconds)
        self.phase = "done"
        report = assess_axis_motion(
            self.baseline,
            self.moved,
            self.axis,
            self.expected_distance,
            distance_tolerance=float(rospy.get_param("~distance_tolerance", 0.15)),
            cross_axis_tolerance=float(
                rospy.get_param("~cross_axis_tolerance", 0.10)
            ),
            yaw_tolerance_deg=float(rospy.get_param("~yaw_tolerance_deg", 15.0)),
        )
        encoded = json.dumps(report, ensure_ascii=False, indent=2)
        self.publisher.publish(String(data=encoded))
        path = rospy.get_param(
            "~report_path", "/tmp/target_follow_axis_{}_test.json".format(self.axis)
        )
        with open(path, "w", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
        rospy.loginfo("Axis motion report saved to %s\n%s", path, encoded)


def main() -> None:
    rospy.init_node("target_follow_axis_motion_test")
    AxisMotionTest().run()


if __name__ == "__main__":
    main()
