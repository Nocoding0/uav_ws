#!/usr/bin/env python3

import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PACKAGE_SRC = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from simple_target_follow.localization_validation import (
    PoseObservation,
    angle_difference_deg,
    assess_axis_motion,
    assess_pose_pairs,
    assess_pose_stream,
    quaternion_yaw,
)


def pose(time_s, position=(0.0, 0.0, 0.0), yaw=0.0, latency=0.02):
    return PoseObservation(
        time_s,
        time_s + latency,
        position,
        (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
    )


class LocalizationValidationTest(unittest.TestCase):
    def test_clean_stationary_stream_passes(self):
        samples = [
            pose(1.0 + index * 0.05, (0.001 * index, 0.0, 1.0))
            for index in range(40)
        ]
        report = assess_pose_stream(
            samples, maximum_stationary_drift_m=0.10
        )
        self.assertTrue(report["pass"], report)
        self.assertGreater(report["rate_hz"], 19.0)

    def test_stream_rejects_latency_jump_drift_and_bad_quaternion(self):
        samples = [
            pose(0.0),
            PoseObservation(0.1, 0.5, (0.4, 0.0, 0.0), (0.0, 0.0, 0.0, 2.0)),
        ]
        report = assess_pose_stream(
            samples, maximum_stationary_drift_m=0.10
        )
        self.assertFalse(report["pass"])
        self.assertGreaterEqual(len(report["reasons"]), 4)

    def test_pose_pair_comparison_wraps_yaw(self):
        pairs = [
            (
                pose(1.0, (1.0, 2.0, 0.5), math.radians(179)),
                pose(1.02, (1.04, 2.0, 0.5), math.radians(-179)),
            )
        ]
        report = assess_pose_pairs(pairs)
        self.assertTrue(report["pass"], report)
        self.assertAlmostEqual(report["yaw_error_p95_deg"], 2.0, places=2)

    def test_pose_pair_rejects_wrong_frame_or_scale(self):
        pairs = [(pose(1.0, (0, 0, 0)), pose(1.01, (0.5, 0, 0)))]
        report = assess_pose_pairs(pairs)
        self.assertFalse(report["pass"])
        self.assertIn("position", " ".join(report["reasons"]))

    def test_positive_axis_motion_passes(self):
        report = assess_axis_motion(
            [pose(0.0)],
            [pose(1.0, (0.51, 0.03, 0.0))],
            "x",
            0.5,
        )
        self.assertTrue(report["pass"], report)

    def test_axis_motion_rejects_reversed_and_cross_axis_motion(self):
        report = assess_axis_motion(
            [pose(0.0)],
            [pose(1.0, (-0.5, 0.2, 0.0))],
            "x",
            0.5,
        )
        self.assertFalse(report["pass"])
        self.assertEqual(len(report["reasons"]), 2)

    def test_positive_yaw_motion_passes(self):
        report = assess_axis_motion(
            [pose(0.0)],
            [pose(1.0, yaw=math.pi / 2.0)],
            "yaw",
            math.pi / 2.0,
        )
        self.assertTrue(report["pass"], report)

    def test_yaw_helpers_use_shortest_angle(self):
        first = quaternion_yaw((0, 0, 1, 0))
        self.assertAlmostEqual(abs(math.degrees(first)), 180.0)
        self.assertAlmostEqual(
            angle_difference_deg(math.radians(179), math.radians(-179)), 2.0
        )


if __name__ == "__main__":
    unittest.main()
