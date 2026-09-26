#!/usr/bin/env python3
"""Deterministic tests for the OFFBOARD hover mission's pure decisions."""

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from simple_target_follow.hover_mission import (
    DisturbanceTracker,
    HoverMissionLimits,
    MissionPhase,
    pilot_took_over,
    pose_window_is_stable,
    position_errors,
    ramped_altitude,
)


class HoverMissionTest(unittest.TestCase):
    def test_limits_require_valid_thresholds_and_rate(self):
        HoverMissionLimits().validate()
        with self.assertRaises(ValueError):
            HoverMissionLimits(setpoint_rate=2.0).validate()
        with self.assertRaises(ValueError):
            HoverMissionLimits(recovery_threshold=0.20).validate()
        with self.assertRaises(ValueError):
            HoverMissionLimits(takeoff_height=math.nan).validate()

    def test_altitude_ramp_is_relative_and_bounded(self):
        self.assertAlmostEqual(ramped_altitude(-0.3, 0.9, 0.25, 0.0), -0.3)
        self.assertAlmostEqual(ramped_altitude(-0.3, 0.9, 0.25, 2.0), 0.2)
        self.assertAlmostEqual(ramped_altitude(-0.3, 0.9, 0.25, 9.0), 0.9)
        with self.assertRaises(ValueError):
            ramped_altitude(0.0, 1.2, 0.25, -1.0)

    def test_position_error_and_nonfinite_rejection(self):
        horizontal, vertical, distance = position_errors(
            (0.3, 0.4, 1.1), (0.0, 0.0, 1.2)
        )
        self.assertAlmostEqual(horizontal, 0.5)
        self.assertAlmostEqual(vertical, -0.1)
        self.assertAlmostEqual(distance, math.sqrt(0.26))
        with self.assertRaises(ValueError):
            position_errors((math.nan, 0, 0), (0, 0, 0))

    def test_preflight_needs_full_three_second_stable_window(self):
        stable = [(0.0, 1.0, 2.0, 0.0), (1.5, 1.02, 2.0, 0.0),
                  (3.0, 1.04, 2.0, 0.0)]
        self.assertTrue(pose_window_is_stable(stable, 3.0, 0.10))
        self.assertFalse(pose_window_is_stable(stable[:2], 3.0, 0.10))
        unstable = stable[:2] + [(3.0, 1.2, 2.0, 0.0)]
        self.assertFalse(pose_window_is_stable(unstable, 3.0, 0.10))

    def test_pilot_mode_change_is_not_reclaimed(self):
        self.assertTrue(pilot_took_over(True, MissionPhase.HOVER, "POSCTL"))
        self.assertTrue(pilot_took_over(True, MissionPhase.TAKEOFF, "MANUAL"))
        self.assertFalse(pilot_took_over(True, MissionPhase.HOVER, "OFFBOARD"))
        self.assertFalse(pilot_took_over(False, MissionPhase.ENTERING_OFFBOARD, "MANUAL"))
        self.assertFalse(pilot_took_over(True, MissionPhase.LANDING, "AUTO.LAND"))

    def test_disturbance_requires_stable_recovery(self):
        tracker = DisturbanceTracker(0.15, 0.12, 1.5)
        self.assertIsNone(tracker.update(0.10, 0.0))
        self.assertEqual(tracker.update(0.21, 1.0), "disturbance")
        self.assertIsNone(tracker.update(0.10, 1.2))
        self.assertIsNone(tracker.update(0.16, 2.0))
        self.assertIsNone(tracker.update(0.11, 2.1))
        self.assertEqual(tracker.update(0.09, 3.7), "recovered")
        self.assertEqual(tracker.count, 1)
        self.assertAlmostEqual(tracker.maximum_error_m, 0.21)
        self.assertAlmostEqual(tracker.last_recovery_s, 2.7)
        self.assertEqual(tracker.update(0.18, 4.0), "disturbance")
        self.assertEqual(tracker.count, 2)


if __name__ == "__main__":
    unittest.main()
