"""Check marker acquisition and bounded motion without running ROS or motors."""

import math
import os
import sys
import unittest

PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from simple_target_follow.marker_follow import (  # noqa: E402
    MarkerLock, MarkerMeasurement, follow_setpoint,
)
from simple_target_follow.hover_mission import (  # noqa: E402
    MissionPhase, pilot_took_over,
)


class MarkerFollowTests(unittest.TestCase):
    def test_requires_complete_pattern_before_lock_and_after_timeout(self):
        lock = MarkerLock(3, 0.5, 0.18, 0.12, 0.35)
        for n in range(3):
            lock.observe((0.5, 0.5, 0.2, False), 1.0 + n * 0.1,
                         1.0 + n * 0.1)
        self.assertIsNone(lock.current(1.2))
        for n in range(3):
            stamp = 1.3 + n * 0.1
            lock.observe((0.5, 0.5, 0.2, True), stamp, stamp)
        self.assertIsNotNone(lock.current(1.5))
        lock.observe((0.52, 0.49, 0.21, False), 1.6, 1.6)
        self.assertIsNone(lock.current(1.6))
        self.assertIsNone(lock.current(2.2))
        lock.observe((0.52, 0.49, 0.21, False), 2.3, 2.3)
        self.assertIsNone(lock.current(2.3))
        for n in range(3):
            stamp = 2.4 + n * 0.1
            lock.observe((0.52, 0.49, 0.21, True), stamp, stamp)
        self.assertIsNotNone(lock.current(2.6))
        lock.observe(None, 2.7, 2.7)
        self.assertIsNone(lock.current(2.7))

    def test_rejects_old_frame_and_false_jump(self):
        lock = MarkerLock(2, 0.5, 0.18, 0.12, 0.35)
        lock.observe((0.5, 0.5, 0.2, True), 1.0, 1.0)
        lock.observe((0.51, 0.5, 0.2, True), 1.1, 1.1)
        before = lock.current(1.1)
        lock.observe((0.9, 0.9, 0.4, True), 1.2, 1.2)
        self.assertEqual(lock.current(1.2), before)
        lock.observe((0.52, 0.5, 0.2, True), 1.1, 1.3)
        self.assertEqual(lock.current(1.3), before)

    def test_downward_camera_tracks_forward_and_right_with_speed_limit(self):
        marker = MarkerMeasurement(0.8, 0.2, 0.2, 1.0)
        x, y = follow_setpoint(
            (0.0, 0.0), (0.0, 0.0), 0.0, marker,
            orientation="downward", camera_yaw_offset=0.0,
            desired_center=(0.5, 0.5), desired_size=0.22,
            forward_gain=1.0, lateral_gain=1.0,
            max_speed=0.2, max_radius=2.0, dt=0.1)
        self.assertGreater(x, 0.0)
        self.assertLess(y, 0.0)  # image right is body right, ROS Y is left
        self.assertLessEqual(math.hypot(x, y), 0.02 + 1e-9)
        self.assertEqual(follow_setpoint(
            (2.0, 0.0), (0.0, 0.0), 0.0, marker,
            orientation="downward", camera_yaw_offset=0.0,
            desired_center=(0.5, 0.5), desired_size=0.22,
            forward_gain=1.0, lateral_gain=1.0,
            max_speed=0.2, max_radius=2.0, dt=0.1), (2.0, 0.0))

    def test_forward_camera_uses_marker_size_for_distance(self):
        common = dict(
            orientation="forward", camera_yaw_offset=0.0,
            desired_center=(0.5, 0.5), desired_size=0.22,
            forward_gain=1.0, lateral_gain=1.0,
            max_speed=0.2, max_radius=2.0, dt=0.1)
        far = MarkerMeasurement(0.5, 0.1, 0.1, 1.0)
        near = MarkerMeasurement(0.5, 0.9, 0.4, 1.0)
        forward = follow_setpoint((0.0, 0.0), (0.0, 0.0), 0.0, far, **common)
        backward = follow_setpoint((0.0, 0.0), (0.0, 0.0), 0.0, near, **common)
        self.assertGreater(forward[0], 0.0)
        self.assertLess(backward[0], 0.0)
        self.assertAlmostEqual(forward[1], 0.0)
        self.assertAlmostEqual(backward[1], 0.0)

    def test_pilot_takeover_applies_during_follow_and_return(self):
        self.assertTrue(pilot_took_over(True, MissionPhase.TRACKING, "POSCTL"))
        self.assertTrue(pilot_took_over(True, MissionPhase.RETURNING, "MANUAL"))


if __name__ == "__main__":
    unittest.main()
