#!/usr/bin/env python3

import math
import unittest

from simple_target_follow.pose_bridge import PoseBridgeGate


class PoseBridgeGateTests(unittest.TestCase):
    def test_valid_enu_pose_warms_up_and_normalizes_quaternion(self):
        gate = PoseBridgeGate(warmup_samples=2)
        first = gate.evaluate((0, 0, 0), (0, 0, 0, 0.8), 10.0, 10.01)
        second = gate.evaluate(
            (0.1, 0.02, 0.01), (0, 0, 0, 0.8), 10.1, 10.11
        )
        self.assertTrue(first.accepted)
        self.assertFalse(first.ready)
        self.assertTrue(second.ready)
        self.assertEqual(second.orientation, (0.0, 0.0, 0.0, 1.0))

    def test_rejects_nonfinite_and_bad_quaternion(self):
        gate = PoseBridgeGate()
        nonfinite = gate.evaluate(
            (math.nan, 0, 0), (0, 0, 0, 1), 10.0, 10.0
        )
        zero_quaternion = gate.evaluate(
            (0, 0, 0), (0, 0, 0, 0), 10.0, 10.0
        )
        self.assertFalse(nonfinite.accepted)
        self.assertFalse(zero_quaternion.accepted)

    def test_rejects_stale_future_and_nonmonotonic_timestamps(self):
        gate = PoseBridgeGate(warmup_samples=1)
        self.assertFalse(
            gate.evaluate((0, 0, 0), (0, 0, 0, 1), 9.0, 10.0).accepted
        )
        self.assertFalse(
            gate.evaluate((0, 0, 0), (0, 0, 0, 1), 10.2, 10.0).accepted
        )
        self.assertTrue(
            gate.evaluate((0, 0, 0), (0, 0, 0, 1), 10.0, 10.0).accepted
        )
        self.assertFalse(
            gate.evaluate((0, 0, 0), (0, 0, 0, 1), 10.0, 10.0).accepted
        )

    def test_rejects_large_position_and_orientation_discontinuities(self):
        gate = PoseBridgeGate(
            warmup_samples=1,
            base_step_m=0.1,
            maximum_speed_mps=0.0,
            base_angular_step_deg=10.0,
            maximum_angular_rate_dps=0.0,
        )
        self.assertTrue(
            gate.evaluate((0, 0, 0), (0, 0, 0, 1), 10.0, 10.0).accepted
        )
        position_jump = gate.evaluate(
            (1, 0, 0), (0, 0, 0, 1), 10.1, 10.1
        )
        self.assertFalse(position_jump.accepted)
        self.assertIn("position discontinuity", position_jump.reason)

        yaw_20_deg = math.radians(20.0) / 2.0
        orientation_jump = gate.evaluate(
            (0.01, 0, 0),
            (0, 0, math.sin(yaw_20_deg), math.cos(yaw_20_deg)),
            10.2,
            10.2,
        )
        self.assertFalse(orientation_jump.accepted)
        self.assertIn("orientation discontinuity", orientation_jump.reason)


if __name__ == "__main__":
    unittest.main()
