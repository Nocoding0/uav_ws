#!/usr/bin/env python3

import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PACKAGE_SRC = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from simple_target_follow.mission_safety import (
    battery_safety_reason,
    fcu_system_status_reason,
    flight_telemetry_reason,
    health_status_reason,
    relative_geofence_reason,
    telemetry_is_fresh,
)


class MissionSafetyTest(unittest.TestCase):
    def test_telemetry_freshness_rejects_missing_future_and_stale_data(self):
        self.assertTrue(telemetry_is_fresh(9.8, 10.0, 0.5))
        self.assertFalse(telemetry_is_fresh(None, 10.0, 0.5))
        self.assertFalse(telemetry_is_fresh(10.1, 10.0, 0.5))
        self.assertFalse(telemetry_is_fresh(9.0, 10.0, 0.5))

    def test_flight_telemetry_uses_independent_state_and_pose_limits(self):
        self.assertIsNone(
            flight_telemetry_reason(8.0, 9.8, True, 10.0, 3.0, 0.5)
        )
        self.assertIn(
            "MAVROS state is stale",
            flight_telemetry_reason(6.9, 9.8, True, 10.0, 3.0, 0.5),
        )
        self.assertIn(
            "local position is stale",
            flight_telemetry_reason(8.0, 9.4, True, 10.0, 3.0, 0.5),
        )

    def test_flight_telemetry_reports_missing_and_disconnected_inputs(self):
        self.assertIn(
            "state has not been received",
            flight_telemetry_reason(None, 9.9, False, 10.0, 3.0, 0.5),
        )
        self.assertIn(
            "FCU is disconnected",
            flight_telemetry_reason(9.9, 9.9, False, 10.0, 3.0, 0.5),
        )
        self.assertIn(
            "local position has not been received",
            flight_telemetry_reason(9.9, None, True, 10.0, 3.0, 0.5),
        )
        self.assertIn(
            "state timestamp is 0.10 s in the future",
            flight_telemetry_reason(10.1, 9.9, True, 10.0, 3.0, 0.5),
        )
        self.assertIn(
            "local-position timestamp is 0.10 s in the future",
            flight_telemetry_reason(9.9, 10.1, True, 10.0, 3.0, 0.5),
        )

    def test_required_battery_rejects_usb_only_and_low_voltage(self):
        self.assertIn(
            "not detected", battery_safety_reason(0.0, -0.01, True, 21.0, 0.15)
        )
        self.assertIn(
            "below", battery_safety_reason(20.5, 0.5, True, 21.0, 0.15)
        )
        self.assertIsNone(battery_safety_reason(24.0, 0.5, True, 21.0, 0.15))

    def test_unknown_percentage_does_not_override_valid_voltage(self):
        self.assertIsNone(battery_safety_reason(24.0, -0.01, True, 21.0, 0.15))
        self.assertIsNone(battery_safety_reason(24.0, math.nan, True, 21.0, 0.15))

    def test_bench_mode_does_not_require_flight_battery(self):
        self.assertIsNone(battery_safety_reason(0.0, -0.01, False, 21.0, 0.15))

    def test_fcu_system_status_only_accepts_standby_or_active(self):
        self.assertIsNone(fcu_system_status_reason(3))
        self.assertIsNone(fcu_system_status_reason(4))
        self.assertIn("CALIBRATING", fcu_system_status_reason(2))
        self.assertIn("CRITICAL", fcu_system_status_reason(5))
        self.assertIn("FLIGHT_TERMINATION", fcu_system_status_reason(8))
        self.assertIn("UNKNOWN", fcu_system_status_reason(255))

    def test_relative_geofence(self):
        bounds = (-1.0, 5.2, -1.0, 4.2, -0.2, 1.8)
        self.assertIsNone(relative_geofence_reason((4.8, 3.8, 1.2), (0, 0, 0), bounds))
        self.assertIn("relative X", relative_geofence_reason((5.3, 0, 1.2), (0, 0, 0), bounds))
        self.assertIn("non-finite", relative_geofence_reason((math.nan, 0, 0), (0, 0, 0), bounds))

    def test_health_status_requires_fresh_well_formed_ready_message(self):
        self.assertIsNone(
            health_status_reason({"ready": True}, 9.8, 10.0, 1.0, "sensors")
        )
        self.assertIn(
            "missing or stale",
            health_status_reason({"ready": True}, 8.0, 10.0, 1.0, "sensors"),
        )
        self.assertIn(
            "malformed",
            health_status_reason({}, 9.8, 10.0, 1.0, "sensors"),
        )
        reason = health_status_reason(
            {"ready": False, "reasons": ["lidar stream is stale"]},
            9.8,
            10.0,
            1.0,
            "sensors",
        )
        self.assertIn("lidar stream is stale", reason)


if __name__ == "__main__":
    unittest.main()
