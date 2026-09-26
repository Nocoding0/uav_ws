#!/usr/bin/env python3
"""Exercise the flight node's service gates without starting ROS or hardware."""

import importlib.util
import os
import sys
import types
import unittest
from unittest import mock


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)


def load_flight_node():
    stubs = {}
    rospy = types.ModuleType("rospy")
    rospy.ServiceException = type("ServiceException", (Exception,), {})
    rospy.ROSException = type("ROSException", (Exception,), {})
    rospy.logerr = mock.Mock()
    rospy.is_shutdown = lambda: False
    stubs["rospy"] = rospy
    for package, message_names in (
        ("geometry_msgs", ("PoseStamped",)),
        ("mavros_msgs", ("EstimatorStatus", "ExtendedState", "State")),
        ("sensor_msgs", ("BatteryState",)),
        ("std_msgs", ("String",)),
    ):
        stub = types.ModuleType(package)
        messages = types.ModuleType(package + ".msg")
        for name in message_names:
            setattr(messages, name, type(name, (), {}))
        if package == "mavros_msgs":
            messages.ExtendedState.LANDED_STATE_ON_GROUND = 1
        stub.msg = messages
        stubs[package] = stub
        stubs[package + ".msg"] = messages
    for package, service_names in (
        ("mavros_msgs", ("CommandBool", "SetMode")),
        ("std_srvs", ("Trigger", "TriggerResponse")),
    ):
        stub = stubs.get(package, types.ModuleType(package))
        services = types.ModuleType(package + ".srv")
        for name in service_names:
            setattr(services, name, type(name, (), {}))
        if package == "std_srvs":
            services.TriggerResponse = lambda success, message: types.SimpleNamespace(
                success=success, message=message
            )
        stub.srv = services
        stubs[package] = stub
        stubs[package + ".srv"] = services
    path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "scripts", "offboard_hover_test.py")
    )
    with mock.patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location("offboard_hover_test_stub", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


NODE = load_flight_node()


class OffboardHoverNodeTest(unittest.TestCase):
    def test_explicit_start_requires_ready_and_allow_arming(self):
        node = NODE.OffboardHoverTest.__new__(NODE.OffboardHoverTest)
        node.phase = NODE.MissionPhase.WAITING
        node.detail = "waiting for localization"
        node.allow_arming = True
        node.start_requested = False
        self.assertFalse(node._start_callback(None).success)
        node.phase = NODE.MissionPhase.READY
        node.allow_arming = False
        self.assertFalse(node._start_callback(None).success)
        node.allow_arming = True
        self.assertTrue(node._start_callback(None).success)
        self.assertTrue(node.start_requested)
        self.assertFalse(node._start_callback(None).success)

    def test_abort_does_not_act_after_pilot_takes_over(self):
        node = NODE.OffboardHoverTest.__new__(NODE.OffboardHoverTest)
        node.phase = NODE.MissionPhase.PILOT_TAKEOVER
        node.abort_requested = False
        self.assertFalse(node._abort_callback(None).success)
        self.assertFalse(node.abort_requested)
        node.phase = NODE.MissionPhase.HOVER
        self.assertTrue(node._abort_callback(None).success)
        self.assertTrue(node.abort_requested)

    def test_preflight_rejects_armed_fcu_and_repeated_bridge_rejections(self):
        node = NODE.OffboardHoverTest.__new__(NODE.OffboardHoverTest)
        node.state = types.SimpleNamespace(
            connected=True, armed=True, mode="MANUAL", system_status=3
        )
        node._fresh = lambda *_args: True
        node.state_max_age = 3.0
        node.pose_max_age = 0.5
        node.estimator_max_age = 0.5
        node.bridge_max_age = 1.0
        node.battery_max_age = 3.0
        node.bridge_progress_max_age = 1.0
        node.minimum_battery_voltage = 21.0
        node.minimum_battery_percentage = 0.15
        node._now = lambda: 10.0
        self.assertIn("already armed", node._preflight_reason())

        node.state.armed = False
        node.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            )
        )
        node.estimator = types.SimpleNamespace(
            velocity_horiz_status_flag=True,
            pos_horiz_rel_status_flag=True,
            const_pos_mode_status_flag=False,
        )
        node.bridge_status = {"ready": True}
        node.bridge_progress_at = 10.0
        node.bridge_rejection_growth = 2
        node.battery = types.SimpleNamespace(voltage=24.0, percentage=0.5)
        self.assertIn("rejection", node._preflight_reason())

    def test_mode_change_skips_landing_request_and_active_control(self):
        node = NODE.OffboardHoverTest.__new__(NODE.OffboardHoverTest)
        node.owned_offboard = True
        node.phase = NODE.MissionPhase.HOVER
        node.state = types.SimpleNamespace(mode="POSCTL")
        node.abort_requested = False
        node.mode_service = mock.Mock()
        self.assertFalse(node._request_land("test failure"))
        self.assertEqual(node.phase, NODE.MissionPhase.PILOT_TAKEOVER)
        node.mode_service.assert_not_called()
        node.phase = NODE.MissionPhase.HOVER
        with self.assertRaises(NODE.PilotTakeover):
            node._check_active()

    def test_rejected_auto_land_stops_without_retrying_mode(self):
        node = NODE.OffboardHoverTest.__new__(NODE.OffboardHoverTest)
        node.owned_offboard = True
        node.phase = NODE.MissionPhase.HOVER
        node.state = types.SimpleNamespace(mode="OFFBOARD")
        node.mode_service = mock.Mock(
            return_value=types.SimpleNamespace(mode_sent=False)
        )
        self.assertFalse(node._request_land("hover complete"))
        self.assertEqual(node.phase, NODE.MissionPhase.LANDING)
        node.mode_service.assert_called_once_with(custom_mode="AUTO.LAND")

    def test_target_is_independent_from_measured_fcu_pose(self):
        node = NODE.OffboardHoverTest.__new__(NODE.OffboardHoverTest)
        node.pose = types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id="map"),
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=1.0, y=2.0, z=0.4),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        with mock.patch.object(
            NODE,
            "PoseStamped",
            side_effect=lambda: types.SimpleNamespace(
                header=types.SimpleNamespace(frame_id=""), pose=None
            ),
        ):
            node._copy_pose_to_target()
        node.target.pose.position.z = 1.6
        self.assertAlmostEqual(node.pose.pose.position.z, 0.4)
        self.assertAlmostEqual(node.target.pose.position.z, 1.6)

    def test_landing_requires_fresh_disarm_and_ground_detection(self):
        node = NODE.OffboardHoverTest.__new__(NODE.OffboardHoverTest)
        node.owned_offboard = True
        node.state = types.SimpleNamespace(mode="OFFBOARD", armed=False)
        node.extended_state = types.SimpleNamespace(landed_state=2)
        node.landing_timeout = 1.0
        node.landing_mode_timeout = 0.5
        node.state_max_age = 3.0
        node._fresh = lambda *_args: True
        node._now = mock.Mock(side_effect=[0.0, 0.0, 0.0, 2.0])
        node.rate = types.SimpleNamespace(sleep=lambda: None)

        def accept_landing(**_kwargs):
            node.state.mode = "AUTO.LAND"
            return types.SimpleNamespace(mode_sent=True)

        node.mode_service = accept_landing
        self.assertFalse(node._request_land("testing"))
        self.assertEqual(node.phase, NODE.MissionPhase.LANDING)

        node.state.mode = "OFFBOARD"
        node.extended_state.landed_state = 1
        node._now = lambda: 0.0
        self.assertTrue(node._request_land("testing"))
        self.assertEqual(node.phase, NODE.MissionPhase.COMPLETE)


if __name__ == "__main__":
    unittest.main()
