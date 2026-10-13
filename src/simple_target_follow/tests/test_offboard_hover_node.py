#!/usr/bin/env python3
"""Exercise the flight node's service gates without starting ROS or hardware."""

import importlib.util
import json
import os
import sys
import threading
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
        ("sensor_msgs", ("BatteryState", "Imu", "Image")),
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
        base = importlib.import_module("simple_target_follow.offboard_mission")
        marker = importlib.import_module("simple_target_follow.marker_mission")
        spec = importlib.util.spec_from_file_location("offboard_hover_test_stub", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        preflight_path = os.path.join(os.path.dirname(path), "hover_preflight_check.py")
        spec = importlib.util.spec_from_file_location("hover_preflight_check_stub", preflight_path)
        preflight = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(preflight)
    return base, marker, module, preflight


NODE, MARKER, ENTRY, PREFLIGHT = load_flight_node()


class OffboardHoverNodeTest(unittest.TestCase):
    def test_entrypoint_selects_hover_or_marker_without_starting_a_mission_on_import(self):
        with mock.patch.dict(sys.modules, {"simple_target_follow.marker_mission": MARKER}), \
             mock.patch.object(ENTRY.rospy, "init_node", create=True), \
             mock.patch.object(ENTRY.rospy, "get_param", return_value=False, create=True), \
             mock.patch.object(ENTRY, "OffboardHoverTest") as hover, \
             mock.patch.object(MARKER, "OffboardMarkerFollow") as marker:
            ENTRY.main()
            hover.return_value.run.assert_called_once_with()
            marker.assert_not_called()
            ENTRY.rospy.get_param.return_value = True
            ENTRY.main()
            marker.return_value.run.assert_called_once_with()
            hover.assert_called_once_with()

    def test_hover_and_read_only_preflight_share_live_safety_evidence(self):
        subscriber = mock.Mock()
        with mock.patch.object(NODE.rospy, "get_param", side_effect=lambda _key, default: default,
                               create=True), \
             mock.patch.object(NODE.rospy, "Subscriber", subscriber, create=True), \
             mock.patch.object(NODE.rospy, "Publisher", create=True), \
             mock.patch.object(NODE.rospy, "Service", create=True), \
             mock.patch.object(NODE.rospy, "ServiceProxy", create=True), \
             mock.patch.object(NODE.rospy, "Timer", create=True), \
             mock.patch.object(NODE.rospy, "Duration", create=True), \
             mock.patch.object(NODE.rospy, "Rate", create=True), \
             mock.patch.object(NODE.rospy, "AnyMsg", create=True):
            flight = NODE.OffboardHoverTest()
            flight_subscriptions = list(subscriber.call_args_list)
            subscriber.reset_mock()
            preflight = PREFLIGHT.HoverPreflightCheck()
        self.assertFalse(flight.allow_arming)
        self.assertFalse(flight.start_requested)
        self.assertEqual(len(flight_subscriptions), 7)
        self.assertEqual(len(subscriber.call_args_list), 9)

        clock = [1.0]
        for node in (flight, preflight):
            node._now = lambda: clock[0]
        for seconds in (1.0, 2.0, 3.0, 4.0):
            clock[0] = seconds
            for node in (flight, preflight):
                node._state_callback(types.SimpleNamespace(
                    connected=True, armed=False, mode="POSCTL", system_status=3))
                node._pose_callback(types.SimpleNamespace(pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0))))
                node._vision_callback(None)
                node._estimator_callback(types.SimpleNamespace(
                    velocity_horiz_status_flag=True, pos_horiz_rel_status_flag=True,
                    const_pos_mode_status_flag=False))
                node._battery_callback(types.SimpleNamespace(voltage=24.0, percentage=0.8))
                node._bridge_callback(types.SimpleNamespace(data=json.dumps({
                    "ready": True, "published_samples": int(seconds), "rejected_samples": 0})))
            for name in ("lidar", "livox_imu", "raw_lio_pose"):
                preflight._mark(name)
            preflight.imu_finite = True
        self.assertEqual(flight._preflight_checks(), preflight._preflight_checks())
        self.assertTrue(all(reason is None for reason in preflight._checks().values()))
        self.assertIsNone(flight._preflight_reason())

        for rejected in (1, 2):
            for node in (flight, preflight):
                node._bridge_callback(types.SimpleNamespace(data=json.dumps({
                    "ready": True, "published_samples": 4, "rejected_samples": rejected})))
        self.assertIn("rejection", flight._preflight_reason())
        self.assertEqual(flight._preflight_checks(), preflight._preflight_checks())
        for node in (flight, preflight):
            node._bridge_callback(types.SimpleNamespace(data="malformed"))
        self.assertFalse(flight._preflight_snapshot().bridge_ready)
        self.assertEqual(flight._preflight_checks(), preflight._preflight_checks())

    def test_marker_task_reuses_hover_then_tracks_and_returns(self):
        node = MARKER.OffboardMarkerFollow.__new__(MARKER.OffboardMarkerFollow)
        node.target = types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=1.0, y=2.0, z=1.2)))
        stages = []
        node._track_marker = lambda: stages.append(("track", node.home_xy))
        node._return_home = lambda: stages.append(("return", node.home_xy))
        with mock.patch.object(NODE.OffboardHoverTest, "_hover",
                               side_effect=lambda: stages.append(("hover", None))):
            node._run_task()
        self.assertEqual(stages, [("hover", None), ("track", (1.0, 2.0)),
                                  ("return", (1.0, 2.0))])

    def test_image_log_distinguishes_missing_and_confirmed_marker(self):
        node = MARKER.OffboardMarkerFollow.__new__(MARKER.OffboardMarkerFollow)
        node._now = lambda: 10.0
        node.cv_bridge = types.SimpleNamespace(imgmsg_to_cv2=lambda *_: object())
        node.cv_bridge_error = RuntimeError
        node.marker_timeout = 0.5
        node.marker_minimum_area = 120.0
        node.marker_color = "black"
        node.marker_minimum_size = 0.045
        node.marker_maximum_size = 0.7
        node.marker_mutex = threading.Lock()
        node.marker_lock = MARKER.MarkerLock(1, 0.5, 0.18, 0.12, 0.35)
        node.marker_status_pub = types.SimpleNamespace(publish=lambda *_: None)
        node.annotated_pub = types.SimpleNamespace(get_num_connections=lambda: 0)
        node._record_event = mock.Mock()
        node._last_observation_state = None
        node._last_observation_log = 0.0
        node.phase = NODE.MissionPhase.WAITING
        message = types.SimpleNamespace(header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(to_sec=lambda: 9.9)))
        complete = (0.6, 0.5, 0.1, [object()], [object()])
        with mock.patch.object(MARKER, "String", side_effect=lambda **kwargs: kwargs), \
             mock.patch.object(MARKER, "detect_concentric_marker",
                               side_effect=[None, complete]):
            node._image_callback(message)
            message.header.stamp.to_sec = lambda: 9.95
            node._image_callback(message)
        events = [call.kwargs for call in node._record_event.call_args_list
                  if call.args[0] == "marker_observation"]
        self.assertEqual([event["state"] for event in events],
                         ["not_detected", "confirmed"])
        self.assertEqual(events[-1]["raw_center"], [0.6, 0.5])

    def test_follow_log_links_image_measurement_to_xy_setpoint(self):
        node = MARKER.OffboardMarkerFollow.__new__(MARKER.OffboardMarkerFollow)
        clock = [0.0]
        node._now = lambda: clock[0]
        node.tracking_duration = 0.16
        node.marker_mutex = threading.Lock()
        node.marker_lock = types.SimpleNamespace(current=lambda _now: types.SimpleNamespace(
            x=0.6, y=0.5, size=0.10, stamp=1.0))
        node.latest_observation = {
            "image_stamp": 1.0, "raw_center": [0.6, 0.5],
            "raw_size": 0.10, "detected": True,
            "complete_pattern": True, "confirmed": True,
        }
        node.target = types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=1.2),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
        node.pose = types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0)))
        node.home_xy = (0.0, 0.0)
        node.last_seen = None
        node.camera_orientation = "forward"
        node.camera_yaw_offset = 0.0
        node.desired_center = (0.5, 0.5)
        node.desired_size = 0.22
        node.forward_gain = 0.75
        node.lateral_gain = 0.75
        node.max_tracking_speed = 0.20
        node.max_tracking_radius = 2.0
        node._check_active = mock.Mock()
        node._checked_follow_error = mock.Mock()
        node._publish_target = mock.Mock()
        node._record_event = mock.Mock()
        node.rate = types.SimpleNamespace(sleep=lambda: clock.__setitem__(0, clock[0] + 0.05))
        node._track_marker()
        steps = [call for call in node._record_event.call_args_list
                 if call.args[0] == "follow_step"]
        self.assertGreaterEqual(len(steps), 2)
        self.assertEqual(steps[-1].kwargs["image"]["raw_size"], 0.10)
        self.assertAlmostEqual(steps[-1].kwargs["image_size_error"], 0.12)
        self.assertNotEqual(steps[-1].kwargs["target_xy_before"],
                            steps[-1].kwargs["target_xy_after"])
        self.assertGreater(steps[-1].kwargs["setpoint_delta_xy"][0], 0.0)
        self.assertEqual(node._record_event.call_args.args[0], "tracking_end")

        clock[0] = 0.0
        node.marker_lock.current = lambda _now: None
        node.latest_observation = None
        node.last_seen = None
        node.acquisition_timeout = 0.1
        node.tracking_duration = 0.2
        node._last_hold_log = -1.0
        node._record_event.reset_mock()
        node._track_marker()
        ends = [call.kwargs["reason"] for call in node._record_event.call_args_list
                if call.args[0] == "tracking_end"]
        self.assertEqual(ends, ["not_acquired"])
        holds = [call.kwargs for call in node._record_event.call_args_list
                 if call.args[0] == "tracking_hold"]
        self.assertFalse(holds[0]["image_available"])

    def test_marker_follow_stays_unready_until_geometry_is_validated(self):
        node = MARKER.OffboardMarkerFollow.__new__(MARKER.OffboardMarkerFollow)
        node.tracking_geometry_validated = False
        with mock.patch.object(NODE.OffboardHoverTest, "_preflight_reason", return_value=None):
            self.assertIn("mounting geometry", node._preflight_reason())

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
