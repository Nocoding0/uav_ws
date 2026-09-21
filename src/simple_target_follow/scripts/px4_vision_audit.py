#!/usr/bin/env python3
"""Read-only audit of the PX4 external-vision data path.

This node never changes parameters, flight mode, arming state, or setpoints.
It reports which version-dependent EKF2 parameters exist and whether MAVROS
is receiving both the external pose input and the fused local pose output.
"""

from __future__ import annotations

import json
from typing import Dict, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, State
from mavros_msgs.srv import ParamGet
from std_msgs.msg import String


PARAMETERS = (
    "EKF2_EV_CTRL",       # newer PX4: external-vision fusion controls
    "EKF2_AID_MASK",      # older PX4: estimator aid mask
    "EKF2_HGT_REF",       # newer PX4: height reference
    "EKF2_HGT_MODE",      # older PX4: height source
    "EKF2_EV_DELAY",
    "EKF2_EV_POS_X",
    "EKF2_EV_POS_Y",
    "EKF2_EV_POS_Z",
)


class Px4VisionAudit:
    def __init__(self) -> None:
        self.duration = float(rospy.get_param("~duration", 8.0))
        self.maximum_age = float(rospy.get_param("~maximum_age", 0.5))
        self.state: Optional[State] = None
        self.estimator: Optional[EstimatorStatus] = None
        self.received: Dict[str, Optional[float]] = {"vision": None, "local": None}
        self.counts = {"vision": 0, "local": 0}
        self.publisher = rospy.Publisher(
            "/uav_localization/px4_vision_audit", String, queue_size=1, latch=True
        )
        rospy.Subscriber("/mavros/state", State, self._state, queue_size=5)
        rospy.Subscriber(
            "/mavros/estimator_status", EstimatorStatus, self._estimator, queue_size=5
        )
        rospy.Subscriber(
            "/mavros/vision_pose/pose", PoseStamped,
            lambda _msg: self._pose("vision"), queue_size=20
        )
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped,
            lambda _msg: self._pose("local"), queue_size=20
        )

    def _state(self, message: State) -> None:
        self.state = message

    def _estimator(self, message: EstimatorStatus) -> None:
        self.estimator = message

    def _pose(self, name: str) -> None:
        self.received[name] = rospy.Time.now().to_sec()
        self.counts[name] += 1

    @staticmethod
    def _param_value(response) -> object:
        if response.value.integer != 0:
            return response.value.integer
        return response.value.real

    def _read_parameters(self) -> Dict[str, object]:
        result: Dict[str, object] = {}
        service_name = "/mavros/param/get"
        try:
            rospy.wait_for_service(service_name, timeout=3.0)
            get_parameter = rospy.ServiceProxy(service_name, ParamGet)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return {"error": "parameter service unavailable: {}".format(exc)}
        for name in PARAMETERS:
            try:
                response = get_parameter(param_id=name)
                if response.success:
                    result[name] = self._param_value(response)
            except rospy.ServiceException as exc:
                result[name] = "query failed: {}".format(exc)
        return result

    def run(self) -> None:
        if self.duration <= 0.0 or self.maximum_age <= 0.0:
            raise ValueError("duration and maximum_age must be positive")
        rospy.loginfo("Collecting read-only PX4 external-vision evidence for %.1f s", self.duration)
        rospy.sleep(self.duration)
        now = rospy.Time.now().to_sec()
        fresh = {
            name: stamp is not None and 0.0 <= now - stamp <= self.maximum_age
            for name, stamp in self.received.items()
        }
        estimator = None
        if self.estimator is not None:
            estimator = {
                "attitude": self.estimator.attitude_status_flag,
                "horizontal_velocity": self.estimator.velocity_horiz_status_flag,
                "vertical_velocity": self.estimator.velocity_vert_status_flag,
                "horizontal_relative_position": self.estimator.pos_horiz_rel_status_flag,
                "vertical_absolute_position": self.estimator.pos_vert_abs_status_flag,
                "constant_position_mode": self.estimator.const_pos_mode_status_flag,
            }
        report = {
            "mavros_connected": bool(self.state and self.state.connected),
            "mode": None if self.state is None else self.state.mode,
            "armed": None if self.state is None else self.state.armed,
            "vision_pose_samples": self.counts["vision"],
            "local_pose_samples": self.counts["local"],
            "vision_pose_fresh": fresh["vision"],
            "local_pose_fresh": fresh["local"],
            "estimator_status": estimator,
            "px4_parameters_present": self._read_parameters(),
            "note": (
                "Parameter meanings depend on the connected PX4 firmware. "
                "This report is read-only and does not prove that EV fusion is active."
            ),
        }
        encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
        self.publisher.publish(String(data=encoded))
        report_path = rospy.get_param(
            "~report_path", "/tmp/uav_px4_vision_audit.json"
        )
        with open(report_path, "w", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
        rospy.loginfo("PX4 external-vision audit saved to %s\n%s", report_path, encoded)


if __name__ == "__main__":
    rospy.init_node("px4_vision_audit")
    Px4VisionAudit().run()
