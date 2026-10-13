#!/usr/bin/env python3
"""Compatible ROS entrypoint for the hover and marker-follow missions."""

import rospy

from simple_target_follow.offboard_mission import OffboardHoverTest


def main() -> None:
    rospy.init_node("offboard_hover_test")
    try:
        if rospy.get_param("~tracking_enabled", False):
            from simple_target_follow.marker_mission import OffboardMarkerFollow
            mission = OffboardMarkerFollow()
        else:
            mission = OffboardHoverTest()
        mission.run()
    except ValueError as exc:
        rospy.logfatal("Invalid hover-test configuration: %s", exc)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
