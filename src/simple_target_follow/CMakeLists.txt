cmake_minimum_required(VERSION 3.0.2)
project(simple_target_follow)
find_package(catkin REQUIRED COMPONENTS rospy cv_bridge geometry_msgs sensor_msgs std_msgs mavros_msgs)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS rospy cv_bridge geometry_msgs sensor_msgs std_msgs mavros_msgs)
catkin_install_python(PROGRAMS
  scripts/color_target_node.py scripts/fast_livo_pose_bridge.py
  scripts/hardware_monitor.py scripts/sensor_monitor.py
  scripts/px4_vision_audit.py
  scripts/localization_quality_test.py scripts/pose_fusion_test.py scripts/axis_motion_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
