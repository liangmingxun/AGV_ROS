cmake_minimum_required(VERSION 3.0.2)
project(multi_agv_vision_bridge)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs rospy std_msgs
)

catkin_install_python(PROGRAMS
  scripts/vision_udp_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
