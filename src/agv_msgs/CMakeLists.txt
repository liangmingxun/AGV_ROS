cmake_minimum_required(VERSION 3.0.2)
project(agv_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  sensor_msgs
  std_msgs
)

add_message_files(FILES
  ChassisCommand.msg
  ChassisFeedback.msg
  CapabilityReport.msg
  DeratingCommand.msg
  CooperativeState.msg
  PathReference.msg
  ControllerState.msg
  ExperimentState.msg
)

generate_messages(DEPENDENCIES geometry_msgs nav_msgs sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  nav_msgs
  sensor_msgs
  std_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_frozen_interfaces.py)
endif()
