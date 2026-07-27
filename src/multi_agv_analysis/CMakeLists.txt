cmake_minimum_required(VERSION 3.0.2)
project(multi_agv_analysis)

find_package(catkin REQUIRED COMPONENTS
  agv_msgs
  rosbag
  rosgraph
  rospy
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/bag_to_csv.py
  scripts/compute_metrics.py
  scripts/record_experiment.py
  scripts/validate_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_metrics.py)
endif()
