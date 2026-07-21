cmake_minimum_required(VERSION 3.0.2)
project(multi_agv_bringup)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  test/test_namespace_isolation.py
  test/test_command_authority.py
  test/test_tf_authority.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
        scripts/setup_local_ros.sh
        scripts/setup_ros_network.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/namespace_isolation.test)
  add_rostest(test/command_authority.test)
  add_rostest(test/tf_authority.test)
endif()
