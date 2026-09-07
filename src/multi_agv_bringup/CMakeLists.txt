cmake_minimum_required(VERSION 3.0.2)
project(multi_agv_bringup)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/analyze_three_car_cooperative_bag.py
  scripts/camera_wheel_turn_calibration.py
  scripts/check_three_car_readonly_gate.py
  scripts/initialize_three_car_camera_formation.py
  scripts/robot3_single_wheel_check.py
  scripts/run_timed_straight_test.py
  scripts/wait_three_car_wheel_stop.py
  test/test_namespace_isolation.py
  test/test_command_authority.py
  test/test_tf_authority.py
  test/test_experiment_recording.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
        scripts/configure_chassis_udev.sh
        scripts/check_local_clock_sync.sh
        scripts/require_three_car_motion_gate.sh
        scripts/run_m1_r1_fake_observer.sh
        scripts/run_m1_r1_serial_unloaded.sh
        scripts/run_robot1_camera_dynamic_capture.sh
        scripts/run_three_car_unloaded_pretest.sh
        scripts/run_three_car_unloaded_straight_pretest.sh
        scripts/start_three_car_chassis.sh
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
  add_rostest(test/three_car_unloaded_straight_pretest.test)
  add_rostest(test/experiment_recording.test)
endif()
