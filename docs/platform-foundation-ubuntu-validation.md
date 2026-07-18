# Platform Foundation Ubuntu/AGV Validation

The Task 0–7 source implementation was prepared on Windows by explicit user
authorization. The checks below are mandatory before algorithm migration or wheel
motion. A checked item must include the command output, host name and tested Git SHA.

## Ubuntu 20.04 / ROS Noetic build gate

- [ ] Record `hostname`, `git rev-parse HEAD`, `rosversion -d`,
      `rosversion roscpp`, `cmake --version` and `g++ --version`.
- [ ] Repair the flattened catkin toplevel file when necessary:
      `test -L src/CMakeLists.txt || catkin_init_workspace src --force`.
- [ ] Run `rosdep install --from-paths src --ignore-src -r -y`.
- [ ] Run `catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo` with zero errors.
- [ ] Run `catkin_make run_tests_agv_msgs run_tests_chassis_controller
      run_tests_multi_agv_bringup`.
- [ ] Run `catkin_test_results --verbose` with zero failures.

## Static launch and model gate

- [ ] `roslaunch-check multi_agv_bringup three_fake_chassis.launch`.
- [ ] `roslaunch-check multi_agv_bringup car1_master.launch`.
- [ ] `roslaunch-check multi_agv_bringup car2_client.launch`.
- [ ] `roslaunch-check multi_agv_bringup car3_client.launch`.
- [ ] `rosrun xacro xacro src/mycar_description/urdf/car.urdf.xacro
      prefix:=agv1/ | check_urdf /dev/stdin` and confirm prefixed links/joints.
- [ ] `bash -n src/multi_agv_bringup/scripts/setup_ros_network.sh`.

## Fake three-car runtime gate

- [ ] Start `roslaunch multi_agv_bringup three_fake_chassis.launch`.
- [ ] Confirm exactly one publisher on each `/agvX/chassis_command` while the test
      controller is running and no publishers on global `/cmd_vel` or `/odom`.
- [ ] Confirm each `/agvX/chassis_feedback` and `/agvX/capability_report` is near
      100 Hz, is non-latched and has the matching `robot_id`.
- [ ] Confirm each odom-to-base transform is near 100 Hz and every frozen TF edge
      has exactly one authority.
- [ ] Repeat the namespace, command-authority and TF-authority rostests three times.

## Raised-wheel hardware gate

- [ ] Source `setup_ros_network.sh` on each host and confirm all hosts use car1's
      `ROS_MASTER_URI`; verify `chronyc tracking` before recording.
- [ ] Check each serial device path and per-car IMU calibration.
- [ ] With wheels raised, verify SI-to-serial conversion (`m/s` to `mm/s`) and
      left/right wheel direction at low speed.
- [ ] Verify acceleration, deceleration, reversal-to-zero and final speed limiting.
- [ ] Apply and restore a car2 derating command; compare applied wheel speed and
      capability report against the configured ratios.
- [ ] Save rosbag and terminal output. Do not proceed to loaded motion if any item
      above fails.
