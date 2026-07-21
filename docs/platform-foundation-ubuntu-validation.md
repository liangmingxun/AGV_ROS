# Platform Foundation Ubuntu/AGV Validation

The Task 0–7 source implementation was prepared on Windows by explicit user
authorization. The checks below are mandatory before algorithm migration or wheel
motion. A checked item must include the command output, host name and tested Git SHA.

## Ubuntu 20.04 / ROS Noetic build gate

- [x] Record `hostname`, `git rev-parse HEAD`, `rosversion -d`,
      `rosversion roscpp`, `cmake --version` and `g++ --version`.
- [x] Confirm `src/CMakeLists.txt` is the committed catkin toplevel symlink.
      If an archive tool flattened it, move the regular file aside and run
      `catkin_init_workspace src`; this version of the command has no `--force`
      option.
- [ ] Run `rosdep install --from-paths src --ignore-src -r -y`.
- [x] Run `catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo` with zero errors.
- [x] Run `catkin_make run_tests_agv_msgs run_tests_chassis_controller
      run_tests_multi_agv_bringup`.
- [x] Run `catkin_test_results --verbose` with zero failures: 56 tests passed.

## Static launch and model gate

- [x] `roslaunch --files multi_agv_bringup three_fake_chassis.launch`.
- [x] `roslaunch --files multi_agv_bringup car1_master.launch`.
- [x] `roslaunch --files multi_agv_bringup car2_client.launch`.
- [x] `roslaunch --files multi_agv_bringup car3_client.launch`.
- [x] Generate `car.urdf.xacro prefix:=agv1/`, validate the XML with `xmllint`,
      and confirm the generated base and wheel links use the `agv1/` prefix.
- [x] `bash -n src/multi_agv_bringup/scripts/setup_ros_network.sh`.

## Fake three-car runtime gate

- [x] Start `roslaunch multi_agv_bringup three_fake_chassis.launch` through rostest.
- [x] Confirm exactly one publisher on each `/agvX/chassis_command` while the test
      controller is running and no publishers on global `/cmd_vel` or `/odom`.
- [x] Confirm each `/agvX/chassis_feedback` and `/agvX/capability_report` is near
      100 Hz, is non-latched and has the matching `robot_id`.
- [x] Confirm every frozen TF edge has exactly one authority; odom-to-base is
      emitted by the same verified 100 Hz state publication path.
- [x] Repeat the namespace, command-authority and TF-authority rostests three times.

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

## Scope boundary

The platform foundation corresponds to implementation Tasks 0–7 only. See
[`理论-软件-实物覆盖审计.md`](理论-软件-实物覆盖审计.md) for the exact theory,
software and physical-test coverage. A passing foundation test suite permits the
raised-wheel single-car gate; it does not approve loaded motion or formal experiments.
