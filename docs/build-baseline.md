# ROS Noetic Build Baseline

## Source revision

- Branch: `fix/platform-foundation-closeout`
- Platform foundation parent revision: `ff64e55`
- Ubuntu validation date: 2026-07-20
- Host: `robot1`

## Windows inspection (2026-07-18)

- `rosversion`: unavailable
- `catkin_make`: unavailable
- Docker/Podman: unavailable
- WSL executable: present, but no Linux distribution is installed
- Native CMake: STM32CubeCLT 3.31.6-compatible executable is present
- Native compiler: MinGW `g++` is present

The original source implementation was prepared on Windows and then closed out on
the Ubuntu ROS host recorded below. The flattened `src/CMakeLists.txt` has been
replaced by the committed catkin toplevel symlink. If an archive tool flattens the
symlink again, move the regular file aside before running `catkin_init_workspace
src`; ROS Noetic's command does not provide a `--force` option.

## Required Ubuntu validation

Run from the workspace root before deploying any package:

```bash
if [ ! -L src/CMakeLists.txt ]; then
  mv src/CMakeLists.txt src/CMakeLists.txt.flattened
  catkin_init_workspace src
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin_make run_tests_agv_msgs run_tests_chassis_controller \
  run_tests_multi_agv_bringup
catkin_test_results --verbose
```

## Ubuntu result

- ROS distribution: `noetic`
- roscpp version: `1.17.0`
- CMake: `3.16.3`
- Compiler: `g++ 9.4.0`
- Catkin: `0.8.11`
- `rosdep check --from-paths src --ignore-src`: one unresolved apt package,
  `qt5-image-formats-plugins`; it did not block compilation or the platform tests.
- Full workspace build: **PASS** (`RelWithDebInfo`, 24 packages).
- Platform tests: **PASS**, 42 tests, 0 errors, 0 failures, 0 skipped.
- Launch XML/package resolution: **PASS** for `three_fake_chassis.launch` and all
  three host launch files using `roslaunch --files`/`roslaunch --nodes`.
- Script syntax: **PASS** for `setup_ros_network.sh`.

The build still reports two pre-existing lslidar warnings for non-void functions
without a return value. They are outside the platform-foundation change set and
must not be mistaken for errors introduced by the multi-AGV refactor.

On a host whose shell exports a physical `ROS_HOSTNAME` that is not reachable from
the current test network, run isolated rostests with `ROS_HOSTNAME=127.0.0.1` and
`ROS_MASTER_URI=http://127.0.0.1:11311`. Real multi-host deployment must instead
use each host's reachable LAN address.
