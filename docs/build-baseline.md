# ROS Noetic Build Baseline

## Source revision

- Branch: `feature/platform-foundation`
- Starting revision: `2c88c56`
- Platform inspected: Windows workspace used for source preparation

## Windows inspection (2026-07-18)

- `rosversion`: unavailable
- `catkin_make`: unavailable
- Docker/Podman: unavailable
- WSL executable: present, but no Linux distribution is installed
- Native CMake: STM32CubeCLT 3.31.6-compatible executable is present
- Native compiler: MinGW `g++` is present

The user explicitly authorized source implementation on Windows with ROS builds and
tests deferred to an Ubuntu 20.04/ROS Noetic host or the AGVs. Therefore this file
does not claim a passing ROS baseline. `src/CMakeLists.txt` remains untouched on
Windows because the repository contains a flattened Linux catkin toplevel target;
on Linux it must be repaired with `catkin_init_workspace src --force` if it is not a
symlink.

## Required Ubuntu validation

Run from the workspace root before deploying any package:

```bash
test -L src/CMakeLists.txt || catkin_init_workspace src --force
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin_make run_tests
catkin_test_results --verbose
```

Record `rosversion -d`, `rosversion roscpp`, CMake, compiler, Git SHA, unresolved
rosdep dependencies, compiler diagnostics and test results below when an Ubuntu
host is available. Until then, all ROS build and rostest gates are **PENDING**.

## Ubuntu result

- ROS distribution/version: **PENDING**
- roscpp version: **PENDING**
- CMake/compiler: **PENDING**
- rosdep result: **PENDING**
- baseline build: **PENDING**
- baseline tests: **PENDING**
