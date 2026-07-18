# AGV_ROS 三移动机器人协同搬运 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不修改 STM32 固件的前提下，将现有 AGV_ROS 改造成 car1 集中计算、三车 ROS 命名空间通信、车端真实限幅与能力反馈、两阶段定位和可执行实验2a/实验1/实验3/实验2b的分层控制平台。

**Architecture:** 保留现有 `chassis_controller` 包和串口/QEKF代码，通过纯 C++ 核心类逐步拆出轮速限幅、降额和里程计，再把ROS节点迁移到冻结消息接口。car1使用一个 `multi_agv_controller_node`承载逻辑分层的上层、下层、能力映射和二维跟踪，避免多个高频节点之间产生不必要的调度抖动；状态估计和实验管理保持独立节点。所有算法先通过gtest性质测试和仿真黄金数据测试，再进入fake-transport rostest和实车测试。

**Tech Stack:** Ubuntu 20.04、ROS Noetic/ROS1、catkin、C++17、roscpp、tf2、Eigen3、gtest/rostest、Python 3、rosbag、NumPy/Pandas（仅离线分析）。

## Global Constraints

- 设计基线：`docs/实物实验设计_v1.2.1_接口冻结版.md`，接口字段、单位、时间语义和发布权不得自行改变。
- car1 是唯一 ROS Master；算法图结构保持原无向连通图，工程计算集中在 car1。
- ROS轮缘线速度单位为 `m/s`、轮缘线加减速度为 `m/s^2`；串口继续使用现有 `mm/s`；理论轮角速度由冻结轮半径换算。
- 车端执行顺序固定为：目标速度限幅 → 非对称加减速度斜坡 → 最终速度安全限幅 → STM32串口帧。
- 反向命令必须先按减速度上限降到零，后续周期再反向加速。
- 首轮频率：车端执行250 Hz，反馈/能力100 Hz，状态估计/中央下层/二维跟踪100 Hz，中央上层25 Hz，降额命令状态重发10 Hz。
- 控制、能力和反馈话题不得锁存；每台车同一时刻只允许一个 `ChassisCommand` 发布者。
- 不修改STM32协议；不加入电流、STM32本地时间、故障字；不实现独立通信看门狗。
- 正式实验不启动 `move_base` 速度输出；现有UDP只允许VOFA监视，不参与控制或能力链。
- 前期里程计仅用于流程预验证；顶部相机完成后才进行正式实验2a配对采集。
- 每个任务使用TDD：先看到目标测试失败，再写最小实现，再通过包级测试和全量回归，再提交。
- 所有ROS构建和rostest命令在装有ROS Noetic的Ubuntu开发机或AGV上执行；Windows工作区只用于代码审阅和文档维护。

---

## Target File Structure

```text
src/
├─ agv_msgs/
│  ├─ msg/{ChassisCommand,ChassisFeedback,CapabilityReport,DeratingCommand,
│  │       CooperativeState,PathReference,ControllerState,ExperimentState}.msg
│  ├─ CMakeLists.txt
│  └─ package.xml
├─ chassis_controller/
│  ├─ include/chassis_controller/
│  │  ├─ wheel_command_limiter.hpp
│  │  ├─ derating_profile.hpp
│  │  ├─ odometry_integrator.hpp
│  │  └─ chassis_core.hpp
│  ├─ src/{wheel_command_limiter,derating_profile,odometry_integrator,chassis_core}.cpp
│  ├─ src/main.cpp
│  ├─ test/{test_wheel_command_limiter,test_derating_profile,
│  │        test_odometry_integrator,test_chassis_core}.cpp
│  └─ test/chassis_node.test
├─ multi_agv_control/
│  ├─ include/multi_agv_control/
│  │  ├─ s_curve_path.hpp
│  │  ├─ path_projector.hpp
│  │  ├─ support_geometry.hpp
│  │  ├─ state_estimator.hpp
│  │  ├─ capability_mapper.hpp
│  │  ├─ dynamic_boundary.hpp
│  │  ├─ upper_reference_generator.hpp
│  │  ├─ lower_channel_controller.hpp
│  │  ├─ planar_support_tracker.hpp
│  │  └─ experiment_supervisor.hpp
│  ├─ src/*_node.cpp and matching core `.cpp` files
│  ├─ test/*.cpp
│  ├─ test/*.test
│  ├─ test/reference/export_golden.py
│  ├─ test/data/{m1,m2b,lower_m1}_golden.csv
│  ├─ CMakeLists.txt
│  └─ package.xml
├─ multi_agv_bringup/
│  ├─ config/*.yaml
│  ├─ launch/{car1_master,car2_client,car3_client,three_fake_chassis,
│  │          odom_pretest,camera_formal,experiment}.launch
│  ├─ test/{namespace_isolation,tf_authority,command_authority}.test
│  ├─ scripts/setup_ros_network.sh
│  ├─ CMakeLists.txt
│  └─ package.xml
└─ multi_agv_analysis/
   ├─ scripts/{bag_to_csv,validate_run,compute_metrics}.py
   ├─ test/test_metrics.py
   ├─ CMakeLists.txt
   └─ package.xml
```

---

### Task 0: Establish a reproducible ROS Noetic baseline

**Files:**
- Modify only if flattened on the Linux host: `src/CMakeLists.txt`
- Create: `docs/build-baseline.md`

**Interfaces:**
- Consumes: clean commit `2ac2111` and an Ubuntu20.04/ROS Noetic machine.
- Produces: recorded baseline build result and a valid catkin workspace before feature changes.

- [ ] **Step 1: Verify the catkin toplevel file and dependencies**

```bash
cd ~/AGV_ROS
test -L src/CMakeLists.txt || catkin_init_workspace src --force
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Expected: `src/CMakeLists.txt` resolves to `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake`; rosdep reports all resolvable dependencies installed. Record any third-party package that rosdep cannot resolve in `docs/build-baseline.md` with the exact package and command output.

- [ ] **Step 2: Run the untouched baseline build**

```bash
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Expected: either PASS or a captured list of pre-existing compiler errors. Do not mix unrelated baseline repairs into Task1; if a build repair is required, commit only that repair as `build: restore ROS Noetic baseline` and rerun until the baseline builds.

- [ ] **Step 3: Record versions**

```bash
rosversion -d
rosversion roscpp
cmake --version
g++ --version
git rev-parse HEAD
```

Write the exact output and build command to `docs/build-baseline.md`.

- [ ] **Step 4: Commit the baseline record**

```bash
git add docs/build-baseline.md src/CMakeLists.txt
git commit -m "docs: record ROS Noetic build baseline"
```

---

### Task 1: Create and compile the frozen `agv_msgs` interfaces

**Files:**
- Create: `src/agv_msgs/msg/ChassisCommand.msg`
- Create: `src/agv_msgs/msg/ChassisFeedback.msg`
- Create: `src/agv_msgs/msg/CapabilityReport.msg`
- Create: `src/agv_msgs/msg/DeratingCommand.msg`
- Create: `src/agv_msgs/msg/CooperativeState.msg`
- Create: `src/agv_msgs/msg/PathReference.msg`
- Create: `src/agv_msgs/msg/ControllerState.msg`
- Create: `src/agv_msgs/msg/ExperimentState.msg`
- Create: `src/agv_msgs/CMakeLists.txt`
- Create: `src/agv_msgs/package.xml`
- Test: generated message metadata via `rosmsg show`

**Interfaces:**
- Consumes: ROS `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`.
- Produces: all cross-package message types used by every later task.

- [ ] **Step 1: Add a build check that currently fails**

Run:

```bash
cd ~/AGV_ROS
catkin_make --pkg agv_msgs
```

Expected: FAIL because package `agv_msgs` does not exist.

- [ ] **Step 2: Create the eight frozen message files**

Use exactly the fields from sections 3.1, 5.3, 5.4, 6.3, 6.5 and 21.0.4 of the frozen design. For example, `ChassisCommand.msg` must be:

```text
std_msgs/Header header
uint8 robot_id
uint32 command_seq
uint8 control_mode
float64 linear_velocity_reference
float64 angular_velocity_reference
float64 wheel_linear_velocity_left_raw
float64 wheel_linear_velocity_right_raw
string experiment_id
string method_id
```

`CooperativeState.msg` must use valid ROS1 fixed-array syntax:

```text
std_msgs/Header header
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_ODOM=1
uint8 SOURCE_CAMERA=2
uint8 SOURCE_FUSED=3
uint8[3] robot_localization_source
bool[3] robot_pose_valid
time[3] robot_pose_stamp
geometry_msgs/Pose2D[3] robot_pose
bool[3] support_pose_valid
geometry_msgs/Pose2D[3] support_pose
bool[3] path_state_valid
float64[3] s_actual
float64[3] s_dot_actual
uint8 load_localization_source
bool load_pose_valid
time load_pose_stamp
geometry_msgs/Pose2D load_pose
bool load_path_state_valid
float64 load_s_actual
float64 load_s_dot_actual
```

The remaining frozen files are:

`DeratingCommand.msg`:

```text
std_msgs/Header header
uint8 robot_id
uint32 command_seq
uint8 mode
bool active
float64 target_speed_ratio_left
float64 target_speed_ratio_right
float64 target_accel_ratio_left
float64 target_accel_ratio_right
float64 target_decel_ratio_left
float64 target_decel_ratio_right
float64 ramp_down_time
float64 ramp_up_time
string experiment_id
```

`CapabilityReport.msg`:

```text
std_msgs/Header header
uint8 robot_id
uint32 capability_seq
float64 max_wheel_linear_velocity_left
float64 max_wheel_linear_velocity_right
float64 max_wheel_linear_acceleration_left
float64 max_wheel_linear_acceleration_right
float64 max_wheel_linear_deceleration_left
float64 max_wheel_linear_deceleration_right
float64 derating_ratio
uint8 derating_mode
bool derating_active
bool speed_limit_active_left
bool speed_limit_active_right
bool accel_limit_active_left
bool accel_limit_active_right
bool decel_limit_active_left
bool decel_limit_active_right
float64 battery_voltage
```

`ChassisFeedback.msg`:

```text
std_msgs/Header header
uint8 robot_id
uint32 feedback_seq
uint32 command_seq_applied
uint32 packet_seq
time serial_receive_stamp
float64 wheel_linear_velocity_left_raw
float64 wheel_linear_velocity_right_raw
float64 wheel_linear_velocity_left_applied
float64 wheel_linear_velocity_right_applied
float64 wheel_linear_velocity_left_actual
float64 wheel_linear_velocity_right_actual
float64 linear_velocity_actual
float64 angular_velocity_actual
sensor_msgs/Imu imu
nav_msgs/Odometry odom
float64 battery_voltage
bool control_loop_overrun
bool speed_limit_active_left
bool speed_limit_active_right
bool accel_limit_active_left
bool accel_limit_active_right
bool decel_limit_active_left
bool decel_limit_active_right
```

`PathReference.msg`:

```text
std_msgs/Header header
string path_id
uint32 path_version
float64 load_path_progress_reference
float64 load_path_velocity_reference
float64 load_path_acceleration_reference
geometry_msgs/Pose2D load_pose_reference
geometry_msgs/Pose2D[3] support_pose_reference
float64[3] chassis_linear_velocity_feedforward
float64[3] chassis_angular_velocity_feedforward
```

`ControllerState.msg`:

```text
std_msgs/Header header
string experiment_id
string method_id
float64[3] path_progress_actual
float64[3] path_velocity_actual
float64[3] path_progress_execute_reference
float64[3] path_velocity_execute_reference
float64[3] channel_input_raw
float64[3] channel_input_limited
float64 common_velocity_lower_bound
float64 common_velocity_upper_bound
float64 common_load_velocity_reference
bool[3] channel_input_limit_active
```

`ExperimentState.msg`:

```text
std_msgs/Header header
string experiment_id
string method_id
string block_id
string run_id
uint8 phase
bool run_active
bool evaluation_active
bool manual_abort
string abort_reason
```

- [ ] **Step 3: Configure message generation**

`src/agv_msgs/CMakeLists.txt` must call:

```cmake
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs message_generation nav_msgs sensor_msgs std_msgs)
add_message_files(FILES
  ChassisCommand.msg ChassisFeedback.msg CapabilityReport.msg
  DeratingCommand.msg CooperativeState.msg PathReference.msg
  ControllerState.msg ExperimentState.msg)
generate_messages(DEPENDENCIES geometry_msgs nav_msgs sensor_msgs std_msgs)
catkin_package(CATKIN_DEPENDS
  geometry_msgs message_runtime nav_msgs sensor_msgs std_msgs)
```

`package.xml` must declare matching build, build-export and exec dependencies without depending on `chassis_controller` or `multi_agv_control`.

- [ ] **Step 4: Build and inspect generated types**

Run:

```bash
catkin_make --pkg agv_msgs
source devel/setup.bash
rosmsg show agv_msgs/CooperativeState
rosmsg show agv_msgs/ChassisFeedback
```

Expected: build succeeds; arrays display as fixed length 3; wheel fields contain `wheel_linear_velocity`, not ambiguous `wheel_speed`.

- [ ] **Step 5: Commit**

```bash
git add src/agv_msgs
git commit -m "feat: add frozen multi-AGV ROS messages"
```

---

### Task 2: Implement the pure wheel command limiter

**Files:**
- Create: `src/chassis_controller/include/chassis_controller/wheel_command_limiter.hpp`
- Create: `src/chassis_controller/src/wheel_command_limiter.cpp`
- Create: `src/chassis_controller/test/test_wheel_command_limiter.cpp`
- Modify: `src/chassis_controller/CMakeLists.txt`
- Modify: `src/chassis_controller/package.xml`

**Interfaces:**
- Consumes: raw wheel-linear velocity, current limits and measured `dt`.
- Produces: `WheelLimitResult WheelCommandLimiter::update(const WheelCommand&, const WheelLimits&, double)`.

- [ ] **Step 1: Write failing gtests for all frozen limiter rules**

```cpp
TEST(WheelCommandLimiter, FinalClampHonorsNewLowerLimit) {
  WheelCommandLimiter limiter({0.8, 0.8});
  WheelLimits limits{0.5, 0.5, 1.0, 1.0, 0.2, 0.2};
  const auto out = limiter.update({0.8, 0.8}, limits, 0.01);
  EXPECT_DOUBLE_EQ(out.applied.left, 0.5);
  EXPECT_TRUE(out.speed_limited_left);
}

TEST(WheelCommandLimiter, ReversalDeceleratesToZeroFirst) {
  WheelCommandLimiter limiter({0.4, 0.4});
  WheelLimits limits{0.9, 0.9, 1.0, 1.0, 2.0, 2.0};
  const auto out = limiter.update({-0.4, -0.4}, limits, 0.1);
  EXPECT_NEAR(out.applied.left, 0.2, 1e-12);
  EXPECT_GT(out.applied.left, 0.0);
}
```

Also test acceleration increase, deceleration decrease, left/right independence, `dt<=0`, NaN rejection and final magnitude clipping.

- [ ] **Step 2: Run the test and confirm failure**

```bash
catkin_make run_tests_chassis_controller_gtest_test_wheel_command_limiter
catkin_test_results --verbose
```

Expected: FAIL because the class is absent.

- [ ] **Step 3: Implement the exact public types**

```cpp
struct WheelCommand { double left; double right; };
struct WheelLimits {
  double max_velocity_left, max_velocity_right;
  double max_acceleration_left, max_acceleration_right;
  double max_deceleration_left, max_deceleration_right;
};
struct WheelLimitResult {
  WheelCommand target;
  WheelCommand applied;
  bool speed_limited_left, speed_limited_right;
  bool accel_limited_left, accel_limited_right;
  bool decel_limited_left, decel_limited_right;
};
```

Implement per wheel as: finite-input validation, target magnitude clamp, same-sign acceleration/deceleration selection, reversal-to-zero handling, final magnitude clamp. Keep all values in `m/s` and `m/s^2`.

- [ ] **Step 4: Run package tests**

```bash
catkin_make run_tests_chassis_controller
catkin_test_results --verbose
```

Expected: all limiter tests pass with zero failures.

- [ ] **Step 5: Commit**

```bash
git add src/chassis_controller
git commit -m "feat: add deterministic wheel command limiter"
```

---

### Task 3: Implement smooth derating as a pure state machine

**Files:**
- Create: `src/chassis_controller/include/chassis_controller/derating_profile.hpp`
- Create: `src/chassis_controller/src/derating_profile.cpp`
- Create: `src/chassis_controller/test/test_derating_profile.cpp`
- Modify: `src/chassis_controller/CMakeLists.txt`

**Interfaces:**
- Consumes: `DeratingTarget` derived from `DeratingCommand` and measured `dt`.
- Produces: six current limit ratios and a monotonic local state revision.

- [ ] **Step 1: Write failing tests**

```cpp
TEST(DeratingProfile, ReachesTargetWithoutJump) {
  DeratingProfile profile;
  DeratingRatios reduced{0.7, 0.7, 0.72, 0.72, 0.72, 0.72};
  ASSERT_TRUE(profile.accept(1, reduced, 1.0));
  EXPECT_NEAR(profile.update(0.1).speed_left, 0.97, 1e-9);
  for (int i = 0; i < 9; ++i) profile.update(0.1);
  EXPECT_NEAR(profile.current().speed_left, 0.7, 1e-9);
}

TEST(DeratingProfile, DuplicateSequenceIsIdempotent) {
  DeratingProfile profile;
  DeratingRatios nominal{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  DeratingRatios reduced{0.7, 0.7, 0.72, 0.72, 0.72, 0.72};
  EXPECT_TRUE(profile.accept(10, nominal, 1.0));
  EXPECT_FALSE(profile.accept(10, reduced, 1.0));
}
```

Also test restoration ramp, ratio bounds `(0,1]`, out-of-order sequence rejection and asymmetric left/right targets.

- [ ] **Step 2: Run and verify failure**

```bash
catkin_make run_tests_chassis_controller_gtest_test_derating_profile
catkin_test_results --verbose
```

- [ ] **Step 3: Implement**

Define `DeratingRatios`, `DeratingTarget` and `DeratingProfile::accept(uint32_t, ...)`, `update(double)`, `current()`. Interpolate from the value at command acceptance to the target using elapsed/ramp time clamped to `[0,1]`; do not interpolate from the nominal value after every 10 Hz repeated command.

- [ ] **Step 4: Run tests and commit**

```bash
catkin_make run_tests_chassis_controller
catkin_test_results --verbose
git add src/chassis_controller
git commit -m "feat: add smooth local derating profile"
```

---

### Task 4: Isolate and test odometry integration

**Files:**
- Create: `src/chassis_controller/include/chassis_controller/odometry_integrator.hpp`
- Create: `src/chassis_controller/src/odometry_integrator.cpp`
- Create: `src/chassis_controller/test/test_odometry_integrator.cpp`
- Modify: `src/chassis_controller/CMakeLists.txt`

**Interfaces:**
- Consumes: actual left/right wheel-linear velocity, filtered IMU yaw rate and `dt`.
- Produces: continuous local `odom` pose and reset operation.

- [ ] **Step 1: Write failing tests**

```cpp
TEST(OdometryIntegrator, IntegratesStraightMotion) {
  OdometryIntegrator odom;
  odom.update(0.2, 0.2, 0.0, 1.0);
  EXPECT_NEAR(odom.state().x, 0.2, 1e-12);
  EXPECT_NEAR(odom.state().y, 0.0, 1e-12);
}

TEST(OdometryIntegrator, ResetUsesRequestedPose) {
  OdometryIntegrator odom;
  odom.reset({1.0, 2.0, 0.5});
  EXPECT_NEAR(odom.state().yaw, 0.5, 1e-12);
}
```

Add turn, yaw wrap, invalid `dt` and zero-motion tests.

- [ ] **Step 2: Implement midpoint integration**

Use average actual wheel-linear velocity for `v`, IMU yaw rate for `w`, and midpoint heading `yaw + 0.5*w*dt` for position. Normalize yaw to `[-pi,pi]` and expose `reset(const Pose2DState&)`.

- [ ] **Step 3: Run and commit**

```bash
catkin_make run_tests_chassis_controller
catkin_test_results --verbose
git add src/chassis_controller
git commit -m "refactor: isolate tested odometry integration"
```

---

### Task 5: Refactor `chassis_controller` onto frozen messages and remove multi-car logic

**Files:**
- Create: `src/chassis_controller/include/chassis_controller/chassis_core.hpp`
- Create: `src/chassis_controller/src/chassis_core.cpp`
- Create: `src/chassis_controller/test/test_chassis_core.cpp`
- Modify: `src/chassis_controller/src/main.cpp`
- Modify: `src/chassis_controller/src/yaml_parameters.hpp`
- Modify: `src/chassis_controller/include/chassis_driver/config.h`
- Modify: `src/chassis_controller/CMakeLists.txt`
- Modify: `src/chassis_controller/package.xml`
- Delete after migration: `src/chassis_controller/msg/ControlCommand.msg`
- Delete after migration: `src/chassis_controller/msg/ImuData.msg`
- Delete after migration: `src/chassis_controller/msg/RobotState.msg`

**Interfaces:**
- Consumes: `agv_msgs/ChassisCommand`, `agv_msgs/DeratingCommand`, current sensor frame.
- Produces: limited wheel commands, `ChassisFeedback`, `CapabilityReport`, odom and IMU.

- [ ] **Step 1: Write core tests before touching `main.cpp`**

```cpp
TEST(ChassisCore, LinksRawAppliedAndCapability) {
  ChassisCore core(testConfig());
  core.acceptCommand(command(7, 0.8, 0.8));
  core.step(0.01);
  EXPECT_EQ(core.feedback().command_seq_applied, 7u);
  EXPECT_LE(std::abs(core.feedback().applied.left),
            core.capability().max_velocity_left);
}
```

Test no cross-robot state, command sequence monotonicity, derating-to-capability consistency, actual feedback unit conversion `mm/s→m/s`, and `control_loop_overrun` when measured `dt` exceeds the configured threshold.

- [ ] **Step 2: Implement `ChassisCore` without ROS or serial dependencies**

Expose:

```cpp
class ChassisCore {
 public:
  bool acceptCommand(const CommandInput&);
  bool acceptDerating(const DeratingInput&);
  WheelCommand step(double dt_seconds);
  void updateSensors(const SensorInput&);
  const FeedbackState& feedback() const;
  const CapabilityState& capability() const;
};
```

The core owns `WheelCommandLimiter`, `DeratingProfile` and `OdometryIntegrator`; it never subscribes to another robot.

- [ ] **Step 3: Replace identity inference and global topics**

Load `robot_id`, `robot_index`, `serial_device`, frame IDs and nominal limits directly from the private node namespace. Remove IP-last-octet identity mapping. Subscribe to private `chassis_command` and `derating_command`; publish private `chassis_feedback`, `capability_report`, `odom`, `imu`.

- [ ] **Step 4: Remove inactive multi-car and UDP receive code**

Delete `SwarmData`, `AlgorithmCommand`, `swarmCommsAlgorithmThread`, `udpReceiverThread`, other-robot subscriptions and `/agvX/state`. Keep VOFA send only behind private parameter `enable_vofa:=false`; it must not affect control when disabled.

- [ ] **Step 5: Add fake transport mode for rostest**

Add private parameter `transport_type` with values `serial` and `fake`. Fake mode echoes applied wheel commands as actual feedback after one 250 Hz step and never opens a device file. Production default remains `serial`.

- [ ] **Step 6: Run unit and node smoke tests**

```bash
catkin_make --pkg agv_msgs chassis_controller
catkin_make run_tests_chassis_controller
catkin_test_results --verbose
rosrun chassis_controller chassis_controller _transport_type:=fake _robot_id:=agv1
```

Expected: no serial error in fake mode; only namespaced topics appear when launched inside `/agv1`.

- [ ] **Step 7: Commit**

```bash
git add src/chassis_controller
git commit -m "refactor: make chassis controller single-robot executor"
```

---

### Task 6: Add three-host bringup, explicit frames and per-car configuration

**Files:**
- Create: `src/multi_agv_bringup/package.xml`
- Create: `src/multi_agv_bringup/CMakeLists.txt`
- Create: `src/multi_agv_bringup/config/common_platform.yaml`
- Create: `src/multi_agv_bringup/config/agv{1,2,3}_chassis.yaml`
- Create: `src/multi_agv_bringup/launch/chassis_single.launch`
- Create: `src/multi_agv_bringup/launch/car{1_master,2_client,3_client}.launch`
- Create: `src/multi_agv_bringup/launch/three_fake_chassis.launch`
- Create: `src/multi_agv_bringup/scripts/setup_ros_network.sh`
- Modify: `src/mycar_description/urdf/car.urdf.xacro`
- Modify: child xacros under `src/mycar_description/urdf/`

**Interfaces:**
- Consumes: Task 5 node parameters.
- Produces: reproducible launches with explicit `agvX/*` names and one TF authority per edge.

- [ ] **Step 1: Add a failing launch check**

```bash
roslaunch-check multi_agv_bringup three_fake_chassis.launch
```

Expected: FAIL because the package is absent.

- [ ] **Step 2: Create explicit per-car YAML**

Each file must contain `robot_id`, `robot_index`, `serial_device`, `wheel_radius`, `wheel_separation`, nominal left/right velocity/acceleration/deceleration, `odom_frame`, `base_frame`, `imu_frame`, `support_frame`, `transport_type` and `enable_vofa`. Use the existing calibrated IMU values and current serial device paths; do not preserve the inconsistent IP-to-ID mapping.

- [ ] **Step 3: Create launch files**

`chassis_single.launch` must wrap the node in `<group ns="$(arg robot_id)">`, load only that car's YAML, publish explicit static `base_link→imu_link` and `base_link→support_link`, and never start `move_base`. `three_fake_chassis.launch` includes it three times with `transport_type:=fake`.

- [ ] **Step 4: Prefix URDF frame and joint names explicitly**

Add xacro argument `prefix` and apply it to all links/joints. Verify generated models contain `agv1/base_link`, not three copies of `base_link`.

- [ ] **Step 5: Validate launch and TF names**

```bash
catkin_make --pkg multi_agv_bringup mycar_description
source devel/setup.bash
roslaunch-check multi_agv_bringup three_fake_chassis.launch
roslaunch multi_agv_bringup three_fake_chassis.launch
rosrun tf view_frames
```

Expected: three disjoint `agvX/odom→agvX/base_link→agvX/{imu_link,support_link}` branches and no global `/odom`, `/cmd_vel`, `base_link` or `imu_link` publisher.

- [ ] **Step 6: Commit**

```bash
git add src/multi_agv_bringup src/mycar_description
git commit -m "feat: add namespaced three-host bringup"
```

---

### Task 7: Add namespace, command-authority and fake-chassis integration rostests

**Files:**
- Create: `src/multi_agv_bringup/test/namespace_isolation.test`
- Create: `src/multi_agv_bringup/test/test_namespace_isolation.py`
- Create: `src/multi_agv_bringup/test/command_authority.test`
- Create: `src/multi_agv_bringup/test/test_command_authority.py`
- Create: `src/multi_agv_bringup/test/tf_authority.test`
- Create: `src/multi_agv_bringup/test/test_tf_authority.py`
- Modify: `src/multi_agv_bringup/CMakeLists.txt`
- Modify: `src/multi_agv_bringup/package.xml`

**Interfaces:**
- Consumes: three fake chassis nodes.
- Produces: automated proof of topic/TF isolation before real hardware use.

- [ ] **Step 1: Write failing rostests**

The test publishes `0.1`, `0.2`, `0.3 m/s` to the three command topics and asserts each feedback matches only its own command. Query ROS master system state and assert exactly one publisher for each `chassis_command`, no publisher for `/cmd_vel` or `/odom`, and one authority for every required TF edge.

- [ ] **Step 2: Run and observe failures**

```bash
catkin_make run_tests_multi_agv_bringup
catkin_test_results --verbose
```

- [ ] **Step 3: Correct launch/topic defects until all pass**

Do not weaken assertions. Queue size for commands is1 and none of the control/capability/feedback publishers is latched.

- [ ] **Step 4: Commit**

```bash
git add src/multi_agv_bringup
git commit -m "test: verify three-car namespace and authority isolation"
```

---

### Task 8: Implement and test the arc-length S-curve and support geometry

**Files:**
- Create: `src/multi_agv_control/package.xml`
- Create: `src/multi_agv_control/CMakeLists.txt`
- Create: `src/multi_agv_control/include/multi_agv_control/s_curve_path.hpp`
- Create: `src/multi_agv_control/src/s_curve_path.cpp`
- Create: `src/multi_agv_control/include/multi_agv_control/support_geometry.hpp`
- Create: `src/multi_agv_control/src/support_geometry.cpp`
- Create: `src/multi_agv_control/test/test_s_curve_path.cpp`
- Create: `src/multi_agv_control/test/test_support_geometry.cpp`
- Create: `src/multi_agv_bringup/config/path_s_curve.yaml`
- Create: `src/multi_agv_bringup/config/support_geometry.yaml`

**Interfaces:**
- Produces: `PathSample SCurvePath::sample(double s)` and `SupportSample SupportGeometry::sample(size_t robot, double s)`.

- [ ] **Step 1: Write mathematical property tests**

```cpp
TEST(SCurvePath, IsArcLengthParameterized) {
  SCurvePath path(testPathConfig());
  for (double s = 0.0; s <= path.length(); s += 0.01)
    EXPECT_NEAR(path.sample(s).tangent.norm(), 1.0, 2e-3);
}

TEST(SupportGeometry, IsNonDegenerate) {
  SupportGeometry geometry(testPath(), testOffsets());
  EXPECT_GT(geometry.minimumSpeedScale(), 0.0);
  EXPECT_LT(geometry.maxCurvatureNormalOffsetProduct(), 1.0);
}
```

Also test endpoints, monotonic inverse arc-length lookup, continuous heading, finite curvature and numerical derivative agreement for `p'`/`p''`.

- [ ] **Step 2: Implement numerical arc-length table**

Sample raw `x=xi`, `y=A*sin(2*pi*xi/L_xi)` densely at startup, integrate chord/Simpson arc length, build monotonic `s↔xi` lookup, and interpolate. Reject configs that violate `1-kappa*q_n>=chi_min` or `g_i>=g_min`.

- [ ] **Step 3: Run and commit**

```bash
catkin_make run_tests_multi_agv_control
catkin_test_results --verbose
git add src/multi_agv_control src/multi_agv_bringup/config
git commit -m "feat: add arc-length S-curve and support geometry"
```

---

### Task 9: Implement local-window projection and unified odometry state

**Files:**
- Create: `src/multi_agv_control/include/multi_agv_control/path_projector.hpp`
- Create: `src/multi_agv_control/src/path_projector.cpp`
- Create: `src/multi_agv_control/include/multi_agv_control/state_estimator.hpp`
- Create: `src/multi_agv_control/src/state_estimator.cpp`
- Create: `src/multi_agv_control/src/path_state_estimator_node.cpp`
- Create: `src/multi_agv_control/test/test_path_projector.cpp`
- Create: `src/multi_agv_control/test/test_state_estimator.cpp`
- Create: `src/multi_agv_control/test/odom_state_estimator.test`
- Create: `src/multi_agv_bringup/config/localization_odom.yaml`

**Interfaces:**
- Consumes: three `/agvX/odom`, support offsets and world-to-odom initial transforms.
- Produces: `/multi_agv/cooperative_state` at100 Hz.

- [ ] **Step 1: Write projection tests**

Test an exact point, noisy normal offset, start/end clamp and an S-curve branch case where global nearest-point jumps but the local search window remains monotonic.

- [ ] **Step 2: Implement projector**

Expose `ProjectionResult project(const Eigen::Vector2d&, double previous_s, double half_window)`. Coarse-scan only the local window, refine the best interval with bounded Newton/golden-section iterations, and return `valid=false` when residual or convergence limits fail.

- [ ] **Step 3: Implement causal speed estimation**

`StateEstimator` stores previous `s`, measurement stamp and filtered speed per robot. Compute finite difference only for increasing stamps and valid projections, then apply the frozen one-pole filter; publish per-robot validity and original measurement stamps.

- [ ] **Step 4: Add odometry rostest**

Feed synthetic namespaced odometry for three different trajectories. Assert `CooperativeState` has source `SOURCE_ODOM`, three independent valid flags, monotonic `s_actual`, no use of `s_ref`, and load pose marked as estimated/pretest.

- [ ] **Step 5: Run and commit**

```bash
catkin_make run_tests_multi_agv_control
catkin_test_results --verbose
git add src/multi_agv_control src/multi_agv_bringup/config/localization_odom.yaml
git commit -m "feat: add unified odometry path state estimator"
```

---

### Task 10: Implement capability mapping from wheel-linear limits to path limits

**Files:**
- Create: `src/multi_agv_control/include/multi_agv_control/capability_mapper.hpp`
- Create: `src/multi_agv_control/src/capability_mapper.cpp`
- Create: `src/multi_agv_control/test/test_capability_mapper.cpp`
- Create: `src/multi_agv_bringup/config/capability_mapping.yaml`

**Interfaces:**
- Consumes: three `CapabilityReport`, path/support geometry and tracking reserves.
- Produces: per-car `v^-`, `v^+`, `a_-`, `a_+`, `v_act` and public minimum.

- [ ] **Step 1: Write failing equation tests**

Use a straight path case with `g=1`, `h=0`, equal wheel limits and zero reserve; expected path speed equals the reported wheel-linear limit. Add curved support-offset cases, asymmetric wheel limits, nonnegative clamp, minimum-across-three and “acceleration not double deducted” tests.

- [ ] **Step 2: Implement typed result**

```cpp
struct PathCapability {
  double lower_velocity, upper_velocity;
  double available_acceleration, available_deceleration;
  double actuator_upper_velocity;
  ros::Time source_stamp;
  uint32_t source_sequence;
  bool valid;
};
```

Convert wheel-linear limits to theory values with frozen wheel radius, apply geometry and reserves exactly once, and retain source stamp/sequence for logging.

- [ ] **Step 3: Run and commit**

```bash
catkin_make run_tests_multi_agv_control
catkin_test_results --verbose
git add src/multi_agv_control src/multi_agv_bringup/config/capability_mapping.yaml
git commit -m "feat: map reported wheel limits to path capability"
```

---

### Task 11: Implement the planar support tracker and constant-reference pretest mode

**Files:**
- Create: `src/multi_agv_control/include/multi_agv_control/planar_support_tracker.hpp`
- Create: `src/multi_agv_control/src/planar_support_tracker.cpp`
- Create: `src/multi_agv_control/test/test_planar_support_tracker.cpp`
- Create: `src/multi_agv_control/src/multi_agv_controller_node.cpp`
- Create: `src/multi_agv_bringup/config/pretest_constant_reference.yaml`
- Create: `src/multi_agv_bringup/launch/odom_pretest.launch`
- Create: `src/multi_agv_control/test/constant_reference_e2e.test`

**Interfaces:**
- Consumes: `CooperativeState`, `s_L_ref`, `v_L_ref`, S-curve/support geometry.
- Produces: three `ChassisCommand`, `PathReference`, `ControllerState`.

- [ ] **Step 1: Write tracker tests**

Test zero pose error returns analytic feedforward, positive body-frame longitudinal error increases linear command, wrapped heading error remains continuous across ±pi, left/right conversion matches wheel separation, and all three references use the same `s_L_ref`.

- [ ] **Step 2: Implement tracker**

Use the frozen equations for `p_i_ref`, `theta_i_ref`, `v_i_ref_2D`, `omega_i_ref_2D`, body-frame error and feedback gains. Output raw wheel-linear velocity in `m/s`; do not apply physical wheel limits in this class.

- [ ] **Step 3: Implement constant-reference node mode**

The node runs at100 Hz, advances `s_L_ref` from a configured constant speed, calls the tracker, publishes one command per car and publishes reference/controller state. Upper updates are disabled in this mode.

- [ ] **Step 4: Run fake three-car end-to-end test**

```bash
rostest multi_agv_control constant_reference_e2e.test
```

Expected: commands are distinct when support geometry requires it; all feedback sequences advance; no global command topics; raw→applied→actual fields are traceable.

- [ ] **Step 5: Commit**

```bash
git add src/multi_agv_control src/multi_agv_bringup
git commit -m "feat: add common-load planar tracking pretest"
```

---

### Task 12: Implement position-triggered car2 derating and verify the causal chain

**Files:**
- Create: `src/multi_agv_control/include/multi_agv_control/experiment_supervisor.hpp`
- Create: `src/multi_agv_control/src/experiment_supervisor.cpp`
- Create: `src/multi_agv_control/src/experiment_supervisor_node.cpp`
- Create: `src/multi_agv_control/test/test_experiment_supervisor.cpp`
- Create: `src/multi_agv_control/test/derating_chain_e2e.test`
- Create: `src/multi_agv_bringup/config/exp2a_derating_pretest.yaml`

**Interfaces:**
- Consumes: actual `s_2` or actual load progress from `CooperativeState`.
- Produces: idempotent `/agv2/derating_command` at10 Hz and `ExperimentState`.

- [ ] **Step 1: Write trigger tests**

Verify no trigger from reference progress, activation exactly on crossing `s_a`, no repeated sequence increment while state is unchanged, restoration on crossing `s_b`, and run reset only while inactive.

- [ ] **Step 2: Implement supervisor state machine**

Use phases `IDLE`, `ARMED`, `RUNNING_NOMINAL`, `DERATING_DOWN`, `DERATED`, `RESTORING`, `FINISHED`, `ABORTED`. A 10 Hz republish retains the same sequence and target. Only a target/mode transition increments sequence.

- [ ] **Step 3: Add end-to-end test**

Drive synthetic actual progress through the degradation window. Assert car2 capability falls smoothly, car1/car3 capability remains nominal, applied car2 command never exceeds the reported current limit, and timestamps allow reconstruction of command→capability→applied delay.

- [ ] **Step 4: Run and commit**

```bash
catkin_make run_tests_multi_agv_control
catkin_test_results --verbose
git add src/multi_agv_control src/multi_agv_bringup/config/exp2a_derating_pretest.yaml
git commit -m "feat: add causal car2 derating experiment chain"
```

---

### Task 13: Implement M1, M2a and M4 upper modes with projection invariants

**Files:**
- Create: `src/multi_agv_control/include/multi_agv_control/dynamic_boundary.hpp`
- Create: `src/multi_agv_control/src/dynamic_boundary.cpp`
- Create: `src/multi_agv_control/include/multi_agv_control/upper_reference_generator.hpp`
- Create: `src/multi_agv_control/src/upper_reference_generator.cpp`
- Create: `src/multi_agv_control/test/test_dynamic_boundary.cpp`
- Create: `src/multi_agv_control/test/test_upper_modes.cpp`
- Create: `src/multi_agv_control/test/reference/export_golden.py`
- Create: `src/multi_agv_control/test/data/m1_golden.csv`
- Create: `src/multi_agv_control/test/data/reference_hashes.txt`
- Create: `src/multi_agv_bringup/config/exp2a_{M1,M2a,M4}.yaml`
- Modify: `src/multi_agv_control/src/multi_agv_controller_node.cpp`

**Interfaces:**
- Consumes: path capabilities, causal risk inputs and virtual-agent graph state.
- Produces: candidate references, projected dynamic bounds and common load reference.

- [ ] **Step 1: Add a deterministic golden-data exporter and generate M1 data**

Create `test/reference/export_golden.py` with this complete behavior:

```python
#!/usr/bin/env python3
import argparse, csv, importlib.util, sys
from pathlib import Path
import numpy as np

def load_module(path: Path):
    spec = importlib.util.spec_from_file_location("agv_reference", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module

def flatten(name, value, row):
    arr = np.asarray(value)
    if arr.ndim == 0:
        row[name] = float(arr)
        return
    for index in np.ndindex(arr.shape):
        row[name + "_" + "_".join(map(str, index))] = float(arr[index])

def export(data, keys, count, output):
    rows = []
    available = len(np.asarray(data["t"]))
    for k in range(min(count, available)):
        row = {"step": k}
        for key in keys:
            flatten(key, np.asarray(data[key])[k], row)
        rows.append(row)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kind", choices=["exp1", "exp2-m1", "exp2-m2"], required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--count", type=int, required=True)
    args = parser.parse_args()
    module = load_module(args.source)
    if args.kind == "exp1":
        data = module.Experiment1Simulator(module.Params()).run()
        keys = ["t", "lower", "upper", "lower_s", "upper_s", "z", "ups", "phi",
                "vL", "sL", "x1", "x2", "es", "ev", "psi", "Gamma",
                "GammaInvUps", "r", "u", "theta_hat", "Dhat"]
    else:
        model = module.Experiment2Model(module.Config(output_dir=str(args.output.parent)))
        method = "M1" if args.kind == "exp2-m1" else "M2"
        data = module.simulate_method(model, method, record=True, reduced=False)
        keys = ["t", "lower", "upper", "lower_s", "upper_s", "z", "ups", "phi",
                "v_ref", "s_ref", "x1", "x2", "es", "ev", "psi", "Gamma",
                "GammaInvUps", "r", "u_raw", "u_act", "paper_delta_inv", "paper_zeta"]
    export(data, keys, args.count, args.output)

if __name__ == "__main__":
    main()
```

Generate the M1 fixture:

```bash
test -n "$EXP2_REFERENCE"
python3 src/multi_agv_control/test/reference/export_golden.py \
  --kind exp2-m1 \
  --source "$EXP2_REFERENCE" \
  --output src/multi_agv_control/test/data/m1_golden.csv \
  --count 10
sha256sum "$EXP2_REFERENCE" > src/multi_agv_control/test/data/reference_hashes.txt
```

Set `EXP2_REFERENCE` to the mounted Linux path corresponding to the approved file `E:\Microsoft Downloads\results_experiment2_v81_final\experiment2_v81_paper_faithful_comparison.py`. The `test -n` guard must fail before generation when the variable is absent.

- [ ] **Step 2: Write invariant and golden-vector tests**

Assert after every update: lower bound ≤ `-epsilon0`, upper bound ≥ `epsilon0`, width ≥ `delta`, common interval nonempty, common speed inside every inner interval, and C++ values match golden values within `1e-8` for the deterministic fixture.

Add mode isolation tests:

```cpp
TEST(UpperModes, M2aIgnoresCapabilityInControl) {
  auto a = runM2a(highCapability());
  auto b = runM2a(lowCapability());
  EXPECT_EQ(a.reference, b.reference);
  EXPECT_NE(a.logged_capability, b.logged_capability);
}
```

M4 must equal the pre-registered fixed speed and reject run-time attempts to rewrite it while a run is active.

- [ ] **Step 3: Implement projection and 25 Hz scheduling**

Implement Euclidean projection onto `Omega_i` with explicit active-constraint flags; do not replace it with independent scalar clips. Run the upper update every fourth 100 Hz controller tick and hold its output between updates.

- [ ] **Step 4: Run and commit**

```bash
catkin_make run_tests_multi_agv_control
catkin_test_results --verbose
git add src/multi_agv_control src/multi_agv_bringup/config/exp2a_*.yaml
git commit -m "feat: add M1 M2a and M4 upper modes"
```

---

### Task 14: Implement the paper lower controller and R1–R4 experiment modes

**Files:**
- Create: `src/multi_agv_control/include/multi_agv_control/lower_channel_controller.hpp`
- Create: `src/multi_agv_control/src/lower_channel_controller.cpp`
- Create: `src/multi_agv_control/test/test_lower_channel_controller.cpp`
- Create: `src/multi_agv_control/test/data/lower_m1_golden.csv`
- Create: `src/multi_agv_bringup/config/exp3_R{1,2,3,4}.yaml`
- Modify: `src/multi_agv_control/src/multi_agv_controller_node.cpp`

**Interfaces:**
- Consumes: actual `x1=s_actual`, `x2=s_dot_actual`, common execution reference and mapped acceleration limits.
- Produces: `u_raw`, `u_limited`, adaptive estimates and channel-speed command.

- [ ] **Step 1: Create deterministic lower-controller vectors from experiment1 simulation**

```bash
test -n "$EXP1_REFERENCE"
python3 src/multi_agv_control/test/reference/export_golden.py \
  --kind exp1 \
  --source "$EXP1_REFERENCE" \
  --output src/multi_agv_control/test/data/lower_m1_golden.csv \
  --count 100
sha256sum "$EXP1_REFERENCE" >> src/multi_agv_control/test/data/reference_hashes.txt
```

Set `EXP1_REFERENCE` to the mounted Linux path corresponding to `D:\Multiple Mobile Robots\experiment1_v37_polished_complete_package\experiment1_v37_polished_sim.py`. This fixture validates the complete lower M1 equations. R1–R4 are validated by explicit switch-isolation tests because the supplied experiment1 script contains the complete method rather than four separate runtime modes.

- [ ] **Step 2: Write tests**

Assert finite values inside the feasible velocity-error interval, projected parameter estimates remain inside configured sets, and complete-M1 C++ outputs match `lower_m1_golden.csv`. For R1–R4, hold identical state/reference inputs and assert only the registered parameter-adaptation and disturbance-compensation switches change; all shared terms must be bitwise equal. Add a test that long physical wheel saturation sets a reported condition and does not silently claim the unsaturated theorem.

- [ ] **Step 3: Implement at100 Hz**

Use actual `dt`; apply the mapped channel acceleration limit once; integrate channel speed and project to path velocity bounds. Keep this theoretical limiting distinct from the later wheel limiter and publish both levels in `ControllerState`.

- [ ] **Step 4: Run and commit**

```bash
catkin_make run_tests_multi_agv_control
catkin_test_results --verbose
git add src/multi_agv_control src/multi_agv_bringup/config/exp3_*.yaml
git commit -m "feat: add constrained adaptive lower controller modes"
```

---

### Task 15: Add the overhead-camera adapter without changing controller interfaces

**Files:**
- Create: `src/multi_agv_control/src/camera_pose_adapter_node.cpp`
- Create: `src/multi_agv_control/test/camera_pose_adapter.test`
- Create: `src/multi_agv_control/test/test_camera_pose_adapter.py`
- Create: `src/multi_agv_bringup/config/localization_camera.yaml`
- Create: `src/multi_agv_bringup/launch/camera_formal.launch`
- Modify: `src/multi_agv_control/src/path_state_estimator_node.cpp`

**Interfaces:**
- Consumes: four calibrated world-frame `PoseStamped` streams for agv1/agv2/agv3/load.
- Produces: unchanged `CooperativeState`; optional world→odom correction owned only by `pose_provider`.

- [ ] **Step 1: Write synthetic camera tests**

Publish three robot tags and one load tag with independent stamps. Drop only agv2 for several frames and assert only `robot_pose_valid[1]` becomes false. Assert raw stamp is retained, `header.stamp` is generation time, and no second publisher creates `world→agvX/base_link`.

- [ ] **Step 2: Implement calibrated rigid transforms**

Apply configured tag→base and tag→load transforms, reject stale/low-confidence data, preserve raw and causal-filter outputs, and select source enum `SOURCE_CAMERA` or `SOURCE_FUSED`. The controller remains subscribed only to `CooperativeState`.

- [ ] **Step 3: Test source switching**

Run the same constant-reference controller once with odom input and once with synthetic camera input. Assert command topic types and controller code path are identical; only localization source and measurement values differ.

- [ ] **Step 4: Commit**

```bash
git add src/multi_agv_control src/multi_agv_bringup
git commit -m "feat: add decoupled overhead camera pose adapter"
```

---

### Task 16: Implement and audit the complete M2b comparison mode

**Files:**
- Create: `src/multi_agv_control/include/multi_agv_control/m2b_controller.hpp`
- Create: `src/multi_agv_control/src/m2b_controller.cpp`
- Create: `src/multi_agv_control/test/test_m2b_controller.cpp`
- Create: `src/multi_agv_control/test/data/m2b_golden.csv`
- Create: `src/multi_agv_bringup/config/exp2b_M2b.yaml`
- Create: `src/multi_agv_bringup/config/exp2b_M1.yaml`
- Modify: `src/multi_agv_control/src/multi_agv_controller_node.cpp`

**Interfaces:**
- Consumes: same actual state, physical limiter and derating chain as M1.
- Produces: full M2b upper/lower variables and raw wheel demands.

- [ ] **Step 1: Export golden data from the approved experiment2 simulation**

Use the same checked-in exporter and the approved experiment2 reference:

```bash
test -n "$EXP2_REFERENCE"
python3 src/multi_agv_control/test/reference/export_golden.py \
  --kind exp2-m2 \
  --source "$EXP2_REFERENCE" \
  --output src/multi_agv_control/test/data/m2b_golden.csv \
  --count 100
```

The reference script names this method `M2`; in AGV_ROS it is registered as `M2b` to distinguish it from the capability-blind `M2a` ablation. Reuse the source hash recorded in Task 13. The exported fields are the complete recorded M2 vectors available from the approved script: `lower`, `upper`, `lower_s`, `upper_s`, `z`, `ups`, `phi`, `v_ref`, `s_ref`, `x1`, `x2`, `es`, `ev`, `psi`, `Gamma`, `GammaInvUps`, `r`, `u_raw`, `u_act`, `paper_delta_inv`, and `paper_zeta`.

- [ ] **Step 2: Write formula and boundary tests**

Compare C++ against golden data, assert fixed speed constraints use the frozen literature bounds, boundary-layer `sat` is continuous, all denominators retain configured positive margins, and formula residuals remain under `1e-8` in the deterministic fixture.

- [ ] **Step 3: Implement full mode**

Implement both upper and lower literature equations in a dedicated class. Do not reuse M1 dynamic capability feedback inside M2b control; still log mapped capability and pass final commands through the identical car-side limiter.

- [ ] **Step 4: Add paired fake-platform test**

Run M1 and M2b against the same recorded `CooperativeState` and car2 derating sequence. Assert identical physical capability reports and limiter configuration, different algorithm outputs, and complete M2b internal logging.

- [ ] **Step 5: Commit**

```bash
git add src/multi_agv_control src/multi_agv_bringup/config/exp2b_*.yaml
git commit -m "feat: add audited complete M2b comparison mode"
```

---

### Task 17: Add reproducible logging, validation and metrics

**Files:**
- Create: `src/multi_agv_analysis/package.xml`
- Create: `src/multi_agv_analysis/CMakeLists.txt`
- Create: `src/multi_agv_analysis/scripts/bag_to_csv.py`
- Create: `src/multi_agv_analysis/scripts/validate_run.py`
- Create: `src/multi_agv_analysis/scripts/compute_metrics.py`
- Create: `src/multi_agv_analysis/test/test_metrics.py`
- Create: `src/multi_agv_bringup/launch/experiment.launch`
- Create: `src/multi_agv_bringup/config/record_topics.yaml`

**Interfaces:**
- Consumes: frozen ROS topics and run manifest.
- Produces: audit report, aligned CSV/Parquet data and paper metrics.

- [ ] **Step 1: Write metric tests with a hand-computed fixture**

Create a ten-sample fixture where raw exceeds the limit for exactly three samples, applied differs for two samples, actual follows with known error and capability recovers at a known time. Assert exact `T_dem,w`, `T_lim,w`, maximum demand ratio, wheel RMSE, path RMSE, recovery time and task time.

- [ ] **Step 2: Implement validation before metrics**

`validate_run.py` must fail a run when required topics are absent, stamps move backward, sequence gaps exceed the registered rule, localization invalidity exceeds the exclusion rule, method/config hashes mismatch, or command authority count differs from one. It must report communication ages without claiming watchdog safety.

- [ ] **Step 3: Implement bag conversion and metrics**

Keep raw stamps and values; never zero-phase-filter data used for causal recovery metrics. Output separate raw/applied/actual columns and separate reported/mapped capability columns.

- [ ] **Step 4: Add experiment launch recording**

Start rosbag with all frozen topics, dump parameter server, write Git SHA/config hashes/interface version and run ID, and verify `move_base` has no command publisher before arming.

- [ ] **Step 5: Run and commit**

```bash
python3 -m unittest discover src/multi_agv_analysis/test -v
catkin_make --pkg multi_agv_analysis multi_agv_bringup
git add src/multi_agv_analysis src/multi_agv_bringup
git commit -m "feat: add reproducible experiment logging and metrics"
```

---

### Task 18: Run staged system gates before any formal experiment

**Files:**
- Create: `docs/test-protocols/stage-a-ros-chassis.md`
- Create: `docs/test-protocols/stage-b-single-car.md`
- Create: `docs/test-protocols/stage-c-three-car.md`
- Create: `docs/test-protocols/stage-d-light-load.md`
- Create: `docs/test-protocols/formal-exp2a-gate.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: all implemented packages and tests.
- Produces: signed evidence that each physical-risk gate passed.

- [ ] **Step 1: Run full software regression on Ubuntu Noetic**

```bash
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin_make run_tests
catkin_test_results --verbose
roslaunch-check multi_agv_bringup three_fake_chassis.launch
roslaunch-check multi_agv_bringup odom_pretest.launch
roslaunch-check multi_agv_bringup experiment.launch
```

Expected: zero failed tests and zero launch-check errors.

- [ ] **Step 2: Stage A—wheels raised or disconnected from load**

Verify three namespaces, one publisher per command, 250/100 Hz timing, wheel unit conversion, limiter rules, manual emergency stop and raw/applied/actual logging. Save bag and validation report.

- [ ] **Step 3: Stage B—single car on floor**

Run straight, rotation, speed ramp, reversal-to-zero, S-curve and car2 derating at low speed. Freeze wheel geometry and nominal acceleration/deceleration from measured data.

- [ ] **Step 4: Stage C—three cars without tray**

Run common start/stop, straight and low-speed S-curve using odometry. Verify no projection branch jump, command authority conflict or cross-car command leakage.

- [ ] **Step 5: Stage D—light tray**

Run straight then S-curve at `0.5v0`, `0.7v0`, `0.85v0`, `v0`; require three non-aborted runs at each accepted level. These remain pretests.

- [ ] **Step 6: Formal experiment2a gate**

Install/calibrate the overhead camera, verify per-tag validity and load pose, freeze M1/M2a/M4 configs, capability reserves, randomization and exclusion rules, then run one non-statistical dress rehearsal before paired data collection.

- [ ] **Step 7: Commit protocols and final implementation state**

```bash
git add docs/test-protocols README.md
git commit -m "docs: add staged AGV experiment verification gates"
git status --short
```

Expected: clean working tree.

---

## Phase Gates and Stop Conditions

1. Tasks 1–7 form the platform foundation. Do not start path/control work until fake three-car namespace, TF and command-authority tests pass.
2. Tasks 8–12 form the odometry pretest system. Do not place the rigid tray until single-car projection and car2 derating-chain tests pass.
3. Task 13 enables experiment2a algorithm flow; odometry runs remain pretests.
4. Task 15 and the formal gate in Task18 are required before experiment2a data enter paper statistics.
5. Task14 is required before claiming complete lower-controller or experiment3 implementation.
6. Task16 is required before calling experiment2b a complete M2b hardware comparison.
7. Any failed unit/rostest, duplicated command publisher, TF conflict, unit mismatch, invalid projection, sustained unexpected limiter activation or manual abort stops progression to the next physical stage.

## Recommended Execution Order

Execute Task0 and Tasks1–13, then Task17's logging foundation, then Tasks15 and18 formal experiment2a gate. After stable experiment2a acquisition, execute Task14 for experiment3 and Task16 for experiment2b. This preserves the laboratory order `实验2a → 实验1 → 实验3 → 实验2b` while allowing logging infrastructure to mature before formal collection.

## Frozen-Spec Coverage Audit

| Frozen requirement | Implementation tasks | Blocking tests/evidence |
|---|---:|---|
| Eight legal ROS1 messages, SI units and timestamps | 1 | message build and `rosmsg show` |
| Single-car executor, 250 Hz limiting and 100 Hz feedback | 2–5 | limiter, derating, odometry and fake-transport tests |
| car1 ROS Master, three namespaces and unique command/TF authority | 6–7 | namespace, publisher-count and TF-authority rostests |
| Arc-length S path, unique support geometry and local projection | 8–9 | derivative, nondegeneracy, branch-jump and invalid-state tests |
| Wheel-to-path capability mapping and engineering reserve | 10 | analytic and Monte Carlo containment tests |
| Common reference to three planar chassis commands | 11 | straight/arc/infeasible-command tests |
| Position-triggered car2 derating and causal capability feedback | 12 | end-to-end timing/order rostest |
| M1/M2a/M4 experiment2a separation | 13 | invariants, golden vectors and mode-isolation tests |
| Complete lower controller and R1–R4 experiment3 modes | 14 | experiment1 golden vectors and switch-isolation tests |
| Two-stage localization with unchanged controller input | 9, 15 | odom/camera source-switch and TF-owner tests |
| Complete M2b experiment2b comparison | 16 | experiment2 golden vectors and paired fake-platform test |
| Raw/applied/actual logging, reproducibility and paper metrics | 17 | hand-computed metrics and run-validation tests |
| Physical safety gates and formal-camera prerequisite | 18 | signed stage protocols and full regression output |

Every mandatory item in `docs/实物实验设计_v1.2.1_接口冻结版.md` is assigned to a file-producing task and a blocking verification. Items explicitly frozen out of scope—STM32 protocol extension, motor-current metrics and an independent communication watchdog—remain absent from the implementation tasks and must not be claimed in experiment reports.
