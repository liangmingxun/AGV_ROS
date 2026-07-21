# AGV_ROS 三车 Ubuntu 移植与上车检查手册（v1.0）

> 适用范围：`feature/platform-foundation` 分支，Task 0–7 平台基础层。
> 目标系统：Ubuntu 20.04、ROS Noetic、三台车载电脑、car1 作为 ROS Master。
> 当前限制：中央协同控制、S形路径、能力映射和论文实验算法尚未迁移；独立通信看门狗尚未实现。
> 安全要求：首次真实串口测试必须举升车轮；人工急停和断电装置必须随时可用。

---

## 1. 先明确：三台车如何确定身份

三台车安装完全相同的 AGV_ROS 代码。车辆身份不根据 IP 或 STM32 UID 自动推断，而由启动文件和逐车 YAML 显式确定：

| 物理车辆 | 启动文件 | 配置文件 | ROS命名空间 | 消息数字ID |
|---|---|---|---|---:|
| car1 | `car1_master.launch` | `agv1_chassis.yaml` | `/agv1` | 1 |
| car2 | `car2_client.launch` | `agv2_chassis.yaml` | `/agv2` | 2 |
| car3 | `car3_client.launch` | `agv3_chassis.yaml` | `/agv3` | 3 |

必须遵守：

- [ ] car1 只启动 `car1_master.launch`。
- [ ] car2 只启动 `car2_client.launch`。
- [ ] car3 只启动 `car3_client.launch`。
- [ ] 不通过修改 C++ 代码区分车辆。
- [ ] 不恢复原有“根据 IP 最后一段判断车辆”的逻辑。
- [ ] 不在三台车上维护三份不同代码；三台车必须使用相同 Git 提交号。

---

## 2. 移植前需要准备的记录

### 2.1 三台电脑信息表

移植前先填写，不要边部署边猜测：

| 项目 | car1 | car2 | car3 |
|---|---|---|---|
| Ubuntu版本 |  |  |  |
| 主机名 |  |  |  |
| 固定IP | `192.168.0.50`（预设） | `192.168.0.54`（预设） | `192.168.0.55`（预设） |
| 有线网卡名 |  |  |  |
| 底盘串口设备 | `/dev/chassis_driver`（预设） | `/dev/ttyACM0`（预设） | `/dev/ttyACM0`（预设） |
| STM32/底盘对应关系 |  |  |  |
| ROS版本 |  |  |  |
| Git提交号 |  |  |  |
| IMU标定日期 |  |  |  |

### 2.2 网络规划

三台电脑必须在同一个可互相访问的实验局域网中。推荐保持当前规划：

```text
car1  192.168.0.50  ROS Master
car2  192.168.0.54  ROS client
car3  192.168.0.55  ROS client
```

- [ ] 三个IP没有被其他设备占用。
- [ ] 三台电脑能互相 `ping`。
- [ ] car2、car3可以访问 car1 的 TCP 11311 端口。
- [ ] 实验网络与不可信公共网络隔离。
- [ ] 若启用主机防火墙，只允许可信实验网段的 ROS1 通信，不随意关闭整机防火墙。
- [ ] 三台电脑完成主机时间同步；正式记录前检查 `chronyc tracking`。

---

## 3. 三台电脑共同安装和复制的内容

### 3.1 基础软件检查

在三台电脑分别运行：

```bash
lsb_release -a
rosversion -d
rosversion roscpp
python3 --version
cmake --version
g++ --version
```

期望：

```text
Ubuntu 20.04
ROS Noetic
Python 3
```

- [ ] 三台电脑ROS发行版一致。
- [ ] 三台电脑系统架构和主要编译器版本已记录。
- [ ] 三台电脑均安装 `chrony` 或实验室统一使用的时间同步工具。
- [ ] 当前登录用户具有串口访问权限。

串口组权限可检查为：

```bash
groups
ls -l /dev/ttyACM0
```

若用户不在 `dialout` 组，按照Ubuntu管理规范加入后重新登录。不要用长期 `chmod 777` 代替正确的设备权限规则。

### 3.2 复制同一版本代码

三台电脑都复制相同工作区，例如：

```text
~/AGV_ROS
```

部署本版本时，确认代码包含提交：

```text
083fd86 fix: preserve chassis cycle audit flags
```

在每台电脑运行：

```bash
cd ~/AGV_ROS
git branch --show-current
git rev-parse HEAD
git status --short
```

要求：

- [ ] 使用 `feature/platform-foundation` 分支，或使用已经验证后合并该分支的提交。
- [ ] 三台电脑 `git rev-parse HEAD` 完全一致。
- [ ] `git status --short` 没有未记录修改。
- [ ] 不从Windows复制 `build/`、`devel/`、`logs/`；每台Ubuntu电脑本地重新构建。

---

## 4. 首次在Ubuntu修复和构建catkin工作区

仓库中的 `src/CMakeLists.txt` 在Windows副本中可能是展开后的Linux路径文本。每台Ubuntu电脑首次构建前运行：

```bash
cd ~/AGV_ROS
source /opt/ros/noetic/setup.bash
if [ ! -L src/CMakeLists.txt ]; then
  mv src/CMakeLists.txt src/CMakeLists.txt.flattened
  catkin_init_workspace src
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
source devel/setup.bash
```

- [ ] `src/CMakeLists.txt`最终是有效catkin顶层链接/文件。
- [ ] `rosdep install`的未解决依赖已经逐项记录和处理。
- [ ] `catkin_make`零错误完成。
- [ ] 没有把编译警告当作无关信息直接忽略；涉及单位、消息、未初始化变量的警告必须处理。

构建后检查新包：

```bash
rospack find agv_msgs
rospack find chassis_controller
rospack find multi_agv_bringup
rosmsg show agv_msgs/ChassisCommand
rosmsg show agv_msgs/ChassisFeedback
rosmsg show agv_msgs/CapabilityReport
```

---

## 5. 每辆车需要修改或核对的YAML

配置目录：

```text
src/multi_agv_bringup/config/
```

### 5.1 car1

文件：`agv1_chassis.yaml`

必须保持的身份组合：

```yaml
robot_id: agv1
robot_index: 1
odom_frame: agv1/odom
base_frame: agv1/base_link
imu_frame: agv1/imu_link
support_frame: agv1/support_link
```

### 5.2 car2

文件：`agv2_chassis.yaml`

必须保持的身份组合：

```yaml
robot_id: agv2
robot_index: 2
odom_frame: agv2/odom
base_frame: agv2/base_link
imu_frame: agv2/imu_link
support_frame: agv2/support_link
```

### 5.3 car3

文件：`agv3_chassis.yaml`

必须保持的身份组合：

```yaml
robot_id: agv3
robot_index: 3
odom_frame: agv3/odom
base_frame: agv3/base_link
imu_frame: agv3/imu_link
support_frame: agv3/support_link
```

### 5.4 可能需要根据实车修改的参数

#### IP记录

```yaml
host_ip: 192.168.0.54
```

注意：`host_ip`当前主要用于配置审计，不直接设置ROS网络。真正参与通信的是启动终端中的：

```text
ROS_IP
ROS_MASTER_URI
```

如果实际IP变化，必须同时更新YAML记录和网络启动命令。

#### 串口路径

```yaml
serial_device: /dev/ttyACM0
```

在每台车检查：

```bash
ls -l /dev/ttyACM*
udevadm info -a -n /dev/ttyACM0
```

推荐根据实际USB属性为三台车建立稳定的udev名称，例如 `/dev/agv_chassis`。在没有读取真实 `idVendor`、`idProduct` 和设备序列属性前，不复制臆测的udev规则。

#### 底盘几何

```yaml
wheel_radius: 0.0325
wheel_separation: 0.114
base_link_z: 0.05969
```

- [ ] 轮半径由实测有效滚动半径确认。
- [ ] 轮距由低速直线/转向标定确认。
- [ ] 三种对比方法使用同一份冻结几何参数。

#### 轮速和加减速度能力

```yaml
nominal:
  max_wheel_linear_velocity_left: 0.9
  max_wheel_linear_velocity_right: 0.9
  max_wheel_linear_acceleration_left: 1.0
  max_wheel_linear_acceleration_right: 1.0
  max_wheel_linear_deceleration_left: 2.0
  max_wheel_linear_deceleration_right: 2.0
```

这些是首轮开发值，不应未经举升轮和单车地面标定就作为论文正式能力值。

#### IMU标定

```yaml
imu:
  acc_bias: [...]
  acc_scale: [...]
  gyro_bias: [...]
  gyro_lpf_tau: 0.02
```

- [ ] 确认标定数据与当前物理底盘一一对应。
- [ ] 若车载电脑、STM32或IMU互换，重新核对标定归属。
- [ ] 静止时检查角速度零偏和加速度方向。

### 5.5 当前不建议修改的内容

- [ ] 不改变ROS层轮速单位：始终为 `m/s`。
- [ ] 不改变串口层轮速单位：继续为现有 `mm/s`。
- [ ] 不修改STM32协议。
- [ ] 不把三车话题改回全局 `/cmd_vel` 或 `/odom`。
- [ ] 不让两套节点同时发布同一辆车的 `chassis_command`。
- [ ] 不让相机或其他定位节点直接重复发布 `odom → base_link`。

---

## 6. ROS网络环境设置

每个新终端都需要设置ROS环境。脚本必须用 `source`，不能直接执行。

### 6.0 单机本地测试

在一台车载电脑上本地启动ROS Master和该车节点时，每个新终端只需执行：

```bash
source /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/src/multi_agv_bringup/scripts/setup_local_ros.sh
```

该脚本会自动完成：

- 定位并切换到当前工作区；
- 加载 `/opt/ros/noetic/setup.bash` 和工作区 `devel/setup.bash`；
- 清除 `ROS_IP`；
- 设置本机 `ROS_MASTER_URI` 和 `ROS_HOSTNAME` 为 `127.0.0.1`。

`source` 不能省略：直接执行子进程无法修改当前终端的环境变量。该配置只用于单机本地测试，不用于三车联网实验。

### 6.1 car1

```bash
cd ~/AGV_ROS
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.0.50 192.168.0.50
```

### 6.2 car2

```bash
cd ~/AGV_ROS
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.0.54 192.168.0.50
```

### 6.3 car3

```bash
cd ~/AGV_ROS
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.0.55 192.168.0.50
```

每台车检查：

```bash
echo "$ROS_IP"
echo "$ROS_MASTER_URI"
```

预期：

```text
car1 ROS_IP=192.168.0.50
car2 ROS_IP=192.168.0.54
car3 ROS_IP=192.168.0.55
三车 ROS_MASTER_URI=http://192.168.0.50:11311
```

从car2、car3检查Master：

```bash
rostopic list
rosnode list
```

若命令长时间阻塞，先检查网络、IP、主机时间和防火墙，不要直接修改车辆ID绕过问题。

---

## 7. 先做单机fake测试，不连接STM32

在car1或开发机上执行：

```bash
cd ~/AGV_ROS
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch multi_agv_bringup three_fake_chassis.launch
```

另开终端检查：

```bash
rosnode list
rostopic list | sort
```

应存在：

```text
/agv1/chassis_feedback
/agv1/capability_report
/agv1/odom
/agv1/imu
/agv2/chassis_feedback
/agv2/capability_report
/agv2/odom
/agv2/imu
/agv3/chassis_feedback
/agv3/capability_report
/agv3/odom
/agv3/imu
```

不应出现三车共同使用的：

```text
/cmd_vel
/odom
/imu
```

检查频率：

```bash
rostopic hz /agv1/chassis_feedback
rostopic hz /agv2/chassis_feedback
rostopic hz /agv3/chassis_feedback
rostopic hz /agv1/capability_report
```

目标约为100 Hz。

---

## 8. 运行自动测试

在Ubuntu工作区执行：

```bash
cd ~/AGV_ROS
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests_agv_msgs
catkin_make run_tests_chassis_controller
catkin_make run_tests_multi_agv_bringup
catkin_test_results --verbose
```

必须通过：

- [ ] 冻结消息接口测试。
- [ ] 轮速限幅器gtest。
- [ ] 平滑降额gtest。
- [ ] 里程计积分gtest。
- [ ] `ChassisCore` gtest。
- [ ] fake transport节点测试。
- [ ] 三车命名空间隔离rostest。
- [ ] 命令唯一发布权rostest。
- [ ] TF唯一发布权rostest。
- [ ] `catkin_test_results --verbose`显示零失败。

任何一项失败都不能进入真实串口和落地测试。

---

## 9. 三辆车真实启动顺序

### 9.1 启动前

- [ ] 三台车动力关闭或驱动轮举升。
- [ ] 急停和物理断电有效。
- [ ] car1、car2、car3网络互通。
- [ ] 三台电脑时间同步。
- [ ] 串口设备路径存在且权限正确。
- [ ] 没有启动 `move_base` 或其他速度发布器。
- [ ] 现场没有人员位于车轮或车辆运动危险区域。

### 9.2 启动car1

```bash
roslaunch multi_agv_bringup car1_master.launch
```

### 9.3 启动car2

```bash
roslaunch multi_agv_bringup car2_client.launch
```

### 9.4 启动car3

```bash
roslaunch multi_agv_bringup car3_client.launch
```

### 9.5 在car1集中检查

```bash
rosnode list
rostopic list
roswtf
```

检查每台车：

```bash
rostopic echo -n 1 /agv1/chassis_feedback
rostopic echo -n 1 /agv2/chassis_feedback
rostopic echo -n 1 /agv3/chassis_feedback
rostopic echo -n 1 /agv1/capability_report
rostopic echo -n 1 /agv2/capability_report
rostopic echo -n 1 /agv3/capability_report
```

重点核对：

- [ ] `robot_id`分别为1、2、3。
- [ ] `header.frame_id`分别使用对应的 `agvX/base_link`。
- [ ] 实际轮速静止时接近零。
- [ ] 电池电压在合理范围内。
- [ ] IMU数值有限且方向正确。
- [ ] `packet_seq`持续更新。
- [ ] `serial_receive_stamp`持续更新。

---

## 10. 举升车轮下的低速命令测试

> **重要：当前版本没有独立通信看门狗。节点会保持最后一条有效命令。手动测试后必须发送更大序列号的零速度命令，不能只停止 `rostopic pub`。**

以下以agv1为例。先确保车轮已经举升。

发送低速命令：

```bash
rostopic pub -1 /agv1/chassis_command agv_msgs/ChassisCommand "
header: {stamp: now}
robot_id: 1
command_seq: 1
control_mode: 1
linear_velocity_reference: 0.05
angular_velocity_reference: 0.0
wheel_linear_velocity_left_raw: 0.05
wheel_linear_velocity_right_raw: 0.05
experiment_id: migration_check
method_id: manual
"
```

发送零速度停止命令：

```bash
rostopic pub -1 /agv1/chassis_command agv_msgs/ChassisCommand "
header: {stamp: now}
robot_id: 1
command_seq: 2
control_mode: 1
linear_velocity_reference: 0.0
angular_velocity_reference: 0.0
wheel_linear_velocity_left_raw: 0.0
wheel_linear_velocity_right_raw: 0.0
experiment_id: migration_check
method_id: manual
"
```

agv2和agv3测试时，必须同时修改：

```text
话题 /agvX/chassis_command
robot_id 数字
command_seq 单调递增
```

逐车验证：

- [ ] 正向低速转动方向正确。
- [ ] 左右轮没有接反。
- [ ] ROS命令 `0.05 m/s`到串口层的量纲换算正确。
- [ ] `raw`等于发送值。
- [ ] `applied`按加速度斜坡逐步接近目标。
- [ ] `actual`由STM32反馈并跟随 `applied`。
- [ ] 停止命令使用更大序列号后有效。
- [ ] 正转到反转时先减速到零，再反向加速。
- [ ] 超出最大速度时 `speed_limit_active`有效。
- [ ] 快速阶跃时 `accel_limit_active`或 `decel_limit_active`有效。

---

## 11. car2软件降额检查

只在车轮举升且低速命令验证完成后进行。

向car2发送降额：

```bash
rostopic pub -1 /agv2/derating_command agv_msgs/DeratingCommand "
header: {stamp: now}
robot_id: 2
command_seq: 1
mode: 1
active: true
target_speed_ratio_left: 0.7
target_speed_ratio_right: 0.7
target_accel_ratio_left: 0.72
target_accel_ratio_right: 0.72
target_decel_ratio_left: 0.72
target_decel_ratio_right: 0.72
ramp_down_time: 2.0
ramp_up_time: 3.0
experiment_id: migration_check
"
```

观察：

```bash
rostopic echo /agv2/capability_report
rostopic echo /agv2/chassis_feedback
```

恢复名义能力：

```bash
rostopic pub -1 /agv2/derating_command agv_msgs/DeratingCommand "
header: {stamp: now}
robot_id: 2
command_seq: 2
mode: 0
active: false
target_speed_ratio_left: 1.0
target_speed_ratio_right: 1.0
target_accel_ratio_left: 1.0
target_accel_ratio_right: 1.0
target_decel_ratio_left: 1.0
target_decel_ratio_right: 1.0
ramp_down_time: 2.0
ramp_up_time: 3.0
experiment_id: migration_check
"
```

验收：

- [ ] 只有car2能力变化，car1和car3不受影响。
- [ ] 能力按2秒斜坡下降，没有瞬间跳变。
- [ ] 速度能力下降到名义值的0.7倍。
- [ ] 加减速度能力下降到名义值的0.72倍。
- [ ] applied轮速服从新的能力边界。
- [ ] 恢复时按3秒斜坡回到名义能力。
- [ ] `CapabilityReport`与车端实际执行边界一致。

---

## 12. TF和话题发布权检查

期望TF：

```text
agvX/odom
└─ agvX/base_link
   ├─ agvX/imu_link
   └─ agvX/support_link
```

检查：

```bash
rosrun tf view_frames
rosrun tf tf_echo agv1/odom agv1/base_link
rosrun tf tf_echo agv2/odom agv2/base_link
rosrun tf tf_echo agv3/odom agv3/base_link
```

命令发布权：

```bash
rostopic info /agv1/chassis_command
rostopic info /agv2/chassis_command
rostopic info /agv3/chassis_command
```

- [ ] 每台车同一时刻只有一个命令发布者。
- [ ] 不存在全局 `/cmd_vel`发布者。
- [ ] 不存在全局 `/odom`发布者。
- [ ] 每条TF边只有一个发布者。
- [ ] 当前阶段没有相机节点重复发布 `world → agvX/base_link`。

---

## 13. 首次落地测试前的停止条件

首次单车落地直线测试使用定时停车工具，禁止用单条非零速度 `rostopic pub`长时保持运动：

```bash
rosrun multi_agv_bringup run_timed_straight_test.py \
  --robot-id 2 \
  --speed 0.03 \
  --duration 2.0 \
  --command-seq 1000 \
  --required-command-subscribers 2 \
  --confirm-wheels-on-floor
```

该工具限制轮速绝对值不超过 `0.05 m/s`、运动时间不超过5秒，正常到时或收到 `Ctrl+C`/`SIGTERM`/终端挂断时会用连续递增序号发送3条零速度命令。录包测试使用 `--required-command-subscribers 2`：只有底盘控制器和rosbag都连接到命令话题，并稳定0.5秒后才发送运动命令。该参数不能代替录包终端的正常 `Ctrl+C` 和 `indexed: True` 检查。

它不能在进程被 `SIGKILL`、ROS Master不可达或下位机通信中断时保证停车，因此不替代底层通信看门狗、人工急停和断电措施。

出现以下任一情况，停止测试并切断动力：

- 车辆ID、命名空间或TF前缀不匹配；
- 左右轮方向错误或接线对应关系不明确；
- 串口设备反复断开；
- 实际轮速出现NaN、异常跳变或长时间不更新；
- `packet_seq`停止更新；
- 一个命令话题出现多个发布者；
- 限幅后速度超过报告能力；
- 反转命令没有先减速到零；
- 循环超时频繁触发；
- IMU静止数据明显异常；
- 三台电脑时间不同步；
- 人工急停或断电装置不可用。

在故障原因确认并重新执行相关检查项前，不恢复动力。

---

## 14. 移植完成验收表

### 14.1 软件与版本

- [ ] 三车Ubuntu/ROS版本记录完成。
- [ ] 三车Git提交号一致。
- [ ] 三车本地构建通过。
- [ ] 全部gtest/rostest零失败。
- [ ] 三车配置文件归档并计算哈希。

### 14.2 身份与网络

- [ ] car1=`agv1/index1`。
- [ ] car2=`agv2/index2`。
- [ ] car3=`agv3/index3`。
- [ ] car1为唯一ROS Master。
- [ ] car2、car3能稳定访问car1 Master。
- [ ] 三车时钟同步满足实验要求。

### 14.3 底盘与数据

- [ ] 三车串口路径稳定。
- [ ] 三车轮方向正确。
- [ ] raw/applied/actual三层数据正确。
- [ ] 速度、加速度、减速度限幅正确。
- [ ] 反转先归零正确。
- [ ] car2降额和恢复正确。
- [ ] 100 Hz反馈与能力报告稳定。
- [ ] 里程计和IMU持续发布。

### 14.4 允许进入下一阶段的结论

只有以上项目全部通过后，才允许：

1. 进行单车低速落地标定；
2. 进行三车无托盘直线和低速联动；
3. 冻结轮速、加速度、减速度和底盘几何参数；
4. 开始Task 8以后S形路径、统一状态接口和中央控制算法迁移。

当前平台基础层通过并不等于论文正式实验已经可以开始。实验2a正式采集仍需完成中央算法和顶部相机定位。

---

## 15. 每次移植/更新后的记录模板

```text
日期：
操作者：
实验室位置：

car1 hostname/IP：
car2 hostname/IP：
car3 hostname/IP：

Git分支：
Git提交号：
配置文件哈希：

catkin_make：通过 / 失败
catkin测试：通过 / 失败
fake三车测试：通过 / 失败
举升轮测试：通过 / 失败
car2降额测试：通过 / 失败

未解决警告：
修改过的参数：
保存的日志/rosbag路径：
是否允许进入下一阶段：是 / 否
签字：
```

配套的Ubuntu命令级门禁清单见：`docs/platform-foundation-ubuntu-validation.md`。
