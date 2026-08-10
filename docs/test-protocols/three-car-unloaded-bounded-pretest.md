# 三车无载有界共同启动/停车

## 适用边界

本入口只适用于已经连续通过第二次只读门槛的冻结工装：

- Robot1 前、Robot2 左后、Robot3 右后；
- 三个转盘中心构成边长 `0.40 m` 的等边三角形；
- 三车初始车头平行；
- 无箱体、无共同托盘、无其他载荷；
- 路径参数 `A=0.05 m`、纵向长度 `1.0 m`；
- 连续推进公共参考弧长 `1.00 m`，中途不停顿；
- 速度固定为 `0.05 m/s`，名义匀速段约 `20 s`。

本步骤是三车无载连续 S 形预检，不是正式载荷对比实验。通用中央控制器、
降额入口和正式载荷入口仍未获得实物运动授权。

## 内置拒绝和联停条件

节点启动前要求：

- 四个显式实物确认均为 true；
- 三条命令话题各至少有两个订阅者，即底盘节点和 rosbag；
- `/multi_agv/cooperative_state` 恰有一个 Publisher；
- 旧中央控制器不再发布通用参考或控制器状态；
- 三车 robot、support、path 和虚拟 load 有效；
- 三车 odom 时间戳跨度不超过 `0.02 s`；
- 三车电压均不低于 `10.5 V`；
- 六个车轮的 raw、applied、actual 均处于停止容差；
- 初始虚拟载荷进度接近零。

上述完整安全条件必须连续满足 20 个控制周期，节点才会进入启动倒计时；单个
瞬时有效样本不会解锁运动。

运动中任一状态失效、反馈超时、控制循环超时、低电压、三车进度分散、载荷
进度误差、底盘跟踪误差或轮速命令越界，都会向三车重复发送零速命令并等待
六轮停止确认。每台车的底盘节点还配置了独立的 `0.20 s` ROS 命令超时：
中央节点退出或网络中断导致速度心跳消失时，底盘会自动把目标切为零并按减速
上限停车。

每台底盘另有 `0.15 s` STM32 反馈看门狗。新串口包停止到达后，底盘锁存故障
并按减速度限制发送零速；通信恢复不会自动恢复原运动，必须在新鲜反馈恢复后
收到零速命令才允许重新解锁。只读门槛和三车有界入口同时检查
`serial_receive_stamp` 年龄与 `packet_seq` 是否继续推进，避免 ROS 节点以
100 Hz 重发旧传感器值时被误认为正常。

这些软件超时保护要求底盘 ROS 进程和 Linux 主机仍在运行，不能覆盖底盘进程崩溃、
主机断电或 STM32 通信链自身失效。因此第一次实物运动仍必须由人员守在物理
断电/急停位置，不能把软件保护当作硬件急停。

## 终端安排

### 推荐：四个终端的一键流程

以下脚本会拒绝脏工作区、错误 hostname/IP、未同步系统时钟、串口缺失或占用
以及三车 Git SHA 不一致。Robot1 总控脚本会自动运行只读门槛、复位三车
里程计、连续通过两次完整有效状态门槛、保存参数快照和 manifest、启动
rosbag，并在录制连接建立后启动有界运动。

三车先按地面标记摆好，保持 Robot1 前、Robot2 左后、Robot3 右后，三个转盘
中心构成边长 `0.40 m` 的等边三角形，三车车头平行。随后依次执行：

Robot1 终端1：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
src/multi_agv_bringup/scripts/start_three_car_chassis.sh 1
```

Robot2 终端1：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
src/multi_agv_bringup/scripts/start_three_car_chassis.sh 2
```

Robot3 终端1：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
src/multi_agv_bringup/scripts/start_three_car_chassis.sh 3
```

三台终端都出现 `agvN READY` 后，在 Robot1 新终端执行：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
src/multi_agv_bringup/scripts/run_three_car_unloaded_pretest.sh \
  --confirm-area-clear \
  --confirm-wheels-on-floor \
  --confirm-unloaded-40cm-fixture
```

最后一个命令会实际驱动三辆车。三个确认参数只能在人员已经核对场地、车轮
落地和无载 40 cm 工装后填写。正常结束会打印 bag、参数快照和 manifest 的
绝对路径。任意门槛失败时脚本退出，不得删减检查参数或调用底层运动节点绕过。

### 手动诊断流程

只有需要逐项诊断时才使用下面的手动终端安排。

保留 Robot1 的 `roscore` 和三台电脑各自的串口底盘节点。三车不得再被人工
移动。停止之前的 `central_odom_pretest.launch` 终端；它退出时会同时停止旧
状态估计器和只读中央控制器。

### Robot1 终端3：记录

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.6.101 192.168.6.101

rosbag record -O /home/etlab/AGV_ROS/three_car_unloaded_bounded_run1.bag \
  /agv1/chassis_command /agv2/chassis_command /agv3/chassis_command \
  /agv1/chassis_feedback /agv2/chassis_feedback /agv3/chassis_feedback \
  /agv1/capability_report /agv2/capability_report /agv3/capability_report \
  /agv1/odom /agv2/odom /agv3/odom \
  /multi_agv/cooperative_state \
  /multi_agv/bounded_pretest/path_reference \
  /multi_agv/bounded_pretest/controller_state
```

等待所有订阅提示出现并保持终端运行。

### Robot1 终端4：有界入口

确认场地净空、急停人员就位、三车仍在 40 cm 标记上且电量合格后执行：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.6.101 192.168.6.101

roslaunch multi_agv_bringup three_car_unloaded_bounded_pretest.launch \
  platform_transport_type:=serial \
  enable_commands:=true \
  confirm_readonly_gate_passed:=true \
  confirm_test_area_clear:=true \
  confirm_wheels_on_floor:=true \
  confirm_unloaded_40cm_fixture:=true
```

节点会先等待输入和 rosbag 连接，再倒计时 `5 s`。正常完成日志必须包含：

```text
Three-car unloaded bounded pretest completed
all-wheel zero feedback confirmed
```

若节点拒绝或中止，不得绕过条件重启；先保存终端日志和 bag。紧急情况下优先
物理断电。普通人工中止可在有界入口终端按 `Ctrl+C`，节点会尝试向三车重复
发送零速命令。

## 完成后

确认三车完全停止，再在 rosbag 终端按 `Ctrl+C`。不要立即重新对位或复位
里程计；先保存有界入口完整日志、bag、Git SHA 和人工观察到的三车启动方向、
相对构型变化及停车情况。
