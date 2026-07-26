# 三车无载只读接入门槛

## 目的和停止边界

本步骤只把 Robot1、Robot2、Robot3 接入同一个 Robot1 ROS Master，验证三车
串口反馈、网络命名空间和中央状态链。它不发布底盘命令或降额命令，不是三车
运动实验。

代码必须保持：

- `enable_commands:=false`；
- `enable_derating:=false`；
- `/agv1/chassis_command`、`/agv2/chassis_command`、
  `/agv3/chassis_command` 均无 Publisher；
- 三个 `/agvX/derating_command` 均无 Publisher。

中央控制器在只读模式下不会注册命令 Publisher。只读拓扑通过之后，仍必须
实测并冻结 `world_to_odom`、无载起始支撑布局和场地 S 形尺寸；在这些参数完成
之前，不得把任何 `hardware_execution_authorized` 改为 `true`。

## 物理和网络准备

- 三车均无载荷，周围留出安全距离，电源开关可立即操作；
- 三台电脑连接同一局域网；
- Robot1 为 ROS Master，冻结地址 `192.168.0.50`；
- Robot2 地址 `192.168.0.54`，Robot3 地址 `192.168.0.55`；
- 三台电脑时钟已同步；
- 每台电脑只运行自己的 `carX_*.launch`；
- `/dev/ttyACM0` 没有被旧程序占用。

如果现场 IP 与上述冻结值不同，应先修改网络配置和实验记录，不要临时混用
`ROS_HOSTNAME=127.0.0.1`。

## Robot1 终端1：启动 ROS Master

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.0.50 192.168.0.50
roscore
```

## Robot1 终端2：启动 Robot1 底盘

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.0.50 192.168.0.50
fuser -v /dev/ttyACM0
roslaunch multi_agv_bringup car1_master.launch transport_type:=serial
```

等待 `USB Connected`。

## Robot2 终端1：启动 Robot2 底盘

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.0.54 192.168.0.50
fuser -v /dev/ttyACM0
roslaunch multi_agv_bringup car2_client.launch transport_type:=serial
```

等待 `USB Connected`。

## Robot3 终端1：启动 Robot3 底盘

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.0.55 192.168.0.50
fuser -v /dev/ttyACM0
roslaunch multi_agv_bringup car3_client.launch transport_type:=serial
```

等待 `USB Connected`。

## Robot1 终端3：启动中央只读状态链

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.0.50 192.168.0.50
roslaunch multi_agv_bringup central_odom_pretest.launch \
  platform_transport_type:=serial \
  enable_commands:=false \
  enable_derating:=false
```

必须看到中央控制器报告只读模式，且不得出现硬件命令授权信息。

## Robot1 终端4：运行拓扑只读门槛

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/multi_agv_bringup/scripts/setup_ros_network.sh \
  192.168.0.50 192.168.0.50
rosrun multi_agv_bringup check_three_car_readonly_gate.py \
  --observe-seconds 5
```

通过条件：

- 三车 odom、feedback、capability 均只有一个 Publisher；
- 上述九个数据话题均不低于 `80 Hz`；
- 三车 ID 和 frame 一一对应；
- 三车静止、无 `control_loop_overrun`、电压不低于 `10.8 V`；
- `/multi_agv/cooperative_state` 只有一个 Publisher 且不低于 `80 Hz`；
- 六个运动/降额命令话题均无 Publisher。

第一次检查是拓扑门槛。即使它通过，状态有效位仍可能因为尚未测量
`world_to_odom` 和支撑布局而为 false，这是预期的参数门控，不允许据此运动。

## 参数冻结后的第二次只读门槛

完成现场起始标记、路径和三个支撑位置测量并更新 YAML 后，仍保持命令关闭，
再次运行：

```bash
rosrun multi_agv_bringup check_three_car_readonly_gate.py \
  --observe-seconds 5 \
  --require-valid-state
```

只有输出 `THREE-CAR READ-ONLY GATE: PASSED`，并且三个 robot、support、path
有效位全部为 true，才进入三车无载共同启动/停止的软件授权评审。

## 任一异常的停止顺序

由于本步骤没有命令 Publisher，正常情况下车轮不会运动。若任一车辆自行运动：

1. 立即物理断电；
2. 停止 Robot1 中央入口；
3. 分别停止三车底盘节点；
4. 检查 ROS 图中的额外 Publisher 和旧自启动程序；
5. 不得继续到参数冻结或三车运动阶段。
