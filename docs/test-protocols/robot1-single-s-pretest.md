# Robot1 空载单车低速短 S 形预检

## 目的与边界

该入口只验证 Robot1 的里程计闭环、左右转向、路径投影、轮速命令和停车链。
它不是实验 1、2a、2b 或 3 的正式数据入口，不启动三车中央控制，也不使用
载荷支撑偏置 `q_tangent/q_normal`。

冻结的预检参数为：

- 正弦单周期：`x=xi, y=A*sin(2*pi*xi/L)`；
- `A=0.05 m`；
- `L=1.0 m`；
- 弧长速度 `0.05 m/s`；
- `base_link -> support_link = (-0.01783, 0) m`；
- 专用轮缘命令硬上限 `0.08 m/s`。

启动时，路径整体旋转和平移，使第一点的底盘参考位姿与 Robot1 当前
`/agv1/odom` 位姿重合。因此无须手动清零里程计，但车辆必须在启动前摆正，
启动后不得搬动车辆。

本正弦曲线的起点切线不是原始曲线的纵向轴。将起点切线与车头对齐后，
理论终点相对起始车体坐标约为 `x=+0.954 m, y=-0.300 m`，终点航向重新
回到起始航向。也就是说车辆会先向右侧展开再反向收曲率，而不是最终回到
车头起始中心线上；现场必须为右侧约 `0.4 m` 的扫掠范围留出净空。

## 安全条件

- Robot1 空载、车轮落地；
- 硬质平整地面；
- 建议净空不小于长 `1.5 m`、宽 `0.8 m`；
- 人员位于车侧，物理断电可立即操作；
- 前方和曲线两侧无人员、线缆、支架或载荷；
- `rosbag` 与底盘节点均已订阅命令话题；
- `/agv1/chassis_command` 不存在其他发布者。

节点在里程计/反馈超过 `0.15 s` 未更新、跟踪误差越界、轮速需求超过
`0.08 m/s`、运行超时或收到终止信号时重复发布零速。正常终点还要求连续
确认实际左右轮速回到 `0.01 m/s` 内。

## 终端 1：Robot1 底盘

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source src/multi_agv_bringup/scripts/setup_local_ros.sh
roslaunch multi_agv_bringup car1_master.launch transport_type:=serial
```

等待 `USB Connected`，不要关闭。

## 终端 2：记录

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source src/multi_agv_bringup/scripts/setup_local_ros.sh
rosbag record -O /home/etlab/AGV_ROS/agv1_single_s_pretest.bag \
  /agv1/chassis_command \
  /agv1/chassis_feedback \
  /agv1/capability_report \
  /agv1/odom \
  /agv1/imu \
  /agv1/s_pretest/reference_path \
  /agv1/s_pretest/chassis_reference \
  /tf \
  /tf_static
```

## 终端 3：只读检查和启动

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source src/multi_agv_bringup/scripts/setup_local_ros.sh
rostopic info /agv1/chassis_command
```

启动前必须看到两个订阅者（底盘和 rosbag），且没有发布者。确认场地后：

```bash
roslaunch multi_agv_bringup robot1_single_s_pretest.launch \
  platform_transport_type:=serial \
  enable_commands:=true \
  confirm_test_area_clear:=true \
  confirm_wheels_on_floor:=true
```

节点先发送零速并倒计时 `3 s`，再自动走完整短 S，最终自动停车。任何时候
可在终端 3 按 `Ctrl+C`；同时保留物理断电作为最终保护。

## 结束和验收

确认车轮停止后，先在终端 2 按 `Ctrl+C` 完成 bag，再关闭终端 1。

通过条件：

- 路径曲率先向一侧、随后向另一侧，航向最终回到起始航向附近；
- 终点相对起点约为前方 `0.954 m`、右侧 `0.300 m`；
- 左右轮速差在路径中换符号；
- 没有持续异常限幅、串口错误或人工中止；
- 节点报告 `zero wheel speed confirmed`；
- bag 包含运动命令及末尾零速命令。

该预检通过后，才能进入三车无载荷 S 形准备；不能据此解除正式实验的路径、
定位、载荷几何和顶部相机门槛。
