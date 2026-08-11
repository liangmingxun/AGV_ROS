# 三车协同相机融合闭环验证 v1

## 控制链路

三台车均使用 `/pose_provider/agvN/base_pose_fused` 参与位置闭环：顶部相机提供
世界坐标系绝对 `x/y/yaw`，已校正 IMU 航向和轮速位移仅在相邻相机帧间进行
100 Hz 短时传播。相机置信度不足、标定 epoch 改变，或者任一融合位姿超过
0.15 s 未更新时，三车 CooperativeState 失效，控制节点向三车统一停车。

无载荷阶段不要求 ID0。状态估计器从三台车的融合 base_link 位姿计算支撑中心，
用冻结的 0.40 m 等边三角形做 SE(2) 刚体拟合，得到虚拟载荷/编队中心。首次有效
同步样本把虚拟载荷自动锚定到路径 `s=0`，因此不再使用固定 `world_to_odom`，也
不要求把车辆摆在相机世界坐标的某个绝对数值上。Task 15 的真实载荷接口继续保留。

原始 `/pose_provider/agvN/base_pose_raw` 仍录包，用于检查融合滞后和编队形变；它与
闭环相机同源，不能称为独立测量真值。

## 实验级别

必须按顺序执行：

1. 短直线门：0.03 m/s、0.30 m；验证融合链路、同步起停和初始编队。
2. S 路径门：0.04 m/s、1.00 m；验证正负转向、编队保持和三车差异化参数。

自动分析采用 raw 相机位姿相对参考轨迹的变化量：直线门的车对距离漂移上限为
20 mm、相对航向误差上限为 3 deg；S 门分别为 30 mm 和 5 deg。`REVIEW` 表示
安全运行已完成但精度超出初始门槛，不等同于控制故障。

## 初始构型

三车车头方向一致，支撑/转盘中心构成约 0.40 m 等边三角形：Robot1 前方、
Robot2 左后、Robot3 右后。软件允许刚体整体平移和旋转，但初始构型拟合 RMS 必须
不超过 25 mm；三车朝向误差还必须满足控制节点的起步航向门。

## 启动

Windows：

```bat
cd C:\Users\lmx\Desktop\windows_sender
.venv\Scripts\activate
python aruco_udp_sender.py --config config.yaml
```

Robot1 窗口1：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
./start_vision_udp_bridge.sh
```

Robot1 窗口2：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
./start_camera_pose_fusion_only.sh
```

Robot1、Robot2、Robot3 分别启动底盘：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
./src/multi_agv_bringup/scripts/start_three_car_chassis.sh 1
```

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
./src/multi_agv_bringup/scripts/start_three_car_chassis.sh 2
```

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
./src/multi_agv_bringup/scripts/start_three_car_chassis.sh 3
```

Robot1 执行短直线门：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
./run_three_car_cooperative_straight_validation.sh \
  --confirm-area-clear \
  --confirm-wheels-on-floor \
  --confirm-unloaded-40cm-fixture
```

短直线安全完成并分析后，重新摆好三车执行 S 门：

```bash
./run_three_car_cooperative_s_validation.sh \
  --confirm-area-clear \
  --confirm-wheels-on-floor \
  --confirm-unloaded-40cm-fixture
```

结果位于：

```text
experiment_data/three_car_cooperative_validation/<时间>_<straight|s>/
```

Robot1 当前为经过五次 S 路径验证的候选参数；Robot2、Robot3 使用各自冻结参数。
