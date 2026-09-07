# 三车相机闭环队形自动初始化

## 功能与目标队形

入口 `run_three_car_camera_formation_init.sh` 使用三车的
`/pose_provider/agvN/base_pose_fused` 作为绝对位姿反馈。Robot1 保持不动，
其当前车头方向定义队形正前方；Robot2、Robot3 依次移动到边长 0.30 m 的
等边三角形支撑点队形：

- Robot1：前方顶点；
- Robot2：左后顶点；
- Robot3：右后顶点；
- 三车最终朝向与 Robot1 平行。

几何计算使用车辆顶部支撑中心，而不是 ArUco 几何中心或驱动轴中心。
当前 `base_link -> support` 分车使用：Robot1 `x=-0.01783 m`，
Robot2/3 `x=+0.09908 m`，三车均为 `y=0`。

## 自动动作

1. 检查 Windows ArUco 链路、三车融合位姿、三套底盘反馈、电池电压和六轮停车状态。
2. 以 Robot1 当前融合位姿计算 Robot2、Robot3 的目标位姿。
3. Robot1、Robot3 保持零速，Robot2 以相机融合位姿闭环完成转向、平移和最终朝向对齐。
4. Robot2 停稳后保持零速，再自动对齐 Robot3。
5. 检查三条支撑点边长、三车平行度和六轮停车状态，发布
   `FORMATION_CONFIRMED`。

限速为 `0.025 m/s`，轮速上限 `0.05 m/s`。任一车辆的初始位置距离其目标
超过 `0.65 m` 时拒绝动作，因此运行前仍需把三车粗略摆成“Robot1 在前，Robot2
左后，Robot3 右后”的布局。两车驱动轴中点（`base_link`）距离小于 `0.22 m`、静止车辆漂移、
相机数据超时、持续欠压或轮速异常时，程序立即向三车重复发送停车命令。
`10.5 V` 门槛要求连续低于门槛 `0.5 s`，用于排除电池采样中持续约
`0.15--0.19 s` 的孤立跳变；持续欠压仍会立即结束动作并停车。
接近目标 `0.06 m` 后降至 `0.010 m/s`，进入 `0.03 m` 停靠区后降至
`0.005 m/s`。控制器允许自动选择前进或倒车；轻微越过目标时直接低速倒回，
不会通过持续绕圈重新追逐目标。开始平移后连续 `12 s` 没有改善位置误差也会中止。
Robot2/3 的动态旋转中心外参校正后，控制采用分阶段差速车停靠：允许选择前进或
倒车并先收敛位置，进入 `15 mm` 位置门后再原地收敛最终航向。位置和航向不再由
一个可能相互抵消的线性控制式同时求解。
最终航向阶段具有状态锁存；转向造成的小幅位置漂移不会触发控制阶段反复切换。
进入 `20 mm` 位置门后，航向稳定确认使用额外 `5 mm` 的位置滞回带；只有离开
`25 mm` 稳定带才重新修正位置，漂移超过 `30 mm` 时强制退出航向阶段。这组
门槛来自 Robot3 实测约 `4.9--5.8 mm` 的转向位置漂移，并与后续路径验证的
`30 mm` 队形误差尺度一致。单车航向通过线仍为 `2°`；三车完成后独立检查
三条支撑点边长，每条相对 `0.30 m` 的误差均不得超过 `30 mm`。

## 运行前提

- Windows 相机发送端、Robot1 视觉桥正在运行；
- Robot1 上的 `pose_provider` 和 `camera_odom_fusion` 正在运行；
- 三台车的 `/agvN/chassis_controller` 正在运行；
- 不运行其他会发布 `/agvN/chassis_command` 的控制或测试节点；
- 测试区内无人员和障碍，急停可随时使用。

## 执行

在 Robot1 的工作树中运行：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux

./run_three_car_camera_formation_init_30cm.sh \
  --confirm-area-clear \
  --confirm-wheels-on-floor \
  --confirm-automatic-formation
```

成功时输出：

```text
PASS: camera-fused three-car formation initialization confirmed.
```

完整相机位姿、融合位姿、底盘命令、反馈、IMU、里程计、目标位姿和最终结果
保存在 `experiment_data/three_car_camera_formation_init/<时间>/`。只有出现
`FORMATION_CONFIRMED` 才能继续运行三车协同验证；`ABORTED` 必须先排除其给出的
具体原因，不能绕过检查。
