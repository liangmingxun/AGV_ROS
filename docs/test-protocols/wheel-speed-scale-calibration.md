# 单车多速度物理轮速尺度标定

该工具只用于闭合“ROS physical m/s ↔ STM32 nominal mm/s”的速度尺度，不属于
M1/M2a正式实验入口。阶段D映射接入后，工具会发送物理域目标并经过正式公共
映射，可用于逐车复验；没有command scale的旧配置仍保留阶段B逆尺度激励方式。

## 前置条件

- Robot1相机、`pose_provider`和`camera_odom_fusion`正常运行；
- 被测车的serial chassis节点已用人工确认过的`0.16 m/s`包络启动；
- 同一时刻只测试一辆车；
- 被测车前后各至少`0.9 m`区域清空，急停可随时触及；
- 电池电压至少`10.5 V`；
- 三台车分别在自己的主机上执行，不远程驱动另一台车。

## 一键采集

以Robot2为例：

```bash
cd ~/AGV_ROS/.worktrees/platform-foundation-linux

./src/multi_agv_bringup/scripts/run_wheel_speed_scale_calibration.sh \
  --robot-id 2 \
  --operator ETLAB \
  --confirm-area-clear \
  --confirm-wheels-on-floor \
  --confirm-emergency-stop-ready \
  --confirm-camera-physical-truth \
  --confirm-bounded-scale-excitation
```

默认依次测试`0.06/0.08/0.10/0.12/0.14/0.16 m/s`，每个速度执行正向和
反向，重复三轮。每段使用`1.5 s`五次平滑加速、`3.0 s`稳态和`1.5 s`
平滑停车。正反向完成后必须回到该速度点的起点附近，否则停止后续动作。

阶段B采集时正式command mapping尚未建立。为避免把`0.16`直接发送成可能约
`0.18 m/s`的物理速度，本工具仅用现有feedback scale的倒数选择保守激励值。
这个倒数只属于本次激励策略，不会被写入正式配置；最终command候选由相机
真值相对于实际firmware target重新拟合。

正式系统中的`0.18 m/s`保持为pre-limit需求诊断阈值，不作为actual中止线。
标定工具独立使用逐反馈样本的`目标速度+0.03 m/s`持续异常门，并保留
`0.20 m/s`单帧绝对停止门；这些门只保护标定过程，不改变正式运行语义。

## 自动输出

结果保存在：

```text
experiment_data/wheel_speed_scale_calibration/
  agvN_wheel_speed_scale_YYYYMMDD_HHMMSS/
```

目录包含：

- 原始rosbag和录包日志；
- `calibration_raw.csv`：physical command、firmware target、STM32原始反馈、
  ROS physical actual、相机位姿、电池及时间戳；
- `speed_point_metrics.csv`：每个速度、方向和重复的稳态结果及残差；
- `scale_fit_report.json`：比例/仿射模型、R²、残差和独立验证结果；
- `scale_candidate.yaml`：只供人工审核的feedback/command scale候选；
- `scale_fit.png/pdf`及`scale_residuals.png/pdf`。

第三轮作为held-out validation，前两轮用于拟合。任何候选都保持
`automatically_applied: false`。采集失败、相机epoch变化或质量门不通过时保留
bag和已完成结果，但不得把候选写入`agvN_chassis.yaml`。

## 三车执行顺序

在Robot1完成后，再分别登录Robot2、Robot3执行同一命令并修改`--robot-id`。
不要同时运行两辆车。三份候选和bag同步到Robot3后，再统一审核是否可进入
physical→firmware command mapping实施阶段。

已经完成采集后，如只需要重新拟合和出图，不会再次运动：

```bash
rosrun multi_agv_bringup analyze_wheel_speed_scale_calibration.py \
  /完整路径/agvN_wheel_speed_scale_YYYYMMDD_HHMMSS
```

## 阶段D冻结映射

2026-09-12审核并接入公共底盘层、2026-09-13经三车物理复验后冻结的比例映射如下：

| 车辆 | 采集run | physical→firmware L/R | firmware feedback→physical L/R |
|---|---|---|---|
| Robot1 | `221354` | `0.927513947 / 0.921275775` | `1.075628349 / 1.082319540` |
| Robot2 | `222410` | `0.907894004 / 0.902080277` | `1.099449201 / 1.101910817` |
| Robot3 | `222935` | `0.912910288 / 0.908267917` | `1.091549715 / 1.099654555` |

映射只作用于底盘串口命令和反馈的单位域转换。`ChassisCommand.raw`、
`ChassisFeedback.applied/actual`及`CapabilityReport`仍使用物理`m/s`；M1、M2a
和其他方法共享同一底盘映射。本次尺度冻结不自动扩大任何既有轮速授权包络。

三车复验均为`PASSED`，机器可读冻结清单为
`config/wheel_speed_scale_stage_d_freeze_v1.yaml`。后续方法比较不得单独修改
任一车辆的command/feedback scale；如需修改，必须建立新的具名冻结版本。

三车分别在自己的主机上复验，每次只运行一辆车：

```bash
./src/multi_agv_bringup/scripts/run_wheel_speed_scale_stage_d_revalidation.sh \
  --robot-id 1 \
  --operator ETLAB \
  --confirm-area-clear \
  --confirm-wheels-on-floor \
  --confirm-emergency-stop-ready \
  --confirm-camera-physical-truth \
  --confirm-stage-d-production-mapping
```

脚本自动执行`0.06/0.08/0.10 m/s`正反向各两轮并录包，随后生成
`stage_d_revalidation.json`。稳态相机物理轮速相对目标误差默认不得超过
`0.010 m/s`，ROS物理反馈相对相机误差不得超过`0.008 m/s`，STM32名义目标
相对`command_scale × physical target`误差不得超过`0.002 m/s`。任一段失败时
返回非零状态并保留全部数据，不会修改正式参数。
