# Robot2/Robot3 相机轮速与转向自动标定

该标定只生成候选参数，不会自动修改底盘配置或 S 路径控制参数。

## 软件接口

每台车的底盘配置分别支持：

```yaml
wheel_feedback_scale_left: 1.0
wheel_feedback_scale_right: 1.0
```

STM32 返回的左右轮 `mm/s` 分别乘以上述比例后，才作为物理轮速发布并用于
里程计积分。允许范围为 `[0.5, 1.5]`。

单车 S 控制器的每车覆盖配置支持：

```yaml
geometry:
  wheel_separation: 0.114
tracker:
  angular_feedforward_scale_positive: 1.0
  angular_feedforward_scale_negative: 1.0
```

正负参数只乘曲率前馈项，不放大横向或航向反馈项。有效范围为
`[0.75, 1.25]`；实际轮速上限和反馈误差安全门不受该范围扩展影响。

Robot2 的三周期长动作标定 `20260810_195509` 与三次独立 S 路径验证
`20260810_202045`、`20260810_202854`、`20260810_203055` 已完成，以下参数
冻结为 `robot2_camera_s_frozen_v1`，不得复制到其他车辆：

```yaml
wheel_feedback_scale_left: 1.091612663
wheel_feedback_scale_right: 1.102973507
wheel_separation: 0.139284482
angular_feedforward_scale_positive: 0.780810219
angular_feedforward_scale_negative: 0.918767785
```

Robot2 启用标定比例后，编码器瞬时尖峰可能连续出现两个约 7 ms 间隔的样本。
因此 Robot2 专属 `0.09 m/s` 硬门要求连续 5 个样本确认；Robot1 保持共享
的连续 2 点门。Robot3 的冻结阈值在下文单独定义，所有车辆的 `0.12 m/s`
单帧紧急停车保持不变。

冻结参数、证据路径、三次汇总指标和排除记录以
`config/robot2_camera_s_freeze_v1.yaml` 为唯一机器可读清单。

Robot3 的高电压三周期长动作标定 `20260811_114743` 四个分项全部通过，随后
`20260811_115644`、`20260811_133644`、`20260811_133735` 三次独立 S 路径
均以 `STOP_CONFIRMED` 完成。以下联合参数冻结为
`robot3_camera_s_frozen_v1`，不得复制到其他车辆：

```yaml
wheel_feedback_scale_left: 1.134911374
wheel_feedback_scale_right: 1.137011300
wheel_separation: 0.136802843
angular_feedforward_scale_positive: 0.850009371
angular_feedforward_scale_negative: 0.883228287
```

Robot3 同时冻结横向增益 `4.0`、航向增益 `3.0` 和 `0.05 s` 曲率预瞄。
标定后的正常反馈峰值接近 `0.10 m/s`，因此其专属监控使用 `0.09 m/s`
警告、`0.105 m/s` 连续 5 帧硬门、`0.10 m/s` 持续 `0.10 s` 门；所有车辆
的 `0.12 m/s` 单帧紧急停车保持不变。冻结清单为
`config/robot3_camera_s_freeze_v1.yaml`。

方向前馈候选与有效轮距候选是联合参数。生成器先测量旧轮距下的转向响应，
再乘以 `nominal_wheel_separation / effective_wheel_separation`，防止同时应用
有效轮距和方向前馈时重复补偿。

## 自动动作

默认执行三轮以下序列：

1. `+0.03 m/s` 前进 7 秒；
2. `-0.03 m/s` 后退 7 秒；
3. `+0.025 m/s, +0.12 rad/s` 圆弧 6 秒；
4. `+0.025 m/s, -0.12 rad/s` 圆弧 6 秒；
5. 每段之间确认轮速归零至少 0.6 秒。

程序持续检查相机链路、位姿新鲜度、置信度、电池电压、命令订阅者和实际
轮速。任一安全门失败都会发送重复零速命令并退出。

普通运动的电池安全门仍为 10.5 V；为了保证辨识结果不随欠压变化，自动
标定使用更严格的 11.20 V 全程质量门。三个周期先分别拟合，再用中位数
聚合；偏离中位数门限的周期会被明确记录为异常周期。轮速、有效轮距、
正向转弯和负向转弯分别给出质量状态，最终总状态只有在四项都通过时才是
`REVIEW_REQUIRED`。

## 运行

标定 Robot1 时，由一键入口临时启动并独占 Robot1 本地底盘串口：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source ./setup_robot_ros.sh 1

./run_robot1_camera_wheel_calibration.sh \
  --confirm-test-area-clear \
  --confirm-wheels-on-floor
```

运行前不得已有 `/agv1/chassis_controller`；脚本结束或中断时会自动停止它。

标定 Robot2 时，在 Robot1 执行：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source ./setup_robot_ros.sh 1

./run_robot2_camera_wheel_calibration.sh \
  --confirm-test-area-clear \
  --confirm-wheels-on-floor
```

标定 Robot3 时，同样在 Robot1 执行：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source ./setup_robot_ros.sh 1

./run_robot3_camera_wheel_calibration.sh \
  --confirm-test-area-clear \
  --confirm-wheels-on-floor
```

需要保证车辆前后各 0.8 m、左右各 0.6 m 无障碍。输出目录包含：

- 原始 rosbag；
- `calibration_report.json`，包含每个动作段和拟合残差；
- `calibration_candidate.yaml`，仅供审核的候选参数；
- `metadata.txt` 和运行日志。

只有候选状态为 `REVIEW_REQUIRED`、拟合残差合格且独立验证动作通过后，才
允许人工写入正式配置。`REJECTED` 候选不得使用。

例外变更记录：Robot1 的 `20260811_140136` 候选因直线段内点不足被自动标记
为 `REJECTED`，但其余三个分项通过。应用户明确要求，该组联合参数以
`robot1_camera_wheel_turn_candidate_v1` 身份临时写入，仅授权执行独立 S 路径
验证，不视为冻结或正式标定。机器可读记录为
`config/robot1_camera_wheel_turn_candidate_v1.yaml`；任何跟踪或安全失败均应
回退，冻结仍要求三次独立 `STOP_CONFIRMED`。
