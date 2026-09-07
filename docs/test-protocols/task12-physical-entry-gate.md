# Task 12 后实物准入门槛

## 当前结论

Task 8–12 的软件门槛已经通过。下一步是无托盘、低风险的实物验证，
不是正式实验，也不得直接放置刚性托盘。仓库内路径、支撑偏置、初始
坐标变换、跟踪增益和能力预留仍标为软件验证值，所有真实命令门控默认关闭。

## 启动边界

- `odom_pretest.launch` 只允许本机三车 `fake` 验证，不接受串口模式。
- 实物分布式运行时，各 Robot 只启动自己的 `carX_*.launch`。
- Robot1 只启动 `central_odom_pretest.launch`，不得在 Robot1 再启动
  Robot2/Robot3 的本地底盘节点。
- 首次启动中央节点必须保持 `enable_commands:=false` 和
  `enable_derating:=false`，先检查状态链。

Robot1 的只读检查入口：

```bash
roslaunch multi_agv_bringup central_odom_pretest.launch \
  platform_transport_type:=serial \
  enable_commands:=false \
  enable_derating:=false
```

## 解除运动门控前必须完成

1. 实测并冻结 `path_s_curve.yaml` 的场地路径尺寸。
2. 实测并冻结 `support_geometry.yaml` 的三个物理支撑偏置。
3. 测量 `localization_odom.yaml` 中三个 `world_to_odom` 初始变换以及
   `base_to_support`，确认 frame 和左右法向约定。当前冻结值为 Robot1
   `x=-0.01783 m`、Robot2/3 `x=+0.09908 m`，三车均为
   `y=0, yaw=0`：`base_link` 是左右驱动轮轴线中点，`support_link`
   是顶部转盘中心。
4. 用举升数据确定 `capability_mapping.yaml` 的模型、映射、同步和链路预留，
   不能保留零预留作为实车参数。
5. 以 Robot1 单车举升方式验证二维跟踪命令方向、左右轮符号、停止命令和
   状态失效后的零命令；Robot2 再单独验证位置触发降额。
6. 保存参数快照、Git SHA、rosbag、操作人、日期和人工急停检查结果。

不得仅通过 launch 参数绕过门控。通用路径、支撑几何和中央控制器的授权继续
保持 false；第一次三车运动只使用独立的有界无载入口及其专用授权。

## 实物验证顺序

1. Robot1 举升、无托盘、恒定参考最低速，核对三层命令和投影状态。
2. Robot2 举升、无托盘，核对实际进度触发降额、能力斜坡和恢复。
3. Robot1 单车落地低速短 S 形，检查局部投影无分支跳变。
4. 三车无托盘共同启动/停止和最低速 S 形。
5. 只有上述阶段全部通过后，才进入轻载托盘预实验。

任何投影无效、命令发布者重复、TF 冲突、轮向错误、持续异常限幅或人工中止，
都必须停止进入下一阶段。

Robot1 的第 3 步必须使用独立的
`robot1_single_s_pretest.launch` 和
`test-protocols/robot1-single-s-pretest.md`。该入口只发布 Robot1 命令，
不得用三车 `central_odom_pretest.launch` 模拟单车运行。

Robot2 的第 2 步必须使用独立的
`robot2_raised_derating_pretest.launch` 和
`test-protocols/robot2-raised-derating-pretest.md`。该入口只发布 Robot2
降额命令；底盘运动由有界定时直线工具负责。不得将通用三车 supervisor 的
软件验证授权直接改为实物授权。

Robot1 单车 S 形和 Robot2 举升降额通过后，必须先执行
`test-protocols/three-car-readonly-entry.md`。中央只读模式不得注册任何
`/agvX/chassis_command` Publisher；第一次只验证分布式拓扑，现场参数冻结后
再用 `check_three_car_readonly_gate.py --require-valid-state` 验证全部状态
有效。两次只读门槛之间不允许三车运动。

三车无载阶段的冻结摆位见
`test-protocols/three-car-unloaded-fixture.md`：Robot1 前、Robot2 左后、
Robot3 右后，三个转盘中心构成边长 `0.30 m` 的等边三角形。该几何只用于
无实体托盘预检；正式载荷实验仍必须重新测量真实接触点。

第二次只读门槛连续通过后，第一次三车共同运动必须使用
`three_car_unloaded_bounded_pretest.launch` 和
`test-protocols/three-car-unloaded-bounded-pretest.md`。该入口只允许
`0.05 m/s`、连续 `1.00 m` 的共同 S 形运动/自动停车，并要求 rosbag 已连接；不得把
`central_odom_pretest.launch` 的 `enable_commands` 改为 true 代替它。
三台底盘还必须使用 `command_timeout_seconds: 0.20` 的独立 ROS 命令心跳
超时保护和 `sensor_feedback_timeout_seconds: 0.15` 的锁存式 STM32 反馈
看门狗；只读门槛与有界入口同时要求 `serial_receive_stamp` 新鲜且
`packet_seq` 持续推进。这些保护依赖 Linux 和串口零速命令能够到达 STM32，
仍不能代替物理急停或 STM32 固件内部看门狗。

推荐用 `three-car-unloaded-bounded-pretest.md` 中的四终端脚本流程启动。每车
底盘脚本会登记 hostname、IP、干净工作区和 Git SHA；Robot1 总控脚本只在
三车 SHA 完全一致、拓扑门槛通过、里程计复位且完整有效状态连续通过两轮后，
才创建参数快照、manifest、bag 并调用有界运动入口。
