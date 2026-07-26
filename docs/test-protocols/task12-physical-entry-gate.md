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
   `base_to_support`，确认 frame 和左右法向约定。当前 CAD/实测冻结的
   平面值为 `x=-0.01783 m, y=0, yaw=0`：`base_link` 是左右驱动轮
   轴线中点，`support_link` 是顶部转盘中心。
4. 用举升数据确定 `capability_mapping.yaml` 的模型、映射、同步和链路预留，
   不能保留零预留作为实车参数。
5. 以 Robot1 单车举升方式验证二维跟踪命令方向、左右轮符号、停止命令和
   状态失效后的零命令；Robot2 再单独验证位置触发降额。
6. 保存参数快照、Git SHA、rosbag、操作人、日期和人工急停检查结果。

以上项目签字通过后，才能逐项把对应 YAML 的
`hardware_execution_authorized` 改为 `true`。不得仅通过 launch 参数绕过门控。

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
