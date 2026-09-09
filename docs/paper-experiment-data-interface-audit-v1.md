# 论文实物实验字段—话题覆盖审计 v1

审计日期：2026-09-02。范围：空载 M1+R1、M2a+R1、M2b 三车对比。
本表不改变任何控制变量含义；现有 raw bag 是唯一原始证据，逻辑 CSV 均由
`process_experiment_run.py` 离线重建。

| 规范数据 | 原始话题/字段 | 发布者 | 频率 | 覆盖状态 |
|---|---|---|---:|---|
| 三车/支撑/等效载荷位姿、路径状态与有效性 | `/multi_agv/cooperative_state` | `path_state_estimator` | 100 Hz | 已覆盖；空载 load 为三支撑刚体拟合 |
| 原始、因果滤波、融合车体位姿 | `/camera/world/agvN_tag_pose`、`/pose_provider/agvN/base_pose_{raw,filtered,fused}` | vision bridge、pose provider、fusion | 30/100 Hz | 已覆盖；ID0 未启用时不得伪造载荷观测 |
| 公共/支撑点路径参考 | `/multi_agv/path_reference` | formal algorithm | 100 Hz | 已覆盖 |
| 实际/执行参考、公共边界、上下层输入 | `/multi_agv/controller_state` | formal algorithm | 100 Hz | 已覆盖 |
| M1/M2a 边界、风险、z/upsilon/phi、R1 内部量 | `/multi_agv/formal_algorithm_state` | formal algorithm | 100 Hz | 90字段固定布局，已覆盖 |
| M2b z/w/phi、映射、固定边界审计量 | `/multi_agv/m2b_algorithm_state` | formal algorithm | 100 Hz | 66字段固定布局，已覆盖 |
| 车端轮速/加减速能力、降额、限幅 | `/agvN/capability_report` | chassis controller | 100 Hz | 已覆盖 |
| 降额命令与路径位置事件 | `/agv2/derating_command`、`/multi_agv/experiment_state` | experiment supervisor | 10 Hz | 已覆盖；触发源为实际推进位置 |
| raw/applied/actual、command/applied/packet序列 | `/agvN/chassis_command`、`/agvN/chassis_feedback` | formal algorithm、chassis controller | 100 Hz | 已覆盖，可因果对齐 |
| IMU、里程计、电池 | `/agvN/{imu,odom,chassis_feedback}` | chassis controller | 100 Hz | 已覆盖 |
| 统一评价窗口 | `/multi_agv/experiment_state.evaluation_active` | experiment supervisor | 10 Hz | 已覆盖；缺失时禁止正式统计 |
| Git、配置、主机/版本、运行身份 | `manifest.yaml`、`run_meta.json` | experiment recorder | 每次运行 | 已覆盖；未知 STM32 固件显式写 `unreported` |

## 本次补齐的软件出口

- `run_meta.json`：运行身份、Git/配置哈希、方法、区组、载荷和定位状态。
- `state.csv`、`upper_layer.csv`、`capability.csv`、`wheel_chain.csv`、
  `lower_layer.csv`、`m2b_internal.csv`：从 causal aligned samples 生成的论文逻辑视图。
- `summary_metrics.json`：复用冻结的统一指标实现。
- `plots/figure2_*.{png,pdf}` 至 `figure7_*.{png,pdf}`：同一入口生成的基础图。

## 仍属于后续阶段的内容

- 真实载荷 ID0 位姿及 tag-to-load 变换尚未启用；当前只能称等效载荷。
- M2a+R1与M2b已具备独立serial启动入口；当前方法授权只覆盖与M1相同的空载、
  无Robot2降额R0.7整圆。Robot2正式降额授权仍保持关闭，实验2a/2b的降额运行
  仍须先通过当前 `0.15→0.06→0.15 m/s` 举升预检并单独授权。
- `minimum_battery_voltage`、raw轮速和反馈新鲜度是工程安全门，不属于论文控制律。
- 正式统计批准注册表仍未放行；当前输出用于 observer/调试证据。
