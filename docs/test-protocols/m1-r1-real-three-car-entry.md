# M1+R1 真实三车首轮入口

本入口只用于当前空载 0.30 m 三支撑队形，方法固定为 M1+R1，定位固定为
`CAMERA_IMU_WHEEL_FUSED_CLOSED_LOOP`。它不会授权 M2a、M2b 或真实载荷。

## 1. 每个新提交一次的编译与软件观察者验证

以下步骤只在相关代码提交发生变化后执行一次。同一提交已经验证并同步到三车后，
后续 pilot 不重复执行。仅修改文档时不执行编译或 observer。更详细的最小测试规则见
[`开发验证与实车检查分级规范`](../开发验证与实车检查分级规范.md)。

在 Robot1 当前 worktree 执行：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
catkin_make --pkg multi_agv_control multi_agv_analysis multi_agv_bringup
source devel/setup.bash

./src/multi_agv_bringup/scripts/run_m1_r1_fake_observer.sh
```

fake/observer 通过只证明消息链、录包门和离线处理可以工作，不是实车证据。

## 2. 实车运行前置条件

本节由启动脚本和现场确认在每轮运行时执行。Stage A-D、单车预检、完整相机标定、
举升轮标定和降额预检不是每轮前置条件；只在相关硬件、配置或环境发生变化，或者出现
对应异常后重做。

操作者于 2026-09-04 报告 Robot1/2/3 均完成 `0.15 m/s` 实测，急停有效，且无
异常振动、失控或通信故障；因此当前 M1+R1 pilot 软件授权已开启。该授权依据是
人工实测报告，不是 CapabilityMapper 扫描或软件测试结果。

- Windows 相机发送端已经运行；Robot1 使用 `start_three_car_chassis.sh 1`
  同时启动底盘、视觉桥和三车相机融合节点；
- Robot2/3 分别用 `start_three_car_chassis.sh 2|3` 启动，三车 Git SHA 一致；
- Robot1 当前代码已提交且工作树干净（`experiment_data/` 可保留）；
- 三车已人工放成约 0.30 m 的空载支撑三角形并完全位于相机视野；
- `/vision/aruco/alive` 为 `True`，三车 fused pose 和
  `/multi_agv/cooperative_state` 连续有效；
- 本工程验证入口不强制归档 Windows 发送器清单；这不会影响闭环、录包数据、
  指标或绘图，但 `manifest.yaml` 会明确记录该清单未提供。

## 3. 唯一实车执行命令

在 Robot1 执行；把清单路径、操作者和配对区组换成真实值：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
source devel/setup.bash

./src/multi_agv_bringup/scripts/run_m1_r1_serial_unloaded.sh \
  --operator ETLAB \
  --pair-block B01 \
  --confirm-area-clear \
  --confirm-wheels-on-floor \
  --confirm-unloaded-30cm-fixture
```

脚本依次检查三台 serial 底盘和 SHA、相机链、5 秒只读门；随后启动等效载荷
状态、M1+R1、统一评价窗口和录包器。录包器真正订阅所有必需话题之前，算法只
发布零命令。运行中若融合状态、能力、底盘反馈、录包心跳或电池安全门失效，
算法 fail-zero。

## 4. 输出和判定边界

输出目录为：

```text
experiment_data/formal_serial_unloaded/m1_r1_serial_YYYYmmdd_HHMMSS/
```

目录中保留唯一原始证据 bag，并自动生成 `manifest.yaml`、`run_meta.json`、
`validation.json`、`summary_metrics.json`、六类论文逻辑 CSV 以及图 2--7 的
PNG/PDF 草图；校验通过后还会自动生成新版中文论文图到：

```text
paper_figures/01_M1_R1_complete_method/
```

无需再手动执行出图命令。只有脚本打印 `PAPER FIGURES COMPLETE`、
`M1+R1 SERIAL RUN COMPLETE` 且
`run_meta.json` 中 `valid_run=true` 才算数据链完整；它仍不等于正式统计批准。
