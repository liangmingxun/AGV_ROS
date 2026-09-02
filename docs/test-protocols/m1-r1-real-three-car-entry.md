# M1+R1 真实三车首轮入口

本入口只用于当前空载 0.40 m 三支撑队形，方法固定为 M1+R1，定位固定为
`CAMERA_IMU_WHEEL_FUSED_CLOSED_LOOP`。它不会授权 M2a、M2b 或真实载荷。

## 1. 编译与软件观察者验证

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

- Windows 相机发送端、Robot1 视觉桥和三车相机融合节点已经运行；
- 三台底盘均用 `start_three_car_chassis.sh 1|2|3` 启动，Git SHA 一致；
- Robot1 当前代码已提交且工作树干净（`experiment_data/` 可保留）；
- 三车已人工放成约 0.40 m 的空载支撑三角形并完全位于相机视野；
- `/vision/aruco/alive` 为 `True`，三车 fused pose 和
  `/multi_agv/cooperative_state` 连续有效；
- Windows 端在本次运行前生成新的发送器清单并复制到 Robot1：

```bat
python create_manifest.py --config config.yaml --test-stage m1_r1_serial
```

## 3. 唯一实车执行命令

在 Robot1 执行；把清单路径、操作者和配对区组换成真实值：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
source devel/setup.bash

./src/multi_agv_bringup/scripts/run_m1_r1_serial_unloaded.sh \
  --windows-manifest /home/etlab/manifest/task15_manifest.json \
  --operator ETLAB \
  --pair-block B01 \
  --confirm-area-clear \
  --confirm-wheels-on-floor \
  --confirm-unloaded-40cm-fixture
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
PNG/PDF 草图。只有脚本打印 `M1+R1 SERIAL RUN COMPLETE` 且
`run_meta.json` 中 `valid_run=true` 才算数据链完整；它仍不等于正式统计批准。
