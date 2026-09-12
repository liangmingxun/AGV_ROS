# 实测速度锚定方案：离线验证，暂不部署

验证对象：候选 v_exec = clamp(LPF(v_measured) + dt*u_R1, lower, upper)。
滤波采用因果一阶 LPF，alpha=1-exp(-dt/tau)，tau=.04/.10/.20 秒。
不改生产节点、运行配置、历史 CSV 或图片；没有启动 ROS 或实车。
该候选不是已部署方案，也不是论文控制律有效性证明。

## 真实数据回放

使用 M1 0.75 原版 20260912_144750 的有效低速稳定段 1400 个样本，
冻结实测状态与已记录 R1 加速度。执行投影保留映射能力界和 Robot2
降额时动态上界减 .005 的单侧执行余量。
回放不重新生成闭环状态，不据此计算“优化后真实进度误差”。

tau=.10 秒时：

| 指标 | Robot1 | Robot3 |
|---|---:|---:|
| 原版平均执行通道参考 m/s | .09899 | .09062 |
| 锚定候选平均执行通道参考 m/s | .07985 | .07951 |
| 原版命令逐样本变化 RMS m/s | .000649 | .000664 |
| 候选命令逐样本变化 RMS m/s | .000584 | .000598 |

候选能让执行状态靠近实测速度，相比未滤波的 v_measured+dt*u
也明显降低逐样本变化；但相对当前积分适配器只略平滑。
tau=.04 秒反而比当前命令更抖。缩小信号差不等于降低真实位置误差。

## R1 简化闭环敏感性测试

直接编译现有 lower_channel_controller.cpp，默认参数与 exp3_R1 一致，
两种适配器共用公共参考、3.2 秒软启动、1 秒降/恢复斜坡、
纵向反馈 1.0、相同需求限幅；保留实测速度作为 R1 输入。
假设一维一阶底盘模型，增益 [.85,1.,1.13,1.25]，
响应时常 [.08,.15,.25] 秒，加入 9/17 Hz 速度测量扰动，
比较三个滤波时常，共 36 个候选组合。

36 个组合的低速进度 RMS 均高于对应积分适配器，
33 个组合峰值速度更高。例：增益1.13、底盘时常.15、滤波时常.10，
原版低速进度 RMS .80 mm，候选12.94 mm。
这些数字只用于暴露候选对增益/延迟的敏感性，不能解释为实际车
误差预测：模型未经系统辨识，没有完整三车平面几何、M1上层、
STM32 速度环、融合有效性或车辆软启动反馈混合。
原版模型也未复现本次实车12 mm偏差，因此不具备实车优劣定量资格。

## 结论

暂不部署“用滤波实测速度直接替代独立积分状态”的方案。
执行参考与实测速度不同可能含必要的底盘/跟踪补偿，不能只因
两条曲线不同就删除补偿状态。现有数据不支持保证真实误差下降。
下一步先辨识执行通道、平面跟踪需求与实测路径速度之间的关系，
或在已有三车闭环模型中重现原版偏差，再验证有反馈校正的积分/
观测器方案，保持方法间共享适配层。不再盲目加纵向增益。

## 可复现命令

在 worktree 根目录执行；只生成临时离线验证程序，不启动实车：

```bash
VALIDATION_DIR=$(mktemp -d /tmp/agv-anchor-validation.XXXXXX)
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -Isrc/multi_agv_control/include \
  src/multi_agv_control/test/velocity_anchor_validation.cpp \
  src/multi_agv_control/src/lower_channel_controller.cpp \
  -o "$VALIDATION_DIR/validate"
python3 src/multi_agv_control/test/validate_velocity_anchor.py \
  "$VALIDATION_DIR/validate" \
  experiment_data/formal_serial_unloaded/m1_r1_circle_r0p7_cw_smooth_exit_0p10_derating_0p75_pilot_20260912_144750
```

C++ 程序同时断言启动初始化、重置、上下限即时投影和 NaN/Inf、
无效 dt 的拒绝行为。拒绝返回NaN仅用于离线检查；没有发布端接口。
Python做编译检查并检查回放数量与模拟数值有效性。
