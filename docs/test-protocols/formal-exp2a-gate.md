# 正式实验门（实验 2a，后续实验共用）

状态：**Robot2实车降额已由操作者授权；正式统计审批仍关闭。**

顶部相机、30 cm等边支撑队形及M1+R1串行pilot已经完成实测。M2a+R1串行
软件入口已补齐。操作者于2026-09-09明确解除Robot2
`0.15→0.06→0.15 m/s`公共实车降额锁；本次授权不冒充举升预检结果，后续
运行清单仍应如实保留授权依据。正式统计审批没有因此自动开启。

当前圆形降额入口统一使用 R=0.7 m 顺时针平滑进出路径和 2.5 s 缓启动。
Robot2 在等效载荷实际进度 `1.50 m` 开始用 0.50 s 从 `0.15 m/s`
降到 `0.06 m/s`，在 `3.50 m` 开始用 0.50 s 恢复。两个触发点均位于
约 `0.60--4.60 m` 的恒定曲率圆弧内。M1、M2a、M2b分别使用：

- `run_m1_r1_serial_unloaded_circle_r0p7_smooth_exit_derating.sh`
- `run_m2a_r1_serial_unloaded_circle_r0p7_smooth_exit_derating.sh`
- `run_m2b_serial_unloaded_circle_r0p7_smooth_exit_derating.sh`

三者共享路径、执行层、降额命令与记录链；M2a仍不把能力用于上层，M2b仍使用
英文论文自己的完整上下层。公共授权仍与三个方法的独立执行范围分开记录。

- [ ] Stage A–D 均有签字证据且无未关闭问题。
- [ ] 顶部相机内外参、world、三车标记到车体变换完成并冻结。
- [ ] 载荷标记接口已标定；在此之前载荷正式位姿门不得开启。
- [ ] 四实体置信度、超时、遮挡和 raw/filtered 因果规则实测冻结。
- [ ] Windows发送器清单（配置快照与源码SHA）和Robot1视觉桥配置已随运行归档。
- [ ] 评价窗口内只有一个非零世界坐标代次；任何地面重新锁定、Windows会话
      重启或代次变化均整次判废并重跑。
- [ ] 通过 `task15-positioning-accuracy-acceptance.md` 的独立静态、距离、
      航向、重复重锁和动态轨迹验收，阈值已预注册而非事后选择。
- [ ] 方法配置、能力预留、评价起止进度、随机化和排除规则预注册。
- [ ] 将精确 SHA256 加入 `approved_config_registry.yaml`，状态改为 `approved`，
      且仅经审核后设置 `formal_statistics_authorized: true`。
- [ ] 完成一次非统计 dress rehearsal，录包清单、配置审批和有界评价窗口均通过。

只有以上全部签字后，才可在记录启动时同时设置
`require_approved_config:=true` 与 `formal_statistics_requested:=true`。
软件不会自动完成或绕过这些物理门。

批准 ID：`________`　日期：`________`　负责人：`________`　安全复核：`________`
