# 正式实验门（实验 2a，后续实验共用）

状态：**关闭。相机物理标定尚未完成，不允许正式统计。**

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
