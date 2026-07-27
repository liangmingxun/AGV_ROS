# AGV_WS

当前开发入口由 `agv_msgs`、`chassis_controller`、`multi_agv_control`、
`multi_agv_bringup` 和 `multi_agv_analysis` 组成。平台层已覆盖三车命名空间、
冻结接口、单车串口执行、物理限幅、平滑降额、预里程计与安全预检；M1/M2a/M4
上层和 R1–R4 下层已接入仅限 fake transport 的正式算法入口；可复现实验记录、
因果转换、合法性审计和基础指标链也已实现。

这不等于正式实物系统已经完成：顶部相机定位（Task 15）、完整 M2b（Task 16）、
正式物理算法入口和分阶段签字门禁（Task 18）仍未完成，所有正式算法硬件授权
保持关闭。开始实车前请先阅读
[`docs/理论-软件-实物覆盖审计.md`](docs/理论-软件-实物覆盖审计.md) 和
[`docs/三车Ubuntu移植与上车检查手册_v1.0.md`](docs/三车Ubuntu移植与上车检查手册_v1.0.md)。

旧 `nav`、建图和键盘控制包保留作历史功能，但不属于冻结控制链，不能在正式测试时
与 `multi_agv_bringup` 同时发布运动命令。

### [v1.1.0] - 2026-03-18
- 更新：成功移雷达功能包；发现步进电机驱动器通信过程会阻塞USB通信
- 相关文件：`src/hector_slam-noetic-devel。src/lsx10`

### [v1.1.1] - 2026-03-20
- 更新：成功移植gmapping。使用雷达与里程计融合并成功建图
- 相关文件：`src/mycar_description、src/nav、src/ros_arduino_bridge-indigo-devel`  

### [v1.1.2] - 2026-03-20
- 更新：实现自主导航建图，效果较好
- 相关文件：`src/nav/launch、src/nav/maps、src/nav/param` 

### [v1.1.3] - 2026-03-21
- 更新：硬件稳定版本；删除不必要功能包（src/ros_arduino_bridge-indigo-devel/）
- 相关文件：`src/`
