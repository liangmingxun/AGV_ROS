# AGV_WS

当前正式开发入口是 `multi_agv_bringup`、`chassis_controller` 和 `agv_msgs`。
平台基础层已覆盖三车命名空间、冻结接口、单车串口执行、物理限幅、平滑降额和
预里程计；完整协同控制与正式实验尚未实现。开始实车前请先阅读
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
