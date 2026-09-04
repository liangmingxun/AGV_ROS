# AGV_WS

当前开发入口由 `agv_msgs`、`chassis_controller`、`multi_agv_control`、
`multi_agv_bringup` 和 `multi_agv_analysis` 组成。平台层已覆盖三车命名空间、
冻结接口、单车串口执行、物理限幅、平滑降额、预里程计与安全预检；M1/M2a/M4、
完整 M2b 和 R1–R4 已接入仅限 fake transport 的算法入口；可复现实验记录、
因果转换、合法性审计、配置审批及有界评价窗口链也已实现。

开发和实车检查采用按改动影响面分级的最小验证策略；不要把历史 Stage 检查、完整
测试套件或正式统计重复机械地放到每轮 pilot 前。具体规则见
[`docs/开发验证与实车检查分级规范.md`](docs/开发验证与实车检查分级规范.md)。

这不等于正式实物系统已经完成：顶部相机定位的软件适配边界和合成掉线测试已经
实现，但真实相机/标记/外参尚未安装标定；Task 18 的 Stage A–D 与正式门文档已经
建立但尚未执行签字，配置注册表只允许非统计软件演练，所有正式算法硬件授权保持关闭。
开始实车前请先阅读
[`docs/理论-软件-实物覆盖审计.md`](docs/理论-软件-实物覆盖审计.md) 和
[`docs/三车Ubuntu移植与上车检查手册_v1.0.md`](docs/三车Ubuntu移植与上车检查手册_v1.0.md)。
无需真实参数的软件辅助工具（标定候选、配对批次、统计绘图、Stage 证据、主机健康
检查和 observer-only ROS 监视器）见
[`docs/参数无关实验工具链使用说明.md`](docs/参数无关实验工具链使用说明.md)。

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
