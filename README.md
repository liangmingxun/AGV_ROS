# AGV_WS

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