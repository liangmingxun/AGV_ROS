# Task 15当前软件验收边界

## 已完成

- Windows四固定码在线米制世界系、畸变点模型和183 mm实体平面求交；
- `DICT_4X4_50`与实体ID `1/2/3/0`、地面ID `8/11/15/30`；
- Windows仅输出`world_T_tag`，载荷暂时禁用；
- UDP v3 CRC、固定源端点、会话、序号、时间和四实体校验；
- Robot1同时校验地面标定模式、ID角色、150 mm边长、平面高度和质量；
- 四路PoseStamped/Float64冻结输入；
- 现有Task 15适配器继续负责SE(2)、raw/filtered、confidence和超时；
- 现有状态估计器继续负责CooperativeState。

本轮离线检查结果：

```text
Python静态编译：通过
四固定码米制几何与183 mm高度求交：通过
协议/CRC/几何/ROS接口合同用例：通过（20项）
standalone与工作树ROS桥关键源文件一致：通过
platform-foundation-linux完整catkin_make：通过
```

## 尚未完成或不能由软件代替

- Windows工业相机真实取流和现场四码锁定；
- 固定码放置后的尺检误差、动态精度与遮挡测试；
- 三台AGV真实`tag_T_base_link`；
- 载荷码高度与`tag_T_load_center`；
- Task 15配置中的零值占位变换冻结；
- ROS运行图与真实UDP收包检查；
- 任何硬件运动授权。

所以该版本可进入相机静态/动态只读联调，但不能仅凭本报告授权车辆运动。
