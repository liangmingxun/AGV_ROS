# Task 15 顶部相机软件门与实物待办

## 已由本机软件完成

- 四路 world 坐标系 `PoseStamped`：agv1、agv2、agv3、load。
- 四路独立置信度输入和低置信度拒绝。
- UDP桥把完整标定纳秒代次发布到
  `/vision/aruco/calibration_epoch`，并把世界坐标代次放入每条
  `PoseStamped.frame_id`的`world@xxxxxxxx`后缀；`header.seq`仍由ROS管理。
- 地面重新锁定/Windows会话变化时，四路位姿滤波和路径状态历史一起清空；
  已退休代次的迟到消息不会切回旧世界系。
- 单码超时后重新出现时，该路低通滤波从新测量重新初始化，不跨失效间隔平滑。
- 标记到 `base_link`/载荷坐标系的平面刚体变换。
- transformed raw 与因果一阶 filtered 位姿分离发布。
- 原始测量时间保留，`CooperativeState.header.stamp` 使用生成时间。
- 相机模式继续发布冻结的 `CooperativeState`，控制器接口不变。
- 单标记超时逐车失效；托盘直接观测不依赖三车刚体拟合。
- `/pose_provider` 不发布 `world→base_link`，避免与 odom TF 双父冲突。
- 记录器保存相机 raw/filtered/置信度，CSV 导出实际三车、支撑点和载荷位姿。
- 记录器保存标定代次与Windows配置/源码清单；正式校验拒绝一包中出现多个
  世界坐标代次。
- 默认配置同时以 `calibration_authorized=false` 和
  `extrinsics_frozen=false` 阻止误接实物。

## 接入真实相机前必须测量

1. 相机型号、驱动、分辨率、帧率和硬件/ROS 时间戳语义。
2. 相机内参与畸变参数。
3. 四固定码在线解算的 `world_T_camera`、重锁重复性和独立尺检误差。
4. 三个机器人标记到各自 `base_link` 的 x、y、yaw。
5. 载荷标记到载荷质心/坐标系的 x、y、yaw。
6. 检测置信度数值范围和可复现的接受阈值。
7. 静态噪声、动态延迟、丢帧、遮挡持续时间和完整 S 路径覆盖。
8. 相机原始位姿话题与配置文件中的四路输入名称。
9. Windows `create_manifest.py`生成的配置快照/源码哈希清单，以及Robot1
   `vision_udp_bridge.yaml`的归档副本。

上述数据归档后，复制并修改 `localization_camera.yaml`，将占位变换替换为
实测值，完成只读静态/动态标记测试。只有测试通过且配置哈希冻结后，才可同时
设置 `extrinsics_frozen=true` 并以
`camera_formal.launch calibration_authorized:=true` 启动。此 launch 只启动
定位链，不授权底盘运动。

当前外参实现由四个固定地面码在线解算，不要求预先输入四点世界坐标；
`world`原点和轴由ID 8/11/30定义。在线拟合残差只能证明内部重复性，不能
代替独立物理尺检。正式精度验收按
`task15-positioning-accuracy-acceptance.md`执行。
