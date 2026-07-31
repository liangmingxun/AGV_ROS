# multi_agv_vision_bridge

Task 15专用UDP v3原始标记位姿桥。固定监听
`192.168.6.101:15001`并只接受`192.168.6.100:15000`。发布四路
`geometry_msgs/PoseStamped`与四路`std_msgs/Float64`；不执行
tag→target变换、滤波、TF、CooperativeState或底盘控制。Robot1按本地
配置复核动态四地码标定身份（8/11/15/30、150 mm、角色顺序）、实体
ID（1/2/3/0）、平面高度、残差和样本数。

桥还锁存发布`/vision/aruco/calibration_epoch`（`std_msgs/UInt64`），并
把由Windows会话和地面锁定时间生成的非零代次写入每条位姿的
`frame_id`（格式`world@xxxxxxxx`）。ROS1会自动改写`header.seq`，因此
它不承载标定语义。代次用于阻止Task 15把两次
`world_T_camera`解算结果混在同一条滤波轨迹中。
