# multi_agv_vision_bridge

Task 15专用UDP v3原始标记位姿桥。固定监听
`192.168.6.101:15001`并只接受`192.168.6.100:15000`。发布四路
`geometry_msgs/PoseStamped`与四路`std_msgs/Float64`；不执行
tag→target变换、滤波、TF、CooperativeState或底盘控制。Robot1按本地
配置复核动态四地码标定身份（8/11/15/30、150 mm、角色顺序）、实体
ID（1/2/3/0）、平面高度、残差和样本数。
