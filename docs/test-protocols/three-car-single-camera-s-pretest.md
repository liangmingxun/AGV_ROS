# 三车通用相机闭环单车 S 轨迹测试

本入口一次只允许一台车运动。Robot1（`192.168.6.101`）运行 ROS Master、
相机 UDP 桥、Task 15 定位融合、控制器与 rosbag；Robot2/3 只在各自电脑上
运行串口底盘节点。Windows 相机端为 `192.168.6.100`。

三个入口使用同一控制器、同一冻结参数和相同安全门：

```text
./run_robot1_camera_s_closed_loop.sh
./run_robot2_camera_s_closed_loop.sh
./run_robot3_camera_s_closed_loop.sh
```

每个命令都必须附加：

```text
--confirm-test-area-clear --confirm-wheels-on-floor
```

需要在当前终端执行 ROS 命令时，三台电脑分别用一行初始化：

```text
source ./setup_robot_ros.sh 1
source ./setup_robot_ros.sh 2
source ./setup_robot_ros.sh 3
```

Robot1 的 Task 15 定位节点可直接用 `./start_camera_formal.sh` 启动。

Robot2/3 测试前，分别在对应电脑运行：

```text
rosrun multi_agv_bringup start_three_car_chassis.sh 2
rosrun multi_agv_bringup start_three_car_chassis.sh 3
```

通用控制器由 `robot_index` 选择 `/agvN/*` 和
`/pose_provider/agvN/base_pose_fused`。入口拒绝并发的底盘命令发布者、错误
`robot_id`、静止条件不满足、低于 10.5 V、电机实际轮速超限、定位陈旧或
标定 epoch 变化。结果分别保存到
`experiment_data/robotN_camera_s_closed_loop/<时间戳>/`。
