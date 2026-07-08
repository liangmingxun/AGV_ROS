# 机器人主状态消息，整合所有模块
Header header           # 标准头，包含时间戳
ImuData imu     # 引用IMU模块
ControlCommand ctrl_cmd # 引用控制指令模块
float32[3] motor_speeds # 实际电机速度（