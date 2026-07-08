# 控制指令模块
uint8 CTRL_MODE_KEYBOARD=0
uint8 CTRL_MODE_ALGORITHM=1

float32 linear_x
float32 linear_y    # 您保留了此字段，可能是为了兼容其他机型
float32 angular_z
uint8 control_mode  # 用于指明当前指令来源
string robot_ip       # 用于记录发送指令的AGV的IP地址
string robot_id       # 用于记录发送指令的AGV的ID
string sender_ip