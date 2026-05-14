# 步态控制命令
std_msgs/Header header

# 步态类型 (0=站立, 1=行走, 2=小跑, 3=其他)
uint8 gait_type

# 运动参数
float32 linear_velocity    # 前进速度 (m/s)
float32 angular_velocity   # 旋转速度 (rad/s)
float32 lateral_velocity   # 横向速度 (m/s)

# 步态参数
float32 step_height       # 步高 (m)
float32 step_length       # 步长 (m)