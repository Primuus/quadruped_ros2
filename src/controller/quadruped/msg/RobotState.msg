# 机器人状态信息
std_msgs/Header header

# 电机状态
float32[12] q              # 当前关节角度
float32[12] dq             # 当前关节速度
float32[12] tau            # 当前关节力矩

# 足端位置
geometry_msgs/Point[4] foot_positions  # 足端位置(世界坐标系)

# 姿态信息
geometry_msgs/Quaternion orientation   # 机器人姿态

# IMU数据
geometry_msgs/Vector3 linear_acceleration   # 线性加速度
geometry_msgs/Vector3 angular_velocity      # 角速度

# 步态信息
uint8 gait_type            # 当前步态类型
uint8[4] gait_phase        # 各腿相位

# 机器人状态
bool is_moving             # 是否在运动
bool emergency_stop        # 是否紧急停止