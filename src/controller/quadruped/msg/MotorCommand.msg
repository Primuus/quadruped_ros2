# 电机控制命令
std_msgs/Header header

# 期望关节角度 (12个电机)
float32[12] q_des

# 期望关节速度
float32[12] dq_des

# 期望关节力矩
float32[12] tau_des

# 位置增益
float32[12] kp

# 阻尼增益
float32[12] kd