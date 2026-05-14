# IMU传感器数据
std_msgs/Header header

# 加速度计数据 (m/s^2)
geometry_msgs/Vector3 linear_acceleration

# 陀螺仪数据 (rad/s)
geometry_msgs/Vector3 angular_velocity

# 姿态四元数 (可选)
geometry_msgs/Quaternion orientation

# 协方差矩阵 (可选)
float64[9] orientation_covariance
float64[9] angular_velocity_covariance
float64[9] linear_acceleration_covariance