# 摇杆控制命令消息
# 用于传输归一化后的摇杆数据

std_msgs/Header header  # 时间戳和坐标系

# 左摇杆 (通常用于移动控制)
float32 x1              # 左摇杆 X 轴 [-1.0, 1.0]，左右方向
float32 y1              # 左摇杆 Y 轴 [-1.0, 1.0]，前后方向

# 右摇杆 (通常用于姿态/视角控制)
float32 x2              # 右摇杆 X 轴 [-1.0, 1.0]
float32 y2              # 右摇杆 Y 轴 [-1.0, 1.0]

# 原始值 (未归一化的原始传感器数据)
float32 x1_raw
float32 y1_raw
float32 x2_raw
float32 y2_raw

# 按键信息
string key              # 按键字符 (如果有)
