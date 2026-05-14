# 四足机器人控制模块

## 使用方法

### 启动四足机器人控制节点
```bash
# 实机模式
ros2 launch quadruped quadruped_robot.launch.py

# 仿真模式
ros2 launch quadruped quadruped_robot.launch.py use_sim_time:=true
```

### 话题
- **发布**: `/robot_state` - 机器人状态信息
- **订阅**: `/cmd_vel` - 速度命令
- **订阅**: `/imu/data` - IMU传感器数据
- **订阅**: `/motor_data` - 电机反馈数据
