# quadruped_ros2

`quadruped_ros2` 是四足机器人传统控制工作区，负责 ROS2、Gazebo Classic、有限状态机和关节 PD 力矩控制。强化学习训练、MuJoCo、TorchScript、ONNX 以及 RL 实机推理不属于本仓库。

当前 Gazebo 模型资源是 Go1，控制模块按四足机器人组织。接入其他四足机器人时，需要提供对应 URDF、关节顺序、控制器配置和运动参数。

## 工程边界

```text
遥控器串口 / ROS2 键盘节点
  -> 速度、模式和 FSM 命令
  -> quadruped_node
  -> 传统步态与关节 PD 力矩
  -> Gazebo effort controller 或 micro-ROS MCU
```

RL 训练位于独立的 `quadruped_train` 仓库，RL 仿真和部署位于独立的 `quadruped` 仓库。三个仓库没有源码导入或运行时路径依赖。

## 目录

| 路径 | 职责 |
| --- | --- |
| `src/controller/quadruped` | 控制节点、FSM、步态、运动学、关节 PD、消息服务和统一 launch |
| `src/input/serial_controller` | QLP 实体遥控器串口输入节点 |
| `src/input/kbd` | ROS2 终端键盘备用输入节点 |
| `src/robot_gazebo` | Gazebo Classic 模型、URDF、world 和 ros2_control 配置 |
| `src/libraries/logger` | C++ 日志库 |
| `src/micro-ROS-Agent/micro_ros_agent` | 上位机 micro-ROS Agent |
| `src/micro_ros_msgs` | micro-ROS 通信消息定义 |

仓库名是 `quadruped_ros2`，主控制 ROS2 包名仍为 `quadruped`，因此启动命令使用 `ros2 launch quadruped ...`。

## 环境

建议使用 Ubuntu 22.04、ROS2 Humble 和 Gazebo Classic 11。构建前需要安装 ROS2、colcon、Eigen3、Gazebo ROS2 Control 及各包在 `package.xml` 中声明的依赖。

可以使用 rosdep 补齐系统依赖：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## 构建

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

检查工作区中的包：

```bash
colcon list
ros2 pkg executables quadruped
ros2 pkg executables serial_controller
ros2 pkg executables kbd
```

## Gazebo 仿真

默认启动 Gazebo、传统控制节点和 QLP 串口遥控器：

```bash
ros2 launch quadruped traditional_pd_sim.launch.py
```

不使用串口遥控器，改用 ROS2 键盘节点：

```bash
ros2 launch quadruped traditional_pd_sim.launch.py \
  start_serial:=false \
  start_kbd:=true
```

指定遥控器串口：

```bash
ros2 launch quadruped traditional_pd_sim.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=115200
```

仿真输出固定发布到 `/joint_effort_controller/commands`，并要求 `/joint_states` 有效后才进入 FSM。

## 传统实机链路

同时打开控制节点、串口遥控器和 micro-ROS Agent 终端：

```bash
ros2 launch quadruped traditional_pd_real_terminals.launch.py
```

指定遥控器和 MCU 串口：

```bash
ros2 launch quadruped traditional_pd_real_terminals.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=115200 \
  mcu_port:=/dev/ttyUSB1 \
  mcu_baud_rate:=115200
```

也可以在同一终端直接启动控制链路：

```bash
ros2 launch quadruped traditional_pd_control.launch.py \
  output_mode:=1.0 \
  lock_output_mode:=true \
  start_micro_ros_agent:=true
```

`output_mode` 含义：

| 值 | 输出话题 |
| --- | --- |
| `0.0` | `/joint_effort_controller/commands`，用于 Gazebo |
| `1.0` | `/real_joint_effort_controller/commands`，用于 MCU |

实机运行前必须确认串口设备、关节顺序、零位、方向、力矩限制和急停链路。不要直接沿用仿真参数驱动未完成标定的机器人。

## FSM 按键

| 按键 | 作用 |
| --- | --- |
| `f` | 允许 Init 起身 |
| `b` | 进入 `Tort` 小跑模式 |
| `h` | 进入 `FixedStand` |
| `g` / `c` | 进入 `Jump` |
| `i` | 进入 `Deinit` |
| `F` | 回到 `Init` |
| `a` / `u` | 进入 `FreeAngle` |
| `d` | 刷新当前状态参数 |

## 常用检查

```bash
ros2 topic echo /input/mode
ros2 topic echo /joystick/command
ros2 topic echo /joint_states
ros2 topic echo /joint_effort_controller/commands
```

运行包测试并查看失败详情：

```bash
colcon test
colcon test-result --verbose
```

## 仓库职责

- 本仓库只维护传统 PD ROS2 链路。
- 不在本仓库加入 RL 策略、MuJoCo 或训练代码。
- 新机器人资源通过新的机器人描述和配置接入，不覆盖已有机器人配置。
- 与 RL 仓库之间只交换明确的数据约定，不直接引用对方源码。
