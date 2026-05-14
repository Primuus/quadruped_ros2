# Quadruped Traditional PD Control

这个工作区只保留以传统 PD 为核心的控制链路，包含：

- `quadruped`：传统 PD FSM 控制节点
- `serial_controller`：QLP 遥控器串口输入，同时发布遥控器按键和摇杆
- `kbd`：本机键盘备份输入
- `robot_gazebo`：Go1 Gazebo Classic 仿真
- `micro_ros_agent` / `micro_ros_msgs`：实机 MCU micro-ROS 通信层
- `logger`：日志库

不包含 EstimatorTort、QP 平衡控制、OCS2 控制器、策略控制器和旧的通用输入链路。

## 构建

```bash
cd /home/ry/project/quadruped
colcon build --symlink-install
source install/setup.bash
```

构建后确认可执行入口：

```bash
ros2 pkg executables quadruped
ros2 pkg executables serial_controller
ros2 pkg executables kbd
ros2 pkg executables micro_ros_agent
```

期望看到：

```text
quadruped quadruped_node
serial_controller serial_controller_node
kbd kbd
micro_ros_agent micro_ros_agent
```

## 控制链路

仿真：

```text
QLP遥控器
  -> /input/mode, /kbd_input, /joystick/command
  -> quadruped_node
  -> /joint_effort_controller/commands
  -> Gazebo Go1
```

实机：

```text
QLP遥控器
  -> /input/mode, /kbd_input, /joystick/command
  -> quadruped_node
  -> /real_joint_effort_controller/commands
  -> micro_ros_agent
  -> MCU
```

实机链路里 MCU 和 Linux 之间只走 `/real_joint_effort_controller/commands`。MCU 不负责向 Linux 发布 `/joint_states`，Linux 侧也不会在实机启动时等待 `/joint_states`。

`/input/mode` 的含义：

- `0.0`：输出到 `/joint_effort_controller/commands`
- `1.0`：输出到 `/real_joint_effort_controller/commands`

仿真 launch 会锁定 `output_mode=0.0`，忽略 `/input/mode`，防止误切到实机输出。

## 仿真运行

启动 Go1 Gazebo 仿真、传统 PD 控制和 QLP 遥控器输入：

```bash
cd /home/ry/project/quadruped
source install/setup.bash
ros2 launch quadruped traditional_pd_sim.launch.py
```

默认会启动 `serial_controller`，一个遥控器节点同时输出按键和摇杆：

```text
遥控器按键 -> /kbd_input
遥控器摇杆 -> /joystick/command
遥控器模式 -> /input/mode
```

无 GUI：

```bash
ros2 launch quadruped traditional_pd_sim.launch.py gui:=false
```

如果仿真时不接遥控器：

```bash
ros2 launch quadruped traditional_pd_sim.launch.py \
  start_serial:=false \
  start_kbd:=true
```

如需指定遥控器串口：

```bash
ros2 launch quadruped traditional_pd_sim.launch.py serial_port:=/dev/ttyUSB0
```

仿真启动后，按控制流程操作：

1. 等待 Gazebo 和 `/joint_states` 正常发布。
2. 按 `f`，允许 Init 起身。
3. 起身完成后按 `b`，进入 `Tort` 小跑模式。
4. 推左摇杆控制方向。
5. 需要卸力时按 `i`，进入 `Deinit`。

## 实机运行

实机通常需要两个串口：

- 遥控器串口：默认 `/dev/ttyUSB0`
- MCU micro-ROS 串口：默认 `/dev/ttyUSB1`

推荐实机启动方式：一条命令打开两个终端，一个看机器狗控制状态，一个看 micro-ROS Agent 状态：

```bash
cd /home/ry/project/quadruped
source install/setup.bash
ros2 launch quadruped traditional_pd_real_terminals.launch.py
```

默认配置：

```text
遥控器串口: /dev/ttyUSB0
MCU micro-ROS 串口: /dev/ttyUSB1
```

指定串口：

```bash
ros2 launch quadruped traditional_pd_real_terminals.launch.py \
  serial_port:=/dev/ttyUSB0 \
  mcu_port:=/dev/ttyUSB1
```

如果不需要分开终端，也可以在当前终端启动传统 PD 控制、QLP 遥控器输入和 micro-ROS Agent：

```bash
cd /home/ry/project/quadruped
source install/setup.bash
ros2 launch quadruped traditional_pd_control.launch.py \
  serial_port:=/dev/ttyUSB0 \
  start_micro_ros_agent:=true \
  mcu_port:=/dev/ttyUSB1 \
  mcu_baud_rate:=115200
```

如果只想启动控制节点和遥控器，不启动 micro-ROS Agent：

```bash
ros2 launch quadruped traditional_pd_control.launch.py \
  serial_port:=/dev/ttyUSB0
```

如果要强制实机输出，不依赖遥控器 `/input/mode`：

```bash
ros2 launch quadruped traditional_pd_control.launch.py \
  output_mode:=1.0 \
  lock_output_mode:=true \
  serial_port:=/dev/ttyUSB0 \
  start_micro_ros_agent:=true \
  mcu_port:=/dev/ttyUSB1
```

实机启动后，按控制流程操作：

1. 确认 MCU 通过 micro-ROS Agent 连接。
2. 确认控制终端打印“实机力矩输出模式不等待 /joint_states，按本地初始状态进入 FSM”。
3. 确认 `/input/mode` 为 `1.0`，或使用 `lock_output_mode:=true output_mode:=1.0`。
4. 按 `f`，允许 Init 起身。
5. 起身完成后按 `b`，进入 `Tort` 小跑模式。
6. 推左摇杆控制方向。
7. 需要卸力时按 `i`。

按 `f` 后用于判断消息是否正常接收的日志顺序：

```text
serial_controller: 遥控器按键: f
quadruped_control: 收到 /joystick/command 按键: f
quadruped_control: 收到f，允许Init起身
quadruped_control: 开始初始化
quadruped_control: 初始起身完成
```

如果只看到 `serial_controller` 的按键日志，说明遥控器串口正常，但控制节点没收到输入话题。  
实机正常情况下不会等待 `/joint_states`；如果仍然看到 `仍在等待 /joint_states`，说明启动参数里 `require_joint_state` 被设成了 `true`，这只适合仿真。

## 输入节点

QLP 遥控器节点同时包含按键和摇杆输入，launch 默认一起启动。需要单独调试时：

```bash
ros2 run serial_controller serial_controller_node \
  --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200
```

本机键盘只作为没有遥控器时的备份输入：

```bash
ros2 run kbd kbd
```

## FSM 按键

当前传统 PD FSM 使用单字符按键：

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

`Tort` 模式下左摇杆方向：

| 摇杆 | 配置 |
| --- | --- |
| 左摇杆前 | `walk` |
| 左摇杆后 | `back` |
| 左摇杆左 | `left` |
| 左摇杆右 | `right` |
| 回中 | `stand` |

## 常用检查

确认输入：

```bash
ros2 topic echo /kbd_input
ros2 topic echo /joystick/command
ros2 topic echo /input/mode
```

确认仿真状态：

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_effort_controller/commands
```

确认实机输出：

```bash
ros2 topic echo /real_joint_effort_controller/commands
```

确认 micro-ROS Agent：

```bash
ros2 node list | grep micro_ros
ros2 topic info /real_joint_effort_controller/commands -v
```

查看串口：

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
dmesg | tail -n 50
```

手动触发起身：

```bash
ros2 topic pub --once /kbd_input std_msgs/msg/String "{data: 'f'}"
```

手动进入小跑：

```bash
ros2 topic pub --once /kbd_input std_msgs/msg/String "{data: 'b'}"
```

## 注意事项

- 仿真使用 `/joint_effort_controller/commands`。
- 实机使用 `/real_joint_effort_controller/commands`。
- 仿真 launch 默认锁定仿真输出，遥控器的 `/input/mode` 不会改变输出话题。
- 实机时遥控器串口和 MCU 串口不能是同一个设备。
- 仿真要求 `quadruped_node` 收到 `/joint_states` 后才进入控制循环；实机默认不等待 `/joint_states`。
- Init 状态下必须按 `f` 才会真正起身并开始发布力矩。
