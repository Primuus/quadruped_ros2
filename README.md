# 四足机器人控制工作区

这个工作区面向四足机器人控制，当前有两套完全独立的框架：

- 传统 PD 控制：ROS2 启动，负责仿真和实机控制
- RL 部署：直接 Python 运行，负责本地 MuJoCo 策略回放

训练侧在另一个工作区 `quadruped_train`，不和这里混用。这里负责本地控制、仿真和策略加载。

## 框架概览

| 框架 | 入口 | 运行方式 | 作用 |
| --- | --- | --- | --- |
| 传统 PD | `ros2 launch quadruped ...` | ROS2 | 仿真 / 实机控制 |
| RL 部署 | `python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py` | 直接 Python | 本地 MuJoCo + TorchScript 策略回放 |

两套框架不共享启动入口：传统 PD 不加载 RL 策略，RL 部署也不接入 `quadruped` 的 ROS2 launch。

## 目录说明

- `src/controller/quadruped`：传统 PD 控制节点、launch、消息和配置
- `src/controller/rl`：RL 部署包，直接 Python 运行，不接 `quadruped` 主 launch
- `src/input/serial_controller`：QLP 遥控器串口输入
- `src/input/kbd`：键盘备用输入
- `src/robot_gazebo`：Gazebo Classic 仿真资源
- `src/micro-ROS-Agent` / `src/micro_ros_msgs`：实机 MCU 通信层
- `src/libraries/logger`：日志库

## 传统 PD

这套链路走 ROS2，适合仿真和实机。

### 构建

```bash
# 进入 quadruped 仓库根目录
colcon build --symlink-install
source install/setup.bash
```

### 仿真

```bash
ros2 launch quadruped traditional_pd_sim.launch.py
```

无 GUI：

```bash
ros2 launch quadruped traditional_pd_sim.launch.py gui:=false
```

不用遥控器、改用键盘输入：

```bash
ros2 launch quadruped traditional_pd_sim.launch.py \
  start_serial:=false \
  start_kbd:=true
```

### 实机

```bash
ros2 launch quadruped traditional_pd_real_terminals.launch.py
```

指定串口：

```bash
ros2 launch quadruped traditional_pd_real_terminals.launch.py \
  serial_port:=/dev/ttyUSB0 \
  mcu_port:=/dev/ttyUSB1
```

### 传统 PD 控制链路

仿真：

```text
QLP 遥控器 / 键盘
  -> /input/mode, /kbd_input, /joystick/command
  -> quadruped_node
  -> /joint_effort_controller/commands
  -> Gazebo
```

实机：

```text
QLP 遥控器
  -> /input/mode, /kbd_input, /joystick/command
  -> quadruped_node
  -> /real_joint_effort_controller/commands
  -> micro_ros_agent
  -> MCU
```

`/input/mode` 的含义：

- `0.0`：仿真输出 `/joint_effort_controller/commands`
- `1.0`：实机输出 `/real_joint_effort_controller/commands`

### FSM 按键

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

## RL 部署

这套链路不走 ROS2 launch，直接运行 Python 脚本。RL 部署不需要 `colcon build`，也不依赖 `micro-ROS`。

完整流程是：

```text
quadruped_train 训练
  -> play 最新 checkpoint 并导出 JIT 策略
  -> 复制 JIT 策略到 quadruped/src/controller/rl/models/policy_1.pt
  -> quadruped 本地 MuJoCo 仿真
```

### 依赖

- `mujoco`
- `torch`
- `numpy`
- `pyyaml`

安装示例：

```bash
pip install mujoco torch numpy pyyaml
```

### 默认文件

- 配置：`src/controller/rl/config/go2.yaml`
- 机器人模型：`src/controller/rl/resources/robots/go2/scene.xml`
- 策略文件：`src/controller/rl/models/policy_1.pt`

仓库默认不内置训练好的策略。训练完成后，把导出的 TorchScript 模型放到默认路径 `src/controller/rl/models/policy_1.pt`，或通过 `--policy-path` 指定路径。

当前部署侧支持 Go2 阶段 3 的 `45*6=270` 维历史观测策略，单帧 45 维观测顺序和 `quadruped_train` 保持一致：

```text
[0:3]   速度命令 vx, vy, yaw_rate
[3:6]   机身角速度
[6:9]   重力方向在机身坐标系下的投影
[9:21]  12 个关节位置偏差
[21:33] 12 个关节速度
[33:45] 上一次策略动作
```

部署代码会维护 6 帧历史缓存，并把 `[当前 45 维, 上一帧 45 维, ..., 更早第 5 帧 45 维]` 输入给 TorchScript 策略。旧版 48 维或 45 维策略不能直接放到这里运行，需要用当前训练配置重新训练并导出新的 `policy_1.pt`。

### 从训练侧导出策略

在训练服务器的 `quadruped_train` 工作区里先完成训练：

```bash
python quadruped_rl/scripts/train_go2.py --headless
```

训练完成后，用 `play_go2.py` 加载最新 checkpoint，并导出 JIT 策略：

```bash
python quadruped_rl/scripts/play_go2.py --headless
```

默认导出路径：

```text
quadruped_train/logs/go2/exported/policies/policy_1.pt
```

如果要指定某一次训练或某个 checkpoint，可以加训练侧参数：

```bash
python quadruped_rl/scripts/play_go2.py \
  --headless \
  --load_run rough \
  --checkpoint 500
```

### 复制到本地部署目录

把导出的策略复制到本仓库的默认模型位置：

```bash
cp /path/to/quadruped_train/logs/go2/exported/policies/policy_1.pt \
  src/controller/rl/models/policy_1.pt
```

如果策略在当前训练服务器上，可以从本地执行：

```bash
scp -P 2222 hurricane@10.109.70.55:~/quadruped_train/logs/go2/exported/policies/policy_1.pt \
  src/controller/rl/models/policy_1.pt
```

### 运行

从仓库根目录直接运行：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py --headless
```

常用参数：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --policy-path src/controller/rl/models/policy_1.pt \
  --xml-path src/controller/rl/resources/robots/go2/scene.xml \
  --device cpu \
  --headless
```

也可以只改命令输入：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --command 0.5 0.0 0.0 \
  --duration 30
```

接 QLP 遥控器时再加：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --remote-port /dev/ttyUSB0 \
  --remote-baud-rate 115200 \
  --headless
```

默认映射：

- `mode`: 遥控使能门控
- 左摇杆 Y: 前进/后退
- 左摇杆 X: 左右平移
- 右摇杆 X: 偏航
- 右摇杆 Y: 速度微调
- `f`: 使能控制
- `F`: 重置仿真
- `i`: 关闭控制
- `b`: 前进预设
- `h`: 站立预设
- `g` / `c`: 左右转向预设
- `a` / `u`: 左右平移预设

如果判定为摔倒，程序只会停止输出力矩，仿真本身继续运行；按 `F` 可以重置。

### 说明

- RL 部署和传统 PD 完全隔离
- 不接 `quadruped` 主 launch
- 不依赖 `micro-ROS`
- 当前只做本地 MuJoCo 策略回放，不包含 sim-to-real 实机链路

## 常用检查

传统 PD 构建后检查 ROS2 入口：

```bash
ros2 pkg executables quadruped
ros2 pkg executables serial_controller
ros2 pkg executables kbd
```

传统 PD 仿真话题检查：

```bash
ros2 topic echo /input/mode
ros2 topic echo /joint_effort_controller/commands
```

RL 部署脚本语法检查：

```bash
python3 -m py_compile src/controller/rl/rl_controller/deploy_mujoco_rl.py
```

## 两套框架的边界

- 传统 PD 只负责 ROS2 控制链路
- RL 只负责本地 Python 回放
- 两者可以共存，但启动入口和依赖是分开的
- 训练在 `quadruped_train`，部署在这个仓库
