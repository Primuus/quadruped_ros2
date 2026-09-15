# 四足机器人控制工作区

`quadruped` 是本地控制、仿真和策略加载工作区。当前包含两套互相独立的框架：

- 传统 PD 控制：ROS2 启动，负责 Gazebo 仿真和实机控制。
- RL 部署：直接 Python 运行，当前负责 MuJoCo 策略回放，并包含未来自研四足实机链路的公共接口和配置基础。

训练侧在 `quadruped_train` 工作区完成，不和本仓库混用。本仓库不训练策略，只负责传统控制链路和本地 RL 策略回放。当前接入的机器人实例是 Go2，但 RL 部署接口按四足机器人通用配置组织，新增机器人时应新增 YAML、MuJoCo 资源和策略文件。

## 框架概览

| 框架 | 入口 | 运行方式 | 作用 |
| --- | --- | --- | --- |
| 传统 PD | `ros2 launch quadruped ...` | ROS2 | Gazebo 仿真 / 实机控制 |
| RL 部署 | `python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py` | 直接 Python | MuJoCo 策略回放；实机链路基础开发 |

两套框架不共享启动入口：传统 PD 不加载 RL 策略，RL 部署也不接入 `quadruped` 的 ROS2 launch。

## 目录说明

- `src/controller/quadruped`：传统 PD 控制节点、FSM、launch、控制参数和消息接口。
- `src/controller/rl`：RL 部署包，包含 MuJoCo runner、观测构造、双推理后端、Linux 本地键盘输入，以及实机配置、硬件协议和固定版本的 Unitree Actuator SDK。
- `src/input/serial_controller`：QLP 遥控器串口输入，服务于传统 PD ROS2 链路。
- `src/input/kbd`：键盘备用输入，服务于传统 PD ROS2 链路。
- `src/robot_gazebo`：Gazebo Classic 仿真资源。
- `src/micro-ROS-Agent` / `src/micro_ros_msgs`：实机 MCU 通信层。
- `src/libraries/logger`：日志库。

## 传统 PD

这套链路走 ROS2，适合 Gazebo 仿真和实机控制。

### 构建

```bash
colcon build --symlink-install
source install/setup.bash
```

### Gazebo 仿真

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

## RL 本地部署

RL 链路不走 ROS2 launch，不依赖 `colcon build`，也不依赖 `micro-ROS`。它的作用是把 `quadruped_train` 从同一部署模型导出的 TorchScript 和 ONNX 策略分别加载到本地 MuJoCo 中验证。

完整流程：

```text
quadruped_train 训练
  -> play_go2.py 直接导出 policy_1.pt / policy_1.onnx
  -> 复制到 src/controller/rl/models/
  -> deploy_mujoco_rl.py 在相同 MuJoCo 闭环中切换推理后端
```

当前 Go2 策略接口：

```text
输入：270 维历史观测
内部：estimator + actor
输出：12 维动作
控制：动作 -> 目标关节角 -> PD 力矩 -> MuJoCo actuator
```

训练侧 critic 的 238 维 privileged obs 只用于训练，不进入本地部署策略输入。

### 实机链路开发状态

实机部署和 MuJoCo 使用独立入口，目标是让未来自研四足通过 Python、ONNX Runtime、YESENSE YIS130 IMU 和 Unitree Actuator SDK 控制 12 个 GO-M8010-6 电机。当前已完成方案阶段 0–5：

- 通用 `JointState`、`ImuState`、`JointCommand` 和硬件后端协议；
- `config/real/mock.yaml` 与严格配置解析；
- `config/real/custom_quadruped.example.yaml` 待标定模板；
- 工程内固定版本的官方 Unitree Actuator SDK 和无硬件构建脚本。
- 不依赖设备和模型文件的 Mock 电机、Mock IMU、Mock 策略及故障注入。
- GO-M8010-6 真实电机后端，支持多条 USB-RS485 总线、12 关节映射、转子侧换算、完整反馈聚合以及总线状态诊断。

当前没有实机 runner 和 YIS130 后端，默认入口也不会向设备发送命令。GO-M8010-6 后端只能由未来实机入口显式实例化，并要求真实配置和已构建的官方 SDK；完整运行循环将在后续阶段实现。阶段和验收顺序见项目根目录的 `方案.md`。

### 依赖

```bash
pip install mujoco torch numpy pyyaml evdev "onnxruntime>=1.16,<2"
```

### 默认文件

- 配置：`src/controller/rl/config/go2.yaml`
- MuJoCo 场景：`src/controller/rl/resources/robots/go2/scene.xml`
- TorchScript 基准策略：`src/controller/rl/models/policy_1.pt`
- ONNX 策略：`src/controller/rl/models/policy_1.onnx`

新增其他四足机器人时，对应新增 `config/<robot>.yaml`、`resources/robots/<robot>/` 和匹配的双格式策略，然后通过 `--config` 指定配置运行。

### 策略复制

训练侧 `play` 导出策略后，在本地电脑终端复制两个模型：

```bash
scp hurricane@192.168.0.214:~/quadruped_train/logs/go2/exported/policies/policy_1.{pt,onnx} \
  /home/ry/project/quadruped/src/controller/rl/models/
```

如果 SSH 需要指定端口，才加 `-P`，例如：

```bash
scp -P 22 hurricane@192.168.0.214:~/quadruped_train/logs/go2/exported/policies/policy_1.{pt,onnx} \
  /home/ry/project/quadruped/src/controller/rl/models/
```


### 运行

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py --headless
```

指定策略、模型和设备：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --policy-path src/controller/rl/models/policy_1.pt \
  --xml-path src/controller/rl/resources/robots/go2/scene.xml \
  --device cpu \
  --headless
```

运行 ONNX Runtime 后端时只更换策略文件，后端会根据 `.onnx` 扩展名自动选择：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --policy-path src/controller/rl/models/policy_1.onnx \
  --device cpu \
  --headless
```

也可以用 `--policy-backend torchscript|onnx` 显式指定后端。显式值与模型格式不匹配时，模型加载会直接失败，不会静默回退。

指定速度命令：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --no-keyboard \
  --command 0.5 0.0 0.0 \
  --duration 30
```

默认会自动查找并同时监听当前 Linux 主机上所有具备完整控制按键的本地键盘。需要限制为某一个设备时：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --keyboard-device /dev/input/by-path/<设备名>-event-kbd
```

设备编号可能在重启后变化，应优先使用 `/dev/input/by-path/*-event-kbd`，不要长期写死 `/dev/input/eventN`。程序读取的是运行主机自己的 evdev 设备；SSH 客户端的按键不会自动转发成远端主机的键盘事件。

不接键盘、仅使用 YAML 或 `--command` 固定命令时传入 `--no-keyboard`。键盘设备打开失败时程序不会进入键盘控制；可执行 `sudo usermod -aG input "$USER"` 并注销后重新登录，或者配置 udev 规则。不要让整个 RL 程序以 root 运行。

打印前几次 policy 推理的调试信息：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --no-keyboard \
  --command 0.0 0.0 0.0 \
  --duration 5 \
  --debug-steps 100
```

也可以使用 RL 子工程里的调试脚本：

```bash
python3 src/controller/rl/test/deploy_mujoco_rl_debug.py
```

该脚本默认静止命令运行 5 秒，并打印前 100 次 policy 推理的 `base_z`、`projected_gravity`、`joint_pos`、`raw_action`、`clipped_action`、`torques` 和 `target_joint_pos`，用于排查 Isaac Gym 能走但 MuJoCo 表现异常的问题。`raw_action` 是 policy 原始输出，`clipped_action` 是按 YAML 中 `clip_actions` 裁剪后实际送入 PD 的动作；当前 Go2 默认 `clip_actions: 100.0`，和训练侧保持一致。

离线比较同一批观测下的 TorchScript / ONNX 输出：

```bash
python3 src/controller/rl/test/compare_policy_backends.py \
  --torchscript-path src/controller/rl/models/policy_1.pt \
  --onnx-path src/controller/rl/models/policy_1.onnx
```

该脚本只比较推理输出并返回误差统计，不改变 MuJoCo、观测或 PD。闭环一致性仍需分别运行两个模型观察。

接入本地键盘做调试：

```bash
python3 src/controller/rl/test/deploy_mujoco_rl_keyboard_debug.py
```

该脚本自动识别键盘，运行 30 秒，并打印前 200 次 policy 推理。需要指定键盘时：

```bash
python3 src/controller/rl/test/deploy_mujoco_rl_keyboard_debug.py \
  --keyboard-device /dev/input/by-path/<设备名>-event-kbd \
  --debug-steps 300
```

键盘控制启动时默认禁用，先按 `F` 使能。使能成功后程序会独占已监听的键盘，控制按键不会再触发 MuJoCo Viewer 的默认快捷键；按 `I` 会禁用控制、清零命令并释放键盘。键盘被独占时，终端也收不到该键盘的 `Ctrl+C`，需要先按 `I` 再终止程序。方向键采用按住式控制，松开后对应速度轴立即归零：

| 按键 | 作用 |
| --- | --- |
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 左移 / 右移 |
| `Q` / `E` | 左转 / 右转 |
| `Space` | 立即清零运动命令，方向键释放后才允许再次移动 |
| `F` | 使能键盘控制；从禁用状态恢复时重置策略历史 |
| `I` | 禁用键盘控制并清零命令 |
| `1` / `2` / `3` | 25% / 50% / 100% 速度档位 |
| `R` | 重置 MuJoCo 仿真 |
| `P` | 打印当前按键、速度档位和命令 |

方向键可以组合；同一轴的反向键同时按下时该轴输出为零。按键和速度比例统一配置在 `config/go2.yaml` 的 `keyboard` 段。

RL 文件夹的完整结构和每个文件职责见 [src/controller/rl/README.md](src/controller/rl/README.md)。

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
python3 -m py_compile src/controller/rl/rl_controller/core/policy.py
python3 -m py_compile src/controller/rl/test/compare_policy_backends.py
python3 -m unittest src/controller/rl/test/test_keyboard_controller.py -v
```

## 边界说明

- 传统 PD 负责 ROS2 控制链路。
- RL 负责本地 Python + MuJoCo 双后端策略回放。
- 训练在 `quadruped_train`。
- 部署和本地仿真在 `quadruped`。
- 两套框架可以共存，但启动入口、依赖和运行方式是分开的。
