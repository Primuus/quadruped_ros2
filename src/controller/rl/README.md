# RL 本地部署子工程说明

`src/controller/rl` 是 `quadruped` 中的 RL 本地部署包。它和传统 PD ROS2 控制链路完全隔离，不走 `ros2 launch`，主要作用是在本地 MuJoCo 中加载 `quadruped_train` 导出的 TorchScript 策略，验证策略在 Go2 模型上的运动效果。

当前 RL 部署不包含训练代码，也不包含实机 sim-to-real 输出链路。

## 总体数据流

```text
MuJoCo state
  -> Go2State
  -> 45 维单帧观测
  -> 6 帧 history buffer，形成 270 维 policy 输入
  -> TorchScript policy(estimator + actor)
  -> 12 维 action
  -> action_scale + default_angles，得到目标关节角
  -> PD 计算力矩
  -> MuJoCo actuator
```

训练侧 critic 使用的 238 维 privileged obs 不进入这里的策略输入。部署侧只负责构造 270 维历史观测并调用 TorchScript。

## 目录结构

```text
src/controller/rl/
  README.md
  package.xml
  setup.py
  setup.cfg
  resource/rl_controller
  config/go2.yaml
  models/
  resources/robots/go2/
  rl_controller/
    deploy_mujoco_rl.py
    core/
    remote/
```

## 顶层文件

| 文件 | 作用 |
| --- | --- |
| `README.md` | 当前说明文档，解释 RL 子工程结构、文件职责和运行方式。 |
| `package.xml` | ROS2/ament Python 包元数据。虽然 RL 入口主要直接 Python 运行，但保留包信息便于工作区识别和安装。 |
| `setup.py` | Python 包安装脚本，收集 `config/`、`models/`、`resources/` 等数据文件，并声明 Python 依赖。 |
| `setup.cfg` | Python entry point 和安装路径相关配置。 |
| `resource/rl_controller` | ament resource marker，用于 ROS2 包索引识别。 |

## 配置文件

### `config/go2.yaml`

Go2 部署配置。它是部署侧最重要的配置文件，负责把训练侧和 MuJoCo 部署侧对齐。

主要内容：

- 策略文件路径：`policy_path`
- MuJoCo 场景路径：`xml_path`
- 仿真时长和步长：`simulation_duration`、`simulation_dt`
- 控制降频：`control_decimation`
- 观测维度：`num_one_step_obs=45`、`history_length=6`、`num_obs=270`
- 训练侧 critic 对齐信息：`num_one_step_privileged_obs=238`、`num_privileged_obs=238`
- 关节顺序：`joint_names`
- actuator 顺序：`actuator_names`
- PD 参数：`kps`、`kds`
- 默认关节角：`default_angles`
- 动作缩放：`action_scale`
- 初始速度命令和命令限制：`command_init`、`command_limits`
- 跌倒检测阈值：`fall_height_threshold`、`fall_gravity_z_threshold`
- 观测缩放：`obs_scale_*`
- 初始机身位姿：`initial_base_pos`、`initial_base_quat`

修改原则：

- 训练侧改了 PD、默认角、action scale 或观测缩放时，这里要同步。
- 部署侧 policy 外部输入必须保持 270 维。
- privileged obs 只用于配置对齐和检查，不会输入 TorchScript policy。

## 策略文件

### `models/`

默认策略目录。仓库不内置训练好的策略，训练完成后把导出的 TorchScript 放到：

```text
models/policy_1.pt
```

相关文件：

- `models/.gitignore`：避免误提交模型权重。
- `models/README.md`：说明模型目录的用途。

策略必须来自当前训练框架导出的 `estimator + actor` TorchScript，外部输入为 270 维历史观测。

## MuJoCo 资源

### `resources/robots/go2/scene.xml`

MuJoCo 场景入口文件。通常包含地面、光照、相机和 Go2 robot include。

### `resources/robots/go2/go2.xml`

Go2 MuJoCo robot 模型文件。部署代码会根据 `go2.yaml` 中的关节名和 actuator 名称，从这个模型中解析 qpos/qvel 地址和 actuator id。

### `resources/robots/go2/urdf/`、`resources/robots/go2/obj/`

Go2 机器人资产文件。用于模型几何、mesh 或 URDF 资源保留。

## Python 包入口

### `rl_controller/deploy_mujoco_rl.py`

RL 本地部署主入口。

负责内容：

- 读取 `go2.yaml`。
- 加载 MuJoCo 模型。
- 加载 TorchScript policy。
- 解析关节和 actuator 索引。
- 初始化仿真状态。
- 每隔 `control_decimation` 个 MuJoCo step 计算一次 policy action。
- 把 action 转成 PD 力矩。
- 写入 MuJoCo actuator。
- 可选接入 QLP 遥控器。
- 检测摔倒后停止输出力矩，但不退出仿真。

主要类：

- `Go2MujocoRunner`：封装 MuJoCo 仿真、策略推理、PD 输出和遥控器输入。

常用命令行参数：

| 参数 | 作用 |
| --- | --- |
| `--config` | 指定 YAML 配置文件。 |
| `--policy-path` | 覆盖策略文件路径。 |
| `--xml-path` | 覆盖 MuJoCo XML 路径。 |
| `--duration` | 覆盖仿真时长。 |
| `--command VX VY WZ` | 指定速度命令。 |
| `--device` | 指定 Torch 推理设备，如 `cpu` 或 `cuda:0`。 |
| `--headless` | 不打开 MuJoCo viewer。 |
| `--remote-port` | 启用 QLP 遥控器串口。 |
| `--remote-baud-rate` | 遥控器串口波特率。 |
| `--remote-deadzone` | 摇杆死区。 |
| `--remote-speed-trim-scale` | 右摇杆 Y 方向速度微调倍率。 |

## core 模块

### `rl_controller/core/config.py`

配置加载和校验模块。

负责内容：

- 定义 `Go2Config` dataclass。
- 从 YAML 读取部署配置。
- 解析相对路径为绝对路径。
- 提供默认关节顺序、默认 PD、默认观测维度等常量。
- 校验动作维度、观测维度、history 长度、关节数量、actuator 数量。

这个文件是部署侧配置一致性的第一道检查。训练侧如果改变和部署有关的配置，应同步更新 `go2.yaml` 和这里的默认值或校验逻辑。

### `rl_controller/core/observer.py`

观测构造模块。

负责内容：

- 定义 `Go2State`，保存 MuJoCo 中读出的机身和关节状态。
- 提供四元数旋转工具：`quat_rotate`、`quat_rotate_inverse`。
- 构造 45 维单帧 actor 观测。
- 维护 6 帧 history buffer，输出 270 维 policy 输入。

45 维观测顺序必须和 `quadruped_train` 保持一致：

```text
[0:3]   速度命令 vx, vy, yaw_rate
[3:6]   机身角速度
[6:9]   重力方向在机身坐标系下的投影
[9:21]  12 个关节位置偏差
[21:33] 12 个关节速度
[33:45] 上一次策略动作
```

### `rl_controller/core/policy.py`

TorchScript 策略加载和推理模块。

负责内容：

- 加载 `policy_1.pt`。
- 设置推理设备。
- 把 numpy observation 转成 Torch tensor。
- 调用 TorchScript model。
- 把输出转回 numpy action。

如果 TorchScript 输出是 tuple/list，会默认取第一个输出作为 action。

### `rl_controller/core/pd.py`

action 到力矩的转换模块。

负责内容：

- 将 policy 输出 action 限幅后映射为目标关节角：

  ```text
  target_pos = default_angles + action * action_scale
  ```

- 用 PD 计算关节力矩：

  ```text
  torque = kp * (target_pos - joint_pos) - kd * joint_vel
  ```

- 按 MuJoCo actuator torque limit 裁剪输出。

这个文件里的控制逻辑必须和训练侧 action scale、默认角、PD 设定保持一致。

### `rl_controller/core/__init__.py`

core 模块导出文件，方便外部直接导入 `Go2Config`、`Go2ObservationHistory` 等核心类型。

## remote 模块

### `rl_controller/remote/qlp_protocol.py`

QLP 遥控器串口协议解析模块。

负责内容：

- QLP frame 编解码。
- CRC16 校验。
- 流式 parser。
- joystick report 打包/解包。
- 将遥控器原始摇杆值归一化到策略命令可用的范围。

### `rl_controller/remote/input.py`

遥控器串口输入线程。

负责内容：

- 打开 `--remote-port` 指定的串口。
- 后台线程持续读取串口数据。
- 调用 `QlpStreamParser` 解析 QLP frame。
- 生成最新 `RemoteSnapshot`。
- 收集按键事件队列。
- 记录串口异常，供主循环关闭遥控控制。

`RemoteSnapshot` 包含：

```text
mode / mode_active
joy_lx / joy_ly / joy_rx / joy_ry
key
timestamp
```

### `rl_controller/remote/controller.py`

遥控器命令映射模块。

负责内容：

- 把摇杆输入转换为速度命令 `[vx, vy, yaw_rate]`。
- 应用摇杆死区。
- 应用右摇杆 Y 的速度微调。
- 根据 `mode_active` 和使能状态决定是否输出命令。
- 处理按键预设和仿真重置。

默认摇杆映射：

| 输入 | 作用 |
| --- | --- |
| 左摇杆 Y | 前进 / 后退 |
| 左摇杆 X | 左右平移 |
| 右摇杆 X | 偏航 |
| 右摇杆 Y | 速度微调 |
| `mode` | 遥控使能门控 |

默认按键：

| 按键 | 作用 |
| --- | --- |
| `f` | 使能控制 |
| `F` | 重置仿真 |
| `i` | 关闭控制 |
| `b` | 前进预设 |
| `h` | 站立预设 |
| `g` | 左转预设 |
| `c` | 右转预设 |
| `a` | 左移预设 |
| `u` | 右移预设 |
| `d` | 打印当前遥控状态 |

### `rl_controller/remote/__init__.py`

remote 模块导出文件，供 `deploy_mujoco_rl.py` 直接导入 `QlpRemoteInput` 和 `RemoteCommandController`。

## 运行方式

先把训练导出的策略放到：

```bash
src/controller/rl/models/policy_1.pt
```

从 `quadruped` 仓库根目录运行：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py --headless
```

打开 MuJoCo viewer：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py
```

指定策略和模型：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --policy-path src/controller/rl/models/policy_1.pt \
  --xml-path src/controller/rl/resources/robots/go2/scene.xml \
  --device cpu
```

指定速度命令：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --command 0.5 0.0 0.0 \
  --duration 30
```

接 QLP 遥控器：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --remote-port /dev/ttyUSB0 \
  --remote-baud-rate 115200
```

## 和训练侧的同步关系

训练侧改动后，需要同步部署侧的情况：

- 改了 `num_one_step_obs`、`history_length` 或 `num_obs`。
- 改了 45 维观测顺序。
- 改了 action 维度或关节顺序。
- 改了 `default_angles`。
- 改了 `action_scale`。
- 改了 PD 参数。
- 改了命令缩放或观测缩放。
- 改了导出策略的外部输入格式。

训练侧只改 critic privileged obs 内容、reward 权重、噪声、域随机化时，通常不需要改部署推理逻辑；但配置说明应保持一致。

## 摔倒处理

部署主循环会根据机身高度和重力方向投影判断是否摔倒。判定摔倒后：

- policy 不再输出有效 action。
- 当前力矩清零。
- history buffer 重置。
- MuJoCo 仿真继续运行。
- 遥控器按 `F` 可以重置仿真。

## 常用检查

语法检查：

```bash
python3 -m py_compile src/controller/rl/rl_controller/deploy_mujoco_rl.py
python3 -m py_compile src/controller/rl/rl_controller/core/config.py
python3 -m py_compile src/controller/rl/rl_controller/core/observer.py
python3 -m py_compile src/controller/rl/rl_controller/core/pd.py
python3 -m py_compile src/controller/rl/rl_controller/core/policy.py
```

配置读取检查：

```bash
PYTHONPATH=src/controller/rl python3 - <<'PY'
from rl_controller.core.config import Go2Config
cfg = Go2Config.from_yaml('src/controller/rl/config/go2.yaml')
print(cfg.num_obs, cfg.num_one_step_privileged_obs, cfg.num_privileged_obs)
PY
```

期望输出：

```text
270 238 238
```
