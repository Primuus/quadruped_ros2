# RL 控制器独立框架 — 执行计划

## 目标

在 `quadruped` 工程内新增独立的 RL 控制框架（Python），加载 Unitree RL 框架训练出的 `.pt` 策略，通过 MuJoCo 仿真实现强化学习驱动的四足机器人控制。与传统 PD 框架完全解耦，不依赖 ROS2。

---

## 总体架构

```
服务器 (GPU)                              本机
┌──────────────────────┐          ┌────────────────────────────────┐
│ unitree_rl_gym/      │          │ quadruped/src/rl_controller/   │
│  train.py → PPO 训练  │  .pt    │                                │
│  play.py  → 导出 .pt │─────────▶│ 核心层 (core/)                  │
│                      │  .yaml  │   config.py    ← yaml 解析      │
└──────────────────────┘          │   observer.py  ← 观测构造       │
                                  │   policy.py    ← 策略推理       │
                                  │   pd.py        ← PD 力矩计算    │
                                  │                                │
                                  │ I/O 层 (io/)                    │
                                  │   serial_joystick.py ← 串口     │
                                  │                                │
                                  │ 入口脚本                        │
                                  │   deploy_mujoco_rl.py ← 仿真    │
                                  │   deploy_real_rl.py   ← 真机    │
                                  └────────────────────────────────┘
```

**核心设计原则**：I/O 层可插拔，核心层通用。仿真换真机只换 I/O 层，核心逻辑一行不动。

---

## 目录结构

```
quadruped/src/rl_controller/
├── core/                           ← 核心层 (仿真/真机共用)
│   ├── __init__.py
│   ├── config.py                   ← yaml 参数解析
│   ├── observer.py                 ← 观测构造 (47 维)
│   ├── policy.py                   ← TorchScript 策略加载+推理
│   └── pd.py                       ← PD 控制器 (关节角→力矩)
│
├── io/                             ← I/O 层 (可插拔)
│   ├── __init__.py
│   └── serial_joystick.py          ← QLP 串口遥控器读取
│
├── deploy_mujoco_rl.py             ← MuJoCo 仿真入口
├── deploy_real_rl.py               ← 真机部署入口 (后期)
│
├── config/                         ← 部署配置
│   ├── go2.yaml                    ← Go2 参数
│   ├── g1.yaml                     ← G1 参数 (参照 unitree_rl_gym)
│   └── ...
│
└── models/                         ← 策略权重
    └── go2_policy.pt               ← (从服务器 scp 过来)
```

---

## 核心层设计

### `config.py` — yaml 参数解析

**职责**：加载 yaml 配置文件，暴露所有部署参数。

**输入** (`config/go2.yaml`)：
```yaml
policy_path: "models/go2_policy.pt"
xml_path: "resources/robots/go2/scene.xml"   # MuJoCo 用
num_actions: 12
num_obs: 47

kps: [20.0, 20.0, 20.0, 20.0, 20.0, 20.0,
      20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
kds: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
      0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
default_angles: [0.1, 0.8, -1.5, -0.1, 0.8, -1.5,
                  0.1, 1.0, -1.5, -0.1, 1.0, -1.5]

ang_vel_scale: 0.25
dof_pos_scale: 1.0
dof_vel_scale: 0.05
action_scale: 0.25
cmd_scale: [2.0, 2.0, 0.25]

simulation_dt: 0.002           # MuJoCo 物理步长
control_decimation: 10         # 每 10 帧一次策略推理 (50Hz)
simulation_duration: 60.0      # 仿真时长 (秒)

max_cmd_vel_x: 1.0             # 摇杆 → 速度映射
max_cmd_vel_y: 0.6
max_cmd_ang: 1.0
```

**接口**：
```python
class RlConfig:
    def __init__(self, yaml_path: str): ...

    # 所有字段直接作为属性
    config.policy_path   # str
    config.kps            # list[float] (12,)
    config.default_angles # list[float] (12,)
    config.cmd_scale      # list[float] (3,)
    ...
```

---

### `policy.py` — 策略推理

**职责**：加载 `.pt`，执行前向推理。

```python
import torch

class RlPolicy:
    def __init__(self, path: str):
        self.model = torch.jit.load(path)
        self.model.eval()

    def infer(self, obs: np.ndarray) -> np.ndarray:
        """ obs: (47,) → action: (12,) """
        with torch.no_grad():
            tensor = torch.from_numpy(obs).unsqueeze(0).float()
            action = self.model(tensor).squeeze().numpy()
        return action
```

**关键点**：推理在 CPU 上，无需 CUDA。

---

### `observer.py` — 观测构造

**职责**：从 MuJoCo 状态 + 遥控器指令拼出 47 维观测。

```python
import numpy as np

def get_gravity_orientation(quat):
    """四元数 [w,x,y,z] → 重力方向 (3,)"""
    qw, qx, qy, qz = quat
    return np.array([
        2 * (-qz * qx + qw * qy),
       -2 * ( qz * qy + qw * qx),
        1 - 2 * (qw * qw + qz * qz)
    ])

class RlObserver:
    def __init__(self, config: RlConfig):
        self.cfg = config
        self.last_action = np.zeros(config.num_actions)
        self.step_counter = 0

    def build(self, q, dq, quat, omega, cmd):
        """
        q:     (12,) 当前关节角 [rad]
        dq:    (12,) 当前关节速度 [rad/s]
        quat:  (4,)  基座四元数 [w,x,y,z]
        omega: (3,)  基座角速度 [rad/s]
        cmd:   (3,)  速度指令 [vx, vy, vyaw]
        """
        # 归一化
        q_obs   = (q - self.cfg.default_angles) * self.cfg.dof_pos_scale
        dq_obs  = dq * self.cfg.dof_vel_scale
        omega_s = omega * self.cfg.ang_vel_scale
        grav    = get_gravity_orientation(quat)
        cmd_s   = cmd * self.cfg.cmd_scale * np.array([
            self.cfg.max_cmd_vel_x, self.cfg.max_cmd_vel_y, self.cfg.max_cmd_ang])

        # 步态相位时钟 (周期 0.8s)
        phase = (self.step_counter * self.cfg.simulation_dt) % 0.8 / 0.8
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)

        # 拼 47 维 (顺序必须和训练侧一致)
        obs = np.zeros(self.cfg.num_obs)
        obs[0:3]   = omega_s
        obs[3:6]   = grav
        obs[6:9]   = cmd_s
        obs[9:21]  = q_obs
        obs[21:33] = dq_obs
        obs[33:45] = self.last_action
        obs[45]    = sin_phase
        obs[46]    = cos_phase

        self.step_counter += 1
        return obs

    def update_action(self, action):
        self.last_action = action
```

---

### `pd.py` — PD 力矩计算

**职责**：策略输出（关节位置偏移）→ 目标关节角 → PD → 力矩。

```python
import numpy as np

def compute_torques(action, q, dq, config):
    """
    action: (12,) 策略输出 (关节位置偏移)
    q:      (12,) 当前关节角
    dq:     (12,) 当前关节速度
    返回:    (12,) 关节力矩
    """
    target_q = config.default_angles + action * config.action_scale
    tau = (target_q - q) * config.kps + (0.0 - dq) * config.kds
    tau = np.clip(tau, -80.0, 80.0)
    return tau
```

---

## I/O 层设计

### `serial_joystick.py` — 串口遥控器

**职责**：直读 QLP 协议串口，解析摇杆/按键，输出归一化速度指令。

```python
import serial
import struct
import threading

class QlpJoystick:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.lx = 0.0; self.ly = 0.0   # 左摇杆
        self.rx = 0.0; self.ry = 0.0   # 右摇杆
        self.buttons = [0] * 16
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        # 解析 QLP 协议帧 (参照 serial_controller/data_processor.py)
        while self._running:
            data = self.ser.read(...)
            if valid_frame(data):
                self.lx, self.ly, self.rx, self.ry, self.buttons = parse(data)

    def get_cmd(self):
        """摇杆 → 速度指令 [vx, vy, vyaw]"""
        return (
            self.ly,              # 前后 → 线速度
            self.lx * -1,         # 左右 → 横向速度
            self.rx * -1          # 右摇杆左右 → 角速度
        )

    def is_pressed(self, button):
        return self.buttons[button] == 1

    def close(self):
        self._running = False
        self.ser.close()
```

---

## 入口脚本

### `deploy_mujoco_rl.py` — MuJoCo 仿真

**数据流**（单进程，无 ROS2）：

```
串口 ──▶ QlpJoystick ──▶ cmd [vx,vy,vyaw]
                              │
MuJoCo ──▶ qpos[7:], qvel[6:], quat, omega
                              │
                    ┌─────────┘
                    ▼
           RlObserver.build() → 47 维观测
                    │
                    ▼
           RlPolicy.infer() → 12 维 action
                    │
                    ▼
           pd.compute_torques() → 12 维 tau
                    │
                    ▼
           d.ctrl[:] = tau
           mj_step(m, d)
           viewer.sync()
```

**完整伪代码**：
```python
def main():
    config = RlConfig("config/go2.yaml")
    policy = RlPolicy(config.policy_path)
    observer = RlObserver(config)

    # MuJoCo 初始化
    m = mujoco.MjModel.from_xml_path(config.xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = config.simulation_dt

    # 遥控器 (可选，没接就用默认指令)
    try:
        joystick = QlpJoystick()
        use_joystick = True
    except:
        use_joystick = False

    target_q = np.array(config.default_angles)  # PD 目标初始化为站姿

    with mujoco.viewer.launch_passive(m, d) as viewer:
        start = time.time()
        while viewer.is_running() and time.time() - start < config.simulation_duration:

            # 每帧都执行 PD
            q  = d.qpos[7:]      # 关节角
            dq = d.qvel[6:]      # 关节速度
            tau = (target_q - q) * config.kps + (0 - dq) * config.kds
            d.ctrl[:] = tau
            mujoco.mj_step(m, d)

            # 每 control_decimation 帧执行一次策略推理
            if observer.step_counter % config.control_decimation == 0:
                # 读状态
                quat  = d.qpos[3:7]
                omega = d.qvel[3:6]

                # 读指令
                if use_joystick:
                    cmd = joystick.get_cmd()
                else:
                    cmd = (0.5, 0.0, 0.0)   # 默认前向 0.5 m/s

                # 策略推理
                obs = observer.build(q, dq, quat, omega, cmd)
                action = policy.infer(obs)
                observer.update_action(action)

                # 更新 PD 目标
                target_q = config.default_angles + action * config.action_scale

            viewer.sync()

    if use_joystick:
        joystick.close()
```

---

## 阶段执行计划

### 阶段一：环境准备

| # | 任务 | 说明 |
|---|------|------|
| 1.1 | 确认 Python 依赖 | `pip install torch mujoco numpy pyyaml pyserial` |
| 1.2 | 获取 Go2 MuJoCo 模型 | 需要 `scene.xml` + mesh 文件，从 URDF 转换或从 Unitree 仓库提取 |
| 1.3 | 获取预训练策略 | 从服务器 scp `.pt` 到 `models/` |
| 1.4 | 确认串口权限 | `sudo usermod -a -G dialout $USER`，重新登录生效 |

### 阶段二：核心层实现

| # | 文件 | 内容 |
|---|------|------|
| 2.1 | `core/config.py` | `RlConfig` 类，解析 yaml |
| 2.2 | `core/policy.py` | `RlPolicy` 类，`torch.jit.load` + `infer()` |
| 2.3 | `core/observer.py` | `RlObserver` 类 + `get_gravity_orientation()` |
| 2.4 | `core/pd.py` | `compute_torques()` 函数 |

### 阶段三：I/O 层实现

| # | 文件 | 内容 |
|---|------|------|
| 3.1 | `io/serial_joystick.py` | `QlpJoystick` 类，解析 QLP 协议帧 |

### 阶段四：入口脚本

| # | 文件 | 内容 |
|---|------|------|
| 4.1 | `deploy_mujoco_rl.py` | MuJoCo 仿真主循环 |
| 4.2 | `config/go2.yaml` | Go2 部署参数配置 |

### 阶段五：测试验证

| # | 任务 | 验证标准 |
|---|------|----------|
| 5.1 | 单元测试核心层 | 加载 yaml → 成功；加载 .pt → 推理输出 shape (12,)；观测构造 → shape (47,) |
| 5.2 | 无策略 PD 保持 | 不加载策略，PD 目标设为 default_angles，机器人能站立 |
| 5.3 | 策略驱动行走 | 加载策略，固定指令 `[0.5, 0, 0]`，机器人稳定前向行走 |
| 5.4 | 遥控器响应 | 接遥控器，摇杆改变行走方向和速度 |
| 5.5 | 与传统 PD 对比 | 分别启动两者，确认互不干扰 |

---

## 与传统 PD 框架的关系

```
quadruped/src/
├── controller/quadruped/     ← 传统 PD (C++, ROS2, Gazebo) — 不动
└── rl_controller/            ← RL 框架   (Python, MuJoCo)   — 新增

两者零依赖、零耦合、不同时启动。
```

---

## 待确认项

| # | 项 | 说明 |
|---|-----|------|
| 1 | Go2 的 MuJoCo XML | 需要 `scene.xml`，可从 URDF 用 `urdf2mjcf` 转换，或从 `unitree_rl_gym/resources/robots/` 提取 |
| 2 | 观测维度 | Go2 的 `num_observations` 可能是 48 而非 47，需查 `legged_robot_config.py` |
| 3 | 关节编号顺序 | 策略输出的 12 维顺序必须和 MuJoCo `d.qpos[7:]` 顺序一致 (FR→FL→RR→RL 每腿 hip/thigh/calf) |
| 4 | QLP 协议解析 | 遥控器协议解析逻辑可直接从 `serial_controller/data_processor.py` 移植 |
| 5 | 真机部署 | 后期把 MuJoCo I/O 层换成 `unitree_sdk2py` 的 DDS I/O 层即可，核心层不动 |

---

## 风险点

| 风险 | 应对 |
|------|------|
| MuJoCo XML 不可用 | 用 `urdf2mjcf` 工具从 URDF 转换，或手写 MJCF |
| 策略 `.pt` 不兼容 | 确认训练侧 PyTorch 版本和本机一致（大版本对齐） |
| 观测构造不匹配 | 对照训练侧的 `compute_observations()` 逐维核对索引 |
| PD 参数不合适 | 从训练侧 yaml 直接复制，再微调 |
| 遥控器数据乱码 | 先 `python -c "import serial; ser=serial.Serial('/dev/ttyUSB0'); print(ser.read(100))"` 看原始字节 |
| 仿真画面卡顿 | 降低 `simulation_dt` 或减少 `control_decimation` |



##### 服务器

```
ssh -p 2222 hurricane@10.109.70.55
```



