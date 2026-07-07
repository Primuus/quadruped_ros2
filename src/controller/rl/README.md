# Go2 RL 部署

这部分和 `quadruped` 的传统 PD 链路完全分开，不走 `ros2 launch`。

## 运行方式

先把训练导出的 TorchScript 策略放到：

```bash
src/controller/rl/models/policy_1.pt
```

这个策略必须是当前 `quadruped_train` 阶段 4 导出的 270 维 Go2 历史观测 actor 策略。训练侧 critic 的 48 维 privileged obs 只用于 value function，不会进入部署模型输入。旧版 48 维或 45 维策略和当前部署代码不兼容，需要重新训练并重新导出。

单帧 45 维观测顺序：

```text
[0:3]   速度命令 vx, vy, yaw_rate
[3:6]   机身角速度
[6:9]   重力方向在机身坐标系下的投影
[9:21]  12 个关节位置偏差
[21:33] 12 个关节速度
[33:45] 上一次策略动作
```

部署代码会维护 6 帧历史缓存，并把 `[当前 45 维, 上一帧 45 维, ..., 更早第 5 帧 45 维]` 输入给策略。

然后直接运行：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py
```

常用参数：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --policy-path src/controller/rl/models/policy_1.pt \
  --xml-path src/controller/rl/resources/robots/go2/scene.xml \
  --device cpu \
```

如果策略文件不在默认位置，就用 `--policy-path` 指定。

接 QLP 遥控器时再加：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py --remote-port /dev/ttyUSB0 --remote-baud-rate 115200
```

按键和摇杆的默认含义：

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

如果判定为摔倒，程序只会停止输出力矩，仿真继续运行；按 `F` 可以重置。
