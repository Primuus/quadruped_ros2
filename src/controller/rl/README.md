# Go2 RL 部署

这部分和 `quadruped` 的传统 PD 链路完全分开，不走 `ros2 launch`。

## 运行方式

先把训练导出的 TorchScript 策略放到：

```bash
src/controller/rl/models/go2_policy.pt
```

然后直接运行：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py --headless
```

常用参数：

```bash
python3 src/controller/rl/rl_controller/deploy_mujoco_rl.py \
  --policy-path src/controller/rl/models/go2_policy.pt \
  --xml-path src/controller/rl/resources/robots/go2/scene.xml \
  --device cpu \
  --headless
```

如果策略文件不在默认位置，就用 `--policy-path` 指定。
