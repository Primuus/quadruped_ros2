# RL 策略模型目录

把 `quadruped_train` 从同一个 `estimator + actor` 部署模型直接导出的四足机器人策略放到这里，默认文件名：

```text
policy_1.pt
policy_1.onnx
```

- `policy_1.pt`：TorchScript 基准模型。
- `policy_1.onnx`：ONNX Runtime 模型，供后续 C++ 部署使用。

两者必须直接来自同一个 PyTorch 权重，ONNX 不经过 TorchScript 二次转换。当前 Go2 实例的外部输入是 270 维 `float32` 历史观测，输出是 12 维 `float32` 动作；训练侧 critic 的 privileged obs 不进入部署模型输入。新增机器人时，模型输入、输出必须和对应 YAML 中的 `num_obs`、`num_actions` 一致。

仓库默认不提交模型权重，避免把大文件或临时训练结果放进 Git。
