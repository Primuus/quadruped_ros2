# RL 策略模型目录

把 `quadruped_train` 导出的四足机器人 TorchScript 策略放到这里，默认文件名：

```text
policy_1.pt
```

当前 Go2 实例加载的是 270 维历史观测 TorchScript 策略。训练侧 critic 的 privileged obs 不进入部署模型输入；导出的模型内部包含 `estimator + actor`，外部输入维度必须和对应 YAML 中的 `num_obs` 一致。

仓库默认不提交模型权重，避免把大文件或临时训练结果放进 Git。
