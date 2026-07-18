# 第三方依赖

## Unitree Actuator SDK

`unitree_actuator_sdk/` 是 Unitree 官方仓库的固定源码快照，用于后续
GO-M8010-6 电机后端。工程不会修改该目录中的厂商源码，机器人关节映射、
机械零位、方向、减速比换算和安全检查均由自己的硬件适配层负责。

上游信息记录在 `unitree_actuator_sdk.version`。更新 SDK 时必须：

1. 记录新的完整提交号。
2. 保留上游 `LICENSE`。
3. 重新执行本地构建和 Python 导入检查。
4. 重新验证 GO-M8010-6 的命令与反馈字段。

构建当前主机对应的 C++ 示例和 Python 绑定：

```bash
bash src/controller/rl/third_party/build_unitree_actuator_sdk.sh
```

脚本只编译和导入模块，不打开串口，也不会向电机发送命令。生成的当前
Python ABI 模块和 `build/` 目录不会提交到 Git。

真实硬件阶段应使用 udev 规则或专用用户组授予 USB-RS485 访问权限，不要
使用 `sudo` 启动完整 RL 控制器。
