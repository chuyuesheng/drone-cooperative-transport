# Drone Cooperative Transport Simulation

基于 Webots R2025a、ArduPilot SITL 和 WSL2 的无人机吊运仿真项目。当前版本完成了第一架 Mavic 2 Pro 的自动起飞、5 米定高和长时间稳定悬停基线。

## 环境要求

- Windows + WSL2（Ubuntu）
- Webots R2025a
- ArduPilot SITL：`~/ardupilot/build/sitl/bin/arducopter`
- Python 环境：`~/venv-ardupilot`
- `pymavlink` 与 MAVProxy

## 启动

在第一个 WSL 终端中：

```bash
cd "/mnt/d/无人机协同调运/20260816/version20260816"
bash wsl/launch_single_drone.sh
```

随后在 Webots 中打开 `worlds/mavic_2_pro.wbt` 并运行仿真。

在第二个 WSL 终端中执行自动悬停测试：

```bash
cd "/mnt/d/无人机协同调运/20260816/version20260816"
~/venv-ardupilot/bin/python3 wsl/test_hover.py \
  --master tcp:127.0.0.1:5762 \
  --altitude 5 \
  --duration 180 \
  --land-after
```

测试脚本会检查高度误差、高度波动、水平偏移和最大倾角，验收成功时输出：

```text
PASS: hover acceptance criteria satisfied
```

## 当前控制边界

当前版本是单机吊载稳定悬停基线。ArduPilot 负责电机升力和高度闭环，Webots 控制器通过 `level_attitude_lock` 抑制柔性长绳引起的横滚与俯仰角冲量。双机 SITL、共享载荷和自由姿态下的协同吊运控制仍待实现。

仿真参数关闭了部分真实飞行安全检查，仅限软件在环仿真，不可直接用于真机。
