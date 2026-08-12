# AI Robot ROS 2

基于 ROS 2 Humble 的非 AI 机器人软件工作区。S1-M0 工程与管理基线已经验收，当前实施范围为 **S1-M1：资源、接口与测试基线**；后续目录仅为规划预留，不代表相应能力已实现或验收。

## 快速开始

```bash
source /opt/ros/humble/setup.bash
bash scripts/environment_check.sh
bash scripts/build.sh
source install/setup.bash
ros2 launch ai_robot_bringup sim_bringup.launch.py
```

测试：

```bash
bash scripts/test.sh
```

## 当前软件包

| 软件包 | 语言 | 职责 |
|---|---|---|
| `ai_robot_base` | C++ | 最小底盘状态节点与命令看门狗逻辑 |
| `ai_robot_tools` | Python | 发布标准 `/diagnostics` 的最小健康节点 |
| `ai_robot_bringup` | Launch | 统一 `mode:=sim|real` 启动入口 |

架构见 [docs/architecture/architecture_baseline.md](docs/architecture/architecture_baseline.md)，公共接口与TF规则分别见 [interface_baseline.md](docs/architecture/interface_baseline.md) 和 [tf_baseline.md](docs/architecture/tf_baseline.md)。实体硬件、仿真本体、传感器、导航与 AI 均未实现。
