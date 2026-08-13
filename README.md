# AI Robot ROS 2

基于 ROS 2 Humble 的非 AI 机器人软件工作区。S1-M0至S1-M3已经验收，当前实施范围为 **S1-M4：运动、感知与安全集成**；后续目录仅为规划预留，不代表相应能力已实现或验收。

## 快速开始

```bash
source /opt/ros/humble/setup.bash
bash scripts/environment_check.sh
bash scripts/build.sh
source install/setup.bash
ros2 launch ai_robot_bringup m4_bringup.launch.py mode:=sim
```

仿真启动后，底盘只接受公共安全入口 `/cmd_vel`；安全节点将限速后的命令送往控制器，并在 0.5 秒无新命令或收到非有限值时发送停车命令：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}"
```

实体模式目前只启动安全的基础节点，不加载任何实体驱动：

```bash
ros2 launch ai_robot_bringup m4_bringup.launch.py mode:=real
```

测试：

```bash
bash scripts/test.sh
```

## 当前软件包

| 软件包 | 语言 | 职责 |
|---|---|---|
| `ai_robot_base` | C++ | 最小底盘状态节点与命令看门狗逻辑 |
| `ai_robot_description` | Xacro | 差速底盘模型、惯量、碰撞体和Frame |
| `ai_robot_sim` | Launch/YAML/SDF | Gazebo Fortress world和`ros2_control`差速控制器 |
| `ai_robot_tools` | Python | 发布标准 `/diagnostics` 的最小健康节点 |
| `ai_robot_bringup` | Launch | 统一 `mode:=sim|real` 启动入口 |

架构见 [docs/architecture/architecture_baseline.md](docs/architecture/architecture_baseline.md)，公共接口与TF规则分别见 [interface_baseline.md](docs/architecture/interface_baseline.md) 和 [tf_baseline.md](docs/architecture/tf_baseline.md)。实体硬件、导航与 AI 均未实现；当前正在实施S1-M4仿真集成。
