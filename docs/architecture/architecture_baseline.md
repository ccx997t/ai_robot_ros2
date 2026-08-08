# S1-M0 架构基线

## 当前状态

| 层级 | 状态 | 责任位置 |
|---|---|---|
| 工程、构建、测试、文档 | 已建立 | 根目录、`scripts/`、`docs/` |
| 最小 C++ / Python 节点 | 已建立 | `src/ai_robot_base`、`src/ai_robot_tools` |
| 模型、仿真、传感器、定位、导航、感知 | 已预留，未实现 | `models/`、`worlds/`、`config/` |
| 实体硬件、安全回路 | 未实现 | `config/real/` |
| AI Agent 与桥接 | 明确排除 | 不创建相关软件包 |

## 公共接口预留

| 名称 | 类型 | 阶段 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | S1-M2 起 |
| `/odom` | `nav_msgs/msg/Odometry` | S1-M2 起 |
| `/scan` | `sensor_msgs/msg/LaserScan` | S1-M3 起 |
| `/imu/data` | `sensor_msgs/msg/Imu` | S1-M3 起 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | S1-M1 起 |
| `map -> odom -> base_link` | TF2 | S1-M2 起 |

`ai_robot_base` 当前不驱动电机；`ai_robot_tools` 当前只输出日志型心跳。接口、QoS 和错误码在 S1-M1 评审后冻结。
