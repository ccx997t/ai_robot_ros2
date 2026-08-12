# S1-M1 公共接口基线

本文件冻结后续仿真和实体实现必须共同遵守的上层接口。S1-M1 只定义契约并提供最小诊断样例，不实现底盘、传感器、定位或导航功能。

## 命名与模式

- 节点内部优先使用可重映射的相对名称；下表以单机器人根命名空间下的解析结果表示。
- `mode:=sim|real` 只在 bringup 和驱动层选择实现，上层不得根据模式改变消息类型或语义。
- 当前单机器人基线不添加 Frame 前缀；进入多机器人范围前必须重新评审命名空间和 TF 前缀。
- 所有 SI 量使用米、秒、弧度及其组合单位。

## Topic契约

| Topic | 类型 | 方向/所有者 | QoS | 频率基线 | Frame与时间戳 | 超时和异常行为 | 实现阶段 |
|---|---|---|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 安全仲裁出口 → 底盘 | Reliable、KeepLast 1、Volatile | 事件驱动，控制期间建议≥10 Hz | 无Header；接收时刻用于看门狗 | 超过`command_timeout_seconds`必须停车；非有限值拒绝；不得绕过限速和急停 | S1-M2 |
| `/odom` | `nav_msgs/msg/Odometry` | 底盘/融合 → 上层 | Reliable、KeepLast 10、Volatile | 目标≥20 Hz | Header=`odom`，child=`base_link`，采样时间戳 | 数据过期、Frame错误或协方差无效时诊断降级，不伪造有效位姿 | S1-M2 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 雷达驱动/适配 → 上层 | BestEffort、KeepLast 5、Volatile | 由设备冻结，目标≥5 Hz | Header=`laser_link`，传感器采样时间 | 超时或范围元数据无效时发布诊断；不得重复旧数据冒充新帧 | S1-M3 |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU驱动/适配 → 上层 | BestEffort、KeepLast 10、Volatile | 由设备冻结，目标≥50 Hz | Header=`imu_link`，传感器采样时间 | 未提供字段的协方差首元素置`-1`；超时或非有限值诊断降级 | S1-M3 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 各节点/诊断聚合 → 运维 | Reliable、KeepLast 10、Volatile | 基础心跳1 Hz；事件可立即发布 | Header使用节点当前ROS时钟 | 使用OK/WARN/ERROR/STALE；节点退出即停止更新，由监控判断超时 | S1-M1 |

传感器Topic采用BestEffort以匹配实时数据流；控制、状态和诊断采用Reliable。具体驱动若无法满足，应通过适配层统一并记录偏差，不能静默改变上层契约。

## 参数基线

| 参数 | 类型 | 默认值 | 合法范围/枚举 | 失败行为 |
|---|---|---|---|---|
| `mode` | string | `sim` | `sim`、`real` | launch阶段拒绝其他值 |
| `use_sim_time` | bool | `false` | bool | sim bringup设为`true`，real设为`false` |
| `command_timeout_seconds` | double | `0.5` | `(0, 2.0]` | 配置非法时节点不得激活控制出口 |
| `max_linear_speed_mps` | double | `0.3` | `(0, 1.0]`，实体值还受硬件限制 | 非法时拒绝启动控制出口 |
| `max_angular_speed_rps` | double | `0.8` | `(0, 3.0]`，实体值还受硬件限制 | 非法时拒绝启动控制出口 |

当前基础节点只消费`mode`；其余参数为后续底盘安全链预留，S1-M2实现前必须增加参数验证测试。

## 控制与安全责任

```text
遥控 / Nav2 / 测试程序
          ↓
命令仲裁、限速与急停监督
          ↓
唯一底盘命令出口
          ↓
sim驱动或real驱动
```

- 上游不得直接连接最终硬件命令接口。
- ROS软件停车不能替代实体物理急停和MCU通信看门狗。
- `mode:=real` 的驱动缺失、状态未知或通信丢失必须保持安全停止。

## 变更规则

公共消息类型、QoS语义、Frame或安全失败行为变更必须更新本文件、相关资源卡、契约测试和验收记录。仅驱动层可感知`sim`与`real`差异。
