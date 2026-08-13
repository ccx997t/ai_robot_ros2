# S1-M2 rosbag验收证据

## 证据标识

- 证据目录：`evidence/rosbags/S1-M2_20260813-020650_sim_base_motion_e11a883`
- 录制提交：`e11a883`
- ROS 2：Humble
- 模式：`sim`
- 场景：前进、后退、原地旋转、限速和0.5秒命令超时停车
- 录制时间：2026-08-13 02:07:07 至 02:08:20
- 时长：72.943914437秒
- 大小：6.9 MiB
- 消息总数：23,313

## Topic统计

| Topic | 类型 | 消息数 |
|---|---|---:|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 70 |
| `/base_controller/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | 1,357 |
| `/odom` | `nav_msgs/msg/Odometry` | 3,646 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 10,892 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 1 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 7,200 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 147 |

## 场景与结论

1. 以10 Hz发布20条`linear.x=0.2`前进命令。
2. 以10 Hz发布20条`linear.x=-0.2`后退命令。
3. 以10 Hz发布20条`angular.z=0.5`原地旋转命令。
4. 以10 Hz发布10条`linear.x=1.0, angular.z=-2.0`超限命令；安全出口按既有自动测试限制为`0.3/-0.8`。
5. 输入结束后继续录制，覆盖0.5秒墙钟看门狗停车窗口和持续零速度输出。

七个要求Topic均已记录，控制输入、受控输出、里程计、关节状态、TF和诊断证据链完整。运动数值及安全断言同时由`test_sim_base_launch.py`自动验证。

## 完整性

- 数据库：`S1-M2_20260813-020650_sim_base_motion_e11a883_0.db3`
- 数据库SHA-256：`6dd447d4f1d147a274fcc9a4ba40f03d0b0ccc6c9beaf6f3f99eb189219f9a02`
- 原始`metadata.yaml` SHA-256：`86a8b079d0a6fc8c6dc75dd78873db9a1ff7177fe8d1713b2396e52bcb75d17d`

按照`rosbag_evidence.md`的保留规则，Git仅保存本摘要；完整bag保存在本地忽略目录中。
