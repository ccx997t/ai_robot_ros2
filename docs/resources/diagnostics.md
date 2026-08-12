# 第三方资源卡：ROS 2 diagnostics消息基线

- 能力对象：节点健康、模式、安全状态和异常的结构化可观察性。
- 来源与许可证：ROS 2 Humble官方apt软件源；`diagnostic_msgs`采用Apache-2.0/BSD系列ROS许可证，最终以本机包版权文件为准。
- 维护状态、版本与锁定提交：`diagnostic_msgs` 4.9.1；当前最小实现直接发布标准消息，`diagnostic_updater`保留为后续候选。
- ROS 2 Humble / Ubuntu 22.04 兼容性：本机标准消息可用，构建、单元测试和launch契约测试通过。
- 安装与可重复构建步骤：`sudo apt install ros-humble-diagnostic-msgs`；项目依赖由`rosdep`解析。
- Topic / Service / Action / QoS / TF / 时间戳要求：`/diagnostics`使用`diagnostic_msgs/msg/DiagnosticArray`、Reliable、KeepLast 10、Volatile，基础频率1 Hz；消息Header使用ROS时钟，定时触发使用稳态时钟以便仿真时钟未就绪时仍可观察。
- CPU、GPU、内存、网络和设备需求：资源消耗低，无GPU或实体设备要求。
- 仿真实现、实体实现与替代方案：上层消息契约共用；实体模式后续增加MCU、急停、电池和设备状态，但不得移除基础安全字段。
- 最小验证命令和结果：`ros2 topic echo /diagnostics --once`收到结构化OK状态；`ros2 topic info /diagnostics --verbose`确认唯一发布者与Reliable/Volatile；launch测试验证消息、频率、QoS和mode。
- 已知问题、风险与回退方式：诊断不能替代硬件急停；监控侧必须对停止更新进行超时判断。回退到M1基线消息字段，接口变更需同步契约测试。
- 准入结论与责任人：S1-M1标准诊断接口准入通过。责任人：项目维护者。
