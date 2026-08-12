# 第三方资源卡：rosbag2

- 能力对象：接口证据记录、问题复现和回放测试。
- 来源与许可证：ROS 2 Humble官方apt软件源；Apache-2.0。
- 维护状态、版本与锁定提交：rosbag2 0.15.16。
- ROS 2 Humble / Ubuntu 22.04 兼容性：本机包可发现，命令入口可用。
- 安装与可重复构建步骤：`sudo apt install ros-humble-rosbag2`。
- Topic / Service / Action / QoS / TF / 时间戳要求：只记录验收清单明确列出的Topic；保留原消息时间戳和bag元数据。
- CPU、GPU、内存、网络和设备需求：无GPU要求；磁盘容量、写入速度和记录Topic带宽必须在测试前核算。
- 仿真实现、实体实现与替代方案：两种模式共用记录流程；实体数据必须遵守隐私、设备和安全规则。
- 最小验证命令和结果：`ros2 pkg prefix rosbag2`定位到`/opt/ros/humble`；记录与回放流程见`docs/testing/rosbag_evidence.md`。
- 已知问题、风险与回退方式：高带宽图像可能丢帧或耗尽磁盘；不得删除原始验收bag，必要时降低测试Topic范围并重新记录。
- 准入结论与责任人：工具准入通过；S1-M1使用最小诊断bag完成流程验证。责任人：项目维护者。
