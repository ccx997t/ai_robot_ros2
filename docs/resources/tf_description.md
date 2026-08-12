# 第三方资源卡：TF2、Xacro与robot_state_publisher

- 能力对象：机器人描述解析、静态传感器外参和TF树发布。
- 来源与许可证：ROS 2 Humble官方apt软件源；具体许可证以各二进制包版权文件为准。
- 维护状态、版本与锁定提交：`tf2_ros` 0.25.20、Xacro 2.1.1、`robot_state_publisher` 3.0.3。
- ROS 2 Humble / Ubuntu 22.04 兼容性：本机Humble二进制包可发现，S1-M1准入通过。
- 安装与可重复构建步骤：`sudo apt install ros-humble-tf2-ros ros-humble-xacro ros-humble-robot-state-publisher`。
- Topic / Service / Action / QoS / TF / 时间戳要求：使用`/tf`和`/tf_static`；责任、Frame和时间规则见`docs/architecture/tf_baseline.md`。
- CPU、GPU、内存、网络和设备需求：无GPU和实体设备要求；模型mesh可能增加内存和加载时间。
- 仿真实现、实体实现与替代方案：两种模式复用描述；动态TF由各自驱动提供，不由上层感知模式。
- 最小验证命令和结果：`ros2 pkg prefix tf2_ros xacro robot_state_publisher`等价逐包检查均定位到`/opt/ros/humble`。
- 已知问题、风险与回退方式：多发布者、错误Frame方向和过期时间戳会破坏整树；回退到S1-M1锁定apt版本和描述文件提交。
- 准入结论与责任人：基础资源准入通过；机器人模型和运行TF测试在S1-M2实施。责任人：项目维护者。
