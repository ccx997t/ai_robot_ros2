# 第三方资源卡：launch_testing

- 能力对象：ROS 2节点启动、接口契约和关闭行为自动测试。
- 来源与许可证：ROS 2 Humble官方apt软件源；Apache-2.0。
- 维护状态、版本与锁定提交：`launch_testing`与`launch_testing_ament_cmake` 1.0.14。
- ROS 2 Humble / Ubuntu 22.04 兼容性：本机二进制包和CMake集成可发现。
- 安装与可重复构建步骤：`sudo apt install ros-humble-launch-testing ros-humble-launch-testing-ament-cmake`。
- Topic / Service / Action / QoS / TF / 时间戳要求：测试按接口基线创建订阅或查询节点图，不定义生产接口。
- CPU、GPU、内存、网络和设备需求：无GPU或设备要求；DDS测试需要本地进程通信权限。
- 仿真实现、实体实现与替代方案：优先用于无硬件和仿真测试；实体测试必须额外满足安全规则。
- 最小验证命令和结果：`ros2 pkg prefix launch_testing_ament_cmake`定位到`/opt/ros/humble`；项目测试由`colcon test`驱动。
- 已知问题、风险与回退方式：受限沙箱可能禁止DDS/UDP；此类失败应在允许本地通信的环境复测并区分基础设施问题。
- 准入结论与责任人：S1-M1测试工具准入通过。责任人：项目维护者。
