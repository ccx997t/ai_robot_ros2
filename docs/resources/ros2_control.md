# 第三方资源卡：ros2_control与Gazebo控制插件

- 能力对象：S1-M2差速底盘控制，以及后续实体阶段的硬件接口抽象。
- 来源与许可证：ROS 2 Humble官方apt软件源；`ros2_control`、`ros2_controllers`和`gz_ros2_control`采用Apache-2.0，最终以本机二进制包版权文件为准。
- 维护状态、版本与锁定提交：`ros2_control` / `controller_manager` 2.54.0；`ros2_controllers`、`diff_drive_controller`、`joint_state_broadcaster` 2.53.3；`gz_ros2_control` 0.7.20。
- ROS 2 Humble / Ubuntu 22.04 兼容性：Ubuntu 22.04.5与ROS 2 Humble二进制包已安装，ROS包索引可发现，S1-M2依赖准入通过。
- 安装与可重复构建步骤：`sudo apt update && sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gz-ros2-control`。
- Topic / Service / Action / QoS / TF / 时间戳要求：上层保持`/cmd_vel`、`/odom`和`odom -> base_link`契约；控制器管理服务只属于bringup/驱动层。详细规则见`docs/architecture/interface_baseline.md`和`tf_baseline.md`。
- CPU、GPU、内存、网络和设备需求：仿真无实体设备要求；实体实现需要MCU通信、实时性、急停和底层看门狗评估。
- 仿真实现、实体实现与替代方案：仿真使用`gz_ros2_control`，实体使用后续自研/选定的硬件接口；两者不得改变上层接口或绕过安全链。
- 最小验证命令和结果：`ros2 pkg prefix`已确认`ros2_control`、`controller_manager`、`diff_drive_controller`、`joint_state_broadcaster`和`gz_ros2_control`均位于`/opt/ros/humble`；`controller_manager`提供`ros2_control_node`和`spawner`等执行入口。
- 已知问题、风险与回退方式：控制器参数、关节方向或多发布者配置错误可能造成失控。实体启用前必须具备物理急停和MCU失联停车。回退时禁用控制器加载并恢复S1-M1无底盘输出基线，不删除验收日志。
- 准入结论与责任人：依赖准入通过；控制器加载、运动、限速、超时停车和里程计将在S1-M2模型实现后验收。责任人：项目维护者。
