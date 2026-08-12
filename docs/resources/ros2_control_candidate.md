# 第三方资源卡：ros2_control候选

- 能力对象：S1-M2差速底盘控制与S1-M8实体硬件接口。
- 来源与许可证：ROS 2生态官方项目；Apache-2.0，最终以安装包版权文件为准。
- 维护状态、版本与锁定提交：Humble版本待安装后锁定；当前目标机尚未安装`controller_manager`和`diff_drive_controller`。
- ROS 2 Humble / Ubuntu 22.04 兼容性：Humble为目标候选；S1-M2前必须通过apt来源、最小样例和Gazebo Fortress集成验证。
- 安装与可重复构建步骤：候选命令`sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers`，执行前核对目标apt源可用性。
- Topic / Service / Action / QoS / TF / 时间戳要求：上层保持`/cmd_vel`、`/odom`和`odom -> base_link`契约；控制管理服务仅属于驱动/bringup层。
- CPU、GPU、内存、网络和设备需求：仿真无实体设备；实体实现需要MCU通信、实时性和安全看门狗评估。
- 仿真实现、实体实现与替代方案：仿真使用Gazebo控制系统插件，实体使用自研硬件接口；两者不改变上层契约。
- 最小验证命令和结果：尚未执行，S1-M2准入前验证控制器加载、限速、命令超时、里程计和TF。
- 已知问题、风险与回退方式：控制链配置错误可造成失控或多发布者；实体启用前必须保留物理急停和MCU失联停车。未准入时回退为不加载任何底盘控制器。
- 准入结论与责任人：候选，不作为S1-M1已安装资源；S1-M2前必须另行准入。责任人：项目维护者。
