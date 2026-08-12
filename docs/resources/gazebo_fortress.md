# 第三方资源卡：Gazebo Fortress 与 ros_gz

- 能力对象：S1-M2 起的机器人仿真平台；S1-M0 仅完成环境准入，不实现仿真机器人。
- 来源与许可证：Ubuntu / ROS 2 apt 软件源；Gazebo 与 `ros_gz` 采用 Apache-2.0。`ros_gz` 本机包版权文件位于 `/usr/share/doc/ros-humble-ros-gz/copyright`。
- 维护状态、版本与锁定提交：Gazebo Fortress 6.18.0；`ros_gz` 0.244.25（二进制包构建修订号以本机 `dpkg-query` 输出为准）。
- ROS 2 Humble / Ubuntu 22.04 兼容性：Ubuntu 22.04.5 LTS、ROS 2 Humble 本机验证通过。
- 安装与可重复构建步骤：`sudo apt update && sudo apt install ros-humble-ros-gz`；随后执行 `source /opt/ros/humble/setup.bash`。
- Topic / Service / Action / QoS / TF / 时间戳要求：Gazebo Transport 已确认 `/clock`、`/world/shapes/clock`、状态、场景和位姿 Topic；ROS 2 Topic 必须通过 `ros_gz_bridge` 显式桥接。S1-M1 冻结具体桥接、QoS、TF 和时间戳契约。
- CPU、GPU、内存、网络和设备需求：S1-M0 未冻结性能指标；GUI 需要可用桌面与 OpenGL/EGL，apt 安装和 Gazebo Fuel 资源获取需要网络。
- 仿真实现、实体实现与替代方案：仿真选用 Fortress + `ros_gz`；实体实现不适用。Gazebo Classic 11 虽在本机存在，但不是项目基线，不得混用 `gazebo_ros` 和 `ros_gz` 的 launch、插件或依赖。
- 最小验证命令和结果：`ign gazebo --versions` 输出 `6.18.0`；`ros2 pkg list | grep '^ros_gz'` 检出 `ros_gz_sim` 与 `ros_gz_bridge`；`ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="shapes.sdf"` GUI 正常显示并由操作者手动关闭；`ign topic -l` 检出 `/clock` 与 `/world/shapes/*`；`ros_gz_bridge` 将 `ignition.msgs.Clock` 桥接为 `rosgraph_msgs/msg/Clock`，`ros2 topic echo /clock --once` 成功收到 `sec: 4, nanosec: 447000000`。
- 已知问题、风险与回退方式：GUI 输出 `libEGL warning: egl: failed to create dri2 screen`，本次未影响显示和干净退出。回退时固定 apt 包版本或移除 `ros-humble-ros-gz`，不得删除原始验收日志；技术栈变更需重新准入。
- 准入结论与责任人：S1-M0 环境准入通过，Gazebo 本体、GUI、Transport Topic 和 ROS 2 `/clock` 桥接均已验证。责任人：项目维护者。
