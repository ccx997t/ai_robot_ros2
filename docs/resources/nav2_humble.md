# 第三方资源卡：Navigation2与AMCL（ROS 2 Humble）

- 能力对象：S1-M5地图加载、二维定位、全局规划、局部控制、行为树导航、恢复行为和任务取消。
- 来源：ROS 2 Humble官方apt软件源，不修改第三方源码。
- 许可证：`nav2_bringup`、规划器和控制器为Apache-2.0；`nav2_amcl`为LGPL-2.1-or-later；`nav2_map_server`为Apache-2.0与BSD-3-Clause。均以各已安装`package.xml`为准。
- 版本：ROS包版本1.1.20；Debian包`ros-humble-navigation2`为`1.1.20-1jammy.20260804.223401`，`ros-humble-nav2-bringup`为`1.1.20-1jammy.20260804.225407`。
- 兼容性：Ubuntu 22.04、ROS 2 Humble；本机核心包、可执行程序及系统依赖检查通过。
- 安装：`sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`。
- 资源需求：CPU运行，无专用设备需求；完整导航进程树CPU、RSS、频率和延迟将在M5场景中实测并冻结。

## 接口与边界

- 定位输入：静态地图、`/scan`、TF中的`odom -> base_link`及`/initialpose`。
- 定位输出：`/amcl_pose`及定位模式下由AMCL独占发布的`map -> odom`。
- 导航主接口：`nav2_msgs/action/NavigateToPose`；必须验证反馈、成功、失败、取消和不可达结果。
- 规划与控制使用`/map`、`/odom`、`/scan`和完整`map -> odom -> base_link -> sensor_link`坐标树。
- Nav2速度命令必须接入现有`/cmd_vel -> cmd_vel_safety_node -> /base_controller/cmd_vel_unstamped`安全链，不得直达控制器。
- 生命周期启动顺序、机器人轮廓、代价地图、规划器、控制器、恢复行为和行为树参数由项目配置冻结，不直接修改`/opt/ros/humble`模板。
- 安装及节点发现只代表资源准入，不代表定位、规划、避障或到点验收通过。

## 已完成的最小准入检查

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix nav2_bringup
ros2 pkg prefix nav2_amcl
ros2 pkg executables nav2_map_server
ros2 pkg executables nav2_amcl
ros2 pkg executables nav2_planner
ros2 pkg executables nav2_controller
ros2 pkg executables nav2_bt_navigator
ros2 pkg executables nav2_behaviors
ros2 pkg executables nav2_lifecycle_manager
dpkg-query -W ros-humble-navigation2 ros-humble-nav2-bringup
rosdep check --from-paths src --ignore-src
```

结果：`nav2_bringup`、地图服务器、AMCL、规划器、控制器、行为树导航器、行为服务器和生命周期管理器均位于`/opt/ros/humble`；对应核心可执行程序可发现；工作区系统依赖全部满足。

## 风险与回退

- 示例参数面向其他机器人尺寸，不能直接作为本项目的footprint、膨胀层、速度或加速度基线。
- AMCL与SLAM同时发布`map -> odom`会造成TF冲突；建图和定位导航必须使用互斥启动入口。
- 多个速度源会造成命令竞争，M5必须通过节点图和动态测试证明导航命令没有绕过安全节点。
- apt升级后必须重跑地图重载、定位、固定路线、障碍、取消、超时、不可达和M4安全回归。
- 卸载命令：`sudo apt remove ros-humble-navigation2 ros-humble-nav2-bringup`；功能集成失败时回退到S1-M4关闭提交`9d23bb4`。

- 准入结论：**依赖准入通过**；版本、许可证、核心组件发现和`rosdep`已验证。Nav2与AMCL功能指标仍待M5自动场景验收。
