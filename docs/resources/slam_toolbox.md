# 第三方资源卡：slam_toolbox

- 能力对象：S1-M5二维激光SLAM、地图序列化和定位候选能力。
- 来源：ROS 2 Humble官方apt软件源，不修改第三方源码。
- 许可证：LGPL，以本机`/opt/ros/humble/share/slam_toolbox/package.xml`为准。
- 版本：ROS包版本2.6.10；Debian包`2.6.10-1jammy.20260804.222728`。
- 兼容性：Ubuntu 22.04、ROS 2 Humble；本机包发现、可执行程序和系统依赖检查通过。
- 安装：`sudo apt install ros-humble-slam-toolbox`。
- 资源需求：CPU运行，无专用设备需求；地图内存、CPU和TF延迟将在M5完整建图场景中实测并冻结。

## 接口与边界

- 计划输入：`/scan` (`sensor_msgs/msg/LaserScan`)、`/odom`及TF中的`odom -> base_link`。
- 计划输出：`/map` (`nav_msgs/msg/OccupancyGrid`)和建图模式下的`map -> odom`。
- 建图模式由`slam_toolbox`独占发布`map -> odom`；不得与AMCL同时承担该TF所有权。
- 所有时间戳和TF必须使用与Gazebo `/clock`一致的仿真时间。
- M5优先评估在线异步建图；最终参数、QoS、地图更新频率、分辨率和保存服务以自动场景验收结果为准。
- 安装及节点发现只代表资源准入，不代表地图质量、保存、重载或定位验收通过。

## 已完成的最小准入检查

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix slam_toolbox
ros2 pkg executables slam_toolbox
dpkg-query -W ros-humble-slam-toolbox
rosdep check --from-paths src --ignore-src
```

结果：包位于`/opt/ros/humble`；可发现同步、异步、定位、地图与定位组合节点及地图合并工具；工作区系统依赖全部满足。安装包同时提供在线同步、在线异步、离线、终身建图和定位参数模板，项目不会直接修改这些第三方文件。

## 风险与回退

- 雷达时间戳、TF延迟、里程计漂移和错误的扫描匹配参数会导致地图重影或跳变，必须在固定世界中自动验收。
- 建图与AMCL并行发布`map -> odom`会造成TF冲突，启动契约必须阻止该组合。
- apt升级后必须重跑地图质量、保存重载、TF唯一性和导航回归。
- 卸载命令：`sudo apt remove ros-humble-slam-toolbox`；功能集成失败时回退到S1-M4关闭提交`9d23bb4`。

- 准入结论：**依赖准入通过**；版本、许可证、节点发现和`rosdep`已验证。SLAM功能与质量指标仍待M5场景验收。
