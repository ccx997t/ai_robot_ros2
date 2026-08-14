# 第三方资源卡：robot_localization

- 能力对象：S1-M4编码器里程计与IMU状态融合。
- 来源：ROS 2 Humble官方apt软件源，不修改第三方源码。
- 许可证：Apache License 2.0，以已安装`package.xml`为准。
- 版本：ROS包版本3.5.4；Debian包`3.5.4-1jammy.20260804.195527`。
- 兼容性：Ubuntu 22.04、ROS 2 Humble；M4使用`ekf_node`实施二维状态融合。
- 安装：`sudo apt install ros-humble-robot-localization`。
- 资源需求：CPU运行，无GPU和专用设备需求；具体CPU、内存和延迟由M4资源基线验收冻结。

## 接口与边界

- 冻结输入：原始编码器里程计`/wheel/odom` (`nav_msgs/msg/Odometry`)和`/imu/data` (`sensor_msgs/msg/Imu`)。
- 冻结输出：`/odom` (`nav_msgs/msg/Odometry`)，Reliable/Volatile，30 Hz目标；Header Frame为`odom`，child Frame为`base_link`。
- 二维融合字段：轮式里程计提供平面位置、偏航、平面速度和偏航角速度；IMU提供偏航和偏航角速度。
- 输出位姿和速度协方差必须为有限非零值；过程噪声矩阵冻结在`ai_robot_bringup/config/ekf_m4.yaml`。
- 只实施二维底盘状态估计，不提前实施M5的SLAM、定位或Nav2。
- 只允许一个权威节点发布`odom -> base_link`；融合与底盘原始TF不得重复发布。

## 已完成的最小准入检查

```bash
ros2 pkg list | grep '^robot_localization$'
ros2 pkg executables robot_localization
dpkg-query -W ros-humble-robot-localization
rosdep check --from-paths src --ignore-src
```

结果：`robot_localization`可发现，`ekf_node`、`ukf_node`和`navsat_transform_node`可执行；工作区系统依赖检查通过。

## 风险与回退

- 错误的Frame、协方差、时间戳或重复TF发布会产生跳变、漂移或TF冲突，必须由launch契约测试覆盖。
- apt升级后必须重跑M4融合和故障注入测试，不以“节点可启动”代替功能验收。
- 功能集成失败时禁用M4融合启动入口，回退到S1-M3最终关闭提交`4fbd776`；卸载命令为`sudo apt remove ros-humble-robot-localization`。

- 准入结论：依赖与可执行节点准入通过；M4输入、输出、Frame、TF所有权和协方差契约已通过自动集成测试，资源指标仍待M4后续验收。
