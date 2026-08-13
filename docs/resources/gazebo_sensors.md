# 第三方资源卡：Gazebo仿真传感器与ROS桥接

- 能力对象：S1-M3激光雷达、相机和IMU仿真数据生成与ROS 2接口桥接。
- 来源：Ubuntu/ROS 2 Humble官方apt软件源，不修改第三方源码。
- 版本：Gazebo Fortress 6.18.0；`ros_gz_bridge`和`ros_gz_image` 0.244.25；`image_transport`和`camera_info_manager` 3.1.13；`sensor_msgs` 4.9.1。
- 许可证：Gazebo/`ros_gz`/`sensor_msgs`为Apache-2.0；`image_transport`和`camera_info_manager`为BSD。准确声明以目标机已安装包的`package.xml`和版权文件为准。
- 兼容性：Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress；与M2使用的`ros_gz_sim`版本一致。

## 已完成的最小准入检查

```bash
ign gazebo --versions
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix ros_gz_image
ros2 pkg prefix image_transport
ros2 pkg prefix camera_info_manager
ros2 pkg prefix sensor_msgs
rosdep check --from-paths src --ignore-src
```

Gazebo的Sensors和IMU系统插件、`parameter_bridge`及`image_bridge`可在目标机发现，依赖准入通过。传感器消息内容、频率、QoS、Frame、时间戳和噪声尚须在M3功能实现后单独验收。

## 接口与替换边界

| 对象 | 仿真实现 | 公共ROS 2接口 | 实体替换边界 |
|---|---|---|---|
| 激光雷达 | Gazebo GPU雷达＋`ros_gz_bridge` | `/scan`、`sensor_msgs/msg/LaserScan` | 品牌驱动或适配节点保持同一契约 |
| 相机 | Gazebo相机＋`ros_gz_image`/桥接 | `/camera/image_raw`、`/camera/camera_info` | 实体相机驱动保持图像和CameraInfo契约 |
| IMU | Gazebo IMU＋`ros_gz_bridge` | `/imu/data`、`sensor_msgs/msg/Imu` | 品牌驱动或适配节点保持同一契约 |
| 编码器 | M2 `joint_state_broadcaster` | `/joint_states` | 实体`ros2_control`硬件接口 |

## 风险与回退

- 无界面环境的相机和GPU雷达依赖渲染后端，EGL警告不等于消息可用；必须用自动接口测试判定。
- Gazebo Transport与ROS 2消息的QoS和时间语义不同，桥接后必须显式验证，不依赖默认值猜测。
- M3失败时回退到S1-M2最终关闭提交`2337440`，不影响底盘安全链和M2验收证据。
