# 对象—资源—实现—测试矩阵

本矩阵冻结 S1-M1 的候选资源、公共接口和准入状态。标记为“候选”的资源尚未进入功能实现，不代表后续里程碑通过。

| 对象 | 选定/候选资源 | 仿真实现 | 实体实现 | 公共接口或输出 | 独立验证 | S1-M1 状态 |
|---|---|---|---|---|---|---|
| 仿真平台 | Gazebo Fortress 6.18.0、`ros_gz` 0.244.25 | `ros_gz_sim` | 不适用 | `/clock`及显式桥接接口 | GUI、Transport、`/clock`桥接 | 已准入 |
| 机器人模型 | URDF、Xacro 2.1.1、`robot_state_publisher` 3.0.3 | 共用描述包 | 共用描述包 | `/robot_description`、`/tf`、`/tf_static` | Xacro解析和TF契约 | 基础资源已准入，M2实现 |
| 坐标变换 | TF2 0.25.20 | 仿真驱动提供动态TF | 实体驱动提供动态TF | `map -> odom -> base_link -> sensor_link` | Frame、频率、时间戳检查 | 已准入 |
| 差速底盘 | `ros2_control` 2.54.0、`diff_drive_controller` 2.53.3 | `gz_ros2_control` 0.7.20 | MCU硬件接口 | `/cmd_vel`、`/odom`、TF | 超时、限速、方向和里程计 | M2已验收 |
| 安全与诊断 | `diagnostic_msgs`，后续评估 `diagnostic_updater` | 共用监督逻辑 | 共用监督逻辑与硬件状态 | `/diagnostics` | 消息、频率、状态和值字段 | 标准消息已准入 |
| 激光雷达 | Gazebo 6.18.0传感器、`ros_gz_bridge` 0.244.25 | Gazebo GPU雷达与独立桥接 | 品牌驱动/适配节点 | `/scan` | 类型、QoS、频率、Frame | M3仿真依赖已准入，功能待实现 |
| 相机 | Gazebo 6.18.0、`ros_gz_image` 0.244.25、`image_transport` 3.1.13 | Gazebo相机与独立图像桥接 | 品牌驱动/适配节点 | `/camera/image_raw`、`/camera/camera_info` | 图像、CameraInfo、频率、时间戳 | M3仿真依赖已准入，功能待实现 |
| IMU与融合 | Gazebo 6.18.0 IMU、`ros_gz_bridge` 0.244.25；`robot_localization`留待M4 | Gazebo IMU与独立桥接 | 品牌IMU/适配节点 | `/imu/data` | 协方差、频率、Frame、异常输入 | M3原始IMU依赖已准入，功能待实现 |
| SLAM | `slam_toolbox`候选 | 复用标准接口 | 复用标准接口 | 地图及定位相关标准接口 | 固定数据集建图与重载 | 候选，M5前准入 |
| 导航 | Nav2、AMCL候选 | 复用标准接口 | 复用标准接口 | Nav2 Action、路径和状态 | 到点、取消、超时和不可达 | 候选，M5前准入 |
| 数据证据 | rosbag2 0.15.16 | 共用 | 共用 | 记录选定Topic | 记录、信息检查、回放 | 已准入 |
| Launch测试 | `launch_testing` 1.0.14 | 共用 | 共用 | 节点图和接口断言 | `colcon test` | 已准入 |

所有版本以资源卡或目标机 `dpkg-query` 证据为准。第三方资源不得直接修改源码；候选转为“已准入”前必须补充兼容性、许可证、最小验证和回退记录。
