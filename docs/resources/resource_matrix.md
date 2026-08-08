# 对象—资源—实现—测试矩阵（S1-M1 待完善）

| 对象 | 候选资源 | 仿真实现 | 实体实现 | 独立测试 | 状态 |
|---|---|---|---|---|---|
| 机器人模型 | URDF/Xacro | 待选 | 不适用 | TF 检查 | 预留 |
| 差速底盘 | ros2_control | 待选 | MCU 接口 | `/cmd_vel`、`/odom` | 预留 |
| 雷达 | 品牌驱动 | 待选 | 待选 | `/scan` | 预留 |
| 相机 | image_pipeline | 待选 | 待选 | 图像契约 | 预留 |
| IMU | robot_localization | 待选 | 待选 | `/imu/data` | 预留 |
| SLAM | slam_toolbox | 待选 | 复用 | 建图场景 | 预留 |
| 导航 | Nav2 / AMCL | 待选 | 复用 | 到点与取消 | 预留 |
