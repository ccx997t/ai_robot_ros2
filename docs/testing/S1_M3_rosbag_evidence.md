# S1-M3 rosbag验收证据

## 证据标识

- 录制时间：2026-08-13 04:58–04:59 PDT
- 录制提交：`b789710`
- ROS 2：Humble
- 模式：`sim`
- 启动：`ros2 launch ai_robot_sim sensors.launch.py`
- 安全链：`/cmd_vel -> cmd_vel_safety_node -> /base_controller/cmd_vel_unstamped`
- 场景：静止采样、0.2 m/s前进、0.5 rad/s左转、-0.2 m/s后退和超时停车

## 四类样本

| 对象 | 本地证据目录 | 时长 | 大小 | 消息数 | 主接口消息数 |
|---|---|---:|---:|---:|---:|
| 雷达 | `evidence/rosbags/S1-M3_20260813-045732_sim_lidar_b789710` | 38.868 s | 1.7 MiB | 2,549 | `/scan`: 388 |
| 相机 | `evidence/rosbags/S1-M3_20260813-045732_sim_camera_b789710` | 44.901 s | 149.8 MiB | 3,846 | image: 677，info: 677 |
| IMU | `evidence/rosbags/S1-M3_20260813-045732_sim_imu_b789710` | 38.828 s | 2.8 MiB | 9,905 | `/imu/data`: 3,871 |
| 编码器 | `evidence/rosbags/S1-M3_20260813-045732_sim_encoder_b789710` | 42.513 s | 2.3 MiB | 11,653 | `/joint_states`: 4,232 |

每个样本均包含`/diagnostics`、`/tf`和`/tf_static`。编码器样本另包含`/cmd_vel`和安全链输出`/base_controller/cmd_vel_unstamped`。

## 完整性

| 对象 | 数据库 SHA-256 | `metadata.yaml` SHA-256 |
|---|---|---|
| 雷达 | `8cb1952788042a7edd473ea3b74082aa1800b8161f262e63e8577bfeaf27c62b` | `3724dce5265f314273ee780f8942a98cd86263ab3726a3a3a1737ba0f30d154f` |
| 相机 | `ffc70b01eb03b08c5e3f7b0e2fb1756788ffb7982a71505512a5fa9ebf21bd5e` | `68a5141e4711949191b1d7bb53623ef3789bc3f637f141600e0ae6cf9cdab019` |
| IMU | `f7c1f081f534cd2f268c4e745b0e238fb597a362bbe1cb93b84f6718c57978f4` | `fc6d8d44d11e13d137ad5490b546ec2492aaf07b1b7fd7b80f5dcfe146720d2e` |
| 编码器 | `f34e21557a82caa3c5e7cb361b8fe27c03fb93b6a8a5a170ab8a28a6b48c5ed0` | `3c01b842d9f107ef3576c976272671d30255b66e2329ed86972d9dbe977ba54a` |

## 回放验证

四个样本均已通过`ros2 bag play <dir> --topics <main_topic>`回放，并成功读取一条主接口消息：

| 对象 | 回放主接口 | `header.frame_id` | 结果 |
|---|---|---|---|
| 雷达 | `/scan` | `laser_link` | 通过 |
| 相机 | `/camera/image_raw` | `camera_optical_link` | 通过 |
| IMU | `/imu/data` | `imu_link` | 通过 |
| 编码器 | `/joint_states` | `base_link` | 通过 |

按`rosbag_evidence.md`的保留规则，Git仅保存本摘要；完整bag保存在本地忽略目录`evidence/rosbags/`。

