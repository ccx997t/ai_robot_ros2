# S1-M4资源与性能基线

- 场景：`m4_bringup.launch.py mode:=sim`，完整底盘、传感器、EKF、灰度处理和安全链
- 采集方式：`scripts/measure_m4_resources.py`
- 采集窗口：启动后预热12秒，连续测量20秒
- 安全约束：命令延迟仅发送零速度探测，不驱动机器人运动
- CPU口径：M4 launch完整进程树的多核累计CPU占用，因此允许超过100%
- 内存口径：完整进程树RSS之和，包含Gazebo、ROS 2节点和桥接进程

## 冻结阈值

| 指标 | 通过阈值 |
|---|---:|
| 进程树CPU P95 | ≤650% |
| 进程树RSS P95 | ≤1536 MiB |
| `/odom`频率 | 24–36 Hz |
| `/scan`频率 | 8–12 Hz |
| `/imu/data`频率 | 80–120 Hz |
| 原始/灰度图像频率 | 12–18 Hz |
| `/odom`、`/scan`、原始图像新鲜度P95 | ≤50 ms |
| `/imu/data`新鲜度P95 | ≤20 ms |
| 灰度图像新鲜度P95 | ≤120 ms |
| 原始图像到灰度图像延迟P95 | ≤100 ms |
| `/cmd_vel`到控制器命令延迟P95 | ≤50 ms |

阈值同时固化在采集脚本中；任一指标缺失或越界时脚本返回非零退出码。

## 首次实测

| 指标 | 结果 |
|---|---:|
| CPU P95 / 最大值 | 512.86% / 512.86% |
| RSS P95 / 最大值 | 1268.64 / 1268.76 MiB |
| `/odom` | 29.95 Hz，数据新鲜度P95 10 ms |
| `/scan` | 9.98 Hz，数据新鲜度P95 8 ms |
| `/imu/data` | 99.94 Hz，数据新鲜度P95 1 ms |
| `/camera/image_raw` | 15.13 Hz，数据新鲜度P95 9 ms |
| `/camera/image_mono` | 15.13 Hz，数据新鲜度P95 67 ms |
| 相机处理延迟P95 | 61.33 ms |
| 命令链延迟P95 | 22.93 ms |

首次实测全部落在冻结阈值内。CPU和RSS阈值是当前开发机的软件仿真回归门槛，不代表实体机器人硬件容量要求；更换Gazebo渲染后端、CPU架构或传感器分辨率时必须重新建立基线。

阈值复测同样判定`PASS`：CPU P95 513.11%、RSS P95 1265.81 MiB、相机处理延迟P95 61.74 ms、命令链延迟P95 22.55 ms；所有频率和数据新鲜度均在冻结范围内。

## 执行命令

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/ai_robot_m4_metrics \
  python3 scripts/measure_m4_resources.py --warmup 12 --duration 20
```
