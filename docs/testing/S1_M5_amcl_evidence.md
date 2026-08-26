# S1-M5 AMCL定位证据

## 入口与契约

- 启动：`ros2 launch ai_robot_bringup localization.launch.py mode:=sim`
- 参数：`src/ai_robot_bringup/config/amcl_m5.yaml`
- 地图：Git固化的`m5_baseline.yaml`与`m5_baseline.pgm`
- 输入：`/scan`、`/odom`、`/initialpose`及TF
- 输出：`/amcl_pose`和`map -> odom`
- TF所有权：定位入口不启动`slam_toolbox`；AMCL独占`map -> odom`，EKF继续独占`odom -> base_link`。

AMCL采用差速运动模型和likelihood-field雷达模型。局部基线初始采用500至2000个粒子；完整地图全局恢复复验后提高并冻结为1000至8000个粒子、每次更新180束雷达。地图服务器与AMCL均由`lifecycle_manager_localization`按顺序配置、激活。

## 动态测试

`test_m5_localization_launch.py`实际启动M5 Gazebo世界、传感器、安全节点、融合里程计、地图服务器和AMCL，并完成以下断言：

1. 从`(0.45, 0.25, 0.20)`带偏差初始位姿启动，初始平面偏差约0.515 m。
2. 仅向公共`/cmd_vel`发送0.30 rad/s旋转命令，经过既有限速和超时停车链，使用雷达与里程计更新粒子滤波。
3. 断言最终平面误差小于初始偏差，`/amcl_pose`协方差非负且`map -> odom`存在。
4. 注入`(1.0, -0.8, 0.20)`错误定位，确认偏差超过0.7 m。
5. 通过公共`/initialpose`重新初始化到当前已知位置，断言恢复误差小于0.25 m。

动态场景已通过，测试用时约18.7秒。测试只在仿真模式运动，不直接发布到底盘控制器输入。

## 边界

当前Git地图是出生点周边的局部旋转采样地图，因此本项证明局部可观测区内的AMCL初始化、偏差收敛和人工重新初始化恢复能力。全地图全局定位、绑架恢复、长路线定位漂移和导航到点仍需后续完整地图及Nav2场景关闭。
