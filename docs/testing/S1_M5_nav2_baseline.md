# S1-M5 Nav2配置与生命周期基线

## 验收范围

本基线关闭Nav2全局规划器、局部控制器、全局/局部代价地图和生命周期启动配置。固定路线到点、动态避障、任务取消与异常恢复仍由后续场景验收关闭。

## 启动与配置

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ai_robot_bringup navigation.launch.py mode:=sim
```

- 启动入口：`src/ai_robot_bringup/launch/navigation.launch.py`
- 参数文件：`src/ai_robot_bringup/config/nav2_m5.yaml`
- 定位前置：M5地图服务器和AMCL；AMCL独占`map -> odom`
- 生命周期节点：`planner_server`、`controller_server`、`behavior_server`、`bt_navigator`
- 生命周期管理器：`lifecycle_manager_navigation`，自动配置并激活以上四个节点
- `mode:=real`保留接口但不启动Nav2或实体驱动，避免未经准入的实体运动

## 冻结参数

| 对象 | 配置基线 |
|---|---|
| 全局规划 | NavFn `GridBased`，5 Hz，不允许规划进入未知区 |
| 局部控制 | DWB `FollowPath`，15 Hz，最大线速度0.25 m/s、角速度0.70 rad/s |
| 机器人轮廓 | 保守矩形0.52 × 0.40 m |
| 局部代价地图 | `odom`，4 × 4 m滚动窗口，0.05 m分辨率，障碍层和膨胀层 |
| 全局代价地图 | `map`，0.05 m分辨率，静态层、障碍层和膨胀层 |
| 障碍输入 | `/scan`，标记和清除均启用 |
| 膨胀半径 | 0.45 m |
| 行为 | Spin、BackUp、Wait；速度仍低于底盘安全上限 |

Nav2输出保持为公共`/cmd_vel`。运行时接口断言证明`controller_server`发布该接口，`cmd_vel_safety_node`订阅该接口，并且只有安全节点向`/base_controller/cmd_vel_unstamped`发布。因此本配置没有绕过限速和命令超时停车链。

## 自动证据

```bash
bash scripts/build.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select ai_robot_bringup \
  --ctest-args -R 'm5_navigation_contract_test|test_test_m5_navigation_launch.py' \
  --output-on-failure
```

- 静态契约检查规划器/控制器插件、速度与加速度上限、Frame、轮廓、代价地图层和启动入口。
- 动态测试实际启动Gazebo、M4安全与融合链、地图服务器、AMCL及Nav2。
- 四个Nav2生命周期节点均到达`Active`。
- 全局代价地图Frame为`map`，尺寸235 × 197格；局部代价地图Frame为`odom`，尺寸80 × 80格；两者分辨率均为0.05 m。
- 动态接口检查确认导航命令经过安全节点，且底盘控制输入不存在旁路发布者。

ROS 2动态测试需要DDS创建本机UDP socket；受限沙箱中出现`Operation not permitted`属于执行环境限制，应在允许本机ROS 2通信的终端执行。

## 当前边界

本项证明Nav2组件能够在既有地图、定位、TF和安全链上完成配置与激活，并发布有效代价地图；尚不证明目标可达率、到点误差、路径耗时、临时障碍绕行、取消停车或异常恢复结果。
