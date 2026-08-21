# S1-M5 Nav2配置与生命周期基线

## 验收范围

本基线关闭Nav2全局规划器、局部控制器、全局/局部代价地图、生命周期启动配置，以及局部地图范围内的固定路线、障碍、任务取消和雷达超时恢复场景。

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
| 局部控制 | DWB `FollowPath`，15 Hz，线速度-0.20至0.25 m/s、最大角速度0.70 rad/s |
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

## 固定路线与障碍自动场景

`test_m5_navigation_scenarios_launch.py`在独立Gazebo/Nav2进程中验证：

- 固定局部路线`local_west(-0.6, 0.0)`与`local_home(0.0, 0.0)`往返两轮，4个`NavigateToPose`目标全部返回`SUCCEEDED`。
- 为适配起点东侧固定墙和差速底盘，DWB允许最低-0.20 m/s的受控倒车；命令仍经公共安全链，绝对速度不超过既有上限。
- 固定墙目标落在致命占用区，`ComputePathToPose`明确返回`ABORTED`且无路径。
- 测试运行时通过Gazebo用户命令生成0.7 × 0.7 × 0.8 m实体障碍；雷达可见表面进入全局障碍层，重新规划成功且路径中心不穿过障碍几何范围。
- 四面封闭且位于当前已知自由空间分量之外的目标返回`ABORTED`且无路径。

公开的`nav_msgs/msg/OccupancyGrid`代价值使用0至100量纲；自动断言以99作为致命占用下限，未混用Costmap2D内部0至255量纲。

执行命令：

```bash
colcon test --packages-select ai_robot_bringup \
  --ctest-args -R test_test_m5_navigation_scenarios_launch.py \
  --output-on-failure
```

## 当前边界

本项已证明局部固定路线到点、固定障碍拒绝、临时障碍入图与绕行、不可达目标拒绝。当前Git地图仍是起点附近的局部建图结果，因此完整世界的长路线成功率、全局到点误差、长程漂移和绑架恢复仍需后续验收。

## 导航取消、雷达超时与恢复场景

专用入口`navigation_fault_recovery.launch.py`仅在`mode:=sim`下启用。它将Gazebo雷达适配输出隔离为`/m5_fault/scan_source`，再由可恢复丢包中继发布公共`/scan`；独立监控器只订阅公共接口并向`/diagnostics`发布`sensors/navigation_lidar`，因此诊断结论来自Nav2实际消费的数据流。

自动launch测试验证：

- 实际`NavigateToPose`产生非零命令后取消，Action最终状态为`CANCELED`，安全链底盘出口出现零速。
- 启用雷达丢包后，公共`/scan`停止增长，`fault_injection/scan`为`WARN`，`sensors/navigation_lidar`为`STALE`且消息为`sensor data stale or not received`。
- 关闭丢包后公共雷达恢复，故障注入器和传感器诊断均回到`OK`。
- 恢复后重新发送`local_west`和`local_home`两个导航目标，均返回`SUCCEEDED`。

执行命令：

```bash
colcon test --packages-select ai_robot_bringup \
  --ctest-args -R test_test_m5_navigation_fault_recovery_launch.py \
  --output-on-failure
```

该场景关闭仿真范围内的导航取消、公共雷达超时诊断传播及数据恢复后再次导航；它不宣称当前Nav2会在活动任务中仅凭雷达超时诊断自动触发急停，实体传感器断线保护仍需硬件阶段验证。

## 实际导航安全场景

动态测试向`NavigateToPose`发送实际目标并验证以下结果：

- Nav2和安全出口均出现非零运动命令，证明断言来自真实导航控制而不是测试节点伪造速度。
- Nav2输出不超过0.25 m/s和0.70 rad/s，安全出口不超过0.30 m/s和0.80 rad/s。
- 取消导航后，底盘命令看门狗在0.40至1.20秒窗口内报告`command timeout; stop sent`并发布零速；目标值为0.5秒，窗口包含定时器和调度误差。
- 测试中将`base_controller`切换为`inactive`模拟底层失联。控制输入仍可到达，但0.8秒观察窗内机器人位移小于0.03 m，证明失活控制器不会继续驱动机器人。
- 将控制器重新激活后状态恢复为`active`，证明该仿真故障可恢复。

本场景关闭导航运动中的限速、命令超时停车和底层控制器失联保护验证；物理通信断线和硬件驱动失联仍属于实体阶段。
