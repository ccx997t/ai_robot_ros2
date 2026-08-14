# TF契约测试规范

S1-M1 冻结测试方法；实际TF发布从S1-M2开始验收。

## 静态检查

```bash
ros2 run tf2_tools view_frames
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
```

检查生成的树是否只有一个根、父子关系是否符合`docs/architecture/tf_baseline.md`、是否存在重复发布者。

## 动态检查

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_monitor odom base_link
```

验收记录至少保存：发布者、平均/最大延迟、更新频率、最大年龄、测试时长和mode。传感器消息的`header.frame_id`必须能在同一时钟域下解析到`base_link`。

## S1-M4融合所有权

- M4仿真中`base_controller`只发布原始里程计，不发布TF。
- `ekf_filter_node`是`odom -> base_link`的唯一权威发布者。
- 原始轮式里程计使用`/wheel/odom`，融合结果使用公共`/odom`；两者均为`nav_msgs/msg/Odometry`，Frame固定为`odom`和`base_link`。
- `robot_state_publisher`继续独占`base_link -> sensor_link`及各传感器静态外参。
- Gazebo `/clock`必须单向桥接到ROS 2，融合输入、输出和TF使用同一仿真时间域。

## 失败场景

- 缺失Frame或树断开时测试失败。
- 同一父子变换有多个发布者时测试失败。
- 动态TF过期、时间倒退或出现未来时间时测试失败。
- sim和real父子关系不同且未经过接口评审时测试失败。
