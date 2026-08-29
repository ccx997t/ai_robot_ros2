# S1-M5 rosbag证据

- 录制日期：2026-08-28
- 录制基线：`9223b05`
- 录制入口：`scripts/record_m5_rosbags.py`
- 机器摘要：`docs/testing/S1_M5_rosbag_manifest.yaml`
- 原始数据：保存在被Git忽略的`evidence/rosbags/`，不提交大体积db3

## 正常导航样本

- 目录：`evidence/rosbags/S1-M5_20260828-193012_sim_normal_9223b05`
- 同步自动测试：`test_test_m5_navigation_scenarios_launch.py`，通过
- 覆盖：重复导航、固定障碍、临时障碍绕行和不可达目标
- 时长：14.587904914秒
- 大小：1,892,352字节db3；11,766条消息
- 关键Topic：`/odom` 303、`/scan` 102、`/cmd_vel` 115、安全出口238、`/amcl_pose` 17、`/diagnostics` 239、`/tf` 605
- db3 SHA-256：`276033eb13d11c62a389d33d7cd9c3ade7bde5dc7850079e332fe48a37a3f13b`
- metadata SHA-256：`2e6fb54570655f1d9ca0e60e934ee2cc2f0a79467aa829cff7653bef039b564f`

## 取消与异常恢复样本

- 目录：`evidence/rosbags/S1-M5_20260828-193012_sim_cancel_fault_9223b05`
- 同步自动测试：`test_test_m5_navigation_fault_recovery_launch.py`，通过
- 覆盖：导航取消、雷达中断、诊断传播、雷达恢复和恢复后再次导航
- 时长：13.490914570秒
- 大小：1,478,656字节db3；9,927条消息
- 关键Topic：`/odom` 256、`/scan` 67、`/cmd_vel` 67、安全出口232、`/amcl_pose` 11、`/diagnostics` 195、`/tf` 494
- db3 SHA-256：`3f58318c2d921d41b4f69b90477bb428e6f80fdc528d9042bab91759804ff7c2`
- metadata SHA-256：`848b787de5af6b5005d5c1c08dec5196a00bbf61ecce90fb2e478083d887465a`

组合样本在同一条时间线上包含取消和异常，自动测试提供事件顺序及结果断言；bag提供接口数据、诊断和运动证据。它不是把同一份文件复制为两个独立场景。

## 可读性与回放

`ros2 bag info`可只读打开两份sqlite3数据库，持续时间、Topic类型和消息计数与机器摘要一致。实际回放使用仅本机回环和隔离DDS域：

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=93 \
  ros2 bag play <normal_dir> --topics /odom
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=93 \
  ros2 topic echo /odom --once --field header.frame_id

ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=94 \
  ros2 bag play <cancel_fault_dir> --topics /diagnostics
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=94 \
  ros2 topic echo /diagnostics --once
```

正常样本成功反序列化`/odom`且Frame为`odom`；取消/异常样本成功反序列化诊断，读到`base/cmd_vel_safety`状态。

## 重新录制

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/record_m5_rosbags.py --scenario all
```

脚本为测试和bag记录器固定相同的`ROS_DOMAIN_ID`与Ignition分区，场景测试失败时样本判定为FAIL。每次运行会生成新时间戳目录及本地JSON摘要；需要纳入Git基线时，应人工核对后更新本机器摘要，不能覆盖既有证据。
