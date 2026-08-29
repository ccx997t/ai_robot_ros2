# S1-M5 完整地图、全局定位与长路线证据

- 验证日期：2026-08-25
- 场景：`m5_navigation.sdf`
- 自动测试：`test_m5_global_localization_launch.py`

## 完整地图

`m5_complete`是依据冻结仿真场景生成的仿真真值地图，不冒充SLAM实测地图。生成入口为`python3 scripts/generate_m5_ground_truth_map.py`，工件由PGM、Nav2 YAML及SHA-256机器摘要组成。

| 指标 | 结果 |
|---|---:|
| 尺寸 | 244 × 204格 |
| 分辨率 | 0.05 m/格 |
| 原点 | [-6.1, -5.1, 0.0] |
| 占用格 | 5,172 |
| 空闲格 | 44,604 |
| 未知格 | 0 |

静态测试逐项核对文件哈希、栅格统计、全部固定模型中心及四个目标点；导航与定位入口默认加载该完整地图，局部SLAM保存工件`m5_baseline`仍被独立保留。

## 绑架与全局恢复

测试先向AMCL注入位于`(4.0, 3.0)`的高置信错误估计，随后调用`/reinitialize_global_localization`均匀撒布粒子并通过公共`/cmd_vel`执行安全旋转观测，全程不提供正确初始位姿。每轮最长60秒，最多三轮。

为完整地图将粒子范围冻结为1,000至8,000、每次更新180束雷达。实测第二轮恢复，耗时65.551秒，恢复后相对仿真出生点的位置误差0.093米；验收阈值为三轮内恢复且误差小于0.55米。

## 长路线与定位残差

固定路线为`原点 → east_room → 原点 → west_bay`，三个NavigateToPose目标全部成功。命令仍经过`/cmd_vel → cmd_vel_safety_node → /base_controller/cmd_vel_unstamped`。

| 指标 | 实测 | 阈值 |
|---|---:|---:|
| 到达率 | 3/3 | 3/3 |
| 里程计累计路线 | 17.144 m | > 12.0 m |
| 路线耗时 | 83.246 s | 记录基线 |
| 最大AMCL到点残差 | 0.162 m | < 0.55 m |
| 安全出口线速度 | ≤ 0.25 m/s | ≤ 0.2501 m/s |

2026-08-28补充阈值复验：四次最大角度误差为0.309、0.410、0.435和0.421 rad，最终阈值按分布冻结为0.50 rad；代表性三段耗时26.532、35.131、34.662秒（单段阈值100秒），总耗时96.326秒（阈值240秒），累计路线17.783米。最终确认轮82.725秒、最大角度残差0.421 rad并通过。资源与障碍净距基线见`docs/testing/S1_M5_resource_baseline.md`。

这里的“漂移”指标是各NavigateToPose完成时AMCL估计相对冻结目标坐标的残差，不是外部运动捕捉真值误差；后续若引入独立Gazebo真值桥，应补充全程绝对轨迹误差。

## 执行结果

```bash
bash scripts/build.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select ai_robot_bringup \
  --ctest-args -R '^test_test_m5_global_localization_launch.py$' \
  --output-on-failure
```

结果：1个专项launch测试通过，总用时2分40秒。随后`bash scripts/test.sh`全量执行84项结果，新增M5专项再次通过（155.98秒）；全量仍有M3 IMU运动采样和M4相机P95延迟两个底层时序断言失败，CTest/包级汇总显示4个失败记录。因此本专项关闭，但M5最终全量基线与rosbag证据仍未关闭。
