# S1-M5地图保存与重载证据

- 地图前缀：`src/ai_robot_bringup/maps/m5_baseline`
- 生成入口：`ros2 launch ai_robot_bringup mapping.launch.py mode:=sim`
- 保存服务：`/map_saver/save_map`（`nav2_msgs/srv/SaveMap`）
- 重载入口：`ros2 launch ai_robot_bringup map_server.launch.py mode:=sim`

## Git摘要

| 项目 | 固化值 |
|---|---|
| 尺寸 | 235 × 197格 |
| 分辨率 | 0.05 m/格 |
| 原点 | `[-5.93, -4.93, 0.0]` |
| 占用/空闲/未知 | 57 / 1227 / 45011格 |
| YAML SHA-256 | `e98ea2fb47272fc2d9f82b16d8d1e7875e473cdd894390f2ff19d99ad6bf2987` |
| PGM SHA-256 | `568ab5977107654dcb84ad72d972ffbd270d5f9d2094a0140a391453fe428d7c` |

机器可读摘要位于`m5_baseline_manifest.yaml`。静态测试重新计算文件摘要、读取PGM像素并核对元数据和三类栅格数量，任何未同步的地图修改都会使测试失败。

## 一致性结果

动态建图测试实际调用保存服务，检查YAML/PGM已生成，并核对文件名、模式、分辨率、原点和图像尺寸。独立回载测试由生命周期管理器激活`map_server`，从Git地图发布`/m5_reloaded_map`，确认Frame、尺寸、分辨率、原点及57/1227/45011三类栅格完全一致。

首次回载发现`free_thresh: 0.25`会把PGM灰度205的未知区解释为空闲区，因此契约修正为`free_thresh: 0.19`。修正后未知区在保存和回载之间保持不变。

验证命令：

```bash
bash scripts/build.sh
bash scripts/test.sh
ros2 launch ai_robot_bringup map_server.launch.py mode:=sim
```

当前地图是SLAM入口旋转采样形成的局部基线，用于证明保存、版本追踪和回载链路；完整世界覆盖仍需在后续固定路线导航验收中扩展，不能把本证据解释为M5全部关闭。
