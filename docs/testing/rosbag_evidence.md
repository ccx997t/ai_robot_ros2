# rosbag2验收证据规范

## 目录和命名

验收bag放在不纳入Git的大文件存储位置，目录名使用：

```text
S1-M<里程碑>_<YYYYMMDD-HHMMSS>_<sim|real>_<场景>_<Git短SHA>
```

同目录保存`README.md`或等价元数据，至少记录操作者、主机、ROS发行版、Git提交、mode、参数文件、启动命令、Topic清单、开始/结束时间和结果。

## S1-M1最小记录

启动基础节点后记录诊断：

```bash
ros2 bag record /diagnostics -o <证据目录>
```

停止后检查：

```bash
ros2 bag info <证据目录>
ros2 bag play <证据目录> --topics /diagnostics
```

另一个终端验证回放：

```bash
ros2 topic echo /diagnostics --once
```

## 后续里程碑最小Topic集

| 阶段 | 建议Topic |
|---|---|
| M2底盘 | `/cmd_vel`、`/odom`、`/tf`、`/tf_static`、`/diagnostics` |
| M3传感器 | 加入`/scan`、`/imu/data`和选定相机Topic |
| M4及以后 | 按场景加入融合、地图、路径和状态接口 |

不要无差别记录所有Topic。记录前估算图像和点云带宽、磁盘容量与测试时长。

## 保留和回退

- 不删除原始验收bag、元数据和失败样本；清理前必须明确确认。
- Git只保存小型元数据、命令和结果摘要，不提交大型bag数据库。
- 回放结果必须标注使用的软件提交与参数；接口变更后旧bag若不兼容，应保留并记录迁移或替代样本。
