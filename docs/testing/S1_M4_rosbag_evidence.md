# S1-M4 rosbag验收证据

## 正常集成样本

- 目录：`evidence/rosbags/S1-M4_20260813-180236_sim_normal_3733648`
- 录制基线：`3733648`
- 场景：完整M4仿真正常运行
- 时长/大小/消息：14.508589712秒，18.0 MiB，18,761条
- Topic：`/clock`、`/cmd_vel`、受控命令、轮式及融合里程计、关节、雷达、IMU、灰度图像、TF和诊断，共11个有效Topic
- 主接口统计：`/odom` 409、`/wheel/odom` 681、`/scan` 131、`/imu/data` 1,312、`/camera/image_mono` 199、`/diagnostics` 105
- 回放：`ros2 bag play <dir> --topics /odom`后，`ros2 topic echo /odom --once`成功读取消息
- 数据库SHA-256：`a4f8b41c5ebeb17426cd7043d7aa3540fb1a2875f56a3af0c9a04cfb9b149e52`
- 元数据SHA-256：`f346000527ba5882fd58dcda7d24ae7b5d85184e2dfc32320b1d21c83aa54410`

## 故障与安全样本

- 目录：`evidence/rosbags/S1-M4_20260813-180236_sim_fault_safety_retry1_3733648`
- 录制基线：`3733648`加当前待提交的诊断启动竞态修正
- 场景：雷达Frame错误及恢复、1.0 m/s限速、命令中断、0.5秒超时停车及0.2 m/s恢复
- 自动断言：`test_m4_fault_safety_launch.py`同步通过
- 时长/大小/消息：5.586695493秒，79.9 KiB，432条
- Topic统计：命令源35、`/cmd_vel` 11、安全出口91、雷达源/故障中继/公共输出各90、`/diagnostics` 25
- 回放：`ros2 bag play <dir> --topics /diagnostics`后，`ros2 topic echo /diagnostics --once`成功读取消息
- 数据库SHA-256：`4abd94bf3a766339fc3d25cb3d5988bb247b194e9537748eb503580dd537bf02`
- 元数据SHA-256：`a67c33b44ef3c9396177ad76f611aea5ecb03e3665697d3191d126576b7712b3`

## 失败样本保留

首次故障录制目录`S1-M4_20260813-180236_sim_fault_safety_3733648`因bag订阅者加入时扩大启动竞态，测试在限速诊断到达前取值而失败。该样本仅0.378565022秒、22条消息，不作为通过证据，但按证据规范保留。测试已改为在2秒窗口内等待对应诊断，替代样本随后通过。

Git仅保存本摘要；原始数据库和元数据保存在被忽略的`evidence/rosbags/`目录，不删除、不提交。
