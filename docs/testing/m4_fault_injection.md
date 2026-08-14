# S1-M4故障注入契约

故障注入器只允许用于仿真验收，不连接实体驱动。它是位于数据源与被测节点之间的可恢复中继，默认关闭故障并原样转发消息。

## 故障映射

| 验收对象 | `message_type` | `fault_mode` | 注入结果 |
|---|---|---|---|
| 传感器中断 | `scan`或`imu` | `drop` | 丢弃消息，触发数据陈旧 |
| 时间戳异常 | `scan`、`imu`或`odometry` | `zero_stamp` | 时间戳置零 |
| Frame错误 | `scan`、`imu`或`odometry` | `bad_frame` | Frame改为`bad_frame`参数 |
| 融合输入异常 | `imu`或`odometry` | `nonfinite_data` | 注入NaN数据或协方差 |
| 命令中断 | `twist` | `drop` | 丢弃速度命令，保留安全节点超时停车能力 |

## 启动与恢复

示例（雷达中断）：

```bash
ros2 launch ai_robot_bringup m4_fault_injection.launch.py \
  message_type:=scan fault_mode:=drop \
  input_topic:=/sim/scan output_topic:=/fault/scan
```

启用故障：

```bash
ros2 service call /fault_injector/enable std_srvs/srv/SetBool "{data: true}"
```

恢复正常转发：

```bash
ros2 service call /fault_injector/enable std_srvs/srv/SetBool "{data: false}"
```

注入器在`/diagnostics`发布`fault_injection/<message_type>`状态和接收、注入、转发计数。真正的被测诊断与安全断言由后续M4故障传播集成测试完成。
