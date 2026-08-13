# AI Robot ROS 2 协作规则

- S1-M2已验收，当前实施S1-M3：仿真传感器单元；目录占位或依赖安装不等于传感器验收通过。
- 不实现 AI Agent、自然语言、模型调用或 AI—ROS 2 桥接。
- 坐标树目标：`map -> odom -> base_link -> sensor_link`。
- 预留公共接口：`/cmd_vel`、`/odom`、`/scan`、`/imu/data`、`/diagnostics`。
- 启动文件必须显式支持 `mode:=sim|real`；上层接口不应感知底层模式。
- 不得绕过急停、命令超时停车、速度限制或底层失联保护。
- 实体测试必须有物理急停、人工监护、低速和限定区域。
- 第三方依赖不改源码；版本、许可证、验证和回退信息记录到 `docs/resources/`。
- 改动后运行 `bash scripts/build.sh` 和 `bash scripts/test.sh`。
