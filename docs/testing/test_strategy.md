# S1-M1测试与证据基线

| 层级 | 位置 | 当前职责 | 统一入口 |
|---|---|---|---|
| 单元测试 | 各包`test/` | 普通类、格式化和消息构造 | `colcon test` |
| Launch测试 | `ai_robot_bringup/test/` | 节点启动、模式传递和节点图 | `colcon test` |
| 接口契约 | `test/contract/`或包内测试 | 类型、QoS、频率、Frame、参数和异常 | `colcon test`/项目脚本 |
| 集成与场景 | `test/integration/`、`test/scenario/` | 后续子系统与仿真场景 | 后续里程碑启用 |
| rosbag回放 | 外部证据目录 + 元数据 | 可重复数据链和故障复现 | 记录在验收文件 |

测试必须具备可失败的真实断言，不允许使用常量自比较代替接口检查。每次里程碑验收保存命令、结果、Git SHA、参数、日志或bag证据以及已知问题。

S1-M1最低门槛：依赖解析通过、全包构建通过、单元和launch测试通过、`/diagnostics`消息契约通过、非法mode被拒绝、sim/real基础节点不执行硬件控制。
