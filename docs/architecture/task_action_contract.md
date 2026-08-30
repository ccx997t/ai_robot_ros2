# S1-M6 统一任务Action契约

## 公共接口

- Action名称：`/execute_robot_task`
- Action类型：`ai_robot_interfaces/action/ExecuteRobotTask`
- 所有者：任务管理器；客户端不应绕过该接口同时操作其Nav2子Action。
- QoS：使用ROS 2 Action默认服务与状态QoS，不额外暴露任务控制Topic。
- 模式：`sim|real`使用同一Action类型和语义；底层模式由bringup决定。

## Goal

| 字段 | 契约 |
|---|---|
| `task_id` | 非空的调用方任务标识；活动任务重复ID以`ERROR_BUSY`拒绝 |
| `target_pose` | Frame必须为`map`；位置和姿态必须有限，四元数必须可归一化 |
| `perception_mode` | `0=NONE`，`1=IMAGE_CHECK`；其他值以`ERROR_INVALID_GOAL`拒绝 |
| `navigation_timeout` | 正Duration表示本次阈值；0表示使用节点默认值 |
| `perception_timeout` | 正Duration表示本次阈值；0表示使用节点默认值 |

越界校验使用当前场景的机器人中心可行边界，必须在调用Nav2前完成。

## Feedback

| 阶段 | 值 | 语义 |
|---|---:|---|
| `VALIDATING` | 0 | 校验Goal、场景边界与安全先决条件 |
| `NAVIGATING` | 1 | Nav2子Action活动 |
| `PERCEIVING` | 2 | 已到点，执行选定感知子任务 |
| `FINALIZING` | 3 | 清理子任务并组装最终结果 |

`progress`为`[0.0, 1.0]`范围的单调非减建议值；客户端不得使用它判定最终成功。`detail`仅用于人类可读诊断，不作为机器分支条件。

## Result和错误码

| 代码 | 常量 | 语义 |
|---:|---|---|
| 0 | `ERROR_NONE` | 任务全部完成 |
| 100 | `ERROR_INVALID_GOAL` | 字段、Frame、姿态、枚举或超时非法 |
| 101 | `ERROR_OUT_OF_BOUNDS` | 目标超出当前场景允许边界 |
| 102 | `ERROR_BUSY` | 已有互斥任务或重复活动`task_id` |
| 200 | `ERROR_NAVIGATION_REJECTED` | Nav2拒绝子目标或导航Action不可用 |
| 201 | `ERROR_NAVIGATION_UNREACHABLE` | Nav2确认目标不可达或规划/控制失败 |
| 202 | `ERROR_NAVIGATION_TIMEOUT` | 导航超过本次或默认时限 |
| 300 | `ERROR_CANCELED` | 客户端取消；子Action已取消且安全停车 |
| 400 | `ERROR_PERCEPTION_UNAVAILABLE` | 所需感知能力或数据源不可用 |
| 401 | `ERROR_PERCEPTION_TIMEOUT` | 到点感知未在时限内完成 |
| 402 | `ERROR_PERCEPTION_INVALID` | 感知结果时间戳、Frame或内容无效 |
| 500 | `ERROR_SAFETY_STOPPED` | 安全链、急停或底层失联保护终止任务 |
| 900 | `ERROR_INTERNAL` | 无法归类的任务管理器内部失败 |

- 只有`success=true`时`error_code`才能为`ERROR_NONE`。
- 失败、取消或超时必须使用非零错误码并填写`message`。
- `elapsed`使用同一ROS时钟计算整个任务耗时。
- `perception_result`为M6最小可追踪结果；`PERCEPTION_NONE`或感知未执行时必须为空。后续若需要结构化感知Schema，必须新增字段/消息并升级契约，不得改变已冻结错误码。

## 取消、超时和安全

1. 任务管理器收到取消后必须先取消活动子Action，再完成父Action取消。
2. 导航超时必须取消Nav2子Action，不得仅返回超时而留下运动任务。
3. 任务管理器不直接发布底盘速度；停车结果由Nav2取消与现有安全链联合保证和验证。
4. 安全触发的优先级高于导航、感知和任务成功；不得自动清除急停或底层保护。

## 版本规则

- 允许在不改变旧字段语义的前提下追加新枚举和新字段。
- 禁止重排、复用或改变已发布数字错误码的语义。
- 任何不兼容变更必须新增Action类型或显式升级主版本，并保留迁移和回退方案。
