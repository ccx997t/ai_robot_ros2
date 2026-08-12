# S1-M1 架构基线

## 当前状态

| 层级 | 状态 | 责任位置 |
|---|---|---|
| 工程、构建、测试、文档 | S1-M0已验收 | 根目录、`scripts/`、`docs/` |
| 资源、接口与测试基线 | S1-M1实施中 | `docs/resources/`、`docs/architecture/`、各包`test/` |
| 最小C++/Python节点 | 已建立 | `src/ai_robot_base`、`src/ai_robot_tools` |
| 结构化诊断 | 最小样例已建立 | `ai_robot_tools/health_reporter`、`/diagnostics` |
| 模型、仿真本体、传感器、定位、导航、感知 | 仅候选与目录预留 | 后续里程碑 |
| 实体硬件与安全回路 | 未实现 | 后续实体阶段 |
| AI Agent与桥接 | 明确排除 | 不创建相关软件包 |

## 分层和依赖方向

```text
标准消息与公共契约
        ↑
驱动 / 适配 / 诊断
        ↑
定位、导航、安全等子系统
        ↑
bringup与非AI任务层
```

上层不得依赖仿真或实体驱动细节。`mode:=sim|real`只在bringup和驱动层选择实现。

## 冻结文档

- 公共Topic、QoS、参数和异常行为：`docs/architecture/interface_baseline.md`
- TF、Frame和时间语义：`docs/architecture/tf_baseline.md`
- 对象、候选资源、实现和测试：`docs/resources/resource_matrix.md`
- 测试层级与证据：`docs/testing/test_strategy.md`
- rosbag记录与回放：`docs/testing/rosbag_evidence.md`

## 阶段边界

S1-M1 可以实现用于验证契约的最小节点和测试，但不实现机器人模型、Gazebo本体、底盘控制、实体驱动、SLAM、Nav2、视觉感知或AI Agent。候选资源和目录存在不代表后续里程碑已经通过。
