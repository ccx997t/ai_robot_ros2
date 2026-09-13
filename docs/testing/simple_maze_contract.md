# Gazebo对称隔墙迷宫示例

## 冻结几何

- Gazebo世界：`src/ai_robot_sim/worlds/simple_maze.sdf`
- 机器可读契约：`src/ai_robot_sim/config/simple_maze.yaml`
- 场地宽度（X）：10 m，范围`[-5, 5]`
- 场地长度（Y）：12 m，范围`[-6, 6]`
- 中央过道：X范围`[-1.5, 1.5]`，宽3 m
- 内部隔墙：左右各4面，共8面，单面尺寸`3.5 × 0.3 × 1.0 m`
- 隔墙Y坐标：`-3.6、-1.2、1.2、3.6 m`
- 起点：`(0, -5)`，朝`+Y`
- 目标点：`(0, 5)`，朝`+Y`

左隔墙中心X坐标为`-3.25`，右隔墙为`3.25`。每面隔墙长3.5 m，因此内侧端点分别为`-1.5`和`1.5`，中间精确保留3 m过道。

## 打开方法

```bash
cd ~/codex_pro/ai_robot_ros2
source /opt/ros/humble/setup.bash
bash scripts/build.sh
source install/setup.bash
ign gazebo -r src/ai_robot_sim/worlds/simple_maze.sdf
```

如果只启动了服务端，可在新终端打开GUI：

```bash
ign gazebo -g
```

## 修改规则

修改隔墙位置或尺寸时，必须同时更新SDF和`simple_maze.yaml`。`test_simple_maze_world_matches_symmetric_partition_contract`会校验隔墙数量、尺寸、左右对称、Y坐标及中央过道宽度。

该文件是独立示例，不替换M5的`m5_navigation.sdf`。如果后续用它做Nav2定位和导航，还需生成与本几何一致的PGM/YAML静态地图。
