# ToGo Prototype 感知 MPC 项目交接书

交接日期：2026-07-13

## 1. 本窗口目标与完成情况

本窗口围绕 ToGo Prototype 的感知 MPC 完成了以下工作：

1. 明确算法配置层级：`model_settings.algorithm` 仅支持 `DDP` / `SQP`；当选择 `DDP` 时，`ddp.algorithm` 可选择 `SLQ` / `ILQR`。
2. 针对下楼梯时同侧前后轮、轮子与小腿连接处过近的问题，实现了独立于地形 SDF 的机器人自碰撞代价。
3. 修复了新增模型设置字段后启动节点出现的 `std::bad_alloc` / `exit code -6`。
4. 对随机箱体（1.4）与随机粗糙地面（1.5）的轮子穿地现象进行了数据化分析。

自碰撞功能已能编译、启动和运行；粗糙地面穿模的根因已定位，但尚未实施针对 1.5 的地形/接触模型修复。

## 2. 当前配置快照

当前 ToGo 配置文件：

- `src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/config/togo_prototype/task.info`
- `src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/config/togo_prototype/frame_declaration.info`

当前重要值如下：

```info
model_settings.algorithm                 SQP
minimumSameSideFootSeparation            0.0
muSelfCollision                          0.5
deltaSelfCollision                       0.005
selfCollisionActivationDistance          0.08
initialRobotState position x             0.00
```

`minimumSameSideFootSeparation = 0.0` 表示禁用“同侧前足世界 x 必须在后足前方”的简化约束。它不能表示轮子与小腿的真实距离，当前由下述自碰撞代价替代。

注意：导出目录 `date/20260713_110814` 的 1.5 数据生成时，机身初始 x 为 `0.50 m`（可由导出的机身位姿确认）；它不是当前 `task.info` 中 `x=0.00` 的验证结果。

## 3. 新增自碰撞架构

### 3.1 设计原则

原有 `collisions` 配置会进入地形 SDF 碰撞代价。若把轮子放入其中，轮子在支撑期也会被地形排斥，导致机器人走路歪扭。因此新增独立的：

```info
selfCollisions {
  collisionSpheres { ... }
  collisionOffsets { ... }
  pairs { ... }
}
```

该部分只用于机器人自身几何之间的避碰，不参与地形 SDF；轮子仍可正常接触地面。

### 3.2 当前几何配置

`frame_declaration.info` 中配置了 8 个自碰撞球体：

| 类别 | 连杆 | 半径 |
|---|---|---:|
| 小腿/膝连接区域 | `Lfl3_knee`、`Lfr3_knee`、`Lrl3_knee`、`Lrr3_knee` | 0.22 m |
| 轮子 | `Lfl4_wheel`、`Lfr4_wheel`、`Lrl4_wheel`、`Lrr4_wheel` | 0.095 m |

轮子球体覆盖半径约 0.09 m、宽度约 0.046 m 的轮胎；小腿球体是对完整小腿网格的保守包络。

当前有 6 个同侧碰撞 pair，最小期望表面间隙均为 0.05 m：

```text
左侧：Lfl4_wheel - Lrl3_knee
左侧：Lrl4_wheel - Lfl3_knee
左侧：Lfl4_wheel - Lrl4_wheel
右侧：Lfr4_wheel - Lrr3_knee
右侧：Lrr4_wheel - Lfr3_knee
右侧：Lfr4_wheel - Lrr4_wheel
```

前两类 pair 同时覆盖“前轮靠近后小腿”和“后轮靠近前小腿”两种方向，避免只保护其中一种折叠姿态。

### 3.3 代码数据流

1. `FrameDeclaration` 增加 `selfCollisions` 与 `selfCollisionPairs` 声明。
2. `QuadrupedPinocchioMapping` 将连杆名映射为 Pinocchio body frame，并解析 pair 的球体索引。
3. `QuadrupedKinematics` 提供自碰撞球体世界坐标和 pair。
4. `SwitchedModelPrecomputation` 同时导出普通地形碰撞球和自碰撞球的 CppAD 线性化及其状态导数。
5. `SelfCollisionAvoidanceCost` 对每个 pair 使用阈值松弛障碍函数：

```text
h = ||p1 - p2|| - r1 - r2 - minimumDistance
```

当 `h < selfCollisionActivationDistance` 时才施加代价。

6. `QuadrupedPointfootInterface` 将该代价以 `SelfCollisionAvoidanceCost` 名称加入 `stateCostPtr`。

新增/主要修改文件：

```text
ocs2_anymal_models/include/ocs2_anymal_models/FrameDeclaration.h
ocs2_anymal_models/src/FrameDeclaration.cpp
ocs2_anymal_models/include/ocs2_anymal_models/QuadrupedPinocchioMapping.h
ocs2_anymal_models/src/QuadrupedPinocchioMapping.cpp
ocs2_anymal_models/include/ocs2_anymal_models/QuadrupedKinematics.h
ocs2_anymal_models/src/QuadrupedKinematics.cpp

ocs2_switched_model_interface/include/.../core/KinematicsModelBase.h
ocs2_switched_model_interface/src/core/KinematicsModelBase.cpp
ocs2_switched_model_interface/include/.../core/SwitchedModel.h
ocs2_switched_model_interface/include/.../core/SwitchedModelPrecomputation.h
ocs2_switched_model_interface/src/core/SwitchedModelPrecomputation.cpp
ocs2_switched_model_interface/include/.../core/ModelSettings.h
ocs2_switched_model_interface/src/core/ModelSettings.cpp
ocs2_switched_model_interface/include/.../cost/SelfCollisionAvoidanceCost.h
ocs2_switched_model_interface/src/cost/SelfCollisionAvoidanceCost.cpp
ocs2_switched_model_interface/test/cost/testSelfCollisionAvoidanceCost.cpp

ocs2_quadruped_interface/src/QuadrupedPointfootInterface.cpp
```

`...` 代表相应包的 include 根目录。

## 4. 已验证事项

### 4.1 单元测试

已执行：

```bash
build/ocs2_switched_model_interface/test_ocs2_switched_model_interface_cost \
  --gtest_filter='SelfCollisionAvoidanceCostTest.*'
```

结果：两个测试均通过，覆盖代价激活范围与梯度推动球体分离的方向。

### 4.2 编译与安装

已构建并安装以下包/目标：

```text
ocs2_switched_model_interface
ocs2_anymal_models
ocs2_quadruped_interface
ocs2_quadruped_loopshaping_interface
ocs2_anymal_mpc
ocs2_anymal_loopshaping_mpc_perceptive_demo
```

`task.info` 和 `frame_declaration.info` 在 install 空间中为软链接，单纯修改配置后无需重新编译 C++；但修改接口头文件或模型设置结构后必须重编译依赖包。

### 4.3 运行验证

已用 ToGo URDF、楼梯下行参数直接运行感知节点：

```bash
install/ocs2_anymal_loopshaping_mpc/lib/ocs2_anymal_loopshaping_mpc/\
ocs2_anymal_loopshaping_mpc_perceptive_demo
```

短距离和 3 m 下楼梯验证均输出：

```text
Completed: 100
```

自碰撞加强后的 3 m 下楼梯临时验证数据导出在：

```text
/tmp/20260713_105819
```

对该轨迹，左前轮与左后小腿的全轨迹网格最小间隙为 202.8 mm；新增反向 pair 在其最接近的球体时刻也保有较大网格余量。

## 5. 已修复的启动崩溃

现象：

```text
process has died, exit code -6
terminate called after throwing std::bad_alloc
```

原因：新增 `ModelSettings` 字段后，只编译了部分下游目标；静态库 `ocs2_anymal_mpc` 仍按旧的 `QuadrupedInterface::Settings` 内存布局传递数据。`defaultGait_` 随后被错误解释，构造 `GaitSchedule` 时触发异常。

修复方法：重新编译并安装 `ocs2_anymal_mpc`，然后重新链接/安装最终的感知 MPC 可执行文件。

若再次修改以下结构体的字段或布局，必须做同样的依赖重编译：

```text
switched_model::ModelSettings
switched_model::QuadrupedInterface::Settings
```

建议命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

cmake --build build/ocs2_switched_model_interface -- -j2
cmake --install build/ocs2_switched_model_interface
cmake --build build/ocs2_anymal_models -- -j2
cmake --install build/ocs2_anymal_models
cmake --build build/ocs2_quadruped_interface -- -j2
cmake --install build/ocs2_quadruped_interface
cmake --build build/ocs2_anymal_mpc --target ocs2_anymal_mpc -- -j2
cmake --install build/ocs2_anymal_mpc
cmake --build build/ocs2_anymal_loopshaping_mpc \
  --target ocs2_anymal_loopshaping_mpc_perceptive_demo -- -j2
cmake --install build/ocs2_anymal_loopshaping_mpc
```

## 6. 1.4 / 1.5 地形穿模分析

分析使用的数据：

```text
date/20260713_110709  # 1.4 随机箱体
date/20260713_110814  # 1.5 随机粗糙地面
```

### 6.1 数据结论

| 场景 | 四个轮接触点相对原始高度图的最小间隙 |
|---|---:|
| 1.4 随机箱体 | +0.3 mm |
| 1.5 随机粗糙地面 | -49.4 mm |

1.5 中，右后轮在 `t=0.97 s` 处的接触点约为：

```text
(x, y, z) = (0.195, -0.278, 0.001) m
原始地形高度约为 0.050 m
```

即启动阶段已经进入地面约 49 mm。后续还出现过前右轮约 -21.8 mm、后左轮约 -11.8 mm 的接触点负间隙。因此这不是单纯的 RViz 显示问题。

### 6.2 根因

1. **初始支撑区不平。** 当时导出轨迹的机身 x 为 0.50 m，后轮接触点已经进入粗糙场；随机箱体的前 2 m 是完整平地，因此不发生该问题。
2. **规划地形与原始高度图有误差。** 感知流水线生成的平面化/滤波高度图，在 1.5 中相对原图的绝对误差 95% 分位为约 20 mm，最大约 40 mm。1.4 的绝大多数区域误差为 0。感知节点由 `SegmentedPlanesTerrainModel` 构建 SDF，而不是直接使用原始高度图。
3. **轮子没有作为地形 SDF 碰撞体。** 普通 `collisions` 只有小腿球体；轮子被故意只放入 `selfCollisions`，否则支撑期会被地面排斥、步态失稳。
4. **支撑期的足端 SDF 碰撞关闭。** `SwitchedModelPrecomputation` 中，足端球仅在摆动期激活；支撑期由局部平面接触约束处理。`contactRadius=0.09 m` 只能将接触平面沿法向偏移，不能让完整轮胎与连续粗糙高度场做精确体积碰撞。
5. **SDF 避碰是软代价。** 即使对摆动腿/小腿，`CollisionAvoidanceCost` 也不是硬几何约束，因此存在与运动跟踪、动力学代价折中的残余穿入。

### 6.3 当前配置的额外注意事项

当前 `initialRobotState.x = 0.00 m`，而 1.5 地图的 world-x 范围为约 `[0, 8] m`。后轮相对机身向后延伸，初始接触点可能落到地图范围外。因此不要仅依靠把机身 x 改为 0 来修复 1.5；应先重新设计地图的平坦起步区和世界坐标对齐关系。

## 7. 建议的后续工作（未实施）

按优先级：

1. **修正 1.5 地图起步区。** 生成粗糙地面时保留至少 1.0 m 长、横向至少覆盖 `y=[-0.5, 0.5] m` 的完全平坦前后平台；随后将机器人初始 x 放在该平坦区中心，并确保四个轮接触点均在地图内。
2. **构建保守地形。** 用原始高度图的上包络（例如高度 max-filter 或 3–5 cm 膨胀）生成 SDF/接触高度，避免平面化误差将小峰值抹掉。
3. **改进摆动轮地形避碰。** 对摆动轮使用包含轮胎半径的 SDF 间隙；不要把同一个轮球无条件加入普通 `collisions`，因为它会在支撑期与地面接触冲突。应新增“仅摆动期启用的轮地形碰撞体”或实现真正的轮-地形接触几何。
4. **重新采集并量化。** 修复后重新导出 1.5 数据，检查四个 `*_wheel_contact` 相对原始地形高度的最小间隙必须非负，并同时检查轮胎网格与地形的最小距离。

## 8. 常用启动命令

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 平地
ros2 launch ocs2_anymal_loopshaping_mpc \
  togo_prototype_flat_mpc_demo.launch.py forward_velocity:=0.5

# 随机箱体（1.4）
ros2 launch ocs2_anymal_loopshaping_mpc \
  togo_prototype_random_boxes_mpc_demo.launch.py forward_velocity:=0.5

# 随机粗糙地面（1.5；当前仍有待修复的穿模问题）
ros2 launch ocs2_anymal_loopshaping_mpc \
  togo_prototype_rough_terrain_mpc_demo.launch.py forward_velocity:=0.5

# 下楼梯
ros2 launch ocs2_anymal_loopshaping_mpc \
  togo_prototype_stairs_down_mpc_demo.launch.py forward_velocity:=0.5
```

## 9. 工作区状态与提交注意事项

当前工作区存在多项未提交改动，除自碰撞实现外还包括地形 PNG、感知参数 YAML、多个 launch 文件、文档及已有文件的修改/删除。提交前应人工拆分提交；不要使用 `git reset --hard`、`git checkout -- .` 等会覆盖其他工作内容的命令。

与本窗口直接相关且应一起提交的内容至少包括：

```text
自碰撞模型、预计算与代价源码
SelfCollisionAvoidanceCost 单元测试及 CMake 注册
togo_prototype/task.info 自碰撞参数
togo_prototype/frame_declaration.info 的 selfCollisions 配置
本交接书
```
