# ToGo Prototype OCS2 MPC 项目总结书

日期：2026-07-09

## 一、项目背景

本项目基于现有 `ocs2_perceptive_anymal` 感知式 MPC 框架，将 ToGo Prototype 四足轮腿机器人接入 `ocs2_anymal_loopshaping_mpc` 控制与可视化流程。原框架主要面向 ANYmal C 点足机器人；ToGo Prototype 的 URDF、关节命名、轮式末端几何和接触语义均与原模型不同，因此需要完成模型适配、配置映射、初始姿态调整和落脚点语义修正。

项目目标不是重写一套 ToGo 专用 MPC，而是在保留 OCS2/ANYmal 现有管线的前提下，使 ToGo Prototype 能通过 ROS 2 launch 启动 MPC、RViz、dummy MRT、gait command 和 target command 等节点，并在感知地形 demo 中生成合理的轮心/落脚参考。

## 二、当前运行入口

普通 MPC 启动入口：

```bash
source install/setup.bash
ros2 launch ocs2_anymal_loopshaping_mpc togo_prototype.launch.py
```

感知地形 demo 启动入口：

```bash
source install/setup.bash
ros2 launch ocs2_anymal_loopshaping_mpc togo_prototype_perceptive_mpc_demo.launch.py
```

主要机器人模型：

```text
src/ToGo.Prototype/urdf/ToGo.Prototype_abs.urdf
```

主要配置目录：

```text
src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/config/togo_prototype/
```

`install/ocs2_anymal_loopshaping_mpc/.../config/togo_prototype` 当前通过 symlink 指回源目录，因此源目录配置修改会被 launch 直接读取。

## 三、模型与自由度

当前 ToGo Prototype 在 OCS2 中按 12 个可动关节处理：

- 每条腿 3 个主动关节。
- 四条腿共 12 个 actuated/joint DoF。
- 若把浮动基座计入整机广义自由度，则为 6 个 base DoF + 12 个关节 DoF = 18 DoF。

URDF 中以下额外关节已经固定，以匹配现有 quadruped MPC 的 4 腿 x 3 关节假设：

```text
J1_waist
Jfl4_wheel
Jfr4_wheel
Jrl4_wheel
Jrr4_wheel
```

## 四、已完成工作

1. 新增/适配 ToGo 启动入口

`togo_prototype.launch.py` 作为薄封装，复用 `mpc.launch.py`，并传入 ToGo 的 URDF、`robot_name=togo_prototype`、`config_name=togo_prototype` 等参数。

`togo_prototype_perceptive_mpc_demo.launch.py` 用于感知式地形 demo，直接加载 ToGo URDF 并运行 `ocs2_anymal_loopshaping_mpc_perceptive_demo`。

2. 建立 ToGo 专用配置

`config/togo_prototype` 中维护：

```text
frame_declaration.info
task.info
loopshaping.info
multiple_shooting.info
```

其中 `loopshaping.info`、`multiple_shooting.info` 继续沿用原框架设置；`frame_declaration.info` 和 `task.info` 已替换为 ToGo 的 link、joint、关节限位和初始状态。

3. 修正初始高度，避免启动穿地

原始 `initialRobotState` 的 base 高度为 `0.25`，按 URDF 初始 FK 计算，轮子最低点在默认地面 `z=0` 下方约 `0.0896 m`，启动时会出现明显穿模。

现在 base 初始高度调整为：

```text
(5,0) 0.34 ; z
```

按当前初始关节角计算：

```text
wheel center z ~= 0.090425
wheel center z - 0.09 ~= 0.000425
```

即轮心位于地面上方约一个轮半径，估算接触点基本贴合地面。

4. 将轮端由“点足接触”改为“轮心 + 半径偏置”

此前 `frame_declaration.info` 将每条腿的 `tip` 指向 `L*4_wheel_contact`。该方式把轮子最低点当成点足端点，虽然可让接触点贴地，但运动中轮子几何与点足模型不一致，容易出现可视化/几何穿模。

现在 ToGo 的 `tip` 改为轮心 link：

```text
Lfl4_wheel
Lfr4_wheel
Lrl4_wheel
Lrr4_wheel
```

并为每条腿增加：

```text
contactRadius 0.09
```

新增的 `contactRadius` 是可选配置。未配置该字段的原 ANYmal/C 系列机器人默认半径为 `0.0`，保持原点足行为。

5. 扩展 frame declaration 解析

`FrameDeclaration` 新增 `contactRadius` 字段，并提供 `getContactRadii()`，在构建 Anymal/ToGo interface 时写入 `SwingTrajectoryPlannerSettings`。

6. 扩展 swing/foothold planner

`SwingTrajectoryPlanner` 在选择地形 plane 后，将 plane 原点沿地形法向上移 `contactRadius`。因此对 ToGo 来说：

- planner、IK 和 normal constraint 跟踪的是轮心位置。
- 轮子实际接地点可近似为轮心沿地形法向下移一个半径。
- 点足机器人因半径默认为 0，行为不变。

## 五、关键文件清单

ToGo 启动：

```text
src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/launch/togo_prototype.launch.py
src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/launch/togo_prototype_perceptive_mpc_demo.launch.py
```

ToGo 配置：

```text
src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/config/togo_prototype/frame_declaration.info
src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/config/togo_prototype/task.info
```

轮心半径解析与传播：

```text
src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_models/include/ocs2_anymal_models/FrameDeclaration.h
src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_models/src/FrameDeclaration.cpp
src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_mpc/src/AnymalInterface.cpp
```

落脚地形偏置：

```text
src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_switched_model_interface/include/ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h
src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_switched_model_interface/src/foot_planner/SwingTrajectoryPlanner.cpp
```

## 六、验证结果

已完成的构建验证：

```bash
source install/setup.bash
colcon build --packages-up-to ocs2_anymal_loopshaping_mpc --symlink-install
```

结果：

```text
Summary: 27 packages finished
```

构建过程中仅出现外部 Pinocchio/coal include 提示，不是本次改动导致的编译错误。

已完成的单元测试验证：

```bash
build/ocs2_anymal_models/TestQuadrupedPinocchio --gtest_filter=TestFrameDeclaration.*:TestFrameMapping.*
```

结果：

```text
2 tests passed
```

已完成的几何检查：

```text
Lfl4_wheel: center_z=0.090425, contact_z_est=0.000425
Lfr4_wheel: center_z=0.090425, contact_z_est=0.000425
Lrl4_wheel: center_z=0.090425, contact_z_est=0.000425
Lrr4_wheel: center_z=0.090425, contact_z_est=0.000425
```

## 七、当前效果

当前版本解决了两个核心问题：

1. 启动姿态下轮子不再明显低于默认地面。
2. ToGo 轮端不再被 OCS2 完全当作轮底点足，而是以轮心作为规划/IK 跟踪点，并通过 `0.09 m` 半径对地形接触高度进行补偿。

这使得模型语义更接近轮式末端：控制器规划轮心轨迹，几何上由轮半径决定地面接触位置。

## 八、已知限制

1. 仍复用 ANYmal 命名空间和管线

当前 package、executable、topic 和部分变量名仍沿用 ANYmal/point-foot 体系。ToGo 只是通过 URDF 与配置适配进入现有框架，并非完整独立的 ToGo MPC 包。

2. 动力学仍是简化接触模型

当前 `contactRadius` 主要影响 swing planner、foothold terrain 和 IK/normal constraint 的参考位置。底层 centroidal/contact force 模型仍是原 OCS2 quadruped 的简化模型。对于切向摩擦力、轮地滚动约束、轮半径导致的精确力矩臂等效应，当前实现仍是近似。

3. 轮子没有显式滚动自由度

URDF 中轮关节已固定，以满足 12 DoF quadruped MPC 假设。因此当前 ToGo 被视为四足轮形足端机器人，而不是完整轮腿动力学模型。

4. `targetCommand.info` 仍复用 c_series

当前普通 launch 的 target command 仍来自：

```text
ocs2_anymal_mpc/config/c_series/targetCommand.info
```

后续如需更符合 ToGo 的默认姿态、速度或高度，应新增 ToGo 专用 target command。

## 九、后续建议

1. 实机或仿真运行 `togo_prototype.launch.py`，观察初始姿态、轮心高度和 MPC 收敛情况。
2. 运行 `togo_prototype_perceptive_mpc_demo.launch.py`，重点检查 swing 过程中的轮子与台阶/坡面/障碍物是否仍有穿模。
3. 若运动中仍出现擦碰，可适当增大 `terrainMargin`、`swingHeight` 或 SDF clearance。
4. 若需要更真实的轮腿行为，应进一步引入轮地接触点动力学、轮半径力矩臂、滚动约束和轮关节自由度，而不是只做轮心高度补偿。
5. 建议后续将 ToGo 相关配置、target command、README 与 launch 文档集中整理，减少 ANYmal/C 系列遗留描述对使用者的干扰。

## 十、结论

本阶段已完成 ToGo Prototype 接入 OCS2 loopshaping MPC 的主要适配工作。当前系统能够使用 ToGo URDF 与 ToGo 专用配置启动，并完成 12 DoF 四足模型、初始高度、轮心半径接触语义和构建验证。项目已经从“点足近似直接套用”推进到“轮心规划 + 半径补偿”的更合理近似，为后续仿真调参、地形穿越验证和更完整的轮腿动力学建模打下了基础。
