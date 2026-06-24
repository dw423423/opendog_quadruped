# OCS2 固定世界坐标落脚区域技术方案

## 1. 背景与目标

当前目标是在 Go2 的 perceptive OCS2 控制链路中，为四条腿分别指定固定的世界坐标矩形落脚区域，并让 MPC 在每条腿 touchdown 时只约束该腿落入自己的区域。第 5 阶段后，四条腿的固定区域已从硬编码常量扩展为 ROS 参数配置。

本方案遵守以下原则：

- 保留现有 perceptive OCS2 foot placement constraint 链路，不绕过 `FootPlacementConstraint`。
- FL / FR / RL / RR 使用独立的固定世界坐标矩形区域。
- 区域选择由 `ConvexRegionSelector` 根据 leg id 完成。
- 每条腿 swing / touchdown 时只检查自己的区域。
- 当前阶段只面向 `static_walk` 验证。
- 暂不接视觉，暂不做楼梯高度变化，暂不切 trot。
- 已验收成功的 FL 单腿区域保持不变。

## 2. 总体链路

固定区域落脚约束仍然沿用原有 perceptive OCS2 数据流：

```text
gait schedule / mode schedule
        |
        v
PerceptiveLeggedReferenceManager::modifyReferences()
        |
        v
ConvexRegionSelector::update()
  - 按 leg id 选择固定世界坐标矩形
  - 生成该腿对应的 convex polygon
  - 写入 feetProjections_ / convexPolygons_
        |
        v
PerceptiveLeggedPrecomputation::request()
  - 从 ConvexRegionSelector 取该腿 polygon
  - 转成 FootPlacementConstraint::Parameter(a, b)
        |
        v
FootPlacementConstraint::getValue() / getLinearApproximation()
  - 按 contactPointIndex_ 取该腿足端位置
  - 计算 polygon 半空间约束残差
        |
        v
OCS2 soft state constraint
```

关键点是：固定区域只替换 `ConvexRegionSelector` 中的 region selection 结果，不改变 `FootPlacementConstraint` 的约束接口，也不直接修改 MPC 解或 swing trajectory。

## 3. 固定区域定义

固定区域的数据结构和默认值定义在：

```text
src/quadruped_ros2_control/controllers/ocs2_quadruped_controller/include/ocs2_quadruped_controller/perceptive/interface/FixedFootholdRegions.h
```

默认区域如下：

| leg id | name | x range | y range | z |
| --- | --- | --- | --- | --- |
| 0 | FL | `[0.25, 0.35]` | `[0.10, 0.18]` | `0.0` |
| 1 | FR | `[0.25, 0.35]` | `[-0.18, -0.10]` | `0.0` |
| 2 | RL | `[-0.05, 0.05]` | `[0.10, 0.18]` | `0.0` |
| 3 | RR | `[-0.05, 0.05]` | `[-0.18, -0.10]` | `0.0` |

其中 FL 保留第三阶段已验收成功的区域：

```text
x[0.25,0.35], y[0.10,0.18]
```

该头文件同时提供默认配置和通用工具：

- `defaultFixedFootholdRegionSettings()`
- `getFixedFootholdRegion(size_t leg)`
- `isInsideFixedFootholdRegionXY(size_t leg, const vector3_t& position)`
- `fixedFootholdRegionToString(size_t leg)`

这样 selector、touchdown 日志、RViz 标记使用同一份区域定义，避免坐标复制后漂移。

运行时配置位于：

```text
src/quadruped_ros2_control/descriptions/unitree/go2_description/config/gazebo.yaml
```

配置块：

```yaml
fixed_foothold_regions:
  enable: true
  frame: "world"
  FL:
    x_min: 0.25
    x_max: 0.35
    y_min: 0.10
    y_max: 0.18
    z: 0.0
  FR:
    x_min: 0.25
    x_max: 0.35
    y_min: -0.18
    y_max: -0.10
    z: 0.0
  RL:
    x_min: -0.05
    x_max: 0.05
    y_min: 0.10
    y_max: 0.18
    z: 0.0
  RR:
    x_min: -0.05
    x_max: 0.05
    y_min: -0.18
    y_max: -0.10
    z: 0.0
```

`CtrlComponent` 启动时读取 `fixed_foothold_regions.*` 参数，并打印最终生效的四腿区域。

## 4. ConvexRegionSelector 设计

实现文件：

```text
src/quadruped_ros2_control/controllers/ocs2_quadruped_controller/src/perceptive/interface/ConvexRegionSelector.cpp
```

### 4.1 按腿选择区域

`ConvexRegionSelector::update()` 遍历 `leg = 0..3`，对每条腿独立处理。当 `fixed_foothold_regions.enable=true` 时，在已有 planar terrain 基础上，将该腿的投影点固定到对应配置矩形中心：

```cpp
const auto& fixedRegion = getFixedFootholdRegion(leg);
const vector3_t targetCenter(
    0.5 * (fixedRegion.xMin + fixedRegion.xMax),
    0.5 * (fixedRegion.yMin + fixedRegion.yMax),
    fixedRegion.z);
```

然后调用：

```cpp
getBestPlanarRegionAtPositionInWorld(targetCenter, planarTerrain_.planarRegions, penaltyFunction)
```

把 projection 放到该固定区域中心，再用矩形顶点生成 `convexRegion`。

当 `fixed_foothold_regions.enable=false` 时，不执行固定区域覆盖，保留原始 perceptive region selection 逻辑中 grow 出来的 `convexRegion`。

### 4.2 矩形转 convex polygon

内部函数 `makeFixedTargetRegion()` 根据区域边界生成 CGAL polygon。`numVertices_` 至少为 4；当 `numVertices_` 大于 4 时，会沿矩形边界均匀采样更多点，保持与原 foot placement polygon constraint 维度兼容。

### 4.3 stance-only 保护

如果当前 schedule 是纯 `STANCE`，某条腿没有 swing phase，此时固定落脚区域不应激活。否则 stance 会出现两类问题：

- `findIndex()` 在单 phase schedule 下可能得到非法 final index。
- 四条腿站立时被固定区域硬约束，可能破坏原地站立。

因此现在增加了 no-swing 保护：

```text
[FootholdRegion] leg=X has no swing phase; fixed foothold constraint disabled for this schedule
```

对应行为：

- `initStandFinalTime_[leg] = infinity`
- 该腿不写入固定 foothold constraint region
- `FootPlacementConstraint` 在 stance-only schedule 下保持 inactive

这保证切到 `stance` 时不再因为固定落脚区域约束崩溃或强行拉脚。

## 5. PerceptiveLeggedPrecomputation

实现文件：

```text
src/quadruped_ros2_control/controllers/ocs2_quadruped_controller/src/perceptive/interface/PerceptiveLeggedPrecomputation.cpp
```

该模块负责把 selector 生成的 polygon 转成约束参数：

```cpp
std::tie(polytopeA, polytopeB) = getPolygonConstraint(convexRegionSelectorPtr_->getConvexPolygon(i, t));
params.a = polytopeA * p * projection.regionPtr->transformPlaneToWorld.inverse().linear();
params.b = polytopeB + polytopeA * projection.regionPtr->transformPlaneToWorld.inverse().translation().head(2);
```

其中：

- `i` 是 leg id。
- `params.a` 和 `params.b` 存入 `footPlacementConParameters_[i]`。
- `FootPlacementConstraint` 后续按相同 leg id 读取参数。

日志会打印当前是否使用固定区域：

```text
[PerceptivePrecomputation] leg=0 fixed_enabled=1 selected_region=FL:x[0.25,0.35],y[0.1,0.18],z=0 time=...
param.a=...
param.b=...
```

当 `enable=false` 时，日志为：

```text
[PerceptivePrecomputation] leg=0 fixed_enabled=0 selected_region=perceptive ...
```

该日志用于确认每条腿拿到自己的区域，而不是全部拿 FL 区域。

## 6. FootPlacementConstraint

实现文件：

```text
src/quadruped_ros2_control/controllers/ocs2_quadruped_controller/src/perceptive/constraint/FootPlacementConstraint.cpp
```

### 6.1 保留原约束链路

每条腿仍然通过 `PerceptiveLeggedInterface::setupOptimalControlProblem()` 注册一个 `FootPlacementConstraint`：

```cpp
new FootPlacementConstraint(*reference_manager_ptr_, *eeKinematicsPtr, i, numVertices_)
```

随后包装成 `StateSoftConstraint` 加入 OCS2 问题：

```cpp
problem_ptr_->stateSoftConstraintPtr->add(
    footName + "_footPlacement",
    std::make_unique<StateSoftConstraint>(...));
```

### 6.2 按腿读取足端位置

`contactPointIndex_` 是该 constraint 对应的 leg id。约束值计算：

```cpp
param.a * footPosition + param.b
```

当前实现处理两种 kinematics 输入：

- 单足 kinematics：`getPosition()` 返回长度 1，使用 `front()`。
- 四足 kinematics：`getPosition()` 返回长度 4，使用 `at(contactPointIndex_)`。

这样避免 FR / RL / RR 使用单足 kinematics 时越界。

### 6.3 PinocchioInterface 绑定

`PinocchioEndEffectorKinematics::clone()` 后不会保留 `pinocchioInterfacePtr_`。为避免运行时报错：

```text
[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()
```

`FootPlacementConstraint` 在 `getValue()` 和 `getLinearApproximation()` 中会从 `PerceptiveLeggedPrecomputation` 取当前 PinocchioInterface，并重新绑定给 cloned kinematics。

## 7. Touchdown 日志

实现文件：

```text
src/quadruped_ros2_control/controllers/ocs2_quadruped_controller/src/FSM/StateOCS2.cpp
```

### 7.1 predicted touchdown

运行中每隔约 0.25s 遍历四条腿，在 MPC policy 的 mode schedule 中查找该腿下一次 swing -> stance 事件：

```text
[PREDICTED_TOUCHDOWN] leg=0 name=FL selected_region=FL:x[0.25,0.35],y[0.1,0.18],z=0 swing_start=... touchdown=... mode=... predicted_foot=(x, y, z) inside_xy=true
```

日志字段：

- `leg`：腿编号。
- `name`：FL / FR / RL / RR。
- `selected_region`：该腿固定区域。
- `swing_start`：MPC 预测 swing 开始时间。
- `touchdown`：MPC 预测 touchdown 时间。
- `predicted_foot`：MPC policy 在 touchdown 时刻的足端世界坐标。
- `inside_xy`：`predicted_foot` 是否在该腿固定区域的 XY 范围内。

### 7.2 actual touchdown

根据当前 observation mode 的 contact flag 检测真实 swing -> stance 边沿：

```text
[ACTUAL_TOUCHDOWN] leg=0 name=FL selected_region=FL:x[0.25,0.35],y[0.1,0.18],z=0 time=... actual_foot=(x, y, z) inside_xy=true
```

日志字段：

- `time`：当前 observation time。
- `actual_foot`：Gazebo / estimator 当前状态对应的足端世界坐标。
- `inside_xy`：真实 touchdown 是否落入对应固定区域 XY。

### 7.3 Pinocchio frame 更新

`PinocchioEndEffectorKinematics::getPosition()` 读取的是 Pinocchio data 中已更新的 frame placement，而不是直接用传入 state 现场计算。因此 touchdown 日志在读足端位置前会调用 `updatePinocchioFrames()`：

```cpp
pinocchio::forwardKinematics(model, data, q);
pinocchio::updateFramePlacements(model, data);
```

这保证：

- predicted 日志使用 `touchdownState` 对应的 frame placement。
- actual 日志使用当前 `observation_.state` 对应的 frame placement。

## 8. RViz 可视化

实现文件：

```text
src/quadruped_ros2_control/controllers/ocs2_quadruped_controller/src/perceptive/visualize/FootPlacementVisualization.cpp
```

当 `fixed_foothold_regions.enable=true` 时，`/foot_placement` marker 会发布四个固定矩形区域：

- namespace: `Fixed Target Regions`
- marker id: `10000 + leg`
- 颜色沿用原 feet color map

这用于在 RViz 中直观看到 FL / FR / RL / RR 各自区域。

当 `enable=false` 时，不发布固定矩形 marker，只保留原 perceptive convex region 可视化。

## 9. static_walk 与 stance 的行为

### 9.1 static_walk

`static_walk` 在 `gait.info` 中定义为四相慢走：

```text
LF_RF_RH
RF_LH_RH
LF_RF_LH
LF_LH_RH
```

每个 phase 都有一条腿 swing，因此固定区域约束会在各腿进入对应站立段后参与优化。

预期日志：

```text
[FootPlacementConstraint] leg=X time=... active=1
[PREDICTED_TOUCHDOWN] leg=X ... inside_xy=true/false
[ACTUAL_TOUCHDOWN] leg=X ... inside_xy=true/false
```

### 9.2 stance

`stance` 是纯四腿接触：

```text
STANCE
```

没有 swing phase，因此固定落脚约束禁用。预期日志：

```text
[FootholdRegion] leg=X has no swing phase; fixed foothold constraint disabled for this schedule
[FootPlacementConstraint] leg=X time=... active=0
```

这属于正常行为：stance 用于稳定站立，不应触发固定 touchdown 区域约束。

## 10. 验收方法

### 10.1 编译验证

```bash
colcon build --packages-select ocs2_quadruped_controller
```

当前已通过编译。现存 warning 主要是已有未用参数 / 未用变量，不影响本功能。

### 10.2 启动验证

```bash
ros2 launch ocs2_quadruped_controller gazebo.launch.py pkg_description:=go2_description
```

### 10.3 stance 验证

切到 stance 后检查：

- 不再出现：

```text
[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set
```

- 不再 terminate。
- 固定 foothold constraint 对无 swing 腿禁用。

### 10.4 static_walk 验证

切到 static_walk 后检查：

- 每条腿都有自己的 `selected_region`。
- `FootPlacementConstraint` 在对应腿约束时出现 `active=1`。
- predicted touchdown 日志中对应腿的 `inside_xy` 应逐步为 `true`。
- actual touchdown 日志中对应腿的 `actual_foot` 应基本落入自己的区域。
- FR / RL / RR 不应被错误吸到 FL 区域。

推荐 grep：

```bash
grep -E "FootholdRegion|FootPlacementConstraint|PerceptivePrecomputation|PREDICTED_TOUCHDOWN|ACTUAL_TOUCHDOWN" <log_file>
```

### 10.5 配置独立性验证

修改 `gazebo.yaml` 中 FL 区域，例如只改：

```yaml
fixed_foothold_regions:
  FL:
    x_min: 0.30
    x_max: 0.40
```

重新启动后检查启动日志和 selector 日志：

```text
[FixedFootholdRegions] leg=0 name=FL x[0.300,0.400] ...
[FixedFootholdRegions] leg=1 name=FR x[0.250,0.350] ...
[FixedFootholdRegions] leg=2 name=RL x[-0.050,0.050] ...
[FixedFootholdRegions] leg=3 name=RR x[-0.050,0.050] ...
```

验收标准：

- 只有 `leg=0 name=FL selected_region=FL:...` 的区域发生变化。
- `leg=1 name=FR`、`leg=2 name=RL`、`leg=3 name=RR` 的 `selected_region` 不变。
- RViz 中只有 FL 固定矩形位置变化。
- touchdown 日志中只有 FL 使用新的 inside_xy 判断边界。

## 11. 当前限制

- 当前矩形区域已支持 ROS YAML 参数配置，但只实现 world frame。
- 当前矩形区域不随机器人 base yaw / body frame 旋转。
- 当前 z 固定为 `0.0`，没有接入楼梯高度变化。
- 当前未接视觉 terrain region selection，仅使用默认平面地形作为投影支撑。
- 只建议在 `static_walk` 下验证；trot / dynamic gait 暂不作为验收目标。

## 12. 后续建议

1. 将 `FixedFootholdRegions.h` 中的区域改为 YAML 参数，方便现场调参。
2. 根据默认站立足端位置微调 FR / RL / RR 区域，减少 nominal foot 与 region 的初始偏差。
3. 在 static_walk 稳定后，再考虑把区域从 world frame 扩展为 body-relative 或 terrain-relative。
4. 接入视觉 / 楼梯高度时，只替换 `ConvexRegionSelector` 的区域来源，不改 `FootPlacementConstraint` 链路。
5. trot 验证应单独进行，因为相位、双腿 swing 和可行域约束强度都会变化。
