# ToGo Prototype 自碰撞硬约束改造任务书

## 1. 任务目标

将当前仅以 `SelfCollisionAvoidanceCost` 实现的同侧轮子/小腿自碰撞保护，改造为能被当前 OCS2 多重射击 SQP 后端实际传入 HPIPM QP 子问题的硬不等式约束。

对每个配置的自碰撞 pair，在每个优化节点满足：

```text
h(x) = ||p1(x) - p2(x)|| - r1 - r2 - minimumDistance >= 0
```

其中：

- `p1(x)`、`p2(x)`：两个碰撞球的世界坐标；
- `r1`、`r2`：球半径；
- `minimumDistance`：配置的最小表面间隙，当前为 0.05 m。

目标是在可行初值和 SQP 收敛的条件下，禁止上述球体进入负间隙；不是只提高自碰撞代价权重。

## 2. 当前基础与约束

### 2.1 已有能力

当前实现已经具备：

- `frame_declaration.info` 中的 `selfCollisions` 球体及 6 个 pair；
- `QuadrupedKinematics` 输出自碰撞球位置；
- `SwitchedModelPreComputation` 输出球位置及相对状态导数；
- `SelfCollisionAvoidanceCost` 对同一个 `h(x)` 计算代价及一阶/二阶近似；
- 配置开关 `enableSelfCollisionAvoidance`。

### 2.2 不能采用的错误方案

不得只把新约束放入：

```cpp
problemPtr_->stateInequalityConstraintPtr
```

当前多重射击 SQP 路径不会自动把通用的纯状态不等式加入 HPIPM QP。因此该做法在接口层看似是硬约束，实际不会约束 SQP 解。

### 2.3 算法边界

当前算法为 `SQP`，配置文件为：

```text
config/togo_prototype/multiple_shooting.info
```

实现应保持现有 SQP + HPIPM 求解器，不切换为其他优化器。改造内容是向 SQP 的 QP 子问题提供线性化后的自碰撞不等式。

## 3. 功能需求

### 3.1 约束定义

新增 `SelfCollisionConstraint`（命名可随项目约定调整），复用现有 pair、球位置和状态导数。

对每一个 pair：

1. 计算当前间隙 `h(x_bar)`；
2. 计算梯度：

   ```text
   dh/dx = ((p1 - p2) / ||p1 - p2||)^T * (dp1/dx - dp2/dx)
   ```

3. 在 SQP 工作点 `x_bar` 处提供仿射化约束：

   ```text
   h(x_bar) + dh/dx * (x - x_bar) >= 0
   ```

4. 球心距离接近零时必须有数值保护，禁止除零或生成 NaN。

约束数量等于当前有效 pair 数量；当前 ToGo 配置为 6。

### 3.2 SQP 接入

必须确认约束进入 SQP 的 QP 约束矩阵，而不是仅存入 `OptimalControlProblem` 的未使用容器。

实现前先定位现有能进入 HPIPM 的状态相关不等式通道；可选择：

- 将纯状态自碰撞约束扩展到 `ocs2_sqp` 的约束线性化与 QP 映射；或
- 使用已被 SQP 后端支持的约束类型，并保证数学含义仍为每个射击节点的 `h(x) >= 0`。

不得将自碰撞伪装成仅影响代价的 slack/hinge 项。

### 3.3 可配置模式

保留现有软代价开关，并新增明确的模式配置，建议：

```info
selfCollisionConstraintMode  hard  ; off | soft | hard
```

行为约定：

| 模式 | 行为 |
|---|---|
| `off` | 不加入自碰撞代价或约束。 |
| `soft` | 保持现有 `SelfCollisionAvoidanceCost`。 |
| `hard` | 加入 QP 硬不等式；默认不叠加软代价。 |

可选支持 `hard_with_soft` 用作数值缓冲，但必须明确记录其含义。旧的 `enableSelfCollisionAvoidance` 应保持兼容，或在迁移中给出确定的优先级和弃用说明。

### 3.4 不可行处理

硬约束会导致问题不可行，必须定义处理策略：

- 若当前实测状态已 `h < 0`，线性化硬约束可能无可行解；
- 初始版本至少要检测并记录每个违反 pair 的 `h`；
- 明确 QP 不可行时 MPC 的行为（保留上一控制、降级为软约束、停止机器人或上报失败）；
- 不得静默忽略 QP 不可行状态。

推荐第一版策略：启动/运行已穿入时记录错误并采用受控软化或安全停车；不要假称严格硬约束仍可从深度穿入状态立即恢复。

## 4. 实施步骤

1. 阅读并梳理 `ocs2_sqp` 中约束线性化、QP 数据结构和 HPIPM 映射代码，确认可扩展点。
2. 复用 `SwitchedModelPreComputation` 的自碰撞球位置及导数，新增独立约束类和解析测试。
3. 将约束接入 SQP 子问题；添加调试统计，输出每个节点的最小 `h`、QP 不可行状态和活跃约束数。
4. 增加 `selfCollisionConstraintMode` 配置解析及接口中的注册逻辑。
5. 重新构建所有受 `ModelSettings` 或 `QuadrupedInterface::Settings` 布局影响的库：

   ```text
   ocs2_switched_model_interface
   ocs2_anymal_models
   ocs2_quadruped_interface
   ocs2_quadruped_loopshaping_interface
   ocs2_anymal_mpc
   ocs2_anymal_loopshaping_mpc_perceptive_demo
   ```

6. 执行单元、QP 接入和完整场景回归测试。

## 5. 测试与验收

### 5.1 单元测试

- 间隙公式、梯度方向和数值有限性；
- 对随机状态做有限差分，验证 `dh/dx`；
- 约束在 `h > 0` 时满足、在 `h < 0` 时违反；
- 球心重合或接近时不产生 NaN/Inf；
- `off`、`soft`、`hard` 三种模式的注册行为正确。

### 5.2 SQP/QP 接入测试

构造一个最小优化问题，使初始预测轨迹违反一个 pair：

- 验证 QP 确实收到对应不等式行；
- 验证求解后线性化间隙非负；
- 验证将该约束移除后，问题可回到原有的重叠解；
- 验证 QP 不可行时有可识别的求解状态和日志。

### 5.3 ToGo 场景回归

至少运行：

```bash
ros2 launch ocs2_anymal_loopshaping_mpc \
  togo_prototype_rough_terrain_mpc_demo.launch.py forward_velocity:=0.5
```

验收指标：

- `hard` 模式下，导出轨迹中全部 6 个 pair 的最小 `h` 不小于数值容差（建议 `-1 mm`）；
- 同时报告球体实际表面间隙与网格最小距离；
- 不出现 QP infeasible、NaN、段错误或无限线搜索；
- 记录每周期耗时、SQP 实际迭代数和最大计算时间；
- 与 `soft` 模式比较速度跟踪、跌倒/失败次数和计算时间。

注意：该验收只保证球模型的间隙。若球体包络不足以覆盖真实轮胎/小腿网格，仍须单独改进几何近似。

## 6. 风险与缓解

| 风险 | 缓解措施 |
|---|---|
| 当前状态已深度穿入，QP 无可行解 | 检测初始违反；实施明确的安全降级策略。 |
| SQP 线性化误差使真实非线性间隙仍为负 | 增加 SQP 迭代次数，必要时引入信赖域/收紧安全边界。 |
| 硬约束挤压接触可行域 | 先在平地、楼梯、粗糙地形逐级测试，逐对启用。 |
| 实时性下降 | 统计 QP 维度和每周期时间；必要时减少 pair 或降低预测节点数。 |
| 结构体变更导致 ABI 崩溃 | 完整重建第 4 节列出的依赖包，不能只重建最终可执行文件。 |

## 7. 交付物

- 自碰撞硬约束实现及 SQP/HPIPM 接入代码；
- 参数模式及 ToGo 配置；
- 单元测试和最小 QP 接入测试；
- 粗糙地形导出数据与间隙分析报告；
- 变更说明：约束模式、不可行处理、实时性与已知限制。
