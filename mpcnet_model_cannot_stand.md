# 训练的模型站不稳

## 1. 问题现象

在启动 OCS2 Legged Robot 的 RaiSim 示例时，机器人本体和关节模型看起来基本正常，
但 RViz 中右前腿（RF）和左后腿（LH）的足端位置球体与真实足端存在明显偏移。

```bash
source /opt/ros/jazzy/setup.bash
source /home/mm/opendog_quadruped/install/setup.bash

export LD_LIBRARY_PATH=/home/mm/raisim2Lib/raisim/lib:${LD_LIBRARY_PATH}

ros2 launch ocs2_legged_robot_raisim \
  legged_robot_ddp_raisim.launch.py \
  taskFile:=/home/mm/opendog_quadruped/install/ocs2_legged_robot/share/ocs2_legged_robot/config/mpc/task.info
```

使用当时的配置训练 MPC-Net 后，得到的神经网络策略无法稳定站立。即使训练损失能够下降，
部署时仍可能出现腿部姿态异常、身体快速失稳或倒地。

足端球体偏移并不是单纯的 RViz 显示问题，而是腿部顺序不一致的外在表现。

## 2. 根本原因

系统中存在两种合法但用途不同的腿顺序：

| 数据类型 | 正确顺序 |
|---|---|
| OCS2/Pinocchio 关节位置、关节速度 | `LF, LH, RF, RH` |
| 接触力、接触模式、摆动相位 | `LF, RF, LH, RH` |
| RaiSim 关节顺序 | `LF, RF, LH, RH` |

原来的部分配置和可视化代码把 OCS2 状态向量错误地标记或解释成了
`LF, RF, LH, RH`，导致中间两个三维关节块 RF 和 LH 对调。

具体表现为：

```text
OCS2 实际关节状态： LF | LH | RF | RH
部分旧配置的解释： LF | RF | LH | RH
                         ^^^^^^^
                         RF/LH 对调
```

这会同时影响：

- `/joint_states` 中关节名称与关节数值的对应关系；
- 初始状态和默认目标状态中的腿部姿态；
- MPC-Net 的默认状态、采样中心和动作语义；
- Pinocchio 计算的足端位置与 RViz 机器人模型之间的一致性；
- 训练数据中 observation、MPC action 和机器人真实关节之间的对应关系。

因此，只修改足端球体的显示位置是不够的。必须统一状态、配置、可视化和
RaiSim 转换边界处的语义。

## 3. 修复原则

### 3.1 OCS2 内部统一关节顺序

所有 OCS2/Pinocchio 关节状态和关节速度统一采用：

```text
LF_HAA LF_HFE LF_KFE
LH_HAA LH_HFE LH_KFE
RF_HAA RF_HFE RF_KFE
RH_HAA RH_HFE RH_KFE
```

即 `LF, LH, RF, RH`。

### 3.2 不修改接触量顺序

接触力、接触模式和摆动相位仍然保持：

```text
LF, RF, LH, RH
```

这些量本来就是按 `contactNames3DoF` 的顺序组织，不能因为修复关节顺序而一起交换。

### 3.3 在 RaiSim 边界显式转换

RaiSim 使用 `LF, RF, LH, RH`，因此进入或离开 OCS2 时显式交换 RF/LH：

```text
RaiSim:          LF | RF | LH | RH
                            ↓ 转换
OCS2/Pinocchio:  LF | LH | RF | RH
```

## 4. 修改位置

| 文件 | 修改内容 |
|---|---|
| `ocs2_legged_robot/common/ModelSettings.h` | 关节名称统一为 `LF, LH, RF, RH` |
| `LeggedRobotVisualizer.cpp` | `/joint_states` 名称与 OCS2 状态向量保持一致 |
| `ocs2_legged_robot/config/mpc/task.info` | 修正初始状态、Q/R 中的关节语义 |
| `ocs2_legged_robot/config/command/reference.info` | 修正默认关节目标状态 |
| `legged_robot.yaml` | 修正 MPC-Net 默认状态、目标状态、缩放和采样语义 |
| `legged_robot_raisim.yaml` | 同步修正 RaiSim 版 MPC-Net 配置 |
| `RaiSimConversions.cpp` | 明确记录 RaiSim 与 OCS2 的转换方向 |
| `testLeggedRobotRaisimConversions.cpp` | 增加精确置换测试，防止错误转换互相抵消后仍通过往返测试 |

## 5. 验证结果

### 5.1 编译和单元测试

相关软件包重新编译成功：

```bash
colcon build --symlink-install --packages-select \
  ocs2_legged_robot \
  ocs2_legged_robot_ros \
  ocs2_legged_robot_raisim \
  ocs2_legged_robot_mpcnet \
  --cmake-args -DBUILD_TESTING=ON
```

测试结果：

```text
ocs2_legged_robot：        7 项通过
ocs2_legged_robot_raisim： 1 项通过
总计：                     8/8 通过
```

### 5.2 运行时足端位置

启动 RaiSim 后，将 `/legged_robot/currentState` 中的足端 Marker 与
`odom -> *_FOOT` TF 进行对比：

| 足端 | Marker 与 TF 的位置误差 |
|---|---:|
| LF | 0.0104 mm |
| RF | 0.0687 mm |
| LH | 0.0812 mm |
| RH | 0.0292 mm |

最大误差小于 `0.1 mm`，原来 RF/LH 的厘米级偏移消失。

运行时 `/joint_states` 的名称顺序也已确认是：

```text
LF, LH, RF, RH
```

## 6. 为什么旧网络站不稳

旧 MPC-Net 配置中的 RF/LH 默认关节块与 OCS2 实际状态语义不一致。训练任务会围绕
错误的 `DEFAULT_STATE` 和 `DEFAULT_TARGET_STATE` 生成初始状态与目标状态，导致专家
MPC 数据和神经网络看到的腿部语义不统一。

这种错误不能依靠增加训练次数解决。网络可能在错误的数据对应关系上收敛，但输出到
真实机器人状态后仍然会把右前腿和左后腿的控制含义混淆，从而无法维持站立。

修复配置后，旧模型不应继续作为最终策略使用，需要从修复后的配置重新生成数据并训练。

## 7. 重新训练

使用 RaiSim 配置重新训练：

```bash
source /opt/ros/jazzy/setup.bash
source /home/mm/opendog_quadruped/install/setup.bash

export LD_LIBRARY_PATH=/home/mm/raisim2Lib/raisim/lib:${LD_LIBRARY_PATH}

cd "/home/mm/opendog_quadruped/src/ocs2_ros2/advance examples/ocs2_mpcnet/ocs2_legged_robot_mpcnet/ocs2_legged_robot_mpcnet"

/home/mm/venvs/mpcnet/bin/python -u train.py legged_robot_raisim.yaml
```

必须显式传入 `legged_robot_raisim.yaml`。如果不传参数，`train.py` 默认加载
`legged_robot.yaml`，不会使用 RaiSim rollout。

本次修复后训练得到的运行目录：

```text
runs/2026-07-16_16-58-30_legged_robot_description/
```

其中 `best_policy.json` 记录的最佳策略为：

| 指标 | 值 |
|---|---:|
| iteration | 170333 |
| evaluation duration | 4.0 s |
| survival time | 4.0000016 s |
| final XY error | 0.02552 m |
| final yaw error | 0.02845 rad |
| incurred Hamiltonian | 38.4228 |

当前训练任务包括 stance、trot_1 和 trot_2，采样权重为 `1:2:2`。这表示策略不仅学习
站立，也学习两种相反起始相位的对角小跑。

## 8. 后续排查清单

如果以后再次出现“训练正常但模型站不稳”，按以下顺序检查：

1. 检查 `/joint_states.name` 是否为 `LF, LH, RF, RH`。
2. 对比四个 `*_FOOT` TF 与 `/legged_robot/currentState` Marker。
3. 确认 OCS2 关节状态/速度使用 `LF, LH, RF, RH`。
4. 确认接触力、模式和摆动相位仍使用 `LF, RF, LH, RH`。
5. 检查 RaiSim 边界是否执行 RF/LH 置换，避免重复交换或漏交换。
6. 检查训练命令是否显式使用 `legged_robot_raisim.yaml`。
7. 检查是否错误加载了修复前的 `init_policy.pt` 或旧 `best_policy`。
8. 分别固定 stance、trot_1、trot_2 做独立评估，避免单次随机评估掩盖问题。

## 9. 结论

训练模型站不稳的根因不是网络规模、学习率或训练轮数，而是 RF/LH 在不同模块中的
关节顺序语义不一致。足端球体偏移提供了直接的可视化证据。统一 OCS2 状态顺序、保留
接触量顺序，并在 RaiSim 边界显式转换后，足端 Marker 与 TF 对齐，重新训练的策略也能
完成站立任务。
