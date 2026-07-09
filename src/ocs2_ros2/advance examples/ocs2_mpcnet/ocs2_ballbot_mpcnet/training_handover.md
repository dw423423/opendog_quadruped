# Ballbot MPC-Net 训练效果差改进交接书

日期：2026-07-08

## 1. 任务背景

当前目标是改进 `ocs2_ballbot_mpcnet` 的 Ballbot MPC-Net 策略模型训练效果。用户反馈训练得到的 `final_policy.onnx` 在 `ballbot_mpcnet.launch.py` 中运行效果很差，需要确认原因并给出后续改进路径。

相关入口：

- 训练脚本：`src/ocs2_ros2/advance examples/ocs2_mpcnet/ocs2_ballbot_mpcnet/ocs2_ballbot_mpcnet/train.py`
- Ballbot MPC-Net 任务采样：`src/ocs2_ros2/advance examples/ocs2_mpcnet/ocs2_ballbot_mpcnet/ocs2_ballbot_mpcnet/mpcnet.py`
- 核心训练循环：`src/ocs2_ros2/advance examples/ocs2_mpcnet/ocs2_mpcnet_core/ocs2_mpcnet_core/mpcnet.py`
- MPC 配置：`src/ocs2_ros2/basic examples/ocs2_ballbot/config/mpc/task.info`
- 运行时实际 MPC 配置：`install/ocs2_ballbot/share/ocs2_ballbot/config/mpc/task.info`
- 当前问题 run：`src/ocs2_ros2/advance examples/ocs2_mpcnet/ocs2_ballbot_mpcnet/ocs2_ballbot_mpcnet/runs/2026-07-08_12-01-38_ballbot_description`

## 2. 当前结论

这次训练不是简单的“训练不够久”，而是训练中后期发生了明显退化。`final_policy.onnx` 不是该 run 中效果最好的模型。

已从 TensorBoard event 中读到：

- 每轮数据量大多为 `900`，符合配置预期。
- 总数据量约 `25200`。
- `survival_time` 初始约 `0.8s`。
- `survival_time` 在 step `1105` 和 `1435` 达到过 `3.0s`。
- 最终 step `9540` 附近 `survival_time` 退化到约 `1.34s`。
- `objective/empirical_loss` 在 step `1000` 附近约 `3.49`，最终 step `9999` 变成约 `24.05`。
- `metric/incurred_hamiltonian` 全程为 `NaN`，说明 policy evaluation 过程中有 rollout 异常或 Hamiltonian 统计失败。

权重检查结果：

- 初始 `LinearPolicy.linear.weight` 范围约 `[-0.285, 0.290]`。
- 最终 `LinearPolicy.linear.weight` 范围约 `[-19.517, 16.547]`。
- final 与 initial 的 weight 差异范数约 `30.88`。

这说明当前训练后期把线性控制律推得过大，策略变得激进且不稳定。

## 3. 已完成的排查与改动

### 3.1 模型结构确认

当前 `train.py` 使用：

```python
policy = LinearPolicy(config)
```

即策略网络本质是单层线性映射：

```text
observation_dim = 10
action_dim      = 3
policy          = Linear(10, 3)
```

对 Ballbot 这种非线性系统来说，单层线性策略表达能力可能不足。

### 3.2 训练数据数量检查

配置中：

```text
DATA_GENERATION_TASKS     = 10
DATA_GENERATION_DURATION  = 3.0
DATA_GENERATION_TIME_STEP = 0.1
DATA_GENERATION_SAMPLES   = 2
```

理论每轮数据量约：

```text
10 * 30 * (1 + 2) = 900
```

实际日志中每轮 `data/new_data_points` 大多为 `900`，数据生成数量正确。

### 3.3 数据质量可视化工具

新增了数据质量报告脚本：

```text
src/ocs2_ros2/advance examples/ocs2_mpcnet/ocs2_ballbot_mpcnet/ocs2_ballbot_mpcnet/data_quality.py
```

运行方式：

```bash
cd /home/mm/opendog_quadruped
source install/setup.bash
/home/mm/venvs/mpcnet/bin/python "src/ocs2_ros2/advance examples/ocs2_mpcnet/ocs2_ballbot_mpcnet/ocs2_ballbot_mpcnet/data_quality.py" --output-dir /home/mm/opendog_quadruped/ballbot_data_quality
```

输出：

```text
/home/mm/opendog_quadruped/ballbot_data_quality/report.html
/home/mm/opendog_quadruped/ballbot_data_quality/summary.json
/home/mm/opendog_quadruped/ballbot_data_quality/raw_data.npz
```

一次完整检查得到：

```text
received 900 data points
PASS: Basic finite-value checks passed
```

含义：基础数据没有 `NaN/Inf`，但还需要继续看 `u` 分布、`||dHdu||`、`dHduu_min_eig` 和 observation 覆盖范围。

### 3.4 Launch 默认模型

之前已将 `ballbot_mpcnet.launch.py` 默认模型切到：

```text
.../runs/2026-07-08_12-01-38_ballbot_description/final_policy.onnx
```

但根据训练曲线，建议不要继续默认使用 final，而应先测试中间模型：

```text
.../runs/2026-07-08_12-01-38_ballbot_description/intermediate_policy_1000.onnx
```

## 4. 主要问题判断

优先级从高到低：

1. **学习率过高且无梯度裁剪**

   当前配置：

   ```yaml
   LEARNING_RATE: 1.e-2
   GRADIENT_CLIPPING: False
   BATCH_SIZE: 32
   ```

   结合权重爆炸现象，`1e-2` 对当前 Hamiltonian loss + 线性策略过激。

2. **没有保存 best policy**

   step `1000` 附近明显比 final 好，但训练脚本最终只把 `final_policy.onnx` 当主要产物。后期退化后，好的中间策略没有被自动选出来。

3. **模型容量不足**

   单层 `LinearPolicy(10 -> 3)` 可能只能拟合局部近似，不能很好覆盖不同目标位置、姿态和速度状态。

4. **数据覆盖偏少**

   当前有效数据总量约 `25200`，任务数和随机扰动范围偏小。若目标分布和实际测试目标不一致，模型会泛化差。

5. **评估指标异常**

   `metric/incurred_hamiltonian` 全是 `NaN`，说明 policy evaluation 过程中存在异常。虽然数据生成 basic finite 检查通过，但评估链路仍需继续排查异常来源。

## 5. 推荐的第一轮改进

### 5.1 先使用更好的中间模型验证

运行：

```bash
cd /home/mm/opendog_quadruped
source install/setup.bash
ros2 launch ocs2_ballbot_mpcnet ballbot_mpcnet.launch.py policy_file_path:="/home/mm/opendog_quadruped/src/ocs2_ros2/advance examples/ocs2_mpcnet/ocs2_ballbot_mpcnet/ocs2_ballbot_mpcnet/runs/2026-07-08_12-01-38_ballbot_description/intermediate_policy_1000.onnx"
```

如果 `intermediate_policy_1000.onnx` 明显好于 `final_policy.onnx`，可以确认核心问题是训练后期退化，而不是数据完全错误。

### 5.2 稳定训练超参数

建议先把 `ballbot.yaml` 中训练参数改为：

```yaml
BATCH_SIZE: 128
LEARNING_RATE: 1.e-3
GRADIENT_CLIPPING: True
GRADIENT_CLIPPING_VALUE: 1.0
```

如果仍然震荡，再试：

```yaml
LEARNING_RATE: 3.e-4
```

### 5.3 增大数据量

建议第一轮改成：

```yaml
DATA_GENERATION_TASKS: 50
DATA_GENERATION_SAMPLES: 4
POLICY_EVALUATION_TASKS: 20
```

注意：这会显著增加训练时间，但能改善数据覆盖。

### 5.4 自动保存 best policy

在训练循环中根据 `survival_time` 保存：

```text
best_policy.onnx
best_policy.pt
```

选择标准：

```text
如果当前 mean survival_time > best_survival_time，则保存 best。
```

这样后续 launch 默认使用 `best_policy.onnx`，而不是 `final_policy.onnx`。

### 5.5 先做 behavioral cloning sanity check

Hamiltonian loss 对 Hessian 和局部二次近似比较敏感。建议增加一个实验：先用 `BehavioralCloningLoss` 拟合 MPC 生成的 `u`。

判断标准：

- 如果 BC 都拟合不了，优先检查 observation、action scaling、数据标签。
- 如果 BC 能拟合，但 Hamiltonian loss 不稳定，优先调整 Hamiltonian loss 的学习率、归一化和 Hessian 条件数。

## 6. 第二轮改进方向

### 6.1 换成非线性策略

当前仓库已有 `NonlinearPolicy`，可以先从它开始：

```python
from ocs2_mpcnet_core.policy import NonlinearPolicy
policy = NonlinearPolicy(config)
```

不过当前 `NonlinearPolicy` hidden dimension 只有 `(10 + 3) / 2 = 6`，仍然偏小。更建议新建一个 MLP，例如：

```text
10 -> 64 -> 64 -> 3
```

激活函数用 `Tanh` 或 `ELU`。

### 6.2 增加 normalization / scaling

当前：

```yaml
OBSERVATION_SCALING: 全 1
ACTION_SCALING: 全 1
```

建议基于 `data_quality.py` 生成的 `raw_data.npz` 统计 observation 和 action 的标准差，然后设置合适的 scaling，避免某些维度主导训练。

### 6.3 训练分布和测试目标对齐

当前随机目标：

```text
x/y position: [-1, 1]
yaw: [-45 deg, 45 deg]
```

初始状态主要随机速度，不随机位置和姿态。若实际测试目标更远或初始姿态扰动更大，需要扩大训练分布。

## 7. 推荐执行顺序

1. 用 `intermediate_policy_1000.onnx` 运行 launch，确认中间模型是否好于 final。
2. 运行 `data_quality.py`，保存并查看 `report.html`。
3. 调低学习率、开启梯度裁剪、增大 batch size。
4. 增加 best policy 保存逻辑。
5. 重新训练一次，比较 `best_policy` 与 `final_policy`。
6. 若仍然差，切换到非线性 MLP 策略。
7. 若 evaluation 仍出现 `incurred_hamiltonian = NaN`，打开终端日志，定位 `MpcNetPolicyEvaluation::run` 的异常原因。

## 8. 常用命令

运行 Ballbot MPC-Net：

```bash
cd /home/mm/opendog_quadruped
source install/setup.bash
ros2 launch ocs2_ballbot_mpcnet ballbot_mpcnet.launch.py
```

指定中间模型：

```bash
ros2 launch ocs2_ballbot_mpcnet ballbot_mpcnet.launch.py policy_file_path:="/home/mm/opendog_quadruped/src/ocs2_ros2/advance examples/ocs2_mpcnet/ocs2_ballbot_mpcnet/ocs2_ballbot_mpcnet/runs/2026-07-08_12-01-38_ballbot_description/intermediate_policy_1000.onnx"
```

生成数据质量报告：

```bash
cd /home/mm/opendog_quadruped
source install/setup.bash
/home/mm/venvs/mpcnet/bin/python "src/ocs2_ros2/advance examples/ocs2_mpcnet/ocs2_ballbot_mpcnet/ocs2_ballbot_mpcnet/data_quality.py" --output-dir /home/mm/opendog_quadruped/ballbot_data_quality
```

打开报告：

```text
file:///home/mm/opendog_quadruped/ballbot_data_quality/report.html
```

重新训练：

```bash
cd /home/mm/opendog_quadruped
source install/setup.bash
cd "src/ocs2_ros2/advance examples/ocs2_mpcnet/ocs2_ballbot_mpcnet/ocs2_ballbot_mpcnet"
/home/mm/venvs/mpcnet/bin/python train.py
```

## 9. 交接备注

- 当前最值得优先验证的是 `intermediate_policy_1000.onnx`，因为该 run 的 survival metric 在 1000 左右达到最佳。
- 不建议继续直接使用 `final_policy.onnx` 作为质量判断依据。
- 数据基础 finite 检查已通过，但 evaluation 中 `incurred_hamiltonian` 为 `NaN` 仍需跟进。
- 若要改配置，优先改源码目录下的 `ballbot.yaml` 或 `task.info`，再 rebuild/install；临时测试可直接改 `install/` 下对应文件。
- 当前工作区已有多处未提交改动，继续开发前建议先用 `git status --short` 确认哪些是本任务需要保留的改动。
