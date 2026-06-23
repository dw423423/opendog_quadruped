# Go2 OCS2 stance 足端力为 0 导致站立晃动问题总结

## 1. 现象

启动命令：

```bash
ros2 launch ocs2_quadruped_controller gazebo.launch.py
```

键盘按两次 `2` 进入 OCS2 stance 后，Gazebo 中 Go2 先轻微晃动，然后逐渐失去平衡。失衡过程中四只脚会在地面上滑动，像是在尝试维持平衡，但最终仍然倒下。

OCS2 启动日志显示：

```text
[OCS2] startup[2] obs_mode=0 planned_mode=15 contacts=[0,0,0,0]
base_rpy=[0.004, -0.005, -0.000] base_z=0.358
foot_forces=[0, 0, 0, 0]
```

关键矛盾是：

- Gazebo 画面中四只脚确实接触地面。
- OCS2 控制器读到的 `foot_forces` 全是 0。
- 状态估计器给出的 `obs_mode=0`，即认为四只脚都没有接触。
- MPC 计划是 `planned_mode=15`，即四脚支撑 stance。

因此这是 **仿真物理接触存在，但 ros2_control 足端力状态接口没有有效数据** 的问题。

## 2. 定位过程

第一步，用临时方式验证是否是接触判断问题：

```yaml
feet_force_threshold: -1.0
```

修改后机器人不再倒下。因为判断逻辑是：

```cpp
contact_flag_[i] = foot_force[i] > feet_force_threshold_;
```

当 `foot_force=0` 且阈值为 `-1.0` 时，四只脚都会被强制判定为接触。这个实验说明 OCS2 stance 本身可以稳定，根因在足端接触观测。

第二步，检查 Ignition topic：

```bash
ign topic -l | grep foot_force
```

没有任何输出，说明 Gazebo ForceTorque sensor 没有通过 Ignition transport 发布足端力 topic。

第三步，检查 xacro/URDF/SDF：

```bash
ros2 run xacro xacro src/quadruped_ros2_control/descriptions/unitree/go2_description/xacro/robot.xacro GAZEBO:=true
ign sdf -p /tmp/go2_gazebo.urdf
```

结果显示 SDF 中确实存在：

```xml
<joint name='FL_foot_fixed' type='revolute'>
  ...
  <sensor name='FL_foot_force' type='force_torque'>
```

也就是说问题不是 xacro 没展开，而是运行时 ForceTorque transport topic 没有形成，导致原硬件插件依赖 topic 的路径拿不到力。

## 3. 根因

原先 `gz_quadruped_hardware` 的足端力读取依赖 Gazebo `ForceTorque` sensor topic：

1. 在 Gazebo ECM 中查找 `ForceTorque` sensor。
2. 读取 sensor topic 名。
3. 通过 Ignition transport 订阅 wrench 消息。
4. 将 wrench 模长写入 ros2_control 的 `foot_force/*` state interface。

但当前环境中 `ign topic -l` 没有足端力 topic，所以第 3 步没有数据进入，最终 `foot_force` 始终为 0。

这会导致：

```text
foot_forces=[0,0,0,0]
contacts=[0,0,0,0]
obs_mode=0
```

OCS2/WBC 在 stance 下仍按四脚支撑求解，但状态估计认为四脚腾空，控制闭环不一致，于是出现晃动、脚滑、倒下。

## 4. 最终修复

修复思路：**不再只依赖 Ignition transport 的 `foot_force` topic，而是在 `gz_quadruped_hardware` 内部直接从足端固定关节的 `JointTransmittedWrench` 读取力。**

修改文件：

```text
src/quadruped_ros2_control/hardwares/gz_quadruped_hardware/include/gz_quadruped_hardware/gz_system.hpp
src/quadruped_ros2_control/hardwares/gz_quadruped_hardware/src/gz_system.cpp
```

核心变化：

- 注册 `foot_force` state interface 时，根据接口名推导足端固定关节：

```text
FL_foot_force -> FL_foot_fixed
FR_foot_force -> FR_foot_fixed
RL_foot_force -> RL_foot_fixed
RR_foot_force -> RR_foot_fixed
```

- 为这些 joint 创建或读取 `JointTransmittedWrench` component。
- 在 `read()` 中直接计算 force 模长：

```cpp
ft_sensor->foot_effort = force.norm();
```

这样即使 `ign topic -l | grep foot_force` 仍然没有输出，ros2_control 的 `foot_force/*` state interface 仍然可以有有效数值。

同时保留/调整了 xacro 侧的辅助修正：

```text
src/quadruped_ros2_control/hardwares/gz_quadruped_hardware/xacro/foot_force_sensor.xacro
src/quadruped_ros2_control/descriptions/unitree/go2_description/xacro/gazebo.xacro
```

- `<provide_feedback>` 改为 `<provideFeedback>`。
- Go2 Gazebo xacro 中 ForceTorque 插件文件名改为当前 Ignition Gazebo 6 环境可找到的插件名。

## 5. 构建

修改后需要重新构建硬件插件：

```bash
colcon build --packages-select gz_quadruped_hardware --symlink-install --allow-overriding gz_quadruped_hardware
```

如果同时改了 Go2 描述包，也可以构建：

```bash
colcon build --packages-up-to go2_description gz_quadruped_hardware --symlink-install --allow-overriding gz_quadruped_hardware
```

启动前务必重新 source：

```bash
source install/setup.bash
```

并确保旧的 `ign gazebo` 进程已经关闭。

## 6. 验证标准

`gazebo.yaml` 中阈值应恢复正常值，例如：

```yaml
feet_force_threshold: 4.0
```

启动：

```bash
ros2 launch ocs2_quadruped_controller gazebo.launch.py height:=0.35
```

进入 OCS2 stance 后，日志应接近：

```text
obs_mode=15
planned_mode=15
contacts=[1,1,1,1]
foot_forces=[非零, 非零, 非零, 非零]
```

注意：修复后 `ign topic -l | grep foot_force` 可能仍然没有输出，这不再是关键验证项。最终判断标准是 OCS2 日志中的 `foot_forces` 和 `obs_mode`。

## 7. 经验结论

这类问题不要只看 Gazebo 画面里“脚是否贴地”。对 OCS2/WBC 来说，更重要的是控制器内部是否真的读到了接触状态：

```text
画面接触 != foot_force state interface 有值
```

如果以后再出现 stance 晃动、脚滑、倒下，优先看：

```text
foot_forces
contacts
obs_mode
planned_mode
base_z
base_rpy
```

其中 `planned_mode=15` 但 `obs_mode=0` 是非常明确的接触观测链路故障信号。
