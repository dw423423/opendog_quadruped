下面这个图可以作为你以后定位问题的**总地图**。你用 OCS2 控制自己的四足狗时，问题基本都能归到其中一层。

`quadruped_ros2_control` 里的 `ocs2_quadruped_controller` 本质是一个 `ros2_control` controller，并且 README 说明它基于 `legged_control` 和 `ocs2_ros2`；它要求硬件侧提供关节 `position / velocity / effort / KP / KD` 命令接口，以及关节状态、IMU、足端力传感器等状态接口。([GitHub][1])

---

## 1. OCS2 控制自己机器人总框架图

```text id="mrjemx"
┌─────────────────────────────────────────────────────────────┐
│  第 0 层：启动 / 编译 / 配置层                                │
│  launch.py、robot.xacro、robot_control.yaml、task.info        │
│  reference.info、gait.info、URDF、CppAD 自动生成库             │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  第 1 层：机器人模型层                                        │
│  URDF / Xacro / mesh / inertial / joint axis / foot frame     │
│  base_link、hip、thigh、calf、foot、IMU、contact sensor        │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  第 2 层：仿真或真机硬件接口层                                │
│  Gazebo / MuJoCo / 真机驱动 / ros2_control hardware plugin    │
│  提供 joint state、IMU、足端力；接收 torque、pos、vel、kp、kd  │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  第 3 层：ros2_control 层                                     │
│  controller_manager                                           │
│  加载 ocs2_quadruped_controller                              │
│  管理 command_interface 和 state_interface                    │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  第 4 层：状态估计层 State Estimator                          │
│  joint state + IMU + foot contact / foot force                │
│  估计 base position、base velocity、base orientation          │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  第 5 层：目标与步态层                                        │
│  TargetManager / GaitManager                                  │
│  目标速度、目标姿态、stance、trot、standing_trot 等模式        │
│  输出 planned contact mode                                    │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  第 6 层：OCS2 MPC 层                                         │
│  根据当前状态 + 目标 + 约束 + 代价函数                         │
│  求解未来一段时间的 MPC policy                                │
│  输出 optimized_state、optimized_input、planned_mode          │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  第 7 层：MRT 层                                              │
│  MPC_MRT_Interface                                            │
│  保存、更新、插值 MPC policy                                  │
│  实时控制循环从这里取当前时刻的参考状态和输入                 │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  第 8 层：WBC 层                                              │
│  Whole Body Control                                           │
│  根据 optimized_state / optimized_input / measured_state      │
│  求解关节 torque、期望 position、期望 velocity、kp、kd         │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  第 9 层：低层执行层                                          │
│  ros2_control 写入 command_interface                          │
│  仿真器或电机执行 torque / PD command                         │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│  第 10 层：机器人动力学反馈                                   │
│  机器人运动、足端接触、IMU变化、关节反馈                       │
│  再反馈回状态估计层                                           │
└─────────────────────────────────────────────────────────────┘
```

更简单地记：

```text id="3fsxws"
配置/模型
  ↓
硬件接口
  ↓
状态估计
  ↓
目标/步态
  ↓
MPC
  ↓
MRT
  ↓
WBC
  ↓
关节命令
  ↓
机器人反馈
```

---

## 2. 出问题后按层定位

### 第 0 层：启动 / 编译 / 配置层

这一层的问题通常表现为：

```text id="xcxi95"
colcon build 失败
launch 后直接退出
找不到 package
找不到 config
找不到 task.info / reference.info / gait.info
CppAD shared library 生成失败
```

重点查：

```text id="6a4mk9"
src/ocs2_ros2 是否完整
robot_description 包名是否正确
ocs2_quadruped_controller 是否编译成功
config/ocs2/task.info 是否存在
config/ocs2/reference.info 是否存在
config/ocs2/gait.info 是否存在
modelFolderCppAd 路径是否可写
```

OCS2 controller README 也说明，第一次启动时会编译 OCS2 模型并生成 CppAD shared library，完成后需要重启 controller。([GitHub][1])

---

### 第 1 层：机器人模型层

这一层最容易导致“机器人一启动就翻、腿方向反、模型姿态怪”。

重点查：

```text id="1h67oo"
URDF 质量 mass 是否合理
惯量 inertia 是否合理
关节轴 joint axis 是否正确
关节零位是否正确
左腿和右腿符号是否对称
foot frame 位置是否正确
base_link 坐标系是否符合 OCS2 假设
腿的顺序是否和 OCS2 配置一致
```

典型现象：

```text id="srcisc"
RViz 里模型正常，但仿真一加载就炸
腿往反方向折
膝盖方向不对
机器人刚站起来就前翻/后翻
足端位置计算明显不对
```

优先判断：

```text id="32zstk"
如果 passive 模式下模型就乱，优先怀疑 URDF / joint axis / inertial。
如果 passive 正常，一切换 MPC 才乱，再往状态估计、MPC、WBC 查。
```

---

### 第 2 层：仿真 / 真机硬件接口层

这是你把自己的机器人接入 OCS2 时最关键的一层。

OCS2 controller 需要的 command interface 是：

```text id="j3msqy"
joint position
joint velocity
joint effort
KP
KD
```

state interface 是：

```text id="jq9r9a"
joint effort
joint position
joint velocity
IMU linear acceleration
IMU angular velocity
IMU orientation
feet force sensor
```

这些接口是仓库 README 里明确列出的。([GitHub][1])

这一层出问题时，常见现象是：

```text id="c6dyr6"
controller_manager 加载 controller 失败
claim interface 失败
找不到某个 joint interface
找不到 imu interface
找不到 feet force sensor
关节状态一直是 0
足端力一直是 0
IMU 方向明显不对
```

重点查：

```text id="5u97my"
robot.xacro 里的 ros2_control tag
hardware plugin 名字
joint name 是否和 controller yaml 完全一致
每个关节有没有 position/velocity/effort state
每个关节有没有 position/velocity/effort/kp/kd command
IMU 名字是否和 controller 配置一致
foot force sensor 名字是否和 controller 配置一致
```

一句话：
**OCS2 能不能控制你的机器人，首先看这一层接口是否完整。**

---

### 第 3 层：ros2_control 层

这一层负责把 controller 和 hardware interface 接起来。ROS2 Control 官方文档也说明，使用 ROS 2 Humble 时需要安装 `ros-humble-ros2-control` 和 `ros-humble-ros2-controllers` 这类基础包。([Control ROS][2])

典型问题：

```text id="d9wv8v"
spawner 超时
controller_manager 不存在
load_controller 失败
configure controller 失败
activate controller 失败
```

常用检查命令：

```bash id="g93odz"
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 control list_hardware_components
ros2 topic list
ros2 node list
```

判断方式：

```text id="6evbu1"
如果 controller 都没 active，问题还没到 OCS2。
如果 controller active 了，但机器人不动，再看 command 是否写入、硬件是否执行。
```

---

### 第 4 层：状态估计层

状态估计层把关节、IMU、足端接触信息融合成机器人当前状态。

它输出给 MPC/WBC 的东西包括：

```text id="ye1y9y"
base position
base orientation
base linear velocity
base angular velocity
joint position
joint velocity
contact state
```

这一层出问题时，典型现象是：

```text id="vqk8ug"
机器人明明站着，但估计的 base 高度不对
机器人没动，但估计速度很大
姿态 roll/pitch/yaw 跳变
一接触地面，状态突然发散
RViz 里 base 飘走
```

重点查：

```text id="m90zcd"
IMU 坐标系方向
base_link 和 imu_link 的 TF
重力方向
足端接触判断
足端力阈值
关节编码器正负号
ground_truth estimator 是否可用
```

判断方式：

```text id="qcrifs"
如果 MPC 还没开始，状态估计已经跳，优先查 IMU / TF / joint sign。
如果 ground_truth estimator 正常，普通 estimator 不正常，说明估计器输入或接触判断有问题。
```

---

### 第 5 层：目标与步态层

这一层决定机器人“应该干什么”。

例如：

```text id="3z5z3x"
passive
stance
trot
standing_trot
flying_trot
```

OCS2 controller README 里也说明键盘 `1` 是 Passive Mode，键盘 `2` 进入 OCS2 MPC Mode，后续 `2/3/4/5` 分别对应 stance、trot、standing_trot、flying_trot。([GitHub][1])

这一层出问题时，典型现象是：

```text id="aiw60d"
按键后模式没切换
planned_mode 不符合预期
本来想站立，却进入 trot
本来四脚支撑，却有脚被规划成 swing
planned_mode 和 measured_mode 长期不一致
```

重点查：

```text id="27m8mf"
gait.info
modeSequenceTemplate
stance / trot 的接触模式
每条腿的顺序
LF RF LH RH 是否和代码假设一致
键盘命令是否真的发出来
GaitManager 是否 ready
TargetManager 是否收到目标速度
```

这一层尤其要注意：
**腿顺序错了，会导致 MPC 以为抬的是左前腿，实际控制的是右后腿。**

---

### 第 6 层：OCS2 MPC 层

MPC 层负责算未来一段时间的最优轨迹，也就是 MPC policy。

它关心：

```text id="q5pxxj"
机器人动力学模型
质量和惯量
足端位置
接触约束
摩擦锥约束
关节限制
输入限制
代价函数权重
目标速度
目标机体高度
步态模式
```

典型问题：

```text id="9jrbia"
MPC policy expired
MPC 求解太慢
MPC 没有 policy
MPC time horizon 太短
solution time window shorter than MPC delay
SQP 不收敛
优化结果很奇怪
```

重点查：

```text id="pzydsu"
task.info 里的权重
reference.info 里的 comHeight
gait.info 里的周期和接触时长
MPC horizon
MPC frequency
模型质量和惯量
足端 frame 名字
摩擦系数
关节限制
初始站姿是否接近 reference
```

判断方式：

```text id="jidsn8"
如果站立都不稳，不要先调 trot。
如果 stance 稳，但 trot 不稳，重点查 gait.info 和接触模式。
如果一进入 MPC 就发散，重点查模型参数、初始姿态、状态估计、WBC。
```

---

### 第 7 层：MRT 层

MRT 负责把 MPC policy 给实时控制循环使用。

典型问题：

```text id="shs4dh"
MPC policy expired
policy_final_time 小于当前 obs_time
MPC delay 太大
控制线程拿不到新的 policy
第一次启动时 policy 没准备好
```

你以前看到的：

```text id="m3z9ab"
[MPC_MRT_Interface::advanceMpc] WARNING
MPC policy expired
solution time window might be shorter than the MPC delay
```

基本就是这一层附近的问题。

重点查：

```text id="8wmf5r"
MPC 是否算得太慢
CPU 是否满载
CppAD 是否第一次正在编译
控制频率是否太高
MPC horizon 是否太短
仿真时间和 ROS 时间是否同步
是否有控制循环 overrun
```

判断方式：

```text id="kmbx23"
如果 policy expired 偶尔出现，可能是计算抖动。
如果持续出现，说明 MPC 供不上实时控制。
如果进入 failsafe，说明实时控制已经拿不到有效 policy。
```

---

### 第 8 层：WBC 层

WBC 把 MPC 输出变成关节命令。

输入大概是：

```text id="4g93wn"
当前测量状态 measured_state
MPC optimized_state
MPC optimized_input
planned_mode
contact flag
机器人动力学模型
足端雅可比
约束
```

输出是：

```text id="9u9tap"
joint torque
joint position desired
joint velocity desired
kp
kd
```

典型问题：

```text id="xt260j"
WBC QP infeasible
输出 torque 很大
某条腿力矩方向反
站立时疯狂抖动
摆腿打地
支撑腿发软
机器人被自己推翻
```

重点查：

```text id="y0eqcr"
足端 Jacobian 是否正确
腿顺序是否正确
contact flag 是否正确
足端力方向是否正确
关节 torque limit
关节方向正负号
机体质量
base frame 坐标系
关节 kp/kd
WBC 权重
```

判断方式：

```text id="x6oryx"
如果 MPC 输出看起来合理，但 WBC 输出 torque 巨大，问题在 WBC 或模型雅可比。
如果 WBC 输出合理，但机器人执行后乱动，问题在硬件接口或关节方向。
```

---

### 第 9 层：低层执行层

这一层把命令真正写给电机或仿真器。

典型问题：

```text id="yxvt7f"
仿真里关节不动
关节突然打满
实际机器人电机啸叫
PD 太硬导致抖动
PD 太软导致站不起来
torque saturation
```

重点查：

```text id="rv3v4f"
kp/kd 是否过大
torque limit 是否过小
关节 command 单位是否是 rad
力矩单位是否是 Nm
电机方向是否和 URDF 一致
仿真 timestep 是否过大
控制频率是否稳定
```

判断方式：

```text id="5yxvsg"
如果关节目标正确但实际关节反向运动，查电机方向/关节 axis。
如果目标正常但执行滞后严重，查 kp/kd、力矩限制、控制频率。
```

---

## 3. 最实用的排错顺序

以后你看到问题，不要从 MPC 开始乱调。按这个顺序：

```text id="98jn5x"
1. passive 模式下模型是否稳定？
   否：查 URDF / inertial / joint axis / 仿真参数

2. controller 是否 active？
   否：查 ros2_control / hardware interface / yaml

3. joint state、IMU、foot force 是否正常？
   否：查 hardware interface / sensor name / 坐标系

4. 状态估计是否合理？
   否：查 IMU、TF、foot contact、ground truth estimator

5. stance 是否稳定？
   否：查 reference.info、comHeight、初始站姿、WBC、质量惯量

6. planned_mode 和 measured_mode 是否一致？
   否：查 gait.info、腿顺序、足端力阈值、contact flag

7. MPC policy 是否过期？
   是：查 MPC 速度、horizon、CPU、控制频率、CppAD

8. WBC torque 是否异常？
   是：查 Jacobian、接触约束、力矩限制、WBC 权重

9. 仿真能跑但真机不行？
   查电机方向、力矩限制、延迟、通信频率、实物参数误差
```

---

## 4. 你应该重点记住这张“定位表”

| 现象                              | 优先怀疑层                                     |
| ------------------------------- | ----------------------------------------- |
| 编译失败                            | 第 0 层：依赖 / CMake / package                |
| launch 后找不到配置                   | 第 0 层：launch / config 路径                  |
| controller 加载失败                 | 第 2～3 层：hardware interface / ros2_control |
| RViz 里腿方向怪                      | 第 1 层：URDF / joint axis                   |
| passive 模式就炸                    | 第 1～2 层：模型 / 仿真                           |
| MPC 一开就摔                        | 第 4～8 层：状态估计 / MPC / WBC                  |
| `planned_mode != measured_mode` | 第 5 层 + 第 4 层：步态 / 接触检测                   |
| `MPC policy expired`            | 第 6～7 层：MPC / MRT                         |
| `WBC QP infeasible`             | 第 8 层：WBC 约束                              |
| torque 巨大                       | 第 8～9 层：WBC / 关节方向 / 力矩限制                 |
| 站立稳定，小跑摔                        | 第 5～6 层：gait.info / MPC 参数                |
| 仿真稳定，真机不稳                       | 第 2 / 9 / 10 层：硬件延迟、力矩限制、模型误差             |

---

## 5. 对你自己的 OpenDog，推荐最小验证框架

不要一开始就跑 trot。你自己的机器人接 OCS2 时，建议这样逐级验证：

```text id="mu1nu9"
阶段 1：只加载模型
目标：RViz / Gazebo / MuJoCo 里模型姿态正确

阶段 2：只跑 ros2_control
目标：controller_manager 能 active，joint state 正常

阶段 3：passive 模式
目标：机器人不炸、不抖、不飞

阶段 4：PD stand
目标：不经过 OCS2，先能保持默认站姿

阶段 5：OCS2 stance
目标：四脚支撑站稳，不移动

阶段 6：OCS2 小幅 base 目标
目标：轻微前后左右移动，不摔

阶段 7：OCS2 trot
目标：再开始调步态
```

最重要的一句话：

```text id="1whd0g"
你的机器人接 OCS2，先证明“模型 + 接口 + 状态估计 + WBC 能站稳”，再证明“MPC 能走路”。
```

也就是说，**站不稳之前，不要调小跑；接口没通之前，不要调 MPC；passive 都炸之前，不要看 WBC。**

[1]: https://github.com/legubiao/quadruped_ros2_control/tree/humble/controllers/ocs2_quadruped_controller "quadruped_ros2_control/controllers/ocs2_quadruped_controller at humble · legubiao/quadruped_ros2_control · GitHub"
[2]: https://control.ros.org/humble/doc/getting_started/getting_started.html "Getting Started — ROS2_Control: Humble Jun 2026 documentation"
