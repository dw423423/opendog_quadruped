# 地形与运动数据采集规格

## 1. 地形几何参数

需要三类地形，均为**直线行走路径**，行走方向沿 x 轴正方向。

### 1.1 上台阶

```
俯视图 (x 为前进方向):

         平台 3.0m        台阶区        平台 3.0m
  |<----------------->|<--------->|<----------------->|
                                                        
  ┌───────────────────┐           ┌───────────────────┐
  │                   │  ╲       │                   │
  │                   │    ╲     │                   │
  │                   │      ╲   │                   │
  │                   │        ╲ │                   │
  │     平台区域       │  台阶区  │     平台区域       │
  │     z=0           │          │    z=H_total       │
  │                   │          │                   │
  │                   │          │                   │
  │                   │          │                   │
  └───────────────────┘          └───────────────────┘
       边界 1.0m                      边界 1.0m
```

| 参数 | 值 | 说明 |
|------|-----|------|
| 每级台阶高度 | 0.10 m | 所有台阶高度一致 |
| 每级台阶深度 | 0.30 m | 沿前进方向的深度 |
| 台阶级数 | 6 级 | 总爬升 0.60 m |
| 台阶总深度 | 1.80 m | 6 × 0.30 m |
| 前后平台长度 | 各 3.0 m | 台阶前后的平地 |
| 边界宽度 | 1.0 m | 台阶区域左右各留 1.0 m |
| 总长度 | 7.80 m | 3.0 + 1.80 + 3.0 |
| 总宽度 | ≥ 2.0 m | |
| 台阶截面形状 | 矩形 | 每级为垂直立面 + 水平踏面 |

实现地图：`stairs_up_6x10cm.png`，位于
`src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/data/`。
其分辨率为 0.02 m，尺寸为 7.80 m × 2.00 m；行走方向为 world x 正方向：
前平台 `x=[0.0, 3.0)` m，六级台阶为 `x=[3.0, 4.8)` m，后平台为
`x=[4.8, 7.8]` m。对应启动入口会自动使用正确的高度缩放和地图原点：

```bash
ros2 launch ocs2_anymal_loopshaping_mpc togo_prototype_stairs_up_mpc_demo.launch.py forward_velocity:=0.5
```

### 1.2 下台阶

与上台阶对称，前进方向为下坡方向（从高处走向低处）。几何参数完全相同：

| 参数 | 值 |
|------|-----|
| 每级台阶高度 | 0.10 m |
| 每级台阶深度 | 0.30 m |
| 台阶级数 | 6 级 |
| 前后平台长度 | 各 3.0 m |
| 边界宽度 | 1.0 m |

实现地图：`stairs_down_6x10cm.png`，位于
`src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/data/`。
其分辨率为 0.02 m，尺寸为 7.80 m × 2.00 m；行走方向为 world x 正方向：
前平台 `x=[0.0, 3.0)` m 的源高度为 0.60 m，六级台阶在
`x=[3.0, 4.8)` m 内每级下降 0.10 m，后平台 `x=[4.8, 7.8]` m 的源高度为
0 m。地图加载时会在 world 原点处归零，因此运行时高度从起点的 0 m 逐级降至
终点的 -0.60 m。对应启动入口会自动使用正确的高度缩放和地图原点：

```bash
ros2 launch ocs2_anymal_loopshaping_mpc togo_prototype_stairs_down_mpc_demo.launch.py forward_velocity:=0.5

```

### 1.3 平地

全场平地，无任何起伏。尺寸同上（7.80 m × 2.0 m），或任意足够机器狗行走 10 秒的长度即可。

实现地图：`flat_7p8x2m.png`，位于
`src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/data/`。
其分辨率为 0.02 m，尺寸为 7.80 m × 2.00 m，所有像素高度均为 0 m；行走方向
为 world x 正方向，地图中心为 `(3.9, 0.0)` m。对应启动入口：

```bash
ros2 launch ocs2_anymal_loopshaping_mpc togo_prototype_flat_mpc_demo.launch.py forward_velocity:=0.5
```

### 1.4 随机箱体

在平坦地面上，以网格方式随机排列高度相同的方形箱体。机器人行走时会遇到随机间隔出现的障碍物。

```
俯视图（布局示意）:

  ┌──────┬──────┬──────┬──────┬──────┐
  │ 0.11 │      │ 0.11 │ 0.11 │      │
  ├──────┼──────┼──────┼──────┼──────┤  ← 每格 0.45m × 0.45m
  │      │ 0.11 │ 0.11 │      │ 0.11 │     数字 = 箱体高度 (m)
  ├──────┼──────┼──────┼──────┼──────┤     空白 = 无箱体
  │ 0.11 │      │      │ 0.11 │      │
  └──────┴──────┴──────┴──────┴──────┘
          ← 前进方向 x →
```

| 参数 | 值 | 说明 |
|------|-----|------|
| 网格宽度 | 0.45 m | 每格边长 |
| 箱体高度 | 0.11 m | 所有箱体统一高度 |
| 平台宽度 | 2.0 m | 箱体区域前后的平地 |
| 箱体区域长度 | 3.0 m | 铺有箱体的区域（≈ 6~7 列网格） |
| 总长度 | 7.0 m | 2.0 + 3.0 + 2.0 |
| 总宽度 | ≥ 2.0 m | |

实现地图：`random_boxes_11cm.png`，位于
`src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/data/`。
它以 0.01 m 分辨率表示 7.00 m × 2.00 m 的场地：前平台为
`x=[0.0, 2.0)` m，箱体区域为 `x=[2.0, 5.0)` m，后平台为
`x=[5.0, 7.0]` m。箱体网格为 6 × 4，每格 0.45 m × 0.45 m，位于
`x=[2.15, 4.85)` m、`y=[-0.90, 0.90)` m；四周保留 0.15 m（x）和
0.10 m（y）空白边距。使用固定随机种子 `20260710` 和 0.40 占用概率，生成
9 个高度为 0.11 m 的箱体，便于重复采集相同场景。对应启动入口：

```bash
ros2 launch ocs2_anymal_loopshaping_mpc togo_prototype_random_boxes_mpc_demo.launch.py forward_velocity:=0.5
```

### 1.5 随机粗糙地面

连续起伏的不规则地面，模拟碎石路或野外泥土地面。

```
侧视图:

    ╱╲    ╱╲  ╱╲
  ╱    ╲╱  ╲╱    ╲╱╲    ← 最高起伏约 ±0.08 m
  ──────────────────────
  |←     8.0 m        →|
```

| 参数 | 值 | 说明 |
|------|-----|------|
| 噪声幅度 | 0.08 m | 地面起伏的最大高度变化 |
| 噪声变化步长 | 0.01 m | 相邻网格点之间高度变化的最小单位 |
| 边界宽度 | 0.25 m | 粗糙区域两侧的平缓过渡带 |
| 总长度 | 8.0 m | |
| 总宽度 | ≥ 2.0 m | |

实现地图：`rough_terrain_8m_8cm.png`，位于
`src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/data/`。
其分辨率为 0.01 m，尺寸为 8.00 m × 2.00 m，使用固定随机种子 `20260711`
生成连续的双线性插值随机场，并将高度量化为 0.01 m。地图源高度以 0.08 m 为
基准，在 `0.00–0.16 m` 内变化；加载器在 world 原点处归零后，实际地形高度为
`[-0.08, +0.08]` m。地图四周均设有 0.25 m 的平滑过渡带，起点和终点处的
高度为 0 m。对应启动入口：

```bash
ros2 launch ocs2_anymal_loopshaping_mpc togo_prototype_rough_terrain_mpc_demo.launch.py forward_velocity:=0.5
```

### 1.6 上坡斜面

平滑的向上倾斜平面，机器人从低处走向高处。

```
侧视图:

                  ╱
                ╱
  平台 2.0m   ╱  斜坡区   平台 2.0m
  ─────────╱──────────
           ↑ 坡度 0.2 rad (≈ 11.5°)
```

| 参数 | 值 | 说明 |
|------|-----|------|
| 坡度 | 0.2 rad (≈ 11.5°) | 斜面倾斜角 |
| 平台宽度 | 2.0 m | 斜坡前后的平地 |
| 边界宽度 | 0.25 m | 斜坡两侧的过渡带 |
| 总长度 | 8.0 m | |

实现地图：`slope_up_0p2rad_8m.png`，位于
`src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/data/`。
其分辨率为 0.01 m，尺寸为 8.00 m × 2.00 m：前平台为
`x=[0.0, 2.0)` m，斜坡为 `x=[2.0, 6.0)` m，后平台为 `x=[6.0, 8.0]` m。
中心线高差为 `4.0 × tan(0.2) = 0.81084 m`；`|y|>0.75 m` 的两侧 0.25 m
区域平滑过渡。对应启动入口：

```bash
ros2 launch ocs2_anymal_loopshaping_mpc togo_prototype_slope_up_mpc_demo.launch.py forward_velocity:=0.5
```

### 1.7 下坡斜面

平滑的向下倾斜平面，与上坡对称，机器人从高处走向低处。

| 参数 | 值 |
|------|-----|
| 坡度 | 0.2 rad (≈ 11.5°) |
| 平台宽度 | 2.0 m |
| 边界宽度 | 0.25 m |

实现地图：`slope_down_0p2rad_8m.png`，位于
`src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/data/`。
其几何尺寸、0.01 m 分辨率和两侧 0.25 m 过渡带与上坡完全相同；沿 world x
正方向，前平台和斜坡从高处连续下降至后平台。地图加载时以前平台 world 原点归零，
运行时高度从 0 m 降至 `-0.81084 m`。对应启动入口：

```bash
ros2 launch ocs2_anymal_loopshaping_mpc togo_prototype_slope_down_mpc_demo.launch.py forward_velocity:=0.5
```

---

## 2. 运动任务

| 参数 | 值 |
|------|-----|
| 前进速度 | 0.5 m/s，直线沿 x 轴正方向 |
| 行走时长 | ≥ 10 秒 |
| 起步位置 | 地形起点处平台区域 |
| 其他方向速度 | 0（无侧向、无转向） |

> 七种地形各采集一条轨迹：上台阶 × 1、下台阶 × 1、平地 × 1、随机箱体 × 1、随机粗糙地面 × 1、上坡 × 1、下坡 × 1。

---

## 3. 导出数据格式

每条轨迹导出为独立的 CSV 文件集。文件列表：

```
dof_positions.csv          # 关节位置
dof_velocities.csv         # 关节速度
body_positions.csv         # 身体各部件世界坐标位置
body_rotations.csv         # 身体各部件世界坐标姿态（四元数）
body_linear_velocities.csv # 身体各部件世界坐标线速度
body_angular_velocities.csv# 身体各部件世界坐标角速度
```

### 3.1 机器人关节（共 12 个，CSV 列顺序严格按此排列）

| 序号 | 关节名 | 说明 |
|------|--------|------|
| 1 | Jfl1_hip_roll | 前左髋横滚 |
| 2 | Jfl2_hip_pitch | 前左髋俯仰 |
| 3 | Jfl3_knee | 前左膝 |
| 4 | Jfr1_hip_roll | 前右髋横滚 |
| 5 | Jfr2_hip_pitch | 前右髋俯仰 |
| 6 | Jfr3_knee | 前右膝 |
| 7 | Jrl1_hip_roll | 后左髋横滚 |
| 8 | Jrl2_hip_pitch | 后左髋俯仰 |
| 9 | Jrl3_knee | 后左膝 |
| 10 | Jrr1_hip_roll | 后右髋横滚 |
| 11 | Jrr2_hip_pitch | 后右髋俯仰 |
| 12 | Jrr3_knee | 后右膝 |

### 3.2 身体部件（共 23 个，CSV 列顺序严格按此排列）

| 序号 | 部件名 | 说明 |
|------|--------|------|
| 1 | F_base | 浮动基座 |
| 2 | L0_body_front | 前机身 |
| 3 | L1_body_rear | 后机身 |
| 4 | Lrl1_hip_roll | 后左髋横滚连杆 |
| 5 | Lrl2_hip_pitch | 后左髋俯仰连杆 |
| 6 | Lrl3_knee | 后左膝连杆 |
| 7 | Lrl4_wheel | 后左轮/足端 |
| 8 | Lrl4_wheel_contact | 后左轮接触点 |
| 9 | Lrr1_hip_roll | 后右髋横滚连杆 |
| 10 | Lrr2_hip_pitch | 后右髋俯仰连杆 |
| 11 | Lrr3_knee | 后右膝连杆 |
| 12 | Lrr4_wheel | 后右轮/足端 |
| 13 | Lrr4_wheel_contact | 后右轮接触点 |
| 14 | Lfl1_hip_roll | 前左髋横滚连杆 |
| 15 | Lfl2_hip_pitch | 前左髋俯仰连杆 |
| 16 | Lfl3_knee | 前左膝连杆 |
| 17 | Lfl4_wheel | 前左轮/足端 |
| 18 | Lfl4_wheel_contact | 前左轮接触点 |
| 19 | Lfr1_hip_roll | 前右髋横滚连杆 |
| 20 | Lfr2_hip_pitch | 前右髋俯仰连杆 |
| 21 | Lfr3_knee | 前右膝连杆 |
| 22 | Lfr4_wheel | 前右轮/足端 |
| 23 | Lfr4_wheel_contact | 前右轮接触点 |

### 3.3 坐标系约定

- `body_positions`：世界坐标系下的绝对位置 (x, y, z)，单位 m
- `body_rotations`：世界坐标系下的姿态四元数，顺序 w,x,y,z
- `body_linear_velocities`：世界坐标系下的线速度 (vx, vy, vz)，单位 m/s
- `body_angular_velocities`：世界坐标系下的角速度 (ωx, ωy, ωz)，单位 rad/s
- `dof_positions`：关节角位置，单位 rad
- `dof_velocities`：关节角速度，单位 rad/s

### 3.4 CSV 格式

- 第一列为时间戳（秒）
- 后续列为对应关节/部件的分量值
- 列头包含名称，如 `time, Jfl1_hip_roll, Jfl2_hip_pitch, ...`
- `body_positions` 每部件占 3 列 (x, y, z)
- `body_rotations` 每部件占 4 列 (w, x, y, z)

### 3.5 帧率

导出帧率 **100 fps**。

---

## 4. 补充说明

- 轨迹应为 OCS2/MPC 的优化结果（即关节空间轨迹满足机器人动力学约束）
- 可以使用 `/home/sim/桌面/togo/amp_data/` 下已有的 CSV 文件作为格式参考
- 三段轨迹可以分别放在不同文件夹中，如 `stairs_up/`、`stairs_down/`、`flat/`
