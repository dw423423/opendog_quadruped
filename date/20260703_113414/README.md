# 20260703_113414 Motion Export

该目录是一份 ToGo Prototype 机器人感知 MPC 计算后的运动数据导出包。导出格式为
`ocs2_amp_motion_csv_bundle`，主要用于后续 AMP 训练数据处理或 `csv2npz`
转换。

## 概览

- 机器人配置：`togo_prototype`
- 机器人模型：`robot.urdf`
- 地形：`step.png`
- 地形缩放：`0.35`
- 期望前向速度：`0.5 m/s`
- 期望前进距离：`3.0 m`
- 采样频率：`100 Hz`
- 帧数：`841`
- 时间范围：`0.0 s` 到 `8.4 s`
- 关节数：`12`
- Body 数：`23`
- 坐标系：world

## 目录内容

| 文件 | 说明 |
| --- | --- |
| `metadata.json` | 数据包主元信息，包括 fps、帧数、关节名、body 名、shape、单位和文件索引。 |
| `export_schema.json` | 导出格式说明，记录 AMP/csv2npz 读取这些文件时需要遵守的字段、单位和 shape。 |
| `scenario_metadata.json` | 场景元信息，包括机器人配置、URDF 来源、地形来源、运动参数和地形地图参数。 |
| `robot.urdf` | 本次运动数据对应的 ToGo Prototype 机器人 URDF 模型。 |
| `terrain_source.png` | 原始地形图片，尺寸为 `100 x 200`。 |
| `terrain_elevation.csv` | 原始地形高度图矩阵，shape 为 `200 x 100`，单位为 m。 |
| `terrain_elevation_points.csv` | 原始地形高度图展开后的点数据，列为 `row,col,x,y,z`。 |
| `terrain_filtered_elevation.csv` | 感知/滤波后的地形高度图矩阵，shape 为 `200 x 100`，单位为 m。 |
| `terrain_filtered_elevation_points.csv` | 感知/滤波后的地形高度图点数据，列为 `row,col,x,y,z`。 |
| `dof_positions.csv` | 12 个关节的位置轨迹，单位为 rad。 |
| `dof_velocities.csv` | 12 个关节的速度轨迹，单位为 rad/s。 |
| `body_positions.csv` | 23 个 body 在 world 坐标系下的位置，单位为 m。 |
| `body_rotations.csv` | 23 个 body 在 world 坐标系下的姿态四元数，顺序为 `w,x,y,z`。 |
| `body_linear_velocities.csv` | 23 个 body 在 world 坐标系下的线速度，单位为 m/s。 |
| `body_angular_velocities.csv` | 23 个 body 在 world 坐标系下的角速度，单位为 rad/s。 |

## CSV 数据格式

所有运动轨迹 CSV 的第一列都是 `time`，后续列按 `metadata.json` 中的
`dof_names` 或 `body_names` 顺序排列。

- `dof_positions.csv` 和 `dof_velocities.csv`：每个关节 1 列。
- `body_positions.csv`、`body_linear_velocities.csv` 和
  `body_angular_velocities.csv`：每个 body 3 列，后缀为 `_x,_y,_z`。
- `body_rotations.csv`：每个 body 4 列，后缀为 `_w,_x,_y,_z`。

地形矩阵 CSV 不带表头，尺寸为 `200 x 100`。对应的 `*_points.csv`
文件带表头，便于按点云形式读取。

## 关节顺序

`dof_positions.csv` 和 `dof_velocities.csv` 中的关节顺序如下：

```text
Jfl1_hip_roll
Jfl2_hip_pitch
Jfl3_knee
Jfr1_hip_roll
Jfr2_hip_pitch
Jfr3_knee
Jrl1_hip_roll
Jrl2_hip_pitch
Jrl3_knee
Jrr1_hip_roll
Jrr2_hip_pitch
Jrr3_knee
```

该顺序需要与 AMP 侧 `robot.data.joint_names` 完全一致，否则 loader
中的关节索引匹配会失败。

## Body 顺序

body 相关 CSV 中的 body 顺序如下：

```text
F_base
L0_body_front
L1_body_rear
Lrl1_hip_roll
Lrl2_hip_pitch
Lrl3_knee
Lrl4_wheel
Lrl4_wheel_contact
Lrr1_hip_roll
Lrr2_hip_pitch
Lrr3_knee
Lrr4_wheel
Lrr4_wheel_contact
Lfl1_hip_roll
Lfl2_hip_pitch
Lfl3_knee
Lfl4_wheel
Lfl4_wheel_contact
Lfr1_hip_roll
Lfr2_hip_pitch
Lfr3_knee
Lfr4_wheel
Lfr4_wheel_contact
```

这些名字用于 AMP 侧的 `reference_body` 和 `key_body_names` 匹配。

## 生成来源

该数据包由仓库中的 perceptive MPC demo 导出：

```text
src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/src/PerceptiveMpcDemo.cpp
```

README 中对应 demo 的说明位于：

```text
src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/README.md
```

默认导出路径为：

```text
/home/dw/workspace/opendog_ros2/date/<timestamp>/
```

## 注意事项

- 姿态四元数顺序是 `wxyz`，不是常见的 `xyzw`。
- body 位置和速度均为 world 坐标系下的绝对量。
- 轨迹数据来自 MPC 解；body 位姿和速度由 Pinocchio 正运动学计算得到。
- 地形高度图已经根据 `terrain_scale` 处理，并以 world 原点高度为基准。
