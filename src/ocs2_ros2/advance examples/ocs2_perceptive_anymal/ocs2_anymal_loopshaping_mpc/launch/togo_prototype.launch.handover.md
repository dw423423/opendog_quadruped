# togo_prototype.launch.py Handover

## Purpose

`togo_prototype.launch.py` is a thin launch wrapper for running the existing
`ocs2_anymal_loopshaping_mpc/launch/mpc.launch.py` pipeline with the
`ToGo.Prototype` robot model.

It follows the same pattern as `anymal_c.launch.py`: the real launch logic stays
in `mpc.launch.py`, while this file only supplies robot-specific arguments.

## Main File

```text
/home/dw/workspace/opendog_ros2/src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/launch/togo_prototype.launch.py
```

## Launch Arguments Passed To mpc.launch.py

```python
'robot_name': 'togo_prototype'
'config_name': 'togo_prototype'
'description_name': '/home/dw/workspace/opendog_ros2/src/ToGo.Prototype/urdf/ToGo.Prototype_abs.urdf'
'urdf_model_path': '/home/dw/workspace/opendog_ros2/src/ToGo.Prototype/urdf/ToGo.Prototype_abs.urdf'
'target_command': '<ocs2_anymal_mpc share>/config/c_series/targetCommand.info'
```

`description_name` is passed to the MPC and dummy MRT nodes as the URDF path.
`urdf_model_path` is used by `robot_state_publisher` inside `mpc.launch.py`.
For ToGo these two values intentionally point to the same URDF file.

## Robot Model

The launch file uses:

```text
/home/dw/workspace/opendog_ros2/src/ToGo.Prototype/urdf/ToGo.Prototype_abs.urdf
```

This is the absolute-path URDF variant. It uses `file:///.../meshes/...` mesh
paths so RViz can load the ToGo STL files reliably.

Before this launch file was added, these five ToGo joints were locked:

```text
J1_waist
Jfl4_wheel
Jfr4_wheel
Jrl4_wheel
Jrr4_wheel
```

After locking them, the active model has 12 movable joints, matching the
quadruped MPC assumption of 4 legs times 3 joints.

## Config Directory

`config_name='togo_prototype'` resolves to:

```text
/home/dw/workspace/opendog_ros2/install/ocs2_anymal_loopshaping_mpc/share/ocs2_anymal_loopshaping_mpc/config/togo_prototype/
```

With `--symlink-install`, that install path links back to:

```text
/home/dw/workspace/opendog_ros2/src/ocs2_ros2/advance examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/config/togo_prototype/
```

The directory contains:

```text
frame_declaration.info
task.info
loopshaping.info
multiple_shooting.info
```

## Config Differences From c_series

`loopshaping.info` and `multiple_shooting.info` are unchanged from `c_series`.

`frame_declaration.info` is changed to use ToGo link and joint names:

```text
root: F_base

left_front:  Jfl1_hip_roll, Jfl2_hip_pitch, Jfl3_knee -> Lfl4_wheel_contact
right_front: Jfr1_hip_roll, Jfr2_hip_pitch, Jfr3_knee -> Lfr4_wheel_contact
left_hind:   Jrl1_hip_roll, Jrl2_hip_pitch, Jrl3_knee -> Lrl4_wheel_contact
right_hind:  Jrr1_hip_roll, Jrr2_hip_pitch, Jrr3_knee -> Lrr4_wheel_contact
```

The contact links are fixed child frames of the wheel links, offset by
`xyz="0 -0.07898243040 0.04314829848"` from the wheel-frame origin. This is the
`0.09 m` wheel radius projected into the wheel frame for the initial standing
posture, so the MPC point-foot contact is placed at the wheel bottom instead of
the wheel axle center.

`task.info` keeps the same MPC/SQP/loopshaping algorithm settings as `c_series`,
but changes robot-specific values:

```text
robotName
joint_lower_limits
joint_upper_limits
joint_velocity_limits
joint_torque_limits
initialRobotState
```

`recompileLibraries` is enabled for this ToGo config so the generated CppAD
libraries are rebuilt with the wheel-bottom contact frames instead of any stale
wheel-center cache under `/tmp/ocs2/togo_prototype_*`.

## How To Build/Install

If this launch file or the `togo_prototype` config directory is newly added or
renamed, rebuild the package so ROS 2 can find it in `install/`:

```bash
cd /home/dw/workspace/opendog_ros2
source install/setup.bash
colcon build --packages-select ocs2_anymal_loopshaping_mpc --symlink-install
source install/setup.bash
```

Without rebuilding, ROS 2 may report:

```text
file 'togo_prototype.launch.py' was not found in the share directory of package 'ocs2_anymal_loopshaping_mpc'
```

## How To Launch

```bash
cd /home/dw/workspace/opendog_ros2
source install/setup.bash
ros2 launch ocs2_anymal_loopshaping_mpc togo_prototype.launch.py
```

If the environment cannot write to the default ROS log directory, set:

```bash
export ROS_LOG_DIR=/tmp/ros2_launch_logs
```

## Validation Already Performed

The following checks were performed when this launch entry was created:

```text
Python syntax check for togo_prototype.launch.py passed.
All links referenced by frame_declaration.info exist in ToGo.Prototype.urdf.
All joints referenced by frame_declaration.info exist in ToGo.Prototype.urdf.
task.info has 12 joint limits and 12 initial joint values.
Initial joint values are within the configured joint limits.
```

## Known Caveats

The underlying OCS2 loopshaping interface still contains ANYmal-oriented naming
in package names, executable names, and some ROS topic code paths. This launch
entry runs the existing ANYmal MPC pipeline with the ToGo URDF and ToGo
configuration; it is not a full package rename to a native ToGo MPC package.

The initial state in `task.info` is a conservative first-pass stance based on
the ToGo URDF joint limits. It may need tuning after the MPC is run and the
resulting posture, contact positions, and solver behavior are observed.

The `target_command` file is still reused from:

```text
ocs2_anymal_mpc/config/c_series/targetCommand.info
```

If ToGo needs a different default command pose or motion target, create a
ToGo-specific target command file and update `togo_prototype.launch.py`.
