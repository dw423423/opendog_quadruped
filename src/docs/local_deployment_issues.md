# Local Deployment Issues and Fixes

本文记录本 workspace 在本地部署 ROS2 + OCS2 + WBC 四足项目时遇到的构建问题、原因、最小修改和验证方式。

当前策略是优先跑通基础示例和 `ocs2_quadruped_controller` 主链路；RL、RaiSim、MPC-Net 等可选高级模块先隔离，避免把非必要 SDK 依赖引入基础部署。

## 1. rl_quadruped_controller 缺少 Torch

现象：

```text
Could not find a package configuration file provided by "Torch"
TorchConfig.cmake
torch-config.cmake
```

原因：

`rl_quadruped_controller` 是强化学习控制器，源码中直接使用 `torch::Tensor` 和 `torch::jit::load()`，因此必须安装 libtorch。基础 `unitree_guide_controller` 示例不依赖该包。

最小修改：

```bash
touch src/quadruped_ros2_control/controllers/rl_quadruped_controller/COLCON_IGNORE
```

作用：

让 colcon 暂时跳过 RL 控制器，先跑通非 RL 示例。后续要跑 RL 时，删除该文件并按该包 README 安装 libtorch。

验证：

```bash
colcon list --names-only | grep rl_quadruped_controller
```

期望无输出。

基础示例构建验证：

```bash
colcon build --packages-up-to unitree_guide_controller go2_description keyboard_input --symlink-install
```

已验证结果：

```text
Summary: 5 packages finished
```

## 2. ocs2_quadruped_controller 缺少 ocs2_legged_robot_ros

现象：

```text
Could not find a package configuration file provided by "ocs2_legged_robot_ros"
ocs2_legged_robot_rosConfig.cmake
ocs2_legged_robot_ros-config.cmake
```

原因：

`ocs2_quadruped_controller` 依赖外部 OCS2 ROS2 源码仓库中的 `ocs2_legged_robot_ros`，该包不在原始 workspace 中。

处理方式：

```bash
cd /home/dw/workspace/opendog_ros2/src
git clone https://github.com/legubiao/ocs2_ros2
cd ocs2_ros2
git submodule update --init --recursive
```

说明：

不需要再切换到其它 ROS2 分支；`legubiao/ocs2_ros2` 本身就是 ROS2 版本仓库。

验证：

```bash
cd /home/dw/workspace/opendog_ros2
colcon list --names-only | grep ocs2_legged_robot_ros
```

期望输出：

```text
ocs2_legged_robot_ros
```

## 3. cgal5_colcon 在 CMake 3.22 下 URL 解析失败

现象：

```text
CMake Error at /usr/share/cmake-3.22/Modules/ExternalProject.cmake
At least one entry of URL is a path (invalid in a list)
```

原因：

`cgal5_colcon` 的 `ExternalProject_Add()` 使用了 `DOWNLOAD_EXTRACT_TIMESTAMP TRUE`。该参数在当前 CMake 3.22 环境中不被识别，被误解析为 URL 列表的一部分。

修改文件：

```text
src/ocs2_ros2/submodules/plane_segmentation_ros2/cgal5_colcon/CMakeLists.txt
```

最小修改：

```cmake
set(CGAL_DOWNLOAD_OPTIONS "")
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
        list(APPEND CGAL_DOWNLOAD_OPTIONS DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
endif()
ExternalProject_Add(cgal
        URL https://github.com/CGAL/cgal/archive/refs/tags/v${CGAL_VERSION}.tar.gz
        ${CGAL_DOWNLOAD_OPTIONS}
        UPDATE_COMMAND ""
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
        -DCMAKE_BUILD_TYPE:STRING=Release
        BUILD_COMMAND $(MAKE)
        INSTALL_COMMAND $(MAKE) install
)
```

验证：

```bash
colcon build --packages-select cgal5_colcon --symlink-install
```

已验证结果：

```text
Finished <<< cgal5_colcon
Summary: 1 package finished
```

备注：

第一次构建可能需要从 GitHub 下载 CGAL 5.3。如果代理或网络不可用，会出现 `Couldn't connect to server`，这不是 CMake 补丁问题，需要修复网络或代理后重试。

## 4. ocs2_raisim_core 缺少 raisim

现象：

```text
Could not find a package configuration file provided by "raisim"
raisimConfig.cmake
raisim-config.cmake
```

原因：

`ocs2_raisim_core` 属于 OCS2 的 RaiSim 高级示例，需要单独安装 RaiSim SDK 和 license。当前 Gazebo / MuJoCo / `ocs2_quadruped_controller` 主链路不需要它。

最小修改：

```bash
touch "src/ocs2_ros2/advance examples/ocs2_raisim/COLCON_IGNORE"
```

验证：

```bash
colcon list --names-only | grep raisim
```

期望无输出。

后续如果要运行 RaiSim 示例，再删除该 `COLCON_IGNORE` 并单独安装 RaiSim。

## 5. grid_map_sdf 找不到 pcl/point_cloud.h

现象：

```text
/opt/ros/humble/include/grid_map_sdf/SignedDistanceField.hpp:16:10:
fatal error: pcl/point_cloud.h: No such file or directory
```

原因：

`grid_map_sdf/SignedDistanceField.hpp` 公开包含 PCL 头文件：

```cpp
#include <pcl/point_cloud.h>
```

但 `grid_map_sdf` 没有可靠地把 PCL include 路径传递给下游。当前机器上 PCL 头文件存在，问题是 `ocs2_quadruped_controller` 没有显式引入 PCL。

修改文件：

```text
src/quadruped_ros2_control/controllers/ocs2_quadruped_controller/CMakeLists.txt
```

最小修改：

```cmake
find_package(PCL REQUIRED COMPONENTS common)
```

并将 PCL 加入 include、link 和 export：

```cmake
target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${qpOASES_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src)
target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
)

ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS} PCL)
```

注意：

这里 `target_link_libraries()` 使用非 keyword 形式，是为了和 `ament_target_dependencies()` 在当前环境中的调用签名保持一致，避免 CMake 报：

```text
All uses of target_link_libraries with a target must be either all-keyword or all-plain.
```

验证：

```bash
colcon build --packages-select ocs2_quadruped_controller --symlink-install
```

已验证结果：

```text
Finished <<< ocs2_quadruped_controller
Summary: 1 package finished
```

## 6. ocs2_mpcnet_core 缺少 onnxruntime

现象：

```text
Could not find a package configuration file provided by "onnxruntime"
onnxruntimeConfig.cmake
onnxruntime-config.cmake
```

原因：

`ocs2_mpcnet_core` 属于 OCS2 的 MPC-Net 高级示例，用于 ONNX 策略网络推理。当前四足 OCS2 控制器主链路不需要它。

最小修改：

```bash
touch "src/ocs2_ros2/advance examples/ocs2_mpcnet/COLCON_IGNORE"
```

验证：

```bash
colcon list --names-only | grep mpcnet
```

期望无输出。

后续如果要运行 MPC-Net 示例，再删除该 `COLCON_IGNORE` 并按 `src/ocs2_ros2/advance examples/ocs2_mpcnet/README.md` 安装 ONNX Runtime。

## 7. ocs2_quadruped_controller 找不到 gazebo_classic.launch.py

现象：

```text
file 'gazebo_classic.launch.py' was not found in the share directory of package 'ocs2_quadruped_controller'
```

原因：

`ocs2_quadruped_controller` 没有提供 `gazebo_classic.launch.py`。该包实际提供的 launch 文件是：

```text
elevation_mapping.launch.py
gazebo.launch.py
mujoco.launch.py
```

`gazebo_classic.launch.py` 存在于其它控制器包中，例如 `unitree_guide_controller` 和 `rl_quadruped_controller`，但不是 OCS2 控制器包的 launch 文件名。

处理方式：

使用 OCS2 README 中的实际命令：

```bash
source install/setup.bash
ros2 launch ocs2_quadruped_controller gazebo.launch.py pkg_description:=go2_description
```

验证：

```bash
find install/ocs2_quadruped_controller/share/ocs2_quadruped_controller/launch -maxdepth 1 -print
```

期望至少看到：

```text
install/ocs2_quadruped_controller/share/ocs2_quadruped_controller/launch/gazebo.launch.py
install/ocs2_quadruped_controller/share/ocs2_quadruped_controller/launch/mujoco.launch.py
```

## 8. Ignition Gazebo 6 找不到 ForceTorque 插件类

现象：

```text
[ignition::plugin::Loader::LookupPlugin] Failed to get info for [gz::sim::systems::ForceTorque].
Could not find a plugin with that name or alias.
Failed to load system plugin [gz::sim::systems::ForceTorque]
```

原因：

当前环境运行的是 `ign gazebo-5`，插件路径为 Ignition Gazebo 6：

```text
/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo-forcetorque-system.so
```

该库实际注册的插件别名是：

```text
ignition::gazebo::systems::ForceTorque
```

而 Go2 的 xacro 中使用的是较新的 GZ 命名空间：

```xml
name="gz::sim::systems::ForceTorque"
```

因此库文件能找到，但类名别名匹配失败。

修改文件：

```text
src/quadruped_ros2_control/descriptions/unitree/go2_description/xacro/gazebo.xacro
```

最小修改：

```xml
<plugin filename="gz-sim-forcetorque-system" name="ignition::gazebo::systems::ForceTorque"/>
```

验证插件库实际别名：

```bash
strings /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libignition-gazebo-forcetorque-system.so | grep ForceTorque
```

验证 xacro 展开结果：

```bash
source install/setup.bash
ros2 run xacro xacro src/quadruped_ros2_control/descriptions/unitree/go2_description/xacro/robot.xacro GAZEBO:=true | grep ForceTorque
```

期望看到：

```text
<plugin filename="gz-sim-forcetorque-system" name="ignition::gazebo::systems::ForceTorque"/>
```

## 当前推荐构建命令

基础 Unitree Guide 示例：

```bash
colcon build --packages-up-to unitree_guide_controller go2_description keyboard_input --symlink-install
```

OCS2 四足控制器主链路：

```bash
colcon build --packages-up-to ocs2_quadruped_controller --symlink-install
```

避免在依赖未完全整理前直接全量构建：

```bash
colcon build --symlink-install
```

因为 `ocs2_ros2` 包含很多高级示例，直接全量构建会继续拉入非必要依赖。

## 当前隔离的可选模块

```text
src/quadruped_ros2_control/controllers/rl_quadruped_controller/COLCON_IGNORE
src/ocs2_ros2/advance examples/ocs2_raisim/COLCON_IGNORE
src/ocs2_ros2/advance examples/ocs2_mpcnet/COLCON_IGNORE
```

这些隔离是本地部署阶段的稳定性措施，不代表模块不可用。后续需要对应功能时，删除相应 `COLCON_IGNORE` 并单独安装其 SDK 依赖。
