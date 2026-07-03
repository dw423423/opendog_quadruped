from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    target_command = get_package_share_directory('ocs2_anymal_mpc') + "/config/c_series/targetCommand.info"
    description_name = "/home/dw/workspace/opendog_ros2/src/ToGo.Prototype/urdf/ToGo.Prototype_abs.urdf"

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/mpc.launch.py']),
            launch_arguments={
                'robot_name': 'togo_prototype',
                'config_name': 'togo_prototype',
                'description_name': description_name,
                'urdf_model_path': description_name,
                'target_command': target_command
            }.items()
        )
    ])
