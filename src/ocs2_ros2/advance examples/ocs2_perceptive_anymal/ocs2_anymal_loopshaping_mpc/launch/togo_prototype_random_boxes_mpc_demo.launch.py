from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def find_workspace_path(relative_path):
    for parent in Path(__file__).resolve().parents:
        candidate = parent / relative_path
        if candidate.exists():
            return str(candidate)
    raise FileNotFoundError(f"Could not find {relative_path} from {__file__}")


def generate_launch_description():
    package_share = get_package_share_directory('ocs2_anymal_loopshaping_mpc')
    export_dataset_dir = find_workspace_path('date')
    base_launch = package_share + '/launch/togo_prototype_perceptive_mpc_demo.launch.py'

    return LaunchDescription([
        # Stops 1 m before the map end, within the post-obstacle platform.
        DeclareLaunchArgument(name='forward_distance', default_value='6.0'),
        DeclareLaunchArgument(name='export_dataset', default_value='true'),
        DeclareLaunchArgument(
            name='export_dataset_dir', default_value=export_dataset_dir),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'terrain_name': 'random_boxes_11cm.png',
                # Full-scale white pixels represent the specified 0.11 m boxes.
                'terrain_scale': '0.11',
                'terrain_center_x': '3.5',
                'forward_distance': LaunchConfiguration('forward_distance'),
                'perception_parameter_file':
                    package_share + '/config/random_boxes_parameters.yaml',
                'export_dataset': LaunchConfiguration('export_dataset'),
                'export_dataset_dir': LaunchConfiguration('export_dataset_dir'),
            }.items()),
    ])
