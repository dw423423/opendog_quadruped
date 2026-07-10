from pathlib import Path

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def find_workspace_path(relative_path):
    for parent in Path(__file__).resolve().parents:
        candidate = parent / relative_path
        if candidate.exists():
            return str(candidate)
    raise FileNotFoundError(f"Could not find {relative_path} from {__file__}")


def generate_launch_description():
    rviz_config_file = get_package_share_directory('ocs2_anymal_loopshaping_mpc') + "/config/rviz/demo_config.rviz"
    urdf_model_path = find_workspace_path("src/ToGo.Prototype/urdf/ToGo.Prototype_abs.urdf")
    export_dataset_dir = find_workspace_path("date")
    robot_description = xacro.process_file(urdf_model_path).toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_name',
            default_value='togo_prototype'
        ),
        DeclareLaunchArgument(
            name='config_name',
            default_value='togo_prototype'
        ),
        DeclareLaunchArgument(
            name='terrain_name',
            default_value='step.png'
        ),
        DeclareLaunchArgument(
            name='terrain_scale',
            default_value='0.35'
        ),
        DeclareLaunchArgument(
            name='terrain_center_x',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            name='forward_distance',
            default_value='3.0'
        ),
        DeclareLaunchArgument(
            name='perception_parameter_file',
            default_value=get_package_share_directory(
                'convex_plane_decomposition_ros') + '/config/parameters.yaml'
        ),
        DeclareLaunchArgument(
            name='export_dataset',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='export_dataset_dir',
            default_value=export_dataset_dir
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'publish_frequency': 100.0,
                'use_tf_static': True,
                'robot_description': robot_description,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_ocs2',
            output='screen',
            arguments=["-d", rviz_config_file]
        ),
        Node(
            package='ocs2_anymal_loopshaping_mpc',
            executable='ocs2_anymal_loopshaping_mpc_perceptive_demo',
            name='ocs2_anymal_loopshaping_mpc_perceptive_demo',
            output='screen',
            parameters=[
                {
                    'config_name': LaunchConfiguration('config_name'),
                    'forward_velocity': 0.25,
                    'forward_distance': LaunchConfiguration('forward_distance'),
                    'terrain_name': LaunchConfiguration('terrain_name'),
                    'ocs2_anymal_description': urdf_model_path,
                    'terrain_scale': LaunchConfiguration('terrain_scale'),
                    'terrain_center_x': LaunchConfiguration('terrain_center_x'),
                    'adaptReferenceToTerrain': True,
                    'export_dataset': ParameterValue(
                        LaunchConfiguration('export_dataset'),
                        value_type=bool),
                    'export_dataset_dir': LaunchConfiguration('export_dataset_dir')
                },
                LaunchConfiguration('perception_parameter_file')
            ]
        ),
    ])
