import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_name = LaunchConfiguration('map_name')
    param_name = LaunchConfiguration('param_name')

    ired_navigation_path = get_package_share_directory('ired_navigation')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='map.yaml',
            description='Name of the map file used in navigation mode.'
        ),
        DeclareLaunchArgument(
            'param_name',
            default_value='navigation.yaml',
            description='Name of the parameter YAML file used in the Navigation Stack.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_file_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': PathJoinSubstitution([ired_navigation_path, 'maps', map_name]),
                'params_file': PathJoinSubstitution([ired_navigation_path, 'param', param_name]),
                'use_sim_time': use_sim_time
            }.items())])
