import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    map_name = LaunchConfiguration('map_name', default='map')
    map_path = get_package_share_directory('ired_navigation')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value=map_name,
            description='Name of map file to save to the ired_navigation package'
        ),
        Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name='map_saver_node',
            output='screen',
            arguments=['-f', PathJoinSubstitution([map_path, 'maps', map_name])]
        )
    ])