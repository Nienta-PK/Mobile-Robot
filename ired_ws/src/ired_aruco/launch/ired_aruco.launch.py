from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ired_aruco',
            executable='aruco_node',
            name='ired_aruco_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/camera/color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'debug_image_topic': '/ired_aruco/image',
                'pose_array_topic': '/ired_aruco/poses',
                'ids_topic': '/ired_aruco/ids',
                'text_topic': '/ired_aruco/pose_text',
                'aruco_dictionary': 'DICT_4X4_50',
                'marker_length_m': 0.05,  # meters; set to your tag size
                'draw_axes': True,
                'publish_debug_image': True,
                'queue_size': 5,
            }],
        ),
    ])
