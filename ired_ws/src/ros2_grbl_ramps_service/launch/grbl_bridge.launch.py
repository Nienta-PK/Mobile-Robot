# grbl_bridge_with_ultrasonic.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue   # <-- keep like your original

def generate_launch_description():
    # --- your original GRBL bits (unchanged style) ---
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')
    baud_arg = DeclareLaunchArgument('baud', default_value='115200')

    node = Node(
            package='ros2_grbl_ramps_service',
            executable='grbl_bridge_node.py',
            name='grbl_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
            }]
        )

    # --- ultrasonic guard (same style) ---
    us_trig_arg  = DeclareLaunchArgument('us_trig_pin', default_value='23')
    us_echo_arg  = DeclareLaunchArgument('us_echo_pin', default_value='24')
    us_chip_arg  = DeclareLaunchArgument('us_gpiochip', default_value='0')
    us_rate_arg  = DeclareLaunchArgument('us_loop_hz', default_value='20.0')
    us_stop_arg  = DeclareLaunchArgument('stop_distance_cm', default_value='15.0')
    us_hyst_arg  = DeclareLaunchArgument('resume_hysteresis_cm', default_value='2.0')
    us_auto_arg  = DeclareLaunchArgument('auto_resume', default_value='true')
    us_topic_arg = DeclareLaunchArgument('us_topic', default_value='ultrasonic/range')
    us_ns_arg    = DeclareLaunchArgument('grbl_namespace', default_value='grbl')

    ultrasonic_node = Node(
            package='ros2_grbl_ramps_service',
            executable='ultrasonic_guard_node.py',      # console_scripts entry point
            name='ultrasonic_guard',
            output='screen',
            parameters=[{
                'trig_pin':             LaunchConfiguration('us_trig_pin'),
                'echo_pin':             LaunchConfiguration('us_echo_pin'),
                'gpiochip':             LaunchConfiguration('us_gpiochip'),
                'loop_hz':              LaunchConfiguration('us_loop_hz'),
                'stop_distance_cm':     LaunchConfiguration('stop_distance_cm'),
                'resume_hysteresis_cm': LaunchConfiguration('resume_hysteresis_cm'),
                'auto_resume':          LaunchConfiguration('auto_resume'),
                'topic':                LaunchConfiguration('us_topic'),
                'grbl_namespace':       LaunchConfiguration('grbl_namespace'),
                'min_range_cm': 2.0,
                'max_range_cm': 400.0,
            }]
        )

    return LaunchDescription([
            # original
            port_arg, baud_arg,
            node,
            # ultrasonic args + node
            us_trig_arg, us_echo_arg, us_chip_arg, us_rate_arg,
            us_stop_arg, us_hyst_arg, us_auto_arg, us_topic_arg, us_ns_arg,
            ultrasonic_node,
        ])
