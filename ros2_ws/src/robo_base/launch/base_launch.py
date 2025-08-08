from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('sys', default_value='1'),
        DeclareLaunchArgument('enc', default_value='0'),
        DeclareLaunchArgument('io', default_value='0'),
        DeclareLaunchArgument('velocity', default_value='1'),
        DeclareLaunchArgument('rate', default_value='50'),
        DeclareLaunchArgument('enc_pulse', default_value='512'),
        DeclareLaunchArgument('pulse_rev_left', default_value='4096'),
        DeclareLaunchArgument('pulse_rev_right', default_value='4096'),
        DeclareLaunchArgument('gear_ratio', default_value='30'),
        DeclareLaunchArgument('sprocket', default_value='83.0'),
        DeclareLaunchArgument('track', default_value='0.6103'),
        DeclareLaunchArgument('cmd_vel', default_value='1'),

        # Node definition
        Node(
            package='robo_base',
            executable='robo_base_ros2',
            name='robo_base',
            output='screen',
            parameters=[
                {'port': LaunchConfiguration('port')},
                {'sys': LaunchConfiguration('sys')},
                {'enc': LaunchConfiguration('enc')},
                {'io': LaunchConfiguration('io')},
                {'velocity': LaunchConfiguration('velocity')},
                {'rate': LaunchConfiguration('rate')},
                {'enc_pulse': LaunchConfiguration('enc_pulse')},
                {'pulse_rev_left': LaunchConfiguration('pulse_rev_left')},
                {'pulse_rev_right': LaunchConfiguration('pulse_rev_right')},
                {'gear_ratio': LaunchConfiguration('gear_ratio')},
                {'sprocket': LaunchConfiguration('sprocket')},
                {'track': LaunchConfiguration('track')},
                {'cmd_vel': LaunchConfiguration('cmd_vel')},
            ],
        ),
    ])
