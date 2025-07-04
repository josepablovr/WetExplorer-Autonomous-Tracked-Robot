from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('mode', default_value='0'),

        # Node definition
        Node(
            package='robo_base',
            executable='mode',
            name='mode',
            output='screen',
            parameters=[
                {'port': LaunchConfiguration('port')},
                {'mode': LaunchConfiguration('mode')},
            ],
        ),
    ])
