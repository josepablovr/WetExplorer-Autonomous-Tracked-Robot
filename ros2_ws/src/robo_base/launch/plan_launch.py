from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('forward0', default_value='10'),
        DeclareLaunchArgument('steer0', default_value='30'),
        DeclareLaunchArgument('stop0', default_value='1'),
        DeclareLaunchArgument('forward1', default_value='100'),
        DeclareLaunchArgument('steer1', default_value='30'),
        DeclareLaunchArgument('stop1', default_value='0'),
        DeclareLaunchArgument('forward2', default_value='100'),
        DeclareLaunchArgument('steer2', default_value='30'),
        DeclareLaunchArgument('stop2', default_value='0'),
        DeclareLaunchArgument('forward3', default_value='100'),
        DeclareLaunchArgument('steer3', default_value='0'),
        DeclareLaunchArgument('stop3', default_value='0'),

        # Node definition
        Node(
            package='robo_base',
            executable='robo_plan',
            name='robo_plan',
            output='screen',
            parameters=[
                {'forward0': LaunchConfiguration('forward0')},
                {'steer0': LaunchConfiguration('steer0')},
                {'stop0': LaunchConfiguration('stop0')},
                {'forward1': LaunchConfiguration('forward1')},
                {'steer1': LaunchConfiguration('steer1')},
                {'stop1': LaunchConfiguration('stop1')},
                {'forward2': LaunchConfiguration('forward2')},
                {'steer2': LaunchConfiguration('steer2')},
                {'stop2': LaunchConfiguration('stop2')},
                {'forward3': LaunchConfiguration('forward3')},
                {'steer3': LaunchConfiguration('steer3')},
                {'stop3': LaunchConfiguration('stop3')},
            ],
        ),
    ])
