from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments for joystick configuration (optional for flexibility)
        DeclareLaunchArgument('dev', default_value='/dev/input/js0', description='Joystick device'),
        DeclareLaunchArgument('deadzone', default_value='0.05', description='Joystick deadzone'),
        DeclareLaunchArgument('autorepeat_rate', default_value='5', description='Joystick autorepeat rate'),

        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                {'dev': LaunchConfiguration('dev')},
                {'deadzone': LaunchConfiguration('deadzone')},
                {'autorepeat_rate': LaunchConfiguration('autorepeat_rate')},
            ],
            respawn=True,
        ),

        # Axes parameters for teleop_joy
        Node(
            package='robo_base',
            executable='teleop_joy',
            name='teleop_joy',
            parameters=[
                {'axis_linear': 3},
                {'axis_angular': 1},
                {'axis_rotation': 4},
                {'scale_linear': 1000.0},
                {'scale_angular': 1000.0},
                {'scale_rotation': 1000.0},
            ],
        ),
    ])
