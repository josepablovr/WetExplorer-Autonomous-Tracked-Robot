from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wetexplorer_vision',
            executable='obstacle_finder',
            name='obstacle_finder',
            parameters=[{
                'config_file': 'install/wetexplorer_vision/share/wetexplorer_vision/config/obstacle_params.yaml'
            }]
        )
    ])
