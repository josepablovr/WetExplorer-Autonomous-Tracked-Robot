from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wetexplorer_vision_predator',
            executable='pose',
            name='pose',
            output='screen',
            parameters=[{
                 # Add parameters if required
            }]
        )
        
    ])
