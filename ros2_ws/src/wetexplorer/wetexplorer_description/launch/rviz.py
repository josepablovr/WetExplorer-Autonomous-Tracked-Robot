import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import launch_ros
from launch_ros.substitutions import FindPackageShare
import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    

   
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("wetexplorer_description"), "config", "nav.rviz"]
    )
   
    
    # Launch RViz2 with the robot model
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Launch all nodes
    return launch.LaunchDescription([        #,
        node_rviz
        
    ])
