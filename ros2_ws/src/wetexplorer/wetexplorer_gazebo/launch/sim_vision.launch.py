from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
import os



def generate_launch_description():
    
 
    pkg = get_package_share_directory('wetexplorer_gazebo') 
    return LaunchDescription([
        #Launch SIM
        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('wetexplorer_gazebo' ),'launch','description_tracks.launch.py'
                    )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
        ),


        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(pkg, "config", "point_cloud:.rviz")])


        

    ])