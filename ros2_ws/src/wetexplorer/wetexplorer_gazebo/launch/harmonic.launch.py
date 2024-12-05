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
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spaceros_gz_sim = get_package_share_directory('wetexplorer_gazebo')   
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])    
    gz_world_path = PathJoinSubstitution([pkg_spaceros_gz_sim, 'worlds'])
    bridge_params = os.path.join(get_package_share_directory('wetexplorer_gazebo'),'config','gz_bridge.yaml')
    resource_world_path = '/ros2_ws/src/wetexplorer'
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            choices=['empty', 'mars', 'enceladus'],
            description='World to load into Gazebo'
        ),
        SetLaunchConfiguration(name='world_file', 
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.world')]),

        # LOAD MODEL PARAMENTER
        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('wetexplorer_description' ),'launch','description_tracks.launch.py'
                    )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
        ),


        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_world_path),       

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([pkg_spaceros_gz_sim, 'worlds',
                                                  LaunchConfiguration('world_file')])],
                'on_exit_shutdown': 'True'
            }.items(),
        ),




       


        # SPAWNN ENTITY
        Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'wetexplorer',                                   
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-z', '0.5',],
                        output='screen'),




        # RUN THE BRIDGE        
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}',
            ]
        )

    ])