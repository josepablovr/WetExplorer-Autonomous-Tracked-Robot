from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('wetexplorer_vision'), 'config', 'realsense_config.yaml'),
        description='Path to the realsense_config.yaml file'
    )
    
    json_file_path_arg = DeclareLaunchArgument(
        'preset_file',
        default_value=os.path.join(
            get_package_share_directory('wetexplorer_vision'), 'config', 'HighAccuracyPreset.json'),
        description='Path to the RealSense preset file'
    )
    
    # Launch the RealSense camera node
   
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'rgb_camera.color_profile': '640x480x15',
            'depth_module.depth_profile': '640x480x15',
            'enable_depth': 'true',
            'enable_color': 'true',
            'align_depth.enable': 'true',
            'json_file_path': LaunchConfiguration('preset_file')
        }.items()
    )


        
    return LaunchDescription([
        config_file_arg,
        json_file_path_arg,
        realsense_node
    ])
