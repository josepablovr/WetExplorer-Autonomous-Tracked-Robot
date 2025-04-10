from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    # Include base IMU launch from phidgets_spatial package
    base_imu_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('phidgets_spatial'),
            'launch',
            'spatial-launch.py'
        ])
    ),
    launch_arguments={
        'use_orientation': 'true'
    }.items()
)

    # Node for transforming IMU frames
    imu_transformer = Node(
        package='imu_transformer',
        executable='imu_transformer_node',
        name='imu_data_transformer',
        output='screen',
        remappings=[
            ('imu_in', 'imu/data_raw'),
            ('imu_out', 'imu/data_transformed')
        ],
        parameters=[{
            'target_frame': 'imu_link_target'  # check this name
        }]
    )

    # Node for signal processing
    imu_signal_processing = Node(
        package='wetexplorer_navigation',
        executable='imu_filtering_node',
        name='imu_filtering_node',
        output='screen'
    )

    return LaunchDescription([
        base_imu_node,
        imu_transformer,
        imu_signal_processing
    ])
