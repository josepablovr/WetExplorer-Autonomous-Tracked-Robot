from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Launch IMU, Orbbec Gemini 330 camera, and robo_base controller."""

    # ──────────────────────────────────────────────────────────────────────────
    # IMU (Phidgets Spatial)                                                    
    # ──────────────────────────────────────────────────────────────────────────
    base_imu_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("phidgets_spatial"),
                "launch",
                "spatial-launch.py",
            ])
        ),
        launch_arguments={
            "use_orientation": "true",
        }.items(),
    )

    imu_transformer = Node(
        package="imu_transformer",
        executable="imu_transformer_node",
        name="imu_data_transformer",
        output="screen",
        remappings=[
            ("imu_in", "imu/data_raw"),
            ("imu_out", "imu/data_transformed"),
        ],
        parameters=[{"target_frame": "imu_link_target"}],  # adjust if needed
    )

    imu_signal_processing = Node(
        package="wetexplorer_navigation",
        executable="imu_filtering_node",
        name="imu_filtering_node",
        output="screen",
    )

    # ──────────────────────────────────────────────────────────────────────────
    # Orbbec Gemini 330 camera                                                 
    # ──────────────────────────────────────────────────────────────────────────
    gemini_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "gemini_330_series.launch.py",
            ])
        )
    )

    # ──────────────────────────────────────────────────────────────────────────
    # robo_base controller                                                     
    # ──────────────────────────────────────────────────────────────────────────
    robo_base_node = Node(
        package="robo_base",
        executable="robo_base_ros2",
        name="robo_base",
        output="screen",
    )

    return LaunchDescription([
        base_imu_node,
        imu_transformer,
        imu_signal_processing,
        gemini_camera_launch,
        robo_base_node,
    ])
