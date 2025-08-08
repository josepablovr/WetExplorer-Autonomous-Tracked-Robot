from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


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

    node_lift = Node(
    package="wetexplorer_hardware",
    executable="lifting_joint_interface",
    name="lifting_joint_interface",
    parameters=[
        {"device_name": "/dev/tty_Lift"},
        {"baudrate": 115200}
    ]
    )
    

    gps_config_directory = os.path.join(
    get_package_share_directory('wetexplorer_hardware'),
    'config'
    )

    gps_params = os.path.join(gps_config_directory, 'gps.yaml')

    ublox_gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        namespace='gps',           # -> topics like /gps/fix, /gps/navsatfix, etc.
        output='both',
        parameters=[gps_params],        
        remappings=[('ublox_gps_node/fix', 'fix')]  # /gps/ublox_gps_node/fix → /gps/fix
    )

    return LaunchDescription([
        base_imu_node,
        imu_transformer,
        imu_signal_processing,
        node_lift,
        node_lift,
        robo_base_node,
        ublox_gps_node,
    ])
