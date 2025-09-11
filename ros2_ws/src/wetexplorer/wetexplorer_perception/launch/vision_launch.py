from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Use env var SIM=true/false to switch remaps
    sim_env = os.getenv("sim", "true").lower() == "true"

    if sim_env:
        # SIM — map to camera1 topics
        remaps = [
            ('/camera/camera_info', '/camera1/camera_info'),
            ('/camera/depth/raw',   '/camera1/depth_raw'),
            ('/camera/image/raw',   '/camera1/image_raw'),
        ]
        params = [{'use_sim_time': True}]
    else:
        # REAL — map to hardware topics
        remaps = [
            ('/camera/camera_info', '/camera/depth/camera_info'),
            ('/camera/depth/raw',   '/camera/depth/image_raw'),
            ('/camera/image/raw',   '/camera/color/image_raw'),
        ]
        params = [{'use_sim_time': False}]

    # Load YOLO config
    config_filename = 'yolo_config.yaml'
    config_path = os.path.join(
        get_package_share_directory('wetexplorer_perception'),
        'config',
        config_filename
    )
    with open(config_path, 'r') as file:
        yolo_params = yaml.safe_load(file)

    # Ensure launch arguments are strings
    yolo_model = str(yolo_params['yolo_node']['ros__parameters']['model'])
    yolo_input_topic = str(yolo_params['yolo_node']['ros__parameters']['input_image_topic'])
    yolo_use_sim_time = 'true' if sim_env else 'false'

    yolo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('yolo_bringup'),
                'launch',
                'yolo.launch.py'
            ])
        ),
        launch_arguments={
            'model': yolo_model,
            'input_image_topic': yolo_input_topic,
            'use_sim_time': yolo_use_sim_time,
        }.items()
    )

    predator_node = Node(
        package='wetexplorer_vision_predator',
        executable='pose',
        name='registration_node',
        output='screen',
        remappings=remaps,
        parameters=params,
    )

    object_localization_light_node = Node(
        package='wetexplorer_vision_predator',
        executable='pose_light',
        name='registration_light_node',
        output='screen',
        remappings=remaps,
        parameters=params,
    )

    node_6d_pose = Node(
        package='wetexplorer_navigation',
        executable='6d_pose_caller',
        name='object_pose_caller',
        output='screen',
        remappings=remaps,
        parameters=params,
    )

    mask_point = Node(
        package='wetexplorer_perception',
        executable='object_depth_cloud_node',
        name='object_depth_cloud_node',
        output='screen',
        remappings=remaps,
        parameters=params,
    )

    rgbd_sync_remappings = [
        ('rgb/image',       '/camera/image_map'),
        ('depth/image',     '/camera/depth_map'),
        ('rgb/camera_info', '/camera1/camera_info'),
        ('scan',            '/jn0/base_scan'),
        ('gps/fix',         '/gps/fake'),
        ('/tf',             '/tf_rtabmap'),
        ('/odom',           '/odometry/global'),
    ]


    mapping_node = Node(
        package='wetexplorer_perception',
        executable='rgbd_map_node',
        output='screen',
        parameters=params,
        remappings=remaps,
    )

    light_localizer = Node(
        package='wetexplorer_perception',
        executable='object_localizer',
        output='screen',
        parameters=params,
        remappings=remaps,
    )

    

    return LaunchDescription([
        yolo_node,
        predator_node,
        node_6d_pose,
        object_localization_light_node,
        mask_point,        
        mapping_node,
        light_localizer,
    ])
