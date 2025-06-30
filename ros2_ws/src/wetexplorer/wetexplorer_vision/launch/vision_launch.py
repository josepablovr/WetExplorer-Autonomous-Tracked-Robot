from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory

def load_yolo_config(context, *args, **kwargs):
    sim_time = context.launch_configurations.get('sim_time', 'false') == 'true'

    config_filename = 'yolo_config.yaml'
    config_path = os.path.join(
        get_package_share_directory('wetexplorer_vision'),
        'config',
        config_filename
    )
  
    with open(config_path, 'r') as file:
        yolo_params = yaml.safe_load(file)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('yolo_bringup'),
                    'launch',
                    'yolo.launch.py'
                ])
            ),
            launch_arguments={
                'model': yolo_params['yolo_node']['ros__parameters']['model'],
                'input_image_topic': yolo_params['yolo_node']['ros__parameters']['input_image_topic']
            }.items()
        )
    ]

def generate_launch_description():
    sim_env = os.getenv("sim", "true").lower() == "true"
    sim_time_param = {"use_sim_time": sim_env}
    sim_time_arg = DeclareLaunchArgument(
        'sim_time',
        default_value='true',
        description='Use simulation time if true'
    )  
    
    
    mask_node = Node(
        package='wetexplorer_vision',
        executable='mask_node',
        name='mask_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('sim_time')}]
    )

    predator_node = Node(
        package='wetexplorer_vision_predator',
        executable='pose',
        name='registration_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('sim_time')}]
    )

    object_localization_light_node = Node(
        package='wetexplorer_vision_predator',
        executable='pose_light',
        name='registration_light_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('sim_time')}]
    )

    image_processing = Node(
        package='wetexplorer_vision',
        executable='image_processing',
        name='image_processing',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('sim_time')}]
    )
   
    if sim_env:
        print("USING SIM TIME PARAMETERS")
        camera1_topic_info = "/camera1/camera_info"
        camera2_topic_info = "/camera2/camera_info"
        camera1_topic_depth = "/camera1/depth_raw"
        camera2_topic_depth = "/camera2/depth_raw"
        camera1_topic_image = "/camera1/image_raw"
        camera2_topic_image = "/camera2/image_raw"
    else:
        print("USING REAL TIME PARAMETERS")
        camera1_topic_info = "/camera/depth/camera_info"
        camera2_topic_info = "/camera2/camera_info"
        camera1_topic_depth = "/camera/depth/image_raw"
        camera2_topic_depth = "/camera2/depth_raw"
        camera1_topic_image = "/camera/color/image_raw"
        camera2_topic_image = "/camera2/image_raw"
        


    node_depth_mux = Node(
        package="topic_tools",
        executable="mux",
        name="mux_depth",
        output="screen",
        arguments=[
            "/camera/depth/raw",
            camera1_topic_depth,
            camera2_topic_depth,
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('sim_time')}]
    )

    node_image_mux = Node(
        package="topic_tools",
        executable="mux",
        name="mux_color",
        output="screen",
        arguments=[
            "/camera/image/raw",
            camera1_topic_image,
            camera2_topic_image,
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('sim_time')}]
    )

    node_camera_info_mux = Node(
        package="topic_tools",
        executable="mux",
        name="mux_info",
        output="screen",
        arguments=[
            "/camera/camera_info",
            camera1_topic_info,
            camera2_topic_info,
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('sim_time')}]
    )

    node_6d_pose = Node(
        package="wetexplorer_navigation",
        executable="6d_pose_caller",
        name="object_pose_caller",
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('sim_time')}]
    )
    
    
    
    return LaunchDescription([
        sim_time_arg,
        OpaqueFunction(function=load_yolo_config),
        mask_node,
        predator_node,
        node_depth_mux,
        node_image_mux,
        node_camera_info_mux,
        node_6d_pose,
        image_processing,
        object_localization_light_node

    ])
