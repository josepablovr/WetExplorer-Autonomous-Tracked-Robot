from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  
    # Load parameters from the config file
    yolo_config_file = get_package_share_directory('wetexplorer_vision') + '/config/yolo_config_real.yaml'
    with open(yolo_config_file, 'r') as file:
        yolo_params = yaml.safe_load(file)

    # Include YOLO launch file with parameters from the YAML file
    yolo_bringup_launch = IncludeLaunchDescription(
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

    mask = Node(
            package='wetexplorer_vision',
            executable='mask_node',
            name='mask_node',
            output='screen',
            parameters=[{
                'config_file': 'install/wetexplorer_vision/share/wetexplorer_vision/config/obstacle_params.yaml'
            }]
        )

    

    

    # Return the combined launch description
    return LaunchDescription([
        yolo_bringup_launch,
        mask 
    ])
