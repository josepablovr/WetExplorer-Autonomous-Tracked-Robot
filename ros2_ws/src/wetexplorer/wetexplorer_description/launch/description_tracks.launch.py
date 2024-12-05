import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import launch_ros
import launch
def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    package_name = 'wetexplorer_description'
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name)
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('wetexplorer_description'))

    rviz_config_path = os.path.join(pkg_path, 'config', 'viewer.rviz')

    #robot_description_path = os.path.join(pkg_share, 'urdf/wheels.xacro')
    xacro_file = os.path.join(pkg_path,'urdf','tracks.xacro')

    #robot_description = open(robot_description_path).read()
    robot_description_config = Command(['xacro ', xacro_file])
    
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    # Create the robot_state_publisher node
    #params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config}]
    )
        
    joint_state_publisher_node = launch_ros.actions.Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher'
    )
    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui'
    # )

    # Launch RViz2 with the robot model
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    # Launch all nodes
    return launch.LaunchDescription([
        joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        #rviz_node
        
    ])
